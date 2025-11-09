package chipyard.example
import chisel3._
import chisel3.util._
import chisel3.util.random.LFSR
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS, FBUS, MBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.resources.SimpleDevice
import freechips.rocketchip.util.UIntIsOneOf
import chipyard.clocking._
import freechips.rocketchip.util.ResetCatchAndSync

/** Top-level MMIO config */
case class OverlapperConfig(base: BigInt, size: BigInt)
case object OverlapperKey extends Field[Option[OverlapperConfig]](None)

/** Tiny register-based FIFO (no SyncReadMem) */
class RegVecFIFO[T <: Data](gen: T, entries: Int) extends Module {
  require(entries > 0)
  private val w = log2Ceil(entries)

  val io = IO(new Bundle {
    val enq   = Flipped(Decoupled(gen))
    val deq   = Decoupled(gen)
    val count = Output(UInt((w + 1).W))
  })

  val storage   = Reg(Vec(entries, chiselTypeOf(io.enq.bits)))
  val enqPtr    = RegInit(0.U(w.W))
  val deqPtr    = RegInit(0.U(w.W))
  val maybeFull = RegInit(false.B)

  val ptrEq = enqPtr === deqPtr
  val empty = ptrEq && !maybeFull
  val full  = ptrEq &&  maybeFull

  io.enq.ready := !full
  when (io.enq.fire) {
    storage(enqPtr) := io.enq.bits
    enqPtr := enqPtr + 1.U
  }

  io.deq.valid := !empty
  io.deq.bits  := storage(deqPtr)
  when (io.deq.fire) {
    deqPtr := deqPtr + 1.U
  }

  when (io.enq.fire =/= io.deq.fire) { maybeFull := io.enq.fire }
  io.count := Mux(full, entries.U, Mux(ptrEq, 0.U, (enqPtr - deqPtr)(w-1, 0)))
}

/** Tester/driver around your KmerOverlapper.
  *
  * MMIO layout (8B beats):
  *  0x00  W  start (write 1 to start; HW auto-clears)
  *  0x04  W  srcAddr (DRAM address of 16-bit kmers)
  *  0x08  W  dstAddr (DRAM address for packed base bytes)
  *  0x0C  W  seqLen16 (number of 16-bit kmers to read)
  *  0x10  R  done
  *  0x14  R  bytesWritten
  */
class KmerOverlapperTester(
  base: BigInt,
  size: BigInt,
  nkmer: Int = 1024,
  maxSrc: Int = 4
)(implicit p: Parameters) extends LazyModule {

  // ----- TL master (memory DMA) -----
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLClientParameters(name = "overlapper-dma", sourceId = IdRange(0, maxSrc)))
  )))

  // ----- Interrupt source (1 line) -----
  val intnode = IntSourceNode(IntSourcePortSimple(num = 1))

  // ----- TL slave (MMIO) -----
  private val dev = new SimpleDevice("overlapper-accel", Seq("ucb,overlapper-accel"))
  val mmioNode = TLRegisterNode(
    address   = Seq(AddressSet(base, size - 1)),
    device    = dev,
    beatBytes = 8
  )

  lazy val module = new LazyModuleImp(this) {
    // ---------------- DUT ----------------
    val dut = Module(new KmerOverlapper(nkmer)) // your streaming base-emitter

    // ---------------- TL plumbing ----------------
    val (mem, edge) = node.out(0)
    val beatBits    = edge.bundle.dataBits      // e.g., 64, 128
    val beatBytes   = beatBits / 8
    require(beatBytes == 8 || beatBytes == 16 || beatBytes == 4, "Expect >= 64b TileLink beat")

    // ---------------- MMIO registers ----------------
    val srcAddr      = RegInit(0.U(edge.bundle.addressBits.W))
    val dstAddr      = RegInit(0.U(edge.bundle.addressBits.W))
    val seqLen16     = RegInit(0.U(32.W))            // number of 16-bit k-mers to read
    val startReg     = RegInit(false.B)              // write 1 to start; auto-clears
    val doneReg      = RegInit(false.B)
    val bytesWritten = RegInit(0.U(32.W))
    val baseTap      = RegInit(0.U(2.W))             // last seen base

    // --- Interrupt control ---
    val irqEnable   = RegInit(true.B)                // enable bit
    val irqPending  = RegInit(false.B)               // pending (sticky, W1C)
    val irqSet      = WireInit(0.U(1.W))    	     // HW set (pulse or level)


    val done_d = RegNext(dut.io.done, init=false.B)
    irqSet := (dut.io.done && !done_d).asUInt

    val irqLine = irqEnable && irqPending.asBool

    // drive the interrupt node (1 line)
    intnode.out.foreach { case (irq, _) => irq(0) := irqLine }

    // one-shot start pulse
    val startReg_d = RegNext(startReg, init = false.B)
    val startPulse = startReg && !startReg_d
    when (startPulse) {
      startReg   := false.B
      doneReg    := false.B
      irqPending := false.B        // clear pending on new start (common pattern)
    }

    // DUT static connections
    dut.io.start  := startPulse
    dut.io.nWords := seqLen16

    // base tap (update only when valid)
    when (dut.io.baseOutValid) { baseTap := dut.io.baseOut }

    // MMIO map (includes irq regs)
    mmioNode.regmap(
      0x00 -> Seq(RegField(1,  startReg)),
      0x04 -> Seq(RegField(edge.bundle.addressBits, srcAddr)),
      0x08 -> Seq(RegField(edge.bundle.addressBits, dstAddr)),
      0x0C -> Seq(RegField(32, seqLen16)),
      0x10 -> Seq(RegField.r(1,  doneReg)),
      0x14 -> Seq(RegField.r(32, bytesWritten)),
      0x18 -> Seq(RegField.r(2,  baseTap)),
      0x1C -> Seq(RegField(1,  irqEnable)),                 // IRQ_EN (RW)
      0x20 -> Seq(RegField.w1ToClear(1, irqPending, irqSet, // IRQ_IP (W1C)
          Some(RegFieldDesc(name = "irq_ip", desc = "IRQ pending (W1C)", reset = Some(0))))), // IRQ_IP (W1C + RO)
      0x24 -> Seq(RegField.r(32, dut.io.basesSeen))         // optional: expose base counter
    )

    // ---------------- Control FSM ----------------
    val sIdle :: sRun :: sDone :: Nil = Enum(3)
    val state = RegInit(sIdle)

    // ---------------- READ engine: single 64b Get at a time ----------------
    val rdAddr     = RegInit(0.U(edge.bundle.addressBits.W))
    val rdRemain   = RegInit(0.U(32.W))                 // 16-bit words remaining
    val rdPending  = RegInit(false.B)                   // one outstanding Get
    val unpackReg  = Reg(UInt(beatBits.W))              // last returned beat
    val laneIdx    = RegInit(0.U(log2Ceil((beatBytes/2) + 1).W)) // 0..(beatBytes/2 - 1)
    val unpacking  = RegInit(false.B)                   // consuming lanes from unpackReg

    // Stage to decouple D from FIFO backpressure
    val rxValid = RegInit(false.B)
    val rxWord  = Reg(SInt(16.W))

    // Input FIFO into DUT
    val inQ = Module(new RegVecFIFO(SInt(16.W), entries = 256))
    dut.io.in <> inQ.io.deq
    inQ.io.enq.valid := false.B
    inQ.io.enq.bits  := 0.S

    // TL defaults
    mem.a.valid := false.B
    mem.a.bits  := DontCare
    mem.d.ready := true.B

    val beatBytesU = (beatBytes).U

    // Gate new reads once DUT declares done
    val canIssueGet = (state === sRun) && (rdRemain =/= 0.U) &&
                      !dut.io.done && !rdPending && !unpacking && !rxValid

    when (canIssueGet) {
      val get = edge.Get(fromSource = 0.U, toAddress = rdAddr, lgSize = log2Ceil(beatBytes).U)
      mem.a.valid := true.B
      mem.a.bits  := get._2
      when (mem.a.fire && (mem.a.bits.opcode === TLMessages.Get)) {
        rdPending := true.B
        rdAddr    := rdAddr + beatBytesU
      }
    }

    // Accept read data
    when (mem.d.fire && mem.d.bits.opcode === TLMessages.AccessAckData) {
      when (rdPending) {
        unpackReg := mem.d.bits.data
        laneIdx   := 0.U
        unpacking := true.B
        rdPending := false.B
      }
    }

    // Extract 16-bit lanes
    val wordsPerBeat = (beatBytes/2).U
    val laneWord     = (unpackReg >> (laneIdx << 4))(15, 0).asSInt

    // Stage next lane if needed
    when (unpacking && !rxValid) {
      rxWord  := laneWord
      rxValid := true.B
      when (laneIdx === (wordsPerBeat - 1.U)) {
        unpacking := false.B
      } .otherwise {
        laneIdx := laneIdx + 1.U
      }
    }

    // Feed staged lane into input FIFO
    inQ.io.enq.valid := (state === sRun) && rxValid && (rdRemain =/= 0.U)
    inQ.io.enq.bits  := rxWord
    when (inQ.io.enq.fire) {
      rxValid  := false.B
      rdRemain := rdRemain - 1.U
    }

    // ---------------- PACK+WRITE engine: pack 32 bases (2b) into 64-bit ----------------
    val packReg        = RegInit(0.U(64.W))              // LSb-first packing
    val packCountBases = RegInit(0.U(6.W))               // 0..32 bases collected
    val wrAddr         = RegInit(0.U(edge.bundle.addressBits.W))
    val wrPending      = RegInit(false.B)
    val lastWrBytes    = RegInit(0.U(8.W))               // how many bytes were written last time
    val finishing      = RegInit(false.B)                // set when done seen; clear when tail acked

    // Accumulate bases as they come
    when (dut.io.baseOutValid) {
      val shift = (packCountBases << 1).asUInt
      packReg        := packReg | (dut.io.baseOut << shift)
      packCountBases := packCountBases + 1.U
    }

    // Helpers for issuing writes
    val packFull   = packCountBases === 32.U
    val bitsUsed   = (packCountBases << 1).asUInt                 // 2 bits per base
    val bytesUsed  = ((bitsUsed + 7.U) >> 3).asUInt               // ceil(bits/8)
    val maskWide   = ((1.U((beatBytes + 1).W) << bytesUsed) - 1.U) // low 'bytesUsed' bytes set
    val byteMask   = maskWide(beatBytes - 1, 0)                    // TL mask width = beatBytes

    val dataBeat = Mux(beatBits.U === 64.U, packReg,
                   Cat(0.U((beatBits - 64).W), packReg)) // place in low half on wider buses

    // Write when: (a) full 8B packet OR (b) finishing and there is a tail
    val wantFull = packFull
    val wantTail = (dut.io.done || finishing) && (packCountBases =/= 0.U)
    val doWrite  = (state === sRun) && !wrPending && (wantFull || wantTail)
    val isTail   = wantTail && !wantFull

    when (doWrite) {
      val busIs64  = (beatBytes.U === 8.U)
      val useMask  = isTail || !busIs64          // tail OR wide bus → masked Put
      val wrLgSize = Mux(useMask, log2Ceil(beatBytes).U, log2Ceil(8).U)
      val wrData   = dataBeat
      val wrMask   = Mux(isTail, byteMask,
                     Mux(busIs64, 0.U, ((1.U << 8) - 1.U)(beatBytes - 1, 0)))
      val writeSz  = Mux(isTail, bytesUsed, 8.U)

      val aBits = Wire(chiselTypeOf(mem.a.bits))
      val aVld  = WireDefault(false.B)

      when (useMask) {
        val p = edge.Put(1.U, wrAddr, wrLgSize, wrData, wrMask)
        aBits := p._2; aVld := true.B
      } .otherwise {
        val p = edge.Put(1.U, wrAddr, wrLgSize, wrData(63, 0))
        aBits := p._2; aVld := true.B
      }

      mem.a.valid := aVld
      mem.a.bits  := aBits

      when (mem.a.fire && (mem.a.bits.opcode === TLMessages.PutFullData ||
                           mem.a.bits.opcode === TLMessages.PutPartialData)) {
        wrPending   := true.B
        lastWrBytes := writeSz
        when (dut.io.done) { finishing := true.B } // remember we’re finishing due to done
      }
    }

    // Write acks
    when (mem.d.fire && mem.d.bits.opcode === TLMessages.AccessAck) {
      when (wrPending) {
        wrPending      := false.B
        wrAddr         := wrAddr + lastWrBytes
        bytesWritten   := bytesWritten + lastWrBytes
        packReg        := 0.U
        packCountBases := 0.U

        // If finishing due to done and tail just flushed, we can assert done + IRQ
        when (finishing) {
          finishing  := false.B
          doneReg    := true.B
          irqPending := true.B          // raise interrupt (sticky; SW clears via W1C)
          state      := sDone
        }
      }
    }

    // ---------------- FSM transitions ----------------
    switch (state) {
      is (sIdle) {
        when (startPulse) {
          // init read/write engines
          rdAddr     := srcAddr
          rdRemain   := seqLen16
          rdPending  := false.B
          unpacking  := false.B
          rxValid    := false.B

          wrAddr         := dstAddr
          wrPending      := false.B
          lastWrBytes    := 0.U
          packReg        := 0.U
          packCountBases := 0.U
          bytesWritten   := 0.U
          finishing      := false.B

          state := sRun
        }
      }

      is (sRun) {
        // If DUT done and no tail/pending write, finish immediately (and IRQ below)
        when (dut.io.done && !wrPending && (packCountBases === 0.U)) {
          doneReg    := true.B
          irqPending := true.B
          state      := sDone
        }
      }

      is (sDone) {
        // hold done high until next start
	  state      := sIdle
        // (irqPending holds until SW W1C; startPulse also clears it)
      }
    }
  }
}



/*class KmerOverlapperTester(
  base: BigInt,
  size: BigInt,
  nkmer: Int = 1024,
  maxSrc: Int = 4
)(implicit p: Parameters) extends LazyModule {

  // TL master (DRAM)
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    clients = Seq(TLClientParameters(name="overlapper-dma", sourceId=IdRange(0, maxSrc)))
  )))

  // TL slave (MMIO)
  private val dev = new SimpleDevice("overlapper-accel", Seq("ucb,overlapper-accel"))
  val mmioNode = TLRegisterNode(
    address   = Seq(AddressSet(base, size-1)),
    device    = dev,
    beatBytes = 8
  )

  lazy val module = new LazyModuleImp(this) {
    // ---------------- DUT ----------------
    val dut = Module(new KmerOverlapper(nkmer))
    dut.io.posOut.ready := true.B // drop positions by default

    // -------------- DMA base -------------
    val (mem, edge) = node.out(0)
    val beatBits    = edge.bundle.dataBits
    val beatBytes   = beatBits/8
    val beatBytesU  = (beatBytes).U
    require(beatBytes == 8 || beatBytes == 16 || beatBytes == 4, "Expect >=64b TL")

    // -------------- MMIO regs ------------
    val srcAddr      = RegInit(0.U(edge.bundle.addressBits.W))
    val dstAddr      = RegInit(0.U(edge.bundle.addressBits.W))
    val seqLen16     = RegInit(0.U(32.W))
    val startReg     = RegInit(false.B)
    val doneReg      = RegInit(false.B)
    val bytesWritten = RegInit(0.U(32.W))

    val startReg_d = RegNext(startReg, init=false.B)
    val startPulse = startReg && !startReg_d
    when (startPulse) { startReg := false.B }
    val baseTap = RegInit(0.U(2.W))
    //baseTap := dut.io.baseOut
    mmioNode.regmap(
      0x00 -> Seq(RegField(1,  startReg)),
      0x04 -> Seq(RegField(edge.bundle.addressBits, srcAddr)),
      0x08 -> Seq(RegField(edge.bundle.addressBits, dstAddr)),
      0x0C -> Seq(RegField(32, seqLen16)),
      0x10 -> Seq(RegField.r(1,  doneReg)),
      0x14 -> Seq(RegField.r(32, bytesWritten)),
      0x18 -> Seq(RegField.r(2, baseTap))   // exposes baseOut
    )

    // -------------- FSM ------------------
    val sIdle :: sRun :: sDrain :: sDone :: Nil = Enum(4)
    val state = RegInit(sIdle)

    // -------------- READ: one 64b Get at a time --------------
    val rdAddr       = RegInit(0.U(edge.bundle.addressBits.W)) // byte address (beat aligned)
    val rdRemain     = RegInit(0.U(32.W))                      // 16-bit words remaining
    val rdPending    = RegInit(false.B)                        // one outstanding Get
    val unpackReg    = Reg(UInt(beatBits.W))                   // last beat data
    val laneIdx      = RegInit(0.U(log2Ceil((beatBytes/2)+1).W)) // 0..(beatBytes/2-1)
    val unpacking    = RegInit(false.B)                        // consuming lanes from unpackReg

    // Stage to decouple D from FIFO backpressure
    val rxValid = RegInit(false.B)
    val rxWord  = Reg(SInt(16.W))

    // Input queue + sentinel arb
    val inQ  = Module(new RegVecFIFO(SInt(16.W), entries=256))
    dut.io.in <> inQ.io.deq
    val arb  = Module(new Arbiter(SInt(16.W), 2)) // 0: sentinel, 1: DMA
    inQ.io.enq <> arb.io.out

    val sentDone = RegInit(false.B)

    // TL defaults
    mem.a.valid := false.B
    mem.a.bits  := DontCare
    mem.d.ready := true.B

    // Issue exactly one full-beat Get when:
    //   need words, none pending, and not currently unpacking/holding a staged word.
    val canIssueGet = (state === sRun) && (rdRemain =/= 0.U) && !rdPending && !unpacking && !rxValid
    when (canIssueGet) {
      val get = edge.Get(fromSource=0.U, toAddress=rdAddr, lgSize=log2Ceil(beatBytes).U)
      mem.a.valid := true.B
      mem.a.bits  := get._2
      when (mem.a.fire) {
        rdPending := true.B
        rdAddr    := rdAddr + beatBytesU // next beat address
      }
    }

    // On AccessAckData for our single outstanding Get:
    when (mem.d.fire && mem.d.bits.opcode === TLMessages.AccessAckData) {
      when (rdPending) {
        unpackReg := mem.d.bits.data
        laneIdx   := 0.U
        unpacking := true.B
        rdPending := false.B
      }
    }
    when (dut.io.baseOut === 0.U && dut.io.baseOutValid){baseTap := 0.U}.elsewhen(dut.io.baseOut === 1.U){baseTap := 1.U}.elsewhen(dut.io.baseOut === 2.U){baseTap := 2.U}.elsewhen(dut.io.baseOut === 3.U){baseTap := 3.U}
    // Present one 16b lane at a time to arb.in(1)
    val wordsPerBeat = (beatBytes/2).U
    val laneWord = (unpackReg >> (laneIdx << 4))(15, 0).asSInt

    // If we don't already have a staged D->FIFO word, stage next lane
    when (unpacking && !rxValid) {
      rxWord  := laneWord
      rxValid := true.B
      // advance lane for next time; stop unpacking on last lane or when no need
      when (laneIdx === (wordsPerBeat - 1.U)) {
        unpacking := false.B
      }.otherwise {
        laneIdx := laneIdx + 1.U
      }
    }

    // Feed staged word into FIFO (arb port 1)
    arb.io.in(1).valid := (state === sRun) && rxValid && (rdRemain =/= 0.U)
    arb.io.in(1).bits  := rxWord
    when (arb.io.in(1).fire) {
      rxValid  := false.B
      rdRemain := rdRemain - 1.U
    }

    // Sentinel injection after all words delivered & FIFO empty
    val fifoEmpty     = inQ.io.count === 0.U
    val canInjectSent = (state === sRun) && !sentDone &&
                        (rdRemain === 0.U) && !rdPending && !unpacking && !rxValid && fifoEmpty
    arb.io.in(0).valid := canInjectSent
    arb.io.in(0).bits  := (-1).S(16.W)
    when (arb.io.in(0).fire) { sentDone := true.B }

    // Kick DUT once FIFO has data (one-shot)
    val started  = RegInit(false.B)
    val startNow = (state === sRun) && !started && (inQ.io.count =/= 0.U)
    dut.io.start := startNow
    when (startNow)    { started := true.B }
    when (state===sIdle) { started := false.B; sentDone := false.B }

    // -------------- WRITE: two 64b PutFull per 128b out --------------
    val outQ     = Module(new RegVecFIFO(UInt(128.W), entries=16))
    outQ.io.enq.valid := dut.io.outBases.valid
    outQ.io.enq.bits  := dut.io.outBases.bits
    dut.io.outBases.ready := outQ.io.enq.ready
    outQ.io.deq.ready := false.B

    val wrAddr    = RegInit(0.U(edge.bundle.addressBits.W))   // must be 8B aligned
    val wrPending = RegInit(false.B)                          // one outstanding Put
    val wrHold    = Reg(UInt(128.W))                          // current 128b payload
    val wrPhase   = RegInit(0.U(1.W))                         // 0 -> low 64b, 1 -> high 64b
    val wrHave    = RegInit(false.B)                          // we latched a 128b chunk

    // Load a new 128b word when writer is idle
    val writerIdle = !wrPending && !wrHave
    when (state === sRun || state === sDrain) {
      when (writerIdle && outQ.io.deq.valid) {
        wrHold := outQ.io.deq.bits
        wrPhase := 0.U
        wrHave := true.B
        outQ.io.deq.ready := true.B
      }
    }

    // Issue one full-beat Put (64b or beatBytes) at a time
    when ((state === sRun || state === sDrain) && wrHave && !wrPending) {
      val data64 = Mux(wrPhase === 0.U, wrHold(63,0), wrHold(127,64))
      val dataBeat =
        if (beatBytes == 8) data64.asUInt
        else if (beatBytes == 16) Cat(0.U(64.W), data64) // place in low half (or adjust as needed)
        else data64.asUInt // 32b beat systems rare here

      val put = edge.Put(fromSource=1.U, toAddress=wrAddr, lgSize=log2Ceil(beatBytes).U, data=dataBeat)
      mem.a.valid := true.B
      mem.a.bits  := put._2
      when (mem.a.fire) { wrPending := true.B }
    }

    // Ack for PutFull (AccessAck)
    when (mem.d.fire && mem.d.bits.opcode === TLMessages.AccessAck) {
      when (wrPending) {
        wrPending    := false.B
        wrAddr       := wrAddr + beatBytesU
        bytesWritten := bytesWritten + beatBytesU
        when (wrPhase === 0.U) { wrPhase := 1.U }          // next half
        .otherwise             { wrHave := false.B }       // done with this 128b
      }
    }

    // -------------- FSM transitions --------------
    switch (state) {
      is (sIdle) {
        doneReg := false.B
        when (startPulse) {
          // init read/write engines
          rdAddr    := srcAddr
          rdRemain  := seqLen16
          rdPending := false.B
          unpacking := false.B
          rxValid   := false.B

          wrAddr     := dstAddr
          wrPending  := false.B
          wrHave     := false.B
          wrPhase    := 0.U
          bytesWritten := 0.U

          // queues/DUT handshakes are already reset-friendly
          state := sRun
        }
      }

      is (sRun) {
        // When DUT asserts done (after sentinel), just move to drain any queued outputs
        when (dut.io.done) { state := sDrain }
      }

      is (sDrain) {
        // Done when writer is idle, outQ empty, and no pending D traffic
        val drainFinished = !wrPending && !wrHave && (outQ.io.count === 0.U)
        when (drainFinished) { state := sDone }
      }

      is (sDone) {
        doneReg := true.B
        when (startPulse) { state := sRun } // allow re-run
      }
    }
  }
}
*/
trait CanHaveOverlapper { this: BaseSubsystem =>
  implicit val p: Parameters

  p(OverlapperKey).map { k =>
    val base = k.base
    val size = k.size
    val fbus = locateTLBusWrapper(FBUS)
    val pbus = locateTLBusWrapper(PBUS)
    val domain = fbus.generateSynchronousDomain.suggestName("overlapper_domain")
    val overlapper = domain { LazyModule(new KmerOverlapperTester(base = base, size = size, nkmer = 1024)(p)) } 
    fbus.coupleFrom("overlapper-dma") { _ := TLWidthWidget(fbus) := overlapper.node }
    pbus.coupleTo("overlapper-mmio") { overlapper.mmioNode := TLFragmenter(pbus) := _ }

    // *** Interrupt routing to the PLIC ***
    // over.intnode is a source; ibus is the SoC-wide interrupt bus
    ibus.fromSync := overlapper.intnode
  }
}

class WithOverlapper(base: BigInt, size: BigInt) extends Config((site, here, up) => {
  case OverlapperKey => Some(OverlapperConfig(base, size))
})


/** LazyModule wrapper that MMIO-controls a KmerOverlapper and does DMA
  * - Reads SInt(16) kmers from DRAM
  * - Streams to KmerOverlapper
  * - Expands 128b packed bases (64×2b) to bytes 0..3 and writes back to DRAM
  *
  * @param address  base address for MMIO
  * @param size     MMIO window size
  * @param nkmer    overlapper nkmer (e.g. 1024)
  * @param maxSrc   number of TL source IDs for the DMA master
  */
/*class KmerOverlapperTester(
  address: BigInt,
  size: BigInt,
  nkmer: Int = 1024,
  maxSrc: Int = 16
)(implicit p: Parameters)
    extends LazyModule {

  // TL master port for DMA
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(
    Seq(TLClientParameters(
      name     = "overlapper-dma",
      sourceId = IdRange(0, maxSrc)
    ))
  )))

  // TL slave port for MMIO registers
  private val dev = new SimpleDevice("overlapper-accel", Seq("ucb,overlapper-accel"))
  val mmioNode = TLRegisterNode(
    address   = Seq(AddressSet(address, size - 1)),
    device    = dev,
    beatBytes = 8 // keep constant; if you want pbus.beatBytes, thread it in via ctor
  )

  lazy val module = new LazyModuleImp(this) {
    // -------------------------
    // DUT
    // -------------------------
    val dut = Module(new KmerOverlapper(nkmer))
    dut.io.posOut.ready := true.B // drain aux stream

    // -------------------------
    // DMA port
    // -------------------------
    val (mem, edge) = node.out(0)
    val beatBits    = edge.bundle.dataBits
    val beatBytes   = beatBits / 8
    val beatBytesU  = (beatBytes).U

    // -------------------------
    // MMIO registers
    // -------------------------
    val srcAddrLo     = RegInit(0.U(32.W))
    val srcAddrHi     = RegInit(0.U(32.W))
    val dstAddrLo     = RegInit(0.U(32.W))
    val dstAddrHi     = RegInit(0.U(32.W))
    val seqLen16      = RegInit(0.U(32.W)) // number of 16-bit words to read
    val triggerW1     = RegInit(false.B)
    val doneReg       = RegInit(false.B)
    val bytesWritten  = RegInit(0.U(32.W))

    mmioNode.regmap(
      0x00 -> Seq(RegField.w(1, triggerW1)),            // write-1 to start
      0x04 -> Seq(RegField(32, srcAddrLo)),
      0x08 -> Seq(RegField(32, srcAddrHi)),
      0x0C -> Seq(RegField(32, dstAddrLo)),
      0x10 -> Seq(RegField(32, dstAddrHi)),
      0x14 -> Seq(RegField(32, seqLen16)),            // 16-bit words in source
      0x18 -> Seq(RegField.r(1,  doneReg)),
      0x1C -> Seq(RegField.r(32, bytesWritten))       // bytes written to dst
    )

    val srcAddr = Cat(srcAddrHi, srcAddrLo) // byte addresses
    val dstAddr = Cat(dstAddrHi, dstAddrLo)

    // -------------------------
    // DMA READ → unpack → Arbiter port 1
    // -------------------------
    val rdAddr   = Reg(UInt(edge.bundle.addressBits.W))
    val rdRemain = Reg(UInt(32.W)) // 16-bit words remaining

    val unpackCnt = RegInit(0.U(log2Ceil(wordsPerBeat + 1).W))
    val unpackReg = Reg(UInt(beatBits.W))
    val unpacking = RegInit(false.B)

    // -------------------------
    // Control FSM
    // -------------------------
    val sIdle :: sRead :: sFeed :: sDrain :: sDone :: Nil = Enum(5)
    val state = RegInit(sIdle)

    val startPulse = triggerW1
    when (startPulse) {
      doneReg      := false.B
      bytesWritten := 0.U
    }

    // -------------------------
    // Input queue & source mux (DMA + terminal sentinel)
    // -------------------------
    val inQ = Module(new Queue(SInt(16.W), 256))
    //val inQ = Module(new SyncSRAMQueue(SInt(16.W), 256))
    dut.io.in <> inQ.io.deq

    // Two input producers → Arbiter: 0 = sentinel (higher prio), 1 = DMA
    val arb = Module(new Arbiter(SInt(16.W), 2))
    inQ.io.enq <> arb.io.out

    // DMA producer wiring (we’ll drive arb.io.in(1))
    arb.io.in(1).valid := false.B
    arb.io.in(1).bits  := 0.S

    // Sentinel injector wiring (arb.io.in(0))
    val sentDone  = RegInit(false.B)
    val injCond   = (state === sFeed) && !sentDone
    val injectNow = injCond && (rdRemain === 0.U) && !unpacking && (inQ.io.count === 0.U)
    arb.io.in(0).valid := injectNow
    arb.io.in(0).bits  := (-1).S(16.W)
    when (arb.io.in(0).fire) { sentDone := true.B }
    when (state === sIdle)   { sentDone := false.B }

    // Defaults
    mem.a.valid := false.B
    mem.a.bits  := DontCare
    mem.d.ready := false.B

    val wordsPerBeat = (beatBytes / 2)
    require(wordsPerBeat >= 1, "Beat must be at least 16 bits wide")


    val canIssueRead = (state === sRead) && !unpacking && (rdRemain =/= 0.U)

    when (canIssueRead) {
      val get = edge.Get(fromSource = 0.U, toAddress = rdAddr, lgSize = log2Ceil(beatBytes).U)
      mem.a.valid := true.B
      mem.a.bits  := get._2
      when (mem.a.fire) {
        rdAddr := rdAddr + beatBytesU // advance to next beat
      }
    }

    when (state === sRead) {
      mem.d.ready := true.B
      when (mem.d.fire) {
        unpackReg := mem.d.bits.data
        unpackCnt := 0.U
        unpacking := true.B
      }
    }

    // 16-bit slice (little-endian within beat)
    val sliceIdx = unpackCnt
    val word16   = (unpackReg >> (sliceIdx << 4)).asUInt(15, 0)
    val wordS    = word16.asSInt

    // Drive DMA producer into arb port 1
    val stillNeed = rdRemain =/= 0.U
    val dmaCanPush = unpacking && stillNeed
    arb.io.in(1).valid := (state === sRead || state === sFeed) && dmaCanPush
    arb.io.in(1).bits  := wordS

    // On successful handoff, consume a word
    when (arb.io.in(1).fire) {
      rdRemain := rdRemain - 1.U
      when (unpackCnt === (wordsPerBeat - 1).U) {
        unpacking := false.B
      }.otherwise {
        unpackCnt := unpackCnt + 1.U
      }
    }

    // Drop tail words in the last beat if we over-fetched
    when (rdRemain === 0.U) { unpacking := false.B }

    // -------------------------
    // DUT start pulse (one cycle)
    // -------------------------
    val started  = RegInit(false.B)
    val startNow = (state === sFeed) && !started && (inQ.io.count =/= 0.U)
    dut.io.start := startNow
    when (startNow)     { started := true.B }
    when (state === sIdle) { started := false.B }

    // -------------------------
    // DUT outBases (128b) → expand to bytes → DMA write
    // -------------------------
    val outBasesQ = Module(new Queue(UInt(128.W), 16))
    outBasesQ.io.enq.valid := dut.io.outBases.valid
    outBasesQ.io.enq.bits  := dut.io.outBases.bits
    dut.io.outBases.ready  := outBasesQ.io.enq.ready

    val byteQ = Module(new Queue(UInt(8.W), 512))

    val expIdle :: expRun :: Nil = Enum(2)
    val expSt   = RegInit(expIdle)
    val expBits = Reg(UInt(128.W))
    val expIdx  = Reg(UInt(6.W)) // 0..63

    byteQ.io.enq.valid := false.B
    byteQ.io.enq.bits  := 0.U
    outBasesQ.io.deq.ready := false.B

    switch (expSt) {
      is (expIdle) {
        when (outBasesQ.io.deq.valid && byteQ.io.enq.ready) {
          expBits := outBasesQ.io.deq.bits
          expIdx  := 0.U
          expSt   := expRun
          outBasesQ.io.deq.ready := true.B
        }
      }
      is (expRun) {
        val twoBits = (expBits >> (expIdx << 1)).asUInt(1, 0)
        when (byteQ.io.enq.ready) {
          byteQ.io.enq.valid := true.B
          byteQ.io.enq.bits  := twoBits
          when (expIdx === 63.U) { expSt := expIdle }
            .otherwise          { expIdx := expIdx + 1.U }
        }
      }
    }

    // -------------------------
    // DMA WRITE (pack bytes → PutFull beats)
    // -------------------------
    val wrAddr  = Reg(UInt(edge.bundle.addressBits.W))
    val packCnt = RegInit(0.U(log2Ceil(64).W)) // enough bits for max beat size
    val packReg = Reg(UInt(beatBits.W))

    // defaults each cycle
    // (read defaults are above; here we handle write in sDrain)
    when (state === sDrain) {
      // consume bytes into packer
      byteQ.io.deq.ready := false.B

      when (byteQ.io.deq.valid && packCnt < beatBytes.U) {
        val shift = packCnt << 3 // *8
        packReg := packReg | (byteQ.io.deq.bits << shift)
        packCnt := packCnt + 1.U
        byteQ.io.deq.ready := true.B
      }

      // If full beat OR there are some bytes and no more available → flush
      val needFlush = (packCnt === beatBytes.U) || (packCnt =/= 0.U && !byteQ.io.deq.valid)

      when (needFlush) {
        val put = edge.Put(
          fromSource = 0.U,
          toAddress  = wrAddr,
          lgSize     = log2Ceil(beatBytes).U,
          data       = packReg
        )
        mem.a.valid := true.B
        mem.a.bits  := put._2
        when (mem.a.fire) {
          wrAddr       := wrAddr + beatBytesU
          bytesWritten := bytesWritten + packCnt
          packReg      := 0.U
          packCnt      := 0.U
        }
      }

      mem.d.ready := true.B // accept responses
    }.otherwise {
      // non-write states: ensure no stray writes
      byteQ.io.deq.ready := false.B
    }

    // -------------------------
    // Top-level FSM transitions
    // -------------------------
    switch (state) {
      is (sIdle) {
        when (startPulse) {
          // Initialize DMA pointers & engines
          rdAddr   := srcAddr
          rdRemain := seqLen16
          wrAddr   := dstAddr
          packCnt  := 0.U
          packReg  := 0.U
          expSt    := expIdle
          doneReg  := false.B
          state    := sRead
        }
      }

      is (sRead) {
        // as soon as some data is in inQ, we can go feed
        when (inQ.io.count =/= 0.U) {
          state := sFeed
        }
        // stay here issuing reads until rdRemain drains and unpacking finishes
      }

      is (sFeed) {
        // let DUT run; expansion and write are independent and can start early
        when (dut.io.done) {
          state := sDrain
        }
      }

      is (sDrain) {
        // finished when all buffers are empty and expander is idle
        val drainFinished =
          (byteQ.io.count === 0.U) &&
          (packCnt === 0.U) &&
          (expSt === expIdle) &&
          !outBasesQ.io.deq.valid

        when (drainFinished) {
          state  := sDone
        }
      }

      is (sDone) {
        doneReg := true.B
        when (startPulse) { state := sRead } // allow immediate re-run
      }
    }
  }
}*/

