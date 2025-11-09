package chipyard.example

import chisel3._
import chisel3.util._


/** Streaming overlapper that outputs one base at a time (2 bits + valid).
  * Mirrors the C reference:
  *  - skip initial negatives
  *  - emit first non-negative k-mer fully (kmer_len bases, MSB-first)
  *  - for each subsequent non-negative k-mer, compute overlap (do-while style)
  *    and emit exactly 'overlap' bases (MSB-first of the suffix)
  *  - negative k-mers in the middle are "stays" (emit nothing)
  *
  * @param nkmer size of k-mer alphabet space (must be 4^k, e.g., 1024 for k=5)
  *              kmer_len = position_highest_bit(nkmer)/2 (e.g., 1024 -> 11/2 -> 5)
  */
class KmerOverlapper(val nkmer: Int) extends Module {
  require(nkmer > 0, "nkmer must be > 0 (e.g., 4^k)")
  private val kmerLenI = (log2Ceil(nkmer) + 1) / 2 // matches position_highest_bit(nkmer)/2
  private val kmerLenU = kmerLenI.U

  val io = IO(new Bundle {
    val start        = Input(Bool())
    val nWords       = Input(UInt(32.W))            // total # of k-mers in the input stream
    val in           = Flipped(Decoupled(SInt(16.W)))
    val baseOut      = Output(UInt(2.W))            // 00 A, 01 C, 10 G, 11 T
    val baseOutValid = Output(Bool())               // registered valid
    val basesSeen    = Output(UInt(32.W))
    val done         = Output(Bool())
  })

  // -----------------------
  // Registers / State
  // -----------------------
  val sIdle :: sFindFirst :: sEmitInit :: sReadNext :: sOverlap :: sEmitSuffix :: sDone :: Nil = Enum(7)
  val state        = RegInit(sIdle)

  val seqRemain    = Reg(UInt(32.W))        // how many input elements left to read
  val prevKmer     = Reg(SInt(16.W))        // previous non-negative k-mer
  val currKmer     = Reg(SInt(16.W))        // current non-negative k-mer (to emit from)

  // Emission control (for first k-mer and suffixes)
  val emitTotal    = Reg(UInt(6.W))         // how many bases to emit this round (kmerLen or overlap)
  val emitIdx      = Reg(UInt(6.W))         // 0 .. emitTotal-1

  // 2-bit base + registered valid
  val baseReg      = RegInit(0.U(2.W))
  val baseValidReg = RegInit(false.B)
  io.baseOut      := baseReg
  io.baseOutValid := baseValidReg

  val basesSeenReg = RegInit(0.U(32.W)).suggestName("basesSeenReg")
  dontTouch(basesSeenReg)
  dontTouch(io.basesSeen)
  io.basesSeen     := basesSeenReg

  // Done latch
  val doneReg      = RegInit(false.B)
  io.done := doneReg

  // Default de-assert the valid each cycle; assert only on emit
  baseValidReg := false.B

  // Input ready by state (we never back the input with a buffer here)
  io.in.ready := false.B
  switch (state) {
    is (sFindFirst) { io.in.ready := true.B }
    is (sReadNext)  { io.in.ready := true.B }
  }

  // -----------------------
  // Helpers
  // -----------------------
  // Extract base 2-bit code (idx: 0 = LSB base; we will use MSB-first via math below)
  private def getBase2b(kmer: SInt, idx: UInt): UInt = {
    ((kmer >> (idx * 2.U)) & 3.S).asUInt
  }

  // Emit exactly one base this cycle (registered valid)
  private def pushBase(base2b: UInt): Unit = {
    baseReg       := base2b
    baseValidReg  := true.B
    basesSeenReg  := basesSeenReg + 1.U 
  }

  // -----------------------
  // Overlap engine (do-while)
  // Implements:
  //   kmer_mask = nkmer - 1;
  //   overlap = 0;
  //   do { kmer_mask >>= 2; k1 &= kmer_mask; k2 >>= 2; overlap += 1; } while (k1 != k2);
  // Runs for up to kmerLenI iterations; completes when equality observed.
  // -----------------------
  val ovBusy     = RegInit(false.B)
  val ovMask     = Reg(UInt(16.W))     // mask register (fits 10b for k=5; 16b safe)
  val ovK1       = Reg(UInt(16.W))     // working k1 (masked down over time)
  val ovK2       = Reg(UInt(16.W))     // working k2 (shifted down over time)
  val ovCount    = Reg(UInt(6.W))      // overlap count (1..kmerLenI)

  def overlapStart(k1: SInt, k2: SInt): Unit = {
    // Initialize to original values; first step happens on next cycle
    ovMask  := (nkmer - 1).U
    ovK1    := k1.asUInt
    ovK2    := k2.asUInt
    ovCount := 0.U
    ovBusy  := true.B
  }

  // One iteration per cycle (do-while step)
  when (state === sOverlap && ovBusy) {
    val nextMask = (ovMask >> 2).asUInt
    val nextK1   = (ovK1 & nextMask).asUInt
    val nextK2   = (ovK2 >> 2).asUInt
    val nextCnt  = ovCount + 1.U
    val matched  = nextK1 === nextK2

    ovMask  := nextMask
    ovK1    := nextK1
    ovK2    := nextK2
    ovCount := nextCnt

    when (matched || (nextCnt === kmerLenU)) {
      // Finish overlap calculation
      emitTotal := nextCnt          // emit exactly 'overlap' bases from currKmer
      emitIdx   := 0.U
      ovBusy    := false.B
      state     := sEmitSuffix
    }
  }

  // -----------------------
  // FSM
  // -----------------------
  switch (state) {
    is (sIdle) {
      when (io.start) {
        seqRemain    := io.nWords
        doneReg      := false.B
        emitIdx      := 0.U
        emitTotal    := 0.U
        ovBusy       := false.B
	basesSeenReg := 0.U
        state        := sFindFirst
      }
    }

    // Find first non-negative (skip "stays")
    is (sFindFirst) {
      when (io.in.fire) {
        seqRemain := seqRemain - 1.U
        when (io.in.bits >= 0.S) {
          prevKmer  := io.in.bits
          currKmer  := io.in.bits
          emitTotal := kmerLenU       // emit full k-mer first
          emitIdx   := 0.U
          state     := sEmitInit
        }.otherwise {
          // keep searching unless we've consumed everything
          when (seqRemain === 0.U) { 
	    state := sDone; 
	    doneReg := true.B 
 	  }
        }
      }
    }

    // Emit the first k-mer fully (MSB-first)
    is (sEmitInit) {
      // idx = (kmerLen-1) .. 0 → MSB-first
      val idxFromMsb = (kmerLenU - 1.U) - emitIdx
      val base2b     = getBase2b(currKmer, idxFromMsb)
      pushBase(base2b)

      emitIdx := emitIdx + 1.U
      when (emitIdx === (emitTotal - 1.U)) {
        state := sReadNext
      }
    }

    // Read subsequent k-mers until we consume nWords
    is (sReadNext) {
      when (seqRemain === 0.U) {
        // nothing more to read; we are done
        state  := sDone
        doneReg:= true.B
      }.elsewhen (io.in.fire) {
        seqRemain := seqRemain - 1.U
        when (io.in.bits >= 0.S) {
          // Start overlap with previous and this k-mer
          currKmer := io.in.bits
          overlapStart(prevKmer, io.in.bits)
          prevKmer := io.in.bits
          state    := sOverlap
        }.otherwise {
          // stay → ignore, keep reading
        }
      }
    }

    // Compute overlap (handled above in ovBusy block)
    is (sOverlap) {
      // wait for ovBusy to finish → state transitions to sEmitSuffix in the same always block
    }

    // Emit exactly 'emitTotal' bases from currKmer (MSB-first of suffix)
    is (sEmitSuffix) {
      val idxFromMsb = (emitTotal - 1.U) - emitIdx
      val base2b     = getBase2b(currKmer, idxFromMsb)
      pushBase(base2b)

      emitIdx := emitIdx + 1.U
      when (emitIdx === (emitTotal - 1.U)) {
        state := sReadNext
      }
    }

    is (sDone) {
      // hold done high until next start
      when (!io.start) { /* stay */ }
      when (io.start) {
        // allow immediate restart
        seqRemain   := io.nWords
        doneReg     := false.B
        emitIdx     := 0.U
        emitTotal   := 0.U
        ovBusy      := false.B
        state       := sFindFirst
      }
    }
  }
}
/*
class KmerOverlapper(val nkmer: Int) extends Module {
  val io = IO(new Bundle {
    val in        = Flipped(Decoupled(SInt(16.W)))   // 16-bit k-mers; negative => sentinel
    val outBases  = Decoupled(UInt(128.W))           // packed 64 bases (2 bits each)
    val posOut    = Decoupled(UInt(16.W))            // position per non-stay k-mer
    val start     = Input(Bool())
    val baseOut   = Output(UInt(2.W))                // 2-bit base each time we emit one
    val baseOutValid  = Output(Bool())                 // 1-cycle pulse when a base is emitted
    val done      = Output(Bool())
  })

  // ----------------------------
  // Constants / Params
  // ----------------------------
  private val kmerLenI        = (log2Ceil(nkmer) + 1) / 2   // e.g., nkmer=1024 -> 5
  private val kmerLenU        = kmerLenI.U
  private val maxPackedBasesI = 64
  private val maxPackedBasesU = maxPackedBasesI.U

  // ----------------------------
  // State
  // ----------------------------
  val sIdle :: sWaitFirst :: sOutputInit :: sStream :: Nil = Enum(4)
  val state         = RegInit(sIdle)

  val prevKmer      = RegInit(0.S(16.W))
  val currentKmer   = Reg(SInt(16.W))
  val outIdx        = RegInit(0.U(6.W))
  val ol            = Reg(UInt(6.W))
  val decoding      = RegInit(false.B)

  // 2-bit register for the **current base** (00=A,01=C,10=G,11=T)
  val baseReg       = RegInit(0.U(2.W))
  io.baseOut := baseReg

  // Output packing (128 bits = 64 bases × 2b)
  val baseBuffer    = RegInit(0.U(128.W))   // live accumulator
  val baseCount     = RegInit(0.U(6.W))     // 0..64 bases accumulated
  val outWordBuf    = RegInit(0.U(128.W))   // latched word to present on outBases
  val haveWord      = RegInit(false.B)      // hold valid until sink fires
  val baseIndex     = RegInit(0.U(16.W))    // total bases emitted (for posOut)

  // Position side-channel (C-like semantics: appended-count offset)
  val posValid      = RegInit(false.B)
  val posToSend     = Reg(UInt(16.W))

  // Done latch
  val doneReg       = RegInit(false.B)

  // ----------------------------
  // I/O defaults
  // ----------------------------
  io.in.ready        := false.B
  io.outBases.valid  := haveWord
  io.outBases.bits   := outWordBuf
  io.posOut.valid    := posValid
  io.posOut.bits     := posToSend
  io.done            := doneReg

  // ----------------------------
  // Helpers
  // ----------------------------
  // Extract the 2-bit base at index `idx` (0 = LSB-side, but we use MSB-first below)
  private def getBase2b(kmer: SInt, idx: UInt): UInt = {
    ((kmer >> (idx * 2.U)) & 3.S).asUInt // 0..3
  }
  // One-cycle valid strobe (wire) — goes high exactly in cycles we emit a base
  val baseOutValidW = RegInit(false.B)
  baseOutValidW := false.B
  io.baseOutValid := baseOutValidW
  // Push exactly one base:
  //  - update baseReg so you can observe the 2-bit code
  //  - pack it immediately into baseBuffer
  private def pushBase(base2b: UInt): Unit = {
    baseReg     := base2b
    baseOutValidW  := true.B          // <-- 1-cycle strobe
    baseBuffer  := baseBuffer | (base2b << (baseCount * 2.U))
    baseCount   := baseCount + 1.U
    baseIndex   := baseIndex + 1.U
  }

  // FIRST-match overlap (smallest i) like your C reference
  private def computeOverlap(k1: SInt, k2: SInt): UInt = {
    val k1u = k1.asUInt
    val k2u = k2.asUInt
    var res = kmerLenU // default = full length
    for (i <- 1 until kmerLenI) {
      val shift = (i * 2).U
      val maskW = (kmerLenU * 2.U) - shift
      val mask  = ((1.U << maskW) - 1.U)
      res = Mux((k1u & mask) === (k2u >> shift), i.U, res)
    }
    res
  }

  // ----------------------------
  // Output handshakes
  // ----------------------------
  // When 64 bases are accumulated, present a 128-bit word and hold valid
  when (baseCount === maxPackedBasesU && !haveWord) {
    outWordBuf := baseBuffer
    haveWord   := true.B
    baseBuffer := 0.U
    baseCount  := 0.U
  }
  when (io.outBases.fire) { haveWord := false.B }

  // Position output completes on fire
  when (io.posOut.fire) { posValid := false.B }

  // ----------------------------
  // FSM
  // ----------------------------
  switch (state) {
    is (sIdle) {
      when (io.start) {
        decoding    := false.B
        baseIndex   := 0.U
        baseCount   := 0.U
        baseBuffer  := 0.U
        outWordBuf  := 0.U
        haveWord    := false.B
        doneReg     := false.B
        state       := sWaitFirst
      }
    }

    is (sWaitFirst) {
      io.in.ready := true.B
      when (io.in.valid) {
        when (io.in.bits >= 0.S) {
          currentKmer := io.in.bits
          outIdx      := 0.U
          decoding    := true.B
          state       := sOutputInit

          // C-style pos: cumulative appended count (exclude first k-mer length)
          posToSend   := Mux(baseIndex >= kmerLenU, baseIndex - kmerLenU, 0.U)
          posValid    := true.B
        } .otherwise {
          //doneReg := true.B
        }
      }
    }

    is (sOutputInit) {
      when (decoding && (outIdx < kmerLenU)) {
        // MSB-first over the first k-mer: idx = kmerLen-1, kmerLen-2, ...
        val base2b = getBase2b(currentKmer, kmerLenU - outIdx - 1.U)
        pushBase(base2b)                 // <-- updates baseReg and packs into buffer
        outIdx := outIdx + 1.U
        when (outIdx === (kmerLenU - 1.U)) {
          decoding := false.B
          prevKmer := currentKmer
          state    := sStream
        }
      }
    }

    is (sStream) {
      // Take a new k-mer when not currently emitting bases
      io.in.ready := !decoding

      when (io.in.valid && !decoding) {
        val kmerIn = io.in.bits
        when (kmerIn >= 0.S) {
          val overlapLen  = computeOverlap(prevKmer, kmerIn)
          currentKmer     := kmerIn
          ol              := overlapLen
          outIdx          := 0.U
          decoding        := true.B
          prevKmer        := kmerIn

          posToSend       := Mux(baseIndex >= kmerLenU, baseIndex - kmerLenU, 0.U)
          posValid        := true.B
        } .otherwise {
          // Sentinel → done; allow any pending 128b word to flush
          //doneReg := true.B
        }
      }

      // Emit exactly `ol` bases from the new k-mer (MSB-first of the suffix)
      when (decoding) {
        val base2b = getBase2b(currentKmer, (ol - outIdx - 1.U))
        pushBase(base2b)                 // <-- updates baseReg and packs
        outIdx := outIdx + 1.U
        when (outIdx === (ol - 1.U)) {
          decoding := false.B
        }
      }
    }
  }
}
*/
/*
import chisel3._
import chisel3.util._

class KmerOverlapper(val nkmer: Int) extends Module {
  val io = IO(new Bundle {
    val in        = Flipped(Decoupled(SInt(16.W)))   
    val outBases  = Decoupled(UInt(128.W))           // Packed 64 bases (2 bits each)
    val posOut    = Decoupled(UInt(16.W))            // Position of each non-stay k-mer
    val start     = Input(Bool())
    val baseOut   = Output(UInt(2.W))
    val done      = Output(Bool())
  })

  // Constants
  val kmerLen = (log2Ceil(nkmer) + 1) / 2  // e.g., 1024 → 5
  val maxPackedBases = 64  // 64 × 2 bits = 128 bits

  // FSM states
  val sIdle :: sWaitFirst :: sOutputInit :: sStream :: Nil = Enum(4)
  val state = RegInit(sIdle)

  // Internal registers
  val prevKmer       = RegInit(0.S(16.W))   // ← Changed to 16 bits
  val currentKmer    = Reg(SInt(16.W))      // ← Changed to 16 bits
  val outIdx         = RegInit(0.U(6.W))
  val ol             = Reg(UInt(6.W))
  val decoding       = RegInit(false.B)
  val firstKmerSeen  = RegInit(false.B)

  val baseReg      = RegInit(0.U(2.W)).suggestName("baseReg")  


  // Packed base output buffer
  val baseBuffer = RegInit(0.U(128.W))
  val baseCount  = RegInit(0.U(6.W))  // 64 = 128 bits

  // Output position tracking
  val baseIndex  = RegInit(0.U(16.W))
  val posValid   = RegInit(false.B)
  val posToSend  = Reg(UInt(16.W))

  // I/O defaults
  io.in.ready        := false.B
  io.outBases.valid  := false.B
  io.outBases.bits   := baseBuffer
  io.posOut.valid    := posValid
  io.posOut.bits     := posToSend
  io.done            := false.B
  io.baseOut         := baseReg
  // ----------------------------------
  // Helper: extract base from k-mer
  def getBase(kmer: SInt, idx: UInt): UInt = {
    ((kmer >> (idx * 2.U)) & 3.S).asUInt
  }

  // Helper: emit base to buffer
  def emitBase(base: UInt): Unit = {
    baseBuffer := baseBuffer | (base << (baseCount * 2.U))
    baseCount := baseCount + 1.U
    baseIndex := baseIndex + 1.U
  }

  val haveWord = RegInit(false.B)
  	

  // Output base buffer if full
  when(baseCount === maxPackedBases.U && io.outBases.ready) {
    io.outBases.valid := true.B
    baseBuffer := 0.U
    baseCount := 0.U
  }

  // Fire position output
  when(io.posOut.fire) {
    haveWord   := false.B
    posValid := false.B
    baseCount := 0.U
  }

  when(io.done && (baseCount =/= 0.U) && !haveWord) {
    haveWord := true.B
  }

  // ----------------------------------
  // Overlap Computation (safe Chisel style)
  /*def computeOverlap(k1: SInt, k2: SInt): UInt = {
    val k1u = k1.asUInt
    val k2u = k2.asUInt
    val overlap = WireDefault(kmerLen.U)

    for (i <- 1 until kmerLen) {
      val shift = i * 2
      val mask = ((1 << (kmerLen * 2 - shift)) - 1).U
      val k1Masked = k1u & mask
      val k2Shifted = k2u >> shift.U
      when(k1Masked === k2Shifted) {
        overlap := i.U
      }
    }

    overlap
  }*/
  def computeOverlap(k1: SInt, k2: SInt): UInt = {
  val k1u = k1.asUInt
  val k2u = k2.asUInt

  // Start with maximum overlap = kmerLen
  var result = kmerLen.U

  for (i <- (1 until kmerLen).reverse) { // reverse so smallest shift has priority
    val shift = i * 2
    val mask  = ((1 << (kmerLen * 2 - shift)) - 1).U
    val k1Masked   = k1u & mask
    val k2Shifted  = k2u >> shift.U
    val equal = k1Masked === k2Shifted
    result = Mux(equal, i.U, result)  // pure combinational, no wire loops
  }

  result
  }
  // ----------------------------------
  // FSM Logic
  switch(state) {
    is(sIdle) {
      when(io.start) {
        firstKmerSeen := false.B
        decoding := false.B
        baseIndex := 0.U
        baseCount := 0.U
        baseBuffer := 0.U
        state := sWaitFirst
      }
    }

    is(sWaitFirst) {
      io.in.ready := true.B
      when(io.in.valid) {
        when(io.in.bits >= 0.S) {
          currentKmer := io.in.bits
          firstKmerSeen := true.B
          outIdx := 0.U
          decoding := true.B
          state := sOutputInit

          posToSend := baseIndex
          posValid := true.B
        } .otherwise {
          io.done := true.B
        }
      }
    }

    is(sOutputInit) {
      when(decoding && outIdx < kmerLen.U) {
        val base = getBase(currentKmer, kmerLen.U - outIdx - 1.U)
        baseReg := base
	emitBase(base)
        outIdx := outIdx + 1.U
        when(outIdx === (kmerLen - 1).U) {
          decoding := false.B
          prevKmer := currentKmer
          state := sStream
        }
      }
    }

    is(sStream) {
      io.in.ready := !decoding

      when(io.in.valid && !decoding) {
        val kmerIn = io.in.bits
        when(kmerIn >= 0.S) {
          val overlapLen = computeOverlap(prevKmer, kmerIn)
          currentKmer := kmerIn
          ol := overlapLen
          outIdx := 0.U
          decoding := true.B
          prevKmer := kmerIn

          posToSend := baseIndex
          posValid := true.B
        } .otherwise {
          //io.done := true.B
        }
      }

      when(decoding) {
        val base = getBase(currentKmer, ol - outIdx - 1.U)
        emitBase(base)
        outIdx := outIdx + 1.U
        when(outIdx === (ol - 1.U)) {
          decoding := false.B
        }
      }
    }
  }
}
*/
