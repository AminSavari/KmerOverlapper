/*
// overlapper_poller.c — bare-metal, no interrupts, polling-only
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "mmio.h"     // reg_write8/32, reg_read8/32 (your existing header)
#include "seq_sse.h"  // provides: extern const int16_t seq[];

// ===== Addresses must match your linker script =====
#define MMIO_BASE   0x00004000UL
#define IN_BASE     0x80800000UL
#define OUT_BASE    0x80A00000UL

// ---------- Accelerator register map ----------
#define REG_TRIGGER         (MMIO_BASE + 0x00)  // W: 1 => start
#define REG_SRC_ADDR        (MMIO_BASE + 0x04)  // W: physical src addr
#define REG_DST_ADDR        (MMIO_BASE + 0x08)  // W: physical dst addr
#define REG_SEQ_LEN16       (MMIO_BASE + 0x0C)  // W: number of 16-bit words in src
#define REG_STATUS_DONE     (MMIO_BASE + 0x10)  // R: bit0 == 1 when done
#define REG_BYTES_WRITTEN   (MMIO_BASE + 0x14)  // R: bytes HW wrote
#define REG_BASES_SEEN      (MMIO_BASE + 0x24)  // (optional debug)

// ---------- Tuning (software-side only) ----------
#define MAX_SEQ_ELEMS   20000      // support up to 20k int16_t inputs
#define MAX_KMER_LEN    8          // worst-case 'k' you expect (nkmer=4^k)
#define NKMER_FROM_K(k) (1u << (2*(k)))

// Derived: worst-case software output bytes (all are non-stays)
#define SW_DST_MAX_BYTES  (MAX_SEQ_ELEMS * MAX_KMER_LEN + 1)

// ---------- DMA windows placed by the linker ----------
__attribute__((section(".dma_src"), aligned(64)))
volatile int16_t dma_src_buf[MAX_SEQ_ELEMS];

__attribute__((section(".dma_dst"), aligned(64)))
volatile char dma_dst_buf[SW_DST_MAX_BYTES];   // used both as HW output window and SW scratch

// ---------- RISC-V full IO fence ----------
static inline void fence_io(void){ __asm__ volatile("fence iorw, iorw" ::: "memory"); }

// ---------- Software overlapper (no malloc) ----------
static const char base_lookup[4] = { 'A', 'C', 'G', 'T' };

static int overlap_sw(int k1, int k2, int nkmer) {
    assert(k1 >= 0); assert(k2 >= 0);
    int kmer_mask = nkmer - 1;
    int ol = 0;
    do {
        kmer_mask >>= 2;
        k1 &= kmer_mask;
        k2 >>= 2;
        ol += 1;
    } while (k1 != k2);
    return ol;
}

static size_t position_highest_bit(size_t x) {
    size_t i = 0; for (; x != 0; i++, x >>= 1) ; return i;
}

static size_t first_nonnegative(const int16_t *s, size_t n) {
    size_t i=0; for (; i<n && s[i]<0; i++) ; return i;
}

// compute output length (without '\0')
static size_t overlapper_sw_len(const int16_t *s, size_t n, uint16_t nkmer)
{
    const size_t kmer_len = position_highest_bit(nkmer)/2;
    const size_t st = first_nonnegative(s, n);
    if (st == n) return 0;

    size_t len = kmer_len;
    int kprev = s[st];
    for (size_t k = st + 1; k < n; k++) {
        if (s[k] < 0) continue;
        len += (size_t)overlap_sw(kprev, s[k], nkmer);
        kprev = s[k];
    }
    return len;
}

/* Fill into 'out' (cap=out_cap). Returns produced length on success.
 * If out_cap < length+1, returns the required capacity (length+1) and writes nothing.
 * If pos!=NULL, it must have at least n entries; pos[k] is updated for all k.
 */
/*
static size_t overlapper_sw_fill(const int16_t *s, size_t n,
                                 uint16_t nkmer, uint16_t *pos,
                                 char *out, size_t out_cap)
{
    const size_t kmer_len = position_highest_bit(nkmer)/2;
    const size_t st = first_nonnegative(s, n);
    if (st == n) { if (out_cap) out[0]='\0'; if (pos) pos[0]=0; return 0; }

    const size_t length = overlapper_sw_len(s, n, nkmer);
    if (out_cap < (length + 1)) return length + 1;

    // first k-mer
    for (size_t kmer = (size_t)s[st], k = 1; k <= kmer_len; k++) {
        const size_t b = kmer & 3u;
        kmer >>= 2;
        out[kmer_len - k] = base_lookup[b];
    }
    if (pos) pos[0] = 0;

    size_t last_idx = kmer_len - 1;
    int kprev = s[st];

    for (size_t k = st + 1; k < n; k++) {
        if (s[k] < 0) { if (pos) pos[k] = pos[k-1]; continue; }

        const int ol = overlap_sw(kprev, s[k], nkmer);
        if (pos) pos[k] = pos[k-1] + (uint16_t)ol;
        kprev = s[k];

        for (size_t kmer = (size_t)s[k], i = 0; i < (size_t)ol; i++) {
            const size_t b = kmer & 3u;
            kmer >>= 2;
            out[last_idx + (size_t)ol - i] = base_lookup[b];
        }
        last_idx += (size_t)ol;
    }

    out[length] = '\0';
    return length;
}

static inline uint8_t base_to_num(char c){
    switch(c){ case 'A': return 0; case 'C': return 1; case 'G': return 2; case 'T': return 3; default: return 0; }
}

// ========== Demo main (polling) ==========
int main(void)
{
    printf("[CORE] Test start\n");

    // 1) Prepare input in .dma_src
    const size_t seq_size = sizeof(seq)/sizeof(seq[0]);
    const size_t n = (seq_size > MAX_SEQ_ELEMS) ? MAX_SEQ_ELEMS : seq_size;

    for (size_t i=0; i<n; i++) { dma_src_buf[i] = seq[i]; }
    fence_io();
    printf("[CORE] Wrote %zu 16-bit kmers to IN_BASE\n", n);

    // 2) Program MMIO + fire
    reg_write32(REG_SRC_ADDR, IN_BASE);
    reg_write32(REG_DST_ADDR, OUT_BASE);
    reg_write32(REG_SEQ_LEN16, (uint32_t)n);
    fence_io();

    reg_write8(REG_TRIGGER, 1);
    printf("[CORE] Triggered; polling DONE...\n");

    // 3) Poll for DONE
    //while ((reg_read8(REG_STATUS_DONE) & 0x1u) == 0u) {  }
    fence_io();

    // 4) Read HW byte count
    const uint32_t bytes_hw = reg_read32(REG_BYTES_WRITTEN);
    printf("[CORE] DONE. bytes_hw=%u\n", bytes_hw);

    // 5) Software reconstruction (no heap)
    static uint16_t pos_buf[MAX_SEQ_ELEMS];       // optional; can pass NULL
    const uint16_t nkmer = NKMER_FROM_K(5);       // <-- set your 'k' here (e.g., k=5 => 1024)
    char *bases_sw = (char*)(uintptr_t)dma_dst_buf;  // reuse .dma_dst as scratch for SW too
    const size_t sw_cap = (size_t)SW_DST_MAX_BYTES;

    const size_t need = overlapper_sw_len((const int16_t*)dma_src_buf, n, nkmer) + 1;
    if (need > sw_cap) {
        printf("[CORE] SW buffer too small: need=%zu cap=%zu\n", need, sw_cap);
        // You can early-exit or only compare the prefix that fits.
    }

    const size_t nbases_sw = overlapper_sw_fill((const int16_t*)dma_src_buf, n,
                                                nkmer, pos_buf,
                                                bases_sw, sw_cap);
    if (nbases_sw == need-1 || need <= sw_cap)
        printf("[CORE] SW reconstructed %zu bases\n", nbases_sw);

    // 6) Compare HW vs SW (on-the-fly convert SW char->num; no extra buffers)
    volatile const uint8_t *out_hw = (volatile const uint8_t*)(uintptr_t)OUT_BASE;

    const size_t cmp = (bytes_hw < nbases_sw) ? bytes_hw : nbases_sw;
    size_t mism = 0;
    for (size_t i = 0; i < cmp; i++) {
        const uint8_t hw = out_hw[i];
        const uint8_t sw = base_to_num(bases_sw[i]);
        if (hw != sw) {
            if (mism < 16)
                printf("Mismatch @%zu: SW=%u HW=%u (char=%c)\n", i, (unsigned)sw, (unsigned)hw, bases_sw[i]);
            mism++;
        }
    }

    if (mism == 0 && bytes_hw == nbases_sw) {
        printf("[CORE] PASS: exact match (%zu bases)\n", nbases_sw);
    } else if (mism == 0) {
        printf("[CORE] PASS (length mismatch): compared=%zu SW=%zu HW=%u\n", cmp, nbases_sw, bytes_hw);
    } else {
        printf("[CORE] FAIL: %zu mismatches out of %zu compared (SW=%zu, HW=%u)\n",
               mism, cmp, nbases_sw, bytes_hw);
    }

    // Optional: dump first few outputs
    const size_t dump = (bytes_hw < 32 ? bytes_hw : 32);
    for (size_t i=0; i<dump; i+=4) {
        uint32_t w = 0;
        if (i + 3 < bytes_hw) {
            w = ((uint32_t)out_hw[i]) | ((uint32_t)out_hw[i+1] << 8)
              | ((uint32_t)out_hw[i+2] << 16) | ((uint32_t)out_hw[i+3] << 24);
        }
        printf("HW[%02zu..%02zu] = 0x%08X\n", i, i+3, w);
    }

    printf("[CORE] Test complete\n");
    for(;;) {  }
}
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <sys/time.h>
#include <stddef.h>

#include "seq_sse.h"
#include "mmio.h"

// PLIC base from DTS
#define PLIC_BASE            0x0C000000UL

// One source total → your accelerator must be source ID 1
#define OVERLAPPER_IRQ_ID    1

// Context 0 = M-mode for hart 0 (first entry in interrupts-extended)
#define HARTID               0
#define PLIC_PRI(id)         (PLIC_BASE + 4*(id))
#define PLIC_ENABLE_M(h)     (PLIC_BASE + 0x2000   + 0x100*(h))
#define PLIC_THRESHOLD_M(h)  (PLIC_BASE + 0x200000 + 0x1000*(h))
#define PLIC_CLAIM_M(h)      (PLIC_BASE + 0x200004 + 0x1000*(h))


// ====== TUNE THESE TO YOUR SYSTEM ====== 
#define MMIO_BASE   0x4000UL       // base of KmerOverlapperTester MMIO window 
#define IN_BASE     0x80800000UL   // DRAM src buffer (16-bit kmers) 
#define OUT_BASE    0x80A00000UL   // DRAM dst buffer (bytes 0..3)

#define BASE_ADDR ((volatile int16_t *)0x80800000)   
// ====================================== 

#define KMERSIZE    5              // k-mer bases (2 bits/base)  -> nkmer = 4^KMERSIZE 
#define SEQ_LEN     20             // number of input kmers (16-bit words) 
#define ALIGNMENT   8		   // Align to 16 bytes
// ---- MMIO offsets (must match the wrapper regmap) ---- 
#define REG_TRIGGER         (MMIO_BASE + 0x00)
#define REG_SRC_ADDR        (MMIO_BASE + 0x04)
#define REG_DST_ADDR        (MMIO_BASE + 0x08)
#define REG_SEQ_LEN16       (MMIO_BASE + 0x0C)
#define REG_STATUS_DONE     (MMIO_BASE + 0x10)  // bit0 = 1 when done 
#define REG_BYTES_WRITTEN   (MMIO_BASE + 0x14)
#define REG_BASE            (MMIO_BASE + 0x18)  // bit0 = 1 when done
#define REG_IRQ_EN          (MMIO_BASE + 0x1C)  // RW: 1 enables device to raise IRQ line
#define REG_IRQ_IP          (MMIO_BASE + 0x20)  // W1C: write 1 to clear device pending
#define REG_BASES_SEEN      (MMIO_BASE + 0x24)

//----------------------------------------------------
//  INTERRUPT CONTROLLER
//----------------------------------------------------

static volatile uint32_t g_accel_done = 0;

static inline void plic_set_priority(uint32_t id, uint32_t prio){ reg_write32(PLIC_PRI(id), prio); }
static inline void plic_enable_src(uint32_t hart, uint32_t id){
    uint32_t v = reg_read32(PLIC_ENABLE_M(hart));
    v |= (1u << id);
    reg_write32(PLIC_ENABLE_M(hart), v);
}
static inline void plic_set_threshold(uint32_t hart, uint32_t th){ reg_write32(PLIC_THRESHOLD_M(hart), th); }
static inline uint32_t plic_claim(uint32_t hart){ return reg_read32(PLIC_CLAIM_M(hart)); }
static inline void plic_complete(uint32_t hart, uint32_t id){ reg_write32(PLIC_CLAIM_M(hart), id); }

static inline void enable_m_interrupts(void){
    asm volatile ("csrs mstatus, %0" :: "r"(0x8));   // MIE
    asm volatile ("csrs mie, %0"     :: "r"(0x800)); // MEIE
}

void trap_handler(void) __attribute__((interrupt));
void trap_handler(void){
    uint32_t id = plic_claim(HARTID);
    if (id == OVERLAPPER_IRQ_ID) {
        // If your device exposes a W1C pending bit, clear it:
        // reg_write8(REG_IRQ_IP, 1);
        g_accel_done = 1;
    }
    plic_complete(HARTID, id);
}
static inline void csr_write_mtvec(void (*h)(void)){
    uintptr_t x = (uintptr_t)h;
    asm volatile ("csrw mtvec, %0" :: "r"(x));
}


//extern const char base_lookup[4];
char *overlapper_sw(const int16_t *seq, size_t n, uint16_t nkmer, uint16_t *pos);

// Utility: convert base letter to numeric encoding (0=A, 1=C, 2=G, 3=T)
static inline int base_to_num(char base) {
    switch (base) { case 'A': return 0; case 'C': return 1; case 'G': return 2; case 'T': return 3; default: return 0; }
}

static size_t bases_to_nums(const char* bases, uint8_t* out) {
    size_t n = 0; for (; bases[n] != '\0'; n++) out[n] = (uint8_t)base_to_num(bases[n]); return n;
}


// Unpack LSb-first 2-bit bases from a byte stream.
// - src: packed bytes from OUT_BASE
// - src_bytes: number of bytes written by HW (REG_BYTES_WRITTEN)
// - dst: where to put unpacked bases (0..3 each)
// - dst_cap: capacity of dst in *bases*
// - base_limit: if >0, stop after exactly base_limit bases (use REG_BASES_SEEN if available)
static size_t unpack_2b_stream(volatile const uint8_t *src,
                               size_t src_bytes,
                               uint8_t *dst, size_t dst_cap,
                               size_t base_limit)
{
    size_t produced = 0;
    for (size_t i = 0; i < src_bytes; i++) {
        uint8_t b = src[i];
        // Extract 4 bases from this byte, LSb-first (00,01,10,11)
        for (int j = 0; j < 4; j++) {
            if (produced >= dst_cap) return produced;
            if (base_limit && produced >= base_limit) return produced;
            dst[produced++] = (uint8_t)(b & 0x3u);
            b >>= 2;
        }
    }
    return produced;
}
static char code_to_base(uint8_t x) {
    static const char LUT[4] = { 'A','C','G','T' };
    return LUT[x & 3u];
}
// ====== Main test ======  
int main(void) {

    printf("[CORE] Test started.\n");

    uint32_t seq_size = (uint32_t)(sizeof(seq)/sizeof(seq[0]));

    int16_t * pos = calloc(seq_size + 1, sizeof(int16_t));    

    printf("[CORE] Memory allocated with %d elements.\n", seq_size);

    __asm__ volatile ("fence");
    for (int i = 0; i < seq_size; i++) {
        BASE_ADDR[i] = seq[i];
    }
    __asm__ volatile("fence":::"memory");

     
    printf("[CORE] wrote the sequence in memory.\n");

    printf("[CORE] configuring the DMA.\n");
    // --- Program MMIO and start --- 
    uint32_t src = IN_BASE;
    uint32_t dst = OUT_BASE;
    reg_write32(REG_SRC_ADDR, src);
    reg_write32(REG_DST_ADDR, dst);
    reg_write32(REG_SEQ_LEN16,  seq_size);

    // PLIC: priority 1 (max allowed), enable source, threshold 0
    //plic_set_priority(OVERLAPPER_IRQ_ID, 1);
    //plic_enable_src(HARTID, OVERLAPPER_IRQ_ID);
    //plic_set_threshold(HARTID, 0);

    // hook ISR and enable M-mode external interrupts
    //csr_write_mtvec(trap_handler);
    //enable_m_interrupts();


    // fire 
    reg_write8(REG_TRIGGER, 1);

    printf("[CORE] Start the overlapper.\n");
    // sleep until interrupt
    //while (!g_accel_done) { asm volatile ("wfi"); }
    // --- Wait for DONE --- 
    while ((reg_read8(REG_STATUS_DONE) & 0x1u) == 0u) {  }
    printf("[CORE] Overlapper is done!\n");
    // How many bytes HW says it wrote 
    uint32_t bytes_hw = reg_read32(REG_BYTES_WRITTEN);
    printf("[CORE] bytes_hw = %u (0x%08x)\n", bytes_hw, bytes_hw);
    
    /*if (bytes_hw == 0) {
        fprintf(stderr, "ERROR: bytes_written=0\n");
        return 1;
    }*/
    
    // --- Read back HW result --- 
    volatile uint8_t *outbuf = (volatile uint8_t*)(uintptr_t)OUT_BASE;
    //flush_dcache_range((void*)outbuf, bytes_hw);
    //fence_rw();
/*
    size_t ncheck = bytes_hw;              // HW writes bytes (0..3 each) 
    //if (ncheck > nbases_sw) ncheck = nbases_sw; // compare overlap 

    uint8_t *nums_hw = (uint8_t*)malloc(ncheck);
    //if (!nums_hw) { perror("malloc"); return 1; } 
    __asm__ volatile ("fence");
    for (size_t i = 0; i < ncheck; i++) 
	nums_hw[i] = outbuf[i];
    __asm__ volatile("fence":::"memory");
*/
uint32_t bases_hw_reg = 0;


// Allocate space for the worst case: 4 bases per byte
size_t hw_cap_bases = (size_t)bytes_hw * 4u;
uint8_t *nums_hw = (uint8_t*)malloc(hw_cap_bases ? hw_cap_bases : 1);
//if (!nums_hw) { printf("malloc failed\n"); for(;;){} }

__asm__ volatile ("fence");  // ensure visibility before reading

// Unpack LSb-first 2-bit bases into nums_hw[]
size_t nbases_hw = unpack_2b_stream(outbuf, bytes_hw, nums_hw, hw_cap_bases, (size_t)bases_hw_reg);

//__asm__ volatile("fence":::"memory");
for (size_t i = 0; i < nbases_hw; i++)
    putchar(code_to_base(nums_hw[i]));
putchar('\n');
//    printf("[CORE] read the bases from the memory!\n");
 /*    
    // --- Software overlapper ---
    __asm__ volatile ("fence"); 
    char *bases_sw = overlapper_sw(seq, seq_size , 1024, pos);
    __asm__ volatile("fence":::"memory");
    //if (!bases_sw) { fprintf(stderr, "overlapper_sw failed\n"); return 1; }
    const size_t nbases_sw = strlen(bases_sw); 
    printf("[CORE] SW reconstructed %d bases\n", (uint16_t)(nbases_sw) );
    
   uint8_t *nums_sw = (uint8_t*)malloc(nbases_sw);
    if (!nums_sw) { perror("malloc"); return 1; }
    bases_to_nums(bases_sw, nums_sw);

    printf("[CORE] bases coverted to numbers\n");
size_t cmp = (nbases_hw < nbases_sw) ? nbases_hw : nbases_sw;
size_t mism = 0;
for (size_t i = 0; i < cmp; i++) {
    uint8_t sw = (bases_sw[i] == 'A') ? 0 :
                 (bases_sw[i] == 'C') ? 1 :
                 (bases_sw[i] == 'G') ? 2 : 3;
    if (nums_hw[i] != sw) {
        if (mism < 16)
            printf("Mismatch @%zu: SW=%u HW=%u (char=%c)\n",
                   i, (unsigned)sw, (unsigned)nums_hw[i], bases_sw[i]);
        mism++;
    }
}

if (mism == 0 && nbases_hw == nbases_sw) {
    printf("PASS: exact match (%zu bases)\n", nbases_sw);
} else if (mism == 0) {
    // lengths can differ because bytes are ceil(2*N/8)
    printf("PASS (prefix): matched %zu bases; SW=%zu, HW=%zu, bytes=%u\n",
           cmp, nbases_sw, nbases_hw, bytes_hw);
} else {
    printf("FAIL: %zu mismatches out of %zu compared; SW=%zu, HW=%zu, bytes=%u\n",
           mism, cmp, nbases_sw, nbases_hw, bytes_hw);
}
*/
    // --- Compare --- 
/*    size_t mism = 0;
    for (size_t i = 0; i < ncheck; i++) {
        if (nums_sw[i] != nums_hw[i]) {
            if (mism < 16) printf("Mismatch @%zu: SW=%u HW=%u\n", i, (unsigned)nums_sw[i], (unsigned)nums_hw[i]);
            mism++;
        }
    }
    printf("[CORE] bases are compared\n");
    if (mism == 0 && ncheck == nbases_sw) {
        printf("PASS: HW matches SW (%zu bases)\n", nbases_sw);
    } else if (mism == 0) {
        printf("PASS (partial): matched %zu bases (SW=%zu, HW=%u)\n", ncheck, nbases_sw, bytes_hw);
    } else {
        printf("FAIL: %zu mismatches out of %zu compared (SW=%zu, HW=%u)\n",
               mism, ncheck, nbases_sw, bytes_hw);
    }
*/
    // (Optional) print a few tuples 
    //for (size_t i = 0; i < (ncheck < 64 ? ncheck : 64); i++) {
    //    printf("(%c,%u) vs %u\n", bases_sw[i], nums_sw[i], nums_hw[i]);
    //}
    
  
    free(nums_hw);
    //free(nums_sw);
    //free(bases_sw);
    free(pos);
    free(seq);

    return 0;
}

// ====== Software overlapper helpers (same as before) ====== 
//#define RETURN_NULL_IF(cond, retval) \
//    do { if (cond) return (retval); } while (0)

const char base_lookup[4] = { 'A', 'C', 'G', 'T' };


int overlap(int k1, int k2, int nkmer) {
    // Neither k1 nor k2 can be stays
    assert(k1 >= 0);
    assert(k2 >= 0);

    int kmer_mask = nkmer - 1;
    int overlap = 0;
    do {
        kmer_mask >>= 2;
        k1 &= kmer_mask;
        k2 >>= 2;
        overlap += 1;
    } while (k1 != k2);

    return overlap;
}

size_t position_highest_bit(size_t x) {
    size_t i = 0;
    for (; x != 0; i++, x >>= 1) ;
    return i;
}

size_t first_nonnegative(const int16_t *seq, size_t n) {
    //RETURN_NULL_IF(NULL == seq, n);
    size_t st;
    for (st = 0; st < n && seq[st] < 0; st++) ;
    return st;
}

char *overlapper_sw(const int16_t *seq, size_t n, uint16_t nkmer, uint16_t *pos) {

    //RETURN_NULL_IF(NULL == seq, NULL);
    const size_t kmer_len = position_highest_bit(nkmer) / 2;

    //  Determine length of final sequence
    size_t length = kmer_len;
    // Find first non-stay
    const size_t st = first_nonnegative(seq, n);
    //RETURN_NULL_IF(st == n, NULL);
    //printf("[CORE] in software\n");
    int kprev = seq[st];
    for (size_t k = st + 1; k < n; k++) {
        if (seq[k] < 0) {
            // Short-cut stays
            continue;
        }
        assert(seq[k] >= 0);
        length += overlap(kprev, seq[k], nkmer);
        kprev = seq[k];
        assert(kprev >= 0);
    }

    // Initialise basespace sequence with terminating null
    char *bases = calloc(length + 1, sizeof(char));
    //RETURN_NULL_IF(NULL == bases, NULL);

    // Fill with first kmer
    for (size_t kmer = seq[st], k = 1; k <= kmer_len; k++) {
        size_t b = kmer & 3;
        kmer >>= 2;
        bases[kmer_len - k] = base_lookup[b];
    }

    if(NULL != pos){
        // Initial pos array if required -- start at beginning
        pos[0] = 0;
    }
    for (size_t last_idx = kmer_len - 1, kprev = seq[st], k = st + 1; k < n; k++) {
        if (seq[k] < 0) {
            // Short-cut stays
            if (NULL != pos) {
                pos[k] = pos[k - 1];
            }
            continue;
        }
        int ol = overlap(kprev, seq[k], nkmer);
	size_t ol_sz = (size_t)ol;
        if (NULL != pos) {
            pos[k] = pos[k - 1] + ol;
        }
        kprev = seq[k];

        for (size_t kmer = seq[k], i = 0; i < ol_sz; i++) {
            size_t b = kmer & 3;
            kmer >>= 2;
            bases[last_idx + ol - i] = base_lookup[b];
        }
        last_idx += ol;
    }

    return bases;
}




// --- Write every 4 consecutive elements to 8 bytes of memory ---
    // Align the memory to an 8-byte boundary
    /*volatile uint64_t *mem_ptr = (volatile uint64_t *)((uintptr_t)IN_BASE & ~0x7);  
    // Write 4 consecutive 16-bit elements as a 64-bit word
    for (int i = 0; i < seq_size; i += 4) {
      if (i + 3 < seq_size) {
        uint64_t combined = 0;
        for (int j = 0; j < 4; j++) {
            combined |= ((uint64_t)(seq[i + j]) << (j * 16));
        }
        // Write the 64-bit word to memory
        *mem_ptr = combined;
        mem_ptr++;  // Move to the next 8-byte memory location
      }
    }*/  

     // --- Software overlapper --- 
    /*char *bases_sw = overlapper_sw(seq, seq_size + 1, 1024, pos);
    if (!bases_sw) { fprintf(stderr, "overlapper_sw failed\n"); return 1; }
    const size_t nbases_sw = strlen(bases_sw); 
    printf("[CORE] SW reconstructed %d bases\n", (uint16_t)(nbases_sw) );
    
    uint8_t *nums_sw = (uint8_t*)malloc(nbases_sw);
    if (!nums_sw) { perror("malloc"); return 1; }
    bases_to_nums(bases_sw, nums_sw);
    */
    /*

    // --- Write inputs to DRAM (16-bit) --- 
    //volatile uint16_t *inbuf = (volatile uint16_t*)(uintptr_t)IN_BASE;
    //for (int i = 0; i < SEQ_LEN; i++) inbuf[i] = (uint16_t)(seq[i] & 0xFFFF);

    //flush_dcache_range((void*)inbuf, SEQ_LEN * sizeof(uint16_t));
    //fence_rw();
    */
  

/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <sys/time.h>

#include "mmio.h"

// ====== TUNE THESE TO YOUR SYSTEM ====== 
#define MMIO_BASE   0x4000UL       // base of KmerOverlapperTester MMIO window 
#define IN_BASE     0x80800000UL   // DRAM src buffer (16-bit kmers) 
#define OUT_BASE    0x88000000UL   // DRAM dst buffer (bytes 0..3)

#define BASE_ADDR ((volatile uint64_t *)0x80800000)   
// ====================================== 

#define KMERSIZE    5              // k-mer bases (2 bits/base)  -> nkmer = 4^KMERSIZE 
#define SEQ_LEN     20             // number of input kmers (16-bit words) 
#define ALIGNMENT   8		   // Align to 16 bytes
// ---- MMIO offsets (must match the wrapper regmap) ---- 
#define REG_TRIGGER         (MMIO_BASE + 0x00)
#define REG_SRC_ADDR        (MMIO_BASE + 0x04)
#define REG_DST_ADDR        (MMIO_BASE + 0x08)
#define REG_SEQ_LEN16       (MMIO_BASE + 0x0C)
#define REG_STATUS_DONE     (MMIO_BASE + 0x10)  // bit0 = 1 when done 
#define REG_BYTES_WRITTEN   (MMIO_BASE + 0x14)


// Optional: light memory barrier 
static inline void fence_rw(void) {
#if defined(__riscv) || defined(__riscv__) || defined(__riscv_xlen)
  __asm__ __volatile__("fence rw, rw" ::: "memory");
#else
  __sync_synchronize();
#endif
}
// Optional: cache flush/invalidate stubs (fill with your platform calls if available) 
static inline void flush_dcache_range(void* addr, size_t size){ (void)addr; (void)size; }

// ====== Software overlapper helpers (same as before) ====== 
#define //RETURN_NULL_IF(cond, retval) \
    do { if (cond) return (retval); } while (0)

int overlap(int k1, int k2, int nkmer) ;

size_t position_highest_bit(size_t x);

size_t first_nonnegative(const int *seq, size_t n);

const char base_lookup[4] = { 'A', 'C', 'G', 'T' };

//extern const char base_lookup[4];
char *overlapper_sw(const int *seq, size_t n, int nkmer, int *pos);

// Utility: convert base letter to numeric encoding (0=A, 1=C, 2=G, 3=T)
static inline int base_to_num(char base) {
    switch (base) { case 'A': return 0; case 'C': return 1; case 'G': return 2; case 'T': return 3; default: return 0; }
}
static size_t bases_to_nums(const char* bases, uint8_t* out) {
    size_t n = 0; for (; bases[n] != '\0'; n++) out[n] = (uint8_t)base_to_num(bases[n]); return n;
}


static uint32_t seed = 123456789;  // Initial seed value, adjust as needed
uint32_t my_rand() {
    seed = seed * 1664525 + 1013904223;  // Parameters for LCG
    return seed;
}

// ====== Main test ====== 
int main(void) {
    printf("[CORE] Test started.\n");
    //srand((unsigned)time(NULL));

    const int nkmer = 1 << (2 * KMERSIZE);   // e.g., KMERSIZE=5 -> nkmer=1024 

    // Allocate memory for seq and pos arrays with extra space for alignment
    uint64_t buffer_seq[SEQ_LEN + 8];  // Extra space for alignment
    uint64_t buffer_pos[SEQ_LEN + 8];  // Extra space for alignment

    // Align the pointers to 16-byte boundary
    int *seq = (int*)(((uintptr_t)buffer_seq + (ALIGNMENT - 1)) & ~((uintptr_t)(ALIGNMENT - 1)));
    int *pos = (int*)(((uintptr_t)buffer_pos + (ALIGNMENT - 1)) & ~((uintptr_t)(ALIGNMENT - 1)));

    // Initialize the arrays (this is just an example, your initialization logic may vary)
    memset(seq, 0, SEQ_LEN * sizeof(int));
    memset(pos, 0, SEQ_LEN * sizeof(int));
    __asm__ volatile ("fence");
    __asm__ volatile("fence":::"memory");

    printf("[CORE] Memory allocated.\n");
    // Build a sequence of valid kmers (NO -1 inside; HW uses a sentinel injected by the wrapper) 
    
    for (int i = 0; i < SEQ_LEN; i++) seq[i] = my_rand() % nkmer;
    
    // --- Print sequence --- 
    printf("[CORE] Sequence values (len=%d):\n", (uint32_t)(SEQ_LEN) );
    for (int i = 0; i < SEQ_LEN; i++) {
        printf("%4d", seq[i]);
        if ((i+1) % 16 == 0) printf("\n");
    }
    printf("\n----------------------------------------\n");


    // --- Write every 4 consecutive elements to 8 bytes of memory ---
    // Align the memory to an 8-byte boundary
uint64_t *mem_ptr = (volatile uint64_t *)((uintptr_t)IN_BASE & ~0x7);  // 8-byte aligned memory

// Write 4 consecutive 16-bit elements as a 64-bit word
for (int i = 0; i < SEQ_LEN; i += 4) {
    if (i + 3 < SEQ_LEN) {
        uint64_t combined = 0;
        for (int j = 0; j < 4; j++) {
            combined |= ((uint64_t)(seq[i + j]) << (j * 16));
        }
        // Write the 64-bit word to memory
        *mem_ptr = combined;
        mem_ptr++;  // Move to the next 8-byte memory location
    }
}


    __asm__ volatile ("fence");
    __asm__ volatile("fence":::"memory");

     // --- Software overlapper --- 
    char *bases_sw = overlapper_sw(seq, SEQ_LEN, nkmer, pos);
    if (!bases_sw) { fprintf(stderr, "overlapper_sw failed\n"); return 1; }
    const size_t nbases_sw = strlen(bases_sw); 
    printf("[CORE] SW reconstructed %d bases\n", (uint16_t)(nbases_sw) );
    
    uint8_t *nums_sw = (uint8_t*)malloc(nbases_sw);
    if (!nums_sw) { perror("malloc"); return 1; }
    bases_to_nums(bases_sw, nums_sw);


    // --- Write inputs to DRAM (16-bit) --- 
    //volatile uint16_t *inbuf = (volatile uint16_t*)(uintptr_t)IN_BASE;
    //for (int i = 0; i < SEQ_LEN; i++) inbuf[i] = (uint16_t)(seq[i] & 0xFFFF);

    //flush_dcache_range((void*)inbuf, SEQ_LEN * sizeof(uint16_t));
    //fence_rw();
    /*
    printf("[CORE] configuring the DMA.\n");
    // --- Program MMIO and start --- 
    uint32_t src = IN_BASE;
    uint32_t dst = OUT_BASE;
    reg_write32(REG_SRC_ADDR, src);
    reg_write32(REG_DST_ADDR, dst);
    reg_write32(REG_SEQ_LEN16,   (uint32_t)SEQ_LEN);

    // fire 
    reg_write32(REG_TRIGGER, 1);

    printf("[CORE] Start the overlapper.\n");
    //fence_rw();
    
    // --- Wait for DONE --- 
    while ((reg_read32(REG_STATUS_DONE) & 0x1u) == 0u) {  }
   
    // How many bytes HW says it wrote 
    uint32_t bytes_hw = reg_read32(REG_BYTES_WRITTEN);
    if (bytes_hw == 0) {
        fprintf(stderr, "ERROR: bytes_written=0\n");
        return 1;
    }

    // --- Read back HW result --- 
    volatile uint8_t *outbuf = (volatile uint8_t*)(uintptr_t)OUT_BASE;
    //flush_dcache_range((void*)outbuf, bytes_hw);
    //fence_rw();

    size_t ncheck = bytes_hw;              // HW writes bytes (0..3 each) 
    if (ncheck > nbases_sw) ncheck = nbases_sw; // compare overlap 

    uint8_t *nums_hw = (uint8_t*)malloc(ncheck);
    if (!nums_hw) { perror("malloc"); return 1; } 
    for (size_t i = 0; i < ncheck; i++) nums_hw[i] = outbuf[i];


    // --- Software overlapper --- 
    char *bases_sw = overlapper_sw(seq, SEQ_LEN, nkmer, pos);
    if (!bases_sw) { fprintf(stderr, "overlapper_sw failed\n"); return 1; }
    const size_t nbases_sw = strlen(bases_sw); 
    printf("[CORE] SW reconstructed %d bases\n", (uint16_t)(nbases_sw) );
    
    uint8_t *nums_sw = (uint8_t*)malloc(nbases_sw);
    if (!nums_sw) { perror("malloc"); return 1; }
    bases_to_nums(bases_sw, nums_sw);


    // --- Compare --- 
    size_t mism = 0;
    for (size_t i = 0; i < ncheck; i++) {
        if (nums_sw[i] != nums_hw[i]) {
            if (mism < 16) printf("Mismatch @%zu: SW=%u HW=%u\n", i, (unsigned)nums_sw[i], (unsigned)nums_hw[i]);
            mism++;
        }
    }

    if (mism == 0 && ncheck == nbases_sw) {
        printf("PASS: HW matches SW (%zu bases)\n", nbases_sw);
    } else if (mism == 0) {
        printf("PASS (partial): matched %zu bases (SW=%zu, HW=%u)\n", ncheck, nbases_sw, bytes_hw);
    } else {
        printf("FAIL: %zu mismatches out of %zu compared (SW=%zu, HW=%u)\n",
               mism, ncheck, nbases_sw, bytes_hw);
    }

    // (Optional) print a few tuples 
    for (size_t i = 0; i < (ncheck < 64 ? ncheck : 64); i++) {
        printf("(%c,%u) vs %u\n", bases_sw[i], nums_sw[i], nums_hw[i]);
    }
    

    free(nums_hw);
    free(nums_sw);
    free(bases_sw);
    free(pos);
    free(seq);
    
    return 0;
}
*/

// main.c — bare-metal RISC-V overlapper accelerator smoke test

/*#include <stdint.h>
#include <stddef.h>
#include "seq_sse.h"

// ---------- MMIO utilities ----------
static inline void reg_write8 (uintptr_t addr, uint8_t  v){ *(volatile uint8_t *) addr = v; }
static inline void reg_write32(uintptr_t addr, uint32_t v){ *(volatile uint32_t*) addr = v; }
static inline uint8_t  reg_read8 (uintptr_t addr){ return *(volatile uint8_t *) addr; }
static inline uint32_t reg_read32(uintptr_t addr){ return *(volatile uint32_t*) addr; }

// Use full memory + I/O ordering before/after MMIO (RISC-V fences)
static inline void fence_io(void){ __asm__ volatile("fence iorw, iorw" ::: "memory"); }

// ---------- Platform base addresses ----------
#define PLIC_BASE            0x0C000000UL   // not used here, kept for reference
#define MMIO_BASE            0x00004000UL   // accelerator MMIO window base

#define IN_BASE              0x80800000UL   // DRAM src buffer (accelerator reads here)
#define OUT_BASE             0x80A00000UL   // DRAM dst buffer (accelerator writes here)

// ---------- Accelerator register map (byte offsets from MMIO_BASE) ----------
#define REG_TRIGGER         (MMIO_BASE + 0x00)  // W: 1 => start
#define REG_SRC_ADDR        (MMIO_BASE + 0x04)  // W: physical src addr
#define REG_DST_ADDR        (MMIO_BASE + 0x08)  // W: physical dst addr
#define REG_SEQ_LEN16       (MMIO_BASE + 0x0C)  // W: number of 16-bit words in src
#define REG_STATUS_DONE     (MMIO_BASE + 0x10)  // R: bit0 == 1 when done
#define REG_BYTES_WRITTEN   (MMIO_BASE + 0x14)  // R: number of bytes HW wrote

// ---------- DMA buffers placed by the linker at IN_BASE / OUT_BASE ----------
// See linker.ld below. These arrays give you typed access to those regions.
// The 'volatile' ensures the compiler does not elide or reorder I/O-visible accesses.

__attribute__((section(".dma_src"), aligned(64)))
volatile uint16_t dma_src_buf[4096/2];    // 4 KiB source window (2 bytes per entry)

__attribute__((section(".dma_dst"), aligned(64)))
volatile uint8_t  dma_dst_buf[4096];      // 4 KiB destination window (bytes)


extern uint8_t __dma_src_start, __dma_dst_start;
#define IN_BASE  ((uintptr_t)&__dma_src_start)
#define OUT_BASE ((uintptr_t)&__dma_dst_start)

// ---------- Minimal console (optional) ----------
// If you have semihosting/uart printf, you can include <stdio.h> and use printf.
// To keep this standalone, we provide a tiny hex print over a polled UART if desired.
// For now, we omit printf and instead expose a weak hook you can implement elsewhere.

__attribute__((weak))
void putch(char c) { (void)c; } // replace with your UART putchar

static void puts_hex(const char *s, uint32_t v){
    static const char hexdig[] = "0123456789ABCDEF";
    while (*s) putch(*s++);
    putch('0'); putch('x');
    for (int i=7; i>=0; --i) { putch(hexdig[(v>>(i*4))&0xF]); }
    putch('\n');
}

static void puts_str(const char *s){ while (*s) putch(*s++); putch('\n'); }


int main(void)
{
    // 1) Prepare source buffer in DRAM at IN_BASE
    const uint32_t seq_size = (uint32_t)(sizeof(seq)/sizeof(seq[0])); // count of 16-bit words
    for (uint32_t i = 0; i < seq_size; i++) {
        dma_src_buf[i] = seq[i];
    }

    // Ensure the stores above reach DRAM before programming the device
    fence_io();

    // 2) Program accelerator MMIO registers
    reg_write32(REG_SRC_ADDR, IN_BASE);
    reg_write32(REG_DST_ADDR, OUT_BASE);
    reg_write32(REG_SEQ_LEN16, seq_size);

    fence_io();

    // 3) Fire
    reg_write8(REG_TRIGGER, 1);

    // 4) Busy-wait for DONE
    while ((reg_read8(REG_STATUS_DONE) & 0x1u) == 0u) {
        // spin; could add a timeout if desired
    }

    // 5) Read how many bytes HW says it wrote
    uint32_t bytes_hw = reg_read32(REG_BYTES_WRITTEN);

    // 6) Consume result bytes from OUT_BASE region
    // If your caches are coherent, this is sufficient. If not, ensure this region is mapped uncached.
    fence_io();

    // Print a short banner and the first few output bytes in hex
    puts_str("[CORE] Overlapper done; dumping first 32 bytes:");
    uint32_t to_dump = (bytes_hw < 32u) ? bytes_hw : 32u;
    for (uint32_t i = 0; i < to_dump; ++i) {
        // Add your own formatting as needed
        const char *prefix = (i % 4 == 0) ? "WORD " : "";
        if (i % 4 == 0) {
            // show 4 bytes per line as a 32-bit word view
            uint32_t w = 0;
            // little-endian packing
            if (i + 3 < bytes_hw) {
                w = ((uint32_t)dma_dst_buf[i])
                  | ((uint32_t)dma_dst_buf[i+1] << 8)
                  | ((uint32_t)dma_dst_buf[i+2] << 16)
                  | ((uint32_t)dma_dst_buf[i+3] << 24);
                puts_hex(prefix, w);
            }
        }
    }

    puts_hex("[CORE] bytes_hw = ", bytes_hw);
    puts_str("[CORE] Test complete.");
    for(;;) { // halt  }
}*/
