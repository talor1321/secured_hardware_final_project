#include "hal.h"
#include "simpleserial.h"
#include <stdint.h>
#define FLAG_HIGH_HW   (1u << 0)
#define FLAG_MASKED    (1u << 1)
#define FLAG_SHUFFLED  (1u << 2)

#define POLY_LOW_HW    0xE0000200u
#define POLY_HIGH_HW   0xEDB88320u

static uint32_t g_lfsr_state = 0;
static uint32_t g_lfsr_mask = 0;
static uint8_t g_mode_flags = 0;

static uint32_t g_masked_state[4] = {0, 0, 0, 0};
static uint32_t g_interleaved_poly[4] = {0, 0, 0, 0};
static uint32_t g_shuffle_rng_state = 0;
volatile uint32_t trigg_junk_var = 0;

#define NOP10 "nop; nop; nop; nop; nop; \n\t" \
              "nop; nop; nop; nop; nop; \n\t"

#define NOP50 NOP10 NOP10 NOP10 NOP10 NOP10

static void trigger_high_modified(void) {
    // junk computation
    // uint32_t polynomial = (g_mode_flags & FLAG_HIGH_HW) ? POLY_HIGH_HW : POLY_LOW_HW;
    // trigg_junk_var ^= polynomial; trigg_junk_var += 0x12345678; trigg_junk_var ^= trigg_junk_var << 3;
    // wait for a few cycles
    // __asm__ volatile(NOP50);
    // activate trigger
    trigger_high();
    // wait for a few cycles
    // __asm__ volatile(NOP50);
}

static void trigger_low_modified(void) {
    trigger_low();
}


static uint32_t unpack_u32_le(const uint8_t* data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[0]);
}

static void compute_interleaved_poly(uint32_t poly)
{
    int j, i;
    for (j = 0; j < 4; j++) {
        g_interleaved_poly[j] = 0;
        for (i = 0; i < 8; i++) {
            if ((poly >> (j * 8 + i)) & 1) {
                g_interleaved_poly[j] |= (1u << (i * 3));
            }
        }
    }
}

static void construct_interleaved_shares_register(uint32_t mask1, uint32_t mask2, uint32_t mask3)
{
    int j, i;
    for (j = 0; j < 4; j++) {
        g_masked_state[j] = 0;
        for (i = 0; i < 8; i++) {
            int bit_idx = j * 8 + i;
            uint32_t s1 = (mask1 >> bit_idx) & 1;
            uint32_t s2 = (mask2 >> bit_idx) & 1;
            uint32_t s3 = (mask3 >> bit_idx) & 1;
            int pos = i * 3;
            g_masked_state[j] |= (s1 << pos) | (s2 << (pos + 1)) | (s3 << (pos + 2));
        }
    }
}

static uint32_t step_plain_v1(uint8_t num_steps, uint32_t polynomial);
static uint32_t step_plain_v2(uint8_t num_steps, uint32_t polynomial);
static uint32_t step_plain_v3(uint8_t num_steps, uint32_t polynomial);

static uint32_t step_plain(uint8_t num_steps, uint32_t polynomial)
{
    return step_plain_v3(num_steps, polynomial);
}

static uint32_t step_plain_v3(uint8_t num_steps, uint32_t polynomial)
{
    uint32_t lsb, temp, garbage; 
    trigger_high_modified();

    __asm__ volatile (
        "cmp %[steps], #0 \n\t"               
        "beq 2f \n"                           
        "1: \n\t"                             
        
        // 1. Extract LSB and shift state right
        "and %[lsb], %[state], #1 \n\t"       
        "lsr %[state], %[state], #1 \n\t"     
        
        // 2. Unconditional XOR into a temporary register
        "eor %[temp], %[state], %[poly] \n\t" 
        
        // 3. Register routing based on LSB
        "cmp %[lsb], #1 \n\t"                 // Compare LSB to 1
        "ite eq \n\t"                         // If-Then-Else block (Condition: Equal)
        "moveq %[state], %[temp] \n\t"        // THEN (LSB=1): Copy temp to state
        "movne %[garb], %[temp] \n\t"         // ELSE (LSB=0): Copy temp to garbage
        
        // --- Loop Control ---
        "subs %[steps], %[steps], #1 \n\t"    
        "bne 1b \n"                           
        "2: \n"                               
        
        : [state] "+r" (g_lfsr_state),
          [steps] "+r" (num_steps),
          [lsb]   "=&r" (lsb),
          [temp]  "=&r" (temp),
          [garb]  "=&r" (garbage)
        : [poly]  "r"  (polynomial)
        : "cc"                                
    );

    trigger_low_modified();

    return g_lfsr_state;
}

static uint32_t step_plain_v2(uint8_t num_steps, uint32_t polynomial)
{
    uint32_t mask; 
    
    trigger_high_modified();

    __asm__ volatile (
        "cmp %[steps], #0 \n\t"               
        "beq 2f \n"                           
        "1: \n\t"                             
        
        // --- Data Processing (No 's' -> No Flag Updates) ---
        "and %[mask], %[state], #1 \n\t"      // Extract LSB
        "lsr %[state], %[state], #1 \n\t"     // Shift state right
        "rsb %[mask], %[mask], #0 \n\t"       // Mask expansion (0 - LSB)
        "and %[mask], %[mask], %[poly] \n\t"  // Mask the polynomial
        "eor %[state], %[state], %[mask] \n\t"// Apply XOR feedback
        
        // --- Loop Control (Keep 's' -> Updates Z flag) ---
        "subs %[steps], %[steps], #1 \n\t"    // Decrement counter
        "bne 1b \n"                           // Branch if Z flag != 1
        "2: \n"                               
        
        : [state] "+r" (g_lfsr_state),
          [steps] "+r" (num_steps),
          [mask]  "=&r" (mask)                
        : [poly]  "r"  (polynomial)
        : "cc"                                // We still clobber CC because of subs/cmp
    );

    trigger_low_modified();
    return g_lfsr_state;
}

static uint32_t step_plain_v1(uint8_t num_steps, uint32_t polynomial)
{
    trigger_high_modified();

    __asm__ volatile (
        "cmp %[steps], #0 \n\t"               // Check if num_steps is 0
        "beq 2f \n"                           // If 0, jump forward to label 2
        "1: \n\t"                             // Loop start
        "lsrs %[state], %[state], #1 \n\t"    // Shift state right by 1. LSB goes into Carry flag
        "bcc 3f \n\t"                         // If Carry is Clear (LSB was 0), skip the XOR
        "eors %[state], %[state], %[poly] \n" // XOR state with polynomial
        "3: \n\t"
        "subs %[steps], %[steps], #1 \n\t"    // Decrement steps
        "bne 1b \n"                           // If steps != 0, branch backward to label 1
        "2: \n"                               // Loop end
        
        // --- Output Operands ---
        : [state] "+r" (g_lfsr_state),        // '+' means read and write
          [steps] "+r" (num_steps)
          
        // --- Input Operands ---
        : [poly]  "r"  (polynomial)           // 'r' means put in a general register
        
        // --- Clobbers ---
        : "cc"                                // Tells compiler we modify condition flags
    );

    trigger_low_modified();
    return g_lfsr_state;
}

static uint32_t step_shuffled(uint8_t num_steps, uint32_t polynomial)
{
    uint8_t i;
    trigger_high_modified();
    for(i = 0; i < num_steps; i++) {
        uint32_t lsb = g_lfsr_state & 1u;
        g_lfsr_state = (g_lfsr_state >> 1) ^ (polynomial & (0u - lsb));
    }
    trigger_low_modified();
    return g_lfsr_state;
}

static uint32_t step_masked_interleaved_asm(uint8_t num_steps, uint32_t polynomial)
{
    uint32_t lsb, mask1_val, mask2_val, mask3_val, temp, poly_val;

    /* Precompute interleaved polynomial (outside the sensitive region) */
    compute_interleaved_poly(polynomial);

    trigger_high_modified();

    __asm__ volatile (
        "cmp %[steps], #0 \n\t"
        "beq 2f \n"
        "1: \n\t"

        /* Step A: Extract LSB group (3 bits from bottom of reg0) */
        "and %[lsb], %[r0], #7 \n\t"

        /* Step B: Multi-register right shift by 3 */
        /* reg0 = (reg0 >> 3) | ((reg1 & 7) << 21) */
        "lsr %[r0], %[r0], #3 \n\t"
        "and %[temp], %[r1], #7 \n\t"
        "orr %[r0], %[r0], %[temp], lsl #21 \n\t"

        /* reg1 = (reg1 >> 3) | ((reg2 & 7) << 21) */
        "lsr %[r1], %[r1], #3 \n\t"
        "and %[temp], %[r2], #7 \n\t"
        "orr %[r1], %[r1], %[temp], lsl #21 \n\t"

        /* reg2 = (reg2 >> 3) | ((reg3 & 7) << 21) */
        "lsr %[r2], %[r2], #3 \n\t"
        "and %[temp], %[r3], #7 \n\t"
        "orr %[r2], %[r2], %[temp], lsl #21 \n\t"

        /* reg3 = reg3 >> 3 (Padded to match the exact ALU signature of reg0-2) */
        "lsr %[r3], %[r3], #3 \n\t"
        "and %[temp], %[r3], #0 \n\t"               /* Forces temp to 0 using the identical 'and' instruction */
        "orr %[r3], %[r3], %[temp], lsl #21 \n\t"   /* Harmlessly ORs the 0, mirroring the power trace */

        /* Step C: Expand each share LSB to a full 32-bit mask */
        /* Uniform UBFX extraction across all shares */
        "ubfx %[mask1], %[lsb], #0, #1 \n\t"
        "rsb %[mask1], %[mask1], #0 \n\t"

        "ubfx %[mask2], %[lsb], #1, #1 \n\t"
        "rsb %[mask2], %[mask2], #0 \n\t"

        "ubfx %[mask3], %[lsb], #2, #1 \n\t"
        "rsb %[mask3], %[mask3], #0 \n\t"

        /* Step D: Polynomial feedback for each register */
        /* feedback = (poly & mask1) | ((poly<<1) & mask2) | ((poly<<2) & mask3) */

        /* ---- reg0 feedback ---- */
        "ldr %[poly], [%[pptr], #0] \n\t"
        "and %[lsb], %[poly], %[mask1] \n\t"
        "and %[temp], %[mask2], %[poly], lsl #1 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "and %[temp], %[mask3], %[poly], lsl #2 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "eor %[r0], %[r0], %[lsb] \n\t"

        /* ---- reg1 feedback ---- */
        "ldr %[poly], [%[pptr], #4] \n\t"
        "and %[lsb], %[poly], %[mask1] \n\t"
        "and %[temp], %[mask2], %[poly], lsl #1 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "and %[temp], %[mask3], %[poly], lsl #2 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "eor %[r1], %[r1], %[lsb] \n\t"

        /* ---- reg2 feedback ---- */
        "ldr %[poly], [%[pptr], #8] \n\t"
        "and %[lsb], %[poly], %[mask1] \n\t"
        "and %[temp], %[mask2], %[poly], lsl #1 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "and %[temp], %[mask3], %[poly], lsl #2 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "eor %[r2], %[r2], %[lsb] \n\t"

        /* ---- reg3 feedback ---- */
        "ldr %[poly], [%[pptr], #12] \n\t"
        "and %[lsb], %[poly], %[mask1] \n\t"
        "and %[temp], %[mask2], %[poly], lsl #1 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "and %[temp], %[mask3], %[poly], lsl #2 \n\t"
        "orr %[lsb], %[lsb], %[temp] \n\t"
        "eor %[r3], %[r3], %[lsb] \n\t"

        /* Step E: Loop control */
        "subs %[steps], %[steps], #1 \n\t"
        "bne 1b \n"
        "2: \n"

        : [r0]    "+r" (g_masked_state[0]),
          [r1]    "+r" (g_masked_state[1]),
          [r2]    "+r" (g_masked_state[2]),
          [r3]    "+r" (g_masked_state[3]),
          [steps] "+r" (num_steps),
          [lsb]   "=&r" (lsb),
          [mask1] "=&r" (mask1_val),
          [mask2] "=&r" (mask2_val),
          [mask3] "=&r" (mask3_val),
          [temp]  "=&r" (temp),
          [poly]  "=&r" (poly_val)
        : [pptr]  "r"  (g_interleaved_poly)
        
        : "cc", "memory"
    );

    trigger_low_modified();

    return 0x00;
}

#define FB_R(n) \
    "ldr %[poly], [%[pptr], #" #n "*4] \n\t" \
    "and %[lsb], %[poly], %[t1_m1] \n\t" \
    "and %[temp], %[t2_m2], %[poly], lsl #1 \n\t" \
    "orr %[lsb], %[lsb], %[temp] \n\t" \
    "and %[temp], %[t3_m3], %[poly], lsl #2 \n\t" \
    "orr %[lsb], %[lsb], %[temp] \n\t" \
    "eor %[r" #n "], %[r" #n "], %[lsb] \n\t"

#define FB_R(n)

static uint32_t step_masked_interleaved_shuffled_asm(uint8_t num_steps, uint32_t polynomial)
{
    uint32_t lsb, t1_m1, t2_m2, t3_m3, temp, poly_val;
    uint32_t rng = g_shuffle_rng_state;

    compute_interleaved_poly(polynomial);
    trigger_high_modified();

    __asm__ volatile (
        "cmp %[steps], #0 \n\t"
        "beq 99f \n"
        "1: \n\t"

        /* Extract LSB group */
        "and %[lsb], %[r0], #7 \n\t"

        /* Extract boundary bits */
        "and %[t1_m1], %[r1], #7 \n\t"
        "and %[t2_m2], %[r2], #7 \n\t"
        "and %[t3_m3], %[r3], #7 \n\t"

        /* Advance RNG (xorshift32) */
        "eor %[rng], %[rng], %[rng], lsl #13 \n\t"
        "eor %[rng], %[rng], %[rng], lsr #17 \n\t"
        "eor %[rng], %[rng], %[rng], lsl #5 \n\t"

        /* --- BRANCHLESS SHIFTS SELECTOR --- */
        "and %[temp], %[rng], #3 \n\t"           /* Mask bits 0,1 */
        "lsl %[temp], %[temp], #2 \n\t"          /* temp = offset (0, 4, 8, or 12 bytes) */
        "adr %[poly], 88f \n\t"                  /* Get jump table base address */
        "ldr %[poly], [%[poly], %[temp]] \n\t"   /* Load destination address */
        "orr %[poly], %[poly], #1 \n\t"          /* Explicitly set Thumb state bit */
        "bx %[poly] \n\t"                        /* Branch and Exchange */

        ".balign 4 \n\t"                         /* Ensure table is strictly word-aligned */
        "88: \n\t"
        ".word 10f \n\t"
        ".word 11f \n\t"
        ".word 12f \n\t"
        ".word 13f \n\t"

        "10: \n\t" 
        "mov %[temp], #0 \n\t" /* Zero out temp for R3's dummy ORR */
        "lsr %[r0], %[r0], #3 \n\t" "orr %[r0], %[r0], %[t1_m1], lsl #21 \n\t"
        "lsr %[r1], %[r1], #3 \n\t" "orr %[r1], %[r1], %[t2_m2], lsl #21 \n\t"
        "lsr %[r2], %[r2], #3 \n\t" "orr %[r2], %[r2], %[t3_m3], lsl #21 \n\t"
        "lsr %[r3], %[r3], #3 \n\t" "orr %[r3], %[r3], %[temp], lsl #21 \n\t"
        "b 20f \n\t"

        "11: \n\t" 
        "mov %[temp], #0 \n\t"
        "lsr %[r1], %[r1], #3 \n\t" "orr %[r1], %[r1], %[t2_m2], lsl #21 \n\t"
        "lsr %[r2], %[r2], #3 \n\t" "orr %[r2], %[r2], %[t3_m3], lsl #21 \n\t"
        "lsr %[r3], %[r3], #3 \n\t" "orr %[r3], %[r3], %[temp], lsl #21 \n\t"
        "lsr %[r0], %[r0], #3 \n\t" "orr %[r0], %[r0], %[t1_m1], lsl #21 \n\t"
        "b 20f \n\t"

        "12: \n\t" 
        "mov %[temp], #0 \n\t"
        "lsr %[r2], %[r2], #3 \n\t" "orr %[r2], %[r2], %[t3_m3], lsl #21 \n\t"
        "lsr %[r3], %[r3], #3 \n\t" "orr %[r3], %[r3], %[temp], lsl #21 \n\t"
        "lsr %[r0], %[r0], #3 \n\t" "orr %[r0], %[r0], %[t1_m1], lsl #21 \n\t"
        "lsr %[r1], %[r1], #3 \n\t" "orr %[r1], %[r1], %[t2_m2], lsl #21 \n\t"
        "b 20f \n\t"

        "13: \n\t" 
        "mov %[temp], #0 \n\t"
        "lsr %[r3], %[r3], #3 \n\t" "orr %[r3], %[r3], %[temp], lsl #21 \n\t"
        "lsr %[r0], %[r0], #3 \n\t" "orr %[r0], %[r0], %[t1_m1], lsl #21 \n\t"
        "lsr %[r1], %[r1], #3 \n\t" "orr %[r1], %[r1], %[t2_m2], lsl #21 \n\t"
        "lsr %[r2], %[r2], #3 \n\t" "orr %[r2], %[r2], %[t3_m3], lsl #21 \n\t"
        "b 20f \n\t"

        "20: \n\t"
        /* --- Shifts Done --- */

        /* Mask expansion (Uniform UBFX extraction) */
        "ubfx %[t1_m1], %[lsb], #0, #1 \n\t"
        "rsb %[t1_m1], %[t1_m1], #0 \n\t"
        "ubfx %[t2_m2], %[lsb], #1, #1 \n\t"
        "rsb %[t2_m2], %[t2_m2], #0 \n\t"
        "ubfx %[t3_m3], %[lsb], #2, #1 \n\t"
        "rsb %[t3_m3], %[t3_m3], #0 \n\t"

        /* --- BRANCHLESS FEEDBACK SELECTOR --- */
        "ubfx %[temp], %[rng], #2, #2 \n\t"      /* Mask bits 2,3 */
        "lsl %[temp], %[temp], #2 \n\t"
        "adr %[poly], 99f \n\t"
        "ldr %[poly], [%[poly], %[temp]] \n\t"   /* Load destination address */
        "orr %[poly], %[poly], #1 \n\t"          /* Explicitly set Thumb state bit */
        "bx %[poly] \n\t"                        /* Branch and Exchange */

        ".balign 4 \n\t"
        "99: \n\t"
        ".word 30f \n\t"
        ".word 31f \n\t"
        ".word 32f \n\t"
        ".word 33f \n\t"

        "30: \n\t"
        FB_R(0) FB_R(1) FB_R(2) FB_R(3)
        "b 40f \n\t"

        "31: \n\t"
        FB_R(1) FB_R(2) FB_R(3) FB_R(0)
        "b 40f \n\t"

        "32: \n\t"
        FB_R(2) FB_R(3) FB_R(0) FB_R(1)
        "b 40f \n\t"

        "33: \n\t"
        FB_R(3) FB_R(0) FB_R(1) FB_R(2)
        "b 40f \n\t"

        "40: \n\t"
        "subs %[steps], %[steps], #1 \n\t"
        "bne 1b \n"
        "99: \n"

        : [r0]    "+r" (g_masked_state[0]),
          [r1]    "+r" (g_masked_state[1]),
          [r2]    "+r" (g_masked_state[2]),
          [r3]    "+r" (g_masked_state[3]),
          [steps] "+r" (num_steps),
          [rng]   "+r" (rng),
          [lsb]   "=&r" (lsb),
          [t1_m1] "=&r" (t1_m1),
          [t2_m2] "=&r" (t2_m2),
          [t3_m3] "=&r" (t3_m3),
          [temp]  "=&r" (temp),
          [poly]  "=&r" (poly_val)
        : [pptr]  "r"  (g_interleaved_poly)
        : "cc", "memory"
    );

    trigger_low_modified();

    g_shuffle_rng_state = rng;
    return 0x00;
}

uint8_t set_mode_flags(uint8_t* msg, uint8_t len)
{
    if(len >= 1) {
        g_mode_flags = msg[0] & (FLAG_HIGH_HW | FLAG_MASKED | FLAG_SHUFFLED);
    }

    simpleserial_put('r', 1, &g_mode_flags);
    return 0x00;
}

uint8_t set_seed_lfsr(uint8_t* msg, uint8_t len)
{
    if(len >= 4) {
        g_lfsr_state = unpack_u32_le(msg);
    }

    if((g_mode_flags & FLAG_MASKED) && (len >= 12)) {
        uint32_t m1 = unpack_u32_le(msg + 4);
        uint32_t m2 = unpack_u32_le(msg + 8);
        uint32_t m3 = unpack_u32_le(msg + 12);
        construct_interleaved_shares_register(m1, m2, m3);
        
        if((g_mode_flags & FLAG_SHUFFLED) && (len >= 16)) {
            g_shuffle_rng_state = unpack_u32_le(msg + 12);
        } else {
            g_shuffle_rng_state = 12345;
        }
    } else if(!(g_mode_flags & FLAG_MASKED)) {
        g_lfsr_mask = 0;
    }

    {
        uint8_t ack = 0xA0 | (g_mode_flags & 0x07);
        simpleserial_put('r', 1, &ack);
    }

    return 0x00;
}

uint8_t step_lfsr(uint8_t* msg, uint8_t len)
{
    uint8_t num_steps = (len > 0) ? msg[0] : 1;
    uint32_t polynomial = (g_mode_flags & FLAG_HIGH_HW) ? POLY_HIGH_HW : POLY_LOW_HW;
    uint32_t logical_state;

    if(g_mode_flags & FLAG_MASKED) {
        if(g_mode_flags & FLAG_SHUFFLED) {
            logical_state = step_masked_interleaved_shuffled_asm(num_steps, polynomial);
        } else {
            logical_state = step_masked_interleaved_asm(num_steps, polynomial);
        }
    } else {
        if(g_mode_flags & FLAG_SHUFFLED) {
            logical_state = step_shuffled(num_steps, polynomial);
        } else {
            logical_state = step_plain(num_steps, polynomial);
        }
    }

    {
        uint8_t res = (uint8_t)(logical_state & 0xFFu);
        simpleserial_put('r', 1, &res);
    }

    return 0x00;
}

int main(void)
{
    platform_init();
    init_uart();
    trigger_setup();
	simpleserial_init();

    // 'f': set flags, 's': set seed(+optional mask), 'c': step
    simpleserial_addcmd('f', 1, set_mode_flags);
    simpleserial_addcmd('s', 16, set_seed_lfsr);
    simpleserial_addcmd('c', 1, step_lfsr);
    
    while(1)
        simpleserial_get();
}

