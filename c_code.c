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

static uint32_t unpack_u32_le(const uint8_t* data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[0]);
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
    
    trigger_high();

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

    trigger_low();
    return g_lfsr_state;
}

static uint32_t step_plain_v2(uint8_t num_steps, uint32_t polynomial)
{
    uint32_t mask; 
    
    trigger_high();

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

    trigger_low();
    return g_lfsr_state;
}

static uint32_t step_plain_v1(uint8_t num_steps, uint32_t polynomial)
{
    trigger_high();

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

    trigger_low();
    return g_lfsr_state;
}

static uint32_t step_shuffled(uint8_t num_steps, uint32_t polynomial)
{
    uint8_t i;
    trigger_high();
    for(i = 0; i < num_steps; i++) {
        uint32_t lsb = g_lfsr_state & 1u;
        g_lfsr_state = (g_lfsr_state >> 1) ^ (polynomial & (0u - lsb));
    }
    trigger_low();
    return g_lfsr_state;
}

static uint32_t step_masked(uint8_t num_steps, uint32_t polynomial)
{
    uint8_t i;
    trigger_high();
    for(i = 0; i < num_steps; i++) {
        uint32_t lsb_state = g_lfsr_state & 1u;
        uint32_t lsb_mask = g_lfsr_mask & 1u;
        g_lfsr_state = (g_lfsr_state >> 1) ^ (polynomial * lsb_state);
        g_lfsr_mask = (g_lfsr_mask >> 1) ^ (polynomial * lsb_mask);
    }
    trigger_low();
    return g_lfsr_state ^ g_lfsr_mask;
}

static uint32_t step_masked_shuffled(uint8_t num_steps, uint32_t polynomial)
{
    uint8_t i;
    trigger_high();
    for(i = 0; i < num_steps; i++) {
        uint32_t lsb_state = g_lfsr_state & 1u;
        uint32_t lsb_mask = g_lfsr_mask & 1u;
        g_lfsr_state = (g_lfsr_state >> 1) ^ (polynomial & (0u - lsb_state));
        g_lfsr_mask = (g_lfsr_mask >> 1) ^ (polynomial & (0u - lsb_mask));
    }
    trigger_low();
    return g_lfsr_state ^ g_lfsr_mask;
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

    if((g_mode_flags & FLAG_MASKED) && (len >= 8)) {
        g_lfsr_mask = unpack_u32_le(msg + 4);
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
            logical_state = step_masked_shuffled(num_steps, polynomial);
        } else {
            logical_state = step_masked(num_steps, polynomial);
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
    simpleserial_addcmd('s', 8, set_seed_lfsr);
    simpleserial_addcmd('c', 1, step_lfsr);
    
    while(1)
        simpleserial_get();
}
