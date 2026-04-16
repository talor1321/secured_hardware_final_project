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

static uint32_t step_unmasked_plain(uint8_t num_steps, uint32_t polynomial)
{
    uint8_t i;
    trigger_high();
    for(i = 0; i < num_steps; i++) {
        uint32_t lsb = g_lfsr_state & 1u;
        g_lfsr_state = (g_lfsr_state >> 1) ^ (polynomial * lsb);
    }
    trigger_low();
    return g_lfsr_state;
}

static uint32_t step_unmasked_shuffled(uint8_t num_steps, uint32_t polynomial)
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

static uint32_t step_masked_plain(uint8_t num_steps, uint32_t polynomial)
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
            logical_state = step_masked_plain(num_steps, polynomial);
        }
    } else {
        if(g_mode_flags & FLAG_SHUFFLED) {
            logical_state = step_unmasked_shuffled(num_steps, polynomial);
        } else {
            logical_state = step_unmasked_plain(num_steps, polynomial);
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
