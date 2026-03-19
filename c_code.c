#include "hal.h"
#include "simpleserial.h"
#include <stdint.h>
#include <stdlib.h>

static uint32_t lfsr_state_low_hw  = 0; 
static uint32_t lfsr_state_high_hw = 0;

uint8_t set_seed_lfsr_low_hw(uint8_t* msg, uint8_t len)
{
    if(len >= 4) {
        lfsr_state_low_hw = ((uint32_t)msg[3] << 24) | 
                            ((uint32_t)msg[2] << 16) | 
                            ((uint32_t)msg[1] << 8)  | 
                            ((uint32_t)msg[0]);
    }
    
    // ACK
    uint8_t res = 0xAA; 
    simpleserial_put('r', 1, &res); 
    
    return 0x00;
}

uint8_t set_seed_lfsr_high_hw(uint8_t* msg, uint8_t len)
{
    if(len >= 4) {
        lfsr_state_high_hw = ((uint32_t)msg[3] << 24) | 
                             ((uint32_t)msg[2] << 16) | 
                             ((uint32_t)msg[1] << 8)  | 
                             ((uint32_t)msg[0]);
    }
    
    // ACK
    uint8_t res = 0xBB; 
    simpleserial_put('r', 1, &res); 
    
    return 0x00;
}
uint8_t step_lfsr_low_hw(uint8_t* msg, uint8_t len)
{
    // Extract number of steps from the message, default to 1 if empty
    uint8_t num_steps = (len > 0) ? msg[0] : 1;

    trigger_high(); 
    
    for(uint8_t i = 0; i < num_steps; i++) {
        uint32_t lsb = lfsr_state_low_hw & 1;
        lfsr_state_low_hw = (lfsr_state_low_hw >> 1) ^ (0xE0000200 * lsb);
    }
    
    trigger_low(); 

    uint8_t res = (uint8_t)(lfsr_state_low_hw & 0xFF);
    simpleserial_put('r', 1, &res); 
    
    return 0x00;
}

uint8_t step_lfsr_high_hw(uint8_t* msg, uint8_t len)
{
    // Extract number of steps from the message, default to 1 if empty
    uint8_t num_steps = (len > 0) ? msg[0] : 1;

    trigger_high(); 
    
    for(uint8_t i = 0; i < num_steps; i++) {
        uint32_t lsb = lfsr_state_high_hw & 1;
        lfsr_state_high_hw = (lfsr_state_high_hw >> 1) ^ (0xEDB88320 * lsb);
    }
    
    trigger_low(); 

    uint8_t res = (uint8_t)(lfsr_state_high_hw & 0xFF);
    simpleserial_put('r', 1, &res); 
    
    return 0x00;
}

int main(void)
{
    platform_init();
    init_uart();
    trigger_setup();
	simpleserial_init();
    
    simpleserial_addcmd('a', 4, set_seed_lfsr_low_hw);
    simpleserial_addcmd('b', 4, set_seed_lfsr_high_hw);
    simpleserial_addcmd('c', 1, step_lfsr_low_hw);
    simpleserial_addcmd('d', 1, step_lfsr_high_hw);
    
    while(1)
        simpleserial_get();
}
