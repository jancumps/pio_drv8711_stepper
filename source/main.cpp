#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>
#include <iterator>
#include <span>

// this code uses ti drv8711. But the stepper code is driver independent.
// the imports get the generic register definitions 
// default configuaration, and
// the Pico spi specific communication code.
// all function or class to deal with this IC have drv8711 in the name
// those are the only code parts that are stepper IC specific
// if you have an otherdriver, that's what you have to replace.
import drv8711; // registers
import drv8711_config; // register pre-configuration
import drv8711_pico; // Pico and project specific code

import stepper; // PIO stepper lib

// #define MICROSTEP_8
#undef MICROSTEP_8

const uint dir = 4U; // implies that step is gpio 5

// config what PIO and IRQ channel to use
const auto piostep = pio1;
// use pio irq channel 0. Can be 0 or 1
const uint pio_irq = 0;

const uint sm = 2U;

#ifdef MICROSTEP_8
const float clock_divider = 3; // works well for 8 microsteps
const uint microstep_x = 8;
#else
const float clock_divider = 16; // works well for no microsteps
const uint microstep_x = 1;
#endif

stepper::stepper_callback_controller motor1(piostep, sm);

// ================================================================
// PIO init

void init_pio() {
    uint offset = pio_add_program(piostep, &stepper_program);
    printf("Loaded program at %d\n", offset);

    motor1.register_pio_interrupt(pio_irq, true);

    stepper_program_init(piostep, sm, offset, dir, clock_divider);
    pio_sm_set_enabled(piostep, sm, true);
}

// ================================================================

void init_everything() {
    stdio_init_all();

    // drv8711 specific config and init
    drv8711_pico::init_drv8711_gpio_hw();
    drv8711_pico::init_drv8711_spi_hw();
    // override any default settings
    #ifdef MICROSTEP_8
    drv8711::reg_ctrl.mode = 0x0003; // MODE 8 microsteps
    drv8711::reg_torque.torque = 0x0020; // try to run cooler
    #else
    drv8711::reg_torque.torque = 0x0080; // try to run cooler
    #endif
    drv8711_pico::init_drv8711_settings();

    init_pio();
}

// stepper demo: execute a series of commands ================================

using commands_t = std::span<stepper::command>;	

void on_complete(stepper::stepper_callback_controller &stepper) {
    if (&motor1 == &stepper) {
        printf("motor1 executed command %d\n", motor1.commands());
    }
}

void demo_with_delay(const commands_t & cmd, uint32_t delay) {
    printf("delay: %d\n", delay);
    motor1.set_delay(delay);
    for(auto c : cmd) {
        motor1.take_steps(c);
    }
    while(motor1.commands() < cmd.size() ) {}
    printf("interrupts expected: %d, received %d\n", cmd.size(), motor1.commands());
    motor1.reset_commands();
    sleep_ms(500); // pause for demo purpose    
}

void full_demo(const commands_t & cmd) {
    // wake up the drv8711
    drv8711_pico::wakeup_drv8711 w;
    sleep_ms(1); // see datasheet

    printf("running on sm %d, with interrupt %d\n", sm, stepper_PIO_IRQ_DONE);
    
    demo_with_delay(cmd, 4300);
    demo_with_delay(cmd, 7000);
    demo_with_delay(cmd, 9000);
    demo_with_delay(cmd, 20000);
}

int main() {

    init_everything();
    std::array<stepper::command, 7> cmd{{
        {20 * microstep_x, true}, 
        {20 * microstep_x, false},
        {20 * microstep_x, false},
        {40 * microstep_x, false},
        {25 * microstep_x, true},
        {35 * microstep_x, true},
        {200 * microstep_x, true}}
    };

    motor1.on_complete_callback(on_complete); 

    while (true) {
        full_demo(cmd);
        sleep_ms(500); // cool off
    }
    return 0;
}