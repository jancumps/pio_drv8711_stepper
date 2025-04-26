#include "hardware/pio.h" // some constant definitions used
#include "hardware/spi.h" // some constant definitions used

#include "pico/stdlib.h"  // for demo section (printf)
#include <array>          // for demo section (commands container)
#include <iterator>       // for demo section (commands container)
#include <span>           // for demo section (commands container)

                          // this project uses ti drv8711. 
                          // But the stepper code is driver independent.
                          // the imports get the generic register definitions 
                          // default configuaration, and
                          // the Pico spi specific communication code.
                          // each class that deals with this IC has drv8711 in the name
                          // those are the only code parts that are stepper IC specific
                          // if you have another driver, that's what you have to replace.
import stepper_driver;    // wakeup class
import drv8711;           // driver classes and registers
import drv8711_pico;      // Pico port for driver
import drv8711_config;    // register pre-configuration

import stepper;           // PIO stepper lib

// #define MICROSTEP_8
#define MICROSTEP_2

const uint dir = 4U; // implies that step is gpio 5

// config what PIO and IRQ channel to use
const auto piostep = pio1;
// use pio irq channel 0. Can be 0 or 1
const uint pio_irq = 0;
// state machine
const uint sm = 2U;

#ifdef MICROSTEP_8
const float clock_divider = 3; // works well for 8 microsteps
const uint microstep_x = 8;
#elifdef MICROSTEP_2
const float clock_divider = 8; // works well for 8 microsteps
const uint microstep_x = 2;
#else
const float clock_divider = 16; // works well for no microsteps
const uint microstep_x = 1;
#endif

using motor_t = stepper::stepper_callback_controller;
motor_t motor1(piostep, sm);

// object to manage the drv8711 IC used for motor1
using driver_t = drv8711_pico::drv8711_pico;
driver_t driver1(
    spi_default, 1000 * 1000,                                      // spi
    PICO_DEFAULT_SPI_CSN_PIN, PICO_DEFAULT_SPI_RX_PIN, // spi
    PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, // spi
    14U, 15U);                                         // n_sleep, reset

void init_pio() {
    // program the pio used for the motors
    // do this only once per used pio
    motor_t::pio_program(piostep);

    // initialise and enable the motor state machine
    motor1.register_pio_interrupt(pio_irq, true);
    motor1.pio_init(dir, clock_divider);
    motor1.enable(true);
}

void init_everything() {
    stdio_init_all();

    // drv8711 specific config and init
#ifdef MICROSTEP_8
    drv8711::reg_torque.torque = 0x0020; // try to run cooler
#elifdef MICROSTEP_2
    drv8711::reg_torque.torque = 0x0060; // try to run cooler
#else
    drv8711::reg_torque.torque = 0x00ca; // try to run cooler
#endif
    driver1.init();
    driver1.microsteps(microstep_x);

    init_pio();
}

// stepper demo: execute a series of commands ================================

using commands_t = std::span<stepper::command>;	

volatile int steps_countdown = 0U;
void on_complete(const motor_t &stepper) {
    if (&motor1 == &stepper) {
        steps_countdown =  steps_countdown - 1;
        printf("motor1 executed command %d\n", motor1.commands());
    }
}

void run_with_delay(const commands_t & cmd, uint32_t delay) {
    printf("delay: %d\n", delay);
    motor1.set_delay(delay);

    steps_countdown = cmd.size();
    for(auto c : cmd) {
        motor1.take_steps(c);
    }
    while(steps_countdown) {}
    printf("interrupts expected: %d, received %d\n", cmd.size(), motor1.commands());
    motor1.reset_commands();
    sleep_ms(500); // pause for demo purpose    
}

void full_demo(const commands_t & cmd) {
    // wake up the drv8711. 
    // It goes back to low power when this object leaves the scope
    stepper_driver::wakeup w(driver1);
    sleep_ms(1); // see datasheet

    run_with_delay(cmd, 4300);
    run_with_delay(cmd, 7000);
    run_with_delay(cmd, 9000);
    run_with_delay(cmd, 20000);
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