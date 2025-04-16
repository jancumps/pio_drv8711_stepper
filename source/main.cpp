#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>
#include <iterator>
#include <span>

import drv8711;
// pre-configured registers:
import drv8711_config;
import stepper;
import pio_irq_util;

#define MICROSTEP_8
// #undef MICROSTEP_8

const uint nsleep = 14U;
const uint reset = 15U;
const uint dir = 4U;
const uint step = 5U;

// config what PIO and IRQ channel to use
const auto piostep = pio1;
// use pio irq channel 0. Can be 0 or 1
const uint pio_irq = 0;

const uint sm = 2U;

#ifdef MICROSTEP_8
const float clock_divider = 3; // works well for 8 microsteps
const uint microstep_multiplier = 8;
#else
const float clock_divider = 16; // works well for no microsteps
const uint microstep_multiplier = 1;
#endif

stepper::stepper_callback_controller motor1(piostep, sm);

class wakeup_drv8711 { // driver out of sleep as long as object in scope
public:    
     wakeup_drv8711() { gpio_put(nsleep, 1); }
     ~wakeup_drv8711() { gpio_put(nsleep, 0); }
};

static inline void cs_drive(bool high) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, high? 1 : 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static void spi_write(const uint16_t &data) {
    cs_drive(true); // drv8711 has CS active high
    spi_write16_blocking(spi_default, &data, 1);
    cs_drive(false);
}

void init_drv8711_settings() {
    // override any default settings
    #ifdef MICROSTEP_8
    drv8711::reg_ctrl.mode = 0x0003; // MODE 8 microsteps
    //drv8711::reg_torque.torque = 0x00ff;
    #endif
    drv8711::reg_torque.torque = 0x0020; // try to run cooler
    // and config over SPI
    spi_write(drv8711::reg_ctrl);
    spi_write(drv8711::reg_torque);
    spi_write(drv8711::reg_off);
    spi_write(drv8711::reg_blank);
    spi_write(drv8711::reg_decay);
    spi_write(drv8711::reg_stall);
    spi_write(drv8711::reg_drive);
    spi_write(drv8711::reg_status);
}

void init_drv8711_spi_hw() {
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000);
    spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // 16 bit registers
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_pulls(PICO_DEFAULT_SPI_RX_PIN, true, false); // drv8711 outputs are open drain
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Chip select is active-high, so we'll initialise it to a driven-low state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
}

void init_drv8711_gpio_hw() {
    // nsleep as output
    gpio_init(nsleep);
    gpio_put(nsleep, 0);
    gpio_set_dir(nsleep, GPIO_OUT);
    // reset as output
    gpio_init(reset);
    gpio_put(reset, 0);
    gpio_set_dir(reset, GPIO_OUT);
}

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
    init_drv8711_gpio_hw();
    init_drv8711_spi_hw();
    init_drv8711_settings();
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
        motor1.set_steps(c);
    }
}

void full_demo(const commands_t & cmd) {
    // wake up the stepper driver
    wakeup_drv8711 w;
    sleep_ms(1); // see datasheet

    printf("running on sm %d, with interrupt %d\n", sm, stepper_PIO_IRQ_DONE);
    int command_count = cmd.size();
    
    demo_with_delay(cmd, 4300);
    while(motor1.commands() < command_count ) {}
    printf("interrupts expected: %d, received %d\n", command_count, motor1.commands());
    motor1.reset_commands();
    sleep_ms(500); // pause for demo purpose
    
    demo_with_delay(cmd, 7000);
    while(motor1.commands() < command_count ) {}
    printf("interrupts expected: %d, received %d\n", command_count, motor1.commands());
    motor1.reset_commands();
    sleep_ms(500); // pause for demo purpose
    
    demo_with_delay(cmd, 9000);
    while(motor1.commands() < command_count ) {}
    printf("interrupts expected: %d, received %d\n", command_count, motor1.commands());
    motor1.reset_commands();
    sleep_ms(500); // pause for demo purpose

    demo_with_delay(cmd, 20000);
    while(motor1.commands() < command_count ) {}
    printf("interrupts expected: %d, received %d\n", command_count, motor1.commands());
    motor1.reset_commands();
}

int main() {

    init_everything();
    std::array<stepper::command, 7> cmd{{
        {20 * microstep_multiplier, true}, 
        {20 * microstep_multiplier, false},
        {20 * microstep_multiplier, false},
        {40 * microstep_multiplier, false},
        {25 * microstep_multiplier, true},
        {35 * microstep_multiplier, true},
        {200 * microstep_multiplier, true}}
    };

    motor1.on_complete_callback(on_complete); 

    while (true) {
        full_demo(cmd);
        sleep_ms(500); // cool off
    }
    return 0;
}