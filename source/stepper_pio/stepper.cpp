module;

#include <stdio.h>
#include "hardware/pio.h"
#include "stepper.pio.h"

export module stepper;
export namespace stepper {

class command {
public:
    command(uint32_t steps, bool reverse) : _cmd(steps << 1 | (reverse ? 0 : 1)) {
        // develop assertion: max steps taken is 2147483647 (highest number that fits in 31 bits)
        assert(steps <= (UINT32_MAX >> 1));
    }
    inline operator uint32_t() const { return _cmd; }
private:
    uint32_t _cmd;
};

class stepper {
public:
    stepper(pio_hw_t *pio, uint sm) : _pio(pio), _sm(sm) {}

    // Write `steps` to TX FIFO. State machine will copy this into X.
    static inline void pio_stepper_set_steps(PIO pio, uint sm, const command& cmd) {
        pio_sm_put_blocking(pio, sm, cmd);
    }

    // Write `steps` to TX FIFO. State machine will copy this into X.
    // max steps taken is 2147483647 (highest number that fits in 31 bits)
    static inline void pio_stepper_set_steps(PIO pio, uint sm, uint32_t steps, bool reverse) {
        pio_sm_put_blocking(pio, sm, command(steps, reverse));
    }

    // call when the PIO is free. It interferes with activities
    static void pio_pwm_set_delay(PIO pio, uint sm, uint32_t delay) {
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_put(pio, sm, delay);
        pio_sm_exec(pio, sm, pio_encode_pull(false, false));
        pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
        pio_sm_set_enabled(pio, sm, true);
    }

    // Write `steps` to TX FIFO. State machine will copy this into X.
    inline void pio_stepper_set_steps(const command& cmd) {
        pio_stepper_set_steps(_pio, _sm, cmd);
    }

    // call when the PIO is free. It interferes with activities
    void pio_pwm_set_delay(uint32_t delay) {
        pio_pwm_set_delay(_pio, _sm, delay);
    }

private:
    pio_hw_t * _pio;
    uint _sm;
};


} // namespace stepper