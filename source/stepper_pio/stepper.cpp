module;

#include <stdio.h>
#include "hardware/pio.h"
#include "stepper.pio.h"

#include <array>

import pio_irq_util;

export module stepper;
export namespace stepper {

/*  Stepper motor command wrapper
    lightweight 
*/
class command {
public:
    inline command(uint32_t steps, bool reverse) : cmd_(steps << 1 | (reverse ? 0 : 1)) {
        // develop assertion: max steps taken is 2147483647 (highest number that fits in 31 bits)
        assert(steps <= (UINT32_MAX >> 1));
    }
    inline operator uint32_t() const { return cmd_; }
private:
    uint32_t cmd_;
};

/*  Stepper motor wrapper for PIO state machine
    this class can be used as object,
    but if prefered, the static fuctions can be used without creating an object, 
    if developer preferes API style development.
*/
class stepper {
public:
    stepper(PIO pio, uint sm) : pio_(pio), sm_(sm) {}
    virtual ~stepper() {}

    // Write `steps` to TX FIFO. State machine will copy this into X
    static inline void set_steps(PIO pio, uint sm, const command& cmd) {
        pio_sm_put_blocking(pio, sm, cmd);
    }

    // Write `steps` to TX FIFO. State machine will copy this into X
    static inline void set_steps(PIO pio, uint sm, uint32_t steps, bool reverse) {
        pio_sm_put_blocking(pio, sm, command(steps, reverse));
    }

    // call when the state machine is free. It interferes with activities
    static void set_delay(PIO pio, uint sm, uint32_t delay) {
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_put(pio, sm, delay);
        pio_sm_exec(pio, sm, pio_encode_pull(false, false));
        pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
        pio_sm_set_enabled(pio, sm, true);
    }

    // Write `steps` to TX FIFO. State machine will copy this into X
    inline void set_steps(const command& cmd) {
        set_steps(pio_, sm_, cmd);
    }

    // call when the state machine is free. It interferes with activities
    inline void set_delay(uint32_t delay) {
        set_delay(pio_, sm_, delay);
    }

protected:
    PIO pio_;
    uint sm_;
};

/*  Stepper motor for PIO state machine, 
    with interrupt and notification support:
    It can notify the caller that a command is finished,
    and / or it can report the number of commands executed
*/
class stepper_interrupt : public stepper {
    typedef void (*notifier_t)(stepper_interrupt&);

public:
    /*
    PIO interrupts can't call object members, 
    this sub class helps translating interrupts to the relevant object
    also enforces this restriction: one stepper_interrupt object per state machine
    because this no longer a wrapper. We maintain state
    */
    class stepper_interrupt_manager {
    public:        // if an object is currently handling a pio + sm combination, it will 
        // be replaced and will no longer receive interrupts
        static bool set_stepper(PIO pio, uint sm, stepper_interrupt * stepper) {
            size_t idx = index(pio, sm);
            stepper_interrupt *old = _steppers[idx];
            _steppers[idx] = stepper;
            return old != nullptr;
        }

        static void interrupt_handler_PIO0() {
            // TODO : how do I get at the PIO?
            uint sm = pio_irq_util::sm_from_interrupt(pio0->irq, stepper_PIO_IRQ_DONE);
            stepper_interrupt *stepper =  _steppers[index(pio0, sm)];
            if (stepper != nullptr) {
                stepper -> handler();
            }
        }
    
        static void interrupt_handler_PIO1() {
            // TODO : how do I get at the PIO?
            uint sm = pio_irq_util::sm_from_interrupt(pio1->irq, stepper_PIO_IRQ_DONE);
            stepper_interrupt *stepper =  _steppers[index(pio1, sm)];
            if (stepper != nullptr) {
                stepper -> handler();
            }
        }

#if (NUM_PIOS > 2) // pico 2       
        static void interrupt_handler_PIO2() {
            // TODO : how do I get at the PIO?
            uint sm = pio_irq_util::sm_from_interrupt(pio2->irq, stepper_PIO_IRQ_DONE);
            stepper_interrupt *stepper =  _steppers.at[index(pio2, sm)];
            if (stepper != nullptr) {
                stepper -> handler();
            }
        }
#endif        

    private:
        // keep pointer to all possible objects
        static std::array<stepper_interrupt *, NUM_PIOS * 4> _steppers;
        static inline size_t index(PIO pio, uint sm) { return PIO_NUM(pio) * 4 + sm; }
    };   

public:
    stepper_interrupt(PIO pio, uint sm) : stepper(pio,sm), commands_(0U),
        callback_(nullptr) {
        stepper_interrupt_manager::set_stepper(pio_, sm_, this);
    }

    virtual ~stepper_interrupt() {
        stepper_interrupt_manager::set_stepper(pio_, sm_, nullptr);
    }

    inline uint commands() const { return commands_; }

    inline void reset_commands() { commands_ = 0U; }

    void set_interrupt(uint irq_channel, bool enable) {
        assert (irq_channel < 2); // develop check that we use 0 or 1 only
        uint irq_num = PIO0_IRQ_0 + 2 * PIO_NUM(pio_) + irq_channel;
        irq_handler_t handler = nullptr;

        if (irq_channel == 0) {
            pio_set_irq0_source_enabled(pio_, pio_irq_util::interrupt_source(pis_interrupt0, 
                pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_)), true);
        } else {
            pio_set_irq1_source_enabled(pio_, pio_irq_util::interrupt_source(pis_interrupt0, 
                pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_)), true);
        }

        switch (PIO_NUM(pio_)) {
        case 0:
            handler = stepper_interrupt_manager::interrupt_handler_PIO0;
            break;
        case 1:
            handler = stepper_interrupt_manager::interrupt_handler_PIO1;
            break;
#if (NUM_PIOS > 2) // pico 2       
        case 2:
            handler = stepper_interrupt_manager::interrupt_handler_PIO2;
            break;
#endif            
        }

        irq_set_exclusive_handler(irq_num, handler);  //Set the handler in the NVIC
        if (enable) {
            irq_set_enabled(irq_num, true);
        }
    }

    void set_callback(notifier_t callback) {
        callback_ = callback;
    }

private:
    void handler() {
        uint ir = pio_irq_util::relative_interrupt(stepper_PIO_IRQ_DONE, sm_);
        assert(pio_->irq == 1 << ir); // develop check: interrupt is from the correct state machine
        commands_ = commands_ + 1;
        pio_interrupt_clear(pio_, ir);
        if (callback_ != nullptr) {
            (callback_)( *this);
        }
    }

    volatile uint commands_; // updated by interrupt handler
    notifier_t callback_;
};

// static data member must be initialised outside of the class, or linker will not have it
std::array<stepper_interrupt *, NUM_PIOS * 4> stepper_interrupt::stepper_interrupt_manager::_steppers;

} // namespace stepper