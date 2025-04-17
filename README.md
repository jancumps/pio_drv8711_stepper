# pio_drv8711_stepper
Raspberry PIO autonomous stepper motor driver

see blog: [Stepper Motor Control with Raspberry Pico PI and DRV8711 driver- Part 5: a more autonomous PIO](https://community.element14.com/products/raspberry-pi/b/blog/posts/stepper-motor-control-with-raspberry-pico-pi-and-drv8711-driver--part-5-a-more-autonomous-pio)  

- 4 motors can be controlled per PIO
- supports DRV8711 (but can be ported to any driver that supports PIN and DIR operation)
- can handle as many commands as PIO FIFO accepts without waiting (default 8). Each command can autonomously handle 2147483647 steps, and the spin direction
- can notify the calling program when an instruction is finished

Example motor instruction batch of 6 instructions:  
```
stepper::stepper_callback_controller motor1(piostep, sm);

void on_complete(stepper::stepper_callback_controller &stepper) {
    if (&motor1 == &stepper) {
        printf("motor1 executed command %d\n", motor1.commands());
    }
}

int main() {

    motor1.on_complete_callback(on_complete);

    std::array<command, 6> cmd{{
        {200, true}, 
        {200, false},
        {200, false},
        {400, false},
        {250, true},
        {350, true}}
    };

    motor1.set_delay(delay);
    for(auto c : cmd) {
        motor1.take_steps(c);
    }

        // ...
```