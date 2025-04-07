# pio_drv8711_stepper
Raspberry PIO autonomous stepper motor driver

see blog: [Stepper Motor Control with Raspberry Pico PI and DRV8711 driver- Part 5: a more autonomous PIO](https://community.element14.com/products/raspberry-pi/b/blog/posts/stepper-motor-control-with-raspberry-pico-pi-and-drv8711-driver--part-5-a-more-autonomous-pio)  

- 4 motors can be controlled per PIO
- supports DRV8711 (but can be reused for drivers that have PIN and DIR)
- can handle as many commands as PIO FIFO accepts (default 8). Each command can autonomously handle 2147483647 steps, and the spin direction

Example motor instruction batch of 6 instructions:  
```
    std::array<command, 6> cmd{{
        {200, true}, 
        {200, false},
        {200, false},
        {400, false},
        {250, true},
        {350, true}}
    };

    {
        wakeup_drv8711 w; // wake up the stepper driver
        sleep_ms(1); // see drv8711 datasheet
        for(auto c : cmd) {
            printf("Steps = %d\n", c.steps);
            pio_stepper_set_steps(piostep, sm, c.steps, c.reverse);
        }

        // ...
```