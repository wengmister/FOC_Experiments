# FOC Experiment

A few mini projects exploring FOC control capabilities.


## ESP32_Gimbal

Starter example with ESP32-S3 and simpleFOCmini (with DRV8313), GMB2804 gimbal, and an AS5600 encoder.

- Position set point
- Velocity set point

See comment in source code for pinout connections.

Not super related but I need to rant this somewhere: somehow adafruit's ESP32-S3-FEATHER-V2 board has a really sneaky change to its NeoPixel pin assignments and I had to grid search for the right combination. Anyways, this configuration worked for me.

```cpp
#define LED_PIN 33
#define NEOPIXEL_POWER 21
```


## ESP32_Duo_Gimbal

Double the fun. Added leader-follower mode - currently hard coded to motor 1.