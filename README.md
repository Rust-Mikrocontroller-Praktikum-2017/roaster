# Setup

```
cd roaster
git clone --branch roaster git@github.com:Rust-Mikrocontroller-Praktikum-2017/roaster-stm32f7-discovery.git stm32f7-discovery
rustup override set nightly
xargo build
```

# Project Overview

- Built for the Quest M3 coffee roaster
- K-type thermocouple + **MAX6675** ADC
    - ⇨ SPI interface
    - ⇨ `temp_sensor::MAX6675`
- User Interface
    - Roast Curve: Time x Temperature
    - Current sensor reading & time
    - Configurable target = `(time,temperature)`
    - ⇨ `stm32f7_discovery::lcd`
        - Drawing primitives (`{Point,Rect,Line}`)
        - TTF font rendering (`{TextBox}`)
            - ⇨ `font-rs` and `sbt_truetype`
    - ⇨ `plot::Plot`
        - Plotting
        - Drag Zones
- Controlling
    - PID controller (`pid::PIDController`)
- `main()`
    - Every iteration:
        - Touch events (`stm32f7_discovery::touch`)
    - Every 0.5 seconds:
        - Gather metrics
        - Feed PID
        - *slow-PWM* (heating element!)

# Licenses

This project is licensed under the terms found in the LICENSE file.

## Vendored Work

- [Leak Crate(`src/leak.rs`)](https://raw.githubusercontent.com/jmesmon/leak/master/src/lib.rs)
- [Roboto Mono Bold Font](https://github.com/google/fonts/blob/master/apache/robotomono/LICENSE.txt)
