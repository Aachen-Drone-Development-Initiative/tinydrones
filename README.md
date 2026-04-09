# Tinydrones

## tinyapi.py

tinyapi.py communicates with a controller over USB (via serialport) and provides low level functions like `write_packet()`
and high level functions like `ping_drone`

## Code Style

#### Naming

- **Functions** and **Variables** in lower_snake_case.
- **Types** in Upper_Snake_Case.
- **Constants** in UPPER_CASE.
- Please don't abbreviate too much.

#### Global Variables

- Limit access to one translation unit (one C-file), by declaring them `static`.
- prefix with `s_` for `static`, so it is clear that the variable is from the global scope.

#### Ducumentation

- Use comments to explain what is not immediately visible, but important to know (duh).
- Comment the functions in header files (the interface to your component). What are the assumptions, gotches, what does it do exactly.
- Don't mindlessly comment every parameter, instead use obvious parameter names.
- You are always welcome to explain the technical details of your implementation in comment-blocks.

#### Component Interfaces

- Should contain only the interface to your code.
- Prefix **all** functions and types with a custom short prefix for your component, for example `mpu6050_`/`MPU6050_` for the MPU6050 component.
