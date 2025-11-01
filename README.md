# Tinydrones Project

## Code style guide 

### General rules

1. Keep everything minimal/simple.
2. Think about how others will understand and use your code (be empathic).
3. Choose explanatory names, but don't make them too long, it's okay to assume some knowledge.
4. Wait a little bit before introducing a new abstraction and implement it when it survived in your head for some time.
5. Think hard about the structure/design of your code, go for a walk when you are stuck or unsatisfied.
6. Commit to high standards.

### Function declarations

1. Opening bracket on the next line (please)
2. Function name in lower-snake-case
3. If the function is only used in the translation unit (C-file) it's defined in, then declare it `static`. This helps the linker and informs the programmer.

```c
static int foo_bar()
{
    // function body
}
```

### Variable declarations

1. All variable names in lower-snake-case

#### Global variables

1. Try to limit access to one translation unit (one C-file), by declaring them `static`.
2. prefix with `s_` for `static`, so it is clear that the variable is from the global scope.

```c
static Component_State s_state = {0};
```

### Type declarations

1. Use `typedef`
2. Capital-snake-case ie. `Packet_Info`

```c
typedef struct {
    // member types
} Packet_Info;
```

### Constants/#Defines

1. Upper-snake-case ie. `#define MAX_ESP_DEVICES (128)`.
2. Always encase the #define value with parenthese ie. `#define FOO (42)`. Typically this is only necessary when defining an expression, but I find it more readable, so let's just do it always :)
3. Clearly seperate implementation specific constants and 'user-changable' constants.
4. Put them at the top of the file, behind includes (especially the user-changable ones).

### Ducumentation/Commenting

1. Use comments to explain what is not immediately visible, but important to know.
2. Comment the functions in header files (the interface to your component). What are the assumptions, gotches, what does it do exactly.
3. Don't comment each parameter out of principle, use descriptive type-names, variable-names instead.

```c
// Reads the output registers on the mpu6050 returns the values converted to SI units.
MPU6050_Data mpu6050_get_data(MPU6050_Handle mpu);
```

4. You are always welcome to explain the technicalities of your implementation, in comment-blocks where appropriate.

### Header files / component interface

1. Should contain only the interface to your code, ie. the function declarations and everything that is required to make these declarations compile (types, #includes, etc.). Everything else must be in the associated C-file(s).
2. Prefix **all** functions with a custom short prefix for your component, for example `mpu6050_` for the MPU6050 component.
3. Prefix **all** types with a custom short prefix for your component, for example `MPU6050_` for the MPU6050 component.
