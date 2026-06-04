# FusionMath

`FusionMath.h` is a small header-only math library used for vector,
quaternion, matrix, and Euler angle operations.

Source: https://github.com/xioTechnologies/Fusion  
License: MIT

## `FUSION_USE_NORMAL_SQRT`

Enabled in CMake:

```cmake
target_compile_definitions(${COMPONENT_LIB} PUBLIC FUSION_USE_NORMAL_SQRT)
```

Or in fusion_math.h

    #define FUSION_USE_NORMAL_SQRT

If defined:

- Uses normal `sqrtf()`.
- Better for ESP32 because the hardware FPU handles square root efficiently.

If not defined:

- Uses Fusion's fast inverse square-root approximation.
- On modern ESP32 chips this can be less accurate and may be slower.

## Notes

- All functions are defined as `static inline` in `FusionMath.h`.
- Euler angles are in degrees.
- Matrices are stored in row-major order.
