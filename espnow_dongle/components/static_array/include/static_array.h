#pragma once

typedef struct {
    
} Array_Handle;

Array_Handle* array_create(size_t element_size, size_t element_count);

void array_destroy(Array_Handle* handle);

void* array_acquire_lock(Array_Handle* handle, size_t idx, TickType_t xTicksToWait);

void array_release_lock()
