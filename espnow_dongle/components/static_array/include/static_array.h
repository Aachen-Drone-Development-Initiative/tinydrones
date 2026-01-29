
#include <stddef.h>

#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h"

typedef struct {
  size_t element_size;
  size_t capacity;
  SemaphoreHandle_t semaphore;
  void *data;
} static_array_handle;

// TODO: Think about where the functions should block and wait for the semaphore

/**
 * initializes static array datastructure
 *
 * @param element_count number of elements the array should hold
 * @param element_size size of one element in bytes
 *
 * @return pointer to static array, NULL on failure
 */ 
static_array_handle *static_array_init_array(size_t element_count, size_t element_size);

/**
 * @brief dealloccates all memory reserved by the array
 *
 * This blocks until the semaphore on the array is released.
 * BE CAREFUL TO NOT ACCESS THIS ARRAY AFTERWARDS
 */
void static_array_destroy_array(static_array_handle *array);

/**
 * directly accesses array element
 * Blocks until semaphore is availiable
 *
 * @param array pointer to array
 * @param index index
 * 
 * @return pointer to element
 */
void *static_array_get_element(static_array_handle *array, size_t index);

/**
 * copies array element into buffer
 * Blocks until semaphore is availiable
 * 
 * @param array pointer to static array
 * @param index index
 * @param buf buffer to copy into
 */
void static_array_copy_element(static_array_handle *array, size_t index, void *buf);

/**
 * copies buf into array at given index
 * Blocks until semaphore is availiable
 * Make sure buf is the same size as the array elements
 *
 * @param array pointer to static array
 * @param index index
 * @param buf buffer to copy from
 */
void static_array_write_element(static_array_handle *array, size_t index, void *buf);
