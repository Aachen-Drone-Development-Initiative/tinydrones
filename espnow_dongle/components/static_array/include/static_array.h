#include <stddef.h>

#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h"

typedef struct {
  size_t element_size;
  size_t capacity;
  SemaphoreHandle_t semaphore;
  void *data;
} static_array;

// TODO: Think about where the functions should block and wait for the semaphore

/**
 * initializes static array datastructure
 *
 * @param element_count number of elements the array should hold
 * @param element_size size of one element in bytes
 *
 * @return pointer to static array, NULL on failure
 */ 
static_array *initArray(size_t element_count, size_t element_size);

/**
 * dealloccates all memory reserved by the array
 */
void destroy_array(static_array *array);

/**
 * directly accesses array element 
 *
 * @param array pointer to array
 * @param index index
 * 
 * @return pointer to element
 */
void *getElement(static_array *array, size_t index);

/**
 * copies array element into buffer
 * 
 * @param array pointer to static array
 * @param index index
 * @param buf buffer to copy into
 */
void copyElement(static_array *array, size_t index, void *buf);
