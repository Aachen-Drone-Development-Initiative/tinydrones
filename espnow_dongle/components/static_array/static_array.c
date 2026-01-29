
#include "include/static_array.h"
#include "freertos/projdefs.h"
#include "include/logging.h"
#include "portmacro.h"
#include <cstddef>
#include <cstdlib>
#include <cstring>

// Array struct:
//typedef struct {
//  size_t element_size;
//  size_t capacity;
//  SemaphoreHandle_t semaphore;
//  void *data;
//} static_array;

static_array *initArray(size_t element_count, size_t element_size) {
  // allocate memory for array struct, data create semaphore

  static_array* arr_ptr = malloc(sizeof(static_array));
  if (arr_ptr == NULL) {
    TINY_ABORT("Static Array pointer memory allocation failed");
  }

  arr_ptr->capacity = element_count;
  arr_ptr->element_size = element_size;
  arr_ptr->semaphore = xSemaphoreCreateBinary();
  if (arr_ptr->semaphore == NULL) {
    TINY_ABORT("Static Array semaphore memory allocation failed");
  }

  arr_ptr->data = malloc(element_count * element_size);
  if (arr_ptr->data == NULL) {
    TINY_ABORT("Static Array data memory allocation failed");
  }
  return arr_ptr;
}

void destroy_array(static_array *array) {
  if (xSemaphoreTake(array->semaphore, portMAX_DELAY) == pdFALSE) {
    TINY_ABORT("Static Array deletion failed (couldnt take semaphore)");
  } 

  free(array->data);
  vSemaphoreDelete(array->semaphore);
  free(array);
}


void *getElement(static_array *array, size_t index) {
  if (xSemaphoreTake(array->semaphore, portMAX_DELAY) == pdFALSE) {
    TINY_ABORT("Static Array access failed (couldnt take semaphore)");
  } 
  
  void* element = array->data + index * array->element_size;

  xSemaphoreGive(array->semaphore);

  return element;
}

void copyElement(static_array *array, size_t index, void *buf) {
  if (xSemaphoreTake(array->semaphore, portMAX_DELAY) == pdFALSE) {
    TINY_ABORT("Static Array access failed (couldnt take semaphore)");
  } 
  
  memcpy(buf, array->data + index * array->element_size, array->element_size);
  
  xSemaphoreGive(array->semaphore);

  return;
}

void writeElement(static_array *array, size_t index, void *buf) {
  if (xSemaphoreTake(array->semaphore, portMAX_DELAY) == pdFALSE) {
    TINY_ABORT("Static Array access failed (couldnt take semaphore)");
  } 
  
  memcpy(array->data + index * array->element_size, buf, array->element_size);
  
  xSemaphoreGive(array->semaphore);

  return;
}
