
/* Declare static arrays like this:
 * 
 * typedef struct {
 *     <Type> items[<capacity>];
 *     SemaphoreHandle_t mutex;
 * } My_Array;
 */

#define s_array_capacity(s_arr) (sizeof((s_arr)->items) / sizeof((s_arr)[0]))

#define s_array_init(s_arr)                                             \
    do {                                                                \
        assert(s_arr->items != NULL);                                   \
        s_arr->mutex = xSemaphoreCreateBinary();                        \
        if (s_arr->mutex == NULL) {                                     \
            DGL_ABORT("s_array_init: failed to create binary semaphore"); \
        }                                                               \
    } while (0);

#define s_array_safe_read(item_ptr, s_arr, idx, ticks_to_wait)          \
    do {                                                                \
        assert(idx >= 0 || idx < s_arr->capacity);                      \
        BaseType_t ret = xSemaphoreTake(a_arr->mutex, ticks_to_wait);   \
        if (ret == pdTRUE) {                                            \
            *item = s_arr->items[idx];                                  \
            xSemaphoreGive(s_arr->mutex);                               \
        }                                                               \
    } while (0);

#define s_array_safe_insert_anywhere(s_arr, item, ticks_to_wait)        \
    do {                                                                \
        BaseType_t ret = xSemaphoreTake(a_arr->mutex, ticks_to_wait);   \
        for (size_t i = 0; i < s_arr->capacity; ++i) {  

        }                                               
    } while (0);                                        

#define s_array_remove(s_arr, item)

#define s_array_contains(s_arr)
