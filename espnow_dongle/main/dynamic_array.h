
#ifndef DA_INIT_CAP
#define DA_INIT_CAP 8
#endif

// Declare dynamic arrays like this:
// 
// typedef struct {
//     Your_Type *items;
//     size_t count;
//     size_t capacity;
// } Dynamic_Array;
//

static const char* DA_TAG =  "dynamic_array"

#define da_reserve(da, expected_capacity)                               \
    do {                                                                \
        if ((expected_capacity) > (da)->capacity) {                     \
            if ((da)->capacity == 0) {                                  \
                (da)->capacity = DA_INIT_CAP;                           \
            }                                                           \
            while ((expected_capacity) > (da)->capacity) {              \
                (da)->capacity *= 2;                                    \
            }                                                           \
            (da)->items = realloc((da)->items, (da)->capacity * sizeof(*(da)->items)); \
            if ((da)->items == NULL) {                                  \
                ESP_LOGE(DA_TAG, "malloc fail");                        \
                abort();                                                \
            }                                                           \
        }                                                               \
    } while (0)

// Append an item to a dynamic array
#define da_append(da, item)                     \
    do {                                        \
        da_reserve((da), (da)->count + 1);      \
        (da)->items[(da)->count++] = (item);    \
    } while (0)

#define da_free(da) free((da).items)

// Append several items to a dynamic array
#define da_append_many(da, new_items, new_items_count)                  \
    do {                                                                \
        da_reserve((da), (da)->count + (new_items_count));              \
        memcpy((da)->items + (da)->count, (new_items), (new_items_count)*sizeof(*(da)->items)); \
        (da)->count += (new_items_count);                               \
    } while (0)

#define da_resize(da, new_size)                 \
    do {                                        \
        da_reserve((da), new_size);             \
        (da)->count = (new_size);               \
    } while (0)

#define da_last(da) (da)->items[(ASSERT((da)->count > 0), (da)->count-1)]
#define da_remove_unordered(da, i)                      \
    do {                                                \
        size_t j = (i);                                 \
        assert(j < (da)->count);                        \
        (da)->items[j] = (da)->items[--(da)->count];    \
    } while(0)
