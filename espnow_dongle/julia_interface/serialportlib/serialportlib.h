#ifndef SERILAPORTLIB_H
#define SERILAPORTLIB_H

#include <stdint.h>

#define SERIALPORTLIB_MAX_SERIALPORT_IDX 1000

#define LIBTYPE_SHARED 1

#ifdef LIBTYPE_SHARED
    #define SPAPI __attribute__((visibility("default")))
#endif

enum {
    SERIALPORT_DTR = 0,
    SERIALPORT_RTS = 1,
    // TODO: add more
};
typedef int Serial_Port_Flags;

#ifdef __cplusplus
extern "C" {
#endif

SPAPI bool serialport_open(uint32_t sp_idx, const char* portname, uint32_t baudrate);
SPAPI bool serialport_close(uint32_t sp_idx);
SPAPI bool serialport_is_open(uint32_t sp_idx);

SPAPI bool serialport_set_flag(uint32_t sp_idx, Serial_Port_Flags flag, bool value);
    
SPAPI int64_t serialport_bytes_available_to_read(uint32_t sp_idx);
SPAPI uint8_t serialport_read_byte(uint32_t sp_idx);
// Try to read as many as 'out_size' bytes. Returns how many bytes were actually read, or -1 on error.
SPAPI int64_t serialport_read_bytes(uint32_t sp_idx, void* out_buffer, uint64_t out_size);

// Comsumes bytes from the serialport until it finds the delimiter. The next bytes read are the ones just after the delimiter.
// You can safely read your data when 'true' is returned. 'false' often means that all available bytes were exhausted.
// This is useful for when your data-stream becomes misaligned because a byte got missing etc.
SPAPI bool serialport_skip_behind_next_delimiter(uint32_t sp_idx, void* delimiter, uint64_t delimiter_size);

SPAPI bool serialport_write_byte(uint32_t sp_idx, uint8_t byte);
SPAPI int64_t serialport_write_bytes(uint32_t sp_idx, void* bytes, uint64_t bytes_size);

#ifdef __cplusplus
}
#endif
    
#endif // SERILAPORTLIB_H
