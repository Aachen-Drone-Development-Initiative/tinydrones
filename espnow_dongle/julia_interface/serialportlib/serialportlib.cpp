#include "serialportlib.h"

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cerrno>

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

typedef uint32_t SP_IDX;

#define MAX_SERIALPORTS (SERIALPORTLIB_MAX_SERIALPORT_IDX + 1)

#define INFO "SERIALPORTLIB INFO: "
#define WARNING "SERIALPORTLIB WARNING: "
#define ERROR "SERIALPORTLIB ERROR: "

struct Serialport {
    char* portname = nullptr;
    bool connected = false;
    int fd = -1;
};

static struct {
    Serialport serialports[MAX_SERIALPORTS];
} gsps; // Global-SerialPortlib-State

static bool valid_serialport_idx(SP_IDX sp_idx) {
    if (sp_idx <= SERIALPORTLIB_MAX_SERIALPORT_IDX) return true;
    printf(ERROR "The serialport-index '%u' is not within the valid fixed range of [0, %d]\n", sp_idx, SERIALPORTLIB_MAX_SERIALPORT_IDX);
    return false;
}

static bool valid_idx_and_connected(SP_IDX sp_idx) {
    if (!valid_serialport_idx(sp_idx))        return false;
    if (gsps.serialports[sp_idx].connected)   return true;
    printf(ERROR "The serialport at index '%u' is not connected.\n", sp_idx);
    return false;
}

// Based on: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
SPAPI bool serialport_open(uint32_t sp_idx, const char* portname, uint32_t baudrate)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    if (strlen(portname) == 0) {
        printf(ERROR "Faild to connect. No portname was specified.");
        return false;
    }

    size_t portname_len = strlen(portname) + 1;
    delete [] sp.portname;
    sp.portname = new char[portname_len];
    strncpy(sp.portname, portname, portname_len);
    
    if (sp.connected) {
        serialport_close(sp_idx);
    }

    sp.fd = open(portname, O_RDWR | O_NONBLOCK);
    if (sp.fd < 0) {
        printf(ERROR "Faild to open serialport at '%s'. errno (%d): %s\n", portname, errno, strerror(errno));
        return false;
    }

    // TERMIOS configuration

    termios port_settings;
    if(tcgetattr(sp.fd, &port_settings) != 0) {
        printf(ERROR "tcgetattr() failed to get attributes of the port at %s. errno (%d): %s\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }

    port_settings.c_cflag = 0x0;
    port_settings.c_cflag |= CS8;            // Settings size to 8 bits per byte
    port_settings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    port_settings.c_lflag = 0x0; // Disable all
    
    port_settings.c_iflag = 0x0; // Disable all

    port_settings.c_oflag = 0x0; // Disable all

    port_settings.c_cc[VTIME] = 0; // Just return all available bytes when read() and none when there are none (don't wait)
    port_settings.c_cc[VMIN] = 0;

    // TODO: test if we can actually set an arbitrary baudrate
    cfsetispeed(&port_settings, baudrate); // Baudrate (in)
    cfsetospeed(&port_settings, baudrate); // (out)

    // Apply settings. This is permanent (changes still persist when the file_handle is closed or when our process terminates)
    if (tcsetattr(sp.fd, TCSANOW, &port_settings) != 0) {
        printf(ERROR "Failed to apply configuration to port '%s'. errno (%d): '%s'\n", portname, errno, strerror(errno));
        serialport_close(sp_idx);
        return false;
    }

    printf(INFO "Successfully connected to port '%s'\n", sp.portname);

    sp.connected = true;
    return true;
}

static void set_flag(int& flags, int flag, bool value)
{
    if (value) {
        flags |= flag;
    }
    else {
        flags &= ~flag;
    }
}

SPAPI bool serialport_set_flag(SP_IDX sp_idx, Serial_Port_Flags flag, bool value)
{
    if (!valid_idx_and_connected(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    switch (flag) {
    case SERIALPORT_DTR:
    case SERIALPORT_RTS:
    {
        int ioctl_flags;
        if (ioctl(sp.fd, TIOCMGET, &ioctl_flags) < 0) {
            printf(ERROR "Failed to get ioctl flags from port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
            return 0;
        }

        switch (flag) {
        case SERIALPORT_DTR: set_flag(ioctl_flags, TIOCM_DTR, value); break;
        case SERIALPORT_RTS: set_flag(ioctl_flags, TIOCM_RTS, value); break;
        }

        if (ioctl(sp.fd, TIOCMSET, &ioctl_flags) < 0) {
            printf(ERROR "Failed to apply ioctl configuration to port '%s'. errno (%d): '%s'\n", sp.portname, errno, strerror(errno));
            serialport_close(sp_idx);
            return false;
        }

    } break;
    }
    
    return true;
}

SPAPI bool serialport_is_open(SP_IDX sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    return gsps.serialports[sp_idx].connected;
}

SPAPI bool serialport_close(SP_IDX sp_idx)
{
    if (!valid_serialport_idx(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];
    if (sp.connected) close(sp.fd);
    sp.connected = false;
    
    return true;
}

SPAPI int64_t serialport_bytes_available_to_read(uint32_t sp_idx)
{
    if (!valid_idx_and_connected(sp_idx)) return 0;
    
    Serialport& sp = gsps.serialports[sp_idx];
    
    int bytes_available;
    if (ioctl(sp.fd, FIONREAD, &bytes_available) < 0) {
        printf(ERROR "Failed to get info on the number of bytes available for reading '%s'. errno (%d): %s\n",
               sp.portname, errno, strerror(errno));
        return -1;
    }

    return bytes_available;
}

SPAPI uint8_t serialport_read_byte(SP_IDX sp_idx)
{
    if(!valid_idx_and_connected(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];

    uint8_t byte;
    ssize_t bytes_read = read(sp.fd, &byte, 1);
    if (bytes_read < 0) {
        printf(ERROR "Failed to from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return 0;
    }
    
    return byte;
}

SPAPI int64_t serialport_read_bytes(SP_IDX sp_idx, void* bytes_buffer, uint64_t buffer_size)
{
    if(!valid_idx_and_connected(sp_idx)) return false;
    
    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_read = read(sp.fd, bytes_buffer, buffer_size);
    if (bytes_read < 0) {
        printf(ERROR "Faild to read from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return -1;
    }
    
    return bytes_read;
}

SPAPI bool serialport_skip_behind_next_delimiter(SP_IDX sp_idx, void* delimiter, uint64_t delimiter_size)
{
    if (delimiter_size == 0)                return false;
    if (!valid_idx_and_connected(sp_idx))   return false;

    Serialport& sp = gsps.serialports[sp_idx];

    uint8_t* delim_cmp_buff = new uint8_t[delimiter_size];

    // fill everything but the first byte (because it gets shifted in the loop)
    ssize_t bytes_read = read(sp.fd, &delim_cmp_buff[1], delimiter_size - 1);
    if (bytes_read != (ssize_t) delimiter_size - 1) {
        return false;
    }

    while (true)
    {
        // shift all bytes in the delim_cmp_buff to the left
        for (uint64_t i = 0; i < delimiter_size - 1; ++i) {
            delim_cmp_buff[i] = delim_cmp_buff[i + 1];
        }
        
        // read the next byte into the delim_cmp_buff
        ssize_t bytes_read = read(sp.fd, &delim_cmp_buff[delimiter_size - 1], 1);

        for (uint64_t i = 0; i < delimiter_size; ++i) {
            printf("%X, ", delim_cmp_buff[i]);
        }
        printf("\n");

        if (bytes_read < 0) {
            printf(ERROR "Faild to read from the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
            return false;
        }
        else if (bytes_read != 1) {
            return false;
        }
        
        // compare the last 'delimiter_length' bytes with the delimiter
        if (memcmp(delim_cmp_buff, delimiter, delimiter_size) == 0) {
            return true;
        }
    }

    delete[] delim_cmp_buff;
}

SPAPI bool serialport_write_byte(uint32_t sp_idx, uint8_t byte)
{
    if (!valid_idx_and_connected(sp_idx)) return false;

    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_written = write(sp.fd, &byte, 1);

    if (bytes_written < 0) {
        printf(ERROR "Faild to write to the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return false;
    }

    if (bytes_written == 1) return true;
    return false;
}
    
SPAPI int64_t serialport_write_bytes(uint32_t sp_idx, void* bytes, uint64_t bytes_size)
{
    if (!valid_idx_and_connected(sp_idx)) return false;

    Serialport& sp = gsps.serialports[sp_idx];

    ssize_t bytes_written = write(sp.fd, bytes, bytes_size);

    if (bytes_written < 0) {
        printf(ERROR "Faild to write to the serialport '%s'. errno (%d): %s\n", sp.portname, errno, strerror(errno));
        return -1;
    }

    return bytes_written;
}
