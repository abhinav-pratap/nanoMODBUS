#include "nanomodbus.h"


// Source - https://stackoverflow.com/a
// Posted by wallyk, modified by community. See post 'Timeline' for change history
// Retrieved 2026-01-08, License - CC BY-SA 4.0

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef struct {
    int fd;
    struct termios tty;
} serial_conn_t;

int open_serial_conn(const char* portname, int baudrate, serial_conn_t* out_conn) {
    int fd = open(portname, O_RDWR);
    if (fd < 0) {
        error_message("error %d opening %s: %s", errno, portname, strerror(errno));
        return -1;
    }

    out_conn->fd = fd;

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty = out_conn->tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag |= PARENB;            // Set parity bit
    tty.c_cflag &= ~PARODD;           // Even parity
    tty.c_cflag &= ~CSTOPB;           // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;            // Clear all bits that set the data size
    tty.c_cflag |= CS8;               // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;          // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;    // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                      // Disable echo
    tty.c_lflag &= ~ECHOE;                     // Disable erasure
    tty.c_lflag &= ~ECHONL;                    // Disable new-line echo
    tty.c_lflag &= ~ISIG;                      // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);    // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL);    // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;    // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;    // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    return 0;
}

/**
 * nanoMODBUS platform configuration struct.
 * Passed to nmbs_server_create() and nmbs_client_create().
 *
 * read() and write() are the platform-specific methods that read/write data to/from a serial port or a TCP connection.
 *
 * Both methods should block until either:
 * - `count` bytes of data are read/written
 * - the byte timeout, with `byte_timeout_ms >= 0`, expires
 *
 * A value `< 0` for `byte_timeout_ms` means infinite timeout.
 * With a value `== 0` for `byte_timeout_ms`, the method should read/write once in a non-blocking fashion and return immediately.
 *
 *
 * Their return value should be the number of bytes actually read/written, or `< 0` in case of error.
 * A return value between `0` and `count - 1` will be treated as if a timeout occurred on the transport side. All other
 * values will be treated as transport errors.
 *
 * Additionally, an optional crc_calc() function can be defined to override the default nanoMODBUS CRC calculation function.
 *
 * These methods accept a pointer to arbitrary user-data, which is the arg member of this struct.
 * After the creation of an instance it can be changed with nmbs_set_platform_arg().
 */
int32_t read_serial(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    serial_conn_t* conn = (serial_conn_t*) arg;
    if (byte_timeout_ms == 0) {
        // non-blocking read

        // update vmin and vtime in termios struct
        if (conn->tty.c_cc[VMIN] != 0 || conn->tty.c_cc[VTIME] != 0) {
            conn->tty.c_cc[VMIN] = 0;
            conn->tty.c_cc[VTIME] = 0;
            if (tcsetattr(conn->fd, TCSANOW, &conn->tty) != 0) {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return 1;
            }
        }
        ssize_t num_read = read(conn->fd, buf, count);
        return (int32_t) num_read;
    }
    else if (byte_timeout_ms > 0) {
        // not supported lmao
        return -1;
    }
    else {
        // infinite timeout
        if (conn->tty.c_cc[VMIN] != count || conn->tty.c_cc[VTIME] != 0) {
            conn->tty.c_cc[VMIN] = count;
            conn->tty.c_cc[VTIME] = 0;
            if (tcsetattr(conn->fd, TCSANOW, &conn->tty) != 0) {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return 1;
            }
        }
        ssize_t num_read = read(conn->fd, buf, count);
        return (int32_t) num_read;
    }
}


int32_t write_serial(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    serial_conn_t* conn = (serial_conn_t*) arg;
    return (int32_t) write(conn->fd, buf, count);
}
