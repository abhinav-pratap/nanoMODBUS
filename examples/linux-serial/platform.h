#include "nanomodbus.h"


// Source - https://stackoverflow.com/a
// Posted by wallyk, modified by community. See post 'Timeline' for change history
// Retrieved 2026-01-08, License - CC BY-SA 4.0

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <gpiod.h>
#include <sys/time.h>

/*
 * Delay value calculation macros.
 * c - number of characters
 * b - bits per character
 * s - bits per second
 */
#define	DV(c, b, s) (c * b * 1000000l / s)

typedef struct {
    int fd;
    struct termios tty;
    struct gpiod_line* line;
    struct gpiod_chip* chip;
} serial_conn_t;

int open_serial_conn(const char* portname, int baudrate, serial_conn_t* out_conn) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("error %d opening %s: %s", errno, portname, strerror(errno));
        return -1;
    }

    out_conn->fd = fd;

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty = out_conn->tty;

    if (tcgetattr(fd, &tty) != 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // --- Configuration ---
    
    // Baud Rate: 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8 Data bits
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

    // Parity: Even
    tty.c_cflag |= PARENB;  // Enable parity generation on output and checking for input
    tty.c_cflag &= ~PARODD; // Clear PARODD to make it Even (Setting it would make it Odd)

    // 1 Stop bit
    tty.c_cflag &= ~CSTOPB; // Clear CSTOPB for 1 stop bit (Set it for 2 stop bits)

    // Hardware flow control: Disable (Common requirement, though not strictly asked)
    tty.c_cflag &= ~CRTSCTS;

    // Local flags: Ignore modem controls, enable reading
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw mode: Disable canonical mode (line-by-line) and echo
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Output processing: Disable (Raw output)
    tty.c_oflag &= ~OPOST;

    // Apply settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    const char* chipname = "gpiochip1";  // GPIO_AON
    unsigned int line_offset = 0;    // Pin PAA.00
    struct gpiod_chip* chip;
    struct gpiod_line* line;
    int ret;

    // 1. Open the GPIO chip
    chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        perror("Open chip failed");
        return 1;
    }

    // 2. Get the specific line (pin)
    line = gpiod_chip_get_line(chip, line_offset);
    if (!line) {
        perror("Get line failed");
        gpiod_chip_close(chip);
        return 1;
    }

    // 3. Request the line as output and set initial value to LOW (0) - active receive
    // "uart_rts" is a label visible in tools like gpioinfo
    ret = gpiod_line_request_output(line, "uart_rts", 0);
    if (ret < 0) {
        perror("Request line as output failed");
        gpiod_chip_close(chip);
        return 1;
    }

    out_conn->chip = chip;
    out_conn->line = line;

    return 0;
}

// used for rts
int set_rts_receive(void* arg) {
    serial_conn_t* conn = (serial_conn_t*) arg;
    // active receive
    return gpiod_line_set_value(conn->line, 0);
}

int set_rts_transmit(void* arg) {
    serial_conn_t* conn = (serial_conn_t*) arg;
    return gpiod_line_set_value(conn->line, 1);
}

void close_serial_conn(serial_conn_t* conn) {
    close(conn->fd);
    gpiod_line_release(conn->line);
    gpiod_chip_close(conn->chip);
}

/*
 * Delay for USEC microsecs
 */
void tty_delay(int usec)
{
    struct timeval tv, ttv;
    long ts;
    gettimeofday(&tv, NULL);
    do
    {
        (void)gettimeofday(&ttv, NULL);
        ts = 1000000l * (ttv.tv_sec - tv.tv_sec) + (ttv.tv_usec - tv.tv_usec);
    } while (ts < usec);
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

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty = conn->tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(conn->fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    if (set_rts_receive(arg) < 0) {
        return -1;
    }
    if (byte_timeout_ms == 0) {
        // non-blocking read

        // update vmin and vtime in termios struct
        if (tty.c_cc[VMIN] != 0 || tty.c_cc[VTIME] != 0) {
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(conn->fd, TCSANOW, &tty) != 0) {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return 1;
            }
        }
        ssize_t num_read = read(conn->fd, buf, count);
        printf("Read %ld bytes\n", num_read);
        // print the data bytes
        for (int i = 0; i < num_read; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
        return (int32_t) num_read;
        return 0;
    }
    else if (byte_timeout_ms > 0) {
        // not supported lmao
        return -1;
    }
    else {
        // infinite timeout
        printf("requesting count=%d bytes with infinite timeout\n", count);
        if (tty.c_cc[VMIN] != count || tty.c_cc[VTIME] != 0) {
            tty.c_cc[VMIN] = count;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(conn->fd, TCSANOW, &tty) != 0) {
                printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return 1;
            }
        }
        ssize_t num_read = read(conn->fd, buf, count);
        printf("Read %ld bytes\n", num_read);
        // print the data bytes
        for (int i = 0; i < num_read; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
        return (int32_t) num_read;
    }
}


int32_t write_serial(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    (void) byte_timeout_ms;    // Unused parameter
    serial_conn_t* conn = (serial_conn_t*) arg;
    if (set_rts_transmit(arg) < 0) {
        return -1;
    }
    
    int32_t num_written = write(conn->fd, buf, count);
    tty_delay(DV((count + 1), 11, 115200)); // wait until all data is transmitted, delay calculation borrowed from mdusd
    // tcdrain(conn->fd); // waits for wayyyyyy too long after write, ~10 ms for 8 bytes at 115200 baud
    fprintf(stderr, "Wrote %d bytes\n", num_written);
    // print the data bytes
    for (int i = 0; i < num_written; i++) {
        fprintf(stderr, "%02X ", buf[i]);
    }
    fprintf(stderr, "\n");
    if (set_rts_receive(arg) < 0) {
        return -1;
    }
    return num_written;
}
