/*
 * This example application connects via rtu to a modbus server at the specified serial port and buadrate, and sends some
 * modbus requests to it.
 *
 * Since the platform for this example is linux, the platform arg is used to pass (to the linux file descriptor
 * read/write functions) a pointer to the file descriptor of our serial connection
 *
 */

#include <stdio.h>

#include "nanomodbus.h"
#include "platform.h"

int main() {

    // Set up the serial connection
    serial_conn_t conn;
    rts_config_t rts = {
            .rts_gpiochip = "gpiochip1",    // GPIO_AON
            .rts_gpio_line_offset = 0,      // PAA.00
            .rts_active_receive = true,     // active receive (0 on receive)
    };
    int ret = open_serial_conn("/dev/ttyTHS1", B115200, &rts, &conn);
    if (ret != 0) {
        fprintf(stderr, "Error connecting to server\n");
        return 1;
    }

    nmbs_platform_conf platform_conf;
    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = read_serial;
    platform_conf.write = write_serial;
    platform_conf.arg = &conn;    // Passing our serial connection handle to the read/write functions

    // Create the modbus client
    nmbs_t nmbs;
    nmbs_error err = nmbs_client_create(&nmbs, &platform_conf);
    if (err != NMBS_ERROR_NONE) {
        fprintf(stderr, "Error creating modbus client\n");
        if (!nmbs_error_is_exception(err))
            return 1;
    }

    nmbs_set_destination_rtu_address(&nmbs, 2);    // Set destination modbus rtu address

    uint16_t w_regs[1] = {
            2307,
    };
    err = nmbs_write_multiple_registers(&nmbs, 110, 1, w_regs);
    if (err != NMBS_ERROR_NONE) {
        fprintf(stderr, "Error writing register at address 110 - %s\n", nmbs_strerror(err));
        if (!nmbs_error_is_exception(err))
            return 1;
    }

    sleep(1);    // wait a bit for the device to process the command

    // Read 1 holding registers from address 110 (CV target)
    uint16_t r_regs[1];
    err = nmbs_read_holding_registers(&nmbs, 110, 1, r_regs);
    if (err != NMBS_ERROR_NONE) {
        fprintf(stderr, "Error reading 1 holding registers at address 110 - %s\n", nmbs_strerror(err));
        if (!nmbs_error_is_exception(err))
            return 1;
    }
    else {
        printf("Register at address 110: %d\n", r_regs[0]);
    }

    // Close the serial connection
    close_serial_conn(&conn);

    // No need to destroy the nmbs instance, bye bye
    return 0;
}
