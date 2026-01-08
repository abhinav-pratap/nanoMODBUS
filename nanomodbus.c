/*
    nanoMODBUS - A compact MODBUS RTU/TCP C library for microcontrollers

    MIT License

    Copyright (c) 2024 Valerio De Benedetto (@debevv)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "nanomodbus.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NMBS_UNUSED_PARAM(x) ((x) = (x))

#ifdef NMBS_DEBUG
#include <stdio.h>
#define NMBS_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define NMBS_DEBUG_PRINT(...) (void) (0)
#endif


static uint8_t get_1(nmbs_t* nmbs) {
    uint8_t result = nmbs->msg.buf[nmbs->msg.buf_idx];
    nmbs->msg.buf_idx++;
    return result;
}


static void put_1(nmbs_t* nmbs, uint8_t data) {
    nmbs->msg.buf[nmbs->msg.buf_idx] = data;
    nmbs->msg.buf_idx++;
}


static void discard_1(nmbs_t* nmbs) {
    nmbs->msg.buf_idx++;
}


static uint16_t get_2(nmbs_t* nmbs) {
    const uint16_t result =
            ((uint16_t) nmbs->msg.buf[nmbs->msg.buf_idx]) << 8 | (uint16_t) nmbs->msg.buf[nmbs->msg.buf_idx + 1];
    nmbs->msg.buf_idx += 2;
    return result;
}


static void put_2(nmbs_t* nmbs, uint16_t data) {
    nmbs->msg.buf[nmbs->msg.buf_idx] = (uint8_t) ((data >> 8) & 0xFFU);
    nmbs->msg.buf[nmbs->msg.buf_idx + 1] = (uint8_t) data;
    nmbs->msg.buf_idx += 2;
}


static uint8_t* get_n(nmbs_t* nmbs, uint16_t n) {
    uint8_t* msg_buf_ptr = nmbs->msg.buf + nmbs->msg.buf_idx;
    nmbs->msg.buf_idx += n;
    return msg_buf_ptr;
}


static void put_regs(nmbs_t* nmbs, const uint16_t* data, uint16_t n) {
    uint16_t* msg_buf_ptr = (uint16_t*) (nmbs->msg.buf + nmbs->msg.buf_idx);
    nmbs->msg.buf_idx += n * 2;
    while (n--) {
        msg_buf_ptr[n] = (data[n] << 8) | ((data[n] >> 8) & 0xFF);
    }
}


static void swap_regs(uint16_t* data, uint16_t n) {
    while (n--) {
        data[n] = (data[n] << 8) | ((data[n] >> 8) & 0xFF);
    }
}


static nmbs_error recv(nmbs_t* nmbs, uint16_t count) {
    if (nmbs->msg.complete) {
        return NMBS_ERROR_NONE;
    }

    const int32_t ret =
            nmbs->platform.read(nmbs->msg.buf + nmbs->msg.buf_idx, count, nmbs->byte_timeout_ms, nmbs->platform.arg);

    if (ret == count)
        return NMBS_ERROR_NONE;

    if (ret < count) {
        if (ret < 0)
            return NMBS_ERROR_TRANSPORT;

        return NMBS_ERROR_TIMEOUT;
    }

    return NMBS_ERROR_TRANSPORT;
}


static nmbs_error send(const nmbs_t* nmbs, uint16_t count) {
    const int32_t ret = nmbs->platform.write(nmbs->msg.buf, count, nmbs->byte_timeout_ms, nmbs->platform.arg);

    if (ret == count)
        return NMBS_ERROR_NONE;

    if (ret < count) {
        if (ret < 0)
            return NMBS_ERROR_TRANSPORT;

        return NMBS_ERROR_TIMEOUT;
    }

    return NMBS_ERROR_TRANSPORT;
}


static void flush(nmbs_t* nmbs) {
    nmbs->platform.read(nmbs->msg.buf, sizeof(nmbs->msg.buf), 0, nmbs->platform.arg);
}


static void msg_buf_reset(nmbs_t* nmbs) {
    nmbs->msg.buf_idx = 0;
}


static void msg_state_reset(nmbs_t* nmbs) {
    msg_buf_reset(nmbs);
    nmbs->msg.unit_id = 0;
    nmbs->msg.fc = 0;
    nmbs->msg.transaction_id = 0;
    nmbs->msg.broadcast = false;
    nmbs->msg.ignored = false;
    nmbs->msg.complete = false;
}


static void msg_state_req(nmbs_t* nmbs, uint8_t fc) {
    if (nmbs->current_tid == UINT16_MAX)
        nmbs->current_tid = 1;
    else
        nmbs->current_tid++;

    // Flush the remaining data on the line before sending the request
    flush(nmbs);

    msg_state_reset(nmbs);
    nmbs->msg.unit_id = nmbs->dest_address_rtu;
    nmbs->msg.fc = fc;
    nmbs->msg.transaction_id = nmbs->current_tid;
    if (nmbs->msg.unit_id == NMBS_BROADCAST_ADDRESS && nmbs->platform.transport == NMBS_TRANSPORT_RTU)
        nmbs->msg.broadcast = true;
}


nmbs_error nmbs_create(nmbs_t* nmbs, const nmbs_platform_conf* platform_conf) {
    if (!nmbs)
        return NMBS_ERROR_INVALID_ARGUMENT;

    memset(nmbs, 0, sizeof(nmbs_t));

    nmbs->byte_timeout_ms = -1;
    nmbs->read_timeout_ms = -1;

    if (!platform_conf || platform_conf->initialized != 0xFFFFDEBE)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (platform_conf->transport != NMBS_TRANSPORT_RTU && platform_conf->transport != NMBS_TRANSPORT_TCP)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (!platform_conf->read || !platform_conf->write)
        return NMBS_ERROR_INVALID_ARGUMENT;

    nmbs->platform = *platform_conf;

    return NMBS_ERROR_NONE;
}


void nmbs_set_read_timeout(nmbs_t* nmbs, int32_t timeout_ms) {
    nmbs->read_timeout_ms = timeout_ms;
}


void nmbs_set_byte_timeout(nmbs_t* nmbs, int32_t timeout_ms) {
    nmbs->byte_timeout_ms = timeout_ms;
}


void nmbs_platform_conf_create(nmbs_platform_conf* platform_conf) {
    memset(platform_conf, 0, sizeof(nmbs_platform_conf));
    platform_conf->crc_calc = nmbs_crc_calc;
    // Workaround for older user code not calling nmbs_platform_conf_create()
    platform_conf->initialized = 0xFFFFDEBE;
}


void nmbs_set_destination_rtu_address(nmbs_t* nmbs, uint8_t address) {
    nmbs->dest_address_rtu = address;
}


void nmbs_set_platform_arg(nmbs_t* nmbs, void* arg) {
    nmbs->platform.arg = arg;
}


uint16_t nmbs_crc_calc(const uint8_t* data, uint32_t length, void* arg) {
    NMBS_UNUSED_PARAM(arg);
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t) data[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }

    return (uint16_t) (crc << 8) | (uint16_t) (crc >> 8);
}

static nmbs_error recv_msg_footer(nmbs_t* nmbs) {
    NMBS_DEBUG_PRINT("\n");

    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU) {
        const uint16_t crc = nmbs->platform.crc_calc(nmbs->msg.buf, nmbs->msg.buf_idx, nmbs->platform.arg);

        const nmbs_error err = recv(nmbs, 2);
        if (err != NMBS_ERROR_NONE)
            return err;

        const uint16_t recv_crc = get_2(nmbs);
        if (recv_crc != crc)
            return NMBS_ERROR_CRC;
    }

    return NMBS_ERROR_NONE;
}


static nmbs_error recv_msg_header(nmbs_t* nmbs, bool* first_byte_received) {
    // We wait for the read timeout here, just for the first message byte
    int32_t old_byte_timeout = nmbs->byte_timeout_ms;
    nmbs->byte_timeout_ms = nmbs->read_timeout_ms;

    msg_state_reset(nmbs);

    *first_byte_received = false;

    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU) {
        nmbs_error err = recv(nmbs, 1);

        nmbs->byte_timeout_ms = old_byte_timeout;

        if (err != NMBS_ERROR_NONE)
            return err;

        *first_byte_received = true;

        nmbs->msg.unit_id = get_1(nmbs);

        err = recv(nmbs, 1);
        if (err != NMBS_ERROR_NONE)
            return err;

        nmbs->msg.fc = get_1(nmbs);
    }
    else if (nmbs->platform.transport == NMBS_TRANSPORT_TCP) {
        nmbs_error err = recv(nmbs, 1);

        nmbs->byte_timeout_ms = old_byte_timeout;

        if (err != NMBS_ERROR_NONE)
            return err;

        *first_byte_received = true;

        // Advance buf_idx
        discard_1(nmbs);

        err = recv(nmbs, 7);
        if (err != NMBS_ERROR_NONE)
            return err;

        // Starting over
        msg_buf_reset(nmbs);

        nmbs->msg.transaction_id = get_2(nmbs);
        const uint16_t protocol_id = get_2(nmbs);
        const uint16_t length = get_2(nmbs);
        nmbs->msg.unit_id = get_1(nmbs);
        nmbs->msg.fc = get_1(nmbs);

        if (length < 2 || length > 255)
            return NMBS_ERROR_INVALID_TCP_MBAP;

        // Receive the rest of the message
        err = recv(nmbs, length - 2);
        if (err != NMBS_ERROR_NONE)
            return err;

        if (protocol_id != 0)
            return NMBS_ERROR_INVALID_TCP_MBAP;

        nmbs->msg.complete = true;
    }

    return NMBS_ERROR_NONE;
}


static void put_msg_header(nmbs_t* nmbs, uint16_t data_length) {
    msg_buf_reset(nmbs);

    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU) {
        put_1(nmbs, nmbs->msg.unit_id);
    }
    else if (nmbs->platform.transport == NMBS_TRANSPORT_TCP) {
        put_2(nmbs, nmbs->msg.transaction_id);
        put_2(nmbs, 0);
        put_2(nmbs, (uint16_t) (1 + 1 + data_length));
        put_1(nmbs, nmbs->msg.unit_id);
    }

    put_1(nmbs, nmbs->msg.fc);
}


static nmbs_error send_msg(nmbs_t* nmbs) {
    NMBS_DEBUG_PRINT("\n");

    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU) {
        const uint16_t crc = nmbs->platform.crc_calc(nmbs->msg.buf, nmbs->msg.buf_idx, nmbs->platform.arg);
        put_2(nmbs, crc);
    }

    const nmbs_error err = send(nmbs, nmbs->msg.buf_idx);

    return err;
}


static nmbs_error recv_res_header(nmbs_t* nmbs) {
    const uint16_t req_transaction_id = nmbs->msg.transaction_id;
    const uint8_t req_unit_id = nmbs->msg.unit_id;
    const uint8_t req_fc = nmbs->msg.fc;

    bool first_byte_received = false;
    nmbs_error err = recv_msg_header(nmbs, &first_byte_received);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (nmbs->platform.transport == NMBS_TRANSPORT_TCP) {
        if (nmbs->msg.transaction_id != req_transaction_id)
            return NMBS_ERROR_INVALID_TCP_MBAP;
    }

    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU && nmbs->msg.unit_id != req_unit_id)
        return NMBS_ERROR_INVALID_UNIT_ID;

    if (nmbs->msg.fc != req_fc) {
        if (nmbs->msg.fc - 0x80 == req_fc) {
            err = recv(nmbs, 1);
            if (err != NMBS_ERROR_NONE)
                return err;

            const uint8_t exception = get_1(nmbs);
            err = recv_msg_footer(nmbs);
            if (err != NMBS_ERROR_NONE)
                return err;

            if (exception < 1 || exception > 4)
                return NMBS_ERROR_INVALID_RESPONSE;

            NMBS_DEBUG_PRINT("%d NMBS res <- address_rtu %d\texception %d\n", nmbs->address_rtu, nmbs->msg.unit_id,
                             exception);
            return (nmbs_error) exception;
        }

        return NMBS_ERROR_INVALID_RESPONSE;
    }

    NMBS_DEBUG_PRINT("%d NMBS res <- address_rtu %d\tfc %d\t", nmbs->address_rtu, nmbs->msg.unit_id, nmbs->msg.fc);

    return NMBS_ERROR_NONE;
}


static void put_req_header(nmbs_t* nmbs, uint16_t data_length) {
    put_msg_header(nmbs, data_length);
#ifdef NMBS_DEBUG
    printf("%d ", nmbs->address_rtu);
    printf("NMBS req -> ");
    if (nmbs->platform.transport == NMBS_TRANSPORT_RTU) {
        if (nmbs->msg.broadcast)
            printf("broadcast\t");
        else
            printf("address_rtu %d\t", nmbs->dest_address_rtu);
    }

    printf("fc %d\t", nmbs->msg.fc);
#endif
}


static nmbs_error recv_read_discrete_res(nmbs_t* nmbs, nmbs_bitfield values) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 1);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t coils_bytes = get_1(nmbs);
    NMBS_DEBUG_PRINT("b %d\t", coils_bytes);

    if (coils_bytes > NMBS_BITFIELD_BYTES_MAX) {
        return NMBS_ERROR_INVALID_RESPONSE;
    }

    err = recv(nmbs, coils_bytes);
    if (err != NMBS_ERROR_NONE)
        return err;

    NMBS_DEBUG_PRINT("coils ");
    for (int i = 0; i < coils_bytes; i++) {
        const uint8_t coil = get_1(nmbs);
        if (values)
            values[i] = coil;
        NMBS_DEBUG_PRINT("%d ", coil);
    }

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return NMBS_ERROR_NONE;
}


static nmbs_error recv_read_registers_res(nmbs_t* nmbs, uint16_t quantity, uint16_t* registers) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 1);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t registers_bytes = get_1(nmbs);
    NMBS_DEBUG_PRINT("b %d\t", registers_bytes);

    if (registers_bytes > 250)
        return NMBS_ERROR_INVALID_RESPONSE;

    err = recv(nmbs, registers_bytes);
    if (err != NMBS_ERROR_NONE)
        return err;

    NMBS_DEBUG_PRINT("regs ");
    for (int i = 0; i < registers_bytes / 2; i++) {
        uint16_t reg = get_2(nmbs);
        if (registers)
            registers[i] = reg;
        NMBS_DEBUG_PRINT("%d ", reg);
    }

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (registers_bytes != quantity * 2)
        return NMBS_ERROR_INVALID_RESPONSE;

    return NMBS_ERROR_NONE;
}


nmbs_error recv_write_single_coil_res(nmbs_t* nmbs, uint16_t address, uint16_t value_req) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 4);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint16_t address_res = get_2(nmbs);
    const uint16_t value_res = get_2(nmbs);

    NMBS_DEBUG_PRINT("a %d\tvalue %d", address, value_res);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (address_res != address)
        return NMBS_ERROR_INVALID_RESPONSE;

    if (value_res != value_req)
        return NMBS_ERROR_INVALID_RESPONSE;

    return NMBS_ERROR_NONE;
}


nmbs_error recv_write_single_register_res(nmbs_t* nmbs, uint16_t address, uint16_t value_req) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 4);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint16_t address_res = get_2(nmbs);
    const uint16_t value_res = get_2(nmbs);
    NMBS_DEBUG_PRINT("a %d\tvalue %d ", address, value_res);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (address_res != address)
        return NMBS_ERROR_INVALID_RESPONSE;

    if (value_res != value_req)
        return NMBS_ERROR_INVALID_RESPONSE;

    return NMBS_ERROR_NONE;
}


nmbs_error recv_write_multiple_coils_res(nmbs_t* nmbs, uint16_t address, uint16_t quantity) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 4);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint16_t address_res = get_2(nmbs);
    const uint16_t quantity_res = get_2(nmbs);
    NMBS_DEBUG_PRINT("a %d\tq %d", address_res, quantity_res);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (address_res != address)
        return NMBS_ERROR_INVALID_RESPONSE;

    if (quantity_res != quantity)
        return NMBS_ERROR_INVALID_RESPONSE;

    return NMBS_ERROR_NONE;
}


nmbs_error recv_write_multiple_registers_res(nmbs_t* nmbs, uint16_t address, uint16_t quantity) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 4);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint16_t address_res = get_2(nmbs);
    const uint16_t quantity_res = get_2(nmbs);
    NMBS_DEBUG_PRINT("a %d\tq %d", address_res, quantity_res);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (address_res != address)
        return NMBS_ERROR_INVALID_RESPONSE;

    if (quantity_res != quantity)
        return NMBS_ERROR_INVALID_RESPONSE;

    return NMBS_ERROR_NONE;
}


nmbs_error recv_read_file_record_res(nmbs_t* nmbs, uint16_t* registers, uint16_t count) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 1);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t response_size = get_1(nmbs);
    if (response_size > 250) {
        return NMBS_ERROR_INVALID_RESPONSE;
    }

    err = recv(nmbs, response_size);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t subreq_data_size = get_1(nmbs) - 1;
    const uint8_t subreq_reference_type = get_1(nmbs);
    uint16_t* subreq_record_data = (uint16_t*) get_n(nmbs, subreq_data_size);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (registers) {
        if (subreq_reference_type != 6)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (count != (subreq_data_size / 2))
            return NMBS_ERROR_INVALID_RESPONSE;

        swap_regs(subreq_record_data, subreq_data_size / 2);
        memcpy(registers, subreq_record_data, subreq_data_size);
    }

    return NMBS_ERROR_NONE;
}


nmbs_error recv_write_file_record_res(nmbs_t* nmbs, uint16_t file_number, uint16_t record_number,
                                      const uint16_t* registers, uint16_t count) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 1);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t response_size = get_1(nmbs);
    if (response_size > 251)
        return NMBS_ERROR_INVALID_RESPONSE;

    err = recv(nmbs, response_size);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t subreq_reference_type = get_1(nmbs);
    const uint16_t subreq_file_number = get_2(nmbs);
    const uint16_t subreq_record_number = get_2(nmbs);
    const uint16_t subreq_record_length = get_2(nmbs);
    NMBS_DEBUG_PRINT("a %d\tr %d\tl %d\t fwrite ", subreq_file_number, subreq_record_number, subreq_record_length);

    uint16_t subreq_data_size = subreq_record_length * 2;
    uint16_t* subreq_record_data = (uint16_t*) get_n(nmbs, subreq_data_size);

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (registers) {
        if (subreq_reference_type != 6)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (subreq_file_number != file_number)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (subreq_record_number != record_number)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (subreq_record_length != count)
            return NMBS_ERROR_INVALID_RESPONSE;

        swap_regs(subreq_record_data, subreq_record_length);
        if (memcmp(registers, subreq_record_data, subreq_data_size) != 0)
            return NMBS_ERROR_INVALID_RESPONSE;
    }

    return NMBS_ERROR_NONE;
}

nmbs_error recv_read_device_identification_res(nmbs_t* nmbs, uint8_t buffers_count, char** buffers_out,
                                               uint8_t buffers_length, const uint8_t* order, uint8_t* ids_out,
                                               uint8_t* next_object_id_out, uint8_t* objects_count_out) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, 6);
    if (err != NMBS_ERROR_NONE)
        return err;

    const uint8_t mei_type = get_1(nmbs);
    if (mei_type != 0x0E)
        return NMBS_ERROR_INVALID_RESPONSE;

    const uint8_t read_device_id_code = get_1(nmbs);
    if (read_device_id_code < 1 || read_device_id_code > 4)
        return NMBS_ERROR_INVALID_RESPONSE;

    const uint8_t conformity_level = get_1(nmbs);
    if (conformity_level < 1 || (conformity_level > 3 && conformity_level < 0x81) || conformity_level > 0x83)
        return NMBS_ERROR_INVALID_RESPONSE;

    const uint8_t more_follows = get_1(nmbs);
    if (more_follows != 0 && more_follows != 0xFF)
        return NMBS_ERROR_INVALID_RESPONSE;

    uint8_t next_object_id = get_1(nmbs);

    const uint8_t objects_count = get_1(nmbs);
    if (objects_count_out)
        *objects_count_out = objects_count;

    if (buffers_count == 0) {
        buffers_out = NULL;
    }
    else if (objects_count > buffers_count)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (more_follows == 0)
        next_object_id = 0x7F;    // This value is reserved in the spec, we use it to signal stream is finished

    if (next_object_id_out)
        *next_object_id_out = next_object_id;

    uint8_t res_size_left = 253 - 7;
    for (int i = 0; i < objects_count; i++) {
        err = recv(nmbs, 2);
        if (err != NMBS_ERROR_NONE)
            return err;

        const uint8_t object_id = get_1(nmbs);
        const uint8_t object_length = get_1(nmbs);
        res_size_left -= 2;

        if (object_length > res_size_left)
            return NMBS_ERROR_INVALID_RESPONSE;

        err = recv(nmbs, object_length);
        if (err != NMBS_ERROR_NONE)
            return err;

        const char* str = (const char*) get_n(nmbs, object_length);

        if (ids_out)
            ids_out[i] = object_id;

        uint8_t buf_index = i;
        if (order)
            buf_index = order[object_id];
        if (buffers_out) {
#ifndef WIN32
            strncpy(buffers_out[buf_index], str, buffers_length);
#else
            strncpy_s(buffers_out[buf_index], buffers_length, str, object_length);
#endif
            buffers_out[buf_index][object_length] = 0;
        }
    }

    return recv_msg_footer(nmbs);
}


nmbs_error nmbs_client_create(nmbs_t* nmbs, const nmbs_platform_conf* platform_conf) {
    return nmbs_create(nmbs, platform_conf);
}


static nmbs_error read_discrete(nmbs_t* nmbs, uint8_t fc, uint16_t address, uint16_t quantity, nmbs_bitfield values) {
    if (quantity < 1 || quantity > NMBS_BITFIELD_MAX)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) address + (uint32_t) quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    msg_state_req(nmbs, fc);
    put_req_header(nmbs, 4);

    put_2(nmbs, address);
    put_2(nmbs, quantity);

    NMBS_DEBUG_PRINT("a %d\tq %d", address, quantity);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return recv_read_discrete_res(nmbs, values);
}


nmbs_error nmbs_read_coils(nmbs_t* nmbs, uint16_t address, uint16_t quantity, nmbs_bitfield coils_out) {
    return read_discrete(nmbs, 1, address, quantity, coils_out);
}


nmbs_error nmbs_read_discrete_inputs(nmbs_t* nmbs, uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out) {
    return read_discrete(nmbs, 2, address, quantity, inputs_out);
}

static nmbs_error read_registers(nmbs_t* nmbs, uint8_t fc, uint16_t address, uint16_t quantity, uint16_t* registers) {
    if (quantity < 1 || quantity > 125)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) address + (uint32_t) quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    msg_state_req(nmbs, fc);
    put_req_header(nmbs, 4);

    put_2(nmbs, address);
    put_2(nmbs, quantity);

    NMBS_DEBUG_PRINT("a %d\tq %d ", address, quantity);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return recv_read_registers_res(nmbs, quantity, registers);
}


nmbs_error nmbs_read_holding_registers(nmbs_t* nmbs, uint16_t address, uint16_t quantity, uint16_t* registers_out) {
    return read_registers(nmbs, 3, address, quantity, registers_out);
}


nmbs_error nmbs_read_input_registers(nmbs_t* nmbs, uint16_t address, uint16_t quantity, uint16_t* registers_out) {
    return read_registers(nmbs, 4, address, quantity, registers_out);
}


nmbs_error nmbs_write_single_coil(nmbs_t* nmbs, uint16_t address, bool value) {
    msg_state_req(nmbs, 5);
    put_req_header(nmbs, 4);

    const uint16_t value_req = value ? 0xFF00 : 0;

    put_2(nmbs, address);
    put_2(nmbs, value_req);

    NMBS_DEBUG_PRINT("a %d\tvalue %d ", address, value_req);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (!nmbs->msg.broadcast)
        return recv_write_single_coil_res(nmbs, address, value_req);

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_write_single_register(nmbs_t* nmbs, uint16_t address, uint16_t value) {
    msg_state_req(nmbs, 6);
    put_req_header(nmbs, 4);

    put_2(nmbs, address);
    put_2(nmbs, value);

    NMBS_DEBUG_PRINT("a %d\tvalue %d", address, value);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (!nmbs->msg.broadcast)
        return recv_write_single_register_res(nmbs, address, value);

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_write_multiple_coils(nmbs_t* nmbs, uint16_t address, uint16_t quantity, const nmbs_bitfield coils) {
    if (quantity < 1 || quantity > 0x07B0)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) address + (uint32_t) quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    uint8_t coils_bytes = (quantity + 7) / 8;

    msg_state_req(nmbs, 15);
    put_req_header(nmbs, 5 + coils_bytes);

    put_2(nmbs, address);
    put_2(nmbs, quantity);
    put_1(nmbs, coils_bytes);
    NMBS_DEBUG_PRINT("a %d\tq %d\tb %d\t", address, quantity, coils_bytes);

    NMBS_DEBUG_PRINT("coils ");
    for (int i = 0; i < coils_bytes; i++) {
        put_1(nmbs, coils[i]);
        NMBS_DEBUG_PRINT("%d ", coils[i]);
    }

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (!nmbs->msg.broadcast)
        return recv_write_multiple_coils_res(nmbs, address, quantity);

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_write_multiple_registers(nmbs_t* nmbs, uint16_t address, uint16_t quantity, const uint16_t* registers) {
    if (quantity < 1 || quantity > 0x007B)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) address + (uint32_t) quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    const uint8_t registers_bytes = quantity * 2;

    msg_state_req(nmbs, 16);
    put_req_header(nmbs, 5 + registers_bytes);

    put_2(nmbs, address);
    put_2(nmbs, quantity);
    put_1(nmbs, registers_bytes);
    NMBS_DEBUG_PRINT("a %d\tq %d\tb %d\t", address, quantity, registers_bytes);

    NMBS_DEBUG_PRINT("regs ");
    for (int i = 0; i < quantity; i++) {
        put_2(nmbs, registers[i]);
        NMBS_DEBUG_PRINT("%d ", registers[i]);
    }

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (!nmbs->msg.broadcast)
        return recv_write_multiple_registers_res(nmbs, address, quantity);

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_read_file_record(nmbs_t* nmbs, uint16_t file_number, uint16_t record_number, uint16_t* registers,
                                 uint16_t count) {
    if (file_number == 0x0000)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (record_number > 0x270F)
        return NMBS_ERROR_INVALID_ARGUMENT;

    // In expected response: max PDU length = 253, assuming a single file request, (253 - 1 - 1 - 1 - 1) / 2 = 124
    if (count > 124)
        return NMBS_ERROR_INVALID_ARGUMENT;

    msg_state_req(nmbs, 20);
    put_req_header(nmbs, 8);

    put_1(nmbs, 7);    // add Byte Count
    put_1(nmbs, 6);    // add Reference Type const
    put_2(nmbs, file_number);
    put_2(nmbs, record_number);
    put_2(nmbs, count);
    NMBS_DEBUG_PRINT("a %d\tr %d\tl %d\t fread ", file_number, record_number, count);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return recv_read_file_record_res(nmbs, registers, count);
}


nmbs_error nmbs_write_file_record(nmbs_t* nmbs, uint16_t file_number, uint16_t record_number, const uint16_t* registers,
                                  uint16_t count) {
    if (file_number == 0x0000)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (record_number > 0x270F)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (count > 122)
        return NMBS_ERROR_INVALID_ARGUMENT;

    const uint16_t data_size = count * 2;

    msg_state_req(nmbs, 21);
    put_req_header(nmbs, 8 + data_size);

    put_1(nmbs, 7 + data_size);    // add Byte Count
    put_1(nmbs, 6);                // add Reference Type const
    put_2(nmbs, file_number);
    put_2(nmbs, record_number);
    put_2(nmbs, count);
    put_regs(nmbs, registers, count);
    NMBS_DEBUG_PRINT("a %d\tr %d\tl %d\t fwrite ", file_number, record_number, count);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (!nmbs->msg.broadcast)
        return recv_write_file_record_res(nmbs, file_number, record_number, registers, count);

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_read_write_registers(nmbs_t* nmbs, uint16_t read_address, uint16_t read_quantity,
                                     uint16_t* registers_out, uint16_t write_address, uint16_t write_quantity,
                                     const uint16_t* registers) {
    if (read_quantity < 1 || read_quantity > 0x007D)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) read_address + (uint32_t) read_quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if (write_quantity < 1 || write_quantity > 0x0079)
        return NMBS_ERROR_INVALID_ARGUMENT;

    if ((uint32_t) write_address + (uint32_t) write_quantity > ((uint32_t) 0xFFFF) + 1)
        return NMBS_ERROR_INVALID_ARGUMENT;

    const uint8_t registers_bytes = write_quantity * 2;

    msg_state_req(nmbs, 23);
    put_req_header(nmbs, 9 + registers_bytes);

    put_2(nmbs, read_address);
    put_2(nmbs, read_quantity);
    put_2(nmbs, write_address);
    put_2(nmbs, write_quantity);
    put_1(nmbs, registers_bytes);

    NMBS_DEBUG_PRINT("read a %d\tq %d ", read_address, read_quantity);
    NMBS_DEBUG_PRINT("write a %d\tq %d\tb %d\t", write_address, write_quantity, registers_bytes);

    NMBS_DEBUG_PRINT("regs ");
    for (int i = 0; i < write_quantity; i++) {
        put_2(nmbs, registers[i]);
        NMBS_DEBUG_PRINT("%d ", registers[i]);
    }

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return recv_read_registers_res(nmbs, read_quantity, registers_out);
}


nmbs_error nmbs_read_device_identification_basic(nmbs_t* nmbs, char* vendor_name, char* product_code,
                                                 char* major_minor_revision, uint8_t buffers_length) {
    const uint8_t order[3] = {0, 1, 2};
    char* buffers[3] = {vendor_name, product_code, major_minor_revision};
    uint8_t total_received = 0;
    uint8_t next_object_id = 0x00;

    while (next_object_id != 0x7F) {
        msg_state_req(nmbs, 43);
        put_msg_header(nmbs, 3);
        put_1(nmbs, 0x0E);
        put_1(nmbs, 1);
        put_1(nmbs, next_object_id);

        nmbs_error err = send_msg(nmbs);
        if (err != NMBS_ERROR_NONE)
            return err;

        uint8_t objects_received = 0;
        err = recv_read_device_identification_res(nmbs, 3, buffers, buffers_length, order, NULL, &next_object_id,
                                                  &objects_received);
        if (err != NMBS_ERROR_NONE)
            return err;

        total_received += objects_received;
        if (total_received > 3)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (objects_received == 0)
            return NMBS_ERROR_INVALID_RESPONSE;
    }

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_read_device_identification_regular(nmbs_t* nmbs, char* vendor_url, char* product_name, char* model_name,
                                                   char* user_application_name, uint8_t buffers_length) {
    const uint8_t order[7] = {0, 0, 0, 0, 1, 2, 3};
    char* buffers[4] = {vendor_url, product_name, model_name, user_application_name};
    uint8_t total_received = 0;
    uint8_t next_object_id = 0x03;

    while (next_object_id != 0x7F) {
        msg_state_req(nmbs, 43);
        put_req_header(nmbs, 3);
        put_1(nmbs, 0x0E);
        put_1(nmbs, 2);
        put_1(nmbs, next_object_id);

        nmbs_error err = send_msg(nmbs);
        if (err != NMBS_ERROR_NONE)
            return err;

        uint8_t objects_received = 0;
        err = recv_read_device_identification_res(nmbs, 4, buffers, buffers_length, order, NULL, &next_object_id,
                                                  &objects_received);
        if (err != NMBS_ERROR_NONE)
            return err;

        total_received += objects_received;
        if (total_received > 4)
            return NMBS_ERROR_INVALID_RESPONSE;

        if (objects_received == 0)
            return NMBS_ERROR_INVALID_RESPONSE;
    }

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_read_device_identification_extended(nmbs_t* nmbs, uint8_t object_id_start, uint8_t* ids, char** buffers,
                                                    uint8_t ids_length, uint8_t buffer_length,
                                                    uint8_t* objects_count_out) {
    if (object_id_start < 0x80)
        return NMBS_ERROR_INVALID_ARGUMENT;

    uint8_t total_received = 0;
    uint8_t next_object_id = object_id_start;

    while (next_object_id != 0x7F) {
        msg_state_req(nmbs, 43);
        put_req_header(nmbs, 3);
        put_1(nmbs, 0x0E);
        put_1(nmbs, 3);
        put_1(nmbs, next_object_id);

        nmbs_error err = send_msg(nmbs);
        if (err != NMBS_ERROR_NONE)
            return err;

        uint8_t objects_received = 0;
        err = recv_read_device_identification_res(nmbs, ids_length - total_received, &buffers[total_received],
                                                  buffer_length, NULL, &ids[total_received], &next_object_id,
                                                  &objects_received);
        if (err != NMBS_ERROR_NONE)
            return err;

        total_received += objects_received;
    }

    *objects_count_out = total_received;

    return NMBS_ERROR_NONE;
}


nmbs_error nmbs_read_device_identification(nmbs_t* nmbs, uint8_t object_id, char* buffer, uint8_t buffer_length) {
    if (object_id > 0x06 && object_id < 0x80)
        return NMBS_ERROR_INVALID_ARGUMENT;

    msg_state_req(nmbs, 43);
    put_req_header(nmbs, 3);
    put_1(nmbs, 0x0E);
    put_1(nmbs, 4);
    put_1(nmbs, object_id);

    const nmbs_error err = send_msg(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    char* buf[1] = {buffer};
    return recv_read_device_identification_res(nmbs, 1, buf, buffer_length, NULL, NULL, NULL, NULL);
}


nmbs_error nmbs_send_raw_pdu(nmbs_t* nmbs, uint8_t fc, const uint8_t* data, uint16_t data_len) {
    msg_state_req(nmbs, fc);
    put_msg_header(nmbs, data_len);

    NMBS_DEBUG_PRINT("raw ");
    for (uint16_t i = 0; i < data_len; i++) {
        put_1(nmbs, data[i]);
        NMBS_DEBUG_PRINT("%d ", data[i]);
    }

    return send_msg(nmbs);
}


nmbs_error nmbs_receive_raw_pdu_response(nmbs_t* nmbs, uint8_t* data_out, uint8_t data_out_len) {
    nmbs_error err = recv_res_header(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    err = recv(nmbs, data_out_len);
    if (err != NMBS_ERROR_NONE)
        return err;

    if (data_out) {
        for (uint16_t i = 0; i < data_out_len; i++)
            data_out[i] = get_1(nmbs);
    }
    else {
        for (uint16_t i = 0; i < data_out_len; i++)
            get_1(nmbs);
    }

    err = recv_msg_footer(nmbs);
    if (err != NMBS_ERROR_NONE)
        return err;

    return NMBS_ERROR_NONE;
}


#ifndef NMBS_STRERROR_DISABLED
const char* nmbs_strerror(nmbs_error error) {
    switch (error) {
        case NMBS_ERROR_INVALID_REQUEST:
            return "invalid request received";

        case NMBS_ERROR_INVALID_UNIT_ID:
            return "invalid unit ID received";

        case NMBS_ERROR_INVALID_TCP_MBAP:
            return "invalid TCP MBAP received";

        case NMBS_ERROR_CRC:
            return "invalid CRC received";

        case NMBS_ERROR_TRANSPORT:
            return "transport error";

        case NMBS_ERROR_TIMEOUT:
            return "timeout";

        case NMBS_ERROR_INVALID_RESPONSE:
            return "invalid response received";

        case NMBS_ERROR_INVALID_ARGUMENT:
            return "invalid argument provided";

        case NMBS_ERROR_NONE:
            return "no error";

        case NMBS_EXCEPTION_ILLEGAL_FUNCTION:
            return "modbus exception 1: illegal function";

        case NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS:
            return "modbus exception 2: illegal data address";

        case NMBS_EXCEPTION_ILLEGAL_DATA_VALUE:
            return "modbus exception 3: data value";

        case NMBS_EXCEPTION_SERVER_DEVICE_FAILURE:
            return "modbus exception 4: server device failure";

        default:
            return "unknown error";
    }
}
#endif
