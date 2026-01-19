#include "ld2402.h"
#include "soft_uart.h"
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* HLK protocol constants (from hlk_ld2402 component) */
static const uint8_t FRAME_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t FRAME_FOOTER[] = {0x04, 0x03, 0x02, 0x01};
static const uint8_t DATA_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_TYPE_DISTANCE = 0x83;
static const uint8_t DATA_FRAME_TYPE_ENGINEERING = 0x84;
/* Footer for binary data frames (seen in sample frames) */
static const uint8_t DATA_FRAME_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};

/* Commands we use */
static const uint16_t CMD_GET_VERSION = 0x0000;
static const uint16_t CMD_ENABLE_CONFIG = 0x00FF;
static const uint16_t CMD_DISABLE_CONFIG = 0x00FE;
static const uint16_t CMD_SET_MODE = 0x0012;
static const uint16_t CMD_READ_PARAMS = 0x0008; /* read sensor parameter configuration */

/* Modes */
static const uint32_t MODE_PRODUCTION = 0x00000064;

/* UART ring buffer for soft_uart bytes */
#define UART_RING_SIZE 512
static volatile uint8_t uart_ring[UART_RING_SIZE];
static volatile uint16_t uart_head = 0;
static volatile uint16_t uart_tail = 0;

/* Last-read module parameter: maximum distance in cm (or -1.0 if unknown) */
static float module_max_distance_cm = -1.0f;

static inline void uart_ring_push(uint8_t b)
{
    unsigned int key = irq_lock();
    uint16_t next = (uart_head + 1) % UART_RING_SIZE;
    if (next != uart_tail) { /* drop if full */
        uart_ring[uart_head] = b;
        uart_head = next;
    }
    irq_unlock(key);
}

static inline int uart_available(void)
{
    unsigned int key = irq_lock();
    int avail = (uart_head + UART_RING_SIZE - uart_tail) % UART_RING_SIZE;
    irq_unlock(key);
    return avail;
}

static inline int uart_read_byte(uint8_t *b)
{
    unsigned int key = irq_lock();
    if (uart_tail == uart_head) {
        irq_unlock(key);
        return 0;
    }
    *b = uart_ring[uart_tail];
    uart_tail = (uart_tail + 1) % UART_RING_SIZE;
    irq_unlock(key);
    return 1;
}

/* soft_uart callback - push into ring buffer */
void ld2402_uart_rx(uint8_t byte)
{
    uart_ring_push(byte);
}

/* --- Parsing state for incoming data frames / text lines --- */
static uint8_t frame_buf[256];
static size_t frame_pos = 0;
static bool in_data_frame = false;
static uint8_t header_idx = 0; /* for detecting DATA_FRAME_HEADER */

static char line_buf[1024];
static size_t line_pos = 0;

/* Parsing for FRAME_HEADER-style command/response frames */
static bool in_cmd_frame = false;
static uint8_t cmd_frame_buf[512];
static size_t cmd_frame_pos = 0;
static uint8_t cmd_header_idx = 0;

/* Report callback */
static ld2402_report_cb_t report_cb = NULL;

void ld2402_set_report_callback(ld2402_report_cb_t cb)
{
    report_cb = cb;
}

/* Parameter callback */
static ld2402_param_cb_t param_cb = NULL;

void ld2402_set_param_callback(ld2402_param_cb_t cb)
{
    param_cb = cb;
}

/* Send a parameter/configuration frame: header + len(2) + cmd(2) + param_id(2) + payload + footer */
void ld2402_send_param_frame(uint16_t cmd, uint16_t param_id, const uint8_t *payload, size_t payload_len)
{
    uint8_t frame[256];
    size_t pos = 0;
    memcpy(&frame[pos], FRAME_HEADER, sizeof(FRAME_HEADER));
    pos += sizeof(FRAME_HEADER);
    uint16_t total_len = (uint16_t)(2 + 2 + payload_len); /* command(2) + param_id(2) + payload */
    frame[pos++] = (uint8_t)(total_len & 0xFF);
    frame[pos++] = (uint8_t)((total_len >> 8) & 0xFF);
    frame[pos++] = (uint8_t)(cmd & 0xFF);
    frame[pos++] = (uint8_t)((cmd >> 8) & 0xFF);
    frame[pos++] = (uint8_t)(param_id & 0xFF);
    frame[pos++] = (uint8_t)((param_id >> 8) & 0xFF);
    if (payload && payload_len) {
        memcpy(&frame[pos], payload, payload_len);
        pos += payload_len;
    }
    memcpy(&frame[pos], FRAME_FOOTER, sizeof(FRAME_FOOTER));
    pos += sizeof(FRAME_FOOTER);

    soft_uart_write_buf(frame, pos);
}

/* Process a textual line (e.g., lines that may contain 'distance:') */
static void process_line(const char *line)
{
    if (!line || !*line) return;

    /* Try to find "distance:" substring */
    const char *p = strstr(line, "distance:");
    if (p) {
        p += strlen("distance:");
        /* skip spaces */
        while (*p == ' ') p++;
        char tmp[32] = {0};
        size_t i = 0;
        while (*p && (isdigit((unsigned char)*p) || *p == '.' ) && i < sizeof(tmp)-1) {
            tmp[i++] = *p++;
        }
        if (i > 0) {
            float d = strtof(tmp, NULL);
            if (report_cb) {
                ld2402_report_t rep = {0};
                rep.timestamp_ms = k_uptime_get_32();
                rep.frame_type = 0;
                rep.data_len = 0;
                rep.distance_cm = d;
                rep.distances_count = 1;
                rep.distances[0] = d;
                /* copy raw line into raw_payload for debug */
                size_t l = strlen(line);
                if (l > sizeof(rep.raw_payload)) l = sizeof(rep.raw_payload);
                memcpy(rep.raw_payload, line, l);
                rep.raw_payload_len = l;
                report_cb(&rep);
            }
        }
        return;
    }

    /* Else, try parse a pure numeric line */
    bool numeric = true;
    for (size_t i = 0; line[i]; i++) {
        if (!isdigit((unsigned char)line[i]) && line[i] != '.') { numeric = false; break; }
    }
    if (numeric) {
        float d = strtof(line, NULL);
        if (report_cb) {
            ld2402_report_t rep = {0};
            rep.timestamp_ms = k_uptime_get_32();
            rep.frame_type = 0;
            rep.data_len = 0;
            rep.distance_cm = d;
            rep.distances_count = 1;
            rep.distances[0] = d;
            size_t l = strlen(line);
            if (l > sizeof(rep.raw_payload)) l = sizeof(rep.raw_payload);
            memcpy(rep.raw_payload, line, l);
            rep.raw_payload_len = l;
            report_cb(&rep);
        }
    }
}

/* Parse HLK binary data frames for distance / engineering frames */
static void process_binary_frame(uint8_t *buf, size_t len)
{
    if (len < 9) return;

    /* Data reporting mode frame format:
     * Header: F4 F3 F2 F1
     * Length: buf[4..5] (little-endian)
     * Detection result: buf[6]
     * Target distance: buf[7..8] (little-endian, cm)
     * Following bytes: sequence of 4-byte little-endian "energy" values
     */
    uint16_t data_len = (uint16_t)(buf[4] | (buf[5] << 8));
    if (data_len == 0) return;

    ld2402_report_t rep = {0};
    rep.timestamp_ms = k_uptime_get_32();
    rep.frame_type = DATA_FRAME_TYPE_DISTANCE; /* semantic */
    rep.data_len = data_len;

    /* Detection result */
    rep.detection_result = buf[6];

    /* Target distance (cm) */
    uint16_t target = (uint16_t)(buf[7] | (buf[8] << 8));
    rep.target_distance_cm = (float)target;
    rep.distance_cm = rep.target_distance_cm; /* primary distance */

    /* Parse subsequent 4-byte energy samples starting at index 9 */
    size_t pos = 9;
    size_t ecount = 0;
    while (pos + 3 < len && ecount < sizeof(rep.energy_values)/sizeof(rep.energy_values[0])) {
        uint32_t v = (uint32_t)buf[pos] | ((uint32_t)buf[pos+1] << 8) | ((uint32_t)buf[pos+2] << 16) | ((uint32_t)buf[pos+3] << 24);
        if (v > 0) {
            /* energy dB = 10 * log10(v) */
            double edb = 10.0 * log10((double)v);
            rep.energy_values[ecount++] = (float)edb;
        } else {
            rep.energy_values[ecount++] = 0.0f;
        }
        pos += 4;
    }
    rep.energy_values_count = ecount;

    /* copy raw payload (limit) */
    size_t copy_len = len;
    if (copy_len > sizeof(rep.raw_payload)) copy_len = sizeof(rep.raw_payload);
    memcpy(rep.raw_payload, buf, copy_len);
    rep.raw_payload_len = copy_len;

    if (report_cb) report_cb(&rep);
}

/* Public processing function - call regularly from main loop */
void ld2402_process(void)
{
    uint8_t b;

    while (uart_read_byte(&b)) {
        /* First, check for command/response FRAME_HEADER sequence */
        if (!in_cmd_frame) {
            if (b == FRAME_HEADER[cmd_header_idx]) {
                cmd_header_idx++;
                if (cmd_header_idx == sizeof(FRAME_HEADER)) {
                    in_cmd_frame = true;
                    cmd_frame_pos = 0;
                    memcpy(cmd_frame_buf, FRAME_HEADER, sizeof(FRAME_HEADER));
                    cmd_frame_pos = sizeof(FRAME_HEADER);
                    cmd_header_idx = 0;
                }
                /* continue to next byte */
                continue;
            } else {
                cmd_header_idx = 0;
            }
        }

        /* Next, check for data frame header sequence if not in data frame */
        if (!in_data_frame) {
            if (b == DATA_FRAME_HEADER[header_idx]) {
                header_idx++;
                if (header_idx == sizeof(DATA_FRAME_HEADER)) {
                    /* start data frame */
                    in_data_frame = true;
                    frame_pos = 0;
                    memcpy(frame_buf, DATA_FRAME_HEADER, sizeof(DATA_FRAME_HEADER));
                    frame_pos = sizeof(DATA_FRAME_HEADER);
                    header_idx = 0;
                }
                continue;
            } else {
                /* not part of header - reset header_idx and treat as text */
                header_idx = 0;
            }
        }

        if (in_data_frame) {
            /* append into frame buffer */
            if (frame_pos < sizeof(frame_buf)) frame_buf[frame_pos++] = b;

            /* need at least 8 bytes to read length field */
            if (frame_pos >= 8) {
                uint16_t data_len = frame_buf[6] | (frame_buf[7] << 8);
                size_t expected = 8 + data_len;
                if (frame_pos >= expected) {
                    /* Verify footer sequence to avoid mixing frames */
                    bool footer_ok = false;
                    /* If footer sits at expected-4..expected-1 */
                    if (expected >= 4) {
                        if (memcmp(&frame_buf[expected - 4], DATA_FRAME_FOOTER, sizeof(DATA_FRAME_FOOTER)) == 0) {
                            footer_ok = true;
                        }
                    }
                    /* Also accept footer at current end (in case length calculation differs) */
                    if (!footer_ok && frame_pos >= sizeof(DATA_FRAME_FOOTER)) {
                        if (memcmp(&frame_buf[frame_pos - sizeof(DATA_FRAME_FOOTER)], DATA_FRAME_FOOTER, sizeof(DATA_FRAME_FOOTER)) == 0) {
                            footer_ok = true;
                        }
                    }

                    if (footer_ok) {
                        /* We have the complete binary frame payload */
                        process_binary_frame(frame_buf, frame_pos);
                        in_data_frame = false;
                        frame_pos = 0;
                        continue;
                    }
                    /* else keep collecting until footer is seen */
                }
                /* else keep collecting */
            }
            continue;
        }

        /* If collecting a command/response frame, append and check for completion */
        if (in_cmd_frame) {
            if (cmd_frame_pos < sizeof(cmd_frame_buf)) cmd_frame_buf[cmd_frame_pos++] = b;
            /* need at least header + 2 length bytes + 2 cmd bytes + 1 ack maybe -> 9 */
            if (cmd_frame_pos >= 8) {
                uint16_t data_len = cmd_frame_buf[4] | (cmd_frame_buf[5] << 8);
                size_t expected = 8 + data_len; /* header(4)+len(2)+cmd(2)+data_len */
                if (cmd_frame_pos >= expected) {
                    /* parse command/response frame */
                    uint16_t cmd = cmd_frame_buf[6] | (cmd_frame_buf[7] << 8);
                    size_t data_start = 8;
                    size_t data_bytes = expected - 8; /* data_len */
                    uint16_t param_id = 0;
                    float param_value = 0.0f;
                    const uint8_t *raw = NULL;
                    size_t raw_len = 0;

                    /* Expected format: param_id(2) + ack(2) + param_value(4) */
                    if (data_bytes >= 8) {
                        param_id = (uint16_t)(cmd_frame_buf[data_start] | (cmd_frame_buf[data_start+1] << 8));
                        /* ack bytes at data_start+2..+3 (ignored except for sanity) */
                        uint16_t ack = (uint16_t)(cmd_frame_buf[data_start+2] | (cmd_frame_buf[data_start+3] << 8));
                        /* ack: treat 0x0000 as the only success indicator */
                        bool ack_ok = (ack == 0x0000);
                        /* 4-byte little-endian parameter value */
                        uint32_t val = (uint32_t)cmd_frame_buf[data_start+4] | ((uint32_t)cmd_frame_buf[data_start+5] << 8) | ((uint32_t)cmd_frame_buf[data_start+6] << 16) | ((uint32_t)cmd_frame_buf[data_start+7] << 24);
                        param_value = (float)val / 10.0f;
                        /* If this was a read-params response, ACK indicates success,
                         * and the parameter id matches the request (0x0001), treat
                         * the returned parameter as maximum distance (cm). */
                        if (ack_ok && cmd == CMD_READ_PARAMS && param_id == 0x0001) {
                            module_max_distance_cm = param_value;
                        }
                        raw = &cmd_frame_buf[data_start+4];
                        raw_len = 4;
                    } else {
                        /* Fallback: try previous heuristic for shorter frames */
                        if (data_bytes >= 2) {
                            uint8_t first = cmd_frame_buf[data_start];
                            if (first == 0x00 || first == 0x01 || first == 0x06) {
                                raw = &cmd_frame_buf[data_start+1];
                                raw_len = data_bytes - 1;
                            } else {
                                param_id = first;
                                raw = &cmd_frame_buf[data_start+1];
                                raw_len = data_bytes - 1;
                            }
                            if (raw_len > 0) {
                                uint64_t acc = 0;
                                for (size_t i = 0; i < raw_len && i < 8; i++) {
                                    acc |= ((uint64_t)raw[i]) << (8*i);
                                }
                                param_value = (float)acc / 10.0f;
                            }
                        }
                    }

                    if (param_cb) param_cb(cmd, param_id, param_value, raw, raw_len);

                    /* reset cmd frame state */
                    in_cmd_frame = false;
                    cmd_frame_pos = 0;
                    continue;
                }
            }
            continue;
        }

        /* If not in binary frame, handle as text line */
        if (b == '\r') continue;
        if (b == '\n') {
            line_buf[line_pos] = '\0';
            process_line(line_buf);
            line_pos = 0;
        } else {
            if (line_pos < sizeof(line_buf)-1) line_buf[line_pos++] = (char)b;
            else line_pos = 0; /* overflow - reset */
        }
    }
}

/* Build and send a framed command: header + len(2) + cmd(2) + data + footer */
static void send_command(uint16_t command, const uint8_t *data, size_t len)
{
    uint8_t frame[128];
    size_t pos = 0;

    /* header */
    memcpy(&frame[pos], FRAME_HEADER, sizeof(FRAME_HEADER));
    pos += sizeof(FRAME_HEADER);

    /* length = 2 (command) + data length */
    uint16_t total_len = (uint16_t)(2 + len);
    frame[pos++] = (uint8_t)(total_len & 0xFF);
    frame[pos++] = (uint8_t)((total_len >> 8) & 0xFF);

    /* command (little endian) */
    frame[pos++] = (uint8_t)(command & 0xFF);
    frame[pos++] = (uint8_t)((command >> 8) & 0xFF);

    /* data */
    if (data != NULL && len > 0) {
        memcpy(&frame[pos], data, len);
        pos += len;
    }

    /* footer */
    memcpy(&frame[pos], FRAME_FOOTER, sizeof(FRAME_FOOTER));
    pos += sizeof(FRAME_FOOTER);

    soft_uart_write_buf(frame, pos);
}

/* Read a framed response into buf, returns length or -1 on timeout */
static int read_response(uint8_t *out_buf, size_t out_size, uint32_t timeout_ms)
{
    uint32_t start = k_uptime_get_32();
    uint8_t temp[512];
    size_t tpos = 0;
    uint8_t header_match = 0;
    uint8_t footer_match = 0;

    while ((k_uptime_get_32() - start) < timeout_ms) {
        uint8_t c;
        if (!uart_read_byte(&c)) {
            k_msleep(1);
            continue;
        }

        /* Look for header first */
        if (header_match < sizeof(FRAME_HEADER)) {
            if (c == FRAME_HEADER[header_match]) {
                header_match++;
                temp[tpos++] = c;
                continue;
            } else {
                header_match = 0;
                tpos = 0;
                continue;
            }
        }

        /* After header, collect bytes until footer sequence seen */
        temp[tpos++] = c;

        /* Look for footer progressively */
        if (c == FRAME_FOOTER[footer_match]) {
            footer_match++;
            if (footer_match == sizeof(FRAME_FOOTER)) {
                /* Found full frame in temp */
                /* Remove header and footer for caller */
                size_t payload_start = sizeof(FRAME_HEADER);
                size_t payload_len = tpos - sizeof(FRAME_HEADER) - sizeof(FRAME_FOOTER);
                if (payload_len > out_size) payload_len = out_size;
                if (payload_len > 0) memcpy(out_buf, &temp[payload_start], payload_len);
                return (int)payload_len;
            }
        } else {
            footer_match = 0;
        }

        /* Prevent overflow */
        if (tpos >= sizeof(temp) - 1) {
            tpos = 0;
            header_match = 0;
            footer_match = 0;
        }
    }

    return -1; /* timeout */
}

/* Request a simple status/version check */
void ld2402_request_status(void)
{
    send_command(CMD_GET_VERSION, NULL, 0);
}

void ld2402_init(void)
{
    /* Ensure soft UART is initialized by caller (main currently does this).
     * Attempt to set the module into production/normal mode so it begins
     * normal output. This mirrors the HLK component behavior. */

    /* Small delay to allow UART to settle */
    k_msleep(50);

    /* Enable config mode (must be done before other commands) */
    uint8_t enable_config_data[2] = {0x01, 0x00};
    send_command(CMD_ENABLE_CONFIG, enable_config_data, sizeof(enable_config_data));
    k_msleep(20);

    /* Send set mode to production */
    /* Normal Working Mode =  (0x64)
        Engineering Mode = (0x04) */
    uint8_t mode_data[6] = {0x00, 0x00, 0x64, 0x00, 0x00, 0x00};
    send_command(CMD_SET_MODE, mode_data, sizeof(mode_data));
    /* Brief pause then request sensor parameter configuration (cmd 0x0008) */
    k_msleep(20);
    /* param_id 1 (0x01 0x00) = request full parameter configuration; payload empty */
    ld2402_send_param_frame(CMD_READ_PARAMS, 0x0001, NULL, 0);

    /* Disable config mode to resume normal working mode */
    k_msleep(20);
    send_command(CMD_DISABLE_CONFIG, NULL, 0);
    /* Do not block here for response; main can call ld2402_request_status or other ops */
}

/* Return last-read maximum distance (cm) from parameter responses, or -1.0 if unknown */
float ld2402_get_max_distance_cm(void)
{
    return module_max_distance_cm;
}
