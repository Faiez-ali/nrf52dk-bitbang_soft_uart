#ifndef LD2402_H
#define LD2402_H

#include <stdint.h>
#include <stddef.h>

void ld2402_init(void);
void ld2402_uart_rx(uint8_t byte);

/* Optional TX commands */
void ld2402_request_status(void);

/* Processing loop - call regularly from main() to parse incoming data */
void ld2402_process(void);

/* Parsed report from the sensor */
typedef struct {
	uint32_t timestamp_ms; /* k_uptime_get_32() when parsed */
	uint8_t frame_type;    /* DATA_FRAME_TYPE_* or 0 for text lines */
	uint16_t data_len;     /* payload length for binary frames */

	/* Primary interpreted distance in cm (0 if not present) */
	float distance_cm;

	/* Detection flag parsed from binary frames (non-zero means presence detected) */
	uint8_t detection_result;

	/* Target distance reported in binary frames (cm) */
	float target_distance_cm;

	/* Parsed distances (cm) - up to 16 values found in binary payload */
	size_t distances_count;
	float distances[16];

	/* Raw payload (first 128 bytes) for additional parsing/debug */
	uint8_t raw_payload[128];
	size_t raw_payload_len;

	/* Sports/energy values parsed from engineering frames: up to 32 samples */
	size_t energy_values_count;
	float energy_values[32];
} ld2402_report_t;

typedef void (*ld2402_report_cb_t)(const ld2402_report_t *report);
void ld2402_set_report_callback(ld2402_report_cb_t cb);

/* Parameter frame callback: called when a parameter response is parsed.
 * cmd: command word
 * param_id: parameter id (if present, else 0)
 * value: converted value (param_decimal / 10.0)
 * raw: raw parameter bytes
 * raw_len: length of raw parameter bytes */
typedef void (*ld2402_param_cb_t)(uint16_t cmd, uint16_t param_id, float value, const uint8_t *raw, size_t raw_len);
void ld2402_set_param_callback(ld2402_param_cb_t cb);

/* Send a parameter/configuration frame: header + len(2) + cmd(2) + param_id(2) + payload + footer */
void ld2402_send_param_frame(uint16_t cmd, uint16_t param_id, const uint8_t *payload, size_t payload_len);

/* Get last-read maximum distance (cm) from parameter responses, or -1.0 if unknown */
float ld2402_get_max_distance_cm(void);

#endif /* LD2402_H */