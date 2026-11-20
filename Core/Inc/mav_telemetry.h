#ifndef MAV_TELEMETRY_H
#define MAV_TELEMETRY_H

#include <string.h>
#include "main.h"
#include "usbd_cdc_if.h"
#include "mavlink/common/mavlink.h"

#define MAV_SYS_ID       1
#define MAV_COMP_ID      MAV_COMP_ID_AUTOPILOT1
#define MAV_DEG2RAD      0.017453292519943295f
#define MAV_RX_BUF_SIZE  256
#define MAV_MAX_PARAMS   12

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Struktur registrasi parameter MAVLink */
typedef struct {
    char   name[16];
    float *ptr;
} mav_param_entry_t;

static mav_param_entry_t mav_params[MAV_MAX_PARAMS];
static uint16_t mav_param_count  = 0;
static uint16_t param_stream_idx = 0xFFFF; // 0xFFFF = tidak sedang mengirim daftar penuh
static uint16_t param_single_mask = 0;     // Bitmask untuk balasan PARAM_SET / REQUEST_READ

/* Buffer TX & Ring Buffer RX (aman dari interrupt USB) */
static uint8_t  mav_tx_buf[512];
static uint16_t mav_tx_len = 0;

static uint8_t           mav_rx_ring[MAV_RX_BUF_SIZE];
static volatile uint16_t mav_rx_head = 0;
static volatile uint16_t mav_rx_tail = 0;

/* Dipanggil dari CDC_Receive_FS (ISR) di usbd_cdc_if.c */
void mav_rx_push(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (mav_rx_head + 1U) % MAV_RX_BUF_SIZE;
        if (next != mav_rx_tail) {
            mav_rx_ring[mav_rx_head] = buf[i];
            mav_rx_head = next;
        }
    }
}

/* Daftarkan variabel float ke tabel parameter QGroundControl */
static void mav_register_param(const char *name, float *var_ptr)
{
    if (mav_param_count < MAV_MAX_PARAMS) {
        memset(mav_params[mav_param_count].name, 0, 16);
        strncpy(mav_params[mav_param_count].name, name, 15);
        mav_params[mav_param_count].ptr = var_ptr;
        mav_param_count++;
    }
}

static void mav_append(const mavlink_message_t *msg)
{
    if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
        mav_tx_len += mavlink_msg_to_send_buffer(mav_tx_buf + mav_tx_len, msg);
    }
}

static void mav_append_named(uint32_t ms, const char *name, float value)
{
    mavlink_message_t msg;
    char n[10] = {0};
    strncpy(n, name, sizeof(n));
    mavlink_msg_named_value_float_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, ms, n, value);
    mav_append(&msg);
}

static void mav_append_param_value(uint16_t idx)
{
    if (idx >= mav_param_count) return;
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                 mav_params[idx].name,
                                 *(mav_params[idx].ptr),
                                 MAV_PARAM_TYPE_REAL32,
                                 mav_param_count,
                                 idx);
    mav_append(&msg);
}

/* Proses seluruh byte masuk dari QGroundControl */
static void mav_process_rx(void)
{
    static mavlink_message_t rx_msg;
    static mavlink_status_t  rx_status;

    while (mav_rx_tail != mav_rx_head) {
        uint8_t c = mav_rx_ring[mav_rx_tail];
        mav_rx_tail = (mav_rx_tail + 1U) % MAV_RX_BUF_SIZE;

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &rx_status)) {
            switch (rx_msg.msgid) {
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    mavlink_param_request_list_t req;
                    mavlink_msg_param_request_list_decode(&rx_msg, &req);
                    if (req.target_system == MAV_SYS_ID || req.target_system == 0) {
                        param_stream_idx = 0; // Mulai kirim semua parameter
                    }
                    break;
                }

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                    mavlink_param_request_read_t req;
                    mavlink_msg_param_request_read_decode(&rx_msg, &req);
                    if (req.target_system == MAV_SYS_ID || req.target_system == 0) {
                        if (req.param_index >= 0 && req.param_index < (int16_t)mav_param_count) {
                            param_single_mask |= (1U << req.param_index);
                        } else {
                            for (uint16_t i = 0; i < mav_param_count; i++) {
                                if (strncmp(req.param_id, mav_params[i].name, 16) == 0) {
                                    param_single_mask |= (1U << i);
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }

                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t set;
                    mavlink_msg_param_set_decode(&rx_msg, &set);
                    if (set.target_system == MAV_SYS_ID || set.target_system == 0) {
                        for (uint16_t i = 0; i < mav_param_count; i++) {
                            if (strncmp(set.param_id, mav_params[i].name, 16) == 0) {
                                *(mav_params[i].ptr) = set.param_value; // Update PID langsung!
                                param_single_mask |= (1U << i);         // Kirim konfirmasi ke QGC
                                break;
                            }
                        }
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }
}

/* Panggil di setiap iterasi while(1) pada main.c */
static void mav_telemetry_update(float comp_roll, float comp_pitch,
                                 float kal_roll,  float kal_pitch)
{
    static uint32_t last_hb  = 0;
    static uint32_t last_imu = 0;
    static uint8_t  hb_sent  = 0;

    // 1. Selalu baca paket perintah masuk dari QGC tanpa menunggu TX siap
    mav_process_rx();

    // 2. Cek apakah USB CDC siap mengirim data
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }

    uint32_t ms = HAL_GetTick();
    mav_tx_len = 0;

    // 3. Heartbeat 1 Hz
    if (!hb_sent || (ms - last_hb) >= 1000) {
        hb_sent = 1;
        last_hb = ms;
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC,
                                   0, 0, MAV_STATE_ACTIVE);
        mav_append(&msg);
    }

    // 4. Data Attitude & Named Value 50 Hz (tiap 20 ms)
    if ((ms - last_imu) >= 20) {
        last_imu = ms;
        mav_append_named(ms, "comp_roll",  comp_roll);
        mav_append_named(ms, "comp_pitch", comp_pitch);

        mavlink_message_t msg;
        mavlink_msg_attitude_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, ms,
                                  kal_roll * MAV_DEG2RAD,
                                  kal_pitch * MAV_DEG2RAD,
                                  0.0f, 0.0f, 0.0f, 0.0f);
        mav_append(&msg);
    }

    // 5. Kirim balasan satuan (hasil PARAM_SET / PARAM_REQUEST_READ)
    for (uint16_t i = 0; i < mav_param_count; i++) {
        if (param_single_mask & (1U << i)) {
            if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
                mav_append_param_value(i);
                param_single_mask &= ~(1U << i);
            }
        }
    }

    // 6. Kirim daftar seluruh parameter saat diminta QGC (PARAM_REQUEST_LIST)
    while (param_stream_idx < mav_param_count) {
        if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
            mav_append_param_value(param_stream_idx);
            param_stream_idx++;
        } else {
            break; // Lanjutkan sisa parameter pada siklus berikutnya jika buffer penuh
        }
    }

    // 7. Kirim seluruh paket yang terkumpul lewat USB CDC
    if (mav_tx_len > 0) {
        CDC_Transmit_FS(mav_tx_buf, mav_tx_len);
    }
}

#endif /* MAV_TELEMETRY_H */
