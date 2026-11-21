#ifndef MAV_TELEMETRY_H
#define MAV_TELEMETRY_H

#include <string.h>
#include "main.h"
#include "common/mavlink.h"

/* ==========================================================================
 * PILIHAN JALUR TELEMETRI MAVLINK:
 * 0 = Via USB CDC (kabel USB langsung ke laptop)
 * 1 = Via USART1 PA9/PA10 (modul radio telemetri SiK / LoRa / ESP)
 * ========================================================================== */
#define MAV_OVER_UART    1

#if MAV_OVER_UART
extern UART_HandleTypeDef huart1;
#else
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

#define MAV_SYS_ID       1
#define MAV_COMP_ID      MAV_COMP_ID_AUTOPILOT1
#define MAV_DEG2RAD      0.017453292519943295f
#define MAV_RX_BUF_SIZE  256
#define MAV_MAX_PARAMS   12

/* Struktur registrasi parameter MAVLink */
typedef struct {
    char   name[16];
    float *ptr;
} mav_param_entry_t;

static mav_param_entry_t mav_params[MAV_MAX_PARAMS];
static uint16_t mav_param_count   = 0;
static uint16_t param_stream_idx  = 0xFFFF; // 0xFFFF = tidak sedang mengirim daftar penuh
static uint16_t param_single_mask = 0;      // Bitmask untuk balasan PARAM_SET / REQUEST_READ

/* Buffer TX & Ring Buffer RX */
static uint8_t           mav_tx_buf[512];
static uint16_t          mav_tx_len = 0;

static uint8_t           mav_rx_ring[MAV_RX_BUF_SIZE];
static volatile uint16_t mav_rx_head = 0;
static volatile uint16_t mav_rx_tail = 0;

#if MAV_OVER_UART
static uint8_t mav_uart_rx_byte = 0;
#endif

/* Push byte masuk ke ring buffer (aman dipanggil dari ISR USB maupun UART) */
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

#if MAV_OVER_UART
/* Mulai interrupt RX 1 byte pada USART1 (dipanggil di USER CODE BEGIN 2) */
static inline void mav_uart_start_rx(void)
{
    /* Bersihkan error flag sisa startup sebelum mulai menerima interrupt */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    HAL_UART_Receive_IT(&huart1, &mav_uart_rx_byte, 1);
}

/* Callback interrupt RX USART1 (dipanggil di HAL_UART_RxCpltCallback) */
static inline void mav_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        mav_rx_push(&mav_uart_rx_byte, 1);
        HAL_UART_Receive_IT(&huart1, &mav_uart_rx_byte, 1);
    }
}

/* Bersihkan error flag USART1 agar RX tidak macet terkena noise radio */
static inline void mav_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &mav_uart_rx_byte, 1);
    }
}
#endif

/* Daftarkan variabel float ke tabel parameter Mission Planner / QGC */
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
    mavlink_msg_param_value_pack(
        MAV_SYS_ID, MAV_COMP_ID, &msg,
        mav_params[idx].name,
        *(mav_params[idx].ptr),
        MAV_PARAM_TYPE_REAL32,
        mav_param_count,
        idx
    );
    mav_append(&msg);
}

/* Proses seluruh byte perintah masuk dari GCS */
static void mav_process_rx(void)
{
    static mavlink_message_t rx_msg;
    static mavlink_status_t  rx_status;

    while (mav_rx_tail != mav_rx_head) {
        uint8_t byte = mav_rx_ring[mav_rx_tail];
        mav_rx_tail = (mav_rx_tail + 1U) % MAV_RX_BUF_SIZE;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
            switch (rx_msg.msgid) {
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    param_stream_idx = 0;
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                    mavlink_param_request_read_t req;
                    mavlink_msg_param_request_read_decode(&rx_msg, &req);
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
                    break;
                }

                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t set;
                    mavlink_msg_param_set_decode(&rx_msg, &set);
                    for (uint16_t i = 0; i < mav_param_count; i++) {
                        if (strncmp(set.param_id, mav_params[i].name, 16) == 0) {
                            *(mav_params[i].ptr) = set.param_value;
                            param_single_mask |= (1U << i);
                            break;
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
static void mav_telemetry_update(float roll_deg,     float pitch_deg,
                                 float pid_roll_out, float pid_pitch_out,
                                 float yaw_deg,      float altitude_m)
{
    static uint32_t last_hb    = 0;
    static uint32_t last_imu   = 0;
    static uint32_t last_named = 0;
    static uint8_t  hb_sent    = 0;

    // 1. Selalu baca paket perintah masuk dari GCS
    mav_process_rx();

    // 2. Cek apakah antarmuka TX (UART1 atau USB CDC) siap mengirim (non-blocking)
#if MAV_OVER_UART
    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }
#else
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }
#endif

    uint32_t ms = HAL_GetTick();
    mav_tx_len = 0;

    // 3. Heartbeat 1 Hz (Prioritas utama agar Mission Planner langsung konek)
    if (!hb_sent || (ms - last_hb) >= 1000) {
        hb_sent = 1;
        last_hb = ms;
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC,
                                   MAV_MODE_FLAG_STABILIZE_ENABLED, 0, MAV_STATE_ACTIVE);
        mav_append(&msg);
    }

    // 4. Data Attitude & VFR_HUD pada 20 Hz (tiap 50 ms - aman untuk radio 57600 bps)
    if ((ms - last_imu) >= 50) {
        last_imu = ms;
        mavlink_message_t msg;
        mavlink_msg_attitude_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, ms,
                                  roll_deg  * MAV_DEG2RAD,
                                  pitch_deg * MAV_DEG2RAD,
                                  yaw_deg   * MAV_DEG2RAD,
                                  0.0f, 0.0f, 0.0f);
        mav_append(&msg);

        int16_t heading = ((int16_t)yaw_deg % 360 + 360) % 360;
        mavlink_msg_vfr_hud_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                 0.0f, 0.0f, heading, 0, altitude_m, 0.0f);
        mav_append(&msg);
    }

    // 5. Data Named Value (Sudut, PID, Altitude) pada 10 Hz (tiap 100 ms agar tidak membanjiri radio)
    if ((ms - last_named) >= 100) {
        last_named = ms;
        mav_append_named(ms, "ROLL_DEG",  roll_deg);
        mav_append_named(ms, "PITCH_DEG", pitch_deg);
        mav_append_named(ms, "PID_ROLL",  pid_roll_out);
        mav_append_named(ms, "PID_PITCH", pid_pitch_out);
        mav_append_named(ms, "BMP_ALT",   altitude_m);
    }

    // 6. Kirim balasan satuan (hasil PARAM_SET / PARAM_REQUEST_READ)
    for (uint16_t i = 0; i < mav_param_count; i++) {
        if (param_single_mask & (1U << i)) {
            if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
                mav_append_param_value(i);
                param_single_mask &= ~(1U << i);
            }
        }
    }

    // 7. Kirim 1 parameter per siklus saat diminta GCS (PARAM_REQUEST_LIST) agar tidak meluapkan buffer radio
    if (param_stream_idx < mav_param_count) {
        if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
            mav_append_param_value(param_stream_idx);
            param_stream_idx++;
        }
    }

    // 8. Kirim seluruh paket lewat jalur yang dipilih
    if (mav_tx_len > 0) {
#if MAV_OVER_UART
        HAL_UART_Transmit_IT(&huart1, mav_tx_buf, mav_tx_len);
#else
        CDC_Transmit_FS(mav_tx_buf, mav_tx_len);
#endif
    }
}

#endif /* MAV_TELEMETRY_H */
