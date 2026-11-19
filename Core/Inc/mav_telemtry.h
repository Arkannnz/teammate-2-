/*
 * mav_telemetry.h  -  kirim data IMU sebagai MAVLink 2 lewat USB CDC (STM32 HAL)
 *
 * Pesan yang dikirim:
 *   HEARTBEAT          1 Hz   (supaya PC / Mission Planner tahu link hidup)
 *   NAMED_VALUE_FLOAT  "comp_roll", "comp_pitch"   (derajat, complementary filter)
 *   ATTITUDE           roll & pitch Kalman (dikirim dalam RADIAN sesuai standar MAVLink)
 *
 * Cara pasang:
 *   1. Salin isi repo c_library_v2 ke  Core/Inc/mavlink/   (harus ada Core/Inc/mavlink/common/mavlink.h)
 *   2. Salin file ini ke Core/Inc/
 *   3. Di main.c:   #include "mav_telemetry.h"
 *   4. Di while(1): panggil mav_send_imu(...) tiap 20 ms (50 Hz)
 *
 * Header ini harus di-include HANYA di satu file .c (main.c), karena fungsinya static.
 */
#ifndef MAV_TELEMETRY_H
#define MAV_TELEMETRY_H

#include <string.h>
#include "main.h"
#include "usbd_cdc_if.h"
#include "mavlink/common/mavlink.h"

#define MAV_SYS_ID    1
#define MAV_COMP_ID   MAV_COMP_ID_AUTOPILOT1
#define MAV_DEG2RAD   0.017453292519943295f

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Buffer harus static: CDC_Transmit_FS() TIDAK menyalin data, ia hanya menyimpan pointer
 * dan mengirimnya secara asinkron setelah fungsi kembali. */
static uint8_t  mav_tx_buf[512];
static uint16_t mav_tx_len;

static void mav_append(const mavlink_message_t *msg)
{
    if (mav_tx_len + MAVLINK_MAX_PACKET_LEN <= sizeof(mav_tx_buf)) {
        mav_tx_len += mavlink_msg_to_send_buffer(mav_tx_buf + mav_tx_len, msg);
    }
}

/* name maksimal 10 karakter (batas field NAMED_VALUE_FLOAT) */
static void mav_append_named(uint32_t ms, const char *name, float value)
{
    mavlink_message_t msg;
    char n[10] = {0};
    strncpy(n, name, sizeof(n));
    mavlink_msg_named_value_float_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, ms, n, value);
    mav_append(&msg);
}

/* Semua sudut dalam DERAJAT. Panggil tiap 20 ms (50 Hz). */
static void mav_send_imu(float comp_roll, float comp_pitch,
                         float kal_roll,  float kal_pitch)
{
    static uint32_t last_hb = 0;
    static uint8_t  hb_sent = 0;

    /* Kalau transfer USB sebelumnya belum selesai (atau PC belum membuka port),
     * lewati siklus ini. Jangan menunggu supaya loop utama tidak macet. */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }

    mavlink_message_t msg;
    uint32_t ms = HAL_GetTick();
    mav_tx_len = 0;

    if (!hb_sent || (ms - last_hb) >= 1000) {
        hb_sent = 1;
        last_hb = ms;
        mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                                   0 /* base_mode */, 0 /* custom_mode */,
                                   MAV_STATE_ACTIVE);
        mav_append(&msg);
    }

    /* NAMED_VALUE_FLOAT dulu, ATTITUDE terakhir (PC memakai ATTITUDE sebagai "pemicu" titik baru) */
    mav_append_named(ms, "comp_roll",  comp_roll);
    mav_append_named(ms, "comp_pitch", comp_pitch);

    mavlink_msg_attitude_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, ms,
                              kal_roll * MAV_DEG2RAD,    /* roll  (rad) */
                              kal_pitch * MAV_DEG2RAD,   /* pitch (rad) */
                              0.0f,                      /* yaw   (rad) - belum dipakai */
                              0.0f, 0.0f, 0.0f);         /* rollspeed, pitchspeed, yawspeed */
    mav_append(&msg);

    CDC_Transmit_FS(mav_tx_buf, mav_tx_len);
}

#endif /* MAV_TELEMETRY_H */
