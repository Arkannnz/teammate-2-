import math
import time
import serial

# Ganti 'COMx' sesuai nomor port UART pada mini debugger di Device Manager
SERIAL_PORT = "COM7"
BAUD_RATE = 100000


def build_sbus_frame(channels, frame_lost=False, failsafe=False):
    """Mengemas 16 kanal (nilai 172 - 1811) menjadi paket SBUS 25-byte."""
    # Pastikan selalu ada 16 kanal dalam rentang valid SBUS (0 - 2047)
    ch = [max(0, min(2047, int(c))) for c in channels[:16]]
    while len(ch) < 16:
        ch.append(992)

    frame = bytearray(25)
    frame[0] = 0x0F  # SBUS_HEADER

    # Bit-packing 16 kanal x 11-bit ke dalam 22 byte (frame[1] .. frame[22])
    bit_buf = 0
    bit_count = 0
    byte_idx = 1

    for val in ch:
        bit_buf |= (val & 0x07FF) << bit_count
        bit_count += 11
        while bit_count >= 8:
            frame[byte_idx] = bit_buf & 0xFF
            byte_idx += 1
            bit_buf >>= 8
            bit_count -= 8

    # Byte 23: Flag digital & status failsafe
    flags = 0x00
    if frame_lost:
        flags |= 1 << 2
    if failsafe:
        flags |= 1 << 3
    frame[23] = flags

    frame[24] = 0x00  # SBUS_FOOTER
    return frame


def main():
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.1,
        )
    except Exception as e:
        print(f"Gagal membuka {SERIAL_PORT}: {e}")
        return

    print(f"Mengirim sinyal SBUS ke {SERIAL_PORT} (100000 8E2)... Tekan Ctrl+C untuk berhenti.")
    t0 = time.time()

    try:
        while True:
            t = time.time() - t0

            # Simulasi gerakan stik (rentang SBUS: 172 min, 992 tengah, 1811 max)
            ch1_roll = int(992 + 500 * math.sin(2 * math.pi * 0.5 * t))   # Ayun kiri-kanan 0.5 Hz
            ch2_pitch = int(992 + 400 * math.cos(2 * math.pi * 0.5 * t))  # Ayun atas-bawah 0.5 Hz
            ch3_thr = 172                                                 # Throttle minimum
            ch4_yaw = 992                                                 # Rudder tengah

            # Channel 5 (indeks 4): Sakelar mode terbang
            # <= 1500 = Mode Manual, > 1500 = Mode Stabilize PID
            # Berganti otomatis setiap 5 detik untuk menguji kedua mode
            ch5_mode = 1800 if (int(t) % 10) >= 5 else 200

            channels = [ch1_roll, ch2_pitch, ch3_thr, ch4_yaw, ch5_mode] + [992] * 11
            packet = build_sbus_frame(channels, frame_lost=False, failsafe=False)

            ser.write(packet)
            mode_str = "STABILIZE" if ch5_mode > 1500 else "MANUAL"
            print(
                f"\rCH1(Roll): {ch1_roll:4d} | CH2(Pitch): {ch2_pitch:4d} | "
                f"CH5: {ch5_mode:4d} ({mode_str})",
                end="",
            )

            # Kirim setiap 14 ms (memberi jeda > 3 ms antar frame sesuai s.bus.c)
            time.sleep(0.014)

    except KeyboardInterrupt:
        print("\nSimulasi dihentikan (menguji timeout/failsafe di STM32).")
    finally:
        ser.close()


if __name__ == "__main__":
    main()