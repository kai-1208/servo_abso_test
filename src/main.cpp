#include "mbed.h"
#include "c610.hpp"
#include "pid.hpp"

const int SERVO_POS_LOW = 80;
const int SERVO_POS_HIGH = 20;

BufferedSerial pc (USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, (int)1e6);
CAN can2(PB_12, PB_13, (int)1e6);
CANMessage msg1;
CANMessage msg2;

C610 mech_brushless(can2);

std::array<uint8_t, 8> servo = {0};  // 0: 右, 1: 中央, 2: 左

int32_t init_pos;
int32_t curr_pos_raw;
int64_t total_count = 0;
bool is_initialized = false;

DigitalIn limit_sw2(PC_11), limit_sw5(PC_1), button(BUTTON1);

// 角度型pid
PidParameter pos_param = {
    .gain = PidGain{.kp = 30.0f, .ki = 0.0f, .kd = 8.0f},
    .min = -3000.0f,
    .max = 3000.0f
};
Pid pos_pid(pos_param);

// 速度型pid
PidParameter vel_param = {
    .gain = PidGain{.kp = 8.0f, .ki = 0.0f, .kd = 0.0f},
    .min = -16000.0f,
    .max = 16000.0f
};
Pid vel_pid(vel_param);

int16_t power = 0;
float target_angle = 0.0f;
float curr_angle = 0.0f;

CANMessage msg3;

int main() {
    limit_sw2.mode(PullUp);
    limit_sw5.mode(PullUp);
    button.mode(PullUp);

    printf("Hello, World!\n");

    mbed::HighResClock::time_point now_timestamp = HighResClock::now ();
    mbed::HighResClock::time_point pos_loop_timestamp = HighResClock::now ();
    mbed::HighResClock::time_point vel_loop_timestamp = HighResClock::now ();

    float target_rpm = 0.0f;
    while(1) {
        now_timestamp = HighResClock::now ();
        // アブソ読み取り
        if (can1.read(msg2)) { // アブソはwhile直下で読みます
            if (msg2.id == 204) {
                float pos_dt = std::chrono::duration<float> (now_timestamp - pos_loop_timestamp).count ();
                pos_loop_timestamp = now_timestamp;
                // for (int i = 0; i < 2; i++) {
                //     int offset = i * 4;

                //     int32_t value =
                //         ((uint32_t)msg2.data[offset + 1]) |
                //         ((uint32_t)msg2.data[offset + 2] << 8) |
                //         ((uint32_t)msg2.data[offset + 3] << 16);

                //     if (value & 0x00800000) {
                //         value |= 0xFF000000;
                //     }

                //     if (!is_initialized) {
                //         init_pos = value;
                //         curr_pos = value;
                //         total_count = 0;
                //         is_initialized = true;
                //         // printf("init_pos: %d\n", init_pos);
                //     }

                //     int32_t delta = value - curr_pos;

                //     if (delta < -2048) {
                //         delta += 4096;
                //     } else if (delta > 2048) {
                //         delta -= 4096;
                //     }

                //     total_count += delta;
                //     curr_pos = value;

                //     curr_angle = (float)(total_count) * 360.0f / 4096.0f;

                //     // printf("ID:0x%02x data:%ld\n", (uint8_t)msg2.data[offset], value);
                //     printf("ID:0x%02x Raw:%ld Relative:%lld Curr_angle: %f\n", (uint8_t)msg2.data[offset], value, total_count, curr_angle);
                // }

                int offset = 0; 
                
                int32_t value =
                    ((uint32_t)msg2.data[offset + 1]) |
                    ((uint32_t)msg2.data[offset + 2] << 8) |
                    ((uint32_t)msg2.data[offset + 3] << 16);

                // 符号拡張 (24bit -> 32bit)
                if (value & 0x00800000) {
                    value |= 0xFF000000;
                }

                if (!is_initialized) {
                    curr_pos_raw = value;
                    total_count = 0; // 開始位置を0度とする
                    is_initialized = true;
                }

                // 差分計算
                int32_t delta = value - curr_pos_raw;
                // ロールオーバー処理 (分解能4096の場合)
                if (delta < -2048) delta += 4096;
                else if (delta > 2048) delta -= 4096;

                total_count += delta;
                curr_pos_raw = value;

                // 角度変換 (ギア比1:1なら4096, 4:1なら4096*4)
                curr_angle = (float)(total_count) * 360.0f / 4096.0f; 
                printf("ID:0x%02x Raw:%ld Relative:%lld Curr_angle: %f\n", (uint8_t)msg2.data[offset], value, total_count, curr_angle);

                // 角度PIDの計算 
                if (pos_dt > 0.0f) {
                    target_rpm = pos_pid.calc(target_angle, curr_angle, pos_dt);
                }
            }
        }

        float vel_dt = std::chrono::duration<float> (now_timestamp - vel_loop_timestamp).count ();
        if (vel_dt > 0.001f) {
            vel_loop_timestamp = now_timestamp;
            // printf("hello\n");
            //サーボ制御
            if (pc.readable()) {
                char buf[1];
                pc.read(buf, 1);

                // else if (buf[0] == '0') for (int i = 0; i < 3; i++) servo[i] = SERVO_POS_LOW;
                // printf("servo: %d, %d, %d\n", servo[0], servo[1], servo[2]);
                // CANMessage msg1(140, reinterpret_cast<const uint8_t *> (servo.data ()), 8);
                // can1.write (msg1);
                if (buf[0] == '1') target_angle = 90.0f;
                else if (buf[0] == '2') target_angle = 0.0f;
            }

            float curr_rpm = mech_brushless.get_rpm(2);
            float pid_output = vel_pid.calc(target_rpm, curr_rpm, vel_dt);

            if (limit_sw2.read() == 0 && pid_output < 0) pid_output = 0;
            if (limit_sw5.read() == 0 && pid_output > 0) pid_output = 0;

            // if (++print_count >= 100) {
            //     printf("curr_angle: %f target_angle: %f pid_output: %f\n", curr_angle, target_angle, pid_output);
            //     print_count = 0;
            // }
            mech_brushless.set_power(2, (int16_t)pid_output);
            mech_brushless.send_message();

            if (button.read() == 0) {
                if (servo[0] == SERVO_POS_LOW) {
                    for (int i = 0; i < 3; i++) servo[i] = SERVO_POS_HIGH;
                } else {
                    for (int i = 0; i < 3; i++) servo[i] = SERVO_POS_LOW;
                }
                // printf("servo: %d, %d, %d\n", servo[0], servo[1], servo[2]);
            }
            CANMessage msg1(140, reinterpret_cast<const uint8_t *> (servo.data ()), 8);
            can1.write (msg1);
        }

        // if (mechanism_loop_dt > 0.001f) {
        //     printf("curr_angle: %f\n", curr_angle);
        // }
    }
}