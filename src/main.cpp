#include "mbed.h"

const int SERVO_POS_LOW = 80;
const int SERVO_POS_HIGH = 20;

BufferedSerial pc (USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, (int)1e6);
CANMessage msg1;
CANMessage msg2;

std::array<uint8_t, 8> servo = {0};  // 0: 右, 1: 中央, 2: 左

int32_t init_pos;
int32_t curr_pos;
int64_t total_count = 0;
bool is_initialized = false;

int main() {

    // mbed::HighResClock::time_point now_timestamp = HighResClock::now ();
    // mbed::HighResClock::time_point mechanism_loop_timestamp = HighResClock::now ();
    while(1) {
        // now_timestamp = HighResClock::now ();
        // float mechanism_loop_dt = std::chrono::duration<float> (now_timestamp - mechanism_loop_timestamp).count ();

        // if (mechanism_loop_dt > 0.05f) {
        //     printf("hello\n");
        //     mechanism_loop_timestamp = now_timestamp;

            // if (pc.readable()) {
            //     char buf[1];
            //     pc.read(buf, 1);
            //     if (buf[0] == '1') servo[0] = SERVO_POS_HIGH;
            //     else if (buf[0] == '2') servo[0] = SERVO_POS_LOW;
            //     else if (buf[0] == '3') servo[1] = SERVO_POS_HIGH;
            //     else if (buf[0] == '4') servo[1] = SERVO_POS_LOW;
            //     else if (buf[0] == '5') servo[2] = SERVO_POS_HIGH;
            //     else if (buf[0] == '6') servo[2] = SERVO_POS_LOW;
            //     else if (buf[0] == '0') for (int i = 0; i < 3; i++) servo[i] = SERVO_POS_LOW;
            //     printf("servo: %d, %d, %d\n", servo[0], servo[1], servo[2]);
            //     CANMessage msg1(140, reinterpret_cast<const uint8_t *> (servo.data ()), 8);
            //     can1.write (msg1);
            // } else {
            //     // printf("omg\n");
            // }

        if (can1.read(msg2)) { // アブソはwhile直下で読みます
            if (msg2.id == 204) {
                for (int i = 0; i < 2; i++) {
                    int offset = i * 4;

                    int32_t value =
                        ((uint32_t)msg2.data[offset + 1]) |
                        ((uint32_t)msg2.data[offset + 2] << 8) |
                        ((uint32_t)msg2.data[offset + 3] << 16);

                    if (value & 0x00800000) {
                        value |= 0xFF000000;
                    }

                    if (!is_initialized) {
                        init_pos = value;
                        curr_pos = value;
                        total_count = 0;
                        is_initialized = true;
                        printf("init_pos: %d\n", init_pos);
                    }

                    int32_t delta = value - curr_pos;

                    if (delta < -2048) {
                        delta += 4096;
                    } else if (delta > 2048) {
                        delta -= 4096;
                    }

                    total_count += delta;
                    curr_pos = value;

                    // printf("ID:0x%02x data:%ld\n", (uint8_t)msg2.data[offset], value);
                    printf("ID:0x%02x Raw:%ld Relative:%lld\n", (uint8_t)msg2.data[offset], value, total_count);
                }
            }
        }
        // }
    }
}