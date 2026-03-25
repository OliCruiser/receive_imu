#include "Arduino_BMI270_BMM150.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const uint8_t FRAME_HEADER_1 = 0xAA;
static const uint8_t FRAME_HEADER_2 = 0x55;
static const uint8_t FRAME_TAIL_1 = 0x55;
static const uint8_t FRAME_TAIL_2 = 0xAA;

static const int GYRO_CALIB_SAMPLES = 200;
static const int GYRO_CALIB_DELAY_MS = 20;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
bool gyroCalibrated = false;

uint8_t calcXorChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        chk ^= data[i];
    }
    return chk;
}

void sendFrame(uint32_t frame_id, const char *payload)
{
    uint8_t len = (uint8_t)strlen(payload);
    uint8_t chk = calcXorChecksum((const uint8_t *)payload, len);

    Serial1.write(FRAME_HEADER_1);
    Serial1.write(FRAME_HEADER_2);
    Serial1.write(len);
    Serial1.write((uint8_t *)&frame_id, 4);
    Serial1.write(payload, len);
    Serial1.write(chk);
    Serial1.write(FRAME_TAIL_1);
    Serial1.write(FRAME_TAIL_2);
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);

    delay(1500);

    if (!IMU.begin())
    {
        Serial.println("IMU init failed");
        while (1)
        {
        }
    }

    Serial.println("IMU init ok");
}

void loop()
{
    static uint32_t frame_id = 0;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    if (IMU.accelerationAvailable() &&
        IMU.gyroscopeAvailable() &&
        IMU.magneticFieldAvailable())
    {

        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);

        uint32_t timestamp = millis();

        char payload[160];
        snprintf(
            payload,
            sizeof(payload),
            "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            (unsigned long)frame_id,
            ax, ay, az,
            gx, gy, gz,
            mx, my, mz);

        sendFrame(frame_id++, payload);
        Serial.println(payload);
    }
    delay(200);
}