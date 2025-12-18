#ifndef LQR_CONTROL_TASK_H
#define LQR_CONTROL_TASK_H

#include <Arduino.h>
#include "MotorController.h"
#include "MotorEncoder.h"
#include "IMUHandler.h"
#include "config.h"
#include "SharedData.h"

class LQRControlTask
{
private:
    IMUHandler *_imu;
    MotorController *_motorLeft;
    MotorController *_motorRight;
    MotorEncoder *_encLeft;
    MotorEncoder *_encRight;
    SharedDataManager *_sharedData;

    float K[4] = {197.362, 46.47, -36.66, -36.21};
    float x[4] = {0, 0, 0, 0};

    TaskHandle_t _taskHandle;
    const int _targetFreq = 100; // 100Hz Alvo

    // Variável para cálculo de frequência
    unsigned long _lastMicros = 0;

    static void taskEntry(void *pvParameters);
    void controlLoop();

public:
    LQRControlTask(IMUHandler *imu,
                   MotorController *mL, MotorController *mR,
                   MotorEncoder *eL, MotorEncoder *eR,
                   SharedDataManager *shared);

    void begin();
};

#endif