#include "ControlTask.h"

#define WHEEL_RADIUS 0.035
LQRControlTask::LQRControlTask(IMUHandler *imu, MotorController *mL, MotorController *mR,
                               MotorEncoder *eL, MotorEncoder *eR, SharedDataManager *shared)
    : _imu(imu), _motorLeft(mL), _motorRight(mR), _encLeft(eL), _encRight(eR), _sharedData(shared) {}

void LQRControlTask::begin()
{
    xTaskCreatePinnedToCore(taskEntry, "LQR_Loop", 4096, this, 5, &_taskHandle, 0);
}

void LQRControlTask::taskEntry(void *pvParameters)
{
    LQRControlTask *instance = (LQRControlTask *)pvParameters;
    instance->controlLoop();
}

void LQRControlTask::controlLoop()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / _targetFreq);

    xLastWakeTime = xTaskGetTickCount();
    _lastMicros = micros(); // Inicializa contador

    for (;;)
    {
        // 1. Medição de tempo para cálculo da taxa real
        unsigned long currentMicros = micros();
        float loopDuration = (float)(currentMicros - _lastMicros);
        _lastMicros = currentMicros;

        // Evita divisão por zero na primeira iteração e converte para Hz
        float currentFreq = (loopDuration > 0) ? (1000000.0f / loopDuration) : 0.0f;

        // --- INICIO DO CONTROLE ---

        _imu->update();

        float angulo = _imu->getPitch() * (PI / 180.0);
        float velAngular = _imu->getGyroRate() * (PI / 180.0);
        float velLinear = ((_encLeft->getVelocityRad() + _encRight->getVelocityRad()) / 2.0) * WHEEL_RADIUS;

        angulo = angulo - 0.034;

        x[0] = angulo;
        x[1] = velAngular;
        x[2] = velLinear;
        x[3] = 0;

        float u = -(K[0] * x[0] + K[1] * x[1] + K[2] * x[2] + K[3] * x[3]);

        _motorLeft->write(u);
        _motorRight->write(u);

        // --- FIM DO CONTROLE ---

        // Envia dados + Frequência medida para o Core 1
        _sharedData->setState(angulo, velAngular, velLinear, u, currentFreq);

        // Bloqueia até completar o ciclo exato
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}