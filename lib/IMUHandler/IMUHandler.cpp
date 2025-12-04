#include "IMUHandler.h"

IMUHandler::IMUHandler(uint8_t addr) : _mpu(addr)
{
    _pitchFiltered = 0.0;
    _pitchRaw = 0.0;
    _gyroRateVal = 0.0;
    _lastTime = 0;
}

void IMUHandler::initKalman()
{
    _kalman.Q_angle = 0.001;
    _kalman.Q_bias = 0.003;
    _kalman.R_measure = 0.03;

    _kalman.angle = 0;
    _kalman.bias = 0;

    _kalman.P[0][0] = 0;
    _kalman.P[0][1] = 0;
    _kalman.P[1][0] = 0;
    _kalman.P[1][1] = 0;
}

bool IMUHandler::begin()
{
    // Inicializa Kalman
    initKalman();

    // Inicializa MPU
    if (!_mpu.init())
    {
        return false;
    }

    // Configurações e Calibração
    Serial.println("[IMU] Calibrando offsets... mantenha parado.");
    _mpu.autoOffsets();

    _mpu.enableGyrDLPF();
    _mpu.setGyrDLPF(MPU6500_DLPF_6);
    _mpu.setSampleRateDivider(5);
    _mpu.setGyrRange(MPU6500_GYRO_RANGE_250);
    _mpu.setAccRange(MPU6500_ACC_RANGE_2G);
    _mpu.enableAccDLPF(true);
    _mpu.setAccDLPF(MPU6500_DLPF_6);

    delay(100);
    _lastTime = micros(); // Reseta o timer para evitar dt gigante na primeira leitura

    return true;
}

void IMUHandler::update()
{
    // 1. Calcular DT (Delta Tempo)
    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0;
    _lastTime = now;

    // Proteção contra dt inválido (primeira iteração ou overflow)
    if (dt <= 0 || dt > 1.0)
        return;

    // 2. Leitura dos Sensores
    xyzFloat acc = _mpu.getGValues();
    xyzFloat gyr = _mpu.getGyrValues();

    // 3. Cálculo do Pitch pelo Acelerômetro (Bruto)
    // atan2 retorna radianos, convertemos para graus
    float accelPitch = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0 / PI;
    _pitchRaw = accelPitch;

    // 4. Taxa do Giroscópio
    // A biblioteca MPU6500_WE já retorna em graus/s
    _gyroRateVal = gyr.y;

    // 5. Filtro de Kalman
    _pitchFiltered = runKalmanMath(accelPitch, _gyroRateVal, dt);
}

float IMUHandler::runKalmanMath(float newAngle, float newRate, float dt)
{
    // Predição
    _kalman.rate = newRate - _kalman.bias;
    _kalman.angle += dt * _kalman.rate;

    _kalman.P[0][0] += dt * (dt * _kalman.P[1][1] - _kalman.P[0][1] - _kalman.P[1][0] + _kalman.Q_angle);
    _kalman.P[0][1] -= dt * _kalman.P[1][1];
    _kalman.P[1][0] -= dt * _kalman.P[1][1];
    _kalman.P[1][1] += _kalman.Q_bias * dt;

    // Atualização
    float S = _kalman.P[0][0] + _kalman.R_measure;

    float K_gain[2]; // Ganho de Kalman
    K_gain[0] = _kalman.P[0][0] / S;
    K_gain[1] = _kalman.P[1][0] / S;

    float y = newAngle - _kalman.angle; // Inovação (Erro)
    _kalman.angle += K_gain[0] * y;
    _kalman.bias += K_gain[1] * y;

    // Atualização da Covariância
    float P00_temp = _kalman.P[0][0];
    float P01_temp = _kalman.P[0][1];

    _kalman.P[0][0] -= K_gain[0] * P00_temp;
    _kalman.P[0][1] -= K_gain[0] * P01_temp;
    _kalman.P[1][0] -= K_gain[1] * P00_temp;
    _kalman.P[1][1] -= K_gain[1] * P01_temp;

    return _kalman.angle;
}

float IMUHandler::getPitch()
{
    return _pitchFiltered;
}

float IMUHandler::getRawPitch()
{
    return _pitchRaw;
}
float IMUHandler::getGyroRate()
{
    return _gyroRateVal;
}