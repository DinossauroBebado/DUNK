#include "MotorController.h"

MotorController::MotorController(uint8_t pinIN1, uint8_t pinIN2, uint8_t channel1, uint8_t channel2)
{
    _pinIN1 = pinIN1;
    _pinIN2 = pinIN2;
    _pwmChannel1 = channel1;
    _pwmChannel2 = channel2;
}

void MotorController::begin()
{
    // Configura os pinos como saída
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);

    // Configura o periférico LEDC (PWM) do ESP32
    // Nota: Essa sintaxe é para o ESP32 Arduino Core v2.x (Padrão atual do PlatformIO)
    ledcSetup(_pwmChannel1, _pwmFreq, _pwmResolution);
    ledcSetup(_pwmChannel2, _pwmFreq, _pwmResolution);

    // Atrela os canais aos pinos
    ledcAttachPin(_pinIN1, _pwmChannel1);
    ledcAttachPin(_pinIN2, _pwmChannel2);
}

int MotorController::calculateDutyCycle(int speed)
{
    // Garante que o valor absoluto da velocidade não exceda 100
    int absSpeed = abs(speed);
    if (absSpeed > 100)
        absSpeed = 100;

    // Mapeia 0-100% para 0-255 (8 bits)
    return map(absSpeed, 0, 100, 0, 255);
}

void MotorController::write(int speed)
{
    int dutyCycle = calculateDutyCycle(speed);

    if (speed > 0)
    {
        // Sentido Horário (Frente)
        ledcWrite(_pwmChannel1, dutyCycle);
        ledcWrite(_pwmChannel2, 0);
    }
    else if (speed < 0)
    {
        // Sentido Anti-horário (Trás)
        ledcWrite(_pwmChannel1, 0);
        ledcWrite(_pwmChannel2, dutyCycle);
    }
    else
    {
        // Parado (Inércia/Freio suave)
        stop();
    }
}

void MotorController::stop()
{
    ledcWrite(_pwmChannel1, 0);
    ledcWrite(_pwmChannel2, 0);
}