#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController
{
private:
    // Pinos físicos conectados à Ponte H
    uint8_t _pinIN1;
    uint8_t _pinIN2;

    // Canais PWM do ESP32 (0-15)
    uint8_t _pwmChannel1;
    uint8_t _pwmChannel2;

    // Configurações do PWM
    const int _pwmFreq = 1000;     // 30kHz (frequência inaudível para a maioria dos motores)
    const int _pwmResolution = 10; // Resolução de 8 bits (0-255)

    /**
     * @brief Converte a porcentagem de velocidade (-100 a 100) para Duty Cycle PWM.
     * @param speed Velocidade em porcentagem.
     * @return Valor do Duty Cycle (0-255).
     */
    int calculateDutyCycle(int speed);

public:
    /**
     * @brief Construtor da classe MotorController.
     * * @param pinIN1 Pino conectado à entrada 1 da Ponte H.
     * @param pinIN2 Pino conectado à entrada 2 da Ponte H.
     * @param channel1 Canal PWM para o pino 1 (Padrão: 0).
     * @param channel2 Canal PWM para o pino 2 (Padrão: 1).
     */
    MotorController(uint8_t pinIN1, uint8_t pinIN2, uint8_t channel1 = 0, uint8_t channel2 = 1);

    /**
     * @brief Inicializa os pinos e configura o periférico LEDC (PWM).
     * Deve ser chamado no setup().
     */
    void begin();

    /**
     * @brief Controla a direção e velocidade do motor.
     * * @param speed Velocidade desejada de -100 (máx trás) a 100 (máx frente). 0 para o motor.
     */
    void write(int speed);

    /**
     * @brief Para o motor imediatamente (freio passivo).
     */
    void stop();
};

#endif