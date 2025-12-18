/*
  MotorEncoder.h - Biblioteca para leitura de Encoder Quadrature no ESP32
  Retorna velocidade em rad/s e RPM.
*/

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

class MotorEncoder
{
public:
    // Construtor: pinA, pinB, pulsos por revolução (PPR), inversão de sentido
    MotorEncoder(int pinA, int pinB, int ppr, bool reverse = false);

    // Configura os pinos (chamar no setup)
    void begin();

    // Deve ser chamada dentro da ISR (Interrupção) no main
    void IRAM_ATTR update();

    // Retorna a velocidade angular em radianos/segundo
    float getVelocityRad();

    // Retorna a velocidade em RPM
    float getRPM();

    // Retorna o total de pulsos acumulados
    int64_t getPulses();

    // Imprime informações de debug no Serial
    void debug();

private:
    int _pinA;
    int _pinB;
    int _ppr;
    bool _reverse;

    // Variáveis voláteis para acesso dentro da interrupção
    volatile int64_t _pulseCount;

    // Variáveis para cálculo de velocidade
    unsigned long _lastTimeMicros;
    int64_t _lastPulseCount;
    float _currentVelocityRad;
    float _currentRPM;

    // Intervalo mínimo para recálculo de velocidade (evita ruído)
    const unsigned long _sampleTimeMicros = 10000; // 20ms
};

#endif