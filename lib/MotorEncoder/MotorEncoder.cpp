#include "MotorEncoder.h"

MotorEncoder::MotorEncoder(int pinA, int pinB, int ppr, bool reverse)
{
    _pinA = pinA;
    _pinB = pinB;
    _ppr = ppr;
    _reverse = reverse;
    _pulseCount = 0;
    _lastPulseCount = 0;
    _lastTimeMicros = 0;
    _currentVelocityRad = 0.0;
    _currentRPM = 0.0;
}

void MotorEncoder::begin()
{
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _lastTimeMicros = micros();
}

// Lógica otimizada para ser chamada dentro da interrupção
void IRAM_ATTR MotorEncoder::update()
{
    // Leitura rápida dos pinos
    int msb = digitalRead(_pinA);
    int lsb = digitalRead(_pinB);

    int encoded = (msb << 1) | lsb;
    int sum = 0;

    // Algoritmo simples de quadratura baseada em estado anterior poderia ser usado,
    // mas aqui simplificamos verificando a transição do pino A.
    // Se B for diferente de A na borda, direção é positiva, senão negativa.

    if (msb == lsb)
    {
        sum = _reverse ? 1 : -1;
    }
    else
    {
        sum = _reverse ? -1 : 1;
    }

    _pulseCount += sum;
}

int64_t MotorEncoder::getPulses()
{
    // Operação atômica recomendada para leitura de int64 em sistema 32bits,
    // mas para fins simples, leitura direta funciona na maioria dos casos.
    noInterrupts();
    int64_t temp = _pulseCount;
    interrupts();
    return temp;
}

void MotorEncoder::debug()
{
    // Garante que os valores de velocidade estejam atualizados
    getVelocityRad();

    Serial.print("PULSOS: ");
    Serial.print(getPulses());
    Serial.print(" \t| RPM: ");
    Serial.print(_currentRPM);
    Serial.print(" \t| RAD/S: ");
    Serial.println(_currentVelocityRad);
}

// Função principal de cálculo
float MotorEncoder::getVelocityRad()
{
    unsigned long now = micros();
    unsigned long dt = now - _lastTimeMicros;

    // Só recalcula se passou o tempo de amostragem (ex: 20ms)
    if (dt >= _sampleTimeMicros)
    {

        // Leitura snapshot segura dos pulsos
        noInterrupts();
        int64_t currentPulses = _pulseCount;
        interrupts();

        long dPulses = currentPulses - _lastPulseCount;

        // Cálculo em rad/s
        // (DeltaPulsos / PPR) * 2PI * (1.000.000 / DeltaTempoMicros)
        double revolutions = (double)dPulses / (double)_ppr;
        double timeSeconds = (double)dt / 1000000.0;

        _currentVelocityRad = (revolutions * 2.0 * PI) / timeSeconds;

        // Cálculo RPM
        _currentRPM = (revolutions * 60.0) / timeSeconds;

        // Atualiza variaveis passadas
        _lastPulseCount = currentPulses;
        _lastTimeMicros = now;
    }
    else if (now - _lastTimeMicros > 500000)
    {
        // Timeout: Se passar 0.5s sem chamar ou sem pulso, zera a velocidade
        _currentVelocityRad = 0;
        _currentRPM = 0;
    }

    return _currentVelocityRad;
}

float MotorEncoder::getRPM()
{
    getVelocityRad(); // Força atualização
    return _currentRPM;
}