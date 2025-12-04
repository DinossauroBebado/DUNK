#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6500_WE.h>

/**
 * @brief Estrutura para armazenar o estado do filtro de Kalman.
 */
struct KalmanState
{
    float Q_angle;   // Ruído do processo (Acelerômetro)
    float Q_bias;    // Ruído do processo (Drift do Giroscópio)
    float R_measure; // Ruído da medição
    float angle;     // O ângulo calculado
    float bias;      // O bias calculado do giroscópio
    float rate;      // A taxa calculada
    float P[2][2];   // Matriz de covariância do erro
};

class IMUHandler
{
private:
    MPU6500_WE _mpu;         // Objeto do sensor
    KalmanState _kalman;     // Estado do filtro
    unsigned long _lastTime; // Armazena tempo para cálculo do dt
    float _pitchFiltered;    // Resultado final
    float _pitchRaw;         // Resultado sem filtro (para debug)
    float _gyroRateVal;
    /**
     * @brief Inicializa as variáveis da matriz do Kalman.
     */
    void initKalman();

    /**
     * @brief Executa a matemática do filtro de Kalman.
     * @param newAngle Ângulo medido pelo acelerômetro.
     * @param newRate Taxa medida pelo giroscópio.
     * @param dt Delta tempo em segundos.
     * @return Ângulo filtrado.
     */
    float runKalmanMath(float newAngle, float newRate, float dt);

public:
    /**
     * @brief Construtor.
     * @param addr Endereço I2C do MPU (Padrão 0x68).
     */
    IMUHandler(uint8_t addr = 0x68);

    /**
     * @brief Inicializa o sensor, calibra e configura o DLPF.
     * Deve ser chamado no setup().
     * @return true se o sensor for detectado, false caso contrário.
     */
    bool begin();

    /**
     * @brief Realiza a leitura dos sensores e atualiza o cálculo do Kalman.
     * Deve ser chamado periodicamente no loop().
     */
    void update();

    /**
     * @return O ângulo Pitch filtrado (estável).
     */
    float getPitch();

    /**
     * @return O ângulo Pitch bruto calculado apenas pelo acelerômetro (instável).
     */
    float getRawPitch();
    /**
     * @brief Retorna a velocidade angular no eixo Y (Pitch Rate).
     * @return Valor em Graus por Segundo (deg/s).
     */
    float getGyroRate();
};

#endif