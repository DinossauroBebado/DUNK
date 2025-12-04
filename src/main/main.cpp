#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "MotorEncoder.h"
#include "IMUHandler.h"
// Removemos o include do LQR_Controller.h pois faremos o cálculo manual
// #include "LQR_Controller.h"

#include "config.h"

IMUHandler imu;

// ============================================================
// GANHOS LQR (Calculados Offline no Computador)
// Para Ts = 0.01s (10ms) e suas matrizes A/B/Q/R
// ============================================================
// ATENÇÃO: Seus valores de matriz A são muito altos (1032.5).
// Isso gerou ganhos altos. Se o robô vibrar muito, reduza esses valores manualmente.
float K[4] = {197.362, 46.47, -36.66, -36.21};

// Variáveis de Estado
float x[4] = {0, 0, 0, 0};

// Instanciação do Hardware
MotorController motorRight(MOTOR_RA1_PIN, MOTOR_RA2_PIN, 0, 1);
MotorController motorLeft(MOTOR_LA1_PIN, MOTOR_LA2_PIN, 2, 3);
MotorEncoder EncoderLeft(ENCL_A_PIN, ENCL_B_PIN, PPR, false);
MotorEncoder EncoderRIGHT(ENCR_A_PIN, ENCR_B_PIN, PPR, false);

void IRAM_ATTR islEncoder() { EncoderLeft.update(); }
void IRAM_ATTR isrEncoder() { EncoderRIGHT.update(); }

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Inicialização IMU
    if (!imu.begin())
    {
        Serial.println("ERRO: MPU6500 não encontrado.");
        while (1)
            ;
    }
    Serial.println("IMU OK");

    // Inicialização Motores e Encoders
    motorLeft.begin();
    motorRight.begin();
    EncoderRIGHT.begin();
    EncoderLeft.begin();

    attachInterrupt(digitalPinToInterrupt(ENCL_A_PIN), islEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCR_A_PIN), isrEncoder, CHANGE);

    Serial.println("Sistema Pronto. Iniciando controle LQR Estático.");

    // Debug dos ganhos carregados
    Serial.print("Ganhos K: [");
    Serial.print(K[0]);
    Serial.print(", ");
    Serial.print(K[1]);
    Serial.print(", ");
    Serial.print(K[2]);
    Serial.print(", ");
    Serial.print(K[3]);
    Serial.println("]");

    motorLeft.write(0);
    motorRight.write(0);
}

void loop()
{
    unsigned long startLoop = millis();

    // 1. Atualiza Sensores
    imu.update();

    // 2. Obtém Estados (Convertendo para SI: Radianos e m/s)
    float angulo = imu.getPitch() * (PI / 180.0);
    float velAngular = imu.getGyroRate() * (PI / 180.0);

    // Média das velocidades das rodas
    float velLinear = ((EncoderLeft.getVelocityRad() + EncoderRIGHT.getVelocityRad()) / 2.0) * WHEEL_RADIUS;

    x[0] = angulo;     // Phi
    x[1] = velAngular; // Omega
    x[2] = velLinear;  // v
    x[3] = 0;          // i (Corrente - assumido 0 se não houver sensor)

    // 3. Calcula Ação de Controle: u = -K * x
    // u = - (k1*ang + k2*gyro + k3*vel + k4*curr)
    float u = -(K[0] * x[0] +
                K[1] * x[1] +
                K[2] * x[2] +
                K[3] * x[3]);

    // 4. Aplica aos motores
    // Nota: O valor de 'u' calculado pelo seu modelo (ganho 1973) será MUITO ALTO (ex: 200V).
    // A função motor.write deve limitar isso (clamp) entre -100 e 100 ou -255 e 255.
    motorLeft.write(u);
    motorRight.write(u);

    // Telemetria
    Serial.print("Ang:");
    Serial.print(angulo, 3);
    Serial.print(" VelAng:");
    Serial.print(velAngular, 3);
    Serial.print(" VelLin:");
    Serial.print(velLinear, 3);
    Serial.print(" PWM_Calc:");
    Serial.println(u, 2);

    // Mantém o loop em ~100Hz (10ms)
    // Usamos while para garantir precisão, delay(10) é impreciso
    while (millis() - startLoop < 10)
        ;
}