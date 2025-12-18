#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "MotorEncoder.h"
#include "IMUHandler.h"
#include "config.h"
#include "ControlTask.h"
#include "SharedData.h"

// --- Instanciação Global do Hardware ---
IMUHandler imu;
MotorController motorRight(MOTOR_RA1_PIN, MOTOR_RA2_PIN, 0, 1);
MotorController motorLeft(MOTOR_LA1_PIN, MOTOR_LA2_PIN, 2, 3);
MotorEncoder encoderLeft(ENCL_A_PIN, ENCL_B_PIN, PPR, false);
MotorEncoder encoderRight(ENCR_A_PIN, ENCR_B_PIN, PPR, false);

// --- Gerenciadores ---
SharedDataManager sharedData;
ControlTask Task(&imu, &motorLeft, &motorRight, &encoderLeft, &encoderRight, &sharedData);

// --- Interrupções (Devem ser globais ou estáticas) ---
void IRAM_ATTR islEncoder() { encoderLeft.update(); }
void IRAM_ATTR isrEncoder() { encoderRight.update(); }

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Aumentar clock do I2C ajuda na velocidade de leitura do IMU
    Wire.setClock(400000);

    // Inicialização IMU
    if (!imu.begin())
    {
        Serial.println("ERRO: MPU6500 não encontrado.");
        while (1)
            ;
    }
    Serial.println("IMU OK");

    // Inicialização Motores
    motorLeft.begin();
    motorRight.begin();
    encoderRight.begin();
    encoderLeft.begin();

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(ENCL_A_PIN), islEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCR_A_PIN), isrEncoder, CHANGE);

    Serial.println("Iniciando Task de Controle LQR no Core 0...");

    // Inicia a thread de controle separada
    Task.begin();
}

void loop()
{
    // --- Loop de Telemetria (Roda no Core 1) ---
    // que não afetará o equilíbrio do robô.
    // Recupera dados seguros do controlador
    RobotState currentData = sharedData.getState();
    // Use o Teleplot ou Serial Plotter
    Serial.print(">ang:");
    Serial.println(currentData.angle);
    Serial.print(">VelAng:");
    Serial.println(currentData.angularVelocity);
    Serial.print(">velLinear:");
    Serial.println(currentData.linearVelocity);
    Serial.print(">u:");
    Serial.println(currentData.controlOutput);
    Serial.print(">Hz:");
    Serial.println(currentData.loopRate);

    // Frequência de atualização da tela/serial (ex: 20Hz é suficiente para olho humano)
    delay(50);
}