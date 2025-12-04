#include <Arduino.h>
#include "MotorController.h"
#include "config.h"

// Instanciação do objeto (usando canais PWM 0 e 1 por padrão)
MotorController motorRight(MOTOR_RA1_PIN, MOTOR_RA2_PIN, 0, 1);
MotorController motorLeft(MOTOR_LA1_PIN, MOTOR_LA2_PIN, 2, 3);

void setup()
{
    Serial.begin(115200);

    // Inicializa o controle do motor
    motorLeft.begin();
    motorRight.begin();

    Serial.println("Sistema Iniciado. Testando motor...");
}

void loop()
{
    // Acelera para frente
    Serial.println("Frente: 50%");
    motorLeft.write(50);
    motorRight.write(50);
    delay(2000);

    Serial.println("Frente: 100%");
    motorLeft.write(100);
    motorRight.write(100);
    delay(2000);

    // Para
    Serial.println("Parando...");
    motorLeft.write(0);
    motorRight.write(0);
    delay(1000);

    // Acelera para trás
    Serial.println("Trás: -75%");
    motorLeft.write(-75);
    motorRight.write(-75);
    delay(2000);

    // Para novamente
    motorLeft.stop();
    motorRight.stop();
    Serial.println("Parando...");
    delay(1000);
}