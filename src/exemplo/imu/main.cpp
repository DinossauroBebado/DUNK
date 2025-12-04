#include <Arduino.h>
#include <Wire.h>
#include "IMUHandler.h"

// Instancia o objeto (Endereço 0x68 padrão)
IMUHandler imu;

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Inicializa a biblioteca (inclui calibração automática)
    Serial.println("Inicializando MPU6500...");
    if (!imu.begin())
    {
        Serial.println("ERRO: MPU6500 não encontrado. Verifique conexões.");
        while (1)
            ; // Trava o código se falhar
    }
    Serial.println("MPU6500 Pronto!");
}

void loop()
{
    // 1. Atualiza leituras e cálculos
    imu.update();

    // 2. Obtém os valores
    float filtered = imu.getPitch();
    float raw = imu.getRawPitch();

    // 3. Imprime para o Serial Plotter
    Serial.print("Pitch_Filtrado:");
    Serial.print(filtered);
    Serial.print("   Pitch_Bruto:"); // Espaços para separar no plotter
    Serial.println(raw);

    // Pequeno delay para estabilidade do loop, mas cuidado para não ser grande demais
    // pois o Kalman depende de um dt constante/pequeno.
    delay(10);
}