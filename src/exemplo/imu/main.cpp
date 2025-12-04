#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>

#define MPU6500_ADDR 0x68

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

float pitch_f = 0;
float pitch_nf = 0;

// Estruturas de Kalman
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float rate;
    float P[2][2];
} Kalman;

Kalman kalmanX; // Não precisa de Y se não for usar agora

unsigned long lastTime = 0;

// Inicialização
void initKalman(Kalman &K)
{
    K.Q_angle = 0.001;
    K.Q_bias = 0.003;
    K.R_measure = 0.03;

    K.angle = 0;
    K.bias = 0;

    K.P[0][0] = 0;
    K.P[0][1] = 0;
    K.P[1][0] = 0;
    K.P[1][1] = 0;
}

// Atualização Kalman
float kalmanUpdate(Kalman &K, float newAngle, float newRate, float dt)
{
    // Predição
    K.rate = newRate - K.bias;
    K.angle += dt * K.rate;

    K.P[0][0] += dt * (dt * K.P[1][1] - K.P[0][1] - K.P[1][0] + K.Q_angle);
    K.P[0][1] -= dt * K.P[1][1];
    K.P[1][0] -= dt * K.P[1][1];
    K.P[1][1] += K.Q_bias * dt;

    // Atualização
    float S = K.P[0][0] + K.R_measure; // Se R_measure for 0, S será 0 -> Erro!

    float K_gain[2];
    K_gain[0] = K.P[0][0] / S;
    K_gain[1] = K.P[1][0] / S;

    float y = newAngle - K.angle;
    K.angle += K_gain[0] * y;
    K.bias += K_gain[1] * y;

    float P00_temp = K.P[0][0];
    float P01_temp = K.P[0][1];

    K.P[0][0] -= K_gain[0] * P00_temp;
    K.P[0][1] -= K_gain[0] * P01_temp;
    K.P[1][0] -= K_gain[1] * P00_temp;
    K.P[1][1] -= K_gain[1] * P01_temp;

    return K.angle;
}

void readMPU_filtered()
{
    xyzFloat acc = myMPU6500.getGValues();
    xyzFloat gyr = myMPU6500.getGyrValues();

    // Tempo (em segundos)
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // Proteção contra dt inválido na primeira leitura
    if (dt <= 0 || dt > 1.0)
        return;

    // Cálculo de ângulos do acelerômetro (em graus)
    // Usar atan2 é mais seguro matematicamente que atan(x/y)
    float accelPitch = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0 / PI;

    // Taxas do giroscópio
    // A biblioteca JÁ retorna em Graus/s. Não converta radianos!
    float gyroPitchRate = gyr.y; // <--- CORREÇÃO: Removido * 180/PI

    // Filtro de Kalman
    pitch_f = kalmanUpdate(kalmanX, accelPitch, gyroPitchRate, dt);
}

void readMPU6050()
{
    xyzFloat gValue = myMPU6500.getGValues();
    // Uso consistente de atan2 para evitar erros de divisão por zero
    float pitch = atan2(gValue.x, sqrt(gValue.y * gValue.y + gValue.z * gValue.z)) * 180.0 / PI;
    pitch_nf = pitch;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    if (!myMPU6500.init())
    {
        Serial.println("MPU6500 does not respond");
    }
    else
    {
        Serial.println("MPU6500 is connected");
    }

    // <--- CORREÇÃO CRÍTICA: Inicializar o Kalman para evitar divisão por zero
    initKalman(kalmanX);

    Serial.println("Calibrating...");
    delay(1000);
    myMPU6500.autoOffsets();
    Serial.println("Done!");

    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);

    delay(200);

    // Inicializar o tempo para o primeiro dt ser correto
    lastTime = micros();

    pinMode(14, OUTPUT);
    pinMode(27, OUTPUT);
    digitalWrite(14, LOW);
    digitalWrite(27, LOW);
}

void loop()
{
    // readMPU6050();      // Leitura bruta (com atan2)
    readMPU_filtered(); // Leitura filtrada

    Serial.print("Pitch_Filtrado:");
    Serial.print(pitch_f);
    Serial.print("   Pitch_Bruto:"); // Removido espaço no nome para o Serial Plotter
    Serial.println(pitch_nf);

    delay(10); // Aumentei um pouco para não saturar a Serial
}