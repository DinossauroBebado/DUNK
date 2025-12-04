#include <Arduino.h>
#include "MotorEncoder.h"

const int canal_pwm_1 = 0; // Canal 0 do LEDC (0-15)
const int canal_pwm_2 = 1;
const int resolucao = 8; // 8 bits (0-255)
const int duty_cycle = 200;

int pinAIN1 = 25; // Direction
int pinAIN2 = 33; // Direction

// Definição do Encoder (Ex: Motor Amarelo TT costuma ser +- 500 a 1000 ticks na saída da caixa)
// Se for direto no motor magnético, costuma ser 11 ou 7 PPR x Redução.
#define ENC_A_PIN 34
#define ENC_B_PIN 35

#define PPR 770 // Ajuste conforme seu motor

// Instanciação do Objeto (reverse = false)
MotorEncoder meuEncoder(ENC_A_PIN, ENC_B_PIN, PPR, false);

// --- Wrappers para Interrupção ---
// O attachInterrupt precisa de uma função void global/estática
void IRAM_ATTR isrEncoder()
{
  meuEncoder.update();
}

void setup()
{
  Serial.begin(115200);

  // Inicializa o objeto
  meuEncoder.begin();

  // Configura a interrupção do hardware
  // CHANGE = detecta borda de subida e descida (aumenta precisão)
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isrEncoder, CHANGE);

  // --- Inicializa o PWM ---
  // No ESP32, primeiro configuramos o canal com uma frequência inicial (ex: 1000Hz)
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  // --- Inicializa o PWM ---
  // No ESP32, primeiro configuramos o canal com uma frequência inicial (ex: 1000Hz)
  ledcSetup(canal_pwm_1, 1000, resolucao);
  ledcSetup(canal_pwm_2, 1000, resolucao);

  // // Depois, conectamos o canal ao pino STEP
  ledcAttachPin(pinAIN1, canal_pwm_1);
  ledcAttachPin(pinAIN2, canal_pwm_2);

  Serial.println("Sistema Iniciado...");
}

void loop()
{

  // 2. Garante que o Duty Cycle continue em 50% (onda quadrada)
  ledcWrite(canal_pwm_1, duty_cycle);

  ledcWrite(canal_pwm_2, 0);

  // Simulação de loop de controle

  // A cada 100ms, printa o debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100)
  {

    // Chama a função de debug solicitada
    meuEncoder.debug();

    // Exemplo de como pegar apenas o valor rad/s para usar num PID
    // float velocidade = meuEncoder.getVelocityRad();

    lastPrint = millis();
  }
}