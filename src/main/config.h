
// left motor
#define MOTOR_LA1_PIN 14 // Direction
#define MOTOR_LA2_PIN 27 // Direction

// Definição do Encoder (Ex: Motor Amarelo TT costuma ser +- 500 a 1000 ticks na saída da caixa)
// Se for direto no motor magnético, costuma ser 11 ou 7 PPR x Redução.
#define ENCL_A_PIN 18
#define ENCL_B_PIN 19

// right motor
#define MOTOR_RA1_PIN 25 // Direction
#define MOTOR_RA2_PIN 33 // Direction

// Definição do Encoder (Ex: Motor Amarelo TT costuma ser +- 500 a 1000 ticks na saída da caixa)
// Se for direto no motor magnético, costuma ser 11 ou 7 PPR x Redução.
#define ENCR_A_PIN 34
#define ENCR_B_PIN 35

#define PPR 770 // Ajuste conforme seu motor

// Matrizes do sistema (exemplo do robô de autoequilíbrio)
const float A[4][4] = {
    {0, 1, 0, 0},
    {1032.5, 0.31, -773.1, 7.2},
    {-105.5, 18.64, -883.0, 430.04},
    {0, 2.4, -71.64, -400}};

const float B[4] = {0, 1, 0, 0};

// Matrizes de peso
float Q[4][4] = {0};
float R = 1.0;

// float x[4] = {0, 0, 0, 0}; // [phi, omega, v, i]
#define WHEEL_RADIUS 0.035