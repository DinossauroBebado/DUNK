// ============================================================
// EXEMPLO DE USO DA BIBLIOTECA LQR CONTROLLER
// ============================================================

#include "LQR_Controller.h"

// Matrizes do sistema (exemplo do robô de autoequilíbrio)
const float A[4][4] = {
    {0, 1, 0, 0},
    {119.1, -0.4206, 17.22, -27.01},
    {-7.699, 0.0457, -2.244, 2.938},
    {0, 6.68, -119.3, -50.4}};

const float B[4] = {0, 1, 0, 0};

// Matrizes de peso
float Q[4][4] = {0};
float R = 1.0;

// Controlador LQR
LQRController lqr;

// Estados do sistema
float x[4] = {0.1, 0, 0, 0}; // [phi, omega, v, i]
// Defina o tempo de amostragem (deve ser o mesmo do seu loop de controle)
const float Ts = 0.0005; // 500 microsegundos
// Matrizes Discretas (serão calculadas no setup)
float Ad[4][4];
float Bd[4];

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Discretizando matrizes...");

    // Calcula Ad = I + A * Ts
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Ad[i][j] = A[i][j] * Ts;

            // Adiciona a Identidade na diagonal principal
            if (i == j)
            {
                Ad[i][j] += 1.0;
            }
        }
    }

    // Calcula Bd = B * Ts
    for (int i = 0; i < 4; i++)
    {
        Bd[i] = B[i] * Ts;
    }

    // ============================================================
    // 2. CONFIGURAÇÃO DO LQR
    // ============================================================

    // Configurar pesos (Q e R mantêm-se os mesmos ou ajustam-se para discreto)
    // Nota: Em LQR discreto, Q as vezes é ajustado como Q*Ts, mas para testes pode manter.
    float Q[4][4] = {
        {120, 0, 0, 0}, // Penalidade Angulo
        {0, 10, 0, 0},  // Penalidade Giroscópio
        {0, 0, 5, 0},   // Penalidade Velocidade
        {0, 0, 0, 0.1}  // Penalidade Corrente
    };

    // Passar as matrizes DISCRETAS para o solver
    lqr.setSystemMatrices(Ad, Bd);
    lqr.setWeightMatrices(Q, R);
    lqr.setAlgorithmParameters(1e-6, 1000);

    Serial.println("Calculando ganhos LQR (Sistema Discreto)...");
    unsigned long start = micros();
    lqr.computeGains();
    unsigned long end = micros();

    Serial.print("Tempo de cálculo: ");
    Serial.print(end - start);
    Serial.println(" μs");

    // Mostrar resultados
    lqr.printSystemInfo(); // Vai mostrar Ad e Bd
    lqr.printGains();      // K deve ser numérico agora

    // Teste
    float u = lqr.computeControl(x);
    if (isnan(u))
    {
        Serial.println("ERRO: O resultado ainda é NaN. Verifique se o sistema é controlável.");
    }
    else
    {
        Serial.print("Ação de controle u: ");
        Serial.println(u, 6);
    }
}

void loop()
{
    // Simular variação de estados
    static unsigned long last_time = 0;
    if (millis() - last_time > 1000)
    {
        x[0] = -x[0]; // Alternar ângulo

        float u = lqr.computeControl(x);

        Serial.print("Ângulo: ");
        Serial.print(x[0], 3);
        Serial.print(" rad, Controle: ");
        Serial.println(u, 3);

        last_time = millis();
    }
}