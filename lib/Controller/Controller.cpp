// ============================================================
// IMPLEMENTAÇÃO DA BIBLIOTECA Control CONTROLLER
// ============================================================

#include "Control_Controller.h"
#include <cmath>

// ============================================================
// CONSTRUTORES
// ============================================================

ControlController::ControlController()
{
    // Valores padrão
    tolerance = 1e-6;
    max_iterations = 1000;
    R = 1.0;

    // Inicializar matrizes com zeros
    memset(A, 0, sizeof(A));
    memset(B, 0, sizeof(B));
    memset(Q, 0, sizeof(Q));
    memset(K, 0, sizeof(K));

    // Matriz identidade padrão para Q
    for (int i = 0; i < 4; i++)
    {
        Q[i][i] = 1.0;
    }
}

ControlController::ControlController(const Matrix4x4 A_matrix, const Vector4 B_vector,
                                     const Matrix4x4 Q_matrix, float R_value)
{
    tolerance = 1e-6;
    max_iterations = 1000;

    // Copiar matrizes
    memcpy(A, A_matrix, sizeof(A));
    memcpy(B, B_vector, sizeof(B));
    memcpy(Q, Q_matrix, sizeof(Q));
    R = R_value;

    // Inicializar ganhos
    memset(K, 0, sizeof(K));
}

// ============================================================
// MÉTODOS PRIVADOS
// ============================================================

float ControlController::matrixDifference(const Matrix4x4 P1, const Matrix4x4 P2)
{
    float max_diff = 0.0;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            float diff = fabs(P1[i][j] - P2[i][j]);
            if (diff > max_diff)
                max_diff = diff;
        }
    }
    return max_diff;
}

void ControlController::solveRiccatiEquation(Matrix4x4 P)
{
    Matrix4x4 P_prev, I;
    int iteration = 0;
    float diff = tolerance + 1.0;

    // Inicializar matriz identidade
    memset(I, 0, sizeof(I));
    for (int i = 0; i < 4; i++)
        I[i][i] = 1.0;

    // Inicializar P com zeros
    memset(P, 0, sizeof(Matrix4x4));

    while (iteration < max_iterations && diff > tolerance)
    {
        // Salvar P anterior
        memcpy(P_prev, P, sizeof(Matrix4x4));

        // Calcular B' * P * B + R
        float BPB = R;
        for (int i = 0; i < 4; i++)
        {
            float temp = 0.0;
            for (int j = 0; j < 4; j++)
            {
                temp += P_prev[j][i] * B[j]; // B' * P
            }
            BPB += temp * B[i];
        }

        // Calcular termo intermediário K_temp = (B' * P * A) / (R + B' * P * B)
        Vector4 K_temp = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                K_temp[i] += P_prev[j][i] * B[j]; // B' * P
            }
        }

        Vector4 K_temp2 = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                K_temp2[i] += K_temp[j] * A[j][i]; // (B' * P) * A
            }
        }

        for (int i = 0; i < 4; i++)
        {
            K_temp2[i] /= BPB;
        }

        // Calcular A' * P * A
        Matrix4x4 APA = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    APA[i][j] += A[k][i] * P_prev[k][j]; // A' * P
                }
            }
        }

        Matrix4x4 APAA = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    APAA[i][j] += APA[i][k] * A[k][j]; // (A' * P) * A
                }
            }
        }

        // Calcular A' * P * B * K_temp2
        Matrix4x4 APBKA = {0};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    APBKA[i][j] += A[k][i] * P_prev[k][j] * B[j] * K_temp2[i];
                }
            }
        }

        // Atualizar P: P = A'*P*A - A'*P*B*K_temp2 + Q
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                P[i][j] = APAA[i][j] - APBKA[i][j] + Q[i][j];
            }
        }

        // Verificar convergência
        diff = matrixDifference(P, P_prev);
        iteration++;
    }

    if (iteration >= max_iterations)
    {
        Serial.println("Aviso: Control não convergiu no número máximo de iterações");
    }
}

void ControlController::calculateGains(const Matrix4x4 P, Vector4 K_out)
{
    // Calcular B' * P
    Vector4 BP = {0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            BP[i] += B[j] * P[j][i]; // B' * P
        }
    }

    // Calcular B' * P * B + R
    float BPB_final = R;
    for (int i = 0; i < 4; i++)
    {
        BPB_final += BP[i] * B[i];
    }

    // Calcular B' * P * A
    Vector4 BPA = {0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            BPA[i] += BP[j] * A[j][i]; // (B' * P) * A
        }
    }

    // Ganhos finais: K = BPA / BPB_final
    for (int i = 0; i < 4; i++)
    {
        K_out[i] = BPA[i] / BPB_final;
    }
}

// ============================================================
// MÉTODOS PÚBLICOS
// ============================================================

void ControlController::computeGains()
{
    Matrix4x4 P;
    solveRiccatiEquation(P);
    calculateGains(P, K);
}

void ControlController::computeGains(const Matrix4x4 A_matrix, const Vector4 B_vector,
                                     const Matrix4x4 Q_matrix, float R_value)
{
    setSystemMatrices(A_matrix, B_vector);
    setWeightMatrices(Q_matrix, R_value);
    computeGains();
}

void ControlController::setSystemMatrices(const Matrix4x4 A_matrix, const Vector4 B_vector)
{
    memcpy(A, A_matrix, sizeof(A));
    memcpy(B, B_vector, sizeof(B));
}

void ControlController::setWeightMatrices(const Matrix4x4 Q_matrix, float R_value)
{
    memcpy(Q, Q_matrix, sizeof(Q));
    R = R_value;
}

void ControlController::setAlgorithmParameters(float tol, int max_iter)
{
    tolerance = tol;
    max_iterations = max_iter;
}

void ControlController::getGains(Vector4 K_out) const
{
    memcpy(K_out, K, sizeof(K));
}

float ControlController::getGain(int index) const
{
    if (index >= 0 && index < 4)
    {
        return K[index];
    }
    return 0.0;
}

float ControlController::computeControl(const Vector4 x) const
{
    float u = 0.0;
    for (int i = 0; i < 4; i++)
    {
        u -= K[i] * x[i]; // u = -K*x
    }
    return u;
}

void ControlController::printGains() const
{
    Serial.println("=== GANHOS Control ===");
    Serial.print("K = [");
    for (int i = 0; i < 4; i++)
    {
        Serial.print(K[i], 6);
        if (i < 3)
            Serial.print(", ");
    }
    Serial.println("]");
}

void ControlController::printSystemInfo() const
{
    Serial.println("=== SISTEMA Control ===");
    Serial.print("R: ");
    Serial.println(R, 6);
    Serial.println("Matriz Q:");
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            Serial.print(Q[i][j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
}