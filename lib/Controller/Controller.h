// ============================================================
// BIBLIOTECA LQR CONTROLLER PARA ARDUINO/ESP32
// ============================================================
// Autor: Baseado em algoritmo LQR equivalente ao MATLAB
// Versão: 1.0
// ============================================================

#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <Arduino.h>

class LQRController {
private:
    // Estruturas de dados
    typedef float Matrix4x4[4][4];
    typedef float Vector4[4];
    
    // Parâmetros do controlador
    float K[4];              // Ganhos LQR
    Matrix4x4 A;             // Matriz de estado
    Vector4 B;               // Vetor de entrada
    Matrix4x4 Q;             // Matriz de peso dos estados
    float R;                 // Peso do controle
    
    // Configuração do algoritmo
    float tolerance;
    int max_iterations;
    
    // Métodos privados
    void solveRiccatiEquation(Matrix4x4 P);
    float matrixDifference(const Matrix4x4 P1, const Matrix4x4 P2);
    void calculateGains(const Matrix4x4 P, Vector4 K_out);
    
public:
    // Construtores
    LQRController();
    LQRController(const Matrix4x4 A_matrix, const Vector4 B_vector, 
                  const Matrix4x4 Q_matrix, float R_value);
    
    // Métodos principais
    void computeGains();
    void computeGains(const Matrix4x4 A_matrix, const Vector4 B_vector, 
                     const Matrix4x4 Q_matrix, float R_value);
    
    // Métodos de configuração
    void setSystemMatrices(const Matrix4x4 A_matrix, const Vector4 B_vector);
    void setWeightMatrices(const Matrix4x4 Q_matrix, float R_value);
    void setAlgorithmParameters(float tol, int max_iter);
    
    // Métodos de acesso
    void getGains(Vector4 K_out) const;
    float getGain(int index) const;
    void printGains() const;
    void printSystemInfo() const;
    
    // Método de controle
    float computeControl(const Vector4 x) const;
};

#endif