#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <freertos/semphr.h>

struct RobotState
{
    float angle;
    float angularVelocity;
    float linearVelocity;
    float controlOutput;
    float loopRate; // <--- NOVO: Frequência real medida em Hz
};

class SharedDataManager
{
private:
    RobotState _state;
    SemaphoreHandle_t _mutex;

public:
    SharedDataManager()
    {
        _mutex = xSemaphoreCreateMutex();
    }

    // Adicionado parametro 'rate'
    void setState(float ang, float angVel, float linVel, float u, float rate)
    {
        if (xSemaphoreTake(_mutex, (TickType_t)5) == pdTRUE)
        { // Timeout reduzido para 5 ticks
            _state.angle = ang;
            _state.angularVelocity = angVel;
            _state.linearVelocity = linVel;
            _state.controlOutput = u;
            _state.loopRate = rate;
            xSemaphoreGive(_mutex);
        }
    }

    RobotState getState()
    {
        RobotState temp;
        // Inicializa com zeros para evitar lixo de memória se falhar o take
        memset(&temp, 0, sizeof(RobotState));

        if (xSemaphoreTake(_mutex, (TickType_t)5) == pdTRUE)
        {
            temp = _state;
            xSemaphoreGive(_mutex);
        }
        return temp;
    }
};

#endif