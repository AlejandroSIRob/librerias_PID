#include "PID.h"

// Constructor para inicializar el controlador PID con los parámetros dados
// Implementación de controlador PID en tiempo discreto (4.1.1)
PIDControl::PIDControl(double Kp, double Ki, double Kd, double dt) {
    _Kp = Kp;   // Inicializar la ganancia proporcional
    _Ki = Ki;   // Inicializar la ganancia integral
    _Kd = Kd;   // Inicializar la ganancia derivativa
    _dt = dt;   // Inicializar el intervalo de tiempo
    _previous_error = 0;  // Inicializar el error previo a 0
    _integral = 0;  // Inicializar la integral del error a 0

    // Calcular coeficientes IIR para la implementación como filtro IIR (4.1.2)
    _A0 = Kp + Ki * dt + Kd / dt;
    _A1 = -Kp - 2 * Kd / dt;
    _A2 = Kd / dt;

    // Inicializar variables de error
    _error[0] = 0;
    _error[1] = 0;
    _error[2] = 0;

    // Inicializar variables derivativas
    _d0 = 0;
    _d1 = 0;
    _fd0 = 0;
    _fd1 = 0;

    // Inicializar parámetros del filtro pasa baja (4.1.3)
    _N = 5; // Valor por defecto
    double tau = Kd / (Kp * _N);
    _alpha = dt / (2 * tau);
    _alpha_1 = _alpha / (_alpha + 1);
    _alpha_2 = (_alpha - 1) / (_alpha + 1);
}

// Método para ajustar los parámetros del PID
// Implementación de controlador PID en tiempo discreto (4.1.1)
void PIDControl::setTunings(double Kp, double Ki, double Kd) {
    _Kp = Kp;   // Ajustar la ganancia proporcional
    _Ki = Ki;   // Ajustar la ganancia integral
    _Kd = Kd;   // Ajustar la ganancia derivativa

    // Recalcular coeficientes IIR (4.1.2)
    _A0 = Kp + Ki * _dt + Kd / _dt;
    _A1 = -Kp - 2 * Kd / _dt;
    _A2 = Kd / _dt;

    // Recalcular parámetros del filtro pasa baja (4.1.3)
    double tau = Kd / (Kp * _N);
    _alpha = _dt / (2 * tau);
    _alpha_1 = _alpha / (_alpha + 1);
    _alpha_2 = (_alpha - 1) / (_alpha + 1);
}

// Método para establecer el punto de ajuste (setpoint)
// Implementación de controlador PID en tiempo discreto (4.1.1)
void PIDControl::setSetpoint(double setpoint) {
    _setpoint = setpoint;
}

// Método para calcular la salida del PID basado en el valor medido
// Implementación de controlador PID en tiempo discreto (4.1.1)
// PID como un filtro IIR (4.1.2)
// Filtro pasa baja en el término derivativo (4.1.3)
double PIDControl::compute(double measured_value) {
    // Calcular el error actual
    double error = _setpoint - measured_value;

    // Calcular la suma acumulada del error (integral)
    _integral += error * _dt;

    // Calcular la derivada del error
    double derivative = (error - _previous_error) / _dt;

    // Calcular la salida del PID
    double output = _Kp * error + _Ki * _integral + _Kd * derivative;

    // Guardar el error actual como el error previo para la próxima iteración
    _previous_error = error;

    // Término PI (4.1.2)
    output += _A0 * error + _A1 * _error[1];

    // Término derivativo filtrado (Filtered D) (4.1.3)
    _d1 = _d0;
    _d0 = _A0 * error + _A1 * _error[1] + _A2 * _error[2];
    _fd1 = _fd0;
    _fd0 = _alpha_1 * (_d0 + _d1) - _alpha_2 * _fd1;
    output += _fd0;

    // Desplazar los valores de error
    _error[2] = _error[1];
    _error[1] = _error[0];
    _error[0] = error;

    return output;
}
