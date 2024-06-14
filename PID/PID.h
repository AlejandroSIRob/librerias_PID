#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {
public:
    PIDControl(double Kp, double Ki, double Kd, double dt);
    void setTunings(double Kp, double Ki, double Kd);
    void setSetpoint(double setpoint);
    double compute(double measured_value);

private:
    double _Kp;       // Ganancia proporcional
    double _Ki;       // Ganancia integral
    double _Kd;       // Ganancia derivativa
    double _dt;       // Intervalo de tiempo
    double _setpoint; // Punto de ajuste (setpoint)
    double _previous_error; // Error previo
    double _integral;       // Suma acumulada del error (integral)
    
    // Variables para el cálculo del filtro IIR
    double _A0;         // Coeficiente A0 del controlador PID
    double _A1;         // Coeficiente A1 del controlador PID
    double _A2;         // Coeficiente A2 del controlador PID
    double _error[3];   // Array para almacenar los errores actuales y anteriores (e(t), e(t-1), e(t-2))
    double _d0;         // Término derivativo actual del controlador PID
    double _d1;         // Término derivativo previo del controlador PID
    double _fd0;        // Salida actual del filtro derivativo (IIR)
    double _fd1;        // Salida previa del filtro derivativo (IIR)
    double _alpha;      // Constante de tiempo del filtro pasa baja (determina el grado de filtrado)
    double _alpha_1;    // Coeficiente auxiliar para el cálculo del filtro pasa baja
    double _alpha_2;    // Coeficiente auxiliar para el cálculo del filtro pasa baja
    int _N;             // Factor de ajuste para la constante de tiempo del filtro pasa baja

};

#endif
