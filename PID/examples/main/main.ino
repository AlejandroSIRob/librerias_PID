#include <Arduino.h>
#include "PID.h"

// Definir los parámetros del PID
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01
#define dt 0.1

// Crear una instancia del controlador PID
PIDControl pid(Kp, Ki, Kd, dt);

void setup() {
    // Configurar el punto de ajuste
    pid.setSetpoint(100.0);
}

void loop() {
    // Medir el valor actual (aquí se debería leer un sensor)
    double measured_value = analogRead(A0);

    // Calcular la salida del PID
    double output = pid.compute(measured_value);

    // Aplicar la salida (aquí se debería escribir a un actuador)
    analogWrite(OUTPUT_PIN, output);

    // Esperar el intervalo de muestreo
    delay(dt * 1000);
}
