#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

#include "motors.h"
#include "robot_config.h"

// Busca un objeto de TARGET_WIDTH_MM de diámetro girando en el lugar.
//
// Por el Serial imprime SOLO la distancia de cada medida, en mm, una por línea.
//
// Idea: mientras gira, el robot detecta los "bordes" del objeto por saltos de
// distancia (la lectura baja de golpe al entrar a la base y sube al salir).
// Cuenta cuántas medidas consecutivas duró el objeto y lo compara con las que
// debería durar un objeto de 5,5 cm a esa distancia:
//
//   N = θ / Δθ
//   θ  = 2·atan(w / 2D)    ángulo que ocupa la base (D = distancia + desfase del sensor)
//   Δθ = ω · T             grados que gira el robot entre dos medidas
//
// Si coincide (± tolerancia), vuelve al centro del objeto y se detiene.

namespace {
// --- Objetivo ---
constexpr float TARGET_WIDTH_MM = 55.0f;       // diámetro del objeto a buscar

// --- Giro (MEDIR: girar 360° con SCAN_PWM, cronometrar t, ω = 360 / t) ---
constexpr int SCAN_PWM = 150;
constexpr float TURN_RATE_DEG_PER_S = 57.0f;   // estimado con 45 rpm, rueda 65 mm, trocha 180 mm

// --- Geometría del robot (MEDIR) ---
constexpr float SENSOR_OFFSET_MM = 100.0f;     // del eje de giro al sensor

// --- Cono del VL53L0X (CALIBRAR) ---
// 0 = solo geometría. El cono hace que el objeto dure más medidas de las que da la fórmula.
constexpr float CONE_DEG = 0.0f;

// --- Sensor a la máxima frecuencia ---
// 20 ms es el tiempo de medición mínimo del VL53L0X (modo rápido, algo menos preciso).
// Una medida cada 25 ms = 40 por segundo (antes 20).
constexpr uint32_t TIMING_BUDGET_US = 20000;
constexpr uint16_t SAMPLE_INTERVAL_MS = 25;

// --- Detección ---
constexpr uint16_t MIN_RANGE_MM = 40;
constexpr uint16_t MAX_RANGE_MM = 1200;        // más allá se considera "nada"
constexpr uint16_t EDGE_JUMP_MM = 100;         // salto mínimo para considerar un borde
constexpr uint8_t SAMPLE_TOLERANCE = 2;        // ± medidas aceptadas
constexpr uint8_t MIN_SAMPLES = 3;             // menos que esto se toma como ruido

// Si el ruido de los motores traba el I2C, Wire corta a los 25 ms en vez de colgarse.
constexpr uint32_t I2C_TIMEOUT_US = 25000;

constexpr uint32_t SERIAL_BAUD = 9600;         // aguanta mejor el ruido que 115200

// Arranque suave: la velocidad sube de 0 al valor pedido en RAMP_MS.
// Evita el pico de corriente que puede disparar la protección de la batería.
constexpr uint16_t RAMP_MS = 300;
constexpr uint8_t RAMP_STEPS = 15;

constexpr float DEG_PER_SAMPLE = TURN_RATE_DEG_PER_S * SAMPLE_INTERVAL_MS / 1000.0f;

VL53L0X tof;
Motors motors;

bool targetLocked = false;
uint32_t lastSampleAt = 0;

bool hasPrevious = false;
uint16_t previousRangeMm = 0;

bool inSegment = false;
uint16_t segmentSamples = 0;
uint32_t segmentRangeSum = 0;
uint16_t segmentRefMm = 0;
uint32_t segmentFirstAt = 0;
uint32_t segmentLastAt = 0;

bool startSensor() {
    tof.setTimeout(100);
    if (!tof.init()) {
        return false;
    }
    tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
    tof.startContinuous(SAMPLE_INTERVAL_MS);
    return true;
}

// Ángulo (grados) que ocupa la base vista desde el eje de giro.
float targetSpanDeg(uint16_t distanceMm) {
    const float d = distanceMm + SENSOR_OFFSET_MM;
    return 2.0f * atan(TARGET_WIDTH_MM / (2.0f * d)) * 180.0f / PI + CONE_DEG;
}

// Medidas consecutivas que debería durar la base a esa distancia.
float expectedSamples(uint16_t distanceMm) {
    return targetSpanDeg(distanceMm) / DEG_PER_SAMPLE;
}

void startSegment(uint16_t rangeMm, uint32_t now) {
    inSegment = true;
    segmentSamples = 1;
    segmentRangeSum = rangeMm;
    segmentRefMm = rangeMm;
    segmentFirstAt = now;
    segmentLastAt = now;
}

void addToSegment(uint16_t rangeMm, uint32_t now) {
    if (segmentSamples == UINT16_MAX) {
        return;  // contador lleno: no sumar más para no romper el promedio
    }
    ++segmentSamples;
    segmentRangeSum += rangeMm;
    segmentLastAt = now;
}

// Lleva los motores de 0 a (left, right) de a poco, en RAMP_MS.
void rampTo(int left, int right) {
    for (uint8_t i = 1; i <= RAMP_STEPS; ++i) {
        motors.setSpeed((long)left * i / RAMP_STEPS, (long)right * i / RAMP_STEPS);
        delay(RAMP_MS / RAMP_STEPS);
    }
}

// Vuelve hasta el centro del objeto y frena.
void centerOnTarget(uint32_t now) {
    const uint32_t centerAt = segmentFirstAt + (segmentLastAt - segmentFirstAt) / 2;
    const uint32_t correctionMs = now - centerAt;

    targetLocked = true;
    motors.stop();
    delay(150);
    // Durante la rampa gira, en promedio, a media velocidad: equivale a RAMP_MS/2 a velocidad plena.
    rampTo(SCAN_PWM, -SCAN_PWM);  // vuelve hacia atrás
    if (correctionMs > RAMP_MS / 2) {
        delay(correctionMs - RAMP_MS / 2);
    }
    motors.stop();
}

// Se llama cuando el objeto terminó (salto hacia arriba de la distancia).
void evaluateSegment(uint32_t now) {
    inSegment = false;

    const uint16_t avgRangeMm = segmentRangeSum / segmentSamples;
    const float diff = fabs(segmentSamples - expectedSamples(avgRangeMm));
    if (segmentSamples >= MIN_SAMPLES && diff <= SAMPLE_TOLERANCE) {
        centerOnTarget(now);
    }
}

// Detección de bordes con una medida ya válida.
void detect(uint16_t rangeMm, uint32_t now) {
    if (rangeMm < MIN_RANGE_MM) {
        return;  // demasiado cerca o lectura basura
    }
    if (rangeMm > MAX_RANGE_MM) {
        rangeMm = MAX_RANGE_MM;  // "nada" cuenta como fondo lejano
    }

    if (!hasPrevious) {
        hasPrevious = true;
        previousRangeMm = rangeMm;
        return;
    }

    if (!inSegment) {
        // Borde de entrada: la distancia baja de golpe.
        if (rangeMm < MAX_RANGE_MM &&
            previousRangeMm >= rangeMm + EDGE_JUMP_MM) {
            startSegment(rangeMm, now);
        }
    } else if (rangeMm >= segmentRefMm + EDGE_JUMP_MM) {
        // Borde de salida: la distancia sube de golpe.
        evaluateSegment(now);
    } else if (rangeMm + EDGE_JUMP_MM <= segmentRefMm) {
        // Apareció algo todavía más cerca: empieza un objeto nuevo.
        startSegment(rangeMm, now);
    } else {
        addToSegment(rangeMm, now);
    }

    previousRangeMm = rangeMm;
}

void sample() {
    const uint32_t now = millis();
    if (now - lastSampleAt < SAMPLE_INTERVAL_MS) {
        return;
    }
    lastSampleAt = now;

    const uint16_t rangeMm = tof.readRangeContinuousMillimeters();
    if (Wire.getWireTimeoutFlag()) {
        // El ruido trabó el I2C: reiniciar el sensor y empezar el objeto de cero.
        Wire.clearWireTimeoutFlag();
        hasPrevious = false;
        inSegment = false;
        startSensor();
        return;
    }
    if (tof.timeoutOccurred()) {
        return;
    }

    Serial.println(rangeMm);  // solo el número

    if (!targetLocked) {
        detect(rangeMm, now);
    }
}
} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    Wire.begin();
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);  // no colgarse si el ruido traba el bus
    motors.begin();
    motors.stop();

    if (!startSensor()) {
        Serial.println(F("ERROR: no encuentro el sensor VL53L0X"));
        while (true) {
            motors.stop();
            delay(1000);
        }
    }

    rampTo(-SCAN_PWM, SCAN_PWM);  // arranque suave del barrido
    lastSampleAt = millis();
}

void loop() {
    sample();
}
