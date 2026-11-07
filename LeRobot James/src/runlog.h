#ifndef RUNLOG_H
#define RUNLOG_H

#include <Arduino.h>

// Registro de corridas en la EEPROM del Nano (1024 bytes, no se borra al apagar).
// Guarda las últimas runlog::SLOTS corridas: un resumen y los objetos que vio cada una.

enum RunStatus : uint8_t {
    RUN_EMPTY = 0,       // slot sin usar
    RUN_INCOMPLETE = 1,  // arrancó y no terminó (se cortó la energía o se reinició)
    RUN_FOUND = 2,       // encontró el objeto y se centró
    RUN_NOT_FOUND = 3,   // dio todas las vueltas sin encontrarlo
    RUN_NOT_CENTERED = 4,  // lo encontró, pero al volver lento no lo volvió a ver
};

enum ObjectResult : uint8_t {
    OBJ_OTHER = 1,   // objeto que no coincide con el buscado
    OBJ_TARGET = 2,  // el objeto buscado
};

struct RunHeader {
    uint16_t runId;         // número de corrida (va subiendo)
    uint8_t status;         // RunStatus
    uint8_t objectCount;
    uint16_t durationDs;    // duración en décimas de segundo
    uint16_t readings;      // medidas tomadas
    uint16_t centerHalfMs;  // mitad del tiempo que tardó en cruzar el objeto, girando lento
    uint16_t targetMm;      // distancia al objeto encontrado
    uint16_t finalMm;       // distancia hacia donde quedó apuntando al final
    uint8_t centerSamples;  // lecturas que vieron el objeto en el último cruce lento
    uint8_t centerTries;    // cruces lentos intentados
};

struct ObjectRecord {
    uint16_t timeDs;        // cuándo terminó el objeto, desde el arranque de la misión
    uint16_t distanceMm;    // distancia promedio
    uint8_t samples;        // medidas que duró
    uint8_t result;         // ObjectResult
};

namespace runlog {
constexpr uint8_t SLOTS = 2;

void begin();       // si la EEPROM no tiene este formato, la inicializa vacía
void clear();       // borra todas las corridas
void startRun();    // empieza una corrida nueva en el slot más viejo
bool addObject(const ObjectRecord& obj);  // false si ya no entran más objetos
void update(uint16_t durationDs, uint16_t readings);
void finish(RunStatus status, uint16_t durationDs, uint16_t readings,
            uint16_t centerHalfMs, uint16_t targetMm, uint16_t finalMm,
            uint8_t centerSamples, uint8_t centerTries);

// "Sin leer": se marca al empezar una corrida y se limpia cuando alguien la ve por USB.
// Mientras haya datos sin leer, el programa no arranca misiones nuevas (no los pisa).
bool hasUnread();
void markRead();

// age 0 = la corrida más reciente, 1 = la anterior...
bool readHeader(uint8_t age, RunHeader& header);
bool readObject(uint8_t age, uint8_t index, ObjectRecord& obj);
uint8_t maxObjects();
}  // namespace runlog

#endif
