#include <EEPROM.h>

#include "runlog.h"

// Mapa de la EEPROM:
//   0-1  'L' 'R'       marca de formato
//   2    versión
//   3    slot de la corrida más reciente
//   4-5  contador de corridas
//   6    1 = hay una corrida sin leer
//   16-  SLOTS bloques: RunHeader + ObjectRecord[]

namespace {
constexpr uint8_t MAGIC0 = 'L';
constexpr uint8_t MAGIC1 = 'R';
constexpr uint8_t VERSION = 3;  // cambiar si cambia RunHeader u ObjectRecord (borra lo guardado)

constexpr int ADDR_MAGIC = 0;
constexpr int ADDR_VERSION = 2;
constexpr int ADDR_LAST_SLOT = 3;
constexpr int ADDR_RUN_COUNTER = 4;
constexpr int ADDR_UNREAD = 6;
constexpr int SLOTS_BASE = 16;
constexpr int EEPROM_SIZE = E2END + 1;
constexpr int SLOT_SIZE = (EEPROM_SIZE - SLOTS_BASE) / runlog::SLOTS;
constexpr uint8_t MAX_OBJECTS = (SLOT_SIZE - sizeof(RunHeader)) / sizeof(ObjectRecord);

uint8_t currentSlot = 0;
RunHeader current = {};

int headerAddr(uint8_t slot) {
    return SLOTS_BASE + slot * SLOT_SIZE;
}

int objectAddr(uint8_t slot, uint8_t index) {
    return headerAddr(slot) + sizeof(RunHeader) + index * sizeof(ObjectRecord);
}

uint8_t slotForAge(uint8_t age) {
    const uint8_t last = EEPROM.read(ADDR_LAST_SLOT) % runlog::SLOTS;
    return (last + runlog::SLOTS - (age % runlog::SLOTS)) % runlog::SLOTS;
}

void saveHeader() {
    EEPROM.put(headerAddr(currentSlot), current);  // put solo escribe los bytes que cambiaron
}
}  // namespace

namespace runlog {

void begin() {
    if (EEPROM.read(ADDR_MAGIC) != MAGIC0 || EEPROM.read(ADDR_MAGIC + 1) != MAGIC1 ||
        EEPROM.read(ADDR_VERSION) != VERSION) {
        clear();
    }
}

void clear() {
    EEPROM.update(ADDR_MAGIC, MAGIC0);
    EEPROM.update(ADDR_MAGIC + 1, MAGIC1);
    EEPROM.update(ADDR_VERSION, VERSION);
    EEPROM.update(ADDR_LAST_SLOT, SLOTS - 1);  // así la próxima corrida va al slot 0
    EEPROM.put(ADDR_RUN_COUNTER, (uint16_t)0);
    EEPROM.update(ADDR_UNREAD, 0);
    const RunHeader empty = {};
    for (uint8_t slot = 0; slot < SLOTS; ++slot) {
        EEPROM.put(headerAddr(slot), empty);
    }
}

void startRun() {
    uint16_t counter = 0;
    EEPROM.get(ADDR_RUN_COUNTER, counter);
    ++counter;
    EEPROM.put(ADDR_RUN_COUNTER, counter);

    currentSlot = (EEPROM.read(ADDR_LAST_SLOT) + 1) % SLOTS;
    current = {};
    current.runId = counter;
    current.status = RUN_INCOMPLETE;
    saveHeader();
    EEPROM.update(ADDR_LAST_SLOT, currentSlot);
    EEPROM.update(ADDR_UNREAD, 1);
}

bool hasUnread() {
    return EEPROM.read(ADDR_UNREAD) == 1;
}

void markRead() {
    EEPROM.update(ADDR_UNREAD, 0);
}

bool addObject(const ObjectRecord& obj) {
    if (current.objectCount >= MAX_OBJECTS) {
        return false;
    }
    EEPROM.put(objectAddr(currentSlot, current.objectCount), obj);
    ++current.objectCount;
    saveHeader();
    return true;
}

void update(uint16_t durationDs, uint16_t readings) {
    current.durationDs = durationDs;
    current.readings = readings;
    saveHeader();
}

void finish(RunStatus status, uint16_t durationDs, uint16_t readings,
            uint16_t centerHalfMs, uint16_t targetMm, uint16_t finalMm,
            uint8_t centerSamples, uint8_t centerTries) {
    current.status = status;
    current.durationDs = durationDs;
    current.readings = readings;
    current.centerHalfMs = centerHalfMs;
    current.targetMm = targetMm;
    current.finalMm = finalMm;
    current.centerSamples = centerSamples;
    current.centerTries = centerTries;
    saveHeader();
}

bool readHeader(uint8_t age, RunHeader& header) {
    EEPROM.get(headerAddr(slotForAge(age)), header);
    return header.status != RUN_EMPTY;
}

bool readObject(uint8_t age, uint8_t index, ObjectRecord& obj) {
    RunHeader header;
    if (!readHeader(age, header) || index >= header.objectCount) {
        return false;
    }
    EEPROM.get(objectAddr(slotForAge(age), index), obj);
    return true;
}

uint8_t maxObjects() {
    return MAX_OBJECTS;
}

}  // namespace runlog
