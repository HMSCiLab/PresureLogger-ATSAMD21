#pragma once

extern uint16_t initial_pressure;
extern uint16_t last_pressure;

void ReadInitialPressure();
bool ValidatePressure();