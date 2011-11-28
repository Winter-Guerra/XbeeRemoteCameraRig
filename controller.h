#ifndef CONTROLLER_H
#define CONTROLLER_H

#if IS_CONTROLLER == 1
void sendTxPositionPacket(uint16_t *stepperPos);

void setupXbeeControllerAddress();

void runControllerSlice();

void setupControllerPins();

void setupControllerSerial();

void readPots();

void eliminateJitter();

void debugPotVals();

void convertToStepPosition();
#endif

#endif
