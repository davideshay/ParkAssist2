#ifndef BLE_H
#define BLE_H

#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include "NimBLEEddystoneTLM.h"
#include "NimBLEBeacon.h"


void bleLoop();
void initBLE();

#endif