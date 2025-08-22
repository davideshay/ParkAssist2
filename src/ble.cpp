#include "parkassist.h"
#include "ble.h"
#include "cars.h"

#define ENDIAN_CHANGE_U16(x) ((((x) & 0xFF00) >> 8) + (((x) & 0xFF) << 8))

int         scanTime = 5 * 1000; // In milliseconds
NimBLEScan* pBLEScan;




class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        
        logData("Next Device",false);
        logData(advertisedDevice->toString().c_str(),false);
        // if (advertisedDevice->haveName()) {
        //     logData(advertisedDevice->toString().c_str());
            // logData("Device name: ",false);
            // logData(advertisedDevice->getName().c_str(),false);
            // char strbuf[100];
            // sprintf(strbuf,"RSSI: %d",advertisedDevice->getRSSI());
            // logData(strbuf,false);
//            sprintf(strbuf,"MAC: %s",advertisedDevice->getAddress());
//            logData(strbuf,false);
        // }
        // if (advertisedDevice->haveServiceUUID()) {
        //     NimBLEUUID devUUID = advertisedDevice->getServiceUUID();
        //     logData("Found ServiceUUID: ",false);
        //     logData(devUUID.toString().c_str(),false);
        // } else if (advertisedDevice->haveManufacturerData() == true) {
        //     std::string strManufacturerData = advertisedDevice->getManufacturerData();
        //     if (strManufacturerData.length() == 25 && strManufacturerData[0] == 0x4C && strManufacturerData[1] == 0x00) {
        //         logData("Found an iBeacon!",false);
        //         NimBLEBeacon oBeacon = NimBLEBeacon();
        //         oBeacon.setData(reinterpret_cast<const uint8_t*>(strManufacturerData.data()), strManufacturerData.length());
        //         logData("iBeacon Frame",false);
        //         char strbuf[256];
        //         sprintf(strbuf,"ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",
        //                       oBeacon.getManufacturerId(),
        //                       ENDIAN_CHANGE_U16(oBeacon.getMajor()),
        //                       ENDIAN_CHANGE_U16(oBeacon.getMinor()),
        //                       oBeacon.getProximityUUID().toString().c_str(),
        //                       oBeacon.getSignalPower());
        //         logData(strbuf,false);
        //     } else {
        //         logData("Found another manufacturers beacon!",false);
        //         char strbuf[256];
        //         sprintf(strbuf,"strManufacturerData: %d ", strManufacturerData.length());
        //         logData(strbuf,false);
        //         char strbuf2[256];
        //         for (int i = 0; i < strManufacturerData.length(); i++) {
        //             size_t current_len = strlen(strbuf2);                    
        //             sprintf(strbuf2+current_len,"[%X]", strManufacturerData[i]);
        //         }
        //         logData(strbuf2,false);
        //     }
        //     return;
        // }

        // NimBLEUUID eddyUUID = (uint16_t)0xfeaa;

        // if (advertisedDevice->getServiceUUID().equals(eddyUUID)) {
        //     std::string serviceData = advertisedDevice->getServiceData(eddyUUID);
        //     if (serviceData[0] == 0x20) {
        //         Serial.println("Found an EddystoneTLM beacon!");
        //         NimBLEEddystoneTLM foundEddyTLM = NimBLEEddystoneTLM();
        //         foundEddyTLM.setData(reinterpret_cast<const uint8_t*>(serviceData.data()), serviceData.length());

        //         Serial.printf("Reported battery voltage: %dmV\n", foundEddyTLM.getVolt());
        //         Serial.printf("Reported temperature from TLM class: %.2fC\n", (double)foundEddyTLM.getTemp());
        //         int   temp     = (int)serviceData[5] + (int)(serviceData[4] << 8);
        //         float calcTemp = temp / 256.0f;
        //         Serial.printf("Reported temperature from data: %.2fC\n", calcTemp);
        //         Serial.printf("Reported advertise count: %d\n", foundEddyTLM.getCount());
        //         Serial.printf("Reported time since last reboot: %ds\n", foundEddyTLM.getTime());
        //         Serial.println("\n");
        //         Serial.print(foundEddyTLM.toString().c_str());
        //         Serial.println("\n");
        //     }
        // }
    }
} scanCallbacks;

void initBLE() {
    NimBLEDevice::init("beacon-scanner");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setScanCallbacks(&scanCallbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    // pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL);
    // for (size_t i = 0; i < NUM_CARTYPES; i++)
    // {
    //     if (!cars[i].carBeaconAddress.isNull()) {
    //         NimBLEDevice::whiteListAdd(cars[i].carBeaconAddress);
    //     };
    // }
    
}

void bleLoop() {
    NimBLEScanResults foundDevices = pBLEScan->getResults(scanTime, false);
    char strbuf[100];
    sprintf(strbuf,"Devices Found: %d",foundDevices.getCount());
    logData(strbuf,true);
    logData("Scan done!",true);
    pBLEScan->clearResults(); // delete results scan buffer to release memory
}