#ifndef __SLIME_HID_REPORT
#define __SLIME_HID_REPORT

#include "CONFIG.h"

void send_report();
void push_report(uint8_t sensorid, uint8_t rssi, const void * reportdata);

#endif // __SLIME_HID_REPORT
