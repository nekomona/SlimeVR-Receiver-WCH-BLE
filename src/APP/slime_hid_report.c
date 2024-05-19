#include "CONFIG.h"
#include <stdbool.h>

// Use 2B pack, transmit 19B with the first one dropped
// HOGP will fill report ID into the first byte to form 20B packet
#pragma pack(2)
struct HidSensorReport {
	uint8_t __type; // Replaced by BLE HOGP report ID
	uint8_t sensorId;
	uint8_t rssi;
	uint8_t battPerc;
	uint16_t battMV;
	uint16_t q[4]; // w, x, y, z
	uint16_t a[3]; // x, y, z
};

struct HidSensorReport reports[32+4];
int report_count = 0;
int report_sent = 0;

extern void hid_start_write(const void * src, int len);
extern bool hid_is_busy();

void send_report() {
    if (report_count == 0) return;

    if (!hid_is_busy()) {
        // Pad report to 3x20B if <3 reports to send
        for (int i = report_count; i < 3; i++) {
            reports[report_sent + i] = reports[report_sent];
        }

        hid_start_write(&reports[report_sent], 3 * sizeof(struct HidSensorReport));

        if (report_count > 3) {
            PRINT("left %d rpt\n", report_count-3);
        }

        report_sent += 3;
        if (report_sent > 32) report_sent = 0;
        report_count = 0;
    }
}

void push_report(uint8_t sensorid, uint8_t rssi, const void * reportdata) {
    if (report_sent + report_count < 32) {
        struct HidSensorReport * rpt = &reports[report_sent+report_count];

        rpt->__type = 0;

        tmos_memcpy(rpt+1, reportdata, sizeof(struct HidSensorReport)-1);
        rpt->sensorId = (sensorid << 4) | (rpt->sensorId & 0x0F);
        rpt->rssi = rssi;

        report_count++;
    } else {
        PRINT("OVF\n");
        report_count = 0;
    }
}

