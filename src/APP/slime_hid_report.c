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

#define SLIME_REPORT_BUFF 32

struct HidSensorReport reports[SLIME_REPORT_BUFF];
int report_count = 0;

extern void hid_start_write(const void * src, int len);
extern bool hid_is_busy();

void send_report() {
    if (report_count == 0) return;

    if (!hid_is_busy()) {
        // Pad report to 3x20B if <3 reports to send
        for (int i = report_count; i < 3; i++) {
            reports[i] = reports[0];
        }

        hid_start_write(reports, 3 * sizeof(struct HidSensorReport));

        if (report_count > 3) {
            PRINT("left %d rpt\n", report_count-3);
        }

        report_count = 0;
    }
}

void push_report(uint8_t sensorid, uint8_t rssi, const void * reportdata) {
    uint8_t combined_sensorid = sensorid << 4 | (((uint8_t*)reportdata)[0] & 0x0F);

    for (int i = 0; i < report_count; i++) {
        // Duplicated result, overwrite data in buffer
        if ( reports[+i].sensorId == combined_sensorid ) {
            tmos_memcpy( ((void*)&reports[+i])+3, reportdata+2, sizeof(struct HidSensorReport)-3);
            return;
        }
    }

    if ( report_count < SLIME_REPORT_BUFF) {
        struct HidSensorReport * rpt = &reports[+report_count];
        
        rpt->__type = 0;
        rpt->sensorId = combined_sensorid;
        rpt->rssi = rssi;

        tmos_memcpy(((void *)rpt)+3, reportdata+2, sizeof(struct HidSensorReport)-3);

        report_count++;
        if (report_count == 3) {
            send_report();
        }
    } else {
        PRINT("OVF\n");
    }
}

