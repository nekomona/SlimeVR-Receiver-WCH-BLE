/********************************** (C) COPYRIGHT *******************************
* File Name          : central.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/08/06
* Description        : �������̣�����ɨ����Χ�豸�������������Ĵӻ��豸��ַ��
*                      Ѱ���Զ������������ִ�ж�д�������ӻ��������ʹ��,
                       �����ӻ��豸��ַ�޸�Ϊ������Ŀ���ַ��Ĭ��Ϊ(84:C2:E4:03:02:02)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "gattprofile.h"
#include "central.h"
#include "slime_hid_report.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                      15

/*********************************************************************
 * CONSTANTS
 */
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                32

// Scan duration in 0.625ms
#define DEFAULT_SCAN_DURATION               6400

// Connection min interval in 1.25ms
#define DEFAULT_MIN_CONNECTION_INTERVAL     6

// Connection max interval in 1.25ms
#define DEFAULT_MAX_CONNECTION_INTERVAL     6

// Connection supervision timeout in 10ms
#define DEFAULT_CONNECTION_TIMEOUT          600

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE              DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN       TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST        FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE        FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST             FALSE

// Default read RSSI period in 0.625ms
#define DEFAULT_RSSI_PERIOD                 3200

// Minimum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL    6

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL    6

// Slave latency to use parameter update
#define DEFAULT_UPDATE_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
#define DEFAULT_UPDATE_CONN_TIMEOUT         600

// Default passcode
#define DEFAULT_PASSCODE                    0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                   TRUE

// Default bonding mode, TRUE to bond, max bonding 6 devices
#define DEFAULT_BONDING_MODE                TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES             GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Default service discovery timer delay in 0.625ms
#define DEFAULT_SVC_DISCOVERY_DELAY         1600

// Default parameter update delay in 0.625ms
#define DEFAULT_PARAM_UPDATE_DELAY          3200

// Default phy update delay in 0.625ms
#define DEFAULT_PHY_UPDATE_DELAY            2400

// Default read or write timer delay in 0.625ms
#define DEFAULT_READ_OR_WRITE_DELAY         1600

// Default write CCCD delay in 0.625ms
#define DEFAULT_WRITE_CCCD_DELAY            3200

// Establish link timeout in 0.625ms
#define ESTABLISH_LINK_TIMEOUT              3200

// Application states
enum
{
    BLE_STATE_IDLE,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
    BLE_DISC_STATE_IDLE, // Idle
    BLE_DISC_STATE_SVC,  // Service discovery
    BLE_DISC_STATE_CHAR, // Characteristic discovery
    BLE_DISC_STATE_CCCD  // client characteristic configuration discovery
};
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8_t centralTaskId;

// Number of scan results
static uint8_t centralScanRes;

// Scan result list
static gapDevRec_t centralDevList[DEFAULT_MAX_SCAN_RES];

// Peer device address
static peerAddrDefItem_t PeerAddrDef[CENTRAL_MAX_CONNECTION] = {
    {{0x83, 0x38, 0x10, 0xA7, 0x14, 0xD4}},
    {{0x73, 0x38, 0x10, 0xA7, 0x14, 0xD4}},
};

// Connection item list
static centralConnItem_t centralConnList[CENTRAL_MAX_CONNECTION];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void centralProcessGATTMsg(gattMsgEvent_t *pMsg);
static void centralRssiCB(uint16_t connHandle, int8_t rssi);
static void centralEventCB(gapRoleEvent_t *pEvent);
static void centralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets);
static void centralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                              uint8_t uiInputs, uint8_t uiOutputs);
static void centralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void central_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void centralGATTDiscoveryEvent(uint8_t connItem, gattMsgEvent_t *pMsg);
static void centralAddDeviceInfo(uint8_t *pAddr, uint8_t addrType);

static uint16_t connect0_ProcessEvent(uint8_t connItem, uint16_t events);
static void     centralConnIistStartDiscovery_0(uint8_t connItem);
static void     centralInitConnItem(centralConnItem_t *centralConnList);
static uint8_t  centralAddrCmp(peerAddrDefItem_t *PeerAddrDef, uint8_t *addr);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t centralRoleCB = {
    centralRssiCB,        // RSSI callback
    centralEventCB,       // Event callback
    centralHciMTUChangeCB // MTU change callback
};

// Bond Manager Callbacks
static gapBondCBs_t centralBondCB = {
    centralPasscodeCB,
    centralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Central_Init
 *
 * @brief   Initialization function for the Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Central_Init()
{
    centralTaskId = TMOS_ProcessEventRegister(Central_ProcessEvent);

    // Setup GAP
    GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_MIN_CONNECTION_INTERVAL);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_MAX_CONNECTION_INTERVAL);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONNECTION_TIMEOUT);

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = DEFAULT_PASSCODE;
        uint8_t  pairMode = DEFAULT_PAIRING_MODE;
        uint8_t  mitm = DEFAULT_MITM_MODE;
        uint8_t  ioCap = DEFAULT_IO_CAPABILITIES;
        uint8_t  bonding = DEFAULT_BONDING_MODE;

        GAPBondMgr_SetParameter(GAPBOND_CENT_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_CENT_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_CENT_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_CENT_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_CENT_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Init Connection Item
    centralInitConnItem(centralConnList);
    // Initialize GATT Client
    GATT_InitClient();
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(centralTaskId);
    // Setup a delayed profile startup
    tmos_set_event(centralTaskId, START_DEVICE_EVT);
}

static void centralInitConnItem(centralConnItem_t *centralConnList)
{
    uint8_t connItem;
    for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
    {
        centralConnList[connItem].taskID = TMOS_ProcessEventRegister(Central_ProcessEvent);
        centralConnList[connItem].connHandle = GAP_CONNHANDLE_INIT;
        centralConnList[connItem].state = BLE_STATE_IDLE;
        centralConnList[connItem].discState = BLE_DISC_STATE_IDLE;
        centralConnList[connItem].procedureInProgress = FALSE;
        centralConnList[connItem].charHdl = 0;
        centralConnList[connItem].svcStartHdl = 0;
        centralConnList[connItem].svcEndHdl = 0;
        centralConnList[connItem].cccHdl = 0;
        centralConnList[connItem].lastRssi = 0;
        centralConnList[connItem].deviceIndex = 255;
    }
}

/*********************************************************************
 * @fn      Central_ProcessEvent
 *
 * @brief   Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Central_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(centralTaskId)) != NULL)
        {
            central_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        // Start the Device
        GAPRole_CentralStartDevice(centralTaskId, &centralBondCB, &centralRoleCB);
        return (events ^ START_DEVICE_EVT);
    }

    if(events & ESTABLISH_LINK_TIMEOUT_EVT)
    {
        GAPRole_TerminateLink(INVALID_CONNHANDLE);
        return (events ^ ESTABLISH_LINK_TIMEOUT_EVT);
    }

    if(events & SEND_HID_REPORT_EVT) {
        send_report();
        return (events ^ SEND_HID_REPORT_EVT);
    }

    uint8_t connItem;
    for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
    {
        if (task_id == centralConnList[connItem].taskID)
        {
            return connect0_ProcessEvent(connItem, events);
        }
    }

    // Discard unknown events
    return 0;
}

static uint16_t connect0_ProcessEvent(uint8_t connItem, uint16_t events)
{
    if(events & START_SVC_DISCOVERY_EVT)
    {
        // start service discovery
        centralConnIistStartDiscovery_0(connItem);
        return (events ^ START_SVC_DISCOVERY_EVT);
    }

    if(events & START_PARAM_UPDATE_EVT)
    {
        // start connect parameter update
        GAPRole_UpdateLink(centralConnList[connItem].connHandle,
                           DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                           DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                           DEFAULT_UPDATE_SLAVE_LATENCY,
                           DEFAULT_UPDATE_CONN_TIMEOUT);
        return (events ^ START_PARAM_UPDATE_EVT);
    }

    if(events & START_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("PHY Update %x...\n", GAPRole_UpdatePHY(centralConnList[connItem].connHandle, 0, 
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));

        return (events ^ START_PHY_UPDATE_EVT);
    }

    if(events & START_WRITE_CCCD_EVT)
    {
        // Write CCCD as 0x01, 0x00 to enable notify
        if(centralConnList[connItem].procedureInProgress == FALSE)
        {
            // Do a write
            attWriteReq_t req;

            req.cmd = FALSE;
            req.sig = FALSE;
            req.handle = centralConnList[connItem].cccHdl;
            req.len = 2;
            req.pValue = GATT_bm_alloc(centralConnList[connItem].connHandle, ATT_WRITE_REQ, req.len, NULL, 0);
            if(req.pValue != NULL)
            {
                req.pValue[0] = 1;
                req.pValue[1] = 0;

                if(GATT_WriteCharValue(centralConnList[connItem].connHandle, &req, centralTaskId) == SUCCESS)
                {
                    centralConnList[connItem].procedureInProgress = TRUE;
                }
                else
                {
                    GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
                }
            }
        }
        return (events ^ START_WRITE_CCCD_EVT);
    }

    if(events & START_READ_RSSI_EVT)
    {
        GAPRole_ReadRssiCmd(centralConnList[connItem].connHandle);
        tmos_start_task(centralConnList[connItem].taskID, START_READ_RSSI_EVT, DEFAULT_RSSI_PERIOD);
        return (events ^ START_READ_RSSI_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      central_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void central_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GATT_MSG_EVENT:
            centralProcessGATTMsg((gattMsgEvent_t *)pMsg);
            break;
    }
}

/*********************************************************************
 * @fn      centralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void centralProcessGATTMsg(gattMsgEvent_t *pMsg)
{
    uint8_t connItem;
    for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
    {
        if(centralConnList[connItem].connHandle == pMsg->connHandle)
            break;
    }
    if(connItem == CENTRAL_MAX_CONNECTION)
    {
        return;
        // Should not go there
    }

    if(centralConnList[connItem].state != BLE_STATE_CONNECTED)
    {
        // In case a GATT message came after a connection has dropped,
        // ignore the message
        GATT_bm_free(&pMsg->msg, pMsg->method);
        return;
    }

    if((pMsg->method == ATT_EXCHANGE_MTU_RSP) ||
       ((pMsg->method == ATT_ERROR_RSP) &&
        (pMsg->msg.errorRsp.reqOpcode == ATT_EXCHANGE_MTU_REQ)))
    {
        if(pMsg->method == ATT_ERROR_RSP)
        {
            uint8_t status = pMsg->msg.errorRsp.errCode;

            PRINT("Exchange MTU Error: %x\n", status);
        }
        centralConnList[connItem].procedureInProgress = FALSE;
    }

    if(pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        PRINT("MTU: %x\n", pMsg->msg.mtuEvt.MTU);
    }

    if((pMsg->method == ATT_READ_RSP) ||
       ((pMsg->method == ATT_ERROR_RSP) &&
        (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
        if(pMsg->method == ATT_ERROR_RSP)
        {
            uint8_t status = pMsg->msg.errorRsp.errCode;

            PRINT("Read Error: %x\n", status);
        }
        else
        {
            // After a successful read, display the read value
            PRINT("Read rsp: %x\n", *pMsg->msg.readRsp.pValue);
        }
        centralConnList[connItem].procedureInProgress = FALSE;
    }
    else if((pMsg->method == ATT_WRITE_RSP) ||
            ((pMsg->method == ATT_ERROR_RSP) &&
             (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
        if(pMsg->method == ATT_ERROR_RSP)
        {
            uint8_t status = pMsg->msg.errorRsp.errCode;

            PRINT("Write Error: 0x%x 0x%x 0x%x\n", status, pMsg->msg.errorRsp.handle, pMsg->msg.errorRsp.reqOpcode);
        }
        else
        {
            // Write success
            PRINT("Write success \n");
        }

        centralConnList[connItem].procedureInProgress = FALSE;
    }
    else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
        int len = pMsg->msg.handleValueNoti.len;
        if (len == 19) {
            push_report(centralConnList[connItem].deviceIndex, centralConnList[connItem].lastRssi, pMsg->msg.handleValueNoti.pValue);
            tmos_set_event(centralTaskId, SEND_HID_REPORT_EVT);
        }
        /*
        PRINT("Receive noti: %x - %d|", connItem, len);
        for (int i = 0; i < len; i++) {
            PRINT("%x ", pMsg->msg.handleValueNoti.pValue[i]);
        }
        PRINT("\n");
        */
    }
    else if(centralConnList[connItem].discState != BLE_DISC_STATE_IDLE)
    {
        centralGATTDiscoveryEvent(connItem, pMsg);
    }
    GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      centralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void centralRssiCB(uint16_t connHandle, int8_t rssi)
{
    uint8_t connItem;
    for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
    {
        if(centralConnList[connItem].connHandle == connHandle) {
            centralConnList[connItem].lastRssi = rssi;
            PRINT("Dev %d RSSI : -%d dB\n", connItem, -rssi);
            break;
        }
    }
    if(connItem == CENTRAL_MAX_CONNECTION)
    {
        return;
        // Should not go there
    }


}

/*********************************************************************
 * @fn      centralHciMTUChangeCB
 *
 * @brief   MTU changed callback.
 *
 * @param   maxTxOctets - Max tx octets
 * @param   maxRxOctets - Max rx octets
 *
 * @return  none
 */
static void centralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets)
{
    PRINT(" HCI data length changed, Tx: %d, Rx: %d, conn %x\n", maxTxOctets, maxRxOctets, connHandle);
}

static uint8_t centralAddrCmp(peerAddrDefItem_t *PeerAddrDef, uint8_t *addr)
{
    uint8_t i;
    for(i = 0; i < CENTRAL_MAX_CONNECTION; i++)
    {
        if(tmos_memcmp(PeerAddrDef[i].peerAddr, addr, B_ADDR_LEN))
            break;
    }
    if(i == CENTRAL_MAX_CONNECTION)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/*********************************************************************
 * @fn      centralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void centralEventCB(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            PRINT("Discovering...\n");
            GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);
        }
        break;

        case GAP_DEVICE_INFO_EVENT:
        {
            // Add device to list
            centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType);
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
        {
            uint8_t i;
            uint8_t found = 0;

            // See if peer device has been discovered
            for(i = 0; i < centralScanRes; i++)
            {
                if(centralAddrCmp(PeerAddrDef, centralDevList[i].addr)) {
                    PRINT("Device %d found...\n", i);
                    GAPRole_CentralEstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                DEFAULT_LINK_WHITE_LIST,
                                                centralDevList[i].addrType,
                                                centralDevList[i].addr);

                    // Start establish link timeout event
                    tmos_start_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT, ESTABLISH_LINK_TIMEOUT);
                    found = 1;
                }
            }

            // Peer device not found
            if(found == 0)
            {
                PRINT("Device not found...\n");
                centralScanRes = 0;
                GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                              DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                              DEFAULT_DISCOVERY_WHITE_LIST);
                PRINT("Discovering...\n");
            }
        }
        break;

        case GAP_LINK_ESTABLISHED_EVENT:
        {
            tmos_stop_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT);
            if(pEvent->gap.hdr.status == SUCCESS)
            {
                uint8_t connItem;
                for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
                {
                    if(centralConnList[connItem].connHandle == GAP_CONNHANDLE_INIT)
                        break;
                }
                if(connItem == CENTRAL_MAX_CONNECTION)
                {
                    GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
                    PRINT("Connection max...\n");
                }
                else
                {
                    centralConnList[connItem].state = BLE_STATE_CONNECTED;
                    centralConnList[connItem].connHandle = pEvent->linkCmpl.connectionHandle;
                    centralConnList[connItem].procedureInProgress = TRUE;

                    for (int i = 0; i < CENTRAL_MAX_CONNECTION; i++) {
                        if(tmos_memcmp(PeerAddrDef[i].peerAddr, pEvent->linkCmpl.devAddr, B_ADDR_LEN) == TRUE) {
                            centralConnList[connItem].deviceIndex = i;
                            break;
                        }
                    }

                    PRINT("cn %d %d - %02x:%02x:%02x:%02x:%02x:%02x\n", connItem, centralConnList[connItem].deviceIndex,
                            pEvent->linkCmpl.devAddr[0], pEvent->linkCmpl.devAddr[1], pEvent->linkCmpl.devAddr[2],
                            pEvent->linkCmpl.devAddr[3], pEvent->linkCmpl.devAddr[4], pEvent->linkCmpl.devAddr[5]
                        );

                    PRINT("Conn %x - Int %x \n", pEvent->linkCmpl.connectionHandle, pEvent->linkCmpl.connInterval);
                    
                    // Update MTU
                    attExchangeMTUReq_t req = {
                        .clientRxMTU = BLE_BUFF_MAX_LEN - 4,
                    };

                    GATT_ExchangeMTU(centralConnList[connItem].connHandle, &req, centralTaskId);
                    
                    // Initiate service discovery
                    tmos_start_task(centralConnList[connItem].taskID, START_SVC_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY);
                    tmos_start_task(centralConnList[connItem].taskID, START_PARAM_UPDATE_EVT, DEFAULT_PARAM_UPDATE_DELAY);
                    tmos_start_task(centralConnList[connItem].taskID, START_READ_RSSI_EVT, DEFAULT_RSSI_PERIOD);

                    PRINT("Connected...\n");

                    // See if need discover again
                    for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
                    {
                        if(centralConnList[connItem].connHandle == GAP_CONNHANDLE_INIT)
                            break;
                    }
                    if(connItem < CENTRAL_MAX_CONNECTION)
                    {
                        PRINT("Discovering...\n");
                        centralScanRes = 0;
                        GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                                      DEFAULT_DISCOVERY_WHITE_LIST);
                    }
                }
            }
            else
            {
                PRINT("Connect Failed...Reason:%X\n", pEvent->gap.hdr.status);
                PRINT("Discovering...\n");
                centralScanRes = 0;
                GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                              DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                              DEFAULT_DISCOVERY_WHITE_LIST);
            }
        }
        break;

        case GAP_LINK_TERMINATED_EVENT:
        {
            uint8_t connItem;
            for(connItem = 0; connItem < CENTRAL_MAX_CONNECTION; connItem++)
            {
                if(centralConnList[connItem].connHandle == pEvent->linkTerminate.connectionHandle)
                    break;
            }
            if(connItem == CENTRAL_MAX_CONNECTION)
            {
                // Should not go there
            }

            PRINT("  %x  Disconnected...Reason:%x\n", centralConnList[connItem].connHandle, pEvent->linkTerminate.reason);
            centralConnList[connItem].state = BLE_STATE_IDLE;
            centralConnList[connItem].connHandle = GAP_CONNHANDLE_INIT;
            centralConnList[connItem].discState = BLE_DISC_STATE_IDLE;
            centralConnList[connItem].charHdl = 0;
            centralScanRes = 0;
            centralConnList[connItem].procedureInProgress = FALSE;
            tmos_stop_task(centralConnList[connItem].taskID, START_READ_RSSI_EVT);
            PRINT("Discovering...\n");
            GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);
        }
        break;

        case GAP_LINK_PARAM_UPDATE_EVENT:
        {
            PRINT("Update %x - Int %x \n", pEvent->linkUpdate.connectionHandle, pEvent->linkUpdate.connInterval);
        }
        break;

        case GAP_PHY_UPDATE_EVENT:
        {
            PRINT("PHY Update %x\n", pEvent->linkUpdate.connectionHandle);
        }
        break;

        
        case GAP_EXT_ADV_DEVICE_INFO_EVENT:
        {
            // Display device addr
            PRINT("Recv ext adv \n");
            // Add device to list
            centralAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType);
        }
        break;

        case GAP_DIRECT_DEVICE_INFO_EVENT:
        {
            // Display device addr
            PRINT("Recv direct adv \n");
            // Add device to list
            centralAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType);
        }
        break;
        
        default:
            break;
    }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void centralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
    if(state == GAPBOND_PAIRING_STATE_STARTED)
    {
        PRINT("Connection %04x - Pairing started:%d\n", connHandle, status);
    }
    else if(state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if(status == SUCCESS)
        {
            PRINT("Connection %04x - Pairing success\n", connHandle);
        }
        else
        {
            PRINT("Connection %04x - Pairing fail\n", connHandle);
        }
    }
    else if(state == GAPBOND_PAIRING_STATE_BONDED)
    {
        if(status == SUCCESS)
        {
            PRINT("Connection %04x - Bonding success\n", connHandle);
        }
    }
    else if(state == GAPBOND_PAIRING_STATE_BOND_SAVED)
    {
        if(status == SUCCESS)
        {
            PRINT("Connection %04x - Bond save success\n", connHandle);
        }
        else
        {
            PRINT("Connection %04x - Bond save failed: %d\n", connHandle, status);
        }
    }
}

/*********************************************************************
 * @fn      centralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void centralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle,
                              uint8_t uiInputs, uint8_t uiOutputs)
{
    uint32_t passcode;

    // Create random passcode
    passcode = tmos_rand();
    passcode %= 1000000;
    // Display passcode to user
    if(uiOutputs != 0)
    {
        PRINT("Passcode:%06d\n", (int)passcode);
    }
    // Send passcode response
    GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      centralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void centralConnIistStartDiscovery_0(uint8_t connItem)
{
    uint8_t uuid[ATT_BT_UUID_SIZE] = {LO_UINT16(HID_SERV_UUID),
                                      HI_UINT16(HID_SERV_UUID)};

    // Initialize cached handles
    centralConnList[connItem].svcStartHdl = centralConnList[connItem].svcEndHdl = centralConnList[connItem].charHdl = 0;

    centralConnList[connItem].discState = BLE_DISC_STATE_SVC;

    // Discovery simple BLE service
    GATT_DiscPrimaryServiceByUUID(centralConnList[connItem].connHandle,
                                  uuid,
                                  ATT_BT_UUID_SIZE,
                                  centralTaskId);
}

/*********************************************************************
 * @fn      centralGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void centralGATTDiscoveryEvent(uint8_t connItem, gattMsgEvent_t *pMsg)
{
    attReadByTypeReq_t req;

    if(centralConnList[connItem].discState == BLE_DISC_STATE_SVC)
    {
        // Service found, store handles
        if(pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
            centralConnList[connItem].svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
            centralConnList[connItem].svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);

            // Display Profile Service handle range
            PRINT("Found Profile Service handle : %x ~ %x \n", centralConnList[connItem].svcStartHdl, centralConnList[connItem].svcEndHdl);
        }
        // If procedure complete
        if((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->hdr.status == bleProcedureComplete) ||
            (pMsg->method == ATT_ERROR_RSP))
        {
            if(centralConnList[connItem].svcStartHdl != 0)
            {
                // Discover CCCD handles
                centralConnList[connItem].discState = BLE_DISC_STATE_CCCD;
                req.startHandle = centralConnList[connItem].svcStartHdl;
                req.endHandle = centralConnList[connItem].svcEndHdl;
                req.type.len = ATT_BT_UUID_SIZE;
                req.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
                req.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);

                GATT_ReadUsingCharUUID(centralConnList[connItem].connHandle, &req, centralTaskId);
            }
        }
    }
    else if(centralConnList[connItem].discState == BLE_DISC_STATE_CCCD)
    {
        // Characteristic found, store handle
        if(pMsg->method == ATT_READ_BY_TYPE_RSP &&
        pMsg->msg.readByTypeRsp.numPairs > 0)
        {
            centralConnList[connItem].cccHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                        pMsg->msg.readByTypeRsp.pDataList[1]);

            centralConnList[connItem].procedureInProgress = FALSE;

            // Start do write CCCD
            tmos_start_task(centralConnList[connItem].taskID, START_WRITE_CCCD_EVT, DEFAULT_WRITE_CCCD_DELAY);

            // Display Characteristic 1 handle
            PRINT("Found client characteristic configuration handle : %x \n", centralConnList[connItem].cccHdl);
        }
        centralConnList[connItem].discState = BLE_DISC_STATE_IDLE;
    }
}

/*********************************************************************
 * @fn      centralAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void centralAddDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    // If result count not at max
    if(centralScanRes < DEFAULT_MAX_SCAN_RES)
    {
        // Check if device is already in scan results
        for(i = 0; i < centralScanRes; i++)
        {
            if(tmos_memcmp(pAddr, centralDevList[i].addr, B_ADDR_LEN))
            {
                return;
            }
        }
        // Add addr to scan result list
        tmos_memcpy(centralDevList[centralScanRes].addr, pAddr, B_ADDR_LEN);
        centralDevList[centralScanRes].addrType = addrType;
        // Increment scan result count
        centralScanRes++;
        // Display device addr
        PRINT("Device %d - Addr %02x %02x %02x %02x %02x %02x \n", centralScanRes,
              centralDevList[centralScanRes - 1].addr[0],
              centralDevList[centralScanRes - 1].addr[1],
              centralDevList[centralScanRes - 1].addr[2],
              centralDevList[centralScanRes - 1].addr[3],
              centralDevList[centralScanRes - 1].addr[4],
              centralDevList[centralScanRes - 1].addr[5]);
    }
}

/************************ endfile @ central **************************/
