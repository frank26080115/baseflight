/*
 * SmartPort Telemetry implementation by frank26080115
 */
#include "board.h"
#include "mw.h"

#include "telemetry_common.h"
#include "telemetry_smartport.h"

enum
{
    FSSP_START_STOP = 0x7E,
    FSSP_DATA_FRAME = 0x10,

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67,
    // reverse engineering tells me that there are plenty more IDs
};

enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_RPM        = 0x050F ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_CAP_USED   = 0x0600 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
};

const uint16_t frSkyDataIdTable[] = {
    FSSP_DATAID_SPEED     ,
    FSSP_DATAID_VFAS      ,
    FSSP_DATAID_CURRENT   ,
    //FSSP_DATAID_RPM       ,
    FSSP_DATAID_ALTITUDE  ,
    FSSP_DATAID_FUEL      ,
    //FSSP_DATAID_ADC1      ,
    //FSSP_DATAID_ADC2      ,
    FSSP_DATAID_LATLONG   ,
    FSSP_DATAID_LATLONG   , // twice
    //FSSP_DATAID_CAP_USED  ,
    FSSP_DATAID_VARIO     ,
    //FSSP_DATAID_CELLS     ,
    //FSSP_DATAID_CELLS_LAST,
    FSSP_DATAID_HEADING   ,
    FSSP_DATAID_ACCX      ,
    FSSP_DATAID_ACCY      ,
    FSSP_DATAID_ACCZ      ,
    FSSP_DATAID_T1        ,
    FSSP_DATAID_T2        ,
    FSSP_DATAID_GPS_ALT   ,
    0
};

#define SMARTPORT_BAUD 57600
#define SMARTPORT_SERVICE_DELAY_MS 5 // telemetry requests comes in at roughly 12 ms intervals, keep this under that
#define SMARTPORT_NOT_CONNECTED_TIMEOUT_MS 5000

uint8_t smartPortIsActive = 0;
char smartPortHasTimedOut = 0;
static uint8_t smartPortHasRequest = 0;
static uint8_t smartPortIdCnt = 0;
static uint32_t smartPortLastRequestTime = 0;
static uint32_t smartPortLastServiceTime = 0;

// this could be used as a callback for USART, but USART1 does not use callbacks
static void smartPortDataReceive(uint16_t c)
{
    uint32_t now = millis();

    // look for a valid request sequence
    static uint8_t lastChar;
    if (lastChar == FSSP_START_STOP) {
        smartPortLastRequestTime = now;
        if ((c == FSSP_SENSOR_ID1) ||
            (c == FSSP_SENSOR_ID2) ||
            (c == FSSP_SENSOR_ID3) ||
            (c == FSSP_SENSOR_ID4)) {
            smartPortHasRequest = 1;
        }
    }
    lastChar = c;
}

static void smartPortSendByte(uint8_t c, uint16_t *crcp)
{
    serialWrite(core.telemport, c);

    if (crcp == NULL)
        return;

    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}

static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    uint16_t crc = 0;
    smartPortSendByte(FSSP_DATA_FRAME, &crc);
    uint8_t *u8p = (uint8_t*)&id;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    u8p = (uint8_t*)&val;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    smartPortSendByte(u8p[2], &crc);
    smartPortSendByte(u8p[3], &crc);
    smartPortSendByte(0xFF - (uint8_t)crc, NULL);

    smartPortLastServiceTime = millis();
}

void freeSmartPortTelemetryPort(void)
{
    if (mcfg.telemetry_port == TELEMETRY_PORT_UART_1) {
        serialInit(mcfg.serial_baudrate);
    }
    smartPortIsActive = 0;
}

void configureSmartPortTelemetryPort(void)
{
    if (mcfg.telemetry_port == TELEMETRY_PORT_UART_1) {
        // USART1 cannot use callbacks
        core.telemport = uartOpen(USART1, NULL, SMARTPORT_BAUD, MODE_BIDIR);
    }
    else if (mcfg.telemetry_port == TELEMETRY_PORT_UART_2) {
        core.telemport = uartOpen(USART2, smartPortDataReceive, SMARTPORT_BAUD, MODE_BIDIR);
    }
    else if (mcfg.telemetry_port == TELEMETRY_PORT_UART_3) {
        core.telemport = uartOpen(USART3, NULL, SMARTPORT_BAUD, MODE_BIDIR);
    }
    smartPortIsActive = 1;
}

bool canSendSmartPortTelemetry(void)
{
    return smartPortIsActive != 0;
}

void handleSmartPortTelemetry(void)
{
    if (!canSendSmartPortTelemetry())
        return;

    if (mcfg.telemetry_port == TELEMETRY_PORT_UART_1 || mcfg.telemetry_port == TELEMETRY_PORT_UART_3) {
        // we are using USART1, which uses DMA for RX, so we call our unregistered callback manually
        // USART3 isn't actually ready for USART yet, no ISR is defined for it, we leave it for I2C
        while (serialTotalBytesWaiting(core.telemport) > 0) {
            uint8_t c = serialRead(core.telemport);
            smartPortDataReceive(c);
        }
    }

    uint32_t now = millis();

    if ((now - smartPortLastRequestTime) > SMARTPORT_NOT_CONNECTED_TIMEOUT_MS) {
        smartPortHasTimedOut = 1;
        return;
    }

    // limit the rate at which we send responses
    if ((now - smartPortLastServiceTime) < SMARTPORT_SERVICE_DELAY_MS)
        return;

    if (smartPortHasRequest) {
        uint16_t id = frSkyDataIdTable[smartPortIdCnt];
        if (id == 0) {
            smartPortIdCnt = 0;
            id = frSkyDataIdTable[smartPortIdCnt];
        }
        smartPortIdCnt++;

        float tmpf;
        int32_t tmpi;
        uint32_t tmpui;
        static uint8_t t1Cnt = 0;

        switch(id) {
            case FSSP_DATAID_SPEED      :
                if (sensors(SENSOR_GPS) && f.GPS_FIX > 0) {
                    tmpf = GPS_speed;
                    tmpf *= 0.36;
                    smartPortSendPackage(id, (uint32_t)roundf(tmpf)); // given in 0.1 m/s, provide in KM/H
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_VFAS       :
                smartPortSendPackage(id, vbat); // given in 0.1V, unknown requested unit
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_CURRENT    :
                smartPortSendPackage(id, amperage); // given in 10mA steps, unknown requested unit
                smartPortHasRequest = 0;
                break;
            //case FSSP_DATAID_RPM        :
            case FSSP_DATAID_ALTITUDE   :
                smartPortSendPackage(id, BaroAlt); // unknown given unit, requested 100 = 1 meter
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_FUEL       :
                smartPortSendPackage(id, mAhdrawn); // given in mAh, unknown requested unit
                smartPortHasRequest = 0;
                break;
            //case FSSP_DATAID_ADC1       :
            //case FSSP_DATAID_ADC2       :
            case FSSP_DATAID_LATLONG    :
                if (sensors(SENSOR_GPS) && f.GPS_FIX > 0) {
                    tmpui = 0;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track                    
                    if (smartPortIdCnt & 1) {
                        tmpui = tmpi = GPS_coord[LON];
                        if (tmpi < 0) {
                            tmpui = -tmpi;
                            tmpui |= 0x40000000;
                        }
                        tmpui |= 0x80000000;
                    }
                    else {
                        tmpui = tmpi = GPS_coord[LAT];
                        if (tmpi < 0) {
                            tmpui = -tmpi;
                            tmpui |= 0x40000000;
                        }
                    }
                    smartPortSendPackage(id, tmpui);
                    smartPortHasRequest = 0;
                }
                break;
            //case FSSP_DATAID_CAP_USED   :
            case FSSP_DATAID_VARIO      :
                smartPortSendPackage(id, vario); // unknown given unit but requested in 100 = 1m/s
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_HEADING    :
                smartPortSendPackage(id, heading / 10); // given in 0.1 deg, requested in 10000 = 100 deg
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCX       :
                smartPortSendPackage(id, accSmooth[X]); // unknown input and unknown output unit, may be requesting a delta value
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCY       :
                smartPortSendPackage(id, accSmooth[Y]);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCZ       :
                smartPortSendPackage(id, accSmooth[Y]);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T1         :
                // we send all the flags as decimal digits for easy reading

                // the t1Cnt simply allows the telemetry view to show at least some changes
                t1Cnt++;
                if (t1Cnt >= 4) {
                    t1Cnt = 1;
                }
                tmpi = t1Cnt * 10000; // start off with at least one digit so the most significant 0 won't be cut off
                // the Taranis seems to be able to fit 5 digits on the screen
                // the Taranis seems to consider this number a signed 16 bit integer

                if (f.OK_TO_ARM)
                    tmpi += 1;
                else if (f.ARMED)
                    tmpi += 2;

                if (f.ANGLE_MODE)
                    tmpi += 10;
                if (f.HORIZON_MODE)
                    tmpi += 20;

                if (f.MAG_MODE)
                    tmpi += 100;
                if (f.BARO_MODE)
                    tmpi += 200;
                if (f.VARIO_MODE)
                    tmpi += 400;

                if (f.GPS_HOLD_MODE)
                    tmpi += 1000;
                if (f.GPS_HOME_MODE)
                    tmpi += 2000;
                if (f.HEADFREE_MODE)
                    tmpi += 4000;

                smartPortSendPackage(id, (uint32_t)tmpi);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T2         :
                if (sensors(SENSOR_GPS)) {
                    // provide GPS lock status
                    smartPortSendPackage(id, (f.GPS_FIX * 1000) + (f.GPS_FIX_HOME * 2000) + GPS_numSat);
                    smartPortHasRequest = 0;
                }
                else {
                    smartPortSendPackage(id, 0);
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_GPS_ALT    :
                if (sensors(SENSOR_GPS) && f.GPS_FIX > 0) {
                    smartPortSendPackage(id, GPS_altitude * 1000); // given in 0.1m , requested in 100 = 1m
                    smartPortHasRequest = 0;
                }
                break;
            default:
                break;
                // if nothing is sent, smartPortHasRequest isn't cleared, we already incremented the counter, just wait for the next loop
        }
    }
}

