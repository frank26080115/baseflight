#include "board.h"
#include "mw.h"
#include <string.h>
#include "frank.h"

void resetConfForFrank(void)
{
    int i;

    featureSet(FEATURE_SERIALRX);
    featureSet(FEATURE_SOFTSERIAL);
    featureSet(FEATURE_GPS);
    featureSet(FEATURE_TELEMETRY);

    mcfg.serialrx_type = 2;
    mcfg.sbus_offset = 1004;
    mcfg.telemetry_provider = TELEMETRY_PROVIDER_SMARTPORT;
    mcfg.gps_port = GPS_PORT_SOFTSERIAL_1;
    mcfg.gps_baudrate = GPS_BAUD_19200;
    mcfg.gps_type = 1;
    mcfg.softserial_1_baudrate = 19200;

    cfg.activate[BOXARM]     = (1 << 0) << (3 * 0);
    cfg.activate[BOXANGLE]   = (1 << 0) << (3 * 2);
    cfg.activate[BOXHORIZON] = (1 << 2) << (3 * 2);

    // copy default config into all 3 profiles
    for (i = 0; i < 3; i++)
        memcpy(&mcfg.profile[i], &cfg, sizeof(config_t));
}