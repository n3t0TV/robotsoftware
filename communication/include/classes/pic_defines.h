#include "libraries/json.hpp"

#define BL_CMD_UNLOCK       0xa0
#define BL_CMD_DATA         0xa1
#define BL_CMD_VERIFY       0xa2
#define BL_CMD_RESET        0xa3
#define BL_CMD_BKSWAP_RESET 0xa4
#define BL_CMD_READY        0Xa5
#define BL_RESP_OK          0x50 //P
#define BL_RESP_ERROR       0x51
#define BL_RESP_INVALID     0x52
#define BL_RESP_CRC_OK      0x53
#define BL_RESP_CRC_FAIL    0x54
#define BL_RESP_INIT        0x55 //U



#define BL_GUARD            0x5048434D

using json = nlohmann::json;

json devices = {
            {"SAME7X"	, {8192, 8192}},
            {"SAME5X"   , {8192, 8192}},
            {"SAMD5X"   , {8192, 8192}},
            {"SAMG5X"   , {8192, 8192}},
            {"SAMC2X"   , {256, 2048}},
            {"SAMD1X"   , {256, 2048}},
            {"SAMD2X"   , {256, 2048}},
            {"SAMDA1"   , {256, 2048}},
            {"SAML1X"   , {256, 2048}},
            {"SAML2X"   , {256, 2048}},
            {"SAMHA1"   , {256, 2048}},
            {"PIC32MK"  , {4096, 8192}},
            {"PIC32MZ"  , {16384, 16384}},
            {"PIC32MX"  , {1024, 4096}}
};
