#include <Arduino.h>

#include <errno.h>

#include "SPI.h"

#include "rfal_platform/rfal_platform.h"


extern "C" {
#include "rfal_core/rfal_nfc.h"             // Includes all of "rfal_nfc[a|b|f|v].h", "rfal_isoDep.h" and "rfal_nfcDep.h".
#include "rfal_core/rfal_t2t.h"
#include "rfal_core/rfal_analogConfig.h"

#include "ndef_dump.h"
}


// REFERENCE: https://www.st.com/resource/en/user_manual/um2890-rfnfc-abstraction-layer-rfal-stmicroelectronics.pdf
// ISO 15693
// ISO 1443(A)


#define REVERSE_BYTES(pData, nDataSize) \
  {unsigned char swap, *lo = ((unsigned char *)(pData)), *hi = ((unsigned char *)(pData)) + (nDataSize) - 1; \
  while (lo < hi) { swap = *lo; *lo++ = *hi; *hi-- = swap; }}



#define DEMO_NFCV_BLOCK_LEN           4     /*!< NFCV Block len                         */
                                                                                        
#define DEMO_NFCV_USE_SELECT_MODE     false /*!< NFCV // checknstrate select mode           */
#define DEMO_NFCV_WRITE_TAG           false /*!< NFCV // checknstrate Write Single Block    */
    
/* Definition of various Listen Mode constants */
#if defined(DEMO_LISTEN_MODE_TARGET) 
#define DEMO_LM_SEL_RES       0x40U         /*!<NFC-A SEL_RES configured for the NFC-DEP protocol    */
#define DEMO_LM_NFCID2_BYTE1  0x01U         /*!<NFC-F SENSF_RES configured for the NFC-DEP protocol  */ 
#define DEMO_LM_SC_BYTE1      0xFFU         /*!<NFC-F System Code byte 1                             */ 
#define DEMO_LM_SC_BYTE2      0xFFU         /*!<NFC-F System Code byte 2                             */ 
#define DEMO_LM_PAD0          0xFFU         /*!<NFC-F PAD0                                           */ 
#else
#define DEMO_LM_SEL_RES       0x20U         /*!<NFC-A SEL_RES configured for Type 4A Tag Platform    */
#define DEMO_LM_NFCID2_BYTE1  0x02U         /*!<NFC-F SENSF_RES configured for Type 3 Tag Platform   */ 
#define DEMO_LM_SC_BYTE1      0x12U         /*!<NFC-F System Code byte 1                             */ 
#define DEMO_LM_SC_BYTE2      0xFCU         /*!<NFC-F System Code byte 2                             */ 
#define DEMO_LM_PAD0          0x00U         /*!<NFC-F PAD0                                           */ 
#endif


/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define EXAMPLE_RFAL_POLLER_DEVICES      10    /* Number of devices supported */
#define EXAMPLE_RFAL_POLLER_RF_BUF_LEN   255   /* RF buffer length            */

#define EXAMPLE_RFAL_POLLER_FOUND_NONE   0x00  /* No device found Flag        */
#define EXAMPLE_RFAL_POLLER_FOUND_A      0x01  /* NFC-A device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_B      0x02  /* NFC-B device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_F      0x04  /* NFC-F device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_V      0x08  /* NFC-V device Flag           */


/*
******************************************************************************
* GLOBAL TYPES
******************************************************************************
*/

/*! Main state                                                                          */
typedef enum{
    EXAMPLE_RFAL_POLLER_STATE_INIT                =  0,  /* Initialize state            */
    EXAMPLE_RFAL_POLLER_STATE_TECHDETECT          =  1,  /* Technology Detection state  */
    EXAMPLE_RFAL_POLLER_STATE_COLAVOIDANCE        =  2,  /* Collision Avoidance state   */
    EXAMPLE_RFAL_POLLER_STATE_ACTIVATION          =  3,  /* Activation state            */
    EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_START  =  4,  /* Data Exchange Start state   */
    EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_CHECK  =  5,  /* Data Exchange Check state   */
    EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION        =  9   /* Deactivation state          */
}exampleRfalPollerState;


/*! Device type                                                                         */
typedef enum{
    EXAMPLE_RFAL_POLLER_TYPE_NFCA  =  0,                 /* NFC-A device type           */
    EXAMPLE_RFAL_POLLER_TYPE_NFCB  =  1,                 /* NFC-B device type           */
    EXAMPLE_RFAL_POLLER_TYPE_NFCF  =  2,                 /* NFC-F device type           */
    EXAMPLE_RFAL_POLLER_TYPE_NFCV  =  3                  /* NFC-V device type           */
}exampleRfalPollerDevType;


/*! Device interface                                                                    */
typedef enum{
    EXAMPLE_RFAL_POLLER_INTERFACE_RF     = 0,            /* RF Frame interface          */
    EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP = 1,            /* ISO-DEP interface           */
    EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP = 2             /* NFC-DEP interface           */
}exampleRfalPollerRfInterface;


/*! Device struct containing all its details                                            */
typedef struct{
    exampleRfalPollerDevType type;                      /* Device's type                */
    union{
        rfalNfcaListenDevice nfca;                      /* NFC-A Listen Device instance */
        rfalNfcbListenDevice nfcb;                      /* NFC-B Listen Device instance */
        rfalNfcfListenDevice nfcf;                      /* NFC-F Listen Device instance */
        rfalNfcvListenDevice nfcv;                      /* NFC-V Listen Device instance */
    }dev;                                               /* Device's instance            */
    
    exampleRfalPollerRfInterface rfInterface;           /* Device's interface           */
    union{
        rfalIsoDepDevice isoDep;                        /* ISO-DEP instance             */
        rfalNfcDepDevice nfcDep;                        /* NFC-DEP instance             */
    }proto;                                             /* Device's protocol            */
    
}exampleRfalPollerDevice;


/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
static uint8_t                 t1tReadReq[]    = { 0x01, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44 };                                                   /* T1T READ Block:0 Byte:0 */
static uint8_t                 t2tReadReq[]    = { 0x30, 0x00 };                                                                                 /* T2T READ Block:0 */
static uint8_t                 t3tCheckReq[]   = { 0x06, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x01, 0x09, 0x00, 0x01, 0x80, 0x00 };   /* T3T Check/Read command */
static uint8_t                 t4tSelectReq[]  = { 0x00, 0xA4, 0x00, 0x00, 0x00 };                                                               /* T4T Select MF, DF or EF APDU  */
static uint8_t                 t5tSysInfoReq[] = { 0x02, 0x2B };                                                                                 /* NFC-V Get SYstem Information command*/
static uint8_t                 nfcbReq[]       = { 0x00 };                                                                                       /* NFC-B proprietary command */
static uint8_t                 llcpSymm[]      = { 0x00, 0x00 };                                                                                 /* LLCP SYMM command */

static uint8_t                 gNfcid3[]       = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A };                                  /* NFCID3 used for ATR_REQ */
static uint8_t                 gGenBytes[]     = { 0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03 }; /* P2P General Bytes: LCCP Connect */

/*******************************************************************************/

static uint8_t                 gDevCnt;                                 /* Number of devices found                         */
static exampleRfalPollerDevice gDevList[EXAMPLE_RFAL_POLLER_DEVICES];   /* Device List                                     */
static exampleRfalPollerState  gState;                                  /* Main state                                      */
static uint8_t                 gTechsFound;                             /* Technologies found bitmask                      */
exampleRfalPollerDevice        *gActiveDev;                             /* Active device pointer                           */
static uint16_t                gRcvLen;                                 /* Received length                                 */
static bool                    gRxChaining;                             /* Rx chaining flag                                */

/*! Transmit buffers union, only one interface is used at a time                                                           */
static union{
    uint8_t                 rfTxBuf[EXAMPLE_RFAL_POLLER_RF_BUF_LEN];    /* RF Tx buffer (not used on this demo)            */
    rfalIsoDepBufFormat     isoDepTxBuf;                                /* ISO-DEP Tx buffer format (with header/prologue) */
    rfalNfcDepBufFormat     nfcDepTxBuf;                                /* NFC-DEP Rx buffer format (with header/prologue) */
}gTxBuf;


/*! Receive buffers union, only one interface is used at a time                                                            */
static union {
    uint8_t                 rfRxBuf[EXAMPLE_RFAL_POLLER_RF_BUF_LEN];    /* RF Rx buffer                                    */
    rfalIsoDepBufFormat     isoDepRxBuf;                                /* ISO-DEP Rx buffer format (with header/prologue) */
    rfalNfcDepBufFormat     nfcDepRxBuf;                                /* NFC-DEP Rx buffer format (with header/prologue) */
}gRxBuf;




/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

/* APDUs communication data */
#if RFAL_FEATURE_ISO_DEP_POLL
static uint8_t ndefSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };
static uint8_t ccSelectFile[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static uint8_t readBinary[] = { 0x00, 0xB0, 0x00, 0x00, 0x0F };

/* For a Payment application a Select PPSE would be needed: 
   ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 } */
#endif /* RFAL_FEATURE_ISO_DEP_POLL */

#if RFAL_FEATURE_NFC_DEP
/* P2P communication data */
static uint8_t ndefLLCPSYMM[] = {0x00, 0x00};
static uint8_t ndefInit[] = {0x05, 0x20, 0x06, 0x0F, 0x75, 0x72, 0x6E, 0x3A, 0x6E, 0x66, 0x63, 0x3A, 0x73, 0x6E, 0x3A, 0x73, 0x6E, 0x65, 0x70, 0x02, 0x02, 0x07, 0x80, 0x05, 0x01, 0x02};
static uint8_t ndefUriSTcom[] = {0x13, 0x20, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x19, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x12, 0x55, 0x00, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d};
#endif /* RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
#if RFAL_SUPPORT_MODE_LISTEN_NFCA
/* NFC-A CE config */
/* 4-byte UIDs with first byte 0x08 would need random number for the subsequent 3 bytes.
 * 4-byte UIDs with first byte 0x*F are Fixed number, not unique, use for this // check
 * 7-byte UIDs need a manufacturer ID and need to assure uniqueness of the rest.*/
static uint8_t ceNFCA_NFCID[]     = {0x5F, 'S', 'T', 'M'};    /* =_STM, 5F 53 54 4D NFCID1 / UID (4 bytes) */
static uint8_t ceNFCA_SENS_RES[]  = {0x02, 0x00};             /* SENS_RES / ATQA for 4-byte UID            */
static uint8_t ceNFCA_SEL_RES     = DEMO_LM_SEL_RES;          /* SEL_RES / SAK                             */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

static uint8_t ceNFCF_nfcid2[]     = {DEMO_LM_NFCID2_BYTE1, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
  /* NFC-F CE config */
static uint8_t ceNFCF_SC[]         = {DEMO_LM_SC_BYTE1, DEMO_LM_SC_BYTE2};
static uint8_t ceNFCF_SENSF_RES[]  = {0x01,                                                       /* SENSF_RES                                */
                                  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,                 /* NFCID2                                   */
                                  DEMO_LM_PAD0, DEMO_LM_PAD0, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00, /* PAD0, PAD1, MRTIcheck, MRTIupdate, PAD2  */
                                  0x00, 0x00 };                                                   /* RD                                       */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */


static rfalNfcDiscoverParam discParam;      // NFC discovery parameters.

bool multiSel = false;                      // Handling of multiple NFC-tags simultaneously.


// Helpers:
static char UID_hex_string[40] = {0};       // Safely hold up to NFCID3 values, i.e. 10-byte (requires 21-byte char-arr)

static char *hex2str(uint8_t *number, uint8_t length)
{
	uint8_t aux_1 = 0;
	uint8_t aux_2 = 0;

	for (uint8_t i=0; i < length; i++)
	{
		aux_1 = number[i] / 16;
		aux_2 = number[i] % 16;

		if (aux_1 < 10)
		{
			UID_hex_string[2*i] = aux_1 + '0';
		}
		else
        {
			UID_hex_string[2*i] = aux_1 + ('A' - 10);
		}

		if (aux_2 < 10)
        {
			UID_hex_string[2*i+1] = aux_2 + '0';
		}
		else
        {
			UID_hex_string[2*i+1] = aux_2 + ('A' - 10);
		}
	} 

	UID_hex_string[length*2] = '\0';

    return((char *)UID_hex_string);
}

struct rfalStateToDescription
{
    rfalNfcState state;
    String desc;
};

#define NUM_RFAL_NFC_STATES     16

static const struct rfalStateToDescription stateToDesc[NUM_RFAL_NFC_STATES] = 
{
    {
        .state = RFAL_NFC_STATE_NOTINIT,
        .desc = "NOT_INITIALIZED",
    },
    {
        .state = RFAL_NFC_STATE_IDLE,
        .desc = "IDLE",
    },
    {
        .state = RFAL_NFC_STATE_START_DISCOVERY,
        .desc = "START_DISCOVERY",
    },
    {
        .state = RFAL_NFC_STATE_WAKEUP_MODE,
        .desc = "WKUP_MODE",
    },
    {
        .state = RFAL_NFC_STATE_POLL_TECHDETECT,
        .desc = "POLL_TECH_DETECT",
    },
    {
        .state = RFAL_NFC_STATE_POLL_COLAVOIDANCE,
        .desc = "POLL_COLL_AVOIDANCE",
    },
    {
        .state = RFAL_NFC_STATE_POLL_SELECT,
        .desc = "POLL_SELECT",
    },
    {
        .state = RFAL_NFC_STATE_POLL_ACTIVATION,
        .desc = "POLL_ACTIVATION",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_TECHDETECT,
        .desc = "LISTEN_TECH_DETECT",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_COLAVOIDANCE,
        .desc = "LISTEN_COLL_AVOIDANCE",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_ACTIVATION,
        .desc = "LISTEN_ACTIVATION",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_SLEEP,
        .desc = "LISTEN_SLEEP",
    },
    {
        .state = RFAL_NFC_STATE_ACTIVATED,
        .desc = "STATE_ACTIVATED",
    },
    {
        .state = RFAL_NFC_STATE_DATAEXCHANGE,
        .desc = "DATA_EXCHANGE_STARTED",
    },
    {
        .state = RFAL_NFC_STATE_DATAEXCHANGE_DONE,
        .desc = "DATA_EXCHANGE_DONE",
    },
    {
        .state = RFAL_NFC_STATE_DEACTIVATION,
        .desc = "DE-ACTIVATION",
    },
};

static const String nonStateDesc = "<invalid state>";

static String state_description(rfalNfcState rfalState)
{
    String stateDesc = nonStateDesc;

    for (int i = 0; i < NUM_RFAL_NFC_STATES; i++)
    {
        struct rfalStateToDescription *entry = (struct rfalStateToDescription *)&stateToDesc[i];

        if (rfalState == entry->state)
        {
            stateDesc = entry->desc;
        }
    }

    return(stateDesc);
}


static void demoNfcv( rfalNfcvListenDevice *nfcvDev )
{
#if RFAL_FEATURE_NFCV
    
    ReturnCode            err;
    uint16_t              rcvLen;
    uint8_t               blockNum = 1;
    uint8_t               rxBuf[ 1 + DEMO_NFCV_BLOCK_LEN + RFAL_CRC_LEN ];                        /* Flags + Block Data + CRC */
    uint8_t               *uid; 
    uint8_t               reqFlag;
#if DEMO_NFCV_WRITE_TAG
    uint8_t               wrData[DEMO_NFCV_BLOCK_LEN] = { 0x11, 0x22, 0x33, 0x99 };             /* Write block example */
#endif /* DEMO_NFCV_WRITE_TAG */
              

    uid     = nfcvDev->InvRes.UID;
    reqFlag = RFAL_NFCV_REQ_FLAG_DEFAULT;
    
    #if DEMO_NFCV_USE_SELECT_MODE
        /*
        * Activate selected state
        */
        err = rfalNfcvPollerSelect( reqFlag, nfcvDev->InvRes.UID );
        Serial0.print(" Select %s \r\n", (err != RFAL_ERR_NONE) ? "FAIL (revert to addressed mode)": "OK" );
        if( err == RFAL_ERR_NONE )
        {
            reqFlag = (RFAL_NFCV_REQ_FLAG_DEFAULT | RFAL_NFCV_REQ_FLAG_SELECT);
            uid     = NULL;
        }
    #endif /* DEMO_NFCV_USE_SELECT_MODE */

    /*
    * Read block using Read Single Block command
    * with addressed mode (uid != NULL) or selected mode (uid == NULL)
    */
    err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
    Serial0.println(" Read Block:");
    Serial0.println( (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:");
    Serial0.println( (err != RFAL_ERR_NONE) ? "" : hex2str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
 
    #if DEMO_NFCV_WRITE_TAG /* Writing example */
        err = rfalNfcvPollerWriteSingleBlock(reqFlag, uid, blockNum, wrData, sizeof(wrData));
        Serial0.print(" Write Block: %s Data: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", hex2Str( wrData, DEMO_NFCV_BLOCK_LEN) );
        err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
        Serial0.print(" Read Block: %s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:", (err != RFAL_ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
    #endif /* DEMO_NFCV_WRITE_TAG */
        
#endif /* RFAL_FEATURE_NFCV */
}


#if 0
// RFAL notification callback.
static void rfalNotifyCb( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;
    
    Serial0.print("RFAL-notify: RFAL NFC-state changed! New state: ");
    Serial0.println( state_description(rfalNfcGetState()) );

    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        Serial0.println("Wake Up mode started ...");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        if( discParam.wakeupEnabled )
        {
            Serial0.println("Wake Up mode terminated. Polling for devices ...");
        }
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Check if in case of multiple devices, selection is already attempted */
        if( (!multiSel) )
        {
            multiSel = true;
            /* Multiple devices were found, activate first of them */
            rfalNfcGetDevicesFound( &dev, &devCnt );
            rfalNfcSelect( 0 );
            
            Serial0.println("Multiple Tags detected! Activating first device. No. of devices: ");
            Serial0.println(devCnt);
        }
        else
        {
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
            Serial0.println("RFAL-notify: de-activating NFC again ...");
        }
    }
    else if( st == RFAL_NFC_STATE_START_DISCOVERY )
    {
        /* Clear multiple device selection flag */
        multiSel = false;
    }
}


void setup() 
{
    Serial0.begin(115200);
    Serial0.println("Init ...");
    
    spi_init();

    // NFC:
    ReturnCode ret = rfalNfcInitialize();      // WAS: 'rfalInitialize()' - but this function is NOT setting NFC-state!!
    
    if (RFAL_ERR_NONE != ret)
    {
        Serial0.println("ERROR: NFC subsystem init failed!\nNFC subsystem init return code:");
        Serial0.println(ret);
        while(1)
        {
            vTaskDelay(1000);
        }
    }

    Serial0.println("NFC subsystem initialized OK...");

    // NFC subsystem init OK - go on to set up polling: 
    rfalNfcDefaultDiscParams( &discParam );
    
    discParam.devLimit      = 1U;

    discParam.wakeupEnabled = false;                        // NOTE: possibly - a better solution is to let user-button switch this TRUE/FALSE during run-time(??).
    
    SMEMCPY( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
    SMEMCPY( &discParam.GB, GB, sizeof(GB) );
    discParam.GBLen         = sizeof(GB);
    discParam.p2pNfcaPrio   = true;

    discParam.notifyCb             = rfalNotifyCb;
    discParam.totalDuration        = 3000U;                         /* Duration of POLL + LISTEN cycle = 1000ms = 1sec. */
    discParam.techs2Find           = RFAL_NFC_TECH_NONE;          /* For the // check, enable the NFC Technologies based on RFAL Feature switches */


#if RFAL_FEATURE_NFCA
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_A;
#endif /* RFAL_FEATURE_NFCA */

#if RFAL_FEATURE_NFCB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_B;
#endif /* RFAL_FEATURE_NFCB */

#if RFAL_FEATURE_NFCF
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_F;
#endif /* RFAL_FEATURE_NFCF */

#if RFAL_FEATURE_NFCV
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_V;
#endif /* RFAL_FEATURE_NFCV */

#if RFAL_FEATURE_ST25TB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_ST25TB;
#endif /* RFAL_FEATURE_ST25TB */

#ifdef ST25R95
        discParam.isoDepFS           = RFAL_ISODEP_FSXI_128;          /* ST25R95 cannot support 256 bytes of data block */
#endif /* ST25R95 */

#if RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP
        discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE
        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE */


#if DEMO_CARD_EMULATION_ONLY
        discParam.totalDuration        = 60000U;              /* 60 seconds */
        discParam.techs2Find           = RFAL_NFC_TECH_NONE;  /* Overwrite any previous poller modes */
#endif /* DEMO_CARD_EMULATION_ONLY */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
        // checkCeInit( ceNFCF_nfcid2 );
    
#if RFAL_SUPPORT_MODE_LISTEN_NFCA
        /* Set configuration for NFC-A CE */
        SMEMCPY( discParam.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN );     /* Set SENS_RES / ATQA */
        SMEMCPY( discParam.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04 );           /* Set NFCID / UID */
        discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;                                  /* Set NFCID length to 7 bytes */
        discParam.lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                        /* Set SEL_RES / SAK */

        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
        /* Set configuration for NFC-F CE */
        SMEMCPY( discParam.lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN );                 /* Set System Code */
        SMEMCPY( &ceNFCF_SENSF_RES[RFAL_NFCF_CMD_LEN], ceNFCF_nfcid2, RFAL_NFCID2_LEN );     /* Load NFCID2 on SENSF_RES */
        SMEMCPY( discParam.lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN );  /* Set SENSF_RES / Poll Response */

        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_F;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */

        ret = rfalNfcDiscover( &discParam );
        if (RFAL_ERR_NONE != ret)
        {
            Serial0.println("ERROR: NFC subsystem init failed!\nNFC subsystem init return code:");
            Serial0.println(ret);
            while(1)
            {
                vTaskDelay(1000);
            }
        }

        Serial0.println("Setup of discovery parameters for NFC-scan OK ...");
        Serial0.print("NFC state BEFORE: ");
        Serial0.println( state_description(rfalNfcGetState()) );
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
        Serial0.print("NFC state AFTER: ");
        Serial0.println( state_description(rfalNfcGetState()) );
}


void loop() 
{
    static rfalNfcDevice *nfcDevice;

    Serial0.println("Worker - start scan ...");

    // put your main code here, to run repeatedly:
    rfalNfcWorker(); //TODO: put in a separate thread. NOTE: was 'rfalWorker()'.

    if( rfalNfcIsDevActivated( rfalNfcGetState() ) )
    {
        Serial0.println("ST25R3911B NFC-xcvr active - polling ...");

        rfalNfcGetActiveDevice( &nfcDevice );
        
        switch( nfcDevice->type )
        {
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCA:
            
                platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
                switch( nfcDevice->dev.nfca.type )
                {
                    case RFAL_NFCA_T1T:
                        //platformLog("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        Serial0.print("ISO14443A/Topaz (NFC-A T1T) TAG found. UID:");
                        break;
                    
                    case RFAL_NFCA_T4T:
                        Serial0.print("NFCA Passive ISO-DEP device found. UID: ");
                    
                        // checkAPDU();
                        break;
                    
                    case RFAL_NFCA_T4T_NFCDEP:
                    case RFAL_NFCA_NFCDEP:
                        Serial0.print("NFCA Passive P2P device found. NFCID: ");
                        
                        // checkP2P( nfcDevice );
                        break;
                        
                    default:
                        Serial0.print("ISO14443A/NFC-A card found. UID: ");
                        
                        // checkT2t();
                        break;
                }

                Serial0.println(hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCB:
                
                platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
                Serial0.print("ISO14443B/NFC-B card found. UID: ");
                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
            
                if( rfalNfcbIsIsoDepSupported( &nfcDevice->dev.nfcb ) )
                {
                    // checkAPDU();
                }
                break;
                
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCF:
                
                platformLedOn(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
                if( rfalNfcfIsNfcDepSupported( &nfcDevice->dev.nfcf ) )
                {
                    Serial0.print("NFCF Passive P2P device found. NFCID: ");
                    // checkP2P( nfcDevice );
                }
                else
                {
                    Serial0.print("Felica/NFC-F card found. UID: ");
                    
                    // checkNfcf( &nfcDevice->dev.nfcf );
                }

                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCV:
                {
                    uint8_t devUID[RFAL_NFCV_UID_LEN];
                    
                    platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
                    
                    SMEMCPY( devUID, nfcDevice->nfcid, nfcDevice->nfcidLen );   /* Copy the UID into local var */
                    REVERSE_BYTES( devUID, RFAL_NFCV_UID_LEN );                 /* Reverse the UID for display purposes */
                    Serial0.print("ISO15693/NFC-V card found. UID: ");
                    Serial0.println( hex2str(devUID, RFAL_NFCV_UID_LEN) );      /* NOTE: previous standards had 7-byte(=56-bit) UID, while NFC-V is 8-byte(=64-bit)!! */
                    
                    //checkNfcv( &nfcDevice->dev.nfcv );
                    demoNfcv( &nfcDevice->dev.nfcv );
                }
                break;
                
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_ST25TB:
                
                platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
                Serial0.print("ST25TB card found. UID: ");
                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_AP2P:
            case RFAL_NFC_POLL_TYPE_AP2P:
                
                platformLedOn(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
                Serial0.print("NFC Active P2P device found. NFCID3: ");
                Serial0.println( hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen) );
            
                // checkP2P( nfcDevice );
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_POLL_TYPE_NFCA:
            case RFAL_NFC_POLL_TYPE_NFCF:
                
                platformLedOn( ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PORT : PLATFORM_LED_F_PORT), 
                                ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PIN  : PLATFORM_LED_F_PIN)  );
                Serial0.print("Activated in CE ");
                Serial0.print( (nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? "NFC-A" : "NFC-F" );
                Serial0.println(" mode."); 

                if( nfcDevice->rfInterface == RFAL_NFC_INTERFACE_NFCDEP )
                {
                    Serial0.println("Interface = NFCDEP (P2P-capable) ..."); 
                    // checkP2P( nfcDevice );
                }
                else
                {
                    // checkCE( nfcDevice );
                }
                break;
            
            /*******************************************************************************/
            default:
                Serial0.println("WARN: unrecognized NFC device!(??)");
                break;
        }
        
         //rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
    }
    else
    {
        Serial0.println("INFO: ST25R3911B NFC-xcvr deactivated ...");
    }

    //rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );   // TODO: assess - is 'rfalNfcDeactivate()' required to be run after a scan??? (then re-activate w. ''rfalNfcDeactivate()' ??)


    vTaskDelay(5000);
}

#endif    // IF 0 


/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/
static bool exampleRfalPollerTechDetetection( void );
static bool exampleRfalPollerCollResolution( void );
static bool exampleRfalPollerActivation( uint8_t devIt );
static bool exampleRfalPollerNfcDepActivate( exampleRfalPollerDevice *device );
static ReturnCode exampleRfalPollerDataExchange( void );
static bool exampleRfalPollerDeactivate( void );


/*
******************************************************************************
* LOCAL FUNCTION IMPLEMENTATION
******************************************************************************
*/
/*!
 ******************************************************************************
 * \brief Poller Technology Detection
 * 
 * This method implements the Technology Detection / Poll for different 
 * device technologies.
 * 
 * \return true         : One or more devices have been detected
 * \return false         : No device have been detected
 * 
 ******************************************************************************
 */
static bool exampleRfalPollerTechDetetection( void )
{
    ReturnCode           err;
    rfalNfcaSensRes      sensRes;
    rfalNfcbSensbRes     sensbRes;
    rfalNfcvInventoryRes invRes;
    uint8_t              sensbResLen;
    
    gTechsFound = EXAMPLE_RFAL_POLLER_FOUND_NONE;
    
    /*******************************************************************************/
    /* NFC-A Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcaPollerInitialize();                                                       /* Initialize RFAL for NFC-A */
    rfalFieldOnAndStartGT();                                                          /* Turns the Field On and starts GT timer */
    
    err = rfalNfcaPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensRes ); /* Poll for NFC-A devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= EXAMPLE_RFAL_POLLER_FOUND_A;
    }
    
    
    /*******************************************************************************/
    /* NFC-B Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcbPollerInitialize();                                                       /* Initialize RFAL for NFC-B */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcbPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensbRes, &sensbResLen ); /* Poll for NFC-B devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= EXAMPLE_RFAL_POLLER_FOUND_B;
    }
    
    
    /*******************************************************************************/
    /* NFC-F Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcfPollerInitialize( RFAL_BR_212 );                                          /* Initialize RFAL for NFC-F */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcfPollerCheckPresence();                                              /* Poll for NFC-F devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= EXAMPLE_RFAL_POLLER_FOUND_F;
    }
    
    
    /*******************************************************************************/
    /* NFC-V Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcvPollerInitialize();                                                       /* Initialize RFAL for NFC-V */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcvPollerCheckPresence( &invRes );                                     /* Poll for NFC-V devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= EXAMPLE_RFAL_POLLER_FOUND_V;
    }
    
    return (gTechsFound != EXAMPLE_RFAL_POLLER_FOUND_NONE);
}

/*!
 ******************************************************************************
 * \brief Poller Collision Resolution
 * 
 * This method implements the Collision Resolution on all technologies that
 * have been detected before.
 * 
 * \return true         : One or more devices identified 
 * \return false        : No device have been identified
 * 
 ******************************************************************************
 */
static bool exampleRfalPollerCollResolution( void )
{
    uint8_t    i;
    uint8_t    devCnt;
    ReturnCode err;
    
    
    /*******************************************************************************/
    /* NFC-A Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & EXAMPLE_RFAL_POLLER_FOUND_A )                                   /* If a NFC-A device was found/detected, perform Collision Resolution */
    {
        rfalNfcaListenDevice nfcaDevList[EXAMPLE_RFAL_POLLER_DEVICES];
        
        rfalNfcaPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcaPollerFullCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (EXAMPLE_RFAL_POLLER_DEVICES - gDevCnt), nfcaDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfca list into global device list */
            {
                gDevList[gDevCnt].type     = EXAMPLE_RFAL_POLLER_TYPE_NFCA;
                gDevList[gDevCnt].dev.nfca = nfcaDevList[i];
                gDevCnt++;
            }
        }
    }
    
    /*******************************************************************************/
    /* NFC-B Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & EXAMPLE_RFAL_POLLER_FOUND_B )                                   /* If a NFC-A device was found/detected, perform Collision Resolution */
    {
        rfalNfcbListenDevice nfcbDevList[EXAMPLE_RFAL_POLLER_DEVICES];
        
        rfalNfcbPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcbPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (EXAMPLE_RFAL_POLLER_DEVICES - gDevCnt), nfcbDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfcb list into global device list */
            {
                gDevList[gDevCnt].type     = EXAMPLE_RFAL_POLLER_TYPE_NFCB;
                gDevList[gDevCnt].dev.nfcb = nfcbDevList[i];
                gDevCnt++;
            }
        }
    }
    
    
    /*******************************************************************************/
    /* NFC-F Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & EXAMPLE_RFAL_POLLER_FOUND_F )                                   /* If a NFC-F device was found/detected, perform Collision Resolution */
    {
        rfalNfcfListenDevice nfcfDevList[EXAMPLE_RFAL_POLLER_DEVICES];
        
        rfalNfcfPollerInitialize( RFAL_BR_212 );
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcfPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (EXAMPLE_RFAL_POLLER_DEVICES - gDevCnt), nfcfDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfcf list into global device list */
            {
                gDevList[gDevCnt].type     = EXAMPLE_RFAL_POLLER_TYPE_NFCF;
                gDevList[gDevCnt].dev.nfcf = nfcfDevList[i];
                gDevCnt++;
            }
        }
    }
    
    /*******************************************************************************/
    /* NFC-V Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & EXAMPLE_RFAL_POLLER_FOUND_V )                                   /* If a NFC-F device was found/detected, perform Collision Resolution */
    {
        rfalNfcvListenDevice nfcvDevList[EXAMPLE_RFAL_POLLER_DEVICES];
        
        rfalNfcvPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcvPollerCollisionResolution(RFAL_COMPLIANCE_MODE_NFC, (EXAMPLE_RFAL_POLLER_DEVICES - gDevCnt), nfcvDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                /* Copy devices found form local Nfcf list into global device list */
            {
                gDevList[gDevCnt].type     = EXAMPLE_RFAL_POLLER_TYPE_NFCV;
                gDevList[gDevCnt].dev.nfcv = nfcvDevList[i];
                gDevCnt++;
            }
        }
    }
    
    return (gDevCnt > 0);
}


/*!
 ******************************************************************************
 * \brief Poller Activation
 * 
 * This method Activates a given device according to it's type and 
 * protocols supported
 *  
 * \param[in]  devIt : device's position on the list to be activated 
 * 
 * \return true         : Activation successful 
 * \return false        : Activation failed
 * 
 ******************************************************************************
 */
static bool exampleRfalPollerActivation( uint8_t devIt )
{
    ReturnCode           err;
    rfalNfcaSensRes      sensRes;
    rfalNfcaSelRes       selRes;
    rfalNfcbSensbRes     sensbRes;
    uint8_t              sensbResLen;
    
    if( devIt > gDevCnt )
    {
        return false;
    }
    
    switch( gDevList[devIt].type )
    {
        /*******************************************************************************/
        /* NFC-A Activation                                                            */
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_TYPE_NFCA:
            
            rfalNfcaPollerInitialize();
            if( gDevList[devIt].dev.nfca.isSleep )                                    /* Check if desired device is in Sleep      */
            {
                err = rfalNfcaPollerCheckPresence( RFAL_14443A_SHORTFRAME_CMD_WUPA, &sensRes ); /* Wake up all cards  */
                if( err != RFAL_ERR_NONE )
                {
                    return false;
                }
                
                err = rfalNfcaPollerSelect( gDevList[devIt].dev.nfca.nfcId1, gDevList[devIt].dev.nfca.nfcId1Len, &selRes ); /* Select specific device  */
                if( err != RFAL_ERR_NONE )
                {
                    return false;
                }
            }
            
            /*******************************************************************************/
            /* Perform protocol specific activation                                        */
            switch( gDevList[devIt].dev.nfca.type )
            {
                /*******************************************************************************/
                case RFAL_NFCA_T1T:
                    
                    /* No further activation needed for a T1T (RID already performed)*/
                    Serial0.print("NFC-A T1T device activated \r\n");                   /* NFC-A T1T device activated */
                    
                    gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_RF;
                    break;
                    
                
                /*******************************************************************************/
                case RFAL_NFCA_T2T:
                  
                    /* No specific activation needed for a T2T */    
                    Serial0.print("NFC-A T2T device activated \r\n");                   /* NFC-A T2T device activated */
                    
                    gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_RF;
                    break;
                
                
                /*******************************************************************************/
                case RFAL_NFCA_T4T:
                
                    /* Perform ISO-DEP (ISO14443-4) activation: RATS and PPS if supported */
                    err = rfalIsoDepPollAHandleActivation( (rfalIsoDepFSxI)RFAL_ISODEP_FSDI_DEFAULT, RFAL_ISODEP_NO_DID, RFAL_BR_424, &gDevList[devIt].proto.isoDep );
                    if( err != RFAL_ERR_NONE )
                    {
                        return false;
                    }
                    
                    Serial0.print("NFC-A T4T (ISO-DEP) device activated \r\n");         /* NFC-A T4T device activated */
                    
                    gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP;
                    break;
                  
                  
                /*******************************************************************************/
                case RFAL_NFCA_T4T_NFCDEP:                                              /* Device supports both T4T and NFC-DEP */
                case RFAL_NFCA_NFCDEP:                                                  /* Device supports NFC-DEP */
                  
                    /* Perform NFC-DEP (P2P) activation: ATR and PSL if supported */
                    if( !exampleRfalPollerNfcDepActivate( &gDevList[devIt] ) )
                    {
                      return false;
                    }
                    
                    Serial0.print("NFC-A P2P (NFC-DEP) device activated \r\n");         /* NFC-A P2P device activated */
                    gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP;
                    break;
            }
            
            break;
        
        /*******************************************************************************/
        /* NFC-B Activation                                                            */
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_TYPE_NFCB:
            
            rfalNfcbPollerInitialize();
            if( gDevList[devIt].dev.nfcb.isSleep )                                    /* Check if desired device is in Sleep */
            {
                /* Wake up all cards. SENSB_RES may return collision but the NFCID0 is available to explicitly select NFC-B card via ATTRIB; so error will be ignored here */
                rfalNfcbPollerCheckPresence( RFAL_NFCB_SENS_CMD_ALLB_REQ, RFAL_NFCB_SLOT_NUM_1, &sensbRes, &sensbResLen );
            }
            
            
            /*******************************************************************************/
            /* Perform ISO-DEP (ISO14443-4) activation: RATS and PPS if supported          */
            err = rfalIsoDepPollBHandleActivation( (rfalIsoDepFSxI)RFAL_ISODEP_FSDI_DEFAULT, RFAL_ISODEP_NO_DID, RFAL_BR_424, 0x00, &gDevList[devIt].dev.nfcb, NULL, 0, &gDevList[devIt].proto.isoDep );
            if( err == RFAL_ERR_NONE )
            {
                Serial0.print("NFC-B T4T (ISO-DEP) device activated \r\n");             /* NFC-B T4T device activated */
                
                gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP ;
                break;
            }
            
            Serial0.print("NFC-B device activated \r\n");                               /* NFC-B  device activated */
            gDevList[devIt].rfInterface =  EXAMPLE_RFAL_POLLER_INTERFACE_RF;
            break;
            
        /*******************************************************************************/
        /* NFC-F Activation                                                            */
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_TYPE_NFCF:
            
            rfalNfcfPollerInitialize( RFAL_BR_212 );
            if( rfalNfcfIsNfcDepSupported( &gDevList[devIt].dev.nfcf ) )
            {
                /* Perform NFC-DEP (P2P) activation: ATR and PSL if supported */
                if( !exampleRfalPollerNfcDepActivate( &gDevList[devIt] ) )
                {
                    return false;
                }
                
                Serial0.print("NFC-F P2P (NFC-DEP) device activated \r\n");             /* NFC-A P2P device activated */
                
                gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP;
                break;
            }
            
            Serial0.print("NFC-F T3T device activated \r\n");                           /* NFC-F T3T device activated */
            gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_RF;
            break;
            
        /*******************************************************************************/
        /* NFC-V Activation                                                            */
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_TYPE_NFCV:
            
            rfalNfcvPollerInitialize();
            
            /* No specific activation needed for a T5T */
            Serial0.print("NFC-V T5T device activated \r\n");                           /* NFC-V T5T device activated */
            
            gDevList[devIt].rfInterface = EXAMPLE_RFAL_POLLER_INTERFACE_RF;
            break;
        
        /*******************************************************************************/
        default:
            return false;
    }
    
    gActiveDev = &gDevList[devIt];                                                    /* Assign active device to be used further on */
    return true;
}


/*!
 ******************************************************************************
 * \brief Poller NFC DEP Activate
 * 
 * This method performs NFC-DEP Activation 
 *  
 * \param[in]  devIt : device to be activated 
 * 
 * \return true         : Activation successful 
 * \return false        : Activation failed
 * 
 ******************************************************************************
 */
static bool exampleRfalPollerNfcDepActivate( exampleRfalPollerDevice *device )
{
    rfalNfcDepAtrParam   param;
                
    /*******************************************************************************/
    /* If Passive F use the NFCID2 retrieved from SENSF                            */
    if( device->type == EXAMPLE_RFAL_POLLER_TYPE_NFCF )
    {
        param.nfcid    = device->dev.nfcf.sensfRes.NFCID2;
        param.nfcidLen = RFAL_NFCF_NFCID2_LEN;
    }
    else
    {
        param.nfcid    = gNfcid3;
        param.nfcidLen = RFAL_NFCDEP_NFCID3_LEN;
    }    
    
    param.BS    = RFAL_NFCDEP_Bx_NO_HIGH_BR;
    param.BRATE = RFAL_NFCDEP_Bx_NO_HIGH_BR;
    param.DID   = RFAL_NFCDEP_DID_NO;
    param.NAD   = RFAL_NFCDEP_NAD_NO;
    param.LR    = RFAL_NFCDEP_LR_254;
    param.GB    = gGenBytes;
    param.GBLen = sizeof(gGenBytes);
    param.commMode  = RFAL_NFCDEP_COMM_PASSIVE;
    param.operParam = (RFAL_NFCDEP_OPER_FULL_MI_EN | RFAL_NFCDEP_OPER_EMPTY_DEP_DIS | RFAL_NFCDEP_OPER_ATN_EN | RFAL_NFCDEP_OPER_RTOX_REQ_EN);
    
    /* Perform NFC-DEP (P2P) activation: ATR and PSL if supported */
    return (rfalNfcDepInitiatorHandleActivation( &param, RFAL_BR_424, &device->proto.nfcDep ) == RFAL_ERR_NONE);
}


/*!
 ******************************************************************************
 * \brief Data Exchange
 * 
 * This method performs Data Exchange by device's type and interface.
 *  
 * 
 * \return RFAL_ERR_REQUEST     : Bad request
 * \return RFAL_ERR_BUSY        : Data Exchange ongoing
 * \return RFAL_ERR_NONE        : Data Exchange terminated successfully
 * 
 ******************************************************************************
 */
static ReturnCode exampleRfalPollerDataExchange( void )
{
    rfalTransceiveContext ctx;
    ReturnCode            err;
    rfalIsoDepTxRxParam   isoDepTxRx;
    rfalNfcDepTxRxParam   nfcDepTxRx;
    uint8_t               *txBuf;
    uint16_t              txBufLen;
    
    
    /*******************************************************************************/
    /* The Data Exchange is divided in two different moments, the trigger/Start of *
     *  the transfer followed by the check until its completion                    */
    if( gState == EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_START )                      /* Trigger/Start the data exchange */
    {
        switch( gActiveDev->rfInterface )                                             /* Check which RF interface shall be used/has been activated */
        {
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_RF:
    
                switch( gActiveDev->type )                                            /* Over RF interface no specific protocol is selected, each device supports a different protocol */
                {
                    /*******************************************************************************/
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCA:
                        switch( gActiveDev->dev.nfca.type )
                        {
                            /*******************************************************************************/
                            case RFAL_NFCA_T1T:
                                
                                /* To perform presence check, on this example a T1T Read command is used */
                                MEMCPY( &t1tReadReq[3], gActiveDev->dev.nfca.nfcId1, RFAL_NFCA_CASCADE_1_UID_LEN );  /* Assign device's NFCID for read command */
                                                        
                                txBuf    = t1tReadReq;
                                txBufLen = sizeof(t1tReadReq);
                                break;
                                
                            /*******************************************************************************/
                            case RFAL_NFCA_T2T:
                                
                                /* To perform presence check, on this example a T2T Read command is used */
                                txBuf    = t2tReadReq;
                                txBufLen = sizeof(t2tReadReq);
                                break;

                            case RFAL_NFCA_T4T:
                                Serial0.print("NFCA Passive ISO-DEP device found. UID: ");
                                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                ndef_dump((rfalNfcDevice *)gActiveDev);
                                rfalIsoDepDeselect(); 
                                break;
                            
                            case RFAL_NFCA_T4T_NFCDEP:
                            case RFAL_NFCA_NFCDEP:
                                platformLog("NFCA Passive P2P device found. NFCID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                //demoP2P();
                                break;
                                
                            default:    // TODO: assess - which of the two are correct???
                                platformLog("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                ndef_dump((rfalNfcDevice *)nfcDevice);
                                rfalNfcaPollerSleep();
                                break;
                            
                            /*******************************************************************************/
                            //default:
                            //    return RFAL_ERR_REQUEST;      // TODO: assess - which of the two are correct???
                        }
                        break;

                        
                    /*******************************************************************************/
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCB:
                        
                        /* To perform presence check, no specific command is used */
                        txBuf    = nfcbReq;
                        txBufLen = sizeof(nfcbReq);
                        break;
                        
                        
                    /*******************************************************************************/
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCF:
                        
                        /* To perform presence check, on this example a T3T Check/Read command is used */
                        MEMCPY( &t3tCheckReq[1], gActiveDev->dev.nfcf.sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN );  /* Assign device's NFCID for Check command */
                        
                        txBuf    = t3tCheckReq;
                        txBufLen = sizeof(t3tCheckReq);
                        break;
                        
                        
                    /*******************************************************************************/
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCV:
                        
                        /* To perform presence check, on this example a Get System Information command is used */
                        txBuf    = t5tSysInfoReq;
                        txBufLen = sizeof(t5tSysInfoReq);
                        break;
                        
                        
                    /*******************************************************************************/
                    default:
                        return RFAL_ERR_REQUEST;
                }
                
                /*******************************************************************************/
                /* Trigger a RFAL Transceive using the previous defined frames                 */
                rfalCreateByteFlagsTxRxContext( ctx, txBuf, txBufLen, gRxBuf.rfRxBuf, sizeof(gRxBuf.rfRxBuf), &gRcvLen, RFAL_TXRX_FLAGS_DEFAULT, rfalConvMsTo1fc(20) );
                return (((err = rfalStartTransceive( &ctx )) == RFAL_ERR_NONE) ? RFAL_ERR_BUSY : err);     /* Signal RFAL_ERR_BUSY as Data Exchange has been started and is ongoing */
                
            case EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP:
                
                MEMCPY( gTxBuf.isoDepTxBuf.inf, t4tSelectReq, sizeof(t4tSelectReq) );
                
                isoDepTxRx.DID          = RFAL_ISODEP_NO_DID;
                isoDepTxRx.ourFSx       = RFAL_ISODEP_FSX_KEEP;
                isoDepTxRx.FSx          = gActiveDev->proto.isoDep.info.FSx;
                isoDepTxRx.dFWT         = gActiveDev->proto.isoDep.info.dFWT;
                isoDepTxRx.FWT          = gActiveDev->proto.isoDep.info.FWT;
                isoDepTxRx.txBuf        = &gTxBuf.isoDepTxBuf;
                isoDepTxRx.txBufLen     = sizeof(t4tSelectReq);
                isoDepTxRx.isTxChaining = false;
                isoDepTxRx.rxBuf        = &gRxBuf.isoDepRxBuf;
                isoDepTxRx.rxLen        = &gRcvLen;
                isoDepTxRx.isRxChaining = &gRxChaining;
                
                /*******************************************************************************/
                /* Trigger a RFAL ISO-DEP Transceive                                           */
                return (((err = rfalIsoDepStartTransceive( isoDepTxRx )) == RFAL_ERR_NONE) ? RFAL_ERR_BUSY : err); /* Signal RFAL_ERR_BUSY as Data Exchange has been started and is ongoing */
                
                
            case EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP:
                
                MEMCPY( gTxBuf.nfcDepTxBuf.inf, llcpSymm, sizeof(llcpSymm) );
                
                nfcDepTxRx.DID          = RFAL_NFCDEP_DID_KEEP;
                nfcDepTxRx.FSx          = rfalNfcDepLR2FS( rfalNfcDepPP2LR( gActiveDev->proto.nfcDep.activation.Target.ATR_RES.PPt ) );
                nfcDepTxRx.dFWT         = gActiveDev->proto.nfcDep.info.dFWT;
                nfcDepTxRx.FWT          = gActiveDev->proto.nfcDep.info.FWT;
                nfcDepTxRx.txBuf        = &gTxBuf.nfcDepTxBuf;
                nfcDepTxRx.txBufLen     = sizeof(llcpSymm);
                nfcDepTxRx.isTxChaining = false;
                nfcDepTxRx.rxBuf        = &gRxBuf.nfcDepRxBuf;
                nfcDepTxRx.rxLen        = &gRcvLen;
                nfcDepTxRx.isRxChaining = &gRxChaining;
                
                /*******************************************************************************/
                /* Trigger a RFAL NFC-DEP Transceive                                           */
                return (((err = rfalNfcDepStartTransceive( &nfcDepTxRx )) == RFAL_ERR_NONE) ? RFAL_ERR_BUSY : err);  /* Signal RFAL_ERR_BUSY as Data Exchange has been started and is ongoing */
                
            default:
                break;
        }
    }
    /*******************************************************************************/
    /* The Data Exchange has been started, wait until completed                    */
    else if( gState == EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_CHECK )
    {
        switch( gActiveDev->rfInterface )
        {
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_RF:
                return rfalGetTransceiveStatus();
                
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP:
                return rfalIsoDepGetTransceiveStatus();
                
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP:
                return rfalNfcDepGetTransceiveStatus();
                
            /*******************************************************************************/
            default:
                return RFAL_ERR_PARAM;
        }
    }
    return RFAL_ERR_REQUEST;
}


/*!
 ******************************************************************************
 * \brief Poller NFC DEP Deactivate
 * 
 * This method Deactivates the device if a deactivation procedure exists 
 * 
 * \return true         : Deactivation successful 
 * \return false        : Deactivation failed
 * 
 ******************************************************************************
 */
static bool exampleRfalPollerDeactivate( void )
{
    if( gActiveDev != NULL )                                                          /* Check if a device has been activated */
    {
        switch( gActiveDev->rfInterface )
        {
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_RF:
                break;                                                                /* No specific deactivation to be performed */
                
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_ISODEP:
                rfalIsoDepDeselect();                                                 /* Send a Deselect to device */
                break;
                
            /*******************************************************************************/
            case EXAMPLE_RFAL_POLLER_INTERFACE_NFCDEP:
                rfalNfcDepRLS();                                                      /* Send a Release to device */
                break;
                
            default:
                return false;
        }
        Serial0.print("Device deactivated \r\n");
    }
    
    return true;
}


/***************************************   SETUP ***************************************/
void setup() 
{
    Serial0.begin(115200);
    Serial0.println("Init ...");
    
    spi_init();

    // NFC:
    ReturnCode ret = rfalNfcInitialize();      // WAS: 'rfalInitialize()' - but this function is NOT setting NFC-state!!
    
    if (RFAL_ERR_NONE != ret)
    {
        Serial0.println("ERROR: NFC subsystem init failed!\nNFC subsystem init return code:");
        Serial0.println(ret);
        while(1)
        {
            vTaskDelay(1000);
        }
    }

    Serial0.println("NFC subsystem initialized OK ...");
}


/*************************************** MAIN task(-loop) ***************************************************/
void loop() 
{
    static rfalNfcDevice *nfcDevice;

    Serial0.println("Worker - start scan ...");

    // put your main code here, to run repeatedly:
    rfalNfcWorker(); //TODO: put in a separate thread. NOTE: was 'rfalWorker()'.

    vTaskDelay(50);

    switch( gState )
    {
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_STATE_INIT:                                     
            
            gTechsFound = EXAMPLE_RFAL_POLLER_FOUND_NONE; 
            gActiveDev  = NULL;
            gDevCnt     = 0;
            
            gState = EXAMPLE_RFAL_POLLER_STATE_TECHDETECT;
            break;
            
            
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_STATE_TECHDETECT:
            
            if( !exampleRfalPollerTechDetetection() )                             /* Poll for nearby devices in different technologies */
            {
                gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;                  /* If no device was found, restart loop */
                break;
            }
            
            gState = EXAMPLE_RFAL_POLLER_STATE_COLAVOIDANCE;                      /* One or more devices found, go to Collision Avoidance */
            break;
            
            
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_STATE_COLAVOIDANCE:
            
            if( !exampleRfalPollerCollResolution() )                              /* Resolve any eventual collision */
            {
                gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;                  /* If Collision Resolution was unable to retrieve any device, restart loop */
                break;
            }
            
            Serial0.print("Device(s) found: ");
            Serial0.println( gDevCnt );
            
            for(int i = 0; i < gDevCnt; i++)
            {
                switch( gDevList[i].type )
                {
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCA:
                        Serial0.print( " NFC-A device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfca.nfcId1, gDevList[i].dev.nfca.nfcId1Len) );
                        //platformLedOn( LED_NFCA_PORT, LED_NFCA_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCB:
                        Serial0.print( " NFC-B device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfcb.sensbRes.nfcid0, RFAL_NFCB_NFCID0_LEN) );
                        //platformLedOn( LED_NFCB_PORT, LED_NFCB_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCF:
                        Serial0.print( " NFC-F device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfcf.sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN) );
                        //platformLedOn( LED_NFCF_PORT, LED_NFCF_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case EXAMPLE_RFAL_POLLER_TYPE_NFCV:
                        Serial0.print( " NFC-V device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfcv.InvRes.UID, RFAL_NFCV_UID_LEN) );
                        //platformLedOn( LED_NFCV_PORT, LED_NFCV_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                }
            }
            //platformDelay(200);
            gState = EXAMPLE_RFAL_POLLER_STATE_ACTIVATION;                        /* Device(s) have been identified, go to Activation */
            break;
        
            
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_STATE_ACTIVATION:
#if 0
            if( !exampleRfalPollerActivation( 0 ) )                               /* Any device previous identified can be Activated, on this example will select the firt on the list */
            {
                gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;                  /* If Activation failed, restart loop */
                break;
            }
            
            //gState = EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_START;                /* Device has been properly activated, go to Data Exchange */
            gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;
        break;
#endif	            
            
        /*******************************************************************************/
#if 0	        
    case EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_START:                       
    case EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_CHECK:
            
        err = exampleRfalPollerDataExchange();                                /* Perform Data Exchange, in this example a simple transfer will executed in order to do device's presence check */
        switch( err )
        {
            case RFAL_ERR_NONE:                                                    /* Data exchange successful  */
                platformDelay(300);                                           /* Wait a bit */
                gState = EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_START;        /* Trigger new exchange with device */
                break;
                
            case RFAL_ERR_BUSY:                                                    /* Data exchange ongoing  */
                gState = EXAMPLE_RFAL_POLLER_STATE_DATAEXCHANGE_CHECK;        /* Once triggered/started the Data Exchange only do check until is completed */
                break;
                
            default:                                                          /* Data exchange not successful, card removed or other transmission error */
                Serial0.print("Data exchange terminated with error: %d \r\n", err);
                gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;              /* Restart loop */
                break;
        }
        break;
            
#endif	            
        /*******************************************************************************/
        case EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION:
#if 0	            
            exampleRfalPollerDeactivate();                                        /* If a card has been activated, properly deactivate the device */
#endif	            
            rfalFieldOff();                                                       /* Turn the Field Off powering down any device nearby */
            platformDelay(2);                                                     /* Remain a certain period with field off */
            gState = EXAMPLE_RFAL_POLLER_STATE_INIT;                              /* Restart the loop */

            //platformLogClear();
            //platformLog2Screen(logBuffer);

            break;
        
        
        /*******************************************************************************/
        default:
            return;

	}

    vTaskDelay(3000);
}


