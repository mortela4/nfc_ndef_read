/**
 * @file main.cpp
 * 
 * @brief Demonstrate NDEF-data readout from NFC-A/B/C/F/V tags. 
 */


#include <Arduino.h>

#include <errno.h>

#include "SPI.h"

#include "rfal_platform/rfal_platform.h"


extern "C" {
#include "utils.h"
#include "rfal_utils.h"
#include "rfal_nfc.h"             // Includes all of "rfal_nfc[a|b|f|v].h", "rfal_isoDep.h" and "rfal_nfcDep.h".
#include "ndef_poller.h"
#include "ndef_message.h"
#include "ndef_types.h"

//#include "ndef_dump.h"
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
#define RFAL_POLLER_DEVICES      10    /* Number of devices supported */
#define RFAL_POLLER_RF_BUF_LEN   255   /* RF buffer length            */

#define RFAL_POLLER_FOUND_NONE   0x00  /* No device found Flag        */
#define RFAL_POLLER_FOUND_A      0x01  /* NFC-A device found Flag     */
#define RFAL_POLLER_FOUND_B      0x02  /* NFC-B device found Flag     */
#define RFAL_POLLER_FOUND_F      0x04  /* NFC-F device found Flag     */
#define RFAL_POLLER_FOUND_V      0x08  /* NFC-V device Flag           */

#define NDEF_MESSAGE_BUF_LEN     8192


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

/*! Main state                                                                          */
typedef enum{
    RFAL_POLLER_STATE_INIT                =  0,  /* Initialize state            */
    RFAL_POLLER_STATE_TECHDETECT          =  1,  /* Technology Detection state  */
    RFAL_POLLER_STATE_COLAVOIDANCE        =  2,  /* Collision Avoidance state   */
    RFAL_POLLER_STATE_ACTIVATION          =  3,  /* Activation state            */
    RFAL_POLLER_STATE_DATAEXCHANGE_START  =  4,  /* Data Exchange Start state   */
    RFAL_POLLER_STATE_DATAEXCHANGE_CHECK  =  5,  /* Data Exchange Check state   */
    RFAL_POLLER_STATE_DEACTIVATION        =  9   /* Deactivation state          */
}RfalPollerState;


/*! Device type                                                                         */
typedef enum{
    RFAL_POLLER_TYPE_NFCA  =  0,                 /* NFC-A device type           */
    RFAL_POLLER_TYPE_NFCB  =  1,                 /* NFC-B device type           */
    RFAL_POLLER_TYPE_NFCF  =  2,                 /* NFC-F device type           */
    RFAL_POLLER_TYPE_NFCV  =  3                  /* NFC-V device type           */
}RfalPollerDevType;


/*! Device interface                                                                    */
typedef enum{
    RFAL_POLLER_INTERFACE_RF     = 0,            /* RF Frame interface          */
    RFAL_POLLER_INTERFACE_ISODEP = 1,            /* ISO-DEP interface           */
    RFAL_POLLER_INTERFACE_NFCDEP = 2             /* NFC-DEP interface           */
}RfalPollerRfInterface;


/*! Device struct containing all its details                                            */
typedef struct{
    RfalPollerDevType type;                      /* Device's type                */
    union{
        rfalNfcaListenDevice nfca;                      /* NFC-A Listen Device instance */
        rfalNfcbListenDevice nfcb;                      /* NFC-B Listen Device instance */
        rfalNfcfListenDevice nfcf;                      /* NFC-F Listen Device instance */
        rfalNfcvListenDevice nfcv;                      /* NFC-V Listen Device instance */
    }dev;                                               /* Device's instance            */
    
    RfalPollerRfInterface rfInterface;           /* Device's interface           */
    union{
        rfalIsoDepDevice isoDep;                        /* ISO-DEP instance             */
        rfalNfcDepDevice nfcDep;                        /* NFC-DEP instance             */
    }proto;                                             /* Device's protocol            */
    
}RfalPollerDevice;


/************************************ LOCAL VARIABLES *****************************************/

static rfalNfcDiscoverParam discParam = { 
		.compMode = RFAL_COMPLIANCE_MODE_NFC,
        .techs2Find = (RFAL_NFC_POLL_TECH_A | RFAL_NFC_POLL_TECH_B | RFAL_NFC_POLL_TECH_F | RFAL_NFC_POLL_TECH_V), 
		.techs2Bail = RFAL_NFC_TECH_NONE, 
		.totalDuration = 1000U, 
		.devLimit = 1U,                         // NOTE: only communicates w. ONE, single tag at the time!!
         // maxBR --> Max bitrate NOT set ...
		.nfcfBR = RFAL_BR_212, 
        .nfcid3 = {0}, 
		.GB = {0}, 
        .GBLen = 0, 
		.ap2pBR = RFAL_BR_424, 
        // p2pNfcaPrio NOT set ...
        // propNfc NOT set ...        
        // isoDepFS NOT set ...  
        // nfcDepLR NOT set ...       
        // lmConfigPA NOT set ...
        // lmConfigPF NOT set ...
		.notifyCb = NULL, 
		.wakeupEnabled = false, 
		.wakeupConfigDefault = true,		
};

static uint8_t rawMessageBuf[NDEF_MESSAGE_BUF_LEN];

static rfalNfcDevice *nfcDevice;        // NFC-device handle --> allocated and returned by API.
static ndefContext ndefCtx;             // NDEF-context handle --> allocated here, to be populated by API.


// static uint8_t                 gDevCnt;                                 /* Number of devices found                         */
// static RfalPollerDevice         gDevList[RFAL_POLLER_DEVICES];                  /* Device List                                     */
// static RfalPollerState          gState;                                  /* Main state                                      */
// static uint8_t                 gTechsFound;                             /* Technologies found bitmask                      */
// RfalPollerDevice                *gActiveDev;                             /* Active device pointer                           */
// static uint16_t                gRcvLen;                                 /* Received length                                 */
// static bool                    gRxChaining;                             /* Rx chaining flag                                */


/************************************ LOCAL FUNCTION PROTOTYPES ************************************/

static char *hex2str(uint8_t *number, uint8_t length);
static String state_description(rfalNfcState rfalState);
static void ndefParseMessage(uint8_t *rawMsgBuf, uint32_t rawMsgLen);
static void ndefParseRecord(ndefRecord *record);
static void ndefPrintString(const uint8_t *str, uint32_t strLen);
static void check_discover_retval(const ndefStatus err);


/*
******************************************************************************
* LOCAL NFC-POLL FUNCTION IMPLEMENTATION
******************************************************************************
*/

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


/************************************ LOCAL NDEF-related FUNCTION IMPLEMENTATION ************************************/

void ndefParseMessage(uint8_t* rawMsgBuf, uint32_t rawMsgLen) 
{
	ndefStatus err; ndefConstBuffer
	bufRawMessage; ndefMessage message;
	ndefRecord* record;
	bufRawMessage.buffer = rawMsgBuf;
	bufRawMessage.length = rawMsgLen;
	
	err = ndefMessageDecode(&bufRawMessage,	&message); 
	if (err != ERR_NONE) 
	{
		return;
	}
	
	record = ndefMessageGetFirstRecord(&message);
	while (record != NULL) 
	{
		ndefParseRecord(record); 
		record = ndefMessageGetNextRecord(record);
	}
}


void ndefPrintString(const uint8_t* str, const uint32_t strLen) 
{
    char ndefInfo[1024];
	
    MEMCPY(ndefInfo, str, strLen);

    ndefInfo[strLen] = '\0';   // Just in case ...
 
    Serial0.println(ndefInfo);  
}


static void ndefParseRecord(ndefRecord* record)
{
	ndefStatus err; 
	ndefType type;
	
	err = ndefRecordToType(record, &type); 
	if (err	!= ERR_NONE) 
	{
		return;
	}
	
	switch (type.id) 
	{
		case NDEF_TYPE_ID_EMPTY: Serial0.println(" * Empty record ..."); break; 
		
		case NDEF_TYPE_ID_RTD_DEVICE_INFO: Serial0.println(" * Device info record"); break; 
		
		case NDEF_TYPE_ID_RTD_TEXT: Serial0.println(" * TEXT record: ");
								    ndefPrintString(type.data.text.bufSentence.buffer, type.data.text.bufSentence.length); 
								    break;
								
		case NDEF_TYPE_ID_RTD_URI: Serial0.println(" * URI record: ");
								   ndefPrintString(type.data.uri.bufUriString.buffer, type.data.uri.bufUriString.length); 
								   break;
								   
		case NDEF_TYPE_ID_RTD_AAR: Serial0.println(" * AAR record: ");
								   ndefPrintString(type.data.aar.bufPayload.buffer, type.data.aar.bufPayload.length); 
								   break;
												
		case NDEF_TYPE_ID_RTD_WLCCAP: /* Fall through */
		case NDEF_TYPE_ID_RTD_WLCSTAI: /* Fall through */
		case NDEF_TYPE_ID_RTD_WLCINFO: /* Fall through */
		case NDEF_TYPE_ID_RTD_WLCCTL: Serial0.print(" * WLC record ..."); break; 
		
		case NDEF_TYPE_ID_BLUETOOTH_BREDR: /* Fall through */
		case NDEF_TYPE_ID_BLUETOOTH_LE: /* Fall through */
		case NDEF_TYPE_ID_BLUETOOTH_SECURE_BREDR: /* Fall through */
		case NDEF_TYPE_ID_BLUETOOTH_SECURE_LE: Serial0.println(" * Bluetooth record ..."); break;
		
		case NDEF_TYPE_ID_MEDIA_VCARD: Serial0.println(" * vCard record ..."); break;
		
		case NDEF_TYPE_ID_MEDIA_WIFI: Serial0.println("* WIFI record ..."); break; 
		
		default: Serial0.println(" * Other record (?) ..."); break;														
	}
}


static void check_discover_retval(const ndefStatus err)
{
    if (RFAL_ERR_WRONG_STATE == err)
    {
        Serial0.println("Discover: WRONG STATE!");
    }
    else if (RFAL_ERR_PARAM == err)
    {
        Serial0.println("Discover: PARAMETER ERROR!");
    }
    else if (RFAL_ERR_DISABLED == err)
    {
        Serial0.println("Discover: FEATURE NOT ENABLED!");
    }
    else if (RFAL_ERR_NOTSUPP == err)
    {
        Serial0.println("Discover: FEATURE NOT SUPPORTED!");
    }
    else
    {
        Serial0.println("Discover: OK ...");
    }
}


static void ndefDumpSysInfo()
{
  ndefSystemInformation *sysInfo;

#if 0
  if (!verbose) {
    return;
  }

  if (!ndef.subCtx.t5t.sysInfoSupported) {
    return;
  }

  sysInfo = &ndef.subCtx.t5t.sysInfo;
  Serial0.print("System Information\r\n");
  Serial0.print(" * ");
  Serial0.print(ndefT5TSysInfoMOIValue(sysInfo->infoFlags) + 1);
  Serial0.print(" byte(s) memory addressing\r\n");
  if (ndefT5TSysInfoDFSIDPresent(sysInfo->infoFlags)) {
    Serial0.print(" * DFSID=");
    Serial0.print(sysInfo->DFSID, HEX);
    Serial0.print("h\r\n");
  }
  if (ndefT5TSysInfoAFIPresent(sysInfo->infoFlags)) {
    Serial0.print(" * AFI=");
    Serial0.print(sysInfo->AFI, HEX);
    Serial0.print("h\r\n");
  }
  if (ndefT5TSysInfoMemSizePresent(sysInfo->infoFlags)) {
    Serial0.print(" * ");
    Serial0.print(sysInfo->numberOfBlock);
    Serial0.print(" blocks, ");
    Serial0.print(sysInfo->blockSize);
    Serial0.print(" bytes per block\r\n");
  }
  if (ndefT5TSysInfoICRefPresent(sysInfo->infoFlags)) {
    Serial0.print(" * ICRef=");
    Serial0.print(sysInfo->ICRef, HEX);
    Serial0.print("h\r\n");
  }
  if (ndefT5TSysInfoCmdListPresent(sysInfo->infoFlags)) {
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoReadSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ReadSingleBlock                \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoWriteSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] WriteSingleBlock               \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoLockSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] LockSingleBlock                \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoReadMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ReadMultipleBlocks             \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoWriteMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] WriteMultipleBlocks            \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoSelectSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] Select                         \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoResetToReadySupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ResetToReady                   \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoGetMultipleBlockSecStatusSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] GetMultipleBlockSecStatus      \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoWriteAFISupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] WriteAFI                       \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoLockAFISupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] LockAFI                        \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoWriteDSFIDSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] WriteDSFID                     \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoLockDSFIDSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] LockDSFID                      \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoGetSystemInformationSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] GetSystemInformation           \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoCustomCmdsSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] CustomCmds                     \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoFastReadMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] FastReadMultipleBlocks         \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtReadSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtReadSingleBlock             \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtWriteSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtWriteSingleBlock            \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtLockSingleBlockSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtLockSingleBlock             \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtReadMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtReadMultipleBlocks          \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtWriteMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtWriteMultipleBlocks         \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoExtGetMultipleBlockSecStatusSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] ExtGetMultipleBlockSecStatus   \r\n");
    Serial0.print(" * [");
    Serial0.print(ndefT5TSysInfoFastExtendedReadMultipleBlocksSupported(sysInfo->supportedCmd) ? 'X' : ' ');
    Serial0.print("] FastExtendedReadMultipleBlocks \r\n");
  }
#endif
  return;
}


static const bool verbose = false;

/* State-to-descriptive-text: */
static uint8_t *ndefStates[] = 
{
  (uint8_t *)"INVALID",
  (uint8_t *)"INITIALIZED",
  (uint8_t *)"READ/WRITE",
  (uint8_t *)"READ-ONLY"
};


static void read_ndef_data(rfalNfcDevice *pNfcDevice)
{
    ReturnCode       err;
    ndefMessage      message;
    uint32_t         rawMessageLen;
    ndefInfo         info;
    ndefBuffer       bufRawMessage;
    ndefConstBuffer  bufConstRawMessage;

    ndefRecord       record1;
    ndefRecord       record2;

    ndefType         text;
    ndefType         uri;
    ndefType         aar;

    ndefConstBuffer8 bufTextLangCode;
    ndefConstBuffer bufTextLangText;
    ndefConstBuffer bufUri;
    ndefConstBuffer bufAndroidPackName;


    /*
    * Perform NDEF Context Initialization
    */
    err = ndefPollerContextInitialization(&ndefCtx, pNfcDevice);
    if (err != ERR_NONE) {
        Serial0.print("NDEF NOT DETECTED (ndefPollerContextInitialization returns ");
        Serial0.print(err);
        Serial0.print(")\r\n");

        return;
    }

    if (verbose & (pNfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCV)) 
    {
        // ndefDumpSysInfo(&ndefCtx);
    }

    /*
    * Perform NDEF Detect procedure
    */
    err = ndefPollerNdefDetect(&ndefCtx, &info);
    if (err != ERR_NONE) 
    {
        Serial0.print("NDEF NOT DETECTED (ndefPollerNdefDetect returns ");
        Serial0.print(err);
        Serial0.print(")\r\n");

        return;
    }
    else 
    {
        Serial0.print((char *)ndefStates[info.state]);
        Serial0.print(" NDEF detected.\r\n");
        //ndefCCDump(&ndefCtx);

        // if (verbose) 
        // {
        //     Serial0.print("NDEF Len: ");
        //     Serial0.print(&ndefCtx.messageLen);
        //     Serial0.print(", Offset=");
        //     Serial0.print(&ndefCtx.messageOffset);
        //     Serial0.print("\r\n");
        // }
    }

       if (info.state == NDEF_STATE_INITIALIZED) 
       {
            Serial0.println("NDEF-state = INITIALIZED --> nothing to READ ...");
            /* Nothing to read... */
            return;
        }

    err = ndefPollerReadRawMessage(&ndefCtx, rawMessageBuf, sizeof(rawMessageBuf), &rawMessageLen, true);   // Last (BOOL-)-arg means 'read SINGLE NDEF-msg'.
    if (err != ERR_NONE) 
    {
        Serial0.print("NDEF message cannot be read (ndefPollerReadRawMessage returns ");
        Serial0.print(err);
        Serial0.print(")\r\n");

        return;
    }

    if (verbose) 
    {
        bufRawMessage.buffer = rawMessageBuf;
        bufRawMessage.length = rawMessageLen;
        //ndefBufferDump(" NDEF Content", (ndefConstBuffer *)&bufRawMessage, verbose);

    }

    bufConstRawMessage.buffer = rawMessageBuf;
    bufConstRawMessage.length = rawMessageLen;

    ndefParseMessage(rawMessageBuf, rawMessageLen);

    // err = ndefMessageDecode(&bufConstRawMessage, &message);
    // if (err != ERR_NONE) 
    // {
    //     Serial0.print("NDEF message cannot be decoded (ndefMessageDecode  returns ");
    //     Serial0.print(err);
    //     Serial0.print(")\r\n");

    //     return;
    // }

    //err = ndefMessageDump(&message, verbose);



    // if (err != ERR_NONE) 
    // {
    //     Serial0.print("NDEF message cannot be displayed (ndefMessageDump returns ");
    //     Serial0.print(err);
    //     Serial0.print(")\r\n");

    //     return;
    // }
}       


/************************************************** Tasks ******************************************* */

// TODO: use enum!
#define NFC_POLL_STATE_NOTINIT               0  
#define NFC_POLL_STATE_START_DISCOVERY       1  
#define NFC_POLL_STATE_DISCOVERY             2  

static int nfcCurrentState = NFC_POLL_STATE_NOTINIT;

	
void poll_for_nfc_tags(void * parameter)
{
    ndefStatus err;
	ReturnCode ret;
    static bool found = false;
    uint32_t rawMessageLen;

    rfalNfcaSensRes       sensRes;
    rfalNfcaSelRes        selRes;
    rfalFeliCaPollRes     cardList[1];

    rfalNfcbSensbRes      sensbRes;
    uint8_t               sensbResLen;

    uint8_t               devCnt = 0;
    uint8_t               collisions = 0U;
    rfalNfcfSensfRes     *sensfRes;

    rfalNfcvInventoryRes  invRes;
    uint16_t              rcvdLen;

    for(;;)
    { 
        rfalNfcWorker(); 

        switch (nfcCurrentState) 
        {
            /*******************************************************************************/
            case NFC_POLL_STATE_START_DISCOVERY:
                rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
                rfalNfcDiscover(&discParam);

                nfcCurrentState = NFC_POLL_STATE_DISCOVERY;
            break;

            /*******************************************************************************/
            case NFC_POLL_STATE_DISCOVERY:
                if (rfalNfcIsDevActivated(rfalNfcGetState())) 
                {
                    rfalNfcGetActiveDevice(&nfcDevice);

                    delay(50);

                    switch (nfcDevice->type) {
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCA:

                        switch (nfcDevice->dev.nfca.type) 
                        {
                        case RFAL_NFCA_T1T:
                            Serial0.print("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            rfalNfcaPollerSleep();
                            break;

                        case RFAL_NFCA_T4T:
                            Serial0.print("NFCA Passive ISO-DEP device found. UID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            read_ndef_data(nfcDevice);
                            rfalIsoDepDeselect();
                            break;

                        case RFAL_NFCA_T4T_NFCDEP:
                        case RFAL_NFCA_NFCDEP:
                            Serial0.print("NFCA Passive P2P device found. NFCID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            //demoP2P();
                            break;

                        default:
                            Serial0.print("ISO14443A/NFC-A card found. UID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            read_ndef_data(nfcDevice);
                            rfalNfcaPollerSleep();
                            break;
                        }
                        /* Loop until tag is removed from the field */
                        Serial0.print("Operation completed\r\nTag can be removed from the field\r\n");
                        rfalNfcaPollerInitialize();
                        while (rfalNfcaPollerCheckPresence(RFAL_14443A_SHORTFRAME_CMD_WUPA, &sensRes) == ERR_NONE) 
                        {
                            if (((nfcDevice->dev.nfca.type == RFAL_NFCA_T1T) && (!rfalNfcaIsSensResT1T(&sensRes))) ||
                                ((nfcDevice->dev.nfca.type != RFAL_NFCA_T1T) && (rfalNfcaPollerSelect(nfcDevice->dev.nfca.nfcId1, nfcDevice->dev.nfca.nfcId1Len, &selRes) != ERR_NONE))) 
                            {
                                break;
                            }
                            rfalNfcaPollerSleep();
                            delay(130);
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCB:
                        // TODO: check - is this relevant??
                        Serial0.print("ISO14443B/NFC-B card found. UID: ");
                        Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                        Serial0.print("\r\n");

                        if (rfalNfcbIsIsoDepSupported(&nfcDevice->dev.nfcb)) 
                        {
                            read_ndef_data(nfcDevice);
                            rfalIsoDepDeselect();
                        } 
                        else 
                        {
                            rfalNfcbPollerSleep(nfcDevice->dev.nfcb.sensbRes.nfcid0);
                        }
                        /* Loop until tag is removed from the field */
                        Serial0.print("Operation completed\r\nTag can be removed from the field\r\n");
                        rfalNfcbPollerInitialize();
                        while (rfalNfcbPollerCheckPresence(RFAL_NFCB_SENS_CMD_ALLB_REQ, RFAL_NFCB_SLOT_NUM_1, &sensbRes, &sensbResLen) == ERR_NONE) 
                        {
                            if (ST_BYTECMP(sensbRes.nfcid0, nfcDevice->dev.nfcb.sensbRes.nfcid0, RFAL_NFCB_NFCID0_LEN) != 0) 
                            {
                                break;
                            }
                            rfalNfcbPollerSleep(nfcDevice->dev.nfcb.sensbRes.nfcid0);
                            delay(130);
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCF:

                        if (rfalNfcfIsNfcDepSupported(&nfcDevice->dev.nfcf)) 
                        {
                            Serial0.print("NFCF Passive P2P device found. NFCID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            // demoP2P();   // NOPE ...
                        } 
                        else 
                        {
                            Serial0.print("Felica/NFC-F card found. UID: ");
                            Serial0.print(hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                            Serial0.print("\r\n");
                            read_ndef_data(nfcDevice);
                        }

                        /* Loop until tag is removed from the field */
                        Serial0.print("Operation completed\r\nTag can be removed from the field\r\n");
                        devCnt = 1;
                        rfalNfcfPollerInitialize(RFAL_BR_212);
                        while (rfalNfcfPollerPoll(RFAL_FELICA_1_SLOT, RFAL_NFCF_SYSTEMCODE, RFAL_FELICA_POLL_RC_NO_REQUEST, cardList, &devCnt, &collisions) == ERR_NONE) {
                        /* Skip the length field byte */
                        sensfRes = (rfalNfcfSensfRes *) & ((uint8_t *)cardList)[1];
                        if (ST_BYTECMP(sensfRes->NFCID2, nfcDevice->dev.nfcf.sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN) != 0) {
                            break;
                        }
                        delay(130);
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCV: 
                        {
                            uint8_t devUID[RFAL_NFCV_UID_LEN];

                            ST_MEMCPY(devUID, nfcDevice->nfcid, nfcDevice->nfcidLen);     /* Copy the UID into local var */
                            REVERSE_BYTES(devUID, RFAL_NFCV_UID_LEN);                   /* Reverse the UID for display purposes */
                            Serial0.print("ISO15693/NFC-V card found. UID: ");
                            Serial0.print(hex2str(devUID, RFAL_NFCV_UID_LEN));
                            Serial0.print("\r\n");

                            read_ndef_data(nfcDevice);

                            /* Loop until tag is removed from the field */
                            Serial0.print("Operation completed\r\nTag can be removed from the field\r\n");
                            rfalNfcvPollerInitialize();
                            while (rfalNfcvPollerInventory(RFAL_NFCV_NUM_SLOTS_1, RFAL_NFCV_UID_LEN * 8U, nfcDevice->dev.nfcv.InvRes.UID, &invRes, &rcvdLen) == ERR_NONE) 
                            {
                                delay(130);
                            }
                        }
                        break;

                    /*******************************************************************************/
                    default:
                        break;
                    }

                    rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
                    delay(500);
                    nfcCurrentState = NFC_POLL_STATE_START_DISCOVERY;
                }
            break;  // CASE NFC_POLL_STATE_DISCOVERY

            /*******************************************************************************/
            case NFC_POLL_STATE_NOTINIT:   // Fall-through ...
            default: break;
        }

        // Pause the task for 100msec:
        vTaskDelay(100 / portTICK_PERIOD_MS);   // NOTE: up to 500ms is OK - but if the total(=incl. 500ms delay above) goes beyond scan period(=1sec), scan may seem to fail(?).
    }
}


/**************************************************** main: SETUP and LOOP **************************************************/

void setup(void) 
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

    nfcCurrentState = NFC_POLL_STATE_START_DISCOVERY;

    xTaskCreate(
        poll_for_nfc_tags,    // Function that should be called
        "NFC-Poll ",       // Name of the task (for debugging)
        4096,            // Stack size (bytes)
        NULL,            // Parameter to pass
        1,               // Task priority
        NULL             // Task handle
    );
}


void loop(void) 
{
    Serial0.println("\r\nALIVE ...\r\n");

    vTaskDelay(10000);
}


