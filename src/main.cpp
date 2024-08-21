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
#define EXAMPLE_RFAL_POLLER_DEVICES      10    /* Number of devices supported */
#define EXAMPLE_RFAL_POLLER_RF_BUF_LEN   255   /* RF buffer length            */

#define EXAMPLE_RFAL_POLLER_FOUND_NONE   0x00  /* No device found Flag        */
#define EXAMPLE_RFAL_POLLER_FOUND_A      0x01  /* NFC-A device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_B      0x02  /* NFC-B device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_F      0x04  /* NFC-F device found Flag     */
#define EXAMPLE_RFAL_POLLER_FOUND_V      0x08  /* NFC-V device Flag           */

#define NDEF_MESSAGE_BUF_LEN             8192


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


/************************************ LOCAL VARIABLES *****************************************/
static rfalNfcDevice *nfcDevice;

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

static ndefContext ndefCtx;

static uint8_t rawMessageBuf[NDEF_MESSAGE_BUF_LEN];


/************************************ LOCAL FUNCTION PROTOTYPES ************************************/

static void ndefParseMessage(uint8_t *rawMsgBuf, uint32_t rawMsgLen);
static void ndefParseRecord(ndefRecord *record);
static void ndefPrintString(const uint8_t *str, uint32_t strLen);
static void check_discover_retval(const ndefStatus err);


/************************************ LOCAL FUNCTION IMPLEMENTATION ************************************/

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
    /*
	uint32_t i;
	const uint8_t* c;
	c = str;
    */

    String ndefInfo;
	
	for (int i = 0; i < strLen; i++) 
	{
		ndefInfo += str[i];
	}

    ndefInfo += '\0';   // Just in case ...
 
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
		
		default: Serial0.println(" * Other record (??) ..."); break;														
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


/**************************************************** main: SETUP and LOOP **************************************************/

void setup(void) 
{
    ndefStatus err;

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

    rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
	
    err = rfalNfcDiscover(&discParam);
    check_discover_retval(err);
}


/*! Main state                                                                          */
typedef enum
{
    RFAL_POLLER_STATE_INIT                =  0,  /* Initialize state            */
    RFAL_POLLER_STATE_TECHDETECT          =  1,  /* Technology Detection state  */
    RFAL_POLLER_STATE_COLAVOIDANCE        =  2,  /* Collision Avoidance state   */
    RFAL_POLLER_STATE_ACTIVATION          =  3,  /* Activation state            */
    RFAL_POLLER_STATE_DATAEXCHANGE_START  =  4,  /* Data Exchange Start state   */
    RFAL_POLLER_STATE_DATAEXCHANGE_CHECK  =  5,  /* Data Exchange Check state   */
    RFAL_POLLER_STATE_DEACTIVATION        =  9   /* Deactivation state          */
} rfalPollerState_t;


static exampleRfalPollerState  gState = RFAL_POLLER_STATE_INIT;


void loop(void) 
{
	ndefStatus err;
	ReturnCode ret;
	uint32_t rawMessageLen;

    static rfalNfcDevice *nfcDevice;

    rfalNfcWorker();

    vTaskDelay(50);
    platformDelay(1000);
    
    vTaskDelay(50);

    switch( gState )
    {
        /*******************************************************************************/
        case RFAL_POLLER_STATE_INIT:                                     
            
            gTechsFound = EXAMPLE_RFAL_POLLER_FOUND_NONE; 
            gActiveDev  = NULL;
            gDevCnt     = 0;
            
            gState = EXAMPLE_RFAL_POLLER_STATE_TECHDETECT;
            break;
            
            
        /*******************************************************************************/
        case RFAL_POLLER_STATE_TECHDETECT:
            
            if( !exampleRfalPollerTechDetetection() )                             /* Poll for nearby devices in different technologies */
            {
                gState = EXAMPLE_RFAL_POLLER_STATE_DEACTIVATION;                  /* If no device was found, restart loop */
                break;
            }
            
            gState = RFAL_POLLER_STATE_COLAVOIDANCE;                      /* One or more devices found, go to Collision Avoidance */
            break;
            
            
        /*******************************************************************************/
        case RFAL_POLLER_STATE_COLAVOIDANCE:
            
            if( !exampleRfalPollerCollResolution() )                              /* Resolve any eventual collision */
            {
                gState = RFAL_POLLER_STATE_DEACTIVATION;                  /* If Collision Resolution was unable to retrieve any device, restart loop */
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

                /* Retrieve NFC device */
        rfalNfcGetActiveDevice(&nfcDevice);
        /* Perform NDEF Context Initialization */
        err = ndefPollerContextInitialization(&ndefCtx, nfcDevice); 
        if( err != ERR_NONE ) 
        {
            Serial0.print("NDEF NOT DETECTED! ndefPollerContextInitialization() returned: ");
            Serial0.println(err);

            return; 
        }
        
        /* No NDEF context-init error(s). Perform NDEF Detect procedure: */
        err = ndefPollerNdefDetect(&ndefCtx, NULL); 
        if(	err != ERR_NONE ) 
        {
            Serial0.print("NDEF NOT DETECTED! ndefPollerNdefDetect() returned: "); 
            Serial0.println(err);
            
            return;
        }
        
        /* No NDEF-detect error(s). Perform Single NDEF-read procedure 
            * (Single=true: uses L-field from NDEF-Detect), else re-READ the L-field: */
        err = ndefPollerReadRawMessage(&ndefCtx, rawMessageBuf, sizeof(rawMessageBuf), &rawMessageLen, true); 
        if( err != ERR_NONE	) 
        {
            Serial0.print("NDEF message cannot be read! ndefPollerReadRawMessage() returned: "); 
            Serial0.println(err);
            
            return;
        }
        
        Serial0.print("NDEF Read successful\r\n");
        
        /* Parse message content */
        ndefParseMessage(rawMessageBuf, rawMessageLen);
        
        /* Restart discovery */
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
	
            //err = rfalNfcDiscover(&discParam);
            //check_discover_retval(err);

            gState = EXAMPLE_RFAL_POLLER_STATE_ACTIVATION;                        /* Device(s) have been identified, go to Activation */
            break;
        
            
        /*******************************************************************************/
        case RFAL_POLLER_STATE_ACTIVATION:
        case RFAL_POLLER_STATE_DEACTIVATION:
            rfalFieldOff();                                                       /* Turn the Field Off powering down any device nearby */
            platformDelay(20);                                                     /* Remain a certain period with field off */
            gState = EXAMPLE_RFAL_POLLER_STATE_INIT;                              /* Restart the loop */

            break;
        
        
        /*******************************************************************************/
        default:
            return;

	}

    vTaskDelay(3000);
}


