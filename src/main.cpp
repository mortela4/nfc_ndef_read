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

#define DEMO_RAW_MESSAGE_BUF_LEN 8192


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
static uint8_t rawMessageBuf[DEMO_RAW_MESSAGE_BUF_LEN];


/************************************ LOCAL FUNCTION PROTOTYPES ************************************/

static void ndefParseMessage(uint8_t *rawMsgBuf, uint32_t rawMsgLen);
static void ndefParseRecord(ndefRecord *record);
static void ndefPrintString(const uint8_t *str, uint32_t strLen);


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

    rfalNfcDeactivate (RFAL_NFC_DEACTIVATE_IDLE);
	rfalNfcDiscover(&discParam);
}


void loop(void) 
{
	ndefStatus err;
	ReturnCode ret;
	uint32_t rawMessageLen;
	
    rfalNfcWorker();
    
    if ( rfalNfcIsDevActivated(rfalNfcGetState()) ) 
    {
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
        
        return;
    }   			// if ( rfalNfcIsDevActivated(rfalNfcGetState()) ) 

    vTaskDelay(3000);
}


