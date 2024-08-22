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

static uint8_t                 gDevCnt;                                 /* Number of devices found                         */
static RfalPollerDevice         gDevList[RFAL_POLLER_DEVICES];                  /* Device List                                     */
static RfalPollerState          gState;                                  /* Main state                                      */
static uint8_t                 gTechsFound;                             /* Technologies found bitmask                      */
RfalPollerDevice                *gActiveDev;                             /* Active device pointer                           */
static uint16_t                gRcvLen;                                 /* Received length                                 */
static bool                    gRxChaining;                             /* Rx chaining flag                                */


/************************************ LOCAL FUNCTION PROTOTYPES ************************************/

static void ndefParseMessage(uint8_t *rawMsgBuf, uint32_t rawMsgLen);
static void ndefParseRecord(ndefRecord *record);
static void ndefPrintString(const uint8_t *str, uint32_t strLen);
static void check_discover_retval(const ndefStatus err);
static bool RfalPollerTechDetetection( void );
static bool RfalPollerCollResolution( void );
static bool RfalPollerActivation( uint8_t devIt );
static bool RfalPollerNfcDepActivate( RfalPollerDevice *device );
//static ReturnCode RfalPollerDataExchange( void );
//static bool RfalPollerDeactivate( void );


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


/*
******************************************************************************
* LOCAL NFC-POLL FUNCTION IMPLEMENTATION
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
static bool RfalPollerTechDetetection( void )
{
    ReturnCode           err;
    rfalNfcaSensRes      sensRes;
    rfalNfcbSensbRes     sensbRes;
    rfalNfcvInventoryRes invRes;
    uint8_t              sensbResLen;
    
    gTechsFound = RFAL_POLLER_FOUND_NONE;
    
    /*******************************************************************************/
    /* NFC-A Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcaPollerInitialize();                                                       /* Initialize RFAL for NFC-A */
    rfalFieldOnAndStartGT();                                                          /* Turns the Field On and starts GT timer */
    
    err = rfalNfcaPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensRes ); /* Poll for NFC-A devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= RFAL_POLLER_FOUND_A;
    }
    
    
    /*******************************************************************************/
    /* NFC-B Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcbPollerInitialize();                                                       /* Initialize RFAL for NFC-B */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcbPollerTechnologyDetection( RFAL_COMPLIANCE_MODE_NFC, &sensbRes, &sensbResLen ); /* Poll for NFC-B devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= RFAL_POLLER_FOUND_B;
    }
    
    
    /*******************************************************************************/
    /* NFC-F Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcfPollerInitialize( RFAL_BR_212 );                                          /* Initialize RFAL for NFC-F */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcfPollerCheckPresence();                                              /* Poll for NFC-F devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= RFAL_POLLER_FOUND_F;
    }
    
    
    /*******************************************************************************/
    /* NFC-V Technology Detection                                                  */
    /*******************************************************************************/
    
    rfalNfcvPollerInitialize();                                                       /* Initialize RFAL for NFC-V */
    rfalFieldOnAndStartGT();                                                          /* As field is already On only starts GT timer */
    
    err = rfalNfcvPollerCheckPresence( &invRes );                                     /* Poll for NFC-V devices */
    if( err == RFAL_ERR_NONE )
    {
        gTechsFound |= RFAL_POLLER_FOUND_V;
    }
    
    return (gTechsFound != RFAL_POLLER_FOUND_NONE);
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
static bool RfalPollerCollResolution( void )
{
    uint8_t    i;
    uint8_t    devCnt;
    ReturnCode err;
    
    
    /*******************************************************************************/
    /* NFC-A Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & RFAL_POLLER_FOUND_A )                                   /* If a NFC-A device was found/detected, perform Collision Resolution */
    {
        rfalNfcaListenDevice nfcaDevList[RFAL_POLLER_DEVICES];
        
        rfalNfcaPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcaPollerFullCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (RFAL_POLLER_DEVICES - gDevCnt), nfcaDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfca list into global device list */
            {
                gDevList[gDevCnt].type     = RFAL_POLLER_TYPE_NFCA;
                gDevList[gDevCnt].dev.nfca = nfcaDevList[i];
                gDevCnt++;
            }
        }
    }
    
    /*******************************************************************************/
    /* NFC-B Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & RFAL_POLLER_FOUND_B )                                   /* If a NFC-A device was found/detected, perform Collision Resolution */
    {
        rfalNfcbListenDevice nfcbDevList[RFAL_POLLER_DEVICES];
        
        rfalNfcbPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcbPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (RFAL_POLLER_DEVICES - gDevCnt), nfcbDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfcb list into global device list */
            {
                gDevList[gDevCnt].type     = RFAL_POLLER_TYPE_NFCB;
                gDevList[gDevCnt].dev.nfcb = nfcbDevList[i];
                gDevCnt++;
            }
        }
    }
    
    
    /*******************************************************************************/
    /* NFC-F Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & RFAL_POLLER_FOUND_F )                                   /* If a NFC-F device was found/detected, perform Collision Resolution */
    {
        rfalNfcfListenDevice nfcfDevList[RFAL_POLLER_DEVICES];
        
        rfalNfcfPollerInitialize( RFAL_BR_212 );
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcfPollerCollisionResolution( RFAL_COMPLIANCE_MODE_NFC, (RFAL_POLLER_DEVICES - gDevCnt), nfcfDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                 /* Copy devices found form local Nfcf list into global device list */
            {
                gDevList[gDevCnt].type     = RFAL_POLLER_TYPE_NFCF;
                gDevList[gDevCnt].dev.nfcf = nfcfDevList[i];
                gDevCnt++;
            }
        }
    }
    
    /*******************************************************************************/
    /* NFC-V Collision Resolution                                                  */
    /*******************************************************************************/
    if( gTechsFound & RFAL_POLLER_FOUND_V )                                   /* If a NFC-F device was found/detected, perform Collision Resolution */
    {
        rfalNfcvListenDevice nfcvDevList[RFAL_POLLER_DEVICES];
        
        rfalNfcvPollerInitialize();
        rfalFieldOnAndStartGT();                                                      /* Ensure GT again as other technologies have also been polled */
        err = rfalNfcvPollerCollisionResolution(RFAL_COMPLIANCE_MODE_NFC, (RFAL_POLLER_DEVICES - gDevCnt), nfcvDevList, &devCnt );
        if( (err == RFAL_ERR_NONE) && (devCnt != 0) )
        {
            for( i=0; i<devCnt; i++ )                                                /* Copy devices found form local Nfcf list into global device list */
            {
                gDevList[gDevCnt].type     = RFAL_POLLER_TYPE_NFCV;
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
static bool RfalPollerActivation( uint8_t devIt )
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
        case RFAL_POLLER_TYPE_NFCA:
            
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
                    
                    gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_RF;
                    break;
                    
                
                /*******************************************************************************/
                case RFAL_NFCA_T2T:
                  
                    /* No specific activation needed for a T2T */    
                    Serial0.print("NFC-A T2T device activated \r\n");                   /* NFC-A T2T device activated */
                    
                    gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_RF;
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
                    
                    gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_ISODEP;
                    break;
                  
                  
                /*******************************************************************************/
                case RFAL_NFCA_T4T_NFCDEP:                                              /* Device supports both T4T and NFC-DEP */
                case RFAL_NFCA_NFCDEP:                                                  /* Device supports NFC-DEP */
                  
                    /* Perform NFC-DEP (P2P) activation: ATR and PSL if supported */
                    if( !RfalPollerNfcDepActivate( &gDevList[devIt] ) )
                    {
                      return false;
                    }
                    
                    Serial0.print("NFC-A P2P (NFC-DEP) device activated \r\n");         /* NFC-A P2P device activated */
                    gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_NFCDEP;
                    break;
            }
            
            break;
        
        /*******************************************************************************/
        /* NFC-B Activation                                                            */
        /*******************************************************************************/
        case RFAL_POLLER_TYPE_NFCB:
            
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
                
                gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_ISODEP ;
                break;
            }
            
            Serial0.print("NFC-B device activated \r\n");                               /* NFC-B  device activated */
            gDevList[devIt].rfInterface =  RFAL_POLLER_INTERFACE_RF;
            break;
            
        /*******************************************************************************/
        /* NFC-F Activation                                                            */
        /*******************************************************************************/
        case RFAL_POLLER_TYPE_NFCF:
            
            rfalNfcfPollerInitialize( RFAL_BR_212 );
            if( rfalNfcfIsNfcDepSupported( &gDevList[devIt].dev.nfcf ) )
            {
                /* Perform NFC-DEP (P2P) activation: ATR and PSL if supported */
                if( !RfalPollerNfcDepActivate( &gDevList[devIt] ) )
                {
                    return false;
                }
                
                Serial0.print("NFC-F P2P (NFC-DEP) device activated \r\n");             /* NFC-A P2P device activated */
                
                gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_NFCDEP;
                break;
            }
            
            Serial0.print("NFC-F T3T device activated \r\n");                           /* NFC-F T3T device activated */
            gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_RF;
            break;
            
        /*******************************************************************************/
        /* NFC-V Activation                                                            */
        /*******************************************************************************/
        case RFAL_POLLER_TYPE_NFCV:
            
            rfalNfcvPollerInitialize();
            
            /* No specific activation needed for a T5T */
            Serial0.print("NFC-V T5T device activated \r\n");                           /* NFC-V T5T device activated */
            
            gDevList[devIt].rfInterface = RFAL_POLLER_INTERFACE_RF;
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
static bool RfalPollerNfcDepActivate( RfalPollerDevice *device )
{
    rfalNfcDepAtrParam   param;
                
    /*******************************************************************************/
    /* If Passive F use the NFCID2 retrieved from SENSF                            */
    if( device->type == RFAL_POLLER_TYPE_NFCF )
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
            
            gTechsFound = RFAL_POLLER_FOUND_NONE; 
            gActiveDev  = NULL;
            gDevCnt     = 0;
            
            gState = RFAL_POLLER_STATE_TECHDETECT;
            break;
            
            
        /*******************************************************************************/
        case RFAL_POLLER_STATE_TECHDETECT:
            
            if( !RfalPollerTechDetetection() )                             /* Poll for nearby devices in different technologies */
            {
                gState = RFAL_POLLER_STATE_DEACTIVATION;                  /* If no device was found, restart loop */
                break;
            }
            
            gState = RFAL_POLLER_STATE_COLAVOIDANCE;                      /* One or more devices found, go to Collision Avoidance */
            break;
            
            
        /*******************************************************************************/
        case RFAL_POLLER_STATE_COLAVOIDANCE:
            
            if( !RfalPollerCollResolution() )                              /* Resolve any eventual collision */
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
                    case RFAL_POLLER_TYPE_NFCA:
                        Serial0.print( " NFC-A device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfca.nfcId1, gDevList[i].dev.nfca.nfcId1Len) );
                        //platformLedOn( LED_NFCA_PORT, LED_NFCA_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case RFAL_POLLER_TYPE_NFCB:
                        Serial0.print( " NFC-B device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfcb.sensbRes.nfcid0, RFAL_NFCB_NFCID0_LEN) );
                        //platformLedOn( LED_NFCB_PORT, LED_NFCB_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case RFAL_POLLER_TYPE_NFCF:
                        Serial0.print( " NFC-F device UID: "); 
                        Serial0.println( hex2str(gDevList[i].dev.nfcf.sensfRes.NFCID2, RFAL_NFCF_NFCID2_LEN) );
                        //platformLedOn( LED_NFCF_PORT, LED_NFCF_PIN  );
                        platformLedOn(LED_TAG_READ_PORT, LED_TAG_READ_PIN); 
                        break;
                        
                    case RFAL_POLLER_TYPE_NFCV:
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
            }
                
            /* Restart discovery */
            rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        
            //err = rfalNfcDiscover(&discParam);
            //check_discover_retval(err);

            gState = RFAL_POLLER_STATE_ACTIVATION;                        /* Device(s) have been identified, go to Activation */
            break;
               
        /*******************************************************************************/
        case RFAL_POLLER_STATE_ACTIVATION:
        case RFAL_POLLER_STATE_DEACTIVATION:
            rfalFieldOff();                                                       /* Turn the Field Off powering down any device nearby */
            platformDelay(20);                                                     /* Remain a certain period with field off */
            gState = RFAL_POLLER_STATE_INIT;                              /* Restart the loop */

            break;
        
        
        /*******************************************************************************/
        default:
            return;

	}

    vTaskDelay(3000);
}


