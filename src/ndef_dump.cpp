/**
 * @file ndef_dump.cpp
 * 
 * @brief NDEF-record readout functionality. 
 */

#if 0
#include "rfal_platform/rfal_platform.h"


extern "C" {
#include "rfal_core/rfal_nfc.h"             // Includes all of "rfal_nfc[a|b|f|v].h", "rfal_isoDep.h" and "rfal_nfcDep.h".
#include "ndef/poller/ndef_poller.h"
#include "ndef/poller/ndef_t2t.h"
#include "ndef/poller/ndef_t4t.h"
#include "ndef/poller/ndef_t5t.h"
#include "ndef/message/ndef_message.h"
#include "ndef/message/ndef_types.h"

#include "ndef_dump.h"
}


/****************** Local function protos ****************/

#if NDEF_FEATURE_FULL_API
static bool ndefIsSTTag(ndefContext *ctx);
#endif
#if NDEF_FEATURE_T2T
static void ndefT2TCCDump(ndefContext *ctx);
#endif /* NDEF_FEATURE_T2T */

#if NDEF_FEATURE_T3T
static void ndefT3TAIBDump(ndefContext *ctx);
#endif /* NDEF_FEATURE_T3T */

#if NDEF_FEATURE_T4T
static void ndefT4TCCDump(ndefContext *ctx);
#endif /* NDEF_FEATURE_T4T */

static void ndefCCDump(ndefContext *ctx);

#if NDEF_FEATURE_T5T
static void ndefT5TCCDump(ndefContext *ctx);
static void ndefDumpSysInfo(ndefContext *ctx)
#endif /* NDEF_FEATURE_T5T */



/*************************** Global (exported) functions implementation. */

void ndef_dump(rfalNfcDevice *pNfcDevice)
{
    ReturnCode       err;
    ndefMessage      message;
    uint32_t         rawMessageLen;
    ndefInfo         info;
    ndefBuffer       bufRawMessage;
    ndefConstBuffer  bufConstRawMessage;
 
#if NDEF_FEATURE_FULL_API
    ndefRecord       record1;
    ndefRecord       record2;

    ndefType         text;
    ndefType         uri;
    ndefType         aar;

    ndefConstBuffer8 bufTextLangCode;
    ndefConstBuffer bufTextLangText;
    ndefConstBuffer bufUri;
    ndefConstBuffer bufAndroidPackName;
#endif /* NDEF_FEATURE_FULL_API */

    /*
     * Perform NDEF Context Initialization
     */
    err = ndefPollerContextInitialization(&ndefCtx, pNfcDevice);
    if( err != ERR_NONE )
    {
        platformLog("NDEF NOT DETECTED (ndefPollerContextInitialization returns %d)\r\n", err);
        return;
    }
    
    #if NDEF_FEATURE_T5T
    if( verbose && (pNfcDevice->type == RFAL_NFC_LISTEN_TYPE_NFCV) )
    {
        ndefDumpSysInfo(&ndefCtx);
    }
    #endif /* RFAL_FEATURE_T5T */
    
    /*
     * Perform NDEF Detect procedure
     */
    err = ndefPollerNdefDetect(&ndefCtx, &info);
    if( err != ERR_NONE )
    {
        platformLog("NDEF NOT DETECTED (ndefPollerNdefDetect returns %d)\r\n", err);
        if( ndefDemoFeature != NDEF_DEMO_FORMAT_TAG)
        {
            return;
        }
    }
    else
    {
        platformLog("%s NDEF detected.\r\n", ndefStates[info.state]);
        ndefCCDump(&ndefCtx);

        if( verbose )
        {
            platformLog("NDEF Len: %d, Offset=%d\r\n", ndefCtx.messageLen, ndefCtx.messageOffset);
        }
    }

    switch( ndefDemoFeature )
    {
        /*
         * Demonstrate how to read the NDEF message from the Tag
         */
        case NDEF_DEMO_READ:
            if( info.state == NDEF_STATE_INITIALIZED )
            {
                /* Nothing to read... */
                return;
            }
            err = ndefPollerReadRawMessage(&ndefCtx, rawMessageBuf, sizeof(rawMessageBuf), &rawMessageLen, true);
            if( err != ERR_NONE )
            {
                platformLog("NDEF message cannot be read (ndefPollerReadRawMessage returns %d)\r\n", err);
                return;
            }
            if( verbose )
            {
                bufRawMessage.buffer = rawMessageBuf;
                bufRawMessage.length = rawMessageLen;
                ndefBufferDump(" NDEF Content", (ndefConstBuffer*)&bufRawMessage, verbose);
            }
            bufConstRawMessage.buffer = rawMessageBuf;
            bufConstRawMessage.length = rawMessageLen;
            err = ndefMessageDecode(&bufConstRawMessage, &message);
            if( err != ERR_NONE )
            {
                platformLog("NDEF message cannot be decoded (ndefMessageDecode returns %d)\r\n", err);
                return;
            }
            err = ndefMessageDump(&message, verbose);
            if( err != ERR_NONE )
            {
                platformLog("NDEF message cannot be displayed (ndefMessageDump returns %d)\r\n", err);
                return;
            }
            break;

#if NDEF_FEATURE_FULL_API
        /*
         * Demonstrate how to encode a text record and write the message to the tag
         */
        case NDEF_DEMO_WRITE_MSG1:
            ndefDemoFeature = NDEF_DEMO_READ; /* returns to READ mode after write */
            err  = ndefMessageInit(&message); /* Initialize message structure */
            bufTextLangCode.buffer = ndefTextLangCode;
            bufTextLangCode.length = strlen((char *)ndefTextLangCode);

            bufTextLangText.buffer = ndefTEXT;
            bufTextLangText.length = strlen((char *)ndefTEXT);

            err |= ndefRtdTextInit(&text, TEXT_ENCODING_UTF8, &bufTextLangCode, &bufTextLangText); /* Initialize Text type structure */
            err |= ndefRtdTextToRecord(&text, &record1); /* Encode Text Record */
            err |= ndefMessageAppend(&message, &record1); /* Append Text record to message */
            if( err != ERR_NONE )
            {
                platformLog("Message creation failed (%d)\r\n", err);
                return;
            }
            err = ndefPollerWriteMessage(&ndefCtx, &message); /* Write message */
            if( err != ERR_NONE )
            {
                platformLog("Message cannot be written (ndefPollerWriteMessage return %d)\r\n", err);
                return;
            }
            platformLog("Wrote 1 record to the Tag\r\n");
            if( verbose )
            {
                /* Dump raw message */
                bufRawMessage.buffer = rawMessageBuf;
                bufRawMessage.length = sizeof(rawMessageBuf);
                err = ndefMessageEncode(&message, &bufRawMessage);
                if( err == ERR_NONE )
                {
                    ndefBufferDump("Raw message", (ndefConstBuffer*)&bufRawMessage, verbose);
                }
            }
            LedNotificationWriteDone();
            break;

        /*
         * Demonstrate how to encode a URI record and a AAR record, how to encode the message to a raw buffer and then how to write the raw buffer
         */
        case NDEF_DEMO_WRITE_MSG2:
            ndefDemoFeature = NDEF_DEMO_READ;  /* returns to READ mode after write */
            err  = ndefMessageInit(&message);  /* Initialize message structure */
            bufUri.buffer = ndefURI;
            bufUri.length = strlen((char *)ndefURI);
            err |= ndefRtdUriInit(&uri, NDEF_URI_PREFIX_HTTP_WWW, &bufUri); /* Initialize URI type structure */
            err |= ndefRtdUriToRecord(&uri, &record1); /* Encode URI Record */

            bufAndroidPackName.buffer = ndefAndroidPackName;
            bufAndroidPackName.length = sizeof(ndefAndroidPackName) - 1U;
            err |= ndefRtdAarInit(&aar, &bufAndroidPackName); /* Initialize AAR type structure */
            err |= ndefRtdAarToRecord(&aar, &record2); /* Encode AAR record */

            err |= ndefMessageAppend(&message, &record1); /* Append URI to message */
            err |= ndefMessageAppend(&message, &record2); /* Append AAR to message (record #2 is an example of preformatted record) */

            bufRawMessage.buffer = rawMessageBuf;
            bufRawMessage.length = sizeof(rawMessageBuf);
            err |= ndefMessageEncode(&message, &bufRawMessage); /* Encode the message to the raw buffer */
            if( err != ERR_NONE )
            {
                platformLog("Raw message creation failed (%d)\r\n", err);
                return;
            }
            err = ndefPollerWriteRawMessage(&ndefCtx, bufRawMessage.buffer, bufRawMessage.length);
            if( err != ERR_NONE )
            {
                platformLog("Message cannot be written (ndefPollerWriteRawMessage return %d)\r\n", err);
                return;
            }
            platformLog("Wrote 2 records to the Tag\r\n");
            if( verbose )
            {
                /* Dump raw message */
                ndefBufferDump("Raw message", (ndefConstBuffer*)&bufRawMessage, verbose);
            }
            LedNotificationWriteDone();
            break;

        /*
         * Demonstrate how to format a Tag
         */
        case NDEF_DEMO_FORMAT_TAG:
            ndefDemoFeature = NDEF_DEMO_READ;
            if( !ndefIsSTTag(&ndefCtx) )
            {
                platformLog("Manufacturer ID not found or not an ST tag. Format aborted \r\n");
                return;
            }
            platformLog("Formatting Tag...\r\n");
            /* Format Tag */
            err = ndefPollerTagFormat(&ndefCtx, NULL, 0);
            if( err != ERR_NONE )
            {
                platformLog("Tag cannot be formatted (ndefPollerTagFormat returns %d)\r\n", err);
                return;
            }
            platformLog("Tag formatted\r\n");
            LedNotificationWriteDone();
            break;
#endif /* NDEF_FEATURE_FULL_API */

        default:
            ndefDemoFeature = NDEF_DEMO_READ;
            break;
    }
    return;
}

#if NDEF_FEATURE_T2T
static void ndefT2TCCDump(ndefContext *ctx)
{
    ndefConstBuffer bufCcBuf;

    platformLog(" * Magic: %2.2Xh Version: %d.%d Size: %d (%d bytes) \r\n * readAccess: %2.2xh writeAccess: %2.2xh \r\n", ctx->cc.t2t.magicNumber, ctx->cc.t2t.majorVersion, ctx->cc.t2t.minorVersion, ctx->cc.t2t.size, ctx->cc.t2t.size * 8U, ctx->cc.t2t.readAccess, ctx->cc.t2t.writeAccess);
    bufCcBuf.buffer = ctx->ccBuf;
    bufCcBuf.length = 4;
    ndefBufferDump(" CC Raw Data", &bufCcBuf, verbose);
  
}
#endif /* NDEF_FEATURE_T2T */

#if NDEF_FEATURE_T3T
static void ndefT3TAIBDump(ndefContext *ctx)
{
    ndefConstBuffer bufCcBuf;

    platformLog(" * Version: %d.%d Size: %d (%d bytes) NbR: %d NbW: %d\r\n * WriteFlag: %2.2xh RWFlag: %2.2xh \r\n", ctx->cc.t3t.majorVersion, ctx->cc.t3t.minorVersion, ctx->cc.t3t.nMaxB, ctx->cc.t3t.nMaxB * 16U, ctx->cc.t3t.nbR, ctx->cc.t3t.nbW, ctx->cc.t3t.writeFlag, ctx->cc.t3t.rwFlag);
    bufCcBuf.buffer = ctx->ccBuf;
    bufCcBuf.length = 16;
    ndefBufferDump(" CC Raw Data", &bufCcBuf, verbose);
}
#endif /* NDEF_FEATURE_T3T */

#if NDEF_FEATURE_T4T
static void ndefT4TCCDump(ndefContext *ctx)
{
    ndefConstBuffer bufCcBuf;
    
    platformLog(" * CCLEN: %d T4T_VNo: %xh MLe: %d MLc: %d FileId: %2.2x%2.2xh FileSize: %d\r\n * readAccess: %2.2xh writeAccess: %2.2xh\r\n", ctx->cc.t4t.ccLen, ctx->cc.t4t.vNo, ctx->cc.t4t.mLe, ctx->cc.t4t.mLc, ctx->cc.t4t.fileId[0], ctx->cc.t4t.fileId[1],ctx->cc.t4t.fileSize, ctx->cc.t4t.readAccess, ctx->cc.t4t.writeAccess);
    bufCcBuf.buffer = ctx->ccBuf;
    bufCcBuf.length = ctx->cc.t4t.ccLen;
    ndefBufferDump(" CC File Raw Data", &bufCcBuf, verbose);
}
#endif /* NDEF_FEATURE_T4T */

#if NDEF_FEATURE_T5T
static void ndefT5TCCDump(ndefContext *ctx)
{
    ndefConstBuffer bufCcBuf;
    
    platformLog(" * Block Length: %d\r\n", ctx->subCtx.t5t.blockLen);
    platformLog(" * %d bytes CC\r\n * Magic: %2.2Xh Version: %d.%d MLEN: %d (%d bytes) \r\n * readAccess: %2.2xh writeAccess: %2.2xh \r\n", ctx->cc.t5t.ccLen, ctx->cc.t5t.magicNumber, ctx->cc.t5t.majorVersion, ctx->cc.t5t.minorVersion, ctx->cc.t5t.memoryLen, ctx->cc.t5t.memoryLen * 8U, ctx->cc.t5t.readAccess, ctx->cc.t5t.writeAccess);
    platformLog(" * [%c] Special Frame\r\n",       ctx->cc.t5t.specialFrame ?      'X' : ' ');
    platformLog(" * [%c] Multiple block Read\r\n", ctx->cc.t5t.multipleBlockRead ? 'X' : ' ');
    platformLog(" * [%c] Lock Block\r\n",          ctx->cc.t5t.lockBlock ?         'X' : ' ');
    bufCcBuf.buffer = ctx->ccBuf;
    bufCcBuf.length = ctx->cc.t5t.ccLen;
    ndefBufferDump(" CC Raw Data", &bufCcBuf, verbose);
}
#endif /* NDEF_FEATURE_T5T */

static void ndefCCDump(ndefContext *ctx)
{
    if( (ctx == NULL) || !verbose)
    {
        return;
    }
    platformLog("%s", (ctx->type == NDEF_DEV_T3T) ? "NDEF Attribute Information Block\r\n" : "NDEF Capability Container\r\n");
    switch( ctx->type )
    {
        #if NDEF_FEATURE_T2T
        case NDEF_DEV_T2T:
            ndefT2TCCDump(ctx);
            break;
        #endif /* NDEF_FEATURE_T2T */
        #if NDEF_FEATURE_T3T
        case NDEF_DEV_T3T:
            ndefT3TAIBDump(ctx);
            break;
        #endif /* NDEF_FEATURE_T3T */
        #if NDEF_FEATURE_T4T
        case NDEF_DEV_T4T:
            ndefT4TCCDump(ctx);
            break;
        #endif /* NDEF_FEATURE_T4T */
        #if NDEF_FEATURE_T5T
        case NDEF_DEV_T5T:
            ndefT5TCCDump(ctx);
            break;
        #endif /* NDEF_FEATURE_T5T */
        default:
            break;
    }
}

#if NDEF_FEATURE_T5T
static void ndefDumpSysInfo(ndefContext *ctx)
{
    ndefSystemInformation *sysInfo;

    if( (ctx == NULL) || !verbose)
    {
        return;
    }
    
    if( !ctx->subCtx.t5t.sysInfoSupported )
    {
        return;
    }
    
    sysInfo = &ctx->subCtx.t5t.sysInfo;
    platformLog("System Information\r\n");
    platformLog(" * %d byte(s) memory addressing\r\n", ndefT5TSysInfoMOIValue(sysInfo->infoFlags) + 1);
    if( ndefT5TSysInfoDFSIDPresent(sysInfo->infoFlags) )
    {
        platformLog(" * DFSID=%2.2Xh\r\n", sysInfo->DFSID);
    }
    if( ndefT5TSysInfoAFIPresent(sysInfo->infoFlags) )
    {
        platformLog(" * AFI=%2.2Xh\r\n", sysInfo->AFI);
    }
    if( ndefT5TSysInfoMemSizePresent(sysInfo->infoFlags) )
    {
        platformLog(" * %d blocks, %d bytes per block\r\n", sysInfo->numberOfBlock, sysInfo->blockSize);
    }
    if( ndefT5TSysInfoICRefPresent(sysInfo->infoFlags) )
    {
        platformLog(" * ICRef=%2.2xh\r\n", sysInfo->ICRef);
    }
    if( ndefT5TSysInfoCmdListPresent(sysInfo->infoFlags) )
    {
        platformLog(" * [%c] ReadSingleBlock                \r\n", ndefT5TSysInfoReadSingleBlockSupported(sysInfo->supportedCmd)                 ? 'X' : ' ');               
        platformLog(" * [%c] WriteSingleBlock               \r\n", ndefT5TSysInfoWriteSingleBlockSupported(sysInfo->supportedCmd)                ? 'X' : ' ');
        platformLog(" * [%c] LockSingleBlock                \r\n", ndefT5TSysInfoLockSingleBlockSupported(sysInfo->supportedCmd)                 ? 'X' : ' ');
        platformLog(" * [%c] ReadMultipleBlocks             \r\n", ndefT5TSysInfoReadMultipleBlocksSupported(sysInfo->supportedCmd)              ? 'X' : ' ');
        platformLog(" * [%c] WriteMultipleBlocks            \r\n", ndefT5TSysInfoWriteMultipleBlocksSupported(sysInfo->supportedCmd)             ? 'X' : ' ');
        platformLog(" * [%c] Select                         \r\n", ndefT5TSysInfoSelectSupported(sysInfo->supportedCmd)                          ? 'X' : ' ');
        platformLog(" * [%c] ResetToReady                   \r\n", ndefT5TSysInfoResetToReadySupported(sysInfo->supportedCmd)                    ? 'X' : ' ');
        platformLog(" * [%c] GetMultipleBlockSecStatus      \r\n", ndefT5TSysInfoGetMultipleBlockSecStatusSupported(sysInfo->supportedCmd)       ? 'X' : ' ');
        platformLog(" * [%c] WriteAFI                       \r\n", ndefT5TSysInfoWriteAFISupported(sysInfo->supportedCmd)                        ? 'X' : ' ');
        platformLog(" * [%c] LockAFI                        \r\n", ndefT5TSysInfoLockAFISupported(sysInfo->supportedCmd)                         ? 'X' : ' ');
        platformLog(" * [%c] WriteDSFID                     \r\n", ndefT5TSysInfoWriteDSFIDSupported(sysInfo->supportedCmd)                      ? 'X' : ' ');
        platformLog(" * [%c] LockDSFID                      \r\n", ndefT5TSysInfoLockDSFIDSupported(sysInfo->supportedCmd)                       ? 'X' : ' ');
        platformLog(" * [%c] GetSystemInformation           \r\n", ndefT5TSysInfoGetSystemInformationSupported(sysInfo->supportedCmd)            ? 'X' : ' ');
        platformLog(" * [%c] CustomCmds                     \r\n", ndefT5TSysInfoCustomCmdsSupported(sysInfo->supportedCmd)                      ? 'X' : ' ');
        platformLog(" * [%c] FastReadMultipleBlocks         \r\n", ndefT5TSysInfoFastReadMultipleBlocksSupported(sysInfo->supportedCmd)          ? 'X' : ' ');
        platformLog(" * [%c] ExtReadSingleBlock             \r\n", ndefT5TSysInfoExtReadSingleBlockSupported(sysInfo->supportedCmd)              ? 'X' : ' '); 
        platformLog(" * [%c] ExtWriteSingleBlock            \r\n", ndefT5TSysInfoExtWriteSingleBlockSupported(sysInfo->supportedCmd)             ? 'X' : ' ');
        platformLog(" * [%c] ExtLockSingleBlock             \r\n", ndefT5TSysInfoExtLockSingleBlockSupported(sysInfo->supportedCmd)              ? 'X' : ' ');
        platformLog(" * [%c] ExtReadMultipleBlocks          \r\n", ndefT5TSysInfoExtReadMultipleBlocksSupported(sysInfo->supportedCmd)           ? 'X' : ' ');
        platformLog(" * [%c] ExtWriteMultipleBlocks         \r\n", ndefT5TSysInfoExtWriteMultipleBlocksSupported(sysInfo->supportedCmd)          ? 'X' : ' ');
        platformLog(" * [%c] ExtGetMultipleBlockSecStatus   \r\n", ndefT5TSysInfoExtGetMultipleBlockSecStatusSupported(sysInfo->supportedCmd)    ? 'X' : ' ');
        platformLog(" * [%c] FastExtendedReadMultipleBlocks \r\n", ndefT5TSysInfoFastExtendedReadMultipleBlocksSupported(sysInfo->supportedCmd)  ? 'X' : ' ');
    }
    return;
}
#endif /* NDEF_FEATURE_T5T */

#if NDEF_FEATURE_FULL_API
static bool ndefIsSTTag(ndefContext *ctx)
{
    bool ret = false;

#if defined(STM32L476xx) /* Enable to format any manufacturer tag while debugging */
    if( (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0)
    {
        ret = true;
    }
#endif
    if( ctx == NULL )
    {   
        return ret;
    }
    switch (ctx->device.type)
    {
        case RFAL_NFC_LISTEN_TYPE_NFCA:
            if( (ctx->device.dev.nfca.nfcId1Len != 4) && (ctx->device.dev.nfca.nfcId1[0] == 0x02 ) )
            {  
                ret = true;
            }
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCF:
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCB:
            break;
        case RFAL_NFC_LISTEN_TYPE_NFCV:
            if( ctx->device.dev.nfcv.InvRes.UID[6] == 0x02 )
            {  
                ret = true;
            }
            break;
        default:
            break;
    }
    return (ret);
}
#endif /* NDEF_FEATURE_FULL_API */


#endif   // IF 0

