/*===========================================================================
FILE:
   QMI.c

DESCRIPTION:
   Qualcomm QMI driver code

FUNCTIONS:
   Generic QMUX functions
      ParseQMUX
      FillQMUX

   Generic QMI functions
      GetTLV
      ValidQMIMessage
      GetQMIMessageID

   Fill Buffers with QMI requests
      QMICTLGetClientIDReq
      QMICTLReleaseClientIDReq
      QMICTLReadyReq
      QMIWDSSetEventReportReq
      QMIWDSGetPKGSRVCStatusReq
      QMIDMSGetMEIDReq
      QMIDMSSWISetFCCAuthReq
      QMIWDASetDataFormatReq
      QMICTLSetDataFormatReq
      QMICTLSyncReq

   Parse data from QMI responses
      QMICTLGetClientIDResp
      QMICTLReleaseClientIDResp
      QMIWDSEventResp
      QMIDMSGetMEIDResp
      QMIWDASetDataFormatResp
      QMICTLSetDataFormatResp
      QMICTLSyncResp

Copyright (c) 2011, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

Alternatively, provided that this notice is retained in full, this software
may be relicensed by the recipient under the terms of the GNU General Public
License version 2 ("GPL") and only version 2, in which case the provisions of
the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
software under the GPL, then the identification text in the MODULE_LICENSE
macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
recipient changes the license terms to the GPL, subsequent recipients shall
not relicense under alternate licensing terms, including the BSD or dual
BSD/GPL terms.  In addition, the following license statement immediately
below and between the words START and END shall also then apply when this
software is relicensed under the GPL:

START

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License version 2 and only version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

END

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#include <asm/unaligned.h>
#include <linux/kernel.h>
#include "Structs.h"
#include "QMI.h"

extern int debug;

#define QMIWDASETDATAFORMATQMAPREQSIZE 39
/*=========================================================================*/
// Get sizes of buffers needed by QMI requests
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMUXHeaderSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMUXHeaderSize( void )
{
   return sizeof( sQMUX );
}

/*===========================================================================
METHOD:
   QMICTLGetClientIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLGetClientIDReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLGetClientIDReqSize( void )
{
   return sizeof( sQMUX ) + 10;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLReleaseClientIDReq

RETURN VALUE:
   u16 - size of header
===========================================================================*/
u16 QMICTLReleaseClientIDReqSize( void )
{
   return sizeof( sQMUX ) + 11;
}

/*===========================================================================
METHOD:
   QMICTLReadyReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLReadyReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLReadyReqSize( void )
{
   return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMIWDSSetEventReportReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSSetEventReportReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSSetEventReportReqSize( void )
{
   return sizeof( sQMUX ) + 15;
}

/*===========================================================================
METHOD:
   QMIWDSGetPKGSRVCStatusReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDSGetPKGSRVCStatusReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDSGetPKGSRVCStatusReqSize( void )
{
   return sizeof( sQMUX ) + 7;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEIDReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIDMSGetMEIDReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIDMSGetMEIDReqSize( void )
{
   return sizeof( sQMUX ) + 7;
}

/*===========================================================================
METHOD:
   QMIDMSSWISetFCCAuthReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIDMSSWISetFCCAuthReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIDMSSWISetFCCAuthReqSize( void )
{
   return sizeof( sQMUX ) + 7;
}

/*===========================================================================
METHOD:
   QMIWDASetDataFormatReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDASetDataFormatReq

PARAMETERS
   te_flow_control [ I ] - TE Flow Control Flag
                          - eSKIP_TE_FLOW_CONTROL_TLV - Not Set TE Flow control.
                          - eTE_FLOW_CONTROL_TLV_0 - Set TE Flow Control disabled.
                          - eTE_FLOW_CONTROL_TLV_1 - Set TE Flow Control enabled.

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDASetDataFormatReqSize( int te_flow_control ,int qmuxenable)
{
   u16 uQmuxLength =0;
   if(qmuxenable!=0)
   {
      uQmuxLength = QMIWDASETDATAFORMATQMAPREQSIZE;
   }
   if(te_flow_control!=eSKIP_TE_FLOW_CONTROL_TLV)
   {
      return sizeof( sQMUX ) + 29 + uQmuxLength; /* TE_FLOW_CONTROL */
   }
   else
   {
      return sizeof( sQMUX ) + 25 + uQmuxLength;
   }
}

/*===========================================================================
METHOD:
   QMICTLSetDataFormatReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLSetDataFormatReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16  QMICTLSetDataFormatReqSize( void )
{
   return sizeof( sQMUX ) + 15;
}

/*===========================================================================
METHOD:
   QMICTLSyncReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLSyncReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16  QMICTLSyncReqSize( void )
{
   return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMICTLGetVersionInfoReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLGetVersionInfoReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16  QMICTLGetVersionInfoReqSize( void )
{
   return sizeof( sQMUX ) + 6;
}

/*=========================================================================*/
// Generic QMUX functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ParseQMUX (Public Method)

DESCRIPTION:
   Remove QMUX headers from a buffer

PARAMETERS
   pClientID       [ O ] - On success, will point to Client ID
   pBuffer         [ I ] - Full Message passed in
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - Positive for size of QMUX header
         Negative errno for error
===========================================================================*/
int ParseQMUX(
   u16 *    pClientID,
   void *   pBuffer,
   u16      buffSize )
{
   sQMUX * pQMUXHeader;

   if (pBuffer == 0 || buffSize < 12)
   {
      return -ENOMEM;
   }

   // QMUX Header
   pQMUXHeader = (sQMUX *)pBuffer;

   if (pQMUXHeader->mTF != 1
   ||  le16_to_cpu(get_unaligned(&pQMUXHeader->mLength)) != buffSize - 1
   ||  pQMUXHeader->mCtrlFlag != 0x80 )
   {
      return -EINVAL;
   }

   // Client ID
   *pClientID = (pQMUXHeader->mQMIClientID << 8) + pQMUXHeader->mQMIService;

   return sizeof( sQMUX );
}

/*===========================================================================
METHOD:
   FillQMUX (Public Method)

DESCRIPTION:
   Fill buffer with QMUX headers

PARAMETERS
   clientID        [ I ] - Client ID
   pBuffer         [ O ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer (must be at least 6)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int FillQMUX(
   u16      clientID,
   void *   pBuffer,
   u16      buffSize )
{
   sQMUX * pQMUXHeader;

   if (pBuffer == 0 ||  buffSize < sizeof( sQMUX ))
   {
      return -ENOMEM;
   }

   // QMUX Header
   pQMUXHeader = (sQMUX *)pBuffer;

   pQMUXHeader->mTF = 1;
   put_unaligned(cpu_to_le16(buffSize - 1), &pQMUXHeader->mLength);
   DBG("pQMUXHeader->mLength = 0x%x, buffSize - 1 = 0x%x\n",pQMUXHeader->mLength, buffSize - 1);
   pQMUXHeader->mCtrlFlag = 0;

   // Service and Client ID
   pQMUXHeader->mQMIService = clientID & 0xff;
   pQMUXHeader->mQMIClientID = clientID >> 8;

   return 0;
}

/*=========================================================================*/
// Generic QMI functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   GetTLV (Public Method)

DESCRIPTION:
   Get data buffer of a specified TLV from a QMI message

   QMI Message shall NOT include SDU

PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer
   type           [ I ] - Desired Type
   pOutDataBuf    [ O ] - Buffer to be filled with TLV
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   u16 - Size of TLV for success
         Negative errno for error
===========================================================================*/
u16 GetTLV(
   void *   pQMIMessage,
   u16      messageLen,
   u8       type,
   void *   pOutDataBuf,
   u16      bufferLen )
{
   u16 pos;
   u16 tlvSize = 0;
   u16 cpyCount;

   if (pQMIMessage == 0 || pOutDataBuf == 0)
   {
      return -ENOMEM;
   }

   for (pos = 4;
        pos + 3 < messageLen;
        pos += tlvSize + 3)
   {
      tlvSize = le16_to_cpu( get_unaligned(((u16 *)(pQMIMessage + pos + 1) )) );
      if (*(u8 *)(pQMIMessage + pos) == type)
      {
         if (bufferLen < tlvSize)
         {
            return -ENOMEM;
         }

         for (cpyCount = 0; cpyCount < tlvSize; cpyCount++)
         {
            *((char*)(pOutDataBuf + cpyCount)) = *((char*)(pQMIMessage + pos + 3 + cpyCount));
         }

         return tlvSize;
      }
   }

   return -ENOMSG;
}

void PrintIPAddr(char *msg, unsigned int addr)
{
   DBG("%s : %d.%d.%d.%d",
        msg,
        addr >> 24,
        (addr >> 16) & 0xff,
        (addr >> 8) & 0xff,
        (addr ) & 0xff
        );
}

/*===========================================================================
METHOD:
   ValidQMIMessage (Public Method)

DESCRIPTION:
   Check mandatory TLV in a QMI message

   QMI Message shall NOT include SDU

PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   int - 0 for success (no error)
         Negative errno for error
         Positive for QMI error code
===========================================================================*/
int ValidQMIMessage(
   void *   pQMIMessage,
   u16      messageLen )
{
   char mandTLV[4];

   if (GetTLV( pQMIMessage, messageLen, 2, &mandTLV[0], 4 ) == 4)
   {
      // Found TLV
      if (*(u16 *)&mandTLV[0] != 0)
      {
         return le16_to_cpu( get_unaligned(&mandTLV[2]) );
      }
      else
      {
         return 0;
      }
   }
   else
   {
      return -ENOMSG;
   }
}

/*===========================================================================
METHOD:
   GetQMIMessageID (Public Method)

DESCRIPTION:
   Get the message ID of a QMI message

   QMI Message shall NOT include SDU

PARAMETERS
   pQMIMessage    [ I ] - QMI Message buffer
   messageLen     [ I ] - Size of QMI Message buffer

RETURN VALUE:
   int - Positive for message ID
         Negative errno for error
===========================================================================*/
int GetQMIMessageID(
   void *   pQMIMessage,
   u16      messageLen )
{
   if (messageLen < 2)
   {
      return -ENODATA;
   }
   else
   {
      return le16_to_cpu( get_unaligned((u16 *)pQMIMessage) );
   }
}

/*=========================================================================*/
// Fill Buffers with QMI requests
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMICTLGetClientIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Get Client ID Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   serviceType     [ I ] - Service type requested

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLGetClientIDReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u8       serviceType )
{
   if (pBuffer == 0 || buffSize < QMICTLGetClientIDReqSize() )
   {
      return -ENOMEM;
   }

   // QMI CTL GET CLIENT ID
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))= 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   put_unaligned(cpu_to_le16(0x0022), (u16 *)(pBuffer + sizeof( sQMUX ) + 2));
   // Size of TLV's
   put_unaligned(cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + 4));
   // QMI Service Type
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 6)  = 0x01;
   // Size
   put_unaligned(cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + 7));
   // QMI svc type
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 9)  = serviceType;

  // success
  return sizeof( sQMUX ) + 10;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Release Client ID Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   clientID        [ I ] - Service type requested

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLReleaseClientIDReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u16      clientID )
{
   if (pBuffer == 0 || buffSize < QMICTLReleaseClientIDReqSize() )
   {
      return -ENOMEM;
   }

   DBG(  "buffSize: 0x%x, transactionID: 0x%x, clientID: 0x%x,\n",
         buffSize, transactionID, clientID );

   // QMI CTL RELEASE CLIENT ID REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1 ) = transactionID;
   // Message ID
   put_unaligned( cpu_to_le16(0x0023), (u16 *)(pBuffer + sizeof( sQMUX ) + 2) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0005), (u16 *)(pBuffer + sizeof( sQMUX ) + 4) );
   // Release client ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 6)  = 0x01;
   // Size
   put_unaligned( cpu_to_le16(0x0002), (u16 *)(pBuffer + sizeof( sQMUX ) + 7));
   // QMI svs type / Client ID
   put_unaligned(cpu_to_le16(clientID), (u16 *)(pBuffer + sizeof( sQMUX ) + 9));

   // success
   return sizeof( sQMUX ) + 11;
}

/*===========================================================================
METHOD:
   QMICTLReadyReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Get Version Info Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLReadyReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID )
{
   if (pBuffer == 0 || buffSize < QMICTLReadyReqSize() )
   {
      return -ENOMEM;
   }

   DBG("buffSize: 0x%x, transactionID: 0x%x\n", buffSize, transactionID);

   // QMI CTL GET VERSION INFO REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   put_unaligned( cpu_to_le16(0x0021), (u16 *)(pBuffer + sizeof( sQMUX ) + 2) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 4) );

  // success
  return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMIWDSSetEventReportReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Set Event Report Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSSetEventReportReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIWDSSetEventReportReqSize() )
   {
      return -ENOMEM;
   }

   // QMI WDS SET EVENT REPORT REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   put_unaligned( cpu_to_le16(transactionID), (u16 *)(pBuffer + sizeof( sQMUX ) + 1));
   // Message ID
   put_unaligned( cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + 3));
   // Size of TLV's
   put_unaligned(cpu_to_le16(0x0008), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));
   // Report channel rate TLV
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 7)  = 0x11;
   // Size
   put_unaligned( cpu_to_le16(0x0005), (u16 *)(pBuffer + sizeof( sQMUX ) + 8));
   // Stats period
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 10)  = 0x01;
   // Stats mask
   put_unaligned( cpu_to_le32(0x000000ff), (u32 *)(pBuffer + sizeof( sQMUX ) + 11) );

  // success
  return sizeof( sQMUX ) + 15;
}

/*===========================================================================
METHOD:
   QMIWDSGetPKGSRVCStatusReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDS Get PKG SRVC Status Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDSGetPKGSRVCStatusReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIWDSGetPKGSRVCStatusReqSize() )
   {
      return -ENOMEM;
   }

   // QMI WDS Get PKG SRVC Status REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   put_unaligned(cpu_to_le16(transactionID), (u16 *)(pBuffer + sizeof( sQMUX ) + 1));
   // Message ID
   put_unaligned(cpu_to_le16(0x0022), (u16 *)(pBuffer + sizeof( sQMUX ) + 3));
   // Size of TLV's
   put_unaligned(cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));

   // success
   return sizeof( sQMUX ) + 7;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEIDReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI DMS Get Serial Numbers Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIDMSGetMEIDReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIDMSGetMEIDReqSize() )
   {
      return -ENOMEM;
   }

   // QMI DMS GET SERIAL NUMBERS REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   put_unaligned( cpu_to_le16(transactionID), (u16 *)(pBuffer + sizeof( sQMUX ) + 1) );
   // Message ID
   put_unaligned( cpu_to_le16(0x0025), (u16 *)(pBuffer + sizeof( sQMUX ) + 3) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));

   // success
   return sizeof( sQMUX ) + 7;
}


/*===========================================================================
METHOD:
   QMIDMSSWISetFCCAuthReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI DMS Get FCC Authentication Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIDMSSWISetFCCAuthReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIDMSSWISetFCCAuthReqSize() )
   {
      return -ENOMEM;
   }

   // QMI DMS SET FCC AUTH REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   put_unaligned( cpu_to_le16(transactionID), (u16 *)(pBuffer + sizeof( sQMUX ) + 1) );
   // Message ID
   put_unaligned( cpu_to_le16(0x555F), (u16 *)(pBuffer + sizeof( sQMUX ) + 3) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));

   // success
   return QMIDMSSWISetFCCAuthReqSize();
}

/*===========================================================================
METHOD:
   QMIWDASetDataFormatReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI WDA Set Data Format Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID
   te_flow_control [ I ] - TE Flow Control Flag

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDASetDataFormatReq(
   void *   pBuffer,
   u16      buffSize,
   u16      transactionID,
   int     te_flow_control,
   int      iDataMode,
   unsigned mIntfNum,
   int      iqmuxenable)
{
   u32 uTotalTlvLength = 0;
   int iIndex = 25;

   if (pBuffer == 0 || buffSize < QMIWDASetDataFormatReqSize(te_flow_control,iqmuxenable) )
   {
      return -ENOMEM;
   }

   // QMI WDA SET DATA FORMAT REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;

   // Transaction ID
   put_unaligned( cpu_to_le16(transactionID), (u16 *)(pBuffer + sizeof( sQMUX ) + 1) );

   // Message ID
   put_unaligned( cpu_to_le16(0x0020), (u16 *)(pBuffer + sizeof( sQMUX ) + 3) );
   if(iqmuxenable!=0)
   {
      uTotalTlvLength = QMIWDASETDATAFORMATQMAPREQSIZE;
   }
   // Size of TLV's
   if(te_flow_control!=eSKIP_TE_FLOW_CONTROL_TLV)
   {
      /* TE_FLOW_CONTROL */
      put_unaligned( cpu_to_le16(0x0016 + uTotalTlvLength), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));
      uTotalTlvLength += 0x0016;
   }
   else
   {
      put_unaligned( cpu_to_le16(0x0012 + uTotalTlvLength), (u16 *)(pBuffer + sizeof( sQMUX ) + 5));
      uTotalTlvLength += 0x0012;
   }

   // Tlv 0x10 QOS Data Format
   /* TLVType QOS Data Format 1 byte  */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  7) = 0x10; // type data format

   /* TLVLength  2 bytes - see spec */
   put_unaligned( cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + 8));

   /* DataFormat: 0-default; 1-QoS hdr present 2 bytes */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 10) = 0; /* no-QOS header */
   uTotalTlvLength -= (1+2+1);
   //End Tlv 0x10

   //Tlv 0x11 Link protocol used by the client
   /* TLVType Link-Layer Protocol  (Optional) 1 byte */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 11) = 0x11;

   /* TLVLength 2 bytes */
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + 12));

   /* LinkProt: 0x1 - ETH; 0x2 - rawIP  4 bytes */
   if(iDataMode==eDataMode_RAWIP)
   {
      /* Set RawIP mode */
      put_unaligned( cpu_to_le32(0x00000002), (u32 *)(pBuffer + sizeof( sQMUX ) + 14));
      DBG("Request RawIP Data Format\n");
   }
   else
   {
      /* Set Ethernet  mode */
      put_unaligned( cpu_to_le32(0x00000001), (u32 *)(pBuffer + sizeof( sQMUX ) + 14));
      DBG("Request Ethernet Data Format\n");
   }
   uTotalTlvLength -= (1+2+4);
   //End Tlv 0x11

   //Tlv 0x13 Downlink Data Aggregation Protocol
   /* TLVType Uplink Data Aggression Protocol - 1 byte */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 18) = 0x13;

   /* TLVLength 2 bytes */
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + 19));

   /* TLV Data */
   if(iqmuxenable!=0)
   {
      //DL QMAP is enabled
      put_unaligned( cpu_to_le32(0x00000005), (u32 *)(pBuffer + sizeof( sQMUX ) + 21));
   }
   else
   {
      //DL data aggregation is disabled
      put_unaligned( cpu_to_le32(0x00000000), (u32 *)(pBuffer + sizeof( sQMUX ) + 21));
   }
   uTotalTlvLength -= (1+2+4);
   //End 0x13

   //Tlv 0x1A TE Flow Control
   if(te_flow_control!=eSKIP_TE_FLOW_CONTROL_TLV)
   {
      /* TLVType Flow Control - 1 byte */
      *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex) = 0x1A;
      iIndex++;
      /* TLVLength 2 bytes */
      put_unaligned( cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
      iIndex+=2;
      /* Flow Control: 0 - not done by TE; 1 - done by TE  1 byte */
      if(te_flow_control==eTE_FLOW_CONTROL_TLV_0)
      {
         *(u8 *)(pBuffer + sizeof( sQMUX ) + 28) = 0; /* flow control done by TE */
      }
      else
      {
         *(u8 *)(pBuffer + sizeof( sQMUX ) + 28) = 1; /* flow control done by TE */
      }
      iIndex+=1;
      uTotalTlvLength -= (1+2+1);
   } /* TE_FLOW_CONTROL */
   //End Tlv 0x1A
   // success
   if(iqmuxenable==0)
   {
      if(uTotalTlvLength!=0)
      {
         printk(KERN_WARNING "uTotalTlvLength:0x%x",uTotalTlvLength);
      }
      return QMIWDASetDataFormatReqSize(te_flow_control,iqmuxenable);
   }
   //Tlv 0x12 Uplink Data Aggregation Protocol
   *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex)  = 0x12;
   iIndex++;
   // Size
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=2;
   // UL QMAP is enabled
   put_unaligned( cpu_to_le32(0x00000005), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=4;
   uTotalTlvLength -= (1+2+4);
   //End Tlv 0x12


   //Tlv 0x15 Downlink Data Aggregation Max Datagrams
   *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex)  = 0x15;
   iIndex++;
   // Size
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=2;
   // Datagram is set as 32768
   put_unaligned( cpu_to_le32(0x00000020), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=4;
   uTotalTlvLength -= (1+2+4);
   //End Tlv 0x15


   //Tlv 0x16 Downlink Data Aggregation Max Size
   // DL Data aggregation Max datagrams
   *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex)  = 0x16;
   iIndex++;
   // Size
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=2;
   // Datagram is set as 32768
   put_unaligned( cpu_to_le32(QMAP_SIZE_OF_RX_BUFFER), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   DBG("Datagramsize:%d\n",QMAP_SIZE_OF_RX_BUFFER);

   iIndex+=4;
   uTotalTlvLength -= (1+2+4);
   //End Tlv 0x16
   //Tlv 0x17 Peripheral End Point ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex)  = 0x17;
   iIndex++;
   // Size
   put_unaligned( cpu_to_le16(0x0008), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=2;
   // Datagram is set as 32768
   put_unaligned( cpu_to_le32(0x00000002), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=4;
   put_unaligned( cpu_to_le32(mIntfNum), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=4;
   uTotalTlvLength -= (1+2+8);
   //End Tlv 0x17

   //Tlv 0x19 QMAP Downlink Minimum Padding
   *(u8 *)(pBuffer + sizeof( sQMUX ) + iIndex)  = 0x19;
   iIndex++;
   // Size
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=2;
   // Set padding length to zero
   put_unaligned(cpu_to_le32(0x00000000), (u32 *)(pBuffer + sizeof( sQMUX ) + iIndex));
   iIndex+=4;
   uTotalTlvLength -= (1+2+4);
   //End Tlv 0x19
   if(uTotalTlvLength!=0)
   {
      printk(KERN_WARNING "uTotalTlvLength:0x%x",uTotalTlvLength);
   }
   return iIndex;
}



/*===========================================================================
METHOD:
   QMICTLSetDataFormatReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Set Data Format Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLSetDataFormatReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID ,
   int      iDataMode)
{
   if (pBuffer == 0 || buffSize < QMICTLSetDataFormatReqSize() )
   {
      return -ENOMEM;
   }

   /* QMI CTL Set Data Format Request */
   /* Request */
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00; // QMICTL_FLAG_REQUEST

   /* Transaction ID 1 byte */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID; /* 1 byte as in spec */

   /* QMICTLType  2 bytes */
   put_unaligned( cpu_to_le16(0x0026), (u16 *)(pBuffer + sizeof( sQMUX ) + 2));

   /* Length  2 bytes  of 2 TLVs  each - see spec */
   put_unaligned( cpu_to_le16(0x0009), (u16 *)(pBuffer + sizeof( sQMUX ) + 4));

   /* TLVType Data Format (Mandatory)  1 byte  */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  6) = 0x01; // type data format

   /* TLVLength  2 bytes - see spec */
   put_unaligned( cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + 7));

   /* DataFormat: 0-default; 1-QoS hdr present 2 bytes */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 9) = 0; /* no-QOS header */

    /* TLVType Link-Layer Protocol  (Optional) 1 byte */
    *(u8 *)(pBuffer + sizeof( sQMUX ) + 10) = TLV_TYPE_LINK_PROTO;

    /* TLVLength 2 bytes */
    put_unaligned( cpu_to_le16(0x0002), (u16 *)(pBuffer + sizeof( sQMUX ) + 11));

   /* LinkProt: 0x1 - ETH; 0x2 - rawIP  2 bytes */
   if(iDataMode==eDataMode_RAWIP)
   {
      /* Set RawIP mode */
      put_unaligned( cpu_to_le16(0x0002), (u16 *)(pBuffer + sizeof( sQMUX ) + 13));
      DBG("Request RawIP Data Format\n");
   }
   else
   {
      /* Set Ethernet  mode */
      put_unaligned( cpu_to_le16(0x0001), (u16 *)(pBuffer + sizeof( sQMUX ) + 13));
      DBG("Request Ethernet Data Format\n");
   }

   /* success */
   return sizeof( sQMUX ) + 15;

}

/*===========================================================================
METHOD:
   QMICTLSyncReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Sync Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLSyncReq(
   void *pBuffer,
   u16  buffSize,
   u16  transactionID )
{
   if (pBuffer == 0 || buffSize < QMICTLSyncReqSize() )
   {
      return -ENOMEM;
   }

   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   put_unaligned( cpu_to_le16(0x0027), (u16 *)(pBuffer + sizeof( sQMUX ) + 2) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 4) );

  // success
  return sizeof( sQMUX ) + 6;
}

/*===========================================================================
METHOD:
   QMICTLGetVersionInfoReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Version Info Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLGetVersionInfoReq(
   void *pBuffer,
   u16  buffSize,
   u16  transactionID )
{
   if (pBuffer == 0 || buffSize < QMICTLGetVersionInfoReqSize() )
   {
      return -ENOMEM;
   }

   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00;
   // Transaction ID
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID;
   // Message ID
   put_unaligned( cpu_to_le16(0x0021), (u16 *)(pBuffer + sizeof( sQMUX ) + 2) );
   // Size of TLV's
   put_unaligned( cpu_to_le16(0x0000), (u16 *)(pBuffer + sizeof( sQMUX ) + 4) );

  // success
  return sizeof( sQMUX ) + 6;
}

/*=========================================================================*/
// Parse data from QMI responses
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMICTLGetClientIDResp (Public Method)

DESCRIPTION:
   Parse the QMI CTL Get Client ID Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pClientID       [ 0 ] - Recieved client ID

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLGetClientIDResp(
   void * pBuffer,
   u16    buffSize,
   u16 *  pClientID )
{
   int result;

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset )
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x22)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   result = GetTLV( pBuffer, buffSize, 0x01, pClientID, 2 );
   if (result != 2)
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMICTLReleaseClientIDResp (Public Method)

DESCRIPTION:
   Verify the QMI CTL Release Client ID Resp is valid

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLReleaseClientIDResp(
   void *   pBuffer,
   u16      buffSize )
{
   int result;

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x23)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIWDSEventResp (Public Method)

DESCRIPTION:
   Parse the QMI WDS Set Event Report Resp/Indication or
      QMI WDS Get PKG SRVC Status Resp/Indication

   Return parameters will only be updated if value was received

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pTXOk           [ O ] - Number of transmitted packets without errors
   pRXOk           [ O ] - Number of recieved packets without errors
   pTXErr          [ O ] - Number of transmitted packets with framing errors
   pRXErr          [ O ] - Number of recieved packets with framing errors
   pTXOfl          [ O ] - Number of transmitted packets dropped due to overflow
   pRXOfl          [ O ] - Number of recieved packets dropped due to overflow
   pTXBytesOk      [ O ] - Number of transmitted bytes without errors
   pRXBytesOk      [ O ] - Number of recieved bytes without errors
   pbLinkState     [ 0 ] - Is the link active?
   pbReconfigure   [ 0 ] - Must interface be reconfigured? (reset IP address)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIWDSEventResp(
   void *   pBuffer,
   u16      buffSize,
   u32 *    pTXOk,
   u32 *    pRXOk,
   u32 *    pTXErr,
   u32 *    pRXErr,
   u32 *    pTXOfl,
   u32 *    pRXOfl,
   u64 *    pTXBytesOk,
   u64 *    pRXBytesOk,
   bool *   pbLinkState,
   bool *   pbReconfigure )
{
   int result;
   u8 pktStatusRead[2] = {0};

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0
   || buffSize < offset
   || pTXOk == 0
   || pRXOk == 0
   || pTXErr == 0
   || pRXErr == 0
   || pTXOfl == 0
   || pRXOfl == 0
   || pTXBytesOk == 0
   || pRXBytesOk == 0
   || pbLinkState == 0
   || pbReconfigure == 0 )
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   // Note: Indications.  No Mandatory TLV required

   result = GetQMIMessageID( pBuffer, buffSize );
   // QMI WDS Set Event Report Resp
   if (result == 0x01)
   {
      // TLV's are not mandatory
      GetTLV( pBuffer, buffSize, 0x10, (void*)pTXOk, 4 );
      put_unaligned( le32_to_cpu(*pTXOk), pTXOk);
      GetTLV( pBuffer, buffSize, 0x11, (void*)pRXOk, 4 );
      put_unaligned( le32_to_cpu(*pRXOk), pRXOk);
      GetTLV( pBuffer, buffSize, 0x12, (void*)pTXErr, 4 );
      put_unaligned( le32_to_cpu(*pTXErr), pTXErr);
      GetTLV( pBuffer, buffSize, 0x13, (void*)pRXErr, 4 );
      put_unaligned( le32_to_cpu(*pRXErr), pRXErr);
      GetTLV( pBuffer, buffSize, 0x14, (void*)pTXOfl, 4 );
      put_unaligned( le32_to_cpu(*pTXOfl), pTXOfl);
      GetTLV( pBuffer, buffSize, 0x15, (void*)pRXOfl, 4 );
      put_unaligned( le32_to_cpu(*pRXOfl), pRXOfl);
      GetTLV( pBuffer, buffSize, 0x19, (void*)pTXBytesOk, 8 );
      put_unaligned( le64_to_cpu(*pTXBytesOk), pTXBytesOk);
      GetTLV( pBuffer, buffSize, 0x1A, (void*)pRXBytesOk, 8 );
      put_unaligned( le64_to_cpu(*pRXBytesOk), pRXBytesOk);
   }
   // QMI WDS Get PKG SRVC Status Resp
   else if (result == 0x22)
   {
      result = GetTLV( pBuffer, buffSize, 0x01, &pktStatusRead[0], 2 );
      // 1 or 2 bytes may be received
      if (result >= 1)
      {
         if (pktStatusRead[0] == 0x02)
         {
            *pbLinkState = true;
         }
         else
         {
            *pbLinkState = false;
         }
      }
      if (result == 2)
      {
         if (pktStatusRead[1] == 0x01)
         {
            *pbReconfigure = true;
         }
         else
         {
            *pbReconfigure = false;
         }
      }

      if (result < 0)
      {
         return result;
      }
   }
   else
   {
      return -EFAULT;
   }

   return 0;
}

int QMIQOSEventResp(
   sGobiUSBNet *    pDev,
   void *   pBuffer,
   u16      buffSize)
{
   int result;

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0
   || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result == QOS_NET_SUPPORT)
   {
      u16 tlv_rtn;
      u8 supported = (u8)-1;
      tlv_rtn = GetTLV( pBuffer, buffSize, 0x01, (void*)&supported, 1 );
      QDBG(" %d\n", tlv_rtn);

      if (supported != (u8)-1)
      {
          QDBG("supported %d", supported);
      }
   }
   else if (result == QOS_STATUS)
   {
      int j;
      sQosFlow flow;
      u16 tlv_rtn;

      tlv_rtn = GetTLV( pBuffer, buffSize, 0x01, (void*)&flow, 6 );
      put_unaligned( le32_to_cpu(flow.id), &flow.id);
      QDBG("tlv_rtn %d\n", tlv_rtn);
      QDBG("flow.id 0x%x\n", flow.id);
      QDBG("flow.status 0x%x\n", flow.status);
      QDBG("flow.event 0x%x\n", flow.event);

      for(j=0;j<MAX_MAP;j++)
      {
          //TODO this only update flow status when mapped
          //share we always update even when user has not assign a map for this qos id
          if (pDev->maps.table[j].qosId== flow.id)
          {
              pDev->maps.table[j].state = flow.status;
          }
      }

   }
   else
   {
      QDBG("unhandled indication 0x%x\n", result);
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEIDResp (Public Method)

DESCRIPTION:
   Parse the QMI DMS Get Serial Numbers Resp

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   pMEID           [ O ] - Device MEID
   meidSize        [ I ] - Size of MEID buffer (at least 14)

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIDMSGetMEIDResp(
   void *   pBuffer,
   u16      buffSize,
   char *   pMEID,
   int      meidSize )
{
   int result;

   // Ignore QMUX and SDU
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0 || buffSize < offset || meidSize < 14)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x25)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      return -EFAULT;
   }

   result = GetTLV( pBuffer, buffSize, 0x12, (void*)pMEID, 14 );
   if (result != 14)
   {
      return -EFAULT;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIWDASetDataFormatResp (Public Method)

DESCRIPTION:
   Parse the QMI WDA Set Data Format Response

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   iDataMode       [ I ] - Data Mode
   ULDatagram      [ I ] - Downlink Data Aggregation Max Datagrams
   ULDatagramSize  [ I ] - Downlink Data Aggregation Max Size
RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMIWDASetDataFormatResp(
   void *   pBuffer,
   u16      buffSize,
   int      iDataMode,
   u32 *    ULDatagram,
   u32 *    ULDatagramSize)
{

   int result;
   u32 u32DlminPadding = (u32)-1;
   u8 pktLinkProtocol[4];

   // Ignore QMUX and SDU
   // QMI SDU is 3 bytes
   u8 offset = sizeof( sQMUX ) + 3;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x20)
   {
      return -EFAULT;
   }

   /* Check response message result TLV */
   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      DBG("EFAULT: Data Format Mode Bad Response\n");
//      return -EFAULT;
      return 0;
   }

   if(ULDatagram!=NULL)
   {
      result = GetTLV( pBuffer, buffSize, 0x15, (void*)ULDatagram, 4 );
      if (result != 4)
      {
         printk(KERN_WARNING"FAIL: ULDatagram\n");
      }
      else
      {
         put_unaligned( le32_to_cpu(*ULDatagram), ULDatagram);
         if(*ULDatagram>QMAP_SIZE_OF_RX_BUFFER)
         {
            printk(KERN_WARNING"WARN: ULDatagram:%u\n",*ULDatagram);
         }
      }
   }
   if(ULDatagramSize!=NULL)
   {
      result = GetTLV( pBuffer, buffSize, 0x16, (void*)ULDatagramSize, 4 );
      if (result != 4)
      {
         printk(KERN_WARNING"FAIL: ULDatagramSize\n");
      }
      else
      {
         put_unaligned( le32_to_cpu(*ULDatagramSize), ULDatagramSize);
         if(*ULDatagramSize>QMAP_SIZE_OF_RX_BUFFER)
         {
            printk(KERN_WARNING"WARN: ULDatagramSize:%u\n",*ULDatagramSize);
         }
      }
   }
   /* Check response message link protocol */
   result = GetTLV( pBuffer, buffSize, 0x11,
                     &pktLinkProtocol[0], 4);
   if (result != 4)
   {
      DBG("EFAULT: Wrong TLV format\n");
      return 0;

   }

   result = GetTLV( pBuffer, buffSize, 0x1A,
                        &u32DlminPadding, 4);
   if (result == 4)
   {
      DBG("u32DlminPadding :%u\n",u32DlminPadding);
   }

   if(iDataMode==eDataMode_RAWIP)
   {
      if (pktLinkProtocol[0] != 2)
      {
         DBG("EFAULT: Data Format Cannot be set to RawIP Mode\n");
         return -EFAULT;
      }
      DBG("Data Format Set to RawIP\n");
   }
   else
   {
      if (pktLinkProtocol[0] != 1)
      {
         DBG("EFAULT: Data Format Cannot be set to Ethernet Mode\n");
         return -EFAULT;
      }
      DBG("Data Format Set to Ethernet Mode \n");
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMICTLSetDataFormatResp (Public Method)

DESCRIPTION:
   Parse the QMI CTL Set Data Format Response

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer
   iDataMode       [ I ] - Data Mode
RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLSetDataFormatResp(
   void *   pBuffer,
   u16      buffSize,
   int      iDataMode)
{

   int result;

   u8 pktLinkProtocol[2];

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x26)
   {
      return -EFAULT;
   }

   /* Check response message result TLV */
   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      DBG("EFAULT: Data Format Mode Bad Response\n");
      return -EFAULT;
   }

   /* Check response message link protocol */
   result = GetTLV( pBuffer, buffSize, TLV_TYPE_LINK_PROTO,
                     &pktLinkProtocol[0], 2);
   if (result != 2)
   {
      DBG("EFAULT: Wrong TLV format\n");
      return -EFAULT;
   }

   if(iDataMode==eDataMode_RAWIP)
   {
      if (pktLinkProtocol[0] != 2)
      {
         DBG("EFAULT: Data Format Cannot be set to RawIP Mode\n");
         return -EFAULT;
      }
      DBG("Data Format Set to RawIP\n");
   }
   else
   {
      if (pktLinkProtocol[0] != 1)
      {
         DBG("EFAULT: Data Format Cannot be set to Ethernet Mode\n");
         return -EFAULT;
      }
      DBG("Data Format Set to Ethernet Mode \n");
   }

   return 0;
}


/*===========================================================================
METHOD:
   QMICTLSyncResp (Public Method)

DESCRIPTION:
   Validate the QMI CTL Sync Response

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLSyncResp(
   void *pBuffer,
   u16  buffSize )
{
   int result;

   // Ignore QMUX (2 bytes for QMI CTL) and SDU
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x27)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );

   return result;
}

/*===========================================================================
METHOD:
   QMICTLGetVersionInfoResp (Public Method)

DESCRIPTION:
   Validate the QMI CTL Version Info Response

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLGetVersionInfoResp(
   void *pBuffer,
   u16   buffSize,
   u8 *  pSvcVersion,
   int   versionInfoSize )
{
   int result;

   // Ignore QMUX (2 bytes for QMI CTL) and SDU
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x21)
   {
      return -EFAULT;
   }

   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      DBG("EFAULT: Get Version Info Bad Response\n");
      return -EFAULT;
   }

   result = GetTLV( pBuffer, buffSize, 0x01, (void*)pSvcVersion, versionInfoSize );
   if (result < 0)
   {
      return result;
   }
   return 0;
}


/*===========================================================================
METHOD:
   QMICTLSetPowerSaveModeReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLSetPowerSaveModeReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLSetPowerSaveModeReqSize( void )
{
   return sizeof( sQMUX ) + 13;
}

/*===========================================================================
METHOD:
   QMICTLSetPowerSaveModeReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Set Power Save Mode Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLSetPowerSaveModeReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u8       mode)
{
   if (pBuffer == 0 || buffSize < QMICTLSetPowerSaveModeReqSize() )
   {
      return -ENOMEM;
   }

   /* QMI CTL Set Power Save Mode Request */
   /* Request */
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00; // QMICTL_FLAG_REQUEST

   /* Transaction ID 1 byte */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID; /* 1 byte as in spec */

   /* Message ID  2 bytes */
   put_unaligned( cpu_to_le16(0x002A), (u16 *)(pBuffer + sizeof( sQMUX ) + 2));

   /* Length  2 bytes  of 1 TLV = 7 bytes */
   put_unaligned( cpu_to_le16(0x0007), (u16 *)(pBuffer + sizeof( sQMUX ) + 4));

   /* TLVType Power save state 1 byte  */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  6) = 0x01;

   /* TLVLength  2 bytes - see spec */
   put_unaligned( cpu_to_le16(0x0004), (u16 *)(pBuffer + sizeof( sQMUX ) + 7));

   /* pwrsave_state  4 byptes */
   //*(u8 *)(pBuffer + sizeof( sQMUX ) + 9) = mode;

   /* pwrsave_state  4 byptes */
   put_unaligned( cpu_to_le32(mode), (u32 *)(pBuffer + sizeof( sQMUX ) + 9) );

   /* success */
   return sizeof( sQMUX ) + 13;

}

/*===========================================================================
METHOD:
   QMICTLSetPowerSaveModeResp (Public Method)

DESCRIPTION:
   Parse the QMI CTL Set Power Save Mode Response

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLSetPowerSaveModeResp(
   void *   pBuffer,
   u16      buffSize )
{
   int result;

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x2A)
   {
      return -EFAULT;
   }

   /* Check response message result TLV */
   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      DBG("EFAULT: Set Power Save Mode Bad Response\n");
      return -EFAULT;
   }
   return 0;
}

/*===========================================================================
METHOD:
   QMICTLConfigPowerSaveSettingsReqSize (Public Method)

DESCRIPTION:
   Get size of buffer needed for QMUX + QMICTLConfigPowerSaveSettingsReq

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMICTLConfigPowerSaveSettingsReqSize( void )
{
   return sizeof( sQMUX ) + 19;
}


/*===========================================================================
METHOD:
   QMICTLConfigPowerSaveSettingsReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI CTL Config Power Save Settings Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMICTLConfigPowerSaveSettingsReq(
   void *   pBuffer,
   u16      buffSize,
   u8       transactionID,
   u8       service,
   u8       indication)
{
   if (pBuffer == 0 || buffSize < QMICTLConfigPowerSaveSettingsReqSize() )
   {
      return -ENOMEM;
   }

   /* QMI CTL Set Power Save Mode Request */
   /* Request */
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0x00; // QMICTL_FLAG_REQUEST

   /* Transaction ID 1 byte */
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 1) = transactionID; /* 1 byte as in spec */

   /* Message ID  2 bytes */
   put_unaligned( cpu_to_le16(0x0029), (u16 *)(pBuffer + sizeof( sQMUX ) + 2));

   /* Length  2 bytes  of 2 TLVs = 13 bytes */
   put_unaligned( cpu_to_le16(0x000D), (u16 *)(pBuffer + sizeof( sQMUX ) + 4));

   /* TLVType Power save state 1 byte  */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  6) = 0x01;

   /* TLVLength  2 bytes - see spec */
   put_unaligned( cpu_to_le16(0x0005), (u16 *)(pBuffer + sizeof( sQMUX ) + 7));

   /* pwrsave_state  4 byptes */
   put_unaligned( cpu_to_le32(0x00000001), (u32 *)(pBuffer + sizeof( sQMUX ) + 9) );

   /* qmi_service 1 byptes */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  13) = service;

   /* TLVType Permitted Indication set 1 byte  */
   *(u8 *)(pBuffer + sizeof( sQMUX ) +  14) = 0x11;

   /* TLVLength  2 bytes*/
   put_unaligned( cpu_to_le16(0x0002), (u16 *)(pBuffer + sizeof( sQMUX ) + 15));

   /* indication_set 2 bytes*/
   put_unaligned( cpu_to_le16(indication), (u16 *)(pBuffer + sizeof( sQMUX ) + 17));

   /* success */
   return sizeof( sQMUX ) + 19;

}

/*===========================================================================
METHOD:
   QMICTLConfigPowerSaveSettingsResp (Public Method)

DESCRIPTION:
   Parse the QMI CTL Config Power Save Settings Request

PARAMETERS
   pBuffer         [ I ] - Buffer to be parsed
   buffSize        [ I ] - Size of pBuffer

RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int QMICTLConfigPowerSaveSettingsResp(
   void *   pBuffer,
   u16      buffSize )
{
   int result;

   // Ignore QMUX and SDU
   //    QMI CTL SDU is 2 bytes, not 3
   u8 offset = sizeof( sQMUX ) + 2;

   if (pBuffer == 0 || buffSize < offset)
   {
      return -ENOMEM;
   }

   pBuffer = pBuffer + offset;
   buffSize -= offset;

   result = GetQMIMessageID( pBuffer, buffSize );
   if (result != 0x29)
   {
      return -EFAULT;
   }

   /* Check response message result TLV */
   result = ValidQMIMessage( pBuffer, buffSize );
   if (result != 0)
   {
      DBG("EFAULT: Config Power Save Settings Request\n");
      return -EFAULT;
   }
   return 0;
}

/*===========================================================================
DESCRIPTION:
   Get size of buffer needed for QMUX + QMIWDASetDataFormatReqSettings

RETURN VALUE:
   u16 - size of buffer
===========================================================================*/
u16 QMIWDASetDataFormatReqSettingsSize( void )
{
   return sizeof( sQMUX ) + 11;
}

/*===========================================================================
METHOD:
   QMIWDASetDataFormatReqSettingsReq (Public Method)

DESCRIPTION:
   Fill buffer with QMI Set QMAP Settings Request

PARAMETERS
   pBuffer         [ 0 ] - Buffer to be filled
   buffSize        [ I ] - Size of pBuffer
   transactionID   [ I ] - Transaction ID

RETURN VALUE:
   int - Positive for resulting size of pBuffer
         Negative errno for error
===========================================================================*/
int QMIWDASetDataFormatReqSettingsReq(
      void *   pBuffer,
      u16      buffSize,
      u16      transactionID )
{
   if (pBuffer == 0 || buffSize < QMIWDASetDataFormatReqSettingsSize() )
   {
      return -ENOMEM;
   }

   // QMI WDA SET DATA FORMAT REQ
   // Request
   *(u8 *)(pBuffer + sizeof( sQMUX ))  = 0;
   // Transaction ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 1) = cpu_to_le16(transactionID);
   // Message ID
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 3) = cpu_to_le16(0x2B);
   // Size of TLV's
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 5) = cpu_to_le16(4);
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 7)  = 0x10;
   *(u16 *)(pBuffer + sizeof( sQMUX ) + 8) = cpu_to_le16(1);
   *(u8 *)(pBuffer + sizeof( sQMUX ) + 10)  = 1;

   // success
   return sizeof( sQMUX ) + 11;
}


void PrintIPV6Addr(ipv6_addr * addr)
{
   if(debug & DEBUG_NETMASK)
   {
      char str[40];
      snprintf(str,40,"%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
      (int)addr->ipv6addr[0], (int)addr->ipv6addr[1],
      (int)addr->ipv6addr[2], (int)addr->ipv6addr[3],
      (int)addr->ipv6addr[4], (int)addr->ipv6addr[5],
      (int)addr->ipv6addr[6], (int)addr->ipv6addr[7],
      (int)addr->ipv6addr[8], (int)addr->ipv6addr[9],
      (int)addr->ipv6addr[10], (int)addr->ipv6addr[11],
      (int)addr->ipv6addr[12], (int)addr->ipv6addr[13],
      (int)addr->ipv6addr[14], (int)addr->ipv6addr[15]);
      NETDBG("IPv6 Addr:%s\n",str);
      return ;
   }
}
