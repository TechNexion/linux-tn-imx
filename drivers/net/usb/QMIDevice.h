/*===========================================================================
FILE:
   QMIDevice.h

DESCRIPTION:
   Functions related to the QMI interface device

FUNCTIONS:
   Generic functions
      IsDeviceValid
      PrintHex
      GobiSetDownReason
      GobiClearDownReason
      GobiTestDownReason

   Driver level asynchronous read functions
      ResubmitIntURB
      ReadCallback
      IntCallback
      StartRead
      KillRead

   Internal read/write functions
      ReadAsync
      UpSem
      ReadSync
      WriteSyncCallback
      WriteSync

   Internal memory management functions
      GetClientID
      ReleaseClientID
      FindClientMem
      AddToReadMemList
      PopFromReadMemList
      AddToNotifyList
      NotifyAndPopNotifyList
      AddToURBList
      PopFromURBList

   Internal userspace wrapper functions
      UserspaceunlockedIOCTL

   Userspace wrappers
      UserspaceOpen
      UserspaceIOCTL
      UserspaceClose
      UserspaceRead
      UserspaceWrite
      UserspacePoll

   Initializer and destructor
      RegisterQMIDevice
      DeregisterQMIDevice

   Driver level client management
      QMIReady
      QMIWDSCallback
      SetupQMIWDSCallback
      QMIDMSGetMEID
      QMIDMSSWISetFCCAuth
      QMICTLGetVersionInfo

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
// Pragmas
//---------------------------------------------------------------------------
#pragma once

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#include "Structs.h"
#include "QMI.h"
#ifdef CONFIG_ANDROID
#include <linux/suspend.h>
#endif

#define MAX_QCQMI 255
#define MAX_QCQMI_PER_INTF 2
#define SEMI_INIT_DEFAULT_VALUE 0
#define QMI_CONTROL_MSG_DELAY_MS 100
#define QMI_CONTROL_MAX_MSG_DELAY_MS QMI_CONTROL_MSG_DELAY_MS * 5
// QMI CTL
#define QMI_CTL_IND 0x02
#define QMI_CTL_SYNC_IND 0x0027

extern int qcqmi_table[MAX_QCQMI];
extern int qmux_table[MAX_QCQMI];
extern sGobiPrivateWorkQueues GobiPrivateWorkQueues[MAX_QCQMI][MAX_QCQMI_PER_INTF];
//Register State
enum {
   eClearCID=0,
   eClearAndReleaseCID=1,
   eForceClearAndReleaseCID=2,
};

//Work queue type
enum {
   eWQ_PROBE=0,
   eWQ_URBCB=1,
};

/*=========================================================================*/
// Generic functions
/*=========================================================================*/

// Basic test to see if device memory is valid

bool IsDeviceDisconnect(sGobiUSBNet *pDev);

#ifdef CONFIG_PM
bool bIsSuspend(sGobiUSBNet *pGobiDev);
#endif

void UsbAutopmGetInterface(struct usb_interface * intf);
void UsbAutopmPutInterface(struct usb_interface * intf);

// Print Hex data, for debug purposes
void PrintHex(
   void *         pBuffer,
   u16            bufSize );

// Print Hex data, for QMAP debug purposes
void NetHex(
   void *      pBuffer,
   u16         bufSize );
// Print Hex data, for QMAP purposes
void ErrHex(
   void *         pBuffer,
   u16            bufSize );

// Sets mDownReason and turns carrier off
void GobiSetDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

// Clear mDownReason and may turn carrier on
void GobiClearDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

// Tests mDownReason and returns whether reason is set
bool GobiTestDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

/*=========================================================================*/
// Driver level asynchronous read functions
/*=========================================================================*/

// Resubmit interrupt URB, re-using same values
int ResubmitIntURB(sGobiUSBNet * pDev, struct urb * pIntURB );

// Read callback
//    Put the data in storage and notify anyone waiting for data
void ReadCallback( struct urb * pReadURB );

// Inturrupt callback
//    Data is available, start a read URB
void IntCallback( struct urb * pIntURB );


// ReadCallback Intrrupt routine
void ReadCallbackInt( struct urb * pReadURB );

// Start continuous read "thread"
int StartRead( sGobiUSBNet * pDev );

// Kill continuous read "thread"
void KillRead( sGobiUSBNet * pDev );

/*=========================================================================*/
// Internal read/write functions
/*=========================================================================*/

// Start asynchronous read
//     Reading client's data store, not device
int ReadAsync(
   sGobiUSBNet *    pDev,
   u16                clientID,
   u16                transactionID,
   void               (*pCallback)(sGobiUSBNet *, u16, void *),
   void *             pData ,
   int                iSpinLock);

// Notification function for synchronous read
void UpSem(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData );

// Start synchronous read
//     Reading client's data store, not device
int ReadSync(
   sGobiUSBNet *    pDev,
   void **            ppOutBuffer,
   u16                clientID,
   u16                transactionID,
   int                *iID,
   struct semaphore   *pReadSem,
   int                *iIsClosing);

// Write callback
void WriteSyncCallback( struct urb * pWriteURB );

// Start synchronous write
int WriteSync(
   sGobiUSBNet *    pDev,
   char *             pInWriteBuffer,
   int                size,
   u16                clientID );

// Start synchronous write without resume device
int WriteSyncNoResume(
   sGobiUSBNet *    pDev,
   char *             pInWriteBuffer,
   int                size,
   u16                clientID );

// Start synchronous write no retry
int WriteSyncNoRetry(
   sGobiUSBNet *    pDev,
   char *             pInWriteBuffer,
   int                size,
   u16                clientID );

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

// Create client and allocate memory
int GetClientID(
   sGobiUSBNet *      pDev,
   u8                 serviceType,
   struct semaphore   *pReadSem);

// Release client and free memory
bool ReleaseClientID(
   sGobiUSBNet *      pDev,
   u16                clientID);

// Find this client's memory
sClientMemList * FindClientMem(
   sGobiUSBNet *      pDev,
   u16                  clientID );

// Add Data to this client's ReadMem list
bool AddToReadMemList(
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void *               pData,
   u16                  dataSize );

// Remove data from this client's ReadMem list if it matches
// the specified transaction ID.
bool PopFromReadMemList(
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void **              ppData,
   u16 *                pDataSize );

// Add Notify entry to this client's notify List
bool AddToNotifyList(
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void                 (* pNotifyFunct)(sGobiUSBNet *, u16, void *),
   void *               pData );

int RemoveAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID ,
   int              iClearCID);

// Remove first Notify entry from this client's notify list
//    and Run function
int NotifyAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID );

// Add URB to this client's URB list
bool AddToURBList(
   sGobiUSBNet *      pDev,
   u16                  clientID,
   struct urb *         pURB );

// Remove URB from this client's URB list
struct urb * PopFromURBList(
   sGobiUSBNet *      pDev,
   u16                  clientID );

/*=========================================================================*/
// Internal userspace wrappers
/*=========================================================================*/

// Userspace unlocked ioctl
long UserspaceunlockedIOCTL(
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg );

/*=========================================================================*/
// Userspace wrappers
/*=========================================================================*/

// Userspace open
int UserspaceOpen(
   struct inode *   pInode,
   struct file *    pFilp );

// Userspace ioctl
int UserspaceIOCTL(
   struct inode *    pUnusedInode,
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg );

// Userspace close
int UserspaceClose(
   struct file *       pFilp,
   fl_owner_t          unusedFileTable );

// Userspace read (synchronous)
ssize_t UserspaceRead(
   struct file *        pFilp,
   char __user *        pBuf,
   size_t               size,
   loff_t *             pUnusedFpos );

// Userspace write (synchronous)
ssize_t UserspaceWrite(
   struct file *        pFilp,
   const char __user *  pBuf,
   size_t               size,
   loff_t *             pUnusedFpos );

unsigned int UserspacePoll(
   struct file *                  pFilp,
   struct poll_table_struct *     pPollTable );

/*=========================================================================*/
// Initializer and destructor
/*=========================================================================*/

// QMI Device initialization function
int RegisterQMIDevice( sGobiUSBNet * pDev, int is9x15 );

// QMI Device cleanup function
void DeregisterQMIDevice( sGobiUSBNet * pDev );

/*=========================================================================*/
// Driver level client management
/*=========================================================================*/

// Check if QMI is ready for use
int QMIReady(
   sGobiUSBNet *    pDev,
   u16                timeout );

// QMI WDS callback function
void QMIWDSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData );

// Fire off reqests and start async read for QMI WDS callback
int SetupQMIWDSCallback( sGobiUSBNet * pDev );

int SetupQMIQOSCallback( sGobiUSBNet * pDev );

// Register client, send req and parse MEID response, release client
int QMIDMSGetMEID( sGobiUSBNet * pDev );

// Register client, send req and parse FCC Authentication response, release client
int QMIDMSSWISetFCCAuth( sGobiUSBNet * pDev );

// Register client, send req and parse Data format response, release client
int QMIWDASetDataFormat( sGobiUSBNet * pDev, int te_flow_control , int iqmuxenable);

// Send set QMAP Data format request and parse response
int QMIWDASetQMAP( sGobiUSBNet * pDev , u16 WDAClientID);

// send req and parse Data format response
int QMICTLSetDataFormat( sGobiUSBNet * pDev );

// send req and parse Data format response
int QMICTLGetVersionInfo( sGobiUSBNet * pDev );

// Initialize Read Sync tasks semaphore
void InitSemID(sGobiUSBNet * pDev);


/***************************************************************************/
// wait_ms
/**************************************************************************/
void wait_ms(unsigned int ms) ;

// Userspace Release (synchronous)
int UserspaceRelease(struct inode *inode, struct file *file);

// Userspace Lock (synchronous)
int UserSpaceLock(struct file *filp, int cmd, struct file_lock *fl);

// sync memory
void gobi_flush_work(void);

// Close Opened File Inode
int CloseFileInode(sGobiUSBNet * pDev,int iCount);
// Set modem in specific power save mode
int SetPowerSaveMode(sGobiUSBNet *pDev,u8 mode);

// config modem qmi wakeup filter
int ConfigPowerSaveSettings(sGobiUSBNet *pDev, u8 service, u8 indication);

// Get TID
u8 QMIXactionIDGet( sGobiUSBNet *pDev);

// Release Specific Client ID Nofitication From Memory List
int ReleaseNotifyList(sGobiUSBNet *pDev,u16 clientID,u8 transactionID);

int gobi_kthread_should_stop(void);

//
int Gobi_usb_control_msg(struct usb_interface *intf, struct usb_device *dev, unsigned int pipe, __u8 request,
                     __u8 requesttype, __u16 value, __u16 index, void *data,
                      __u16 size, int timeout);

int AddClientToMemoryList(sGobiUSBNet *pDev,u16 clientID);
//Wait control message semaphore to be up with timeout
void wait_control_msg_semaphore_timeout(struct semaphore *pSem, unsigned int timeout);

static inline int IsInterfacefDisconnected(struct usb_interface *intf)
{
   if(!interface_to_usbdev(intf))
      return 1;
   if (interface_to_usbdev(intf)->state == USB_STATE_NOTATTACHED )
   {
      return 1;
   }
   return 0;
}

static inline int gobi_usb_autopm_get_interface(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return -ENXIO;
   }
   return usb_autopm_get_interface(intf);
}
void gobi_usb_autopm_put_interface(struct usb_interface *intf);
void gobi_usb_autopm_get_interface_no_resume(struct usb_interface *intf);
void gobi_usb_autopm_put_interface_no_resume(struct usb_interface *intf);
int gobi_usb_autopm_get_interface_async(struct usb_interface *intf);
void gobi_usb_autopm_put_interface_async(struct usb_interface *intf);
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
void gobi_usb_autopm_enable(struct usb_interface *intf);
#endif

//unregister qmap netdev
void gobi_qmimux_unregister_device(struct net_device *dev);
//register qmap netdev
struct net_device * gobi_qmimux_register_device(struct net_device *real_dev,int iNumber, u8 mux_id);
//Check SKB is a QMUX packet
int iIsValidQmuxSKB(struct sk_buff *skb);
//Get MUX ID from skb
int iGetQmuxIDFromSKB(struct sk_buff *skb);
//Get Number of QMAP framed Packet In SKB
int iNumberOfQmuxPacket(struct sk_buff *skb,int iDisplay);
//Check SKB packet is QMAP framed.
int iIsQmuxPacketComplete(struct sk_buff *skb);
//Check NULL QMAP framed packet
int iIsQmuxZeroPacket(struct sk_buff *skb);
//Print QMAP Framed packet.
int PrintQmuxPacket(struct sk_buff *skb);
//Get QMAP framed packet length.
u32 u32GetSKBQMAPPacketLength(struct sk_buff *skb,int iOffset);
//Check QMAP header in SKB.
int iIsValidQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset);
//Check NULL QMAP header in SKB.
int iIsZeroQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset);
//Check Command QMAP header in SKB.
int iIsCMDQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset);
//Remove QMAP header and Padding Bytes
int iRemoveQMAPPaddingBytes(struct sk_buff *skb);

// Init WorkQueues
int GobiInitWorkQueue(sGobiUSBNet *pGobiDev);
// Destory WorkQueues
void GobiDestoryWorkQueue(sGobiUSBNet *pGobiDev);
// Clean up work queues in sGobiPrivateWorkQueues
int iClearWorkQueuesByTableIndex(int index);

#ifdef CONFIG_ANDROID
#define  DELAY_MS_DEFAULT  round_jiffies_relative(30*HZ)
//
void SetTxRxStat(sGobiUSBNet *pGobiDev,int state);
//
int GetTxRxStat(sGobiUSBNet *pGobiDev,int Channel);
//
void gobiLockSystemSleep(sGobiUSBNet *pGobiDev);
//
void gobiUnLockSystemSleep(sGobiUSBNet *pGobiDev);
//
void gobiStayAwake(sGobiUSBNet *pGobiDev);
//
void gobiPmRelax(sGobiUSBNet *pGobiDev);
//
int GenerateProcessName(const char *pPrefix,char *szProcessName,unsigned sizeofName,sGobiUSBNet *pGobiDev );
#endif
