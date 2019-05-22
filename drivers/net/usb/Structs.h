/*===========================================================================
FILE:
   Structs.h

DESCRIPTION:
   Declaration of structures used by the Qualcomm Linux USB Network driver

FUNCTIONS:
   none

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
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#ifdef CONFIG_PM
#include <linux/pm_runtime.h>
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,24 ))
   #include "usbnet.h"
#else
   #include <linux/usb/usbnet.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION( 2,6,25 ))
   #include <linux/fdtable.h>
#else
   #include <linux/file.h>
#endif

#include <linux/semaphore.h>

#define MAX_MAP (9)
#define MAX_DSCP_ID        0x3F
#define UNIQUE_DSCP_ID     0x40

#define MAX_READ_SYNC_TASK_ID 255
#define MAX_RETRY_LOCK_NUMBER 10
#define MAX_RETRY_LOCK_MSLEEP_TIME 10
#define MAX_RETRY_TASK_LOCK_TIME 10
#define MAX_RETRY_TASK_MSLEEP_TIME 5
#define MAX_DEVICE_MEID_SIZE 14
#define MAX_SVC_VERSION_SIZE 512

//Max number of QMUX supported
#define MAX_MUX_NUMBER_SUPPORTED 8
//First supported QMUX ID
#define MUX_ID_START 0x80
//Last supported QMUX ID
#define MUX_ID_END   MUX_ID_START + MAX_MUX_NUMBER_SUPPORTED
#define RMNET_QMAP_STRING "rmnet-qmap-"

// Used in recursion, defined later below
struct sGobiUSBNet;

#define MAX_WQ_PROC_NAME_SIZE 512

#ifdef CONFIG_ANDROID
#define MAX_WS_NAME_SIZE 64
#endif

/*=========================================================================*/
// Struct sGobiPrivateWorkQueues
//
//    Structure that defines an entry work queues per deivce
/*=========================================================================*/
typedef struct sGobiPrivateWorkQueues{
   char szProcessName[MAX_WQ_PROC_NAME_SIZE];
   struct workqueue_struct *wqprobe;
   struct workqueue_struct *wqProcessReadCallback;
}sGobiPrivateWorkQueues;

/*=========================================================================*/
// Struct sReadMemList
//
//    Structure that defines an entry in a Read Memory linked list
/*=========================================================================*/
typedef struct sReadMemList
{
   /* Data buffer */
   void *                     mpData;

   /* Transaction ID */
   u16                        mTransactionID;

   /* Size of data buffer */
   u16                        mDataSize;

   /* Next entry in linked list */
   struct sReadMemList *      mpNext;

} sReadMemList;

/*=========================================================================*/
// Struct sNotifyList
//
//    Structure that defines an entry in a Notification linked list
/*=========================================================================*/
typedef struct sNotifyList
{
   /* Function to be run when data becomes available */
   void                  (* mpNotifyFunct)(struct sGobiUSBNet *, u16, void *);

   /* Transaction ID */
   u16                   mTransactionID;

   /* Data to provide as parameter to mpNotifyFunct */
   void *                mpData;

   /* Next entry in linked list */
   struct sNotifyList *  mpNext;

} sNotifyList;

/*=========================================================================*/
// Struct sURBList
//
//    Structure that defines an entry in a URB linked list
/*=========================================================================*/
typedef struct sURBList
{
   /* The current URB */
   struct urb *       mpURB;

   /* Next entry in linked list */
   struct sURBList *  mpNext;

} sURBList;

/*=========================================================================*/
// Struct sClientMemList
//
//    Structure that defines an entry in a Client Memory linked list
//      Stores data specific to a Service Type and Client ID
/*=========================================================================*/
typedef struct sClientMemList
{
   /* Client ID for this Client */
   u16                          mClientID;

   /* Linked list of Read entries */
   /*    Stores data read from device before sending to client */
   sReadMemList *               mpList;

   /* Linked list of Notification entries */
   /*    Stores notification functions to be run as data becomes
         available or the device is removed */
   sNotifyList *                mpReadNotifyList;

   /* Linked list of URB entries */
   /*    Stores pointers to outstanding URBs which need canceled
         when the client is deregistered or the device is removed */
   sURBList *                   mpURBList;

   /* Next entry in linked list */
   struct sClientMemList *      mpNext;

   /* Wait queue object for poll() */
   wait_queue_head_t    mWaitQueue;

} sClientMemList;

/*=========================================================================*/
// Struct sURBSetupPacket
//
//    Structure that defines a USB Setup packet for Control URBs
//    Taken from USB CDC specifications
/*=========================================================================*/
typedef struct sURBSetupPacket
{
   /* Request type */
   u8    mRequestType;

   /* Request code */
   u8    mRequestCode;

   /* Value */
   u16   mValue;

   /* Index */
   u16   mIndex;

   /* Length of Control URB */
   u16   mLength;

} sURBSetupPacket;

// Common value for sURBSetupPacket.mLength
#define DEFAULT_READ_URB_LENGTH 0x1000


/*=========================================================================*/
// Struct sAutoPM
//
//    Structure used to manage AutoPM thread which determines whether the
//    device is in use or may enter autosuspend.  Also submits net
//    transmissions asynchronously.
/*=========================================================================*/
typedef struct sAutoPM
{
   /* Thread for atomic autopm function */
   struct task_struct *       mpThread;

   /* Signal for completion when it's time for the thread to work */
   struct completion          mThreadDoWork;

   /* Time to exit? */
   bool                       mbExit;

   /* List of URB's queued to be sent to the device */
   sURBList *                 mpURBList;

   /* URB list lock (for adding and removing elements) */
   spinlock_t                 mURBListLock;

   /* Length of the URB list */
   atomic_t                   mURBListLen;

   /* Active URB */
   struct urb *               mpActiveURB;

   /* Active URB lock (for adding and removing elements) */
   spinlock_t                 mActiveURBLock;

   /* Duplicate pointer to USB device interface */
   struct usb_interface *     mpIntf;

} sAutoPM;


/*=========================================================================*/
// Struct sQMIDev
//
//    Structure that defines the data for the QMI device
/*=========================================================================*/
typedef struct sQMIDev
{
   /* Device number */
   dev_t                      mDevNum;

   /* Device class */
   struct class *             mpDevClass;

   /* cdev struct */
   struct cdev                mCdev;

   /* is mCdev initialized? */
   bool                       mbCdevIsInitialized;

   /* Pointer to read URB */
   struct urb *               mpReadURB;

   /* Read setup packet */
   sURBSetupPacket *          mpReadSetupPacket;

   /* Read buffer attached to current read URB */
   void *                     mpReadBuffer;

   /* Inturrupt URB */
   /*    Used to asynchronously notify when read data is available */
   struct urb *               mpIntURB;

   /* Buffer used by Inturrupt URB */
   void *                     mpIntBuffer;

   /* Pointer to memory linked list for all clients */
   sClientMemList *           mpClientMemList;

   /* Spinlock for client Memory entries */
   spinlock_t                 mClientMemLock;
   unsigned long              mFlag;
   struct task_struct         *pTask;
    /* semaphore for Notify */
   struct semaphore           mNotifyMemLock;

   /* Transaction ID associated with QMICTL "client" */
   atomic_t                   mQMICTLTransactionID;

   unsigned char              qcqmi;

   int                        iInterfaceNumber;
   struct proc_dir_entry *    proc_file;
} sQMIDev;

enum qos_flow_state {
    FLOW_ACTIVATED = 0x01,
    FLOW_SUSPENDED = 0x02,
    FLOW_DELETED = 0x03,
    FLOW_MODIFIED,
    FLOW_ENABLED,
    FLOW_DISABLED,
    FLOW_INVALID = 0xff
};

typedef struct {
    u8  dscp;
    u32 qosId;
    u8  state;
} sMapping;

typedef struct {
    u8 count;
    sMapping table[MAX_MAP];
} sMappingTable;

typedef struct {
  u32 rx_packets;
  u32 tx_packets;
  u64 rx_bytes;
  u64 tx_bytes;
  u32 rx_errors;
  u32 tx_errors;
  u32 rx_overflows;
  u32 tx_overflows;
} sNetStats;

#define IPV6_ADDR_LEN 16
typedef struct ipv6_addr
{
   u8         ipv6addr[IPV6_ADDR_LEN];
   u8         prefix;
}__attribute__((__packed__)) ipv6_addr;


typedef struct {
    u8 instance;
    unsigned int ipAddress;
    ipv6_addr    ipV6Address;
} sQMuxIPTable;

typedef struct gobi_qmimux_hdr{
   u8 pad;
   u8 mux_id;
   __be16 pkt_len;
}gobi_qmimux_hdr;

typedef struct qmap_ipv4_header
{
#ifdef LITTLE_ENDIAN
   unsigned char ihl:4;
   unsigned char version:4;
   unsigned char ecn:2;
   unsigned char dscp:6;
#else
   unsigned char version:4;
   unsigned char ihl:4;
   unsigned char dscp:6;
   unsigned char ecn:2;
#endif
   unsigned short total_length;
   unsigned short identification;
   unsigned short fragment_offset;
   unsigned char ttl;
   unsigned char protocol;
   unsigned short header_checksum;
   unsigned int src_address;
   unsigned int dst_address;
} __attribute__ ((aligned (1))) *qmap_ipv4_header_t;

typedef struct qmap_ipv6_header
{
   unsigned int version:4;
   unsigned int traffic_class:8;
   unsigned int flow_label:20;
   unsigned short length;
   unsigned char next_header;
   unsigned char hop_limit;
   unsigned char src_address[16];
   unsigned char dst_address[16];
} __attribute__ ((aligned (1))) *qmap_ipv6_header_t;

enum{
   eDataMode_Unknown=-1,
   eDataMode_Ethernet,
   eDataMode_RAWIP,
};

enum{
   eNetDeviceLink_Unknown=-1,
   eNetDeviceLink_Disconnected,
   eNetDeviceLink_Connected,
};

#ifdef CONFIG_ANDROID
#define RESUME_TX_RX_DISABLE 0x00
#define RESUME_RX_OKAY 0x01
#define RESUME_TX_OKAY 0x02
#endif
/*=========================================================================*/
// Struct sGobiUSBNet
//
//    Structure that defines the data associated with the Qualcomm USB device
/*=========================================================================*/
typedef struct sGobiUSBNet
{
   /* Net device structure */
   struct usbnet *        mpNetDev;

   /* Usb device interface */
   struct usb_interface * mpIntf;

   /* Pointers to usbnet_open and usbnet_stop functions */
   int                  (* mpUSBNetOpen)(struct net_device *);
   int                  (* mpUSBNetStop)(struct net_device *);

   /* Reason(s) why interface is down */
   /* Used by Gobi*DownReason */
   unsigned long          mDownReason;
#define NO_NDIS_CONNECTION    0
#define CDC_CONNECTION_SPEED  1
#define DRIVER_SUSPENDED      2
#define NET_IFACE_STOPPED     3

   /* QMI "device" status */
   bool                   mbQMIValid;
   int                   mbUnload;
   int                   mReleaseClientIDFail;
   /* QMI "device" memory */
   sQMIDev                mQMIDev;

   /* Device MEID */
   char                   mMEID[MAX_DEVICE_MEID_SIZE];

   /* Service version Info */
   u8                     svcVersion[MAX_SVC_VERSION_SIZE];

   /* AutoPM thread */
   sAutoPM                mAutoPM;

   /* Ethernet header templates */
   /* IPv4 */
   u8  eth_hdr_tmpl_ipv4[ETH_HLEN];
   /* IPv6 */
   u8  eth_hdr_tmpl_ipv6[ETH_HLEN];
   #ifdef CONFIG_ANDROID
   /* for backup USB auto suspend delay */
   ssize_t                autosuspend_delay;
   bool                   autosuspend_overrided;
   #endif
   u32 tx_qlen;

   sMappingTable maps;

   /*
    * Read write semaphore so that ReleaseClientID() waits until WriteSync() exits to handle
    * below limitation
    * If a thread in your driver uses this call, make sure your disconnect()
    * method can wait for it to complete.  Since you don't have a handle on the
    * URB used, you can't cancel the request.
    */
   struct rw_semaphore shutdown_rwsem;
   int iShutdown_read_sem;
   int iShutdown_write_sem;

   struct timer_list read_tmr;
   u16 readTimeoutCnt;
   u16 writeTimeoutCnt;

   bool bLinkState;
   u16 mtu;
   #ifdef CONFIG_PM
   bool bSuspend;
   spinlock_t sSuspendLock;
   #ifdef CONFIG_ANDROID
   int iSuspendReadWrite;
   #endif
   #endif
   bool mIs9x15;
   struct usb_interface *mUsb_Interface;
   int iTaskID;
   struct task_struct *task;

   int iReasSyncTaskID[MAX_READ_SYNC_TASK_ID];
   struct semaphore readSem[MAX_READ_SYNC_TASK_ID];
   struct semaphore ReadsyncSem;

   struct semaphore taskIDSem;
   int iIsClosing;
   struct device *qcqmidev;
   struct device *dev;
   u16 WDSClientID;
   int iNetLinkStatus;
   int iDataMode;
   spinlock_t urb_lock;
   spinlock_t notif_lock;
   int iUSBState;
   int iDeviceMuxID;
   int iQMUXEnable;
   int iStoppingNetDev;
   int nRmnet;
   int iMaxMuxID;
   int iPacketInComplete;
   struct sk_buff *pLastSKB;
   struct net_device *pNetDevice[MAX_MUX_NUMBER_SUPPORTED];
   sQMuxIPTable      qMuxIPTable[MAX_MUX_NUMBER_SUPPORTED];
   u32 ULDatagramSize;
   u32 ULDatagram;
   int iIPAlias;
   /*
    * Workqueue and Delaywork to probe device.
    */
   struct workqueue_struct *wqprobe;
   struct delayed_work dwprobe;
   /*
    * Workqueue and Delaywork to Process Interrupt URB.
    */
   struct workqueue_struct *wqProcessReadCallback;
   struct delayed_work dwProcessReadCallback;
   #ifdef CONFIG_ANDROID
   /*
    * Workqueue and Delaywork to Process Lock System Sleep.
    */
   struct workqueue_struct *wqLockSystemSleep;
   struct delayed_work dwLockSystemSleep;

   /*
    * Workqueue and Delaywork to Process UnLock System Sleep.
    */
   struct workqueue_struct *wqUnLockSystemSleep;
   struct delayed_work dwUnLockSystemSleep;
   #endif
   struct urb *pReadURB;
   #ifdef CONFIG_ANDROID
   struct wakeup_source __rcu *ws;
   #endif
   int iIsUSBReset;
   /**
    * Workaround spin_is_locked not function as expected.
    **/
#define CLIENT_MEMORY_LOCK 1
#define CLIENT_MEMORY_UNLOCK 0
   atomic_t aClientMemIsLock;
} sGobiUSBNet;

/*=========================================================================*/
// Struct sQMIFilpStorage
//
//    Structure that defines the storage each file handle contains
//       Relates the file handle to a client
/*=========================================================================*/
typedef struct sQMIFilpStorage
{
   /* Client ID */
   u16                  mClientID;
   int                  mDeviceInvalid;
   /* Device pointer */
   sGobiUSBNet *          mpDev;
   int iSemID ;
   struct semaphore       mReadSem;
   int iReleaseSemID ;
   struct semaphore       mReleasedSem;
   int                    iIsClosing;
   int                    iReadSyncResult;
   int                    iInfNum;
   struct task_struct     *pOpenTask;
   struct task_struct     *pReadTask;
   struct task_struct     *pWriteTask;
   struct task_struct     *pIOCTLTask;
   int                    iCount;
} sQMIFilpStorage;

struct gobi_qmimux_priv {
   struct net_device *real_dev;
   u8 mux_id;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION( 3,0,0 )
#ifndef kstrtol
#define kstrtol strict_strtol
#endif
#endif
