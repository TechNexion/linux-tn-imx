/*===========================================================================
FILE:
   QMIDevice.c

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
      WriteSync

   Internal memory management functions
      GetClientID
      ReleaseClientID
      FindClientMem
      AddToReadMemList
      PopFromReadMemList
      AddToNotifyList
      NotifyAndPopNotifyList

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
#include "QMIDevice.h"
#include "Structs.h"
#include <linux/module.h>
#include <linux/proc_fs.h> // for the proc filesystem
#include <linux/device.h>
#include <linux/file.h>
#include <linux/rtnetlink.h>
#include <linux/netdevice.h>
#include <net/sch_generic.h>
#include <linux/if_arp.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 4,11,0 ))
#include <linux/sched/signal.h>
#endif

#if (LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,35 ))
#warning "Fix compilation error 'include/linux/compat.h:233: error: in /usr/src/linux-headers-2.6.35-22-generic/include/linux/compat.h, line 233, the variable name of second parameter,  replace *u32 to *u"
#endif
#include <linux/usbdevice_fs.h>
#ifdef CONFIG_PROVE_RCU
#define _SIG_LOCK_ 1
#else
#define _SIG_LOCK_ 0
#endif
#include <linux/sched.h>
#include <linux/dnotify.h>
//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

extern int debug;
extern int is9x15;
extern int interruptible;
extern int iTEEnable;
extern void *gobi_skb_push(struct sk_buff *pSKB, unsigned int len);
const bool clientmemdebug = 0;
enum {
 eNotifyListEmpty=-1,
 eNotifyListNotFound=0,
 eNotifyListFound=1,
};

#define SEND_ENCAPSULATED_COMMAND (0)
#define GET_ENCAPSULATED_RESPONSE (1)
#define USB_WRITE_TIMEOUT 500   // must be less than AM timeout
#define USB_WRITE_RETRY (2)
#define USB_READ_TIMEOUT (500)
#define MAX_RETRY 5
#define ENABLE_MAX_RETRY_LOCK_MSLEEP_TIME 10
#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
#define raw_spin_is_locked(x) (&(x)->raw_lock == 0)
#endif
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 4,15,0 ))
#define gobi_setup_timer(timer, fn, data) setup_timer(timer, fn, data)
#else
#define gobi_setup_timer(timer, fn, data) timer_setup(timer, fn, 0)
#endif

#define RETURN_WHEN_DEVICE_ERR(x) \
if(isModuleUnload(x))\
{\
   return -EFAULT;\
}\
else if(IsDeviceDisconnect(x))\
{\
  DBG( "Device Disconnected!\n" );\
  return -ENXIO;\
}\

/* initially all zero */
int qcqmi_table[MAX_QCQMI];
int qmux_table[MAX_QCQMI];

//Store all work queues per device per interface
sGobiPrivateWorkQueues GobiPrivateWorkQueues[MAX_QCQMI][MAX_QCQMI_PER_INTF];

extern bool isModuleUnload(sGobiUSBNet *pDev);
extern int iRAWIPEnable;
extern int iQMUXEnable;
static inline bool IsDeviceValid( sGobiUSBNet * pDev );
static void gobiProcessReadURB(sGobiUSBNet *pGobiDev);
static void ProcessReadWorkFunction(struct work_struct *w);
int IsOtherTaskUsingFilp(struct file *pFilp);
int IsOpenTaskIsCurrent(struct file *pFilp);
int IsCurrentTaskExit(void);
int ClientTransactionIDExist(sGobiUSBNet * pDev, u16 clientID,u16 u16TransactionID);

int GenerateProcessName(const char *pPrefix,char *szProcessName,unsigned sizeofName,sGobiUSBNet *pGobiDev );
int GetPrivateWorkQueuesInterfaceTableIndex(sGobiUSBNet *pGobiDev);
int AddPrivateWorkQueues(sGobiUSBNet *pGobiDev);
int GetPrivateWorkQueuesIndex(sGobiUSBNet *pGobiDev);
int SetPrivateWorkQueuesWQByTableIndex(int i,int j,struct workqueue_struct *wq,int type);
struct workqueue_struct *GetPrivateWorkQueuesWQByTableIndex(int i,int j,int type);
int ClearPrivateWorkQueuesProcessByTableIndex(int i,int j);
void GobiCancelReadCallBackWorkQueue(sGobiUSBNet *pGobiDev);
void GobiCancelProbeWorkQueue(sGobiUSBNet *pGobiDev);
#ifdef CONFIG_ANDROID
void GobiCancelLockSystemSleepWorkQueue(sGobiUSBNet *pGobiDev);
void GobiCancelUnLockSystemSleepWorkQueue(sGobiUSBNet *pGobiDev);
#endif

bool TransceiveReleaseClientID(
   sGobiUSBNet *    pDev,
   u16                clientID);
void GobiCancelDelayWorkWorkQueue(
   sGobiUSBNet *pGobiDev,
   struct workqueue_struct *wq,
   struct delayed_work *dw);
#ifdef CONFIG_ANDROID
void GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice(
   sGobiUSBNet *pGobiDev,
   struct workqueue_struct *wq,
   struct delayed_work *dw);
#endif

#define CLIENT_READMEM_SNAPSHOT(clientID, pdev)\
   if( (debug & DEBUG_QMI) && clientmemdebug )\
   {\
      sClientMemList *pclnt;\
      sReadMemList *plist;\
      pclnt = FindClientMem((pdev), (clientID));\
      plist = pclnt->mpList;\
      if( (pdev) != NULL){\
          while(plist != NULL)\
          {\
             DBG(  "clientID 0x%x, mDataSize = %u, mpData = 0x%p, mTransactionID = %u,  \
                    mpNext = 0x%p\n", (clientID), plist->mDataSize, plist->mpData, \
                    plist->mTransactionID, plist->mpNext  ) \
             /* advance to next entry */\
             plist = plist->mpNext;\
          }\
      }\
   }

#ifdef CONFIG_PM
// Prototype to GobiNetSuspend function
int GobiNetSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent );
#endif /* CONFIG_PM */

int wakeup_inode_process(struct file *pFilp,struct task_struct * pTask);
void gobi_try_wake_up_process(struct task_struct * pTask);
int gobi_work_busy(struct delayed_work *dw);
extern int GobiUSBLockReset( struct usb_interface *pIntf );
extern int iIsRemoteWakeupSupport(struct usbnet *pDev);
extern bool iIsSpinIsLockedSupported;

// IOCTL to generate a client ID for this service type
#define IOCTL_QMI_GET_SERVICE_FILE 0x8BE0 + 1

// IOCTL to get the VIDPID of the device
#define IOCTL_QMI_GET_DEVICE_VIDPID 0x8BE0 + 2

// IOCTL to get the MEID of the device
#define IOCTL_QMI_GET_DEVICE_MEID 0x8BE0 + 3

#define IOCTL_QMI_RELEASE_SERVICE_FILE_IOCTL  (0x8BE0 + 4)

#define IOCTL_QMI_ADD_MAPPING 0x8BE0 + 5
#define IOCTL_QMI_DEL_MAPPING 0x8BE0 + 6
#define IOCTL_QMI_CLR_MAPPING 0x8BE0 + 7

#define IOCTL_QMI_QOS_SIMULATE 0x8BE0 + 8
#define IOCTL_QMI_GET_TX_Q_LEN 0x8BE0 + 9

#define IOCTL_QMI_EDIT_MAPPING 0x8BE0 + 10
#define IOCTL_QMI_READ_MAPPING 0x8BE0 + 11
#define IOCTL_QMI_DUMP_MAPPING 0x8BE0 + 12
#define IOCTL_QMI_GET_USBNET_STATS 0x8BE0 + 13
#define IOCTL_QMI_SET_DEVICE_MTU 0x8BE0 + 14
#define IOCTL_QMI_GET_QMAP_SUPPORT 0x8BE0 + 15
#define IOCTL_QMI_SET_IP_ADDRESS   0x8BE0 + 16
#define IOCTL_QMI_SET_IPV6_ADDRESS   0x8BE0 + 17
#define IOCTL_QMI_GET_IPALIAS_MODE   0x8BE0 + 18
#define IOCTL_QMI_GET_SVC_VERSION_INFO   0x8BE0 + 19

// CDC GET_ENCAPSULATED_RESPONSE packet
#define CDC_GET_ENCAPSULATED_RESPONSE_LE 0x01A1ll
#define CDC_GET_ENCAPSULATED_RESPONSE_BE 0xA101000000000000ll
/* The following masks filter the common part of the encapsulated response
 * packet value for Gobi and QMI devices, ie. ignore usb interface number
 */
#define CDC_RSP_MASK_BE 0xFFFFFFFF00FFFFFFll
#define CDC_RSP_MASK_LE 0xFFFFFFE0FFFFFFFFll

#define _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_ 1

const int i = 1;
#define is_bigendian() ( (*(char*)&i) == 0 )
#define CDC_GET_ENCAPSULATED_RESPONSE(pcdcrsp, pmask)\
{\
   *pcdcrsp  = is_bigendian() ? CDC_GET_ENCAPSULATED_RESPONSE_BE \
                          : CDC_GET_ENCAPSULATED_RESPONSE_LE ; \
   *pmask = is_bigendian() ? CDC_RSP_MASK_BE \
                           : CDC_RSP_MASK_LE; \
}

// CDC CONNECTION_SPEED_CHANGE indication packet
#define CDC_CONNECTION_SPEED_CHANGE_LE 0x2AA1ll
#define CDC_CONNECTION_SPEED_CHANGE_BE 0xA12A000000000000ll
/* The following masks filter the common part of the connection speed change
 * packet value for Gobi and QMI devices
 */
#define CDC_CONNSPD_MASK_BE 0xFFFFFFFFFFFF7FFFll
#define CDC_CONNSPD_MASK_LE 0XFFF7FFFFFFFFFFFFll
#define CDC_GET_CONNECTION_SPEED_CHANGE(pcdccscp, pmask)\
{\
   *pcdccscp  = is_bigendian() ? CDC_CONNECTION_SPEED_CHANGE_BE \
                          : CDC_CONNECTION_SPEED_CHANGE_LE ; \
   *pmask = is_bigendian() ? CDC_CONNSPD_MASK_BE \
                           : CDC_CONNSPD_MASK_LE; \
}

#define SPIN_LOCK_DEBUG 0

/*=========================================================================*/
// QMAP netdev define
/*=========================================================================*/
#define ARPHRD_NONE     0xFFFE
#ifndef netdev_tx_t
#define netdev_tx_t int
#endif

#if (__GNUC__ > 7 && defined(_ASM_X86_ATOMIC_H))|| \
    (__GNUC__ == 7 && defined(_ASM_X86_ATOMIC_H) && (__GNUC_MINOR__ > 0 || \
                       (__GNUC_MINOR__ == 0 && \
                        __GNUC_PATCHLEVEL__ > 0)))
#define gobi_atomic_read(x) atomic_read((const atomic_t *)x)
#else
#define gobi_atomic_read(x) atomic_read((atomic_t *)x)
#endif
#define in_serving_hardirq() (hardirq_count() &  HARDIRQ_OFFSET)
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,37 ))

#define in_serving_softirq()	(softirq_count() & SOFTIRQ_OFFSET)
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 5,0,0 ))
   #define gobi_dev_change_flags(dev,flags) dev_change_flags(dev, flags, NULL)
#else
   #define gobi_dev_change_flags(dev,flags) dev_change_flags(dev, flags)
#endif

static int gobi_qmimux_open(struct net_device *dev)
{
   struct gobi_qmimux_priv *priv = netdev_priv(dev);
   struct net_device *real_dev = priv->real_dev;
   unsigned short oflags;
   DBG("\n");
   if (!(priv->real_dev->flags & (IFF_UP|IFF_RUNNING)))
   {
      printk("Adaptor Not Up\n");
      oflags = dev->flags;
      if (gobi_dev_change_flags(real_dev, oflags | IFF_UP | IFF_RUNNING) < 0)
      {
          printk("IP-Config: Failed to open %s\n",
          dev->name);
      }
      netif_carrier_on(real_dev);
   }

   netif_carrier_on(real_dev);
   netif_start_queue(real_dev);
   netif_carrier_on(dev);
   netif_start_queue(dev);
   return 0;
}

static int gobi_qmimux_stop(struct net_device *dev)
{
   NETDBG("\n");
   netif_carrier_off(dev);
   return 0;
}

struct sk_buff *GobiNetDriverQmuxTxFixup(
   struct sk_buff *pSKB,
   gfp_t flags,
   u8 mux_id)
{

   DBG( "\n" );
   if (pSKB->len >= 4)
   {

      // Skip Ethernet header from message
      NETDBG( "Before sending to device modified: Len:0x%x",pSKB->len);
      NetHex (pSKB->data, pSKB->len);
      return pSKB;
   }
   else
   {
      NETDBG( "Packet Dropped Length");
   }

   // Filter the packet out, release it
   dev_kfree_skb_any(pSKB);
   return NULL;
}

static netdev_tx_t gobi_qmimux_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
   struct gobi_qmimux_priv *priv = netdev_priv(dev);
   unsigned int len = skb->len;
   struct gobi_qmimux_hdr *hdr;

   skb->dev = priv->real_dev;
   if(dev->type == ARPHRD_ETHER)
   {
      NETDBG("Remove ETH Header\n");
      NetHex (skb->data, skb->len);
      skb_pull(skb,ETH_HLEN);
      hdr = (struct gobi_qmimux_hdr *)gobi_skb_push(skb, sizeof(struct gobi_qmimux_hdr));
      len = skb->len - sizeof(struct gobi_qmimux_hdr);
   }
   else
   {
      hdr = (struct gobi_qmimux_hdr *)gobi_skb_push(skb, sizeof(struct gobi_qmimux_hdr));
   }
   hdr->pad = 0;
   hdr->mux_id = priv->mux_id;
   hdr->pkt_len = cpu_to_be16(len);
   skb->dev = priv->real_dev;
   NETDBG("mux_id:0x%x\n",priv->mux_id);
   if(iIsValidQmuxSKB(skb)==0)
   {
      NETDBG( "Invalid Packet\n" );
      return NETDEV_TX_BUSY;
   }
   skb = GobiNetDriverQmuxTxFixup( skb, GFP_ATOMIC,priv->mux_id);
   if (skb == NULL)
   {
      NETDBG( "unable to tx_fixup skb\n" );
      return NETDEV_TX_BUSY;
   }
   dev->stats.tx_packets++;
   dev->stats.tx_bytes += skb->len;
   #if (LINUX_VERSION_CODE < KERNEL_VERSION( 4,7,0 ))
   dev->trans_start = jiffies;
   #else
   netif_trans_update(dev);
   #endif
   return dev_queue_xmit(skb);
}

static const struct net_device_ops gobi_qmimux_netdev_ops = {
   .ndo_open       = gobi_qmimux_open,
   .ndo_stop       = gobi_qmimux_stop,
   .ndo_start_xmit = gobi_qmimux_start_xmit,
};

/*=========================================================================*/
#ifdef CONFIG_ANDROID
#define GOBI_MIN_AUTO_SUSPEND_DELAY 1000
#define GOBI_MAX_AUTO_SUSPEND_DELAY 10000
void BackupAutoSuspend_Delay(sGobiUSBNet *pDev)
{
    struct device *dev = &pDev->mpNetDev->udev->dev;
    if(pDev->autosuspend_overrided == false)
    {
        if(dev->power.runtime_auto == true)
        {
            //backup delay set by kernel (default is 2 seconds)
            pDev->autosuspend_delay = dev->power.autosuspend_delay;
            DBG("autosuspend = %zu \n", pDev->autosuspend_delay);
            //allow more auto suspend delay for QMI msg req/resp
            pm_runtime_set_autosuspend_delay(dev,GOBI_MAX_AUTO_SUSPEND_DELAY);
            pDev->autosuspend_overrided = true;
        }
    }
}
void RestoreAutoSuspend_Delay(sGobiUSBNet *pDev)
{
    struct device *dev = &pDev->mpNetDev->udev->dev;
    if(pDev->autosuspend_overrided == true)
    {
        if(dev->power.runtime_auto == true)
        {
            DBG("autosuspend delay restore from %d to %zu \n",dev->power.autosuspend_delay, pDev->autosuspend_delay);
            //small delay may cause issue
            if(pDev->autosuspend_delay<GOBI_MIN_AUTO_SUSPEND_DELAY) pDev->autosuspend_delay=GOBI_MIN_AUTO_SUSPEND_DELAY;
            //restore delay set by kernel
            pm_runtime_set_autosuspend_delay(dev,pDev->autosuspend_delay);
            pDev->autosuspend_overrided = false;
        }
    }
}
#define SET_CONTROL_LINE_STATE_REQUEST             0x22
#define CONTROL_DTR                     0x01
#define CONTROL_RTS                     0x02
#endif


/*=========================================================================*/
// UserspaceQMIFops
//    QMI device's userspace file operations
/*=========================================================================*/
static const struct file_operations UserspaceQMIFops =
{
   .owner     = THIS_MODULE,
   .read      = UserspaceRead,
   .write     = UserspaceWrite,
#ifdef CONFIG_COMPAT
   .compat_ioctl = UserspaceunlockedIOCTL,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,36 ))
   .unlocked_ioctl = UserspaceunlockedIOCTL,
#else
   .ioctl     = UserspaceIOCTL,
#endif
   .open      = UserspaceOpen,
   .flush     = UserspaceClose,
   .poll      = UserspacePoll,
   .release   = UserspaceRelease,
   .lock      = UserSpaceLock
};

void RemoveProcessFile(sGobiUSBNet * pDev);
void RemoveCdev(sGobiUSBNet * pDev);
void wakeup_target_process(struct task_struct * pTask);

inline int wait_preempt(void)
{
   int count = 0;
   while (preempt_count()>0)
   {
      msleep_interruptible(50);
      if(count>10)
      {
         return 0;
      }
   }
   return 1;
}

inline void wait_interrupt(void)
{
   int count = 0;
   while(in_interrupt()||
      in_softirq()|| //Soft IRQ
      in_irq()|| //Hard IRQ
      in_serving_softirq()||
      in_serving_hardirq())
   {

      #if defined(cpu_relax)
      cpu_relax();
      #elif defined(rep_nop)
      rep_nop();
      #else
      wait_ms(50);
      #endif
      if(count++>1000)
      {
         printk(KERN_ERR "timeout");
         break;
      }
   }
}

int isPreempt(void)
{
   return in_atomic();
}

void GobiSyncRcu(void)
{
   if(isPreempt()!=0)
   {
      printk("preempt_enabled");
   }
   mb();
}
/*=========================================================================*/
// Generic functions
/*=========================================================================*/
u8 QMIXactionIDGet( sGobiUSBNet *pDev)
{
   u8 transactionID;

   if( 0 == (transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID)) )
   {
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
   }

   return transactionID;
}

static struct usb_endpoint_descriptor *GetEndpoint(
    struct usb_interface *pintf,
    int type,
    int dir )
{
   int i;
   struct usb_host_interface *iface = pintf->cur_altsetting;
   struct usb_endpoint_descriptor *pendp;

   for( i = 0; i < iface->desc.bNumEndpoints; i++)
   {
      pendp = &iface->endpoint[i].desc;
      if( ((pendp->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == dir)
          &&
          (usb_endpoint_type(pendp) == type) )
      {
         return pendp;
      }
   }

   return NULL;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,9,0 ))
static inline struct inode *file_inode(const struct file *f)
{
   return f->f_path.dentry->d_inode;
}
#endif

int gobi_filp_close(struct file *filp, fl_owner_t id)
{
   int retval = 0;
   struct inode *inode = NULL;
   if(filp==NULL)
      return -EIO;
   inode = file_inode(filp);
   if(inode == NULL)
      return -EIO;
   if (is_bad_inode(inode))
      return -EIO;
   if (!file_count(filp))
   {
      printk(KERN_ERR "VFS: Close: file count is 0\n");
      return 0;
   }

   if (filp->f_op->flush)
      retval = filp->f_op->flush(filp, id);
   fput(filp);
   return retval;
}

int ForceFilpClose(struct file *pFilp)
{
   int iRet = -1;
   if(pFilp==NULL)
   {
      printk("NULL Inode\n");
      return 0;
   }
   if (file_count(pFilp)>0)
   {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,9,0 ))
      if(file_inode(pFilp)!=NULL)
      {
         sQMIFilpStorage * pFilpData = NULL;
         pFilpData = (sQMIFilpStorage *)pFilp->private_data;
         if(file_inode(pFilp)!=NULL)
         {
            DBG("ino:%lu\n",file_inode(pFilp)->i_ino);
         }
         else
         {
            DBG("ino:NULL\n");
            return -EIO;
         }


         if(pFilpData ==NULL)
         {
            printk( KERN_INFO "bad file data\n" );
            return -EBADF;
         }
         if(pFilpData->iIsClosing == 1)
         {
            printk( KERN_INFO "Closing\n" );
            return -EBADF;
         }
         iRet = gobi_filp_close(pFilp, NULL);
      }
      else
      {
         printk("NULL Inode\n");
      }

#else
      iRet = gobi_filp_close(pFilp, NULL);
#endif
   }
   GobiSyncRcu();
   mb();
   return iRet;
}

static inline int LocalClientMemLockSpinIsLock( sGobiUSBNet * pDev)
{
   if(pDev!=NULL)
   {
      if(iIsSpinIsLockedSupported==false)
      {
         return atomic_read(&pDev->aClientMemIsLock);
      }
      return spin_is_locked(&pDev->mQMIDev.mClientMemLock);
   }
   return 0;
}

static inline unsigned long LocalClientMemLockSpinLockIRQSave( sGobiUSBNet * pDev, int line)
{
   #if SPIN_LOCK_DEBUG
   printk("(%d)%s :%d\n",task_pid_nr(current),__FUNCTION__,line);
   #endif
   mb();

   if(pDev!=NULL)
   {
      spin_lock_irq(&pDev->mQMIDev.mClientMemLock);
      mb();
      #if SPIN_LOCK_DEBUG
      printk("(%d)%s :%d Locked\n",task_pid_nr(current),__FUNCTION__,line);
      #endif
      pDev->mQMIDev.pTask = current;
      if(LocalClientMemLockSpinIsLock(pDev) == 0)
      {
         iIsSpinIsLockedSupported = false;
         DBG("spin_is_locked is not supported\n");
      }
      if(iIsSpinIsLockedSupported==false)
      {
         atomic_set(&pDev->aClientMemIsLock,CLIENT_MEMORY_LOCK);
      }
      return 0;
   }
   mb();
   return 0;
}

static inline int LocalClientMemUnLockSpinLockIRQRestore( sGobiUSBNet * pDev, unsigned long ulFlags, int line)
{
   mb();
   if(pDev!=NULL)
   {
      if(LocalClientMemLockSpinIsLock(pDev)==0)
      {

         #if SPIN_LOCK_DEBUG
         printk(KERN_WARNING "(%d)%s :%d Not Locked\n",task_pid_nr(current),__FUNCTION__,line);
         #endif
         return 0;
      }
      #if SPIN_LOCK_DEBUG
      printk("(%d)%s %d :%d\n",task_pid_nr(current),__FUNCTION__,__LINE__,line);
      #endif
      pDev->mQMIDev.pTask = NULL;
      if(iIsSpinIsLockedSupported==false)
      {
         atomic_set(&pDev->aClientMemIsLock,CLIENT_MEMORY_UNLOCK);
      }
      spin_unlock_irq( &pDev->mQMIDev.mClientMemLock);
   }
   else
   {
      #if SPIN_LOCK_DEBUG
      printk("(%d)%s %d :%d\n",task_pid_nr(current),__FUNCTION__,__LINE__,line);
      #endif
   }
   mb();
   return 0;
}

int gobi_down_interruptible(struct semaphore *sem, sGobiUSBNet *pDev)
{
   return down_interruptible(sem);
}


/*===========================================================================
METHOD:
   IsDeviceValid (Public Method)

DESCRIPTION:
   Basic test to see if device memory is valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   bool
===========================================================================*/
static inline bool IsDeviceValid( sGobiUSBNet * pDev )
{
   if (pDev == NULL)
   {
      return false;
   }

   if (pDev->mbQMIValid == false)
   {
      return false;
   }

   return true;
}

/*===========================================================================
METHOD:
   PrintHex (Public Method)

DESCRIPTION:
   Print Hex data, for debug purposes

PARAMETERS:
   pBuffer       [ I ] - Data buffer
   bufSize       [ I ] - Size of data buffer

RETURN VALUE:
   None
===========================================================================*/
void PrintHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;

   if (!(debug & DEBUG_QMI))
   {
       return;
   }
   if(bufSize==(u16)(-1))
   {
       DBG( "No Data\n" );
   }
   pPrintBuf = kmalloc( bufSize * 3 + 1, GOBI_GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );

   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)),
                         4,
                         "%02X ",
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         kfree( pPrintBuf );
         return;
      }
   }

   DBG( "   : %s\n", pPrintBuf );

   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;
}

/*===========================================================================
METHOD:
   GobiSetDownReason (Public Method)

DESCRIPTION:
   Sets mDownReason and turns carrier off

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   None
===========================================================================*/
void GobiSetDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   set_bit( reason, &pDev->mDownReason );
   if(reason==NO_NDIS_CONNECTION)
   {
      pDev->iNetLinkStatus = eNetDeviceLink_Disconnected;
   }
   #ifdef CONFIG_ANDROID
   if (DRIVER_SUSPENDED != reason)
   {
      //Android 6.0 dhcpcd detect device down/up during host suspend/resume
      //causes dhcpcd delete/add route after resume, but it's not working and cause issue ANDROIDRIL-310
      //workaround : do not make device off during suspend
      //usbnet.c->usbnet_bh() also need to modify to incorporate this change
      netif_carrier_off( pDev->mpNetDev->net );
   }
   #else
   netif_carrier_off( pDev->mpNetDev->net );
   #endif
}

/*===========================================================================
METHOD:
   GobiClearDownReason (Public Method)

DESCRIPTION:
   Clear mDownReason and may turn carrier on

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is no longer down

RETURN VALUE:
   None
===========================================================================*/
void GobiClearDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   clear_bit( reason, &pDev->mDownReason );
   if(reason==NO_NDIS_CONNECTION)
   {
      pDev->iNetLinkStatus = eNetDeviceLink_Connected;
   }
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,11,0 ))
    netif_carrier_on( pDev->mpNetDev->net );
#else
   if (pDev->mDownReason == 0)
   {
      netif_carrier_on( pDev->mpNetDev->net );
   }
   else if ( netif_running(pDev->mpNetDev->net) &&
             (reason==NO_NDIS_CONNECTION) &&
             !test_bit(DRIVER_SUSPENDED, &pDev->mDownReason) )
   {
      netif_carrier_on( pDev->mpNetDev->net );
   }
#endif
}

/*===========================================================================
METHOD:
   GobiTestDownReason (Public Method)

DESCRIPTION:
   Test mDownReason and returns whether reason is set

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   bool
===========================================================================*/
bool GobiTestDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   return test_bit( reason, &pDev->mDownReason );
}

/*=========================================================================*/
// Driver level asynchronous read functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ResubmitIntURB (Public Method)

DESCRIPTION:
   Resubmit interrupt URB, re-using same values

PARAMETERS
   pIntURB       [ I ] - Interrupt URB

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int ResubmitIntURB(sGobiUSBNet * pDev, struct urb * pIntURB )
{
   int status;
   int interval;

   // Sanity test
   if ( (pIntURB == NULL)
   ||   (pIntURB->dev == NULL) )
   {
      return -EINVAL;
   }
   if( atomic_read(&pIntURB->reject))
   {
      DBG( "%s reject!\n" ,__FUNCTION__);
      return -EINVAL;
   }

   // Interval needs reset after every URB completion
   // QC suggestion, 4ms per poll:
   //   bInterval 6 = 2^5 = 32 frames = 4 ms per poll
   interval = (pIntURB->dev->speed == USB_SPEED_HIGH) ?
                 6 : max((int)(pIntURB->ep->desc.bInterval), 3);
   mb();

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -EINVAL;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Disconnected!\n" );
      usb_unlink_urb(pIntURB);
      return -EINVAL;
   }
   spin_lock_irq(&(pDev->urb_lock));
   // Reschedule interrupt URB
   usb_fill_int_urb( pIntURB,
                     pIntURB->dev,
                     pIntURB->pipe,
                     pIntURB->transfer_buffer,
                     pIntURB->transfer_buffer_length,
                     pIntURB->complete,
                     pIntURB->context,
                     interval );
   mb();
   status = usb_submit_urb( pIntURB, GOBI_GFP_ATOMIC );
   spin_unlock_irq(&(pDev->urb_lock));
   if (status != 0)
   {
      DBG( "Error re-submitting Int URB %d\n", status );
   }

   return status;
}

/*===========================================================================
METHOD:
   ReadCallback (Public Method)

DESCRIPTION:
   Put the data in storage and notify anyone waiting for data

PARAMETERS
   pReadURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void ReadCallback( struct urb * pReadURB )
{
   int result;
   u16 clientID;
   sClientMemList * pClientMem;
   void * pData = NULL;
   void * pDataCopy = NULL;
   u16 dataSize;
   sGobiUSBNet * pDev;
   unsigned long flags = 0;
   u16 transactionID;
   int iResult = 0;

   if (pReadURB == NULL)
   {
      DBG( "bad read URB\n" );
      return;
   }

   pDev = pReadURB->context;
   del_timer(&pDev->read_tmr);

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      usb_unlink_urb(pReadURB);
      return;
   }

   if(IsDeviceDisconnect(pDev))
   {
      DBG( "%s Disconnected!\n" ,__FUNCTION__);
      usb_unlink_urb(pReadURB);
      return;
   }
   if( atomic_read(&pReadURB->reject))
   {
      DBG( "%s reject!\n" ,__FUNCTION__);
      return;
   }
   if (pReadURB->status != 0)
   {
      DBG( "Read status = %d\n", pReadURB->status );
      if ((pReadURB->status == -ECONNRESET) && (pReadURB->actual_length > 0))
      {
          pDev->readTimeoutCnt++;
          // Read URB unlinked after receiving data, send data to client
          DBG( "Read URB timeout/kill after recv data\n" );
          printk(KERN_WARNING "Read URB timeout/kill, recv data len (%d), cnt (%d)\n",
                  pReadURB->actual_length, pDev->readTimeoutCnt);
          ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );
          return;
      }
      else
      {
          // Resubmit the interrupt URB
          if (IsDeviceValid( pDev ) == false)
          {
             DBG( "Invalid device!\n" );
             usb_unlink_urb(pReadURB);
             return;
          }
          if(IsDeviceDisconnect(pDev))
          {
            DBG( "%s Disconnected!\n" ,__FUNCTION__);
            usb_unlink_urb(pReadURB);
            return;
          }
          ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );
          return;
      }
   }
   DBG( "Read %d bytes\n", pReadURB->actual_length );

   pData = pReadURB->transfer_buffer;
   if((int)(pReadURB->actual_length)>=0)
   {
      dataSize = pReadURB->actual_length;
   }
   else
   {
      if(IsDeviceDisconnect(pDev))
      {
         DBG( "%s Disconnected!\n" ,__FUNCTION__);
         usb_unlink_urb(pReadURB);
         return;
      }
      ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );
      return;
   }

   PrintHex( pData, dataSize );

   result = ParseQMUX( &clientID,
                       pData,
                       dataSize );
   if(clientID==QMICTL)
   {
      u8 u8Type = 0;
      u16 u16MsgID = 0;
      u8Type = *(u8*)(pData + result);
      transactionID = *(u8*)(pData + result + 1);
      u16MsgID = le16_to_cpu( get_unaligned((u16*)(pData + result + 2)) );
      if(u8Type==QMI_CTL_IND)
      {
         DBG("u8Type:0x%02x, TID:0x%02x ,MSGID:0x%02x\n",
         u8Type, transactionID,u16MsgID);
         if(u16MsgID==QMI_CTL_SYNC_IND)
         {
            if(pDev)
            {
               if(pDev->dev!=NULL)
               {
                  printk(KERN_INFO "RESET DEVICE IND\n");
                  GobiUSBLockReset(pDev->mpIntf);
                  return;
               }
               else
               {
                  DBG("IGNORE REGISTER\n");
               }
            }
         }
      }
   }
   if (result < 0)
   {
      DBG( "Read error parsing QMUX %d\n", result );
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         usb_unlink_urb(pReadURB);
         return;
      }
      if(IsDeviceDisconnect(pDev))
      {
         DBG( "%s Disconnected!\n" ,__FUNCTION__);
         usb_unlink_urb(pReadURB);
         return;
      }
      // Resubmit the interrupt URB
      ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );

      return;
   }

   // Grab transaction ID

   // Data large enough?
   if (dataSize < result + 3)
   {
      DBG( "Data buffer too small to parse\n" );
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         usb_unlink_urb(pReadURB);
         return;
      }
      if(IsDeviceDisconnect(pDev))
      {
         DBG( "%s Disconnected!\n" ,__FUNCTION__);
         usb_unlink_urb(pReadURB);
         return;
      }
      // Resubmit the interrupt URB
      ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );

      return;
   }

   // Transaction ID size is 1 for QMICTL, 2 for others
   if (clientID == QMICTL)
   {
      transactionID = *(u8*)(pData + result + 1);
   }
   else
   {
      transactionID = le16_to_cpu( get_unaligned((u16*)(pData + result + 1)) );
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__ );

   // Find memory storage for this service and Client ID
   // Not using FindClientMem because it can't handle broadcasts
   pClientMem = pDev->mQMIDev.mpClientMemList;
   while (pClientMem != NULL)
   {
      if(IsDeviceDisconnect(pDev))
      {
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         usb_unlink_urb(pReadURB);
         return ;
      }
      if (pClientMem->mClientID == clientID
      ||  (pClientMem->mClientID | 0xff00) == clientID)
      {
         // Make copy of pData
         pDataCopy = kmalloc( dataSize, GOBI_GFP_ATOMIC );
         if (pDataCopy == NULL)
         {
            DBG( "Error allocating client data memory\n" );

            // End critical section
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               return;
            }
            if(IsDeviceDisconnect(pDev))
            {
               DBG( "%s Disconnected!\n" ,__FUNCTION__);
               usb_unlink_urb(pReadURB);
               return;
            }
            // Resubmit the interrupt URB
            ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );

            return;
         }

         memcpy( pDataCopy, pData, dataSize );
         mb();
         if (AddToReadMemList( pDev,
                               pClientMem->mClientID,
                               transactionID,
                               pDataCopy,
                               dataSize ) == false)
         {
            DBG( "Error allocating pReadMemListEntry "
                 "read will be discarded\n" );
            kfree( pDataCopy );

            // End critical section
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               if(pDev)
               {
                  LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               }
               else
               {
                  LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               }
               return;
            }
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            if(!IsDeviceDisconnect(pDev))
            {
               // Resubmit the interrupt URB
               ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );
            }
            return;
         }

         // Success
         CLIENT_READMEM_SNAPSHOT(clientID, pDev);
         #if 0
         DBG( "Creating new readListEntry for client 0x%04X, TID 0x%x\n",
              clientID,
              transactionID );
         #endif
         // Notify this client data exists
         iResult = NotifyAndPopNotifyList( pDev,
                             pClientMem->mClientID,
                             transactionID );
         if (iResult==eNotifyListFound)
          {
                //DBG("%s:%d Found ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
          }
          else if (iResult==eNotifyListEmpty)
          {
                DBG("%s:%d Empty ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
          }
          else
          {
            DBG("%s:%d Not Found ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               return;
            }
            if(  (pDev->mbUnload >= eStatUnloading)||
                  IsDeviceDisconnect(pDev) )
            {
               DBG( "Unload:%s\n", __FUNCTION__);
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               usb_unlink_urb(pReadURB);
               return ;
            }
         }
         if (iResult==eNotifyListFound)
         {
            // Possibly notify poll() that data exists
            wake_up_interruptible( &pClientMem->mWaitQueue );
         }
         else
         {
            void * pReadBuffer = NULL;
            u16 readBufferSize;
             // Pop the read data
             if (PopFromReadMemList( pDev,
                                     pClientMem->mClientID,
                                     transactionID,
                                     &pReadBuffer,
                                     &readBufferSize ) == true)
             {
                // Success
                DBG( "Remove Not Found Memory from read list\n" );
                if(pReadBuffer)
                kfree( pReadBuffer );
                pReadBuffer=NULL;
             }
         }
         // Not a broadcast
         if (clientID >> 8 != 0xff)
         {
            break;
         }
      }
      barrier();
      // Next element
      pClientMem = pClientMem->mpNext;
      mb();
   }
   mb();
   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      usb_unlink_urb(pReadURB);
      return;
   }

   if(IsDeviceDisconnect(pDev))
   {
      DBG( "%s Disconnected!\n" ,__FUNCTION__);
      usb_unlink_urb(pReadURB);
      return;
   }
   // Resubmit the interrupt URB
   ResubmitIntURB(pDev, pDev->mQMIDev.mpIntURB );
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 4,15,0 ))
void read_tmr_cb( struct urb * pReadURB )
{
   int result;
#else
void read_tmr_cb( struct timer_list *t)
{
   int result;
   sGobiUSBNet *pDev = NULL;
   struct urb *pReadURB = NULL;
   pDev = from_timer(pDev, t, read_tmr);
   if(pDev!=NULL)
   pReadURB = pDev->mQMIDev.mpReadURB;
#endif

  DBG( "%s called (%ld).\n", __func__, jiffies );

  if ((pReadURB != NULL) && (pReadURB->status == -EINPROGRESS))
  {
     // Asynchronously unlink URB. On success, -EINPROGRESS will be returned,
     // URB status will be set to -ECONNRESET, and ReadCallback() executed
     result = usb_unlink_urb( pReadURB );
     DBG( "%s called usb_unlink_urb, result = %d\n", __func__, result);
  }
}

/*===========================================================================
METHOD:
   IntCallback (Public Method)

DESCRIPTION:
   Data is available, fire off a read URB

PARAMETERS
   pIntURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void IntCallback( struct urb * pIntURB )
{
   int status;
   u64 CDCEncResp;
   u64 CDCEncRespMask;

   sGobiUSBNet * pDev = (sGobiUSBNet *)pIntURB->context;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Disconnected!\n" );
      pIntURB->status = -EIO;
      usb_unlink_urb(pIntURB);
      return;
   }
   // Verify this was a normal interrupt
   if (pIntURB->status != 0)
   {
        DBG( "Int status = %d\n", pIntURB->status );

      // Ignore EOVERFLOW errors
      if (pIntURB->status != -EOVERFLOW)
      {
         // Read 'thread' dies here
         usb_unlink_urb(pIntURB);
         return;
      }
      if(pIntURB->status<0)
      {
         usb_unlink_urb(pIntURB);
         return;
      }
   }
   else
   {
      //TODO cast transfer_buffer to struct usb_cdc_notification

      // CDC GET_ENCAPSULATED_RESPONSE
      CDC_GET_ENCAPSULATED_RESPONSE(&CDCEncResp, &CDCEncRespMask)
      #if 0
      DBG( "IntCallback: Encapsulated Response = 0x%llx\n",
          (*(u64*)pIntURB->transfer_buffer));
      #endif
      //AR7554RD returned interrupt buffer not matching expected mask
      //thus, length check only
      if (pIntURB->actual_length == 8)

      {
         // Time to read
         usb_fill_control_urb( pDev->mQMIDev.mpReadURB,
                               pDev->mpNetDev->udev,
                               usb_rcvctrlpipe( pDev->mpNetDev->udev, 0 ),
                               (unsigned char *)pDev->mQMIDev.mpReadSetupPacket,
                               pDev->mQMIDev.mpReadBuffer,
                               DEFAULT_READ_URB_LENGTH,
                               ReadCallbackInt,
                               pDev );
         gobi_setup_timer( &pDev->read_tmr, (void*)read_tmr_cb, (unsigned long)pDev->mQMIDev.mpReadURB );
         mod_timer( &pDev->read_tmr, jiffies + msecs_to_jiffies(USB_READ_TIMEOUT) );
         mb();
         status = usb_submit_urb( pDev->mQMIDev.mpReadURB, GOBI_GFP_ATOMIC );
         if (status != 0)
         {
            DBG( "Error submitting Read URB %d\n", status );
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               return;
            }
            if(IsDeviceDisconnect(pDev))
            {
               DBG( "Disconnected!\n" );
               return;
            }
            // Resubmit the interrupt urb
            ResubmitIntURB( pDev,pIntURB );
            return;
         }

         // Int URB will be resubmitted during ReadCallback
         return;
      }
      // CDC CONNECTION_SPEED_CHANGE
      else if ((pIntURB->actual_length == 16)
           &&  (CDC_GET_CONNECTION_SPEED_CHANGE(&CDCEncResp, &CDCEncRespMask))
           &&  ((*(u64*)pIntURB->transfer_buffer & CDCEncRespMask) == CDCEncResp ) )
      {
         DBG( "Connection Speed Change = 0x%llx\n",
              (*(u64*)pIntURB->transfer_buffer));

         // if upstream or downstream is 0, stop traffic.  Otherwise resume it
         if ((*(u32*)(pIntURB->transfer_buffer + 8) == 0)
         ||  (*(u32*)(pIntURB->transfer_buffer + 12) == 0))
         {
            GobiSetDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "traffic stopping due to CONNECTION_SPEED_CHANGE\n" );
         }
         else
         {
            GobiClearDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "resuming traffic due to CONNECTION_SPEED_CHANGE\n" );
         }
      }
      else
      {
         DBG( "ignoring invalid interrupt in packet\n" );
         PrintHex( pIntURB->transfer_buffer, pIntURB->actual_length );
      }
   }
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Disconnected!\n" );
      pIntURB->status = -EIO;
      usb_unlink_urb(pIntURB);
      return;
   }
   // Resubmit the interrupt urb
   ResubmitIntURB(pDev,pIntURB);

   return;
}

/*===========================================================================
METHOD:
   StartRead (Public Method)

DESCRIPTION:
   Start continuous read "thread" (callback driven)

   Note: In case of error, KillRead() should be run
         to remove urbs and clean up memory.

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int StartRead( sGobiUSBNet * pDev )
{
   int interval;
   struct usb_endpoint_descriptor *pendp;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Allocate URB buffers
   pDev->mQMIDev.mpReadURB = usb_alloc_urb( 0, GOBI_GFP_KERNEL );
   if (pDev->mQMIDev.mpReadURB == NULL)
   {
      DBG( "Error allocating read urb\n" );
      return -ENOMEM;
   }

   pDev->mQMIDev.mpIntURB = usb_alloc_urb( 0, GOBI_GFP_KERNEL );
   if (pDev->mQMIDev.mpIntURB == NULL)
   {
      DBG( "Error allocating int urb\n" );
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   // Create data buffers
   pDev->mQMIDev.mpReadBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GOBI_GFP_KERNEL );
   if (pDev->mQMIDev.mpReadBuffer == NULL)
   {
      DBG( "Error allocating read buffer\n" );
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   pDev->mQMIDev.mpIntBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GOBI_GFP_KERNEL );
   if (pDev->mQMIDev.mpIntBuffer == NULL)
   {
      DBG( "Error allocating int buffer\n" );
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   pDev->mQMIDev.mpReadSetupPacket = kmalloc( sizeof( sURBSetupPacket ),
                                              GOBI_GFP_KERNEL );
   if (pDev->mQMIDev.mpReadSetupPacket == NULL)
   {
      DBG( "Error allocating setup packet buffer\n" );
      kfree( pDev->mQMIDev.mpIntBuffer );
      pDev->mQMIDev.mpIntBuffer = NULL;
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   // CDC Get Encapsulated Response packet
   pDev->mQMIDev.mpReadSetupPacket->mRequestType =
       USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
   pDev->mQMIDev.mpReadSetupPacket->mRequestCode = GET_ENCAPSULATED_RESPONSE;
   pDev->mQMIDev.mpReadSetupPacket->mValue = 0;
   pDev->mQMIDev.mpReadSetupPacket->mIndex =
      cpu_to_le16(pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber); /* interface number */
   pDev->mQMIDev.mpReadSetupPacket->mLength = cpu_to_le16(DEFAULT_READ_URB_LENGTH);

   pendp = GetEndpoint(pDev->mpIntf, USB_ENDPOINT_XFER_INT, USB_DIR_IN);
   if (pendp == NULL)
   {
      DBG( "Invalid interrupt endpoint!\n" );
      kfree(pDev->mQMIDev.mpReadSetupPacket);
      pDev->mQMIDev.mpReadSetupPacket = NULL;
      kfree( pDev->mQMIDev.mpIntBuffer );
      pDev->mQMIDev.mpIntBuffer = NULL;
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENXIO;
   }

   // Interval needs reset after every URB completion
   interval = (pDev->mpNetDev->udev->speed == USB_SPEED_HIGH) ?
                 6 : max((int)(pendp->bInterval), 3);

   // Schedule interrupt URB
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   mb();
   wait_interrupt();
   spin_lock_irq(&(pDev->urb_lock));
   usb_fill_int_urb( pDev->mQMIDev.mpIntURB,
                     pDev->mpNetDev->udev,
                     /* QMI interrupt endpoint for the following
                      * interface configuration: DM, NMEA, MDM, NET
                      */
                     usb_rcvintpipe( pDev->mpNetDev->udev,
                                     pendp->bEndpointAddress),
                     pDev->mQMIDev.mpIntBuffer,
                     le16_to_cpu(pendp->wMaxPacketSize),
                     IntCallback,
                     pDev,
                     interval );
   spin_unlock_irq(&(pDev->urb_lock));
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   mb();
   return usb_submit_urb( pDev->mQMIDev.mpIntURB, GOBI_GFP_ATOMIC );
}

/*===========================================================================
METHOD:
   KillRead (Public Method)

DESCRIPTION:
   Kill continuous read "thread"

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void KillRead( sGobiUSBNet * pDev )
{

   if(pDev ==NULL)
   {
      DBG( "pDev NULL\n" );
      return ;
   }
   #ifndef CONFIG_ANDROID
   mb();
   #endif
   local_irq_disable();
   GobiCancelReadCallBackWorkQueue(pDev);
   // Stop reading
   if (pDev->mQMIDev.mpReadURB != NULL)
   {
      DBG( "Killng read URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpReadURB );
   }

   if (pDev->mQMIDev.mpIntURB != NULL)
   {
      DBG( "Killng int URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpIntURB );
   }

   local_irq_enable();

   // Release buffers
   kfree( pDev->mQMIDev.mpReadSetupPacket );
   pDev->mQMIDev.mpReadSetupPacket = NULL;
   kfree( pDev->mQMIDev.mpReadBuffer );
   pDev->mQMIDev.mpReadBuffer = NULL;
   kfree( pDev->mQMIDev.mpIntBuffer );
   pDev->mQMIDev.mpIntBuffer = NULL;

   // Release URB's
   usb_free_urb( pDev->mQMIDev.mpReadURB );
   pDev->mQMIDev.mpReadURB = NULL;
   usb_free_urb( pDev->mQMIDev.mpIntURB );
   pDev->mQMIDev.mpIntURB = NULL;
}

/*===========================================================================
METHOD:
   InitSemID (Public Method)

DESCRIPTION:
   Initialize Read Sync tasks semaphore

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/

void InitSemID(sGobiUSBNet * pDev)
{
   int i = 0;
   if(pDev==NULL)
   {
      DBG("%s NULL\n",__FUNCTION__);
      return ;
   }
   sema_init( &(pDev->ReadsyncSem), SEMI_INIT_DEFAULT_VALUE );
   mb();
   up(&(pDev->ReadsyncSem));

   for(i=0;i<MAX_READ_SYNC_TASK_ID;i++)
   {
     pDev->iReasSyncTaskID[i]=-__LINE__;
     sema_init( &(pDev->readSem[i]), SEMI_INIT_DEFAULT_VALUE );
     mb();
   }
   up(&(pDev->ReadsyncSem));
}
int gobi_kthread_should_stop(void)
{
   //kthread_should_stop();
   return 0;
}

/*=========================================================================*/
// Internal read/write functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ReadAsync (Public Method)

DESCRIPTION:
   Start asynchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pCallback         [ I ] - Callback to be executed when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified)
                             to callback

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int ReadAsync(
   sGobiUSBNet *    pDev,
   u16                clientID,
   u16                transactionID,
   void               (*pCallback)(sGobiUSBNet*, u16, void *),
   void *             pData ,
   int                iSpinLock)
{
   sClientMemList * pClientMem = NULL;
   sReadMemList ** ppReadMemList = NULL;
   unsigned long flags = 0;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Critical section
   if(iSpinLock==1)
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Find memory storage for this client ID
   pClientMem = FindClientMem( pDev, clientID );
   if(clientID ==0)
   {
      if(iSpinLock==1)
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      AddClientToMemoryList(pDev,clientID);
      if(iSpinLock==1)
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
      pClientMem = FindClientMem( pDev, clientID );
   }
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );

      // End critical section
      if(iSpinLock==1)
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENXIO;
   }

   ppReadMemList = &(pClientMem->mpList);

   // Does data already exist?
   while (*ppReadMemList != NULL)
   {
      // Is this element our data?
      if (transactionID == 0
      ||  transactionID == (*ppReadMemList)->mTransactionID)
      {
         // End critical section
         if(iSpinLock==1)
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         if((pCallback==UpSem) && (pData ==NULL))
         {
            DBG("%d clientID:0x%X,mpData:0x%p",__LINE__,clientID,pData);
            return 0;
         }
         DBG("%d clientID:0x%X,mpData:0x%p",__LINE__,clientID,pData);
         // Run our own callback
         pCallback( pDev, clientID, pData );

         return 0;
      }

      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }
   mb();
   // Data not found, add ourself to list of waiters
   if (AddToNotifyList( pDev,
                        clientID,
                        transactionID,
                        pCallback,
                        pData ) == false)
   {
      DBG( "Unable to register for notification\n" );
   }

   // End critical section
   if(iSpinLock==1)
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   // Success
   return 0;
}

/*===========================================================================
METHOD:
   UpSem (Public Method)

DESCRIPTION:
   Notification function for synchronous read

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   pData             [ I ] - Buffer that holds semaphore to be up()-ed

RETURN VALUE:
   None
===========================================================================*/
void UpSem(
   sGobiUSBNet * pDev,
   u16             clientID,
   void *          pData )
{
   //DBG( "0x%04X\n", clientID );
   mb();
   if(pData!=NULL)
   {
      struct semaphore *sem = (struct semaphore *)pData;
      barrier();
      up(sem);
   }
   return;
}

/*===========================================================================
METHOD:
   ReadSync (Public Method)

DESCRIPTION:
   Start synchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   ppOutBuffer       [I/O] - On success, will be filled with a
                             pointer to read buffer
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   int - size of data read for success
         negative errno for failure
===========================================================================*/
int ReadSync(
   sGobiUSBNet *    pDev,
   void **            ppOutBuffer,
   u16                clientID,
   u16                transactionID,
   int                *iID,
   struct semaphore   *pReadSem,
   int                *iIsClosing)
{
   int result;
   sClientMemList * pClientMem;
   sNotifyList ** ppNotifyList, * pDelNotifyListEntry;
   void * pData = NULL;
   u16 dataSize;
   struct semaphore *pLocalreadSem = NULL;
   unsigned long flags;
   //DBG("\n");
   if(pReadSem==NULL)
   if(*iID<0)
   {
      DBG( "Could not find matching SemID\n");
      return -ENXIO;
   }

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "unloaded\n" );
      return -EFAULT;
   }

   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }

   if(pReadSem==NULL)
   {
      pLocalreadSem = &(pDev->readSem[*iID]);
   }
   else
   {
     pLocalreadSem = pReadSem;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Find memory storage for this Client ID
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENXIO;
   }

   // Note: in cases where read is interrupted,
   //    this will verify client is still valid
   while (PopFromReadMemList( pDev,
                              clientID,
                              transactionID,
                              &pData,
                              &dataSize ) == false)
   {
      // Data does not yet exist, wait

      if(pDev->mbUnload >= eStatUnloading)
      {
         DBG("Unloading\n");
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return -ENXIO;
      }
      // Add ourself to list of waiters
      if (AddToNotifyList( pDev,
                           clientID,
                           transactionID,
                           UpSem,
                           pLocalreadSem ) == false)
      {
         DBG( "unable to register for notification\n" );
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return -EFAULT;
      }

      // End critical section while we block
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

      // Wait for notification
      if (signal_pending(current))
      {
         return -ERESTARTSYS;
      }
      result = gobi_down_interruptible( pLocalreadSem,pDev);
      DBG("result:%d , CID:0x%04x\n",result,clientID);
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         return -EFAULT;
      }
      if (pDev->mbUnload > eStatUnloading)
      {
         DBG( "unloaded\n" );
          if(pReadSem==NULL)
          pDev->iReasSyncTaskID[*iID] = -__LINE__;
          else
          *iID = -__LINE__;
           return -EFAULT;
      }
      if(iIsClosing!=NULL)
      {
         if(*iIsClosing>0)
         {
           DBG( "filp is closing\n" );
           if(pReadSem==NULL)
           pDev->iReasSyncTaskID[*iID] = -__LINE__;
           else
           *iID = -__LINE__;
           mb();
           flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
           RemoveAndPopNotifyList(pDev,clientID,transactionID,eClearAndReleaseCID);
           LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
           return -EFAULT;
         }
      }
      if (result < 0)
      {
         if(pDev)
         {
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
         }
         //userspace neglect -EINTR
         if(result!=-EINTR)
         {
             flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
             RemoveAndPopNotifyList(pDev,clientID,transactionID,eClearCID);
             LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         }
         return result;//-EFAULT;EINTR resume error
      }
      if(*iID<-1)
      {
         DBG( "%s:%d Interrupted %d iID:%d\n",__FUNCTION__,__LINE__, result,*iID );
         return -EFAULT;
      }
      if (signal_pending(current))
      {
         return -ERESTARTSYS;
      }
      if (result != 0)
      {
         DBG( "Interrupted %d\n", result );

         // readSem will fall out of scope,
         // remove from notify list so it's not referenced
         if(pDev==NULL)
         {
            return -EFAULT;
         }
         if(pDev->iIsClosing)
         {
            DBG( "Closing device!\n" );
            if(pDev)
            {
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
            }
            return -EFAULT;
         }
         if(pDev->mbUnload >= eStatUnloading)
         {
            DBG("Unloading");
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
            return -ENXIO;
         }
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         if(pDev)
         {
            flags = pDev->mQMIDev.mFlag;
         }
         ppNotifyList = &(pClientMem->mpReadNotifyList);
         pDelNotifyListEntry = NULL;

         // Find and delete matching entry
         while (*ppNotifyList != NULL)
         {
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               /* SWI_START */
               /* workaround to leave the system in cleaner state:
                * must enable interrupts and enable pre-emption.
                * TBD what the correct action should be when the device is gone.
                */
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               //preempt_enable();
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
               /* SWI_STOP */
               return -EFAULT;
            }
            if(*iID<0)
            {
               DBG( "Invalid device!\n" );
               /* SWI_START */
               /* must restore irq, pre-emption, locks before returning */
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               if(pReadSem==NULL)
               {
                  if(pDev)
                  pDev->iReasSyncTaskID[*iID] = -__LINE__;
               }
               else
               *iID = -__LINE__;
               /* SWI_STOP */
               return -EFAULT;
            }
            if(pDev==NULL)
            {
               DBG( "Invalid device!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               return -EFAULT;
            }
            if (pDev->mbUnload > eStatUnloading)
            {
               DBG( "UNLOADING!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               if(pReadSem==NULL)
               pDev->iReasSyncTaskID[*iID] = -__LINE__;
               else
               *iID = -__LINE__;
               return -EFAULT;
            }
            if(ppNotifyList==NULL)
            {
                DBG( "UNLOADING!\n" );
                LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
                return -EFAULT;
            }
            if ((*ppNotifyList)->mpData ==
                  pLocalreadSem)
            {
               pDelNotifyListEntry = *ppNotifyList;
               *ppNotifyList = (*ppNotifyList)->mpNext;
               kfree( pDelNotifyListEntry );
               mb();
               break;
            }
            barrier();
            // Next
            ppNotifyList = &(*ppNotifyList)->mpNext;
            mb();
         }
         mb();
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         if(pDev)
        {
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
        }
         return -EINTR;
      }

      // Verify device is still valid
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         if(pDev)
         {
             if(pReadSem==NULL)
             pDev->iReasSyncTaskID[*iID] = -__LINE__;
             else
             *iID = -__LINE__;
         }
         return -ENXIO;
      }

      // Restart critical section and continue loop
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   }

   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   // Success
   *ppOutBuffer = pData;
   if(pDev)
   {
      if(pReadSem==NULL)
      pDev->iReasSyncTaskID[*iID] = -__LINE__;
      else
      *iID = -__LINE__;
   }
   return dataSize;
}

/*===========================================================================
METHOD:
   WriteSync (Public Method)

DESCRIPTION:
   Start synchronous write

PARAMETERS:
   pDev                 [ I ] - Device specific memory
   pWriteBuffer         [ I ] - Data to be written
   writeBufferSize      [ I ] - Size of data to be written
   clientID             [ I ] - Client ID of requester

RETURN VALUE:
   int - write size (includes QMUX)
         negative errno for failure
===========================================================================*/
int WriteSync(
   sGobiUSBNet *          pDev,
   char *                 pWriteBuffer,
   int                    writeBufferSize,
   u16                    clientID )
{
   int i;
   int result;
   int iLockRetry =0;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "Unloading device!\n" );
      return -ENXIO;
   }

   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }

   // Fill writeBuffer with QMUX
   result = FillQMUX( clientID, pWriteBuffer, writeBufferSize );
   if (result < 0)
   {
      return result;
   }

   // Wake device
   result = gobi_usb_autopm_get_interface( pDev->mpIntf );
   if (result < 0)
   {
      DBG( "unable to resume interface: %d\n", result );

      // Likely caused by device going from autosuspend -> full suspend
      if (result == -EPERM)
      {
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
         pDev->mpNetDev->udev->auto_pm = 0;
#endif
         GobiNetSuspend( pDev->mpIntf, PMSG_SUSPEND );
#endif /* CONFIG_PM */
      }
      return result;
   }

   DBG( "Actual Write:\n" );
   PrintHex( pWriteBuffer, writeBufferSize );

   // Write Control URB, protect with read semaphore to track in-flight USB control writes in case of disconnect
   for(i=0;i<USB_WRITE_RETRY;i++)
   {

      if(isModuleUnload(pDev))
      {
         DBG( "unloaded\n" );
         return -EFAULT;
      }
      if(IsDeviceDisconnect(pDev))
      {
         DBG( "Device Disconnected!\n" );
         return -ENXIO;
      }
      pDev->iShutdown_read_sem= __LINE__;
      if(signal_pending(current))
      {
         return -ERESTARTSYS;
      }

      iLockRetry = 0;
      mb();
      while(down_read_trylock(&(pDev->shutdown_rwsem))!=1)
      {
         wait_ms(5);
         mb();
         if(iLockRetry++>100)
         {
            DBG("down_read_trylock timeout");
            return -EFAULT;
         }
         if(pDev==NULL)
         {
            DBG( "NULL\n" );
            return -EFAULT;
         }
         if (pDev->mbUnload >= eStatUnloading)
         {
            DBG( "unloaded\n" );
            return -EFAULT;
         }
         if(IsDeviceDisconnect(pDev))
         {
            DBG( "Device Disconnected!\n" );
            return -ENXIO;
         }
      }
      mb();
      result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev, usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
             SEND_ENCAPSULATED_COMMAND,
             USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
             0, pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
             (void*)pWriteBuffer, writeBufferSize,
             USB_WRITE_TIMEOUT );
       up_read(&pDev->shutdown_rwsem);
       if(signal_pending(current))
       {
          return -ERESTARTSYS;
       }
       if(pDev==NULL)
       {
          return -EFAULT;
       }
       if (IsDeviceDisconnect(pDev) )
       {
         return -ENXIO;
       }
       pDev->iShutdown_read_sem=- __LINE__;

       if (pDev->mbUnload >= eStatUnloading)
       {
          DBG( "unloaded\n" );
          return -EFAULT;
       }

       if (signal_pending(current))
       {
           return -ERESTARTSYS;
       }

       if (result < 0)
       {
          printk(KERN_WARNING "usb_control_msg failed (%d)", result);
       }
       // Control write transfer may occasionally timeout with certain HCIs, attempt a second time before reporting an error
       if (result == -ETIMEDOUT)
       {
           pDev->writeTimeoutCnt++;
           printk(KERN_WARNING "Write URB timeout, cnt(%d)\n", pDev->writeTimeoutCnt);
       }
       else if(result < 0 )
       {
          DBG( "%s no device!\n" ,__FUNCTION__);
           return result;
       }
       else
       {
           break;
       }
       if (IsDeviceValid( pDev ) == false)
       {
          DBG( "%s Invalid device!\n" ,__FUNCTION__);
          return -ENXIO;
       }
       if(IsDeviceDisconnect(pDev))
       {
          DBG( "Device Disconnected!\n" );
          return -ENXIO;
       }
       if (pDev->mbUnload > eStatUnloading)
       {
         DBG( "unloaded\n" );
         return -EFAULT;
       }
   }

   // Write is done, release device
   gobi_usb_autopm_put_interface( pDev->mpIntf );


   return result;
}

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   AddClientToMemoryList (Public Method)

DESCRIPTION:
   Add Client To Memory List

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Client ID

RETURN VALUE:
   int - Add Client ID to Memory List for success (positive)
         Negative errno for error
===========================================================================*/
int AddClientToMemoryList(sGobiUSBNet *pDev,u16 clientID)
{
   unsigned long flags;
   sClientMemList ** ppClientMem;
   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Verify client is not already allocated
   if (FindClientMem( pDev, clientID ) != NULL)
   {
      DBG( "Client memory already exists CID:0x%x\n",clientID );
      // End Critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ETOOMANYREFS;
   }
   //DBG( "Add Client memory CID:0x%x\n",clientID );
   // Go to last entry in client mem list
   ppClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppClientMem != NULL)
   {
      ppClientMem = &(*ppClientMem)->mpNext;
   }

   // Create locations for read to place data into
   *ppClientMem = kmalloc( sizeof( sClientMemList ), GOBI_GFP_ATOMIC );
   if (*ppClientMem == NULL)
   {
      DBG( "Error allocating read list\n" );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENOMEM;
   }

   (*ppClientMem)->mClientID = clientID;
   (*ppClientMem)->mpList = NULL;
   (*ppClientMem)->mpReadNotifyList = NULL;
   (*ppClientMem)->mpURBList = NULL;
   (*ppClientMem)->mpNext = NULL;
   mb();
   // Initialize workqueue for poll()
   init_waitqueue_head( &(*ppClientMem)->mWaitQueue );

   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return (int)clientID;
}

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   GetClientID (Public Method)

DESCRIPTION:
   Request a QMI client for the input service type and initialize memory
   structure

PARAMETERS:
   pDev           [ I ] - Device specific memory
   serviceType    [ I ] - Desired QMI service type

RETURN VALUE:
   int - Client ID for success (positive)
         Negative errno for error
===========================================================================*/
int GetClientID(
   sGobiUSBNet *    pDev,
   u8                 serviceType ,
   struct semaphore   *pReadSem)
{
   u16 clientID;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u8 transactionID;
   unsigned long flags;
   struct semaphore readSem;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   // Run QMI request to be asigned a Client ID
   if (serviceType != 0)
   {
      writeBufferSize = QMICTLGetClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         return -ENOMEM;
      }

      /* transactionID cannot be 0 */
      transactionID = QMIXactionIDGet(pDev);
      if (transactionID != 0)
      {
         result = QMICTLGetClientIDReq( pWriteBuffer,
                                        writeBufferSize,
                                        transactionID,
                                        serviceType );
         if (result < 0)
         {
            if(pWriteBuffer)
            {
               kfree( pWriteBuffer );
               pWriteBuffer = NULL;
            }
            return result;
         }
      }
      else
      {
         if(pWriteBuffer)
         {
            kfree( pWriteBuffer );
            pWriteBuffer = NULL;
         }
         DBG( "Invalid transaction ID!\n" );
         return -EINVAL;
      }

      result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);
      if (result < 0)
      {
         DBG( "ReadAsync Error!\n" );
         if(pWriteBuffer)
         {
            kfree( pWriteBuffer );
            pWriteBuffer = NULL;
         }
         return result;
      }
      result = WriteSync( pDev,
                          pWriteBuffer,
                          writeBufferSize,
                          QMICTL );
      if(pWriteBuffer)
      {
         kfree( pWriteBuffer );
         pWriteBuffer = NULL;
      }
      if (result < 0)
      {
         // Timeout, remove the async read
         DBG( "%s Timeout!\n" ,__FUNCTION__);
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         // Timeout, remove the async read
         RemoveAndPopNotifyList(pDev,QMICTL,transactionID,eClearAndReleaseCID);
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return result;
      }
      wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
      mb();
      // Enter critical section
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
      barrier();
      spin_lock_irq(&(pDev->notif_lock));
      if (down_trylock( &readSem ) == 0)
      {
         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 QMICTL,
                                 transactionID,
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            // Success
            DBG( "Success!\n" );
            spin_unlock_irq(&(pDev->notif_lock));
            // End critical section
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            result = QMICTLGetClientIDResp( pReadBuffer,
                                      readBufferSize,
                                      &clientID );
            // We don't care about the result
            DBG( "QMICTLGetClientIDResp Result:%d!\n" ,result);
            if(pReadBuffer)
            kfree( pReadBuffer );
            pReadBuffer=NULL;
         }
         else
         {
            // Read mismatch/failure, unlock and continue
            DBG( "Read mismatch/failure, unlock and continue!\n" );
            spin_unlock_irq(&(pDev->notif_lock));
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            result = -ETIMEDOUT;
         }
      }
      else
      {
             // Timeout, remove the async read
             DBG( "Timeout, remove the async read!\n" );
             //ReleaseNotifyList( pDev, QMICTL, transactionID );
             // Timeout, remove the async read
             RemoveAndPopNotifyList(pDev,QMICTL,transactionID,eClearAndReleaseCID);
             // End critical section
             spin_unlock_irq(&(pDev->notif_lock));
             LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
             result = -ETIMEDOUT;
      }
     /* Upon return from QMICTLGetClientIDResp, clientID
      * low address contains the Service Number (SN), and
      * clientID high address contains Client Number (CN)
      * For the ReadCallback to function correctly,we swap
      * the SN and CN on a Big Endian architecture.
      */
      clientID = le16_to_cpu(clientID);

      if (result < 0)
      {
         DBG( "OVERWRITE result:%d!\n",result );
         result = -ETIMEDOUT;
         return result;
      }
   }
   else
   {
      // QMI CTL will always have client ID 0
      clientID = 0;
   }
   return AddClientToMemoryList(pDev,clientID);
}

/*===========================================================================
METHOD:
   ReleaseClientID (Public Method)

DESCRIPTION:
   Release QMI client and free memory

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   true - 0 success.
   false - on error.
===========================================================================*/
bool ReleaseClientID(
   sGobiUSBNet *    pDev,
   u16                clientID)
{
   sClientMemList ** ppDelClientMem;
   sClientMemList * pNextClientMem;
   void * pDelData = NULL;
   u16 dataSize;
   unsigned long flags = 0;
   bool bReturn = true;

   // Is device is still valid?
   DBG("clientID:0x%x\n",clientID);
   if (pDev->mbUnload > eStatUnloaded)
   {
      DBG( "unloaded\n" );
      return false;
   }
   barrier();

   if(TransceiveReleaseClientID(pDev,clientID)==false)
   {
      pDev->mReleaseClientIDFail = 1;
   }

   // Cleaning up client memory
   DBG("Release clientID:0x%x memory\n",clientID);
   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Can't use FindClientMem, I need to keep pointer of previous
   ppDelClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppDelClientMem != NULL)
   {
      mb();
      if ((*ppDelClientMem)->mClientID == clientID)
      {
         barrier();
         pNextClientMem = (*ppDelClientMem)->mpNext;

         // Notify all clients
         while (NotifyAndPopNotifyList( pDev,
                                        clientID,
                                        0 ) == eNotifyListFound );
         // Free any unread data
         while (PopFromReadMemList( pDev,
                                    clientID,
                                    0,
                                    &pDelData,
                                    &dataSize ) == true )
         {
            kfree( pDelData );
            pDelData = NULL;
         }
         //DBG("Delete client Mem\r\n");
         if(*ppDelClientMem!=NULL)
         {
            // Delete client Mem
            kfree( *ppDelClientMem );
         }
         else
         {
            bReturn = false;
         }
         *ppDelClientMem = NULL;
         //DBG("Prepare Next Delete client Mem\r\n");
         // Overwrite the pointer that was to this client mem
         barrier();
         *ppDelClientMem = pNextClientMem;
      }
      else
      {
         // I now point to (a pointer of ((the node I was at)'s mpNext))
          if(*ppDelClientMem==NULL)
          {
              DBG("ppDelClientMem NULL %d\r\n",__LINE__);
              break;
          }
          //DBG("mpClientMemList:%p,*ppDelClientMem:%p",&pDev->mQMIDev.mpClientMemList,*ppDelClientMem);
          barrier();
          pNextClientMem = (*ppDelClientMem)->mpNext;
          barrier();
          ppDelClientMem = &(*ppDelClientMem)->mpNext;
      }
   }
   mb();
   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return bReturn;
}

/*===========================================================================
METHOD:
   FindClientMem (Public Method)

DESCRIPTION:
   Find this client's memory

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   sClientMemList - Pointer to requested sClientMemList for success
                    NULL for error
===========================================================================*/
sClientMemList * FindClientMem(
   sGobiUSBNet *      pDev,
   u16              clientID )
{
   sClientMemList * pClientMem;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return NULL;
   }
   mb();
#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   pClientMem = pDev->mQMIDev.mpClientMemList;
   while (pClientMem != NULL)
   {
      if (pClientMem->mClientID == clientID)
      {
         // Success
         //DBG("Found client's 0x%x memory\n", clientID);
         return pClientMem;
      }

      pClientMem = pClientMem->mpNext;
   }

   //DBG( "Could not find client mem 0x%04X\n", clientID );
   return NULL;
}

/*===========================================================================
METHOD:
   AddToReadMemList (Public Method)

DESCRIPTION:
   Add Data to this client's ReadMem list

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID
   transactionID  [ I ] - Transaction ID or 0 for any
   pData          [ I ] - Data to add
   dataSize       [ I ] - Size of data to add

RETURN VALUE:
   bool
===========================================================================*/
bool AddToReadMemList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void *           pData,
   u16              dataSize )
{
   sClientMemList * pClientMem;
   sReadMemList ** ppThisReadMemList;

#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }

   // Go to last ReadMemList entry
   ppThisReadMemList = &pClientMem->mpList;
   while (*ppThisReadMemList != NULL)
   {
      ppThisReadMemList = &(*ppThisReadMemList)->mpNext;
   }
   mb();
   *ppThisReadMemList = kmalloc( sizeof( sReadMemList ), GOBI_GFP_ATOMIC );
   if (*ppThisReadMemList == NULL)
   {
      DBG( "Mem error\n" );

      return false;
   }

   (*ppThisReadMemList)->mpNext = NULL;
   (*ppThisReadMemList)->mpData = pData;
   (*ppThisReadMemList)->mDataSize = dataSize;
   (*ppThisReadMemList)->mTransactionID = transactionID;
   mb();
   return true;
}

/*===========================================================================
METHOD:
   PopFromReadMemList (Public Method)

DESCRIPTION:
   Remove data from this client's ReadMem list if it matches
   the specified transaction ID.

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   ppData            [I/O] - On success, will be filled with a
                             pointer to read buffer
   pDataSize         [I/O] - On succces, will be filled with the
                             read buffer's size

RETURN VALUE:
   bool
===========================================================================*/
bool PopFromReadMemList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void **          ppData,
   u16 *            pDataSize )
{
   sClientMemList * pClientMem;
   sReadMemList * pDelReadMemList, ** ppReadMemList;
   //DBG("");
#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }

   ppReadMemList = &(pClientMem->mpList);
   pDelReadMemList = NULL;

   // Find first message that matches this transaction ID
   CLIENT_READMEM_SNAPSHOT(clientID, pDev);
   while (*ppReadMemList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  transactionID == (*ppReadMemList)->mTransactionID )
      {
         pDelReadMemList = *ppReadMemList;
         #if 0
         DBG(  "*ppReadMemList = 0x%p pDelReadMemList = 0x%p\n",
               *ppReadMemList, pDelReadMemList );
         #endif
         break;
      }

      //DBG( "skipping 0x%04X data TID = 0x%x\n", clientID, (*ppReadMemList)->mTransactionID );

      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }
   mb();
   //DBG(  "*ppReadMemList = 0x%p pDelReadMemList = 0x%p\n",
   //      *ppReadMemList, pDelReadMemList );
   if (pDelReadMemList != NULL)
   {
       if(*ppReadMemList==NULL)
       {
           DBG("ppReadMemList NULL\n");
           return false;
       }
      *ppReadMemList = (*ppReadMemList)->mpNext;

      // Copy to output
      *ppData = pDelReadMemList->mpData;
      *pDataSize = pDelReadMemList->mDataSize;
      #if 0
      DBG(  "*ppData = 0x%p pDataSize = %u\n",
            *ppData, *pDataSize );
      #endif
      // Free memory
      kfree( pDelReadMemList );
      pDelReadMemList = NULL;
      mb();
      return true;
   }
   else
   {
      DBG( "No read memory to pop, Client 0x%04X, TID = 0x%x\n",
           clientID,
           transactionID );
      return false;
   }
}

/*===========================================================================
METHOD:
   AddToNotifyList (Public Method)

DESCRIPTION:
   Add Notify entry to this client's notify List

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pNotifyFunct      [ I ] - Callback function to be run when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified)
                             to callback

RETURN VALUE:
   bool
===========================================================================*/
bool AddToNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void             (* pNotifyFunct)(sGobiUSBNet *, u16, void *),
   void *           pData )
{
   sClientMemList * pClientMem;
   sNotifyList ** ppThisNotifyList;
   //DBG("ClientID:0x%x, TID:0x%x\n",clientID,transactionID);
   mb();
   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return false;
   }

   // Go to last URBList entry
   ppThisNotifyList = &pClientMem->mpReadNotifyList;
   while (*ppThisNotifyList != NULL)
   {
      ppThisNotifyList = &(*ppThisNotifyList)->mpNext;
   }
   mb();
   *ppThisNotifyList = kmalloc( sizeof( sNotifyList ), GOBI_GFP_ATOMIC );
   if (*ppThisNotifyList == NULL)
   {
      DBG( "Mem error\n" );
      return false;
   }

   (*ppThisNotifyList)->mpNext = NULL;
   (*ppThisNotifyList)->mpNotifyFunct = pNotifyFunct;
   (*ppThisNotifyList)->mpData = pData;
   (*ppThisNotifyList)->mTransactionID = transactionID;
   mb();
   return true;
}

/*===========================================================================
METHOD:
   NotifyAndPopNotifyList (Public Method)

DESCRIPTION:
   Remove first Notify entry from this client's notify list
   and Run function

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   bool
===========================================================================*/
int NotifyAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID )
{
   sClientMemList * pClientMem;
   sNotifyList * pDelNotifyList = NULL, **ppNotifyList;

   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return eNotifyListEmpty;
   }

   ppNotifyList = &(pClientMem->mpReadNotifyList);
   pDelNotifyList = NULL;

   // Remove from list
   CLIENT_READMEM_SNAPSHOT(clientID,pDev);
   while (*ppNotifyList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  (*ppNotifyList)->mTransactionID == 0
      ||  transactionID == (*ppNotifyList)->mTransactionID)
      {
         if((clientID==0)&&(transactionID!=(*ppNotifyList)->mTransactionID))
         {

         }
         else
         {
            pDelNotifyList = *ppNotifyList;
            mb();
            break;
         }
      }

      DBG( "skipping data TID = %x\n", (*ppNotifyList)->mTransactionID );

      // next
      barrier();
      ppNotifyList = &(*ppNotifyList)->mpNext;
      mb();
   }
   mb();
   if (pDelNotifyList != NULL)
   {
      // Remove element
      *ppNotifyList = (*ppNotifyList)->mpNext;

      // Run notification function
      if (pDelNotifyList->mpNotifyFunct != NULL)
      {
          // Unlock for callback
          if(((clientID==QMICTL)&&(pDev->mbUnload>=eStatUnloading)) ||
               IsDeviceDisconnect(pDev))
          {

          }
          else
          {
                if((pDelNotifyList->mpNotifyFunct == UpSem) && (pDelNotifyList->mpData==NULL))
                {

                }
                else
                {
                  barrier();
                  pDelNotifyList->mpNotifyFunct( pDev,
                                           clientID,
                                           pDelNotifyList->mpData );
                }
          }
      }

      // Delete memory
      kfree( pDelNotifyList );
      //DBG( "notify for Client:0x%x, TID 0x%x\n",clientID, transactionID );
      mb();
      return eNotifyListFound;
   }
   else
   {
      DBG( "no one to notify for Client:0x%x, TID 0x%x\n",clientID, transactionID );
      return eNotifyListNotFound;
   }
}

int map_mux_id_to_ipv4(
        sGobiUSBNet *pDev,
        unsigned long arg
        )
{
    int idx;
    int status;
    sQMuxIPTable table;
    if (arg == 0)
    {
        DBG( "Bad IP Table IOCTL buffer\n" );
        return -EINVAL;
    }
    status = copy_from_user( &table, (void*)arg, sizeof(table) );
    if (status != 0)
    {
        DBG( "Unable to copy data from userspace %d\n", status );
        return -EINVAL;
    }

    idx = table.instance - MUX_ID_START;

    /* a little bit difference between sQMuxIPTable and arg, arg passed from user space is mux id
       and ip address, but instance of sQMuxIPTable is the index from 0 to MAX_MUX_NUMBER_SUPPORTED-1,
       so the difference is the MUX_ID_START offset */
    if (
            ( idx >= MAX_MUX_NUMBER_SUPPORTED ) ||
            ( idx < 0 )
       )
    {
        DBG( "invalid indexing muxid to ipv4 table: %d\n", idx);
        return -EINVAL;
    }

    pDev->qMuxIPTable[idx].instance = idx;
    if ( table.ipAddress == 0 )
    {
        pDev->qMuxIPTable[idx].ipAddress = 0;
    }
    else if ( table.ipAddress > 0 )
    {
        pDev->qMuxIPTable[idx].ipAddress= table.ipAddress;
    }

    DBG(" Set IP Address Mux ID : 0x%02x\n", table.instance);
    PrintIPAddr( "Set IP Address : ", table.ipAddress);
    return 0;
}

int iIsZeroIPv6Addr(ipv6_addr *pAddr)
{
   if(pAddr)
   {
      int i = 0;
      for(i=0;i<IPV6_ADDR_LEN;i++)
      {
         if(pAddr->ipv6addr[i]!=0)
            return 0;
      }
   }
   return 1;
}

int map_mux_id_to_ipv6(
   sGobiUSBNet *pDev,
   unsigned long arg )
{
    int idx;
    int status;
    sQMuxIPTable table;
    if (arg == 0)
    {
        DBG( "Bad IP Table IOCTL buffer\n" );
        return -EINVAL;
    }
    status = copy_from_user( &table, (void*)arg, sizeof(table) );
    if (status != 0)
    {
        DBG( "Unable to copy data from userspace %d\n", status );
        return -EINVAL;
    }

    idx = table.instance - MUX_ID_START;

    /* a little bit difference between sQMuxIPTable and arg, arg passed from user space is mux id
       and ip address, but instance of sQMuxIPTable is the index from 0 to MAX_MUX_NUMBER_SUPPORTED-1,
       so the difference is the MUX_ID_START offset */
    if (
            ( idx >= MAX_MUX_NUMBER_SUPPORTED ) ||
            ( idx < 0 )
       )
    {
        DBG( "invalid indexing muxid to ipv6 table: %d\n", idx);
        return -EINVAL;
    }

    pDev->qMuxIPTable[idx].instance = idx;
    if ( iIsZeroIPv6Addr(&table.ipV6Address)==1 )
    {
        memset(&pDev->qMuxIPTable[idx].ipV6Address , 0,sizeof(ipv6_addr));
    }
    else
    {
        memcpy(&pDev->qMuxIPTable[idx].ipV6Address,&table.ipV6Address,sizeof(ipv6_addr));
    }

    NETDBG(" Set IP Address Mux ID : 0x%02x\n", table.instance);
    PrintIPV6Addr( &table.ipV6Address);
    return 0;
}

/*=========================================================================*/
// Internal userspace wrappers
/*=========================================================================*/

/*===========================================================================
METHOD:
   UserspaceunlockedIOCTL (Public Method)

DESCRIPTION:
   Internal wrapper for Userspace IOCTL interface

PARAMETERS
   pFilp        [ I ] - userspace file descriptor
   cmd          [ I ] - IOCTL command
   arg          [ I ] - IOCTL argument

RETURN VALUE:
   long - 0 for success
          Negative errno for failure
===========================================================================*/
long UserspaceunlockedIOCTL(
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg )
{
   int j;
   int result;
   u32 devVIDPID;

   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   unsigned long i_ino = -1;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }
   if(file_inode(pFilp) !=NULL)
   {
      i_ino = file_inode(pFilp)->i_ino;
   }
   else
   {
      DBG("File Inode Null %d\n",__LINE__);
      return -EIO;
   }
   DBG( "%d (%d)i_ino:%lu cmd:%X iCount:%d\n",__LINE__ ,pFilpData->iInfNum,i_ino,cmd,pFilpData->iCount);
   if(cmd==USBDEVFS_RESET)
   {
      DBG( "RESET 0\n" );
      DBG( "%d (%d)i_ino:%lu cmd:%X\n",__LINE__ ,pFilpData->iInfNum,i_ino,cmd);
      return 0;
   }
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid==1)
   {
      DBG( "Clsoing.." );
      gobi_filp_close(pFilp,NULL);
      return -ENXIO;
   }
   if(pFilpData->iIsClosing==1)
   {
       DBG( "Invalid device! Updating f_ops\n" );
       return -ENXIO;

   }
   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }
   pFilpData->pIOCTLTask = current;
   switch (cmd)
   {
      case IOCTL_QMI_GET_SERVICE_FILE:
         DBG( "Setting up QMI for service %lu\n", arg );
         if ((u8)arg == 0)
         {
            DBG( "Cannot use QMICTL from userspace\n" );
            return -EINVAL;
         }

         // Connection is already setup
         if (pFilpData->mClientID !=  0xffff)
         {
            DBG( "Close the current connection before opening a new one\n" );
            return -EBADR;
         }

         pFilpData->iSemID = __LINE__;
         result = GetClientID( pFilpData->mpDev, (u8)arg ,&(pFilpData->mReadSem));
         pFilpData->iSemID = -__LINE__;
         mb();
         if(IsDeviceDisconnect(pFilpData->mpDev))
         {
            DBG( "Device Disconnected!\n" );
            return -ENXIO;
         }
         if (result < 0)
         {
            pFilpData->mDeviceInvalid = 1;
            return result;
         }
         if(pFilpData->iIsClosing ==1)
         {
            return -ENXIO;
         }
         pFilpData->mClientID = (u16)result;
         pFilpData->iReadSyncResult = 0;
         mb();
         return 0;
         break;


      case IOCTL_QMI_GET_DEVICE_VIDPID:
         if (arg == 0)
         {
            DBG( "Bad VIDPID buffer\n" );
            return -EINVAL;
         }

         // Extra verification
         if (pFilpData->mpDev->mpNetDev == 0)
         {
            DBG( "Bad mpNetDev\n" );
            return -ENOMEM;
         }
         if (pFilpData->mpDev->mpNetDev->udev == 0)
         {
            DBG( "Bad udev\n" );
            return -ENOMEM;
         }

         devVIDPID = ((le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idVendor ) << 16)
                     + le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idProduct ) );

         result = copy_to_user( (unsigned int *)arg, &devVIDPID, 4 );
         if (result != 0)
         {
            DBG( "Copy to userspace failure %d\n", result );
         }

         return result;

         break;

      case IOCTL_QMI_GET_DEVICE_MEID:
         if (arg == 0)
         {
            DBG( "Bad MEID buffer\n" );
            return -EINVAL;
         }
         result = copy_to_user( (unsigned int *)arg, &pFilpData->mpDev->mMEID[0], MAX_DEVICE_MEID_SIZE);
         if (result != 0)
         {
            DBG( "Copy to userspace failure %d\n", result );
         }

         return result;

         break;

      case IOCTL_QMI_GET_SVC_VERSION_INFO:
         if (arg == 0)
         {
            DBG( "Bad Svc Info buffer\n" );
            return -EINVAL;
         }
         result = copy_to_user( (unsigned int *)arg, &pFilpData->mpDev->svcVersion[0], MAX_SVC_VERSION_SIZE);
         if (result != 0)
         {
            DBG( "Copy to userspace failure %d\n", result );
         }

         return result;

      case IOCTL_QMI_ADD_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             sMapping *pmap = (sMapping*) arg;

             DBG( "add mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }
             DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             //check for existing map
             for (j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     DBG("mapping already exists at slot #%d\n", j);
                     return -EINVAL;
                 }
             }

             //check if this is a request to redirect all IP traffic to default bearer
             if (UNIQUE_DSCP_ID == pmap->dscp)
             {
                 DBG("set slot (%d) to indicate IP packet redirection is needed\n", MAX_MAP-1);
                 pDev->maps.table[MAX_MAP-1].dscp = UNIQUE_DSCP_ID;
                 pDev->maps.table[MAX_MAP-1].qosId = pmap->qosId;
                 pDev->maps.count++;
                 return 0;
             }

             //find free slot to hold new mapping
             for(j=0;j<MAX_MAP-1;j++)
             {
                 if (pDev->maps.table[j].dscp == 0xff)
                 {
                     pDev->maps.table[j].dscp = pmap->dscp;
                     pDev->maps.table[j].qosId = pmap->qosId;
                     pDev->maps.count++;
                     return 0;
                 }
             }

             DBG("no free mapping slot\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_EDIT_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             sMapping *pmap = (sMapping*) arg;
             DBG( "edit mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }
             DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     pDev->maps.table[j].qosId = pmap->qosId;
                     return 0;
                 }
             }

             DBG("no matching tos for edit mapping\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_READ_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             sMapping *pmap = (sMapping*) arg;
             DBG( "read mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     pmap->qosId = pDev->maps.table[j].qosId;
                     DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

                     result = copy_to_user( (unsigned int *)arg, &pDev->maps.table[j], sizeof(sMapping));
                     if (result != 0)
                     {
                         DBG( "Copy to userspace failure %d\n", result );
                     }

                     return result;
                 }
             }

             DBG("no matching tos for read mapping\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_DEL_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             sMapping *pmap = (sMapping*) arg;
             DBG( "Delete mapping\n" );
             if (arg == 0)
             {
                 DBG( "null pointer\n" );
                 return -EINVAL;
             }
             DBG( "DSCP 0x%x\n", pmap->dscp );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     // delete mapping table entry
                     memset(&pDev->maps.table[j], 0xff, sizeof(pDev->maps.table[0]));
                     if (pDev->maps.count) pDev->maps.count--;
                     return 0;
                 }
             }

             DBG("no matching mapping slot\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_CLR_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             DBG( "Clear mapping\n" );
             memset(pDev->maps.table, 0xff, sizeof(pDev->maps.table));
             pDev->maps.count = 0;
             return 0;
         }
         break;

#ifdef QOS_SIMULATE
      case IOCTL_QMI_QOS_SIMULATE:
         {
             int result;
             u8 supported = (u8)-1;
             DBG( "simulate indication\n" );
             u8 qos_support_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x27,0x00,0x09,0x00,0x01,0x01,0x00,0x01,0x10,0x02,0x00,0x01,0x80
             };
             u8 qos_flow_activate_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x01,0x01
             };
             u8 qos_flow_suspend_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x02,0x02
             };
             u8 qos_flow_gone_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x03,0x03
             };
             result = QMIQOSEventResp( qos_support_ind,
                     sizeof(qos_support_ind));
             result = QMIQOSEventResp( qos_flow_activate_ind,
                     sizeof(qos_flow_activate_ind));
             result = QMIQOSEventResp( qos_flow_suspend_ind,
                     sizeof(qos_flow_suspend_ind));
             result = QMIQOSEventResp( qos_flow_gone_ind,
                     sizeof(qos_flow_gone_ind));
             return 0;
         }
         break;
#endif

      case IOCTL_QMI_GET_TX_Q_LEN:
         {

             sGobiUSBNet * pDev = pFilpData->mpDev;

             if (arg == 0)
             {
                 DBG( "Bad Tx Queue buffer\n" );
                 return -EINVAL;
             }

             // Extra verification
             if (pFilpData->mpDev->mpNetDev == 0)
             {
                 DBG( "Bad mpNetDev\n" );
                 return -ENOMEM;
             }
             if (pFilpData->mpDev->mpNetDev->udev == 0)
             {
                 DBG( "Bad udev\n" );
                 return -ENOMEM;
             }

             result = copy_to_user( (unsigned int *)arg, &pDev->tx_qlen, sizeof(pDev->tx_qlen) );
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }

             return result;
         }

         break;

      case IOCTL_QMI_DUMP_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             DBG( "dump mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }

             result = copy_to_user( (unsigned int *)arg, &pDev->maps.table[0], sizeof(pDev->maps.table));
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }
             return result;
         }

      case IOCTL_QMI_GET_USBNET_STATS:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             sNetStats netStats;
             #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 4,12,0 ))
             struct rtnl_link_stats64 Stats64;
             struct rtnl_link_stats64 *pStats = &Stats64;
             memset(&Stats64,0,sizeof(Stats64));
             usbnet_get_stats64(pDev->mpNetDev->net,pStats);
             #else
             struct net_device_stats * pStats = &(pDev->mpNetDev->net->stats);
             #endif

             if (arg == 0)
             {
                 DBG( "Bad usbnet statistic buffer\n" );
                 return -EINVAL;
             }

             // Extra verification
             if (pFilpData->mpDev->mpNetDev == 0)
             {
                 DBG( "Bad mpNetDev\n" );
                 return -ENOMEM;
             }

             memset(&netStats,0,sizeof(netStats));
             /* copy the value from struct net_device_stats to struct sNetStats */
             netStats.rx_packets = pStats->rx_packets;
             netStats.tx_packets = pStats->tx_packets;
             netStats.rx_bytes = pStats->rx_bytes;
             netStats.tx_bytes = pStats->tx_bytes;
             netStats.rx_errors = pStats->rx_errors;
             netStats.tx_errors = pStats->tx_errors;
             netStats.rx_overflows = pStats->rx_fifo_errors;
             netStats.tx_overflows = pStats->tx_fifo_errors;

             result = copy_to_user( (unsigned int *)arg, &netStats, sizeof(sNetStats) );
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }

             return result;
         }

         break;
         case IOCTL_QMI_SET_DEVICE_MTU:
         {
             sGobiUSBNet *pDev = pFilpData->mpDev;
             // struct usbnet * pNet = netdev_priv( pDev->mpNetDev->net );
             int iArgp = (int)arg;
             if (iArgp <= 0)
             {
                 DBG( "Bad MTU buffer\n" );
                 return -EINVAL;
             }
             DBG( "new mtu :%d ,qcqmi:%d\n",iArgp,(int)pDev->mQMIDev.qcqmi );
             pDev->mtu = iArgp;
             usbnet_change_mtu(pDev->mpNetDev->net ,pDev->mtu);
         }
         return 0;
         case IOCTL_QMI_GET_QMAP_SUPPORT:
         {
            sGobiUSBNet *pDev = pFilpData->mpDev;

            if (arg == 0)
            {
               DBG( "Bad QMAP IOCTL buffer\n" );
               return -EINVAL;
            }
            result = copy_to_user( (unsigned int *)arg, &pDev->nRmnet, sizeof(pDev->nRmnet));
            if (result != 0)
            {
               DBG( "Copy to userspace failure %d\n", result );
            }
            DBG( "nRmnet:%d\n",(int)pDev->nRmnet);
            return result;
          }
         case IOCTL_QMI_SET_IP_ADDRESS:
            return map_mux_id_to_ipv4(pFilpData->mpDev, arg);
         case IOCTL_QMI_SET_IPV6_ADDRESS:
            return map_mux_id_to_ipv6(pFilpData->mpDev, arg);
         case IOCTL_QMI_GET_IPALIAS_MODE:
            {
                sGobiUSBNet *pDev = pFilpData->mpDev;
                result = -1;
                if (arg == 0)
                {
                   DBG( "Bad GET IP ALIAS IOCTL buffer\n" );
                   return -EINVAL;
                }
                if(pDev->iQMUXEnable)
                {
                    result = copy_to_user( (unsigned int *)arg, &pDev->iIPAlias, sizeof(pDev->iIPAlias));
                    if (result != 0)
                    {
                       DBG( "Copy to userspace failure %d\n", result );
                    }
                    DBG( "iIPAlias:%d\n",(int)pDev->iIPAlias);
                }
                return result;
            }
      default:
         return -EBADRQC;
   }
}

/*=========================================================================*/
// Userspace wrappers
/*=========================================================================*/

/*===========================================================================
METHOD:
   UserspaceOpen (Public Method)

DESCRIPTION:
   Userspace open
      IOCTL must be called before reads or writes

PARAMETERS
   pInode       [ I ] - kernel file descriptor
   pFilp        [ I ] - userspace file descriptor

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceOpen(
   struct inode *         pInode,
   struct file *          pFilp )
{
   sQMIFilpStorage * pFilpData;
   sQMIDev * pQMIDev = NULL;
   sGobiUSBNet * pDev = NULL;
   static int count = 0;
   //DBG( "\n" );
   // Optain device pointer from pInode
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   pQMIDev = container_of( pInode->i_cdev,
                                     sQMIDev,
                                     mCdev );
   pDev = container_of( pQMIDev,
                                    sGobiUSBNet,
                                    mQMIDev );
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   pFilp->private_data = NULL;

   if (IsDeviceValid( pDev ) == false)
   {
      printk( KERN_INFO "Invalid device\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if(pDev->mbUnload)
   {
       printk( KERN_INFO "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }
   if(pDev->iIsClosing)
   {
     printk( KERN_INFO "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }
   // Setup data in pFilp->private_data
   pFilp->private_data = kmalloc( sizeof( sQMIFilpStorage ), GOBI_GFP_KERNEL );
   if (pFilp->private_data == NULL)
   {
      printk( KERN_INFO "Mem error\n" );
      return -ENOMEM;
   }

   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   pFilpData->mClientID =  0xffff;
   pFilpData->mDeviceInvalid = 0;
   pFilpData->mpDev = pDev;
   pFilpData->iSemID = -1;
   pFilpData->iIsClosing = 0;
   pFilpData->iReadSyncResult = -1;
   pFilpData->iInfNum = pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber;
   pFilpData->iCount = count++;
   pFilpData->pOpenTask = current;
   pFilpData->pReadTask = current;
   pFilpData->pWriteTask = current;
   pFilpData->pIOCTLTask = current;
   sema_init(&pFilpData->mReadSem , SEMI_INIT_DEFAULT_VALUE );
   mb();
   if(file_inode(pFilp)!=NULL)
   {
      DBG( "%d CID:0x%x, (%d)i_ino:%lu icount:%d\n",__LINE__ ,pFilpData->mClientID,pFilpData->iInfNum,file_inode(pFilp)->i_ino,pFilpData->iCount);
   }
   else
   {
      DBG( "%d CID:0x%x, (%d)i_ino:NULL icount:%d\n",__LINE__ ,pFilpData->mClientID,pFilpData->iInfNum,pFilpData->iCount);
   }
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceIOCTL (Public Method)

DESCRIPTION:
   Userspace IOCTL functions

PARAMETERS
   pUnusedInode [ I ] - (unused) kernel file descriptor
   pFilp        [ I ] - userspace file descriptor
   cmd          [ I ] - IOCTL command
   arg          [ I ] - IOCTL argument

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceIOCTL(
   struct inode *    pUnusedInode,
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg )
{
   int ret = 0;
   unsigned long  i_ino = -1;
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   // call the internal wrapper function

   if(file_inode(pFilp)!=NULL)
      i_ino = file_inode(pFilp)->i_ino;
   DBG("%d i_ino:%lu",__LINE__,i_ino);
   ret = (int)UserspaceunlockedIOCTL( pFilp, cmd, arg );
   if(file_inode(pFilp)!=NULL)
   DBG("%d i_ino:%lu ret:%d",__LINE__,i_ino,ret);
   return ret;
}

/*===========================================================================
METHOD:
   UserspaceClose (Public Method)

DESCRIPTION:
   Userspace close
      Release client ID and free memory

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   unusedFileTable [ I ] - (unused) file table

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceClose(
   struct file *       pFilp,
   fl_owner_t          unusedFileTable )
{
   sQMIFilpStorage * pFilpData = NULL;
   u16 u16ClientID = 0xFFFF;
   unsigned long i_ino = -1;
   int iInfNum, iCount;
   struct task_struct *pOpenTask = NULL;
   struct task_struct *pReadTask = NULL;
   struct task_struct *pWriteTask = NULL;
   struct task_struct *pIOCTLTask = NULL;
   pid_t pid = -1;
   int iTimeout = 0;
   int iFile_count = 0;
   long refcnt = 0;
   mb();
   if(pFilp ==NULL)
   {
      printk( KERN_INFO "bad file data\n" );
      return -EBADF;
   }
   refcnt = atomic_long_read(&pFilp->f_count);
   if (refcnt > 1)
   {
      if(current->exit_signal!=SIGCHLD)
      {
          if((IsOtherTaskUsingFilp(pFilp) ==1)&&
                (IsOpenTaskIsCurrent(pFilp)||
                 IsCurrentTaskExit()))
          {
             DBG( "f_count %ld - ignoring close\n", refcnt);
             return -EBUSY;
          }
      }
      else
      {
         DBG( "SIGCHLD %ld \n", refcnt);
         if(!IsOpenTaskIsCurrent(pFilp))
         {
            DBG( "f_count %ld - ignoring close\n", refcnt);
            return -EBUSY;
         }
      }
      DBG("f_count %ld - close %d %d\n", refcnt,IsOtherTaskUsingFilp(pFilp),IsOpenTaskIsCurrent(pFilp));
   }
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      printk( KERN_INFO "bad file data private \n" );
      return -EBADF;
   }
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      printk( KERN_INFO "%s Invalid device! Updating f_ops\n",__FUNCTION__ );
   }
   wait_interrupt();
   if(pFilpData->iIsClosing==1)
   {
      u16ClientID = pFilpData->mClientID;
      iInfNum = pFilpData->iInfNum;
      iCount = pFilpData->iCount;
      iFile_count = file_count(pFilp);
      pOpenTask = pFilpData->pOpenTask;
      pReadTask = pFilpData->pReadTask;
      pWriteTask = pFilpData->pWriteTask;
      pIOCTLTask = pFilpData->pIOCTLTask;
      if(file_inode(pFilp)!=NULL)
      {
         i_ino = file_inode(pFilp)->i_ino;
      }
      else
      {
         DBG("File Inode Null %d\n",__LINE__);
         return 0;
      }
      if(pFilpData->pOpenTask!=NULL)
      {
         pid = pFilpData->pOpenTask->pid;
      }
      DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d pid:%d, iFile_count:%d\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount,pid,iFile_count);
      if(pWriteTask!=pOpenTask)
      {
         wakeup_target_process(pWriteTask);
         wait_interrupt();
      }
      if(pReadTask!=pOpenTask)
      {
         wakeup_target_process(pReadTask);
         wait_interrupt();
      }
      if(pIOCTLTask!=pOpenTask)
      {
         wakeup_target_process(pIOCTLTask);
         wait_interrupt();
      }
      if(pOpenTask!=current)
      {
         wakeup_target_process(pOpenTask);
         wait_interrupt();
         do
         {
            wait_ms(100);
            mb();
            if(pFilp==NULL)
            {
               return 0;
            }
            if(pFilp->private_data ==NULL)
            {
               return 0;
            }
            if(iTimeout++>5)
            {
               DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d pid:%d timeout\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount,pid);
               break;
            }
            if(iFile_count!=file_count(pFilp))
            {
               DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d pid:%d count:%d/%d\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount,pid,iFile_count,(int)file_count(pFilp));
               break;

            }
            else if(file_count(pFilp)==0)
            {
               DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d pid:%d count==0\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount,pid);
               break;
            }

         }while(pFilpData!=NULL);
         DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d pid:%d return\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount,pid);
      }
   }

   pFilpData->iIsClosing = 1;
   pFilpData->mDeviceInvalid = 1;
   mb();
   u16ClientID = pFilpData->mClientID;
   iInfNum = pFilpData->iInfNum;
   iCount = pFilpData->iCount;
   pOpenTask = pFilpData->pOpenTask;
   pReadTask = pFilpData->pReadTask;
   pWriteTask = pFilpData->pWriteTask;
   pIOCTLTask = pFilpData->pIOCTLTask;
   if(file_inode(pFilp)!=NULL)
   {
      i_ino = file_inode(pFilp)->i_ino;
   }
   else
   {
      DBG("File Inode Null %d\n",__LINE__);
      return 0;
   }
   GobiSyncRcu();

   DBG( "%d CID:0x%x, (%d)i_ino:%lu, icount:%d\n",__LINE__ ,u16ClientID,iInfNum,i_ino,iCount);
   if( ((pFilpData->iSemID > 0) && (pFilpData->mClientID != 0xffff) ) ||
       ((pFilpData->pOpenTask != pFilpData->pIOCTLTask )&&(pFilpData->iSemID > 0)) )
   {
      int iRetry = 0;
      int iReturn = 0;
      int iLockCount = 0;
      if(pFilpData->mpDev->mbUnload)
      {
         iReturn = -EAGAIN;
      }

      while(pFilpData->iSemID > 0)
      {
          GobiSyncRcu();
          if(!pOpenTask)
          {
             break;
          }
          if((signal_pending(current))||(signal_pending(pOpenTask)))
          {
            DBG( "%d wait next\n",__LINE__ );
            break;
          }
          if(LocalClientMemLockSpinIsLock(pFilpData->mpDev)!=0)
          {
             if(pFilpData->mpDev->mQMIDev.pTask!=NULL)
             {
                wakeup_target_process(pFilpData->mpDev->mQMIDev.pTask);
                wait_interrupt();
             }
             if(iLockCount++ > 10)
             {
                DBG("locked!");
                return -EAGAIN;
             }
             else
             {
                gobi_flush_work();
                wait_ms(100);
                continue;
             }
          }
          iLockCount = 0;
          mb();
          if((pFilpData==NULL) || (pFilp==NULL))
          {
            iReturn = 0;
            break;
          }
          barrier();
          if(!down_trylock(&(pFilpData->mReadSem)))
          {
              DBG("NOT locked : %d",pFilpData->iSemID);
              barrier();
              up(&(pFilpData->mReadSem));
              pFilpData->iSemID = -1;
              mb();
              break;
          }
          barrier();
          up(&(pFilpData->mReadSem));
          if((signal_pending(current))||(signal_pending(pOpenTask)))
          {
            DBG( "%d wait next\n",__LINE__ );
            break;
          }
          if(pWriteTask!=NULL)
          {
              wakeup_target_process(pWriteTask);
          }
          if(pReadTask!=NULL)
          {
             wakeup_target_process(pReadTask);
          }
          wakeup_inode_process(pFilp,pOpenTask);
          if((pFilpData==NULL) || (pFilp==NULL))
          {
             iReturn = 0;
             DBG( "%d NULL\n",__LINE__ );
             break;
          }
          if(iRetry++>10)
          {
              iReturn = -EAGAIN;
              printk("Timeout!");
              return iReturn;
          }
          wait_ms(500);
      };
      GobiSyncRcu();
   }

   if(pFilpData->mpDev->mbUnload)
   {
      if(pOpenTask==current)
      {
         wait_interrupt();
         kfree( pFilp->private_data );
         pFilp->private_data = NULL;
         GobiSyncRcu();
         mb();
      }
      return 0;
   }

   if (pFilpData->mpDev->mbUnload > eStatUnloading)
   {
      if(pOpenTask==current)
      {
         wait_interrupt();
         kfree( pFilp->private_data );
         pFilp->private_data = NULL;
         GobiSyncRcu();
         mb();
      }
      return 0;
   }

   DBG( "CID 0x%04X\n", u16ClientID );

   if (pFilpData->mClientID !=  0xffff)
   {
     pFilpData->iSemID = __LINE__;
     if ( (pFilpData->iReadSyncResult>=0) &&
         (pFilpData->mpDev->mbUnload < eStatUnloading) &&
         !IsDeviceDisconnect(pFilpData->mpDev))
     {
          DBG( "Release CID 0x%04X\n", u16ClientID );
          ReleaseClientID( pFilpData->mpDev,
                      pFilpData->mClientID);
     }
     else
     {
         unsigned long flags;
         flags = LocalClientMemLockSpinLockIRQSave( pFilpData->mpDev , __LINE__);
         RemoveAndPopNotifyList(pFilpData->mpDev,
                      pFilpData->mClientID,0,eClearAndReleaseCID);
         LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);

     }
     pFilpData->iSemID = -__LINE__;
     mb();
     pFilpData->mClientID =  0xffff;
   }
   wait_interrupt();
   if(pFilp->private_data)
      kfree( pFilp->private_data );

   // Disable pFilpData so they can't keep sending read or write
   //    should this function hang
   // Note: memory pointer is still saved in pFilpData to be deleted later
   pFilp->private_data = NULL;
   GobiSyncRcu();
   mb();
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceRead (Public Method)

DESCRIPTION:
   Userspace read (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - read buffer
   size            [ I ] - size of read buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceRead(
   struct file *          pFilp,
   char __user *          pBuf,
   size_t                 size,
   loff_t *               pUnusedFpos )
{
   int result = -1;
   void * pReadData = NULL;
   void * pSmallReadData = NULL;
   sQMIFilpStorage * pFilpData = NULL;
   int iCount = -1;
   //DBG("\n");
   if(pFilp==NULL)
   {
       return -EBADF;
   }
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }

   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid)
   {
      DBG( "mDeviceInvalid\n");
      return -ENXIO;
   }

   if (pFilpData->mClientID ==  0xffff)
   {
      DBG( "Client ID must be set before reading 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   if(pFilpData->iIsClosing==1)
   {
      DBG( "filep Clsoing.." );
      return -ENXIO;
   }

   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }
   iCount = pFilpData->iCount;
   pFilpData->pReadTask = current;

   pFilpData->iSemID = __LINE__;
   // Perform synchronous read
   result = ReadSync( pFilpData->mpDev,
                      &pReadData,
                      pFilpData->mClientID,
                      0,
                      &(pFilpData->iSemID),&(pFilpData->mReadSem),&(pFilpData->iIsClosing));
   if(pFilp==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if(pFilpData==NULL)
   {
      return -ENXIO;
   }
   pFilpData->iSemID = -__LINE__;
   mb();
   GobiSyncRcu();
   if(result<0)
   {
      if(file_inode(pFilp)!=NULL)
      {
      DBG("Read Error!CID:0x%04x, (%d)i_ino:%lu iCount:%d\n",pFilpData->mClientID,pFilpData->iInfNum, file_inode(pFilp)->i_ino,iCount);
      }
      else
      {
         DBG("Read Error!CID:0x%04x, (%d)i_ino:NULL iCount:%d\n",pFilpData->mClientID,pFilpData->iInfNum,iCount);
         DBG("File Inode Null %d\n",__LINE__);
         return -ENXIO;
      }
   }
   else
   {
      PrintHex(pReadData,result);
   }
   if(pFilp==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if(pFilpData->mpDev==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if((pFilpData->mpDev->mbUnload)||(pFilpData->iIsClosing))
   {
      return -ENXIO;
   }
   if (result <= 0)
   {
      #ifdef CONFIG_PM
      if(bIsSuspend(pFilpData->mpDev))
      {
         DBG("SUSPEND\n");
      }
      #endif
      if(result == -EINTR)
      {
         DBG("RETRY\n");
      }
      return result;
   }

   // Discard QMUX header
   result -= QMUXHeaderSize();
   pSmallReadData = pReadData + QMUXHeaderSize();

   if (result > size)
   {
      DBG( "Read data is too large for amount user has requested\n" );
      if(pReadData)
      kfree( pReadData );
      return -EOVERFLOW;
   }

   DBG(  "pBuf = 0x%p pSmallReadData = 0x%p, result = %d",
         pBuf, pSmallReadData, result );

   if (copy_to_user( pBuf, pSmallReadData, result ) != 0)
   {
      DBG( "Error copying read data to user\n" );
      result = -EFAULT;
   }
   pSmallReadData = NULL;
   // Reader is responsible for freeing read buffer
   kfree( pReadData );

   return result;
}

/*===========================================================================
METHOD:
   UserspaceWrite (Public Method)

DESCRIPTION:
   Userspace write (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - write buffer
   size            [ I ] - size of write buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceWrite(
   struct file *        pFilp,
   const char __user *  pBuf,
   size_t               size,
   loff_t *             pUnusedFpos )
{
   int status;
   void * pWriteBuffer;
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   pFilpData->pWriteTask = current;

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid)
   {
      DBG( "mDeviceInvalid\n");
      gobi_filp_close(pFilp,NULL);
      return -ENXIO;
   }

   if (pFilpData->mClientID ==  0xffff)
   {
      DBG( "Client ID must be set before writing 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   if(pFilpData->iIsClosing==1)
   {
      DBG( "Filep Clsoing.." );
      return -ENXIO;
   }
   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }

   // Copy data from user to kernel space
   pWriteBuffer = kmalloc( size + QMUXHeaderSize(), GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }
   status = copy_from_user( pWriteBuffer + QMUXHeaderSize(), pBuf, size );
   if (status != 0)
   {
      DBG( "Unable to copy data from userspace %d\n", status );
      kfree( pWriteBuffer );
      return status;
   }

   status = WriteSync( pFilpData->mpDev,
                       pWriteBuffer,
                       size + QMUXHeaderSize(),
                       pFilpData->mClientID );

   kfree( pWriteBuffer );
   if(pFilpData!=NULL)
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   // On success, return requested size, not full QMI reqest size
   if (status == size + QMUXHeaderSize())
   {
      return size;
   }
   else
   {
      pFilpData->mDeviceInvalid = 1;
      if(status<0)
      {
         pFilpData->iIsClosing=1;
         return -ENXIO;
      }
      return status;
   }
}

/*===========================================================================
METHOD:
   UserspacePoll (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pFilp              [ I ] - userspace file descriptor
   pPollTable         [I/O] - Wait object to notify the kernel when data
                              is ready

RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
unsigned int UserspacePoll(
   struct file *                  pFilp,
   struct poll_table_struct *     pPollTable )
{
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   sClientMemList * pClientMem;
   unsigned long flags;

   // Always ready to write
   unsigned int status = POLLOUT | POLLWRNORM;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return POLLERR;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return POLLERR;
   }

   if (pFilpData->mClientID ==  0xffff)
   {
      DBG( "Client ID must be set before polling 0x%04X\n",
           pFilpData->mClientID );
      return POLLERR;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pFilpData->mpDev , __LINE__);

   // Get this client's memory location
   pClientMem = FindClientMem( pFilpData->mpDev,
                              pFilpData->mClientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           pFilpData->mClientID );

      LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);
      return POLLERR;
   }
   if (ClientTransactionIDExist(pFilpData->mpDev,
                           pClientMem->mClientID,
                           0)==0)
   {
      if (AddToNotifyList( pFilpData->mpDev,
                           pClientMem->mClientID,
                           0,
                           NULL,
                           NULL ) == false)
      {
         LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);
         return POLLERR;
      }
   }
   else
   {
       DBG("SKIP AddToNotifyList\n");
   }

   poll_wait( pFilp, &pClientMem->mWaitQueue, pPollTable );

   if (pClientMem->mpList != NULL)
   {
      status |= POLLIN | POLLRDNORM;
   }

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);

   // Always ready to write
   return (status | POLLOUT | POLLWRNORM);
}

void gobi_try_wake_up_process(struct task_struct *pTask)
{
   int count = 0;
   if(pTask==NULL)
      return ;
   if(pTask==current)
      return ;
   if(wait_preempt()==0)
      return ;
   #if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
   while(raw_spin_is_locked(&pTask->pi_lock))
   #endif
   {
      if(count++>10)
      {
         DBG( "task Locked %d\n",__LINE__ );
         return ;
      }
      msleep_interruptible(100);
      mb();
      if(pTask==NULL)
            return;
      if(signal_pending(pTask))
      return ;
   }
   DBG( "%d pid:%d\n",__LINE__,pTask->pid );
   wake_up_process(pTask);
}

int wakeup_inode_process(struct file *pFilp,struct task_struct * pTask)
{
   struct task_struct *pEachTask=NULL;
   #if _SIG_LOCK_
   struct sighand_struct *sighand;
   #endif
   unsigned long i_no =-1;
   int iFound =0;
   struct list_head *list;
   mb();
   if(pFilp==NULL)
      return 0;
   if(file_inode(pFilp)!=NULL)
   {
      i_no = file_inode(pFilp)->i_ino;
   }
   else
   {
      DBG("File Inode Null %d\n",__LINE__);
      return 0;
   }
   DBG( "%d i_ino:%lu\n",__LINE__ ,i_no);
   if(!pTask)
   {
      DBG( "pTask NULL\n");
      return 0;
   }
   if(signal_pending(pTask))
   {
      DBG("signal_pending %d\n",__LINE__);
      return 0;
   }
   if(pTask)
   {
         #if _SIG_LOCK_
         if(lockdep_tasklist_lock_is_held())
         {
            DBG( "lockdep_tasklist_lock_is_held %d\n",__LINE__ );
         }
         sighand = rcu_dereference_check(pTask->sighand,
         lockdep_tasklist_lock_is_held());
         if(sighand)
         spin_lock_irq(&sighand->siglock);
         else
         {
            DBG( "spin_lock %d\n",__LINE__ );
            return 0;
         }
         #endif
         if(pTask->state==TASK_STOPPED)
         {
            if (unlikely(pTask->signal->notify_count < 0))
            {
               int count = 0;
               if(pTask!=current)
               {
                  gobi_try_wake_up_process(pTask);
               }
               list_for_each(list, &pTask->children)
               {
                  struct task_struct *task;
                  task = list_entry(list, struct task_struct, sibling);
                  if((task!=NULL)&&(task!=current))
                  {
                     gobi_try_wake_up_process(task);
                  }
               }
               count++;
               do{
                  wait_ms(100);
                  mb();
                  if(count++>10)
                     break;
               }while(file_count(pFilp)>0);
            }
         }
         else
         {
            DBG( "not wakeup %d\n",__LINE__ );
         }
         #if _SIG_LOCK_
         if(sighand)
         spin_unlock_irq(&sighand->siglock);
         #endif
         return 0;

   }

   if(pEachTask!=NULL)
   {
      int count =0;
      if(pEachTask!=current)
      gobi_try_wake_up_process(pEachTask);
      do{
         wait_ms(100);
         mb();
         if(count++>10)
            break;
      }while(file_count(pFilp)>0);
   }

   if(iFound==1)
   {
      DBG( "%d i_ino:%lu\n",__LINE__ ,i_no);
      return 0;
   }
   barrier();
   for_each_process( pEachTask )
   {
      int count = 0;
      struct fdtable * pFDT;
      if (pEachTask == NULL || pEachTask->files == NULL)
      {
         // Some tasks may not have files (e.g. Xsession)
            continue;
      }
      pFDT = files_fdtable( pEachTask->files );
      for (count = 0; count < pFDT->max_fds; count++)
      {
         if (pFDT->fd[count] == pFilp)
         {
            iFound = 1;
            #if _SIG_LOCK_
            if(lockdep_tasklist_lock_is_held())
            {
              DBG( "lockdep_tasklist_lock_is_held %d\n",__LINE__ );
            }
            sighand = rcu_dereference_check(pEachTask->sighand,
            lockdep_tasklist_lock_is_held());
            if(sighand)
            spin_lock_irq(&sighand->siglock);
            else
            {
              return 0;
            }
            #endif
            if(pEachTask->state==TASK_STOPPED)//(pEachTask->state != TASK_STOPPED)
            {
              if (unlikely(pEachTask->signal->notify_count < 0))
              {
                 int count =0;
                 if(pEachTask!=current)
                 gobi_try_wake_up_process(pEachTask);
                 list_for_each(list, &pEachTask->children)
                 {
                    struct task_struct *task;
                    task = list_entry(list, struct task_struct, sibling);
                    if((task!=NULL)&&(task!=current))
                    {
                       gobi_try_wake_up_process(task);
                    }
                 }
                 do{
                    wait_ms(100);
                    mb();
                    if(count++>10)
                       break;
                 }while(file_count(pFilp)>0);
                 mb();
              }
            }
            else
            {
              DBG( "not wakeup %d\n",__LINE__ );
            }
            #if _SIG_LOCK_
            if(sighand)
            spin_unlock_irq(&sighand->siglock);
            #endif
            count = pFDT->max_fds;
         }
      }
   }
   return 0;
}

/*===========================================================================
METHOD:
   UserSpaceLock (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pFilp      [ I ] - The file to apply the lock to
   cmd        [ I ] - type of locking operation (F_SETLK, F_GETLK, etc.)
   fl         [I/O] - The lock to be applied

RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
int UserSpaceLock(struct file *filp, int cmd, struct file_lock *fl)
{
   if((filp!=NULL) && (fl!=NULL))
   {
      #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,9,0 ))
      if(file_inode(filp)!=NULL)
      {
         return posix_lock_file(filp, fl, NULL);
      }
      #else
      return posix_lock_file(filp, fl, NULL);
      #endif
   }
   return -ENOLCK;
}

/*===========================================================================
METHOD:
   UserspaceRelease (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pInode              [I/O] - userspace file descriptor
   pFilp               [I/O] - userspace file descriptor

RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
int UserspaceRelease(struct inode *pInode, struct file *pFilp)
{
   sQMIFilpStorage * pFilpData = NULL;
   mb();
   if(pFilp==NULL)
   {
      return 0;
   }

   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if(pFilpData!=NULL)
   {
     if(pFilp->private_data)
     {
       kfree(pFilp->private_data);
       pFilp->private_data = NULL;
     }
   }
   mb();
   return 0;
}
/*=========================================================================*/
// Initializer and destructor
/*=========================================================================*/
int QMICTLSyncProc(sGobiUSBNet *pDev)
{
   void *pWriteBuffer;
   void *pReadBuffer;
   int result;
   u16 writeBufferSize;
   u8 transactionID;
   struct semaphore readSem;
   unsigned long flags;
   u16 readBufferSize;

   RETURN_WHEN_DEVICE_ERR(pDev);

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   writeBufferSize= QMICTLSyncReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   transactionID = QMIXactionIDGet(pDev);

   /* send a QMI_CTL_SYNC_REQ (0x0027) */
   result = QMICTLSyncReq( pWriteBuffer,
                           writeBufferSize,
                           transactionID );
   if (result < 0)
   {
      if(pWriteBuffer)
      {
         kfree( pWriteBuffer );
         pWriteBuffer = NULL;
      }
      return result;
   }

   result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);
   if(result == 0)
   {
      result = WriteSync( pDev,
                    pWriteBuffer,
                    writeBufferSize,
                    QMICTL );
   }
   if(pWriteBuffer)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
   }
   if(result<0)
   {
      return result;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = QMICTLSyncResp(pReadBuffer,
                                               readBufferSize);
         // We don't care about the result
         if(pReadBuffer)
         kfree( pReadBuffer );
      }
      else
      {
         // Read mismatch/failure, unlock and continue
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
         spin_unlock_irq(&(pDev->notif_lock));
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      }
   }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
      spin_unlock_irq(&(pDev->notif_lock));
      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      result = -1;
   }

   if (result < 0) /* need to re-sync */
   {
      DBG( "sync response error code %d\n", result );
      /* start timer and wait for the response */
      /* process response */
      return result;
   }

   // Success
   return 0;
}

static int
qmi_show(struct seq_file *m, void *v)
{
    sGobiUSBNet * pDev = (sGobiUSBNet*) m->private;
    seq_printf(m, "readTimeoutCnt %d\n", pDev->readTimeoutCnt);
    seq_printf(m, "writeTimeoutCnt %d\n", pDev->writeTimeoutCnt);
    return 0;
}

static int
qmi_open(struct inode *inode, struct file *file)
{
    char *data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,10,0 ))
    data=PDE_DATA(inode);
#else
    data=PDE(inode)->data;
#endif

    return single_open(file, qmi_show, data);
}

static const struct file_operations proc_fops = {
    .owner      = THIS_MODULE,
    .open       = qmi_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/*===========================================================================
METHOD:
   RegisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device initialization function

PARAMETERS:
   pDev     [ I ] - Device specific memory
   is9x15   [ I ]

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int RegisterQMIDevice( sGobiUSBNet * pDev, int is9x15 )
{
   char qcqmi_dev_name[10];
   int i;
   int result;
   dev_t devno;
   pDev->mQMIDev.proc_file = NULL;
   if (pDev->mQMIDev.mbCdevIsInitialized == true)
   {
      // Should never happen, but always better to check
      DBG( "device already exists\n" );
      return -EEXIST;
   }

   pDev->mbQMIValid = true;
   pDev->mbUnload = eStatRegister;
   pDev->mReleaseClientIDFail=0;
   pDev->readTimeoutCnt = 0;
   pDev->writeTimeoutCnt = 0;
   pDev->mtu = 0;
   pDev->iShutdown_write_sem = -1;
   pDev->iShutdown_read_sem = -1;
   init_rwsem(&pDev->shutdown_rwsem);
   InitSemID(pDev);

   i=0;
   do
   {
      // Set up for QMICTL
      //    (does not send QMI message, just sets up memory)
      if(gobi_kthread_should_stop())
      {
         return -1;
      }
      RETURN_WHEN_DEVICE_ERR(pDev);
      result = GetClientID( pDev, QMICTL ,NULL);

      if(gobi_kthread_should_stop())
      {
         return -1;
      }

      RETURN_WHEN_DEVICE_ERR(pDev);
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            pDev->mbQMIValid = false;
            return result;
         }
      }
   }while(result!=0);
   atomic_set( &pDev->mQMIDev.mQMICTLTransactionID, 1 );

   // Start Async reading
   result = StartRead( pDev );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }

   // Send SetControlLineState request (USB_CDC)
   //   Required for Autoconnect and 9x30 to wake up
   result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             CONTROL_DTR,
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
   if (result < 0)
   {
      DBG( "Bad SetControlLineState status %d\n", result );
      return result;
   }


   // Device is not ready for QMI connections right away
   //   Wait up to 30 seconds before failing
   result = QMIReady( pDev, 30000 );
   if(result==-1)
   {
      pDev->mbUnload = eStatUnloading;
      return -EFAULT;
   }
   else if (result == false)
   {
      DBG( "Device unresponsive to QMI\n" );
      return -ETIMEDOUT;
   }
   RETURN_WHEN_DEVICE_ERR(pDev);
   // Initiate QMI CTL Sync Procedure
   DBG( "Sending QMI CTL Sync Request\n" );
   i=0;
   do
   {
      result = QMICTLSyncProc(pDev);
      RETURN_WHEN_DEVICE_ERR(pDev);
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            DBG( "QMI CTL Sync Procedure Error\n" );
            return result;
         }
      }
      else
      {
         DBG( "QMI CTL Sync Procedure Successful\n" );
      }
   }while(result!=0);
   if(iTEEnable<eSKIP_TE_FLOW_CONTROL_TLV)
   {
      iTEEnable = eSKIP_TE_FLOW_CONTROL_TLV;
   }
   // Setup Data Format
   if (is9x15)
   {
      i=0;
      if (pDev->iQMUXEnable!=0)
      {
         pDev->iDataMode = eDataMode_RAWIP;
      }
      else if(iRAWIPEnable==0)
      {
         pDev->iDataMode = eDataMode_Ethernet;
      }
      else
      {
         pDev->iDataMode = eDataMode_RAWIP;
      }

      do
      {
         if(iTEEnable!=eSKIP_TE_FLOW_CONTROL_TLV)//TE_FLOW_CONTROL
         {
            result = QMIWDASetDataFormat (pDev, iTEEnable,pDev->iQMUXEnable);
         }
         else
         {
            result = QMIWDASetDataFormat (pDev, eSKIP_TE_FLOW_CONTROL_TLV,pDev->iQMUXEnable);
         }
         RETURN_WHEN_DEVICE_ERR(pDev);
         if(i++>MAX_RETRY)
         {
            if(pDev->iDataMode==eDataMode_Ethernet)
            {
               pDev->iDataMode=eDataMode_RAWIP;
               i = 0;
            }
            else
            {
               break;
            }
         }
      }while(result!=0);
       if(result != 0)
       {
          if(iTEEnable==eTE_FLOW_CONTROL_TLV_1)//TE_FLOW_CONTROL
          {
             result = QMIWDASetDataFormat (pDev, eTE_FLOW_CONTROL_TLV_0,pDev->iQMUXEnable);
             if(result != 0)
             {
                printk(KERN_INFO "Set Data Format Fail\n");
             }
             else
             {
                 printk(KERN_INFO "TE Flow Control disabled\n");
             }
          }
          else if(iTEEnable==eTE_FLOW_CONTROL_TLV_0)//TE_FLOW_CONTROL
          {
             result = QMIWDASetDataFormat (pDev, eSKIP_TE_FLOW_CONTROL_TLV,pDev->iQMUXEnable);
             if(result != 0)
             {
                printk(KERN_INFO "Set Data Format Fail No TE flow control\n");
             }
          }
       }
       else
       {
          if(iTEEnable!=eSKIP_TE_FLOW_CONTROL_TLV)
          {
             if(iTEEnable==eTE_FLOW_CONTROL_TLV_1)//TE_FLOW_CONTROL
             {
                printk(KERN_INFO "TE Flow Control Enabled\n");
             }
             else
             {
                printk(KERN_INFO "TE Flow Control disabled\n");
             }
          }
       }
   }
   else
   {
       pDev->iDataMode = eDataMode_Ethernet;
       result = QMICTLSetDataFormat (pDev);
       if(result!=0)
       {
         pDev->iDataMode = eDataMode_RAWIP;
         result = QMICTLSetDataFormat (pDev);
       }
   }

   if (result != 0)
   {
       return result;
   }

   i=0;
   do
   {
      // Setup WDS callback
      result = SetupQMIWDSCallback( pDev );
      RETURN_WHEN_DEVICE_ERR(pDev);
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
         return result;
         }
      }
   }while(result!=0);

   if (is9x15)
   {
       // Set FCC Authentication
       i=0;
       do
       {
          result = QMIDMSSWISetFCCAuth( pDev );
          RETURN_WHEN_DEVICE_ERR(pDev);
          if (result != 0)
          {
            if(i++>MAX_RETRY)
            {
               return result;
            }
          }
       }while(result!=0);
   }
   // Fill MEID for device
   i=0;
   do
   {
      result = QMIDMSGetMEID( pDev );
      RETURN_WHEN_DEVICE_ERR(pDev);
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            return result;
         }
      }
   }while(result!=0);

   // Fill Get Version info
   memset(&pDev->svcVersion[0], 0, sizeof(pDev->svcVersion));
   i=0;
   do
   {
      result = QMICTLGetVersionInfo(pDev);
      RETURN_WHEN_DEVICE_ERR(pDev);
      if (result != 0)
      {
         if(i++ > MAX_RETRY)
         {
            DBG( "QMI CTL Service Versions Procedure Error\n" );

            // Don't treat it fatal for driver loading, move on
            break;
         }
      }
      else
      {
         DBG( "QMI CTL Service Versions Procedure Successful\n" );
      }
   }while(result!=0);

   // allocate and fill devno with numbers
   result = alloc_chrdev_region( &devno, 0, 1, "qcqmi" );
   if (result < 0)
   {
      return result;
   }
   for(i=0;i<MAX_QCQMI;i++)
   {
       if (qcqmi_table[i] == 0)
           break;
   }

   if (i == MAX_QCQMI)
   {
       printk(KERN_WARNING "no free entry available at qcqmi_table array\n");
       return -ENOMEM;
   }
   qcqmi_table[i] = 1;
   pDev->mQMIDev.qcqmi = i;

   // Always print this output
   printk( KERN_INFO "creating qcqmi%d\n",
           pDev->mQMIDev.qcqmi );

   // Create cdev
   cdev_init( &pDev->mQMIDev.mCdev, &UserspaceQMIFops );
   pDev->mQMIDev.mCdev.owner = THIS_MODULE;
   pDev->mQMIDev.mCdev.ops = &UserspaceQMIFops;
   pDev->mQMIDev.mbCdevIsInitialized = true;

   result = cdev_add( &pDev->mQMIDev.mCdev, devno, 1 );
   if (result != 0)
   {
      DBG( "error adding cdev\n" );
      return result;
   }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,27 ))
   // kernel 2.6.27 added a new fourth parameter to device_create
   //    void * drvdata : the data to be added to the device for callbacks
   pDev->dev = device_create( pDev->mQMIDev.mpDevClass,
                  &pDev->mpIntf->dev,
                  devno,
                  NULL,
                  "qcqmi%d",
                  pDev->mQMIDev.qcqmi );
#else
   pDev->dev = device_create( pDev->mQMIDev.mpDevClass,
                  &pDev->mpIntf->dev,
                  devno,
                  "qcqmi%d",
                  pDev->mQMIDev.qcqmi );
#endif

   pDev->mQMIDev.mDevNum = devno;

   memset(pDev->maps.table, 0xff, sizeof(pDev->maps.table));
   pDev->maps.count = 0;

   sprintf(qcqmi_dev_name, "qcqmi%d", (int)pDev->mQMIDev.qcqmi);
   pDev->mQMIDev.proc_file = proc_create_data(qcqmi_dev_name, 0, NULL, &proc_fops, pDev);

   if (!pDev->mQMIDev.proc_file) {
       return -ENOMEM;
   }

  // Success
   return 0;
}

/*===========================================================================
METHOD:
   wakeup_target_process (Public Method)

DESCRIPTION:
   Close File Inode

PARAMETERS:
   pTask     [ I ] - task struct

RETURN VALUE:
   None
===========================================================================*/
void wakeup_target_process(struct task_struct * pTask)
{
   if(pTask!=NULL)
   {
      struct list_head *list = NULL;
      int count=0;
      #if _SIG_LOCK_
      struct sighand_struct *sighand = NULL;
      #endif
      if(wait_preempt()==0)
      return ;
      if(signal_pending(pTask))
         return ;
      if(pTask==current)
         return ;
      if(pTask->state!=TASK_STOPPED)
         return ;
      #if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
      while(raw_spin_is_locked(&pTask->pi_lock))
      #endif
      {
         if(count++>10)
         {
            DBG( "task Locked %d, pid:%d\n",__LINE__ ,pTask->pid);
            return ;
         }
         msleep_interruptible(100);
         mb();
         if(pTask==NULL)
            return;
         if(signal_pending(pTask))
         return ;
      }
      if(signal_pending(pTask))
         return ;
      #if _SIG_LOCK_
      if(lockdep_tasklist_lock_is_held())
      {
      DBG( "lockdep_tasklist_lock_is_held %d, pid:%d\n",__LINE__ ,pTask->pid);
      }
      sighand = rcu_dereference_check(pTask->sighand,
      lockdep_tasklist_lock_is_held());
      if(sighand)
      spin_lock_irq(&sighand->siglock);
      else
      {
      DBG( "spin_lock %d, pid:%d\n",__LINE__ ,pTask->pid);
      return ;
      }
      #endif
      if(pTask->state==TASK_STOPPED)
      {
         if (unlikely(pTask->signal->notify_count < 0))
         {
             if(pTask!=current)
             {
                 gobi_try_wake_up_process(pTask);
             }
             list_for_each(list, &pTask->children)
             {
                 struct task_struct *task;
                 task = list_entry(list, struct task_struct, sibling);
                 if(task!=NULL)
                 if(task!=current)
                 {
                    DBG( "%d pid:%d\n",__LINE__,task->pid );
                    gobi_try_wake_up_process(task);
                 }
             }
             wait_ms(500);
         }
      }
      #if _SIG_LOCK_
      if(sighand)
      spin_unlock_irq(&sighand->siglock);
      #endif
      return ;
   }
}

/*===========================================================================
METHOD:
   CloseFileInode (Public Method)

DESCRIPTION:
   Close File Inode

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int CloseFileInode(sGobiUSBNet * pDev,int iCount)
{
   struct inode * pOpenInode=NULL;
   struct task_struct * pEachTask = NULL;
   struct fdtable * pFDT;
   struct file * pFilp;
   unsigned long i_no =-1;
   int count = 0;
   //DBG("\n");
   if(pDev==NULL)
   {
      return 0;
   }
   GobiSyncRcu();
   if(!list_empty_careful(&pDev->mQMIDev.mCdev.list))
   if(!list_empty(&pDev->mQMIDev.mCdev.list))
    {
       list_for_each_entry(pOpenInode,&pDev->mQMIDev.mCdev.list,i_devices)
       {
          if(!pOpenInode)
          {
             break;
          }
          // Get the inode
          if (pOpenInode != NULL && (IS_ERR( pOpenInode ) == false))
          {
             i_no = pOpenInode->i_ino;
             DBG("OpenedInode:%lu\n",i_no);
             // Look for this inode in each task
             for_each_process( pEachTask )
             {
                int max_fds = 0;
                if (pEachTask == NULL )
                {
                   break;
                }
                if(pEachTask->files == NULL)
                {
                   // Some tasks may not have files (e.g. Xsession)
                   continue;
                }
                if(signal_pending(pEachTask))
                {
                  DBG("Task exiting OpenedInode:%lu\n",i_no);
                  return 1;
                }
                // For each file this task has open, check if it's referencing
                // our inode.
                pFDT = files_fdtable( pEachTask->files );
                if(pFDT)
                {
                    int iFound = 0;
                    max_fds = pFDT->max_fds;
                    for (count = 0; count < max_fds; count++)
                    {
                       if(pFDT==NULL)
                       {
                          break;
                       }
                       if(signal_pending(pEachTask))
                       {
                           break;
                       }
                       pFilp = pFDT->fd[count];

                       #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,19,0 ))
                       if (pFilp != NULL &&  pFilp->f_path.dentry != NULL)
                       #else
                       if (pFilp != NULL &&  pFilp->f_dentry != NULL)
                       #endif
                       {
                          #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,19,0 ))
                          if (pFilp->f_path.dentry->d_inode == pOpenInode)
                          #else
                          if (pFilp->f_dentry->d_inode == pOpenInode)
                          #endif
                          {
                             sQMIFilpStorage * pFilpData = NULL;
                             int reffrom = 0;
                             if(file_inode(pFilp)!=NULL)
                             {
                                i_no = file_inode(pFilp)->i_ino;
                             }
                             else
                             {
                                DBG("File Inode Null %d\n",__LINE__);
                                break;
                             }
                             printk( KERN_INFO "forcing close of opened file handle i_ino:%lu\n", i_no);

                             reffrom = gobi_atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
                             if (reffrom<2)
                             {
                                DBG("opened file handle i_ino:%lu\n", i_no);
                                break;
                             }
                             pFilpData = (sQMIFilpStorage *)pFilp->private_data;
                             if(pFilpData!=NULL)
                             {
                                 struct task_struct *pOpenTask = pFilpData->pOpenTask;
                                 struct task_struct *pReadTask = pFilpData->pReadTask;
                                 struct task_struct *pIOCTLTask = pFilpData->pIOCTLTask;
                                 struct task_struct *pWriteTask = pFilpData->pWriteTask;
                                 reffrom = 0;
                                 if((!signal_pending(pEachTask))&&
                                    (pFilpData->iSemID >0))
                                 {
                                    if(!down_trylock(&(pFilpData->mReadSem)))
                                    {
                                         DBG("NOT locked : %d",pFilpData->iSemID);
                                    }
                                    barrier();
                                    up(&(pFilpData->mReadSem));
                                 }
                                 if(pReadTask!=pOpenTask)
                                 {
                                    DBG("pReadTask:%d\n", pReadTask->pid);
                                    wakeup_target_process(pReadTask);
                                    pFilpData->pReadTask = pFilpData->pOpenTask;
                                    reffrom++;
                                    mb();
                                 }
                                 if(pIOCTLTask!=pOpenTask)
                                 {
                                    DBG("pIOCTLTask:%d\n", pIOCTLTask->pid);
                                    wakeup_target_process(pIOCTLTask);
                                    pFilpData->pIOCTLTask = pFilpData->pOpenTask;
                                    reffrom++;
                                    mb();
                                 }
                                 if(pWriteTask!=pOpenTask)
                                 {
                                    reffrom++;
                                    DBG("pWriteTask:%d\n", pWriteTask->pid);
                                    wakeup_target_process(pWriteTask);
                                    pFilpData->pWriteTask = pFilpData->pOpenTask;
                                    mb();
                                 }
                                 if(reffrom==0)
                                 if( (pOpenTask==pReadTask)&&
                                     (pOpenTask==pIOCTLTask)&&
                                     (pOpenTask==pWriteTask))
                                 {
                                    wakeup_target_process(pOpenTask);
                                 }
                                 if(reffrom>0)
                                 {
                                    return 1;
                                 }
                             }
                             else
                             {
                                 DBG("NULL private_data\n");
                             }
                             if(!signal_pending(pEachTask))
                             {
                                reffrom++;
                                wakeup_target_process(pEachTask);
                             }
                             if((!signal_pending(pEachTask))&&
                                (iCount>5))
                             {
                                DBG("ForceFilpClose:%d\n",pEachTask->pid);
                                ForceFilpClose(pFilp);
                             }
                             //Cannot return 1 here, task won't wakeup to close
                             if(reffrom>0)
                             {
                                 return 1;
                             }
                          }//if (pFilp->f_dentry->d_inode == pOpenInode)
                       }

                    }//for (count = 0; count < max_fds; count++)
                    if(iFound==0)
                    {
                       if(!signal_pending(pEachTask))
                       {
                          wakeup_target_process(pEachTask);
                       }
                       else
                       {
                          return 1;
                       }
                       iFound = 0;
                    }
                }//if(pFDT)
             }//for_each_process
          }//// Get the inode
       }//list_for_each_entry
      }//list_empty
   gobi_flush_work();
   mb();
   return 0;
}

/*===========================================================================
METHOD:
   gobi_flush_work (Public Method)

DESCRIPTION:
   sync memory

PARAMETERS:
   None

RETURN VALUE:
   None
===========================================================================*/
void gobi_flush_work(void)
{
    GobiSyncRcu();
    return ;
}

/*===========================================================================
METHOD:
   DeregisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device cleanup function

   NOTE: When this function is run the device is no longer valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void UnLocalClientMemLockSpinLock(sGobiUSBNet * pDev )
{
   int count =0;
   mb();
    while(LocalClientMemLockSpinIsLock( pDev ) != 0)
    {
      mb();
      if(count>10)
      {
         break;
      }
      count++;
      wait_ms(50);
    };
}
void DeregisterQMIDevice( sGobiUSBNet * pDev )
{
   int tries = 0;
   int result = -1;
   int i = 0;
   int iIntfNum = 0;
   unsigned long flags = 0;
   if(isPreempt()!=0)
   {
      printk("preempt_disable");
      preempt_disable();
   }
   #ifdef CONFIG_ANDROID
   if(pDev)
   {
      gobiLockSystemSleep(pDev);
   }
   #endif
   pDev->mbUnload = eStatUnloading;
   qmux_table[pDev->iDeviceMuxID]=0;
   // Should never happen, but check anyway
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "wrong device\n" );
      pDev->iNetLinkStatus = eNetDeviceLink_Disconnected;
      RemoveProcessFile(pDev);
      RemoveCdev(pDev);
      KillRead( pDev );
      GobiDestoryWorkQueue(pDev);
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      // Send SetControlLineState request (USB_CDC)
      result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             0, // DTR not present
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
      pDev->mbUnload = eStatUnloaded;
      gobi_flush_work();
      return;
   }
   pDev->iNetLinkStatus = eNetDeviceLink_Disconnected;
   RemoveProcessFile(pDev);
   pDev->mQMIDev.mCdev.ops = NULL;
   mb();
   tries = 0;
   iIntfNum = pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber;
   pDev->mQMIDev.mCdev.ops = NULL;
   UnLocalClientMemLockSpinLock(pDev);

   while(LocalClientMemLockSpinIsLock(pDev)!=0)
   {
      gobi_flush_work();
      if((tries%10)==0)
      {
         printk("Spinlocked\n");
      }
      if(200> tries++)
      {
         break;
      }
   }

   /* clear the qmux ip table */
   for ( i = 0; i < MAX_MUX_NUMBER_SUPPORTED; i++)
   {
      pDev->qMuxIPTable[i].instance = 0;
      pDev->qMuxIPTable[i].ipAddress = 0;
   }
   // Stop all reads
   KillRead( pDev );
   wait_interrupt();
   GobiDestoryWorkQueue(pDev);
   if(pDev->WDSClientID!=(u16)-1)
   ReleaseClientID( pDev, pDev->WDSClientID );

   gobi_flush_work();
   wait_interrupt();
   // Release all clients
   while (pDev->mQMIDev.mpClientMemList != NULL)
   {
      DBG( "release 0x%04X\n", pDev->mQMIDev.mpClientMemList->mClientID );
      if(pDev->mQMIDev.mpClientMemList->mClientID==QMICTL)
      {
          unsigned long flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
          // Timeout, remove the async read
          RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
          // End critical section
          LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
          break;
      }
      else
      {

      if (ReleaseClientID(pDev,
                       pDev->mQMIDev.mpClientMemList->mClientID) == false)
          break;
      // NOTE: pDev->mQMIDev.mpClientMemList will
      //       be updated in ReleaseClientID()
      }
      gobi_flush_work();
      wait_interrupt();
   }
   tries = 0;
   do
   {
      int ref = 0;
      mb();
      ref = gobi_atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
      DBG("%s:%d(%d) tries:%d ref:%d\n",__FUNCTION__,__LINE__,iIntfNum,tries,ref);
      if (ref > 1)
      {
         int iLoop = 0;
         mb();
         wait_interrupt();
         ref = gobi_atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
         if(wait_preempt()==1)
         if(ref>1)
         {

            while(CloseFileInode(pDev,tries)==1)
            {
               if(iLoop++>10)
               {
                  break;
               }
               if(preempt_count()>0)
               {
                  wait_ms(100);
               }
               else
               {
                  msleep_interruptible(100);
               }
            }
            gobi_flush_work();
            mb();
         }

         if(iLoop==0)
         {
            if(preempt_count()>0)
            {
               wait_ms(400);
            }
            else
            {
               msleep_interruptible(400);
            }
         }
         #ifdef CONFIG_ANDROID
         gobiStayAwake(pDev);
         #endif
      }
      else
      {
         break;
      }
      wait_ms(100);
   }while(20> tries++);
   gobi_flush_work();


   pDev->mbQMIValid = false;

   if (pDev->mQMIDev.mbCdevIsInitialized == false)
   {
      pDev->mbUnload = eStatUnloaded;
      return;
   }

   // Find each open file handle, and manually close it

   // Generally there will only be only one inode, but more are possible
   mb();
   if(pDev->iShutdown_write_sem>0)
   {
      down_write_trylock(&(pDev->shutdown_rwsem));
      up_write(&pDev->shutdown_rwsem);
   }
   mb();
   if(pDev->iShutdown_read_sem>0)
   {
      down_read_trylock(&(pDev->shutdown_rwsem));
      up_read(&pDev->shutdown_rwsem);
   }
   i =0;
   while(!down_trylock( &(pDev->ReadsyncSem) ))
   {
      i++;
      if(i>MAX_RETRY_LOCK_NUMBER)
      {
         break;
      }
      set_current_state(TASK_INTERRUPTIBLE);
      wait_ms(MAX_RETRY_LOCK_MSLEEP_TIME);
      if(signal_pending(current))
      {
        break;
      }
      if(pDev==NULL)
      {
         return ;
      }
   }
   up(&(pDev->ReadsyncSem));
   set_current_state(TASK_RUNNING);
   // Send SetControlLineState request (USB_CDC)
   result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             0, // DTR not present
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
   if (result < 0)
   {
      DBG( "SetControlLineState:%d\n", result );
   }

   // Remove device (so no more calls can be made by users)
   if (IS_ERR( pDev->mQMIDev.mpDevClass ) == false)
   {
      device_destroy( pDev->mQMIDev.mpDevClass,
                      pDev->mQMIDev.mDevNum );
   }

   qcqmi_table[pDev->mQMIDev.qcqmi] = 0;

   // Hold onto cdev memory location until everyone is through using it.
   // Timeout after 30 seconds (10 ms interval).  Timeout should never happen,
   // but exists to prevent an infinate loop just in case.
   #ifdef CONFIG_ANDROID
   for (tries = 0; tries < 10; tries++)
   #else
   for (tries = 0; tries < 60; tries++)
   #endif
   {
      int ref = gobi_atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
      if (ref > 1)
      {
         #ifdef CONFIG_ANDROID
         gobiStayAwake(pDev);
         #endif
         wait_ms(500);
         ref = gobi_atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
         if(ref>1)
         {
            printk( KERN_WARNING "cdev in use by %d tasks\n", ref - 1 );
            CloseFileInode(pDev,tries);
            wait_ms(500);
         }
      }
      else
      {
         break;
      }
   }

   cdev_del( &pDev->mQMIDev.mCdev );

   unregister_chrdev_region( pDev->mQMIDev.mDevNum, 1 );
   pDev->mbUnload = eStatUnloaded;
   return;
}

/*=========================================================================*/
// Driver level client management
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMIReady (Public Method)

DESCRIPTION:
   Send QMI CTL GET VERSION INFO REQ and SET DATA FORMAT REQ
   Wait for response or timeout

PARAMETERS:
   pDev     [ I ] - Device specific memory
   timeout  [ I ] - Milliseconds to wait for response

RETURN VALUE:
   int
===========================================================================*/
int QMIReady(
   sGobiUSBNet *    pDev,
   u16                timeout )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   struct semaphore readSem;
   u16 curTime;
   u8 transactionID;
   unsigned long flags;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return false;
   }

   writeBufferSize = QMICTLReadyReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return false;
   }

   // An implimentation of down_timeout has not been agreed on,
   //    so it's been added and removed from the kernel several times.
   //    We're just going to ignore it and poll the semaphore.

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   // Send a write every 1000 ms and see if we get a response
   for (curTime = 0; curTime < timeout; curTime += 1000)
   {
      if(gobi_kthread_should_stop())
      {
         return -1;
      }
      if(isModuleUnload(pDev))
      {
         if(pWriteBuffer)
            kfree( pWriteBuffer );
         return -EFAULT;
      }
      else if(IsDeviceDisconnect(pDev))
      {
         DBG( "Device Disconnected!\n" );
         if(pWriteBuffer)
            kfree( pWriteBuffer );
         return -ENXIO;\
      }
      #ifdef CONFIG_ANDROID
      gobiStayAwake(pDev);
      #endif
      // Start read
      transactionID =QMIXactionIDGet(pDev);

      result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);
      if (result != 0)
      {
         kfree( pWriteBuffer );
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return false;
      }

      // Fill buffer
      result = QMICTLReadyReq( pWriteBuffer,
                               writeBufferSize,
                               transactionID );
      if (result < 0)
      {
         kfree( pWriteBuffer );
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return false;
      }

      // Disregard status.  On errors, just try again
      WriteSync( pDev,
                 pWriteBuffer,
                 writeBufferSize,
                 QMICTL );
      if(gobi_kthread_should_stop())
      {
         set_current_state(TASK_RUNNING);
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return -1;
      }
      if(isModuleUnload(pDev))
      {
         if(pWriteBuffer)
            kfree( pWriteBuffer );
         return -EFAULT;
      }
      else if(IsDeviceDisconnect(pDev))
      {
         DBG( "Device Disconnected!\n" );
         if(pWriteBuffer)
            kfree( pWriteBuffer );
         return -ENXIO;\
      }

      if(curTime < timeout)
      {
         int iScaleCount = 0;
         for(iScaleCount=0;iScaleCount<100;iScaleCount++)
         {
            if( gobi_kthread_should_stop() ||
                IsDeviceDisconnect(pDev) )
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               KillRead( pDev );
               flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
               RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
               LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               return -1;
            }
            msleep( 10 );//wait_ms(10);//msleep( 10 );
            if(gobi_kthread_should_stop()||
                IsDeviceDisconnect(pDev))
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               KillRead( pDev );
               flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
               RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
               LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               return -1;
            }
            if(isModuleUnload(pDev))
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               KillRead( pDev );
               flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
               RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
               LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               return -1;
            }
         }

      }
      mb();
      // Enter critical section
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
      barrier();
      spin_lock_irq(&(pDev->notif_lock));
      if (down_trylock( &readSem ) == 0)
      {
         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 QMICTL,
                                 transactionID,
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            // Success
            spin_unlock_irq(&(pDev->notif_lock));
            // End critical section
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

            // We don't care about the result
            if(pReadBuffer)
            kfree( pReadBuffer );
            break;
         }
         else
         {
            RemoveAndPopNotifyList(pDev,QMICTL,transactionID,eClearCID);
            spin_unlock_irq(&(pDev->notif_lock));
            // Read mismatch/failure, unlock and continue
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         }
      }
      else
      {
         if( (pDev->mbUnload < eStatUnloading) ||
            IsDeviceDisconnect(pDev))
         {
             // Enter critical section
             // Timeout, remove the async read
             RemoveAndPopNotifyList(pDev,QMICTL,transactionID,eClearCID);
             // End critical section
         }
         spin_unlock_irq(&(pDev->notif_lock));
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      }
   }
   kfree( pWriteBuffer );
   // Did we time out?
   if (curTime >= timeout)
   {
      return false;
   }
   DBG( "QMI Ready after %u milliseconds\n", curTime );
   if(SetPowerSaveMode(pDev,0)<0)
   {
      DBG("Set Power Save Mode error\n");
   }

   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   // Timeout, remove the async read
   RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   // Success
   return true;
}

/*===========================================================================
METHOD:
   QMIWDSCallback (Public Method)

DESCRIPTION:
   QMI WDS callback function
   Update net stats or link state

PARAMETERS:
   pDev     [ I ] - Device specific memory
   clientID [ I ] - Client ID
   pData    [ I ] - Callback data (unused)

RETURN VALUE:
   None
===========================================================================*/
void QMIWDSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData )
{
   bool bRet;
   int result;
   void * pReadBuffer=NULL;
   u16 readBufferSize;
   u32 TXOk = (u32)-1;
   u32 RXOk = (u32)-1;
   u32 TXErr = (u32)-1;
   u32 RXErr = (u32)-1;
   u32 TXOfl = (u32)-1;
   u32 RXOfl = (u32)-1;
   u64 TXBytesOk = (u64)-1;
   u64 RXBytesOk = (u64)-1;
   bool bReconfigure;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return;
   }

   bRet = PopFromReadMemList( pDev,
                              clientID,
                              0,
                              &pReadBuffer,
                              &readBufferSize );


   if (bRet == false)
   {
      DBG( "WDS callback failed to get data\n" );
      if(pReadBuffer)
         kfree( pReadBuffer );
      pReadBuffer = NULL;
      return;
   }

   // Default values
   pDev->bLinkState = ! GobiTestDownReason( pDev, NO_NDIS_CONNECTION );
   bReconfigure = false;

   result = QMIWDSEventResp( pReadBuffer,
                             readBufferSize,
                             &TXOk,
                             &RXOk,
                             &TXErr,
                             &RXErr,
                             &TXOfl,
                             &RXOfl,
                             &TXBytesOk,
                             &RXBytesOk,
                             (u8*)&pDev->bLinkState,
                             &bReconfigure );
   if (result < 0)
   {
      DBG( "bad WDS packet\n" );
   }
   else
   {
      if (bReconfigure == true)
      {
         DBG( "Net device link reset\n" );
         GobiSetDownReason( pDev, NO_NDIS_CONNECTION );
         GobiClearDownReason( pDev, NO_NDIS_CONNECTION );
      }
      else
      {
         if (pDev->bLinkState == true)
         {
            DBG( "Net device link is connected\n" );
            GobiClearDownReason( pDev, NO_NDIS_CONNECTION );
            #ifdef CONFIG_ANDROID
            SetTxRxStat(pDev, RESUME_RX_OKAY | RESUME_TX_OKAY);
            #endif
         }
         else
         {
            DBG( "Net device link is disconnected\n" );
            GobiSetDownReason( pDev, NO_NDIS_CONNECTION );
            #ifdef CONFIG_ANDROID
            SetTxRxStat(pDev, RESUME_TX_RX_DISABLE);
            #endif
         }
      }
   }
   if(pReadBuffer)
   kfree( pReadBuffer );
   pReadBuffer = NULL;

   // Setup next read
   result = ReadAsync( pDev,
                       clientID,
                       0,
                       QMIWDSCallback,
                       pData ,0);
   #ifdef CONFIG_ANDROID
   if(pDev)
   {
      PRINT_WS_LOCK(pDev->ws);
   }
   #endif

   if (result != 0)
   {
      DBG( "unable to setup next async read\n" );
   }

   return;
}

void QMIQOSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData )
{
   bool bRet;
   int result;
   void * pReadBuffer;
   u16 readBufferSize;

   if (IsDeviceValid( pDev ) == false)
   {
      QDBG( "Invalid device\n" );
      return;
   }

   bRet = PopFromReadMemList( pDev,
                              clientID,
                              0,
                              &pReadBuffer,
                              &readBufferSize );


   if (bRet == false)
   {
      QDBG( "QOS callback failed to get data\n" );
      return;
   }

   result = QMIQOSEventResp(pDev, pReadBuffer, readBufferSize);

   if (result < 0)
   {
      QDBG( "bad QOS packet\n" );
   }
   if(pReadBuffer)
   kfree( pReadBuffer );

   // Setup next read
   result = ReadAsync( pDev,
                       clientID,
                       0,
                       QMIQOSCallback,
                       pData ,0);

   if (result != 0)
   {
      QDBG( "unable to setup next async read\n" );
   }

   return;
}

/*===========================================================================
METHOD:
   SetupQMIWDSCallback (Public Method)

DESCRIPTION:
   Request client and fire off reqests and start async read for
   QMI WDS callback

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int SetupQMIWDSCallback( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   u16 WDSClientID;

   RETURN_WHEN_DEVICE_ERR(pDev);

   result = GetClientID( pDev, QMIWDS,NULL );
   if (result < 0)
   {
      return result;
   }
   pDev->WDSClientID = WDSClientID = result;

   // QMI WDS Set Event Report
   writeBufferSize = QMIWDSSetEventReportReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDSSetEventReportReq( pWriteBuffer,
                                     writeBufferSize,
                                     1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // QMI WDS Get PKG SRVC Status
   writeBufferSize = QMIWDSGetPKGSRVCStatusReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDSGetPKGSRVCStatusReq( pWriteBuffer,
                                       writeBufferSize,
                                       2 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // Setup asnyc read callback
   result = ReadAsync( pDev,
                       WDSClientID,
                       0,
                       QMIWDSCallback,
                       NULL ,1);
   if (result != 0)
   {
      DBG( "unable to setup async read\n" );
      return result;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSSWISetFCCAuth (Public Method)

DESCRIPTION:
   Register DMS client
   send FCC Authentication req and parse response
   Release DMS client

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMIDMSSWISetFCCAuth( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 DMSClientID;
   struct semaphore readSem;
   unsigned long flags;
   DBG("\n");

   RETURN_WHEN_DEVICE_ERR(pDev);

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();

   result = GetClientID( pDev, QMIDMS ,NULL);
   if (result < 0)
   {
      return result;
   }
   DMSClientID = result;

   // QMI DMS Get Serial numbers Req
   writeBufferSize = QMIDMSSWISetFCCAuthReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIDMSSWISetFCCAuthReq( pWriteBuffer,
                                    writeBufferSize,
                                    1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
      return result;
   }

   result = ReadAsync( pDev, DMSClientID, 1, UpSem, &readSem ,1);
   if(result == 0)
   {
      result = WriteSync( pDev,
                    pWriteBuffer,
                    writeBufferSize,
                    DMSClientID );
   }
   if(pWriteBuffer)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              DMSClientID,
                              1,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = 0;
         // We don't care about the result
         if(pReadBuffer)
         kfree( pReadBuffer );
      }
      else
      {
         // Read mismatch/failure, unlock and continue
         DBG( "Read mismatch/failure, unlock and continue\n" );
         RemoveAndPopNotifyList(pDev,DMSClientID,1,eClearCID);
         spin_unlock_irq(&(pDev->notif_lock));
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      }
   }
   else
   {
      DBG( "Timeout\n" );
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,DMSClientID,1,eClearCID);
      spin_unlock_irq(&(pDev->notif_lock));
      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      result = -1;
   }

   if (result < 0)
   {
      // Non fatal error, device did not return FCC Auth response
      DBG( "Bad FCC Auth resp\n" );
   }
   ReleaseClientID( pDev, DMSClientID );

   // Success
   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEID (Public Method)

DESCRIPTION:
   Register DMS client
   send MEID req and parse response
   Release DMS client

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMIDMSGetMEID( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 DMSClientID;
   unsigned long flags;
   struct semaphore readSem;

   RETURN_WHEN_DEVICE_ERR(pDev);

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   result = GetClientID( pDev, QMIDMS ,NULL);
   if (result < 0)
   {
      return result;
   }
   DMSClientID = result;

   // QMI DMS Get Serial numbers Req
   writeBufferSize = QMIDMSGetMEIDReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIDMSGetMEIDReq( pWriteBuffer,
                              writeBufferSize,
                              1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
      return result;
   }

   result = ReadAsync( pDev, DMSClientID, 1, UpSem, &readSem ,1);
   if(result == 0)
   {
      result = WriteSync( pDev,
                    pWriteBuffer,
                    writeBufferSize,
                    DMSClientID );
   }
   if(pWriteBuffer)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              DMSClientID,
                              1,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = QMIDMSGetMEIDResp( pReadBuffer,
                               readBufferSize,
                               &pDev->mMEID[0],
                               14 );
         // We don't care about the result
         if(pReadBuffer)
         kfree( pReadBuffer );
      }
      else
      {
         RemoveAndPopNotifyList(pDev,DMSClientID,1,eClearAndReleaseCID);
         spin_unlock_irq(&(pDev->notif_lock));
         // Read mismatch/failure, unlock and continue
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      }
   }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,DMSClientID,1,eClearAndReleaseCID);
      spin_unlock_irq(&(pDev->notif_lock));
      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      result = -1;
      if (result < 0)
      {
         DBG( "bad get MEID resp\n" );
         // Non fatal error, device did not return any MEID
         //    Fill with 0's
         memset( &pDev->mMEID[0], '0', 14 );
      }
   }

   if (result < 0)
   {
      DBG( "bad get MEID resp\n" );

      // Non fatal error, device did not return any MEID
      //    Fill with 0's
      memset( &pDev->mMEID[0], '0', 14 );
   }

   ReleaseClientID( pDev, DMSClientID );

   // always return Success as MEID is only available on CDMA devices only
   return 0;
}

/*===========================================================================
METHOD:
   QMICTLSetDataFormat (Public Method)

DESCRIPTION:
   send Data format request and parse response

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMICTLSetDataFormat( sGobiUSBNet * pDev )
{
   u8 transactionID;
   struct semaphore readSem;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;

   //DBG("\n");
   RETURN_WHEN_DEVICE_ERR(pDev);

   // Send SET DATA FORMAT REQ
   writeBufferSize = QMICTLSetDataFormatReqSize();

   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   // Start read
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();

   transactionID = QMIXactionIDGet(pDev);

   // Fill buffer
   result = QMICTLSetDataFormatReq( pWriteBuffer,
                            writeBufferSize,
                            transactionID ,
                            pDev->iDataMode);

   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   DBG("Sending QMI Set Data Format Request, TransactionID: 0x%x\n", transactionID );

   WriteSync( pDev,
              pWriteBuffer,
              writeBufferSize,
              QMICTL );

   //msleep( 100 );
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         // Success
         PrintHex(pReadBuffer, readBufferSize);
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         // We care about the result: call Response  function
         result = QMICTLSetDataFormatResp( pReadBuffer, readBufferSize,pDev->iDataMode);
         if(pReadBuffer)
         kfree( pReadBuffer );

         if (result != 0)
         {
            DBG( "Device cannot set requested data format\n" );
            if(pWriteBuffer)
            kfree( pWriteBuffer );
            return result;
         }
      }
      else
      {
         spin_unlock_irq(&(pDev->notif_lock));
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      }
   }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,transactionID,eForceClearAndReleaseCID);
      // End critical section

   }
   spin_unlock_irq(&(pDev->notif_lock));
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   kfree( pWriteBuffer );

   return 0;
}


/*===========================================================================
METHOD:
   QMIWDASetQMAP (Public Method)

DESCRIPTION:
   Send set QMAP Data format request and parse response

PARAMETERS:
   pDev            [ I ] - Device specific memory
   WDAClientID     [ I ] - WDA Client ID
RETURN VALUE:
   int - 0 for success
            Negative errno for failure
===========================================================================*/
int QMIWDASetQMAP( sGobiUSBNet * pDev , u16 WDAClientID)
{
   int result;
   void *pWriteBuffer;
   u16 writeBufferSize;
   void *pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;
   struct semaphore readSem;
   u16 uTID=2;
   DBG("\n");

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();

   // QMI WDA Set Data Format Request
   writeBufferSize = QMIWDASetDataFormatReqSettingsSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDASetDataFormatReqSettingsReq( pWriteBuffer,
                                    writeBufferSize,
                                    uTID);
   if (result < 0)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
      return result;
   }

   result = ReadAsync( pDev, WDAClientID, uTID, UpSem, &readSem ,1);
   if(result == 0)
   {
      result = WriteSync( pDev,
                          pWriteBuffer,
                          writeBufferSize,
                          WDAClientID );
   }
   if(pWriteBuffer)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
   }
   if (result < 0)
   {
      DBG( "WriteSync Fail\n" );
      return result;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
       // Enter critical section
         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 WDAClientID,
                                 uTID,
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            PrintHex( pReadBuffer, readBufferSize );
            result = 0;
            // We don't care about the result
            if(pReadBuffer)
            kfree( pReadBuffer );
         }
         else
         {
            // Read mismatch/failure, unlock and continue
            RemoveAndPopNotifyList(pDev,WDAClientID,uTID,eClearAndReleaseCID);
         }
   }
   else
   {
      DBG( "Timeout\n" );
      result = -1;
      // Timeout, remove the async read
      barrier();
      RemoveAndPopNotifyList(pDev,WDAClientID,uTID,eClearAndReleaseCID);
   }
   spin_unlock_irq(&(pDev->notif_lock));
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   if (result < 0)
   {
      DBG( "Data Format Cannot be set\n" );
   }
   return result;
}
/*===========================================================================
METHOD:
   QMICTLGetVersionInfo (Public Method)

DESCRIPTION:
   send get version info request and parse response

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMICTLGetVersionInfo( sGobiUSBNet * pDev )
{
   u8 transactionID;
   struct semaphore readSem;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;

   DBG("\n");

   RETURN_WHEN_DEVICE_ERR(pDev);

   // Send CTL GET VERSION INFO REQ
   writeBufferSize = QMICTLGetVersionInfoReqSize();

   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   // Start read
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();

   transactionID = QMIXactionIDGet(pDev);

   // Fill buffer
   result = QMICTLGetVersionInfoReq( pWriteBuffer,
                            writeBufferSize,
                            transactionID);

   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   DBG("Sending QMI CTL Get Version Info Request, TransactionID: 0x%x\n", transactionID );

   result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);

   if (result == 0) {
       WriteSync( pDev,
              pWriteBuffer,
              writeBufferSize,
              QMICTL );
   }

   kfree( pWriteBuffer );
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         // Success
         PrintHex(pReadBuffer, readBufferSize);
         spin_unlock_irq(&(pDev->notif_lock));

         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

         // We care about the result: call Response  function
         result = QMICTLGetVersionInfoResp(
                      pReadBuffer,
                      readBufferSize,
                      &pDev->svcVersion[0],
                      sizeof(pDev->svcVersion));

         if(pReadBuffer)
             kfree( pReadBuffer );

         if (result != 0)
         {
            DBG( "Could not parse version info correctly in QMI response %d\n",result );
            return result;
         }
      }
      else
      {
         // Read mismatch/failure, unlock and continue
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
         spin_unlock_irq(&(pDev->notif_lock));
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = -1;
      }
   }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
      spin_unlock_irq(&(pDev->notif_lock));
      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      result = -1;
   }

   if (result < 0)
   {
      DBG( "QMI CTL Get Version Info error code %d\n", result );
      return result;
   }

   // Success
   return 0;
}
/*===========================================================================
METHOD:
   QMIWDASetDataFormat (Public Method)

DESCRIPTION:
   Register WDA client
   send Data format request and parse response
   Release WDA client

PARAMETERS:
   pDev            [ I ] - Device specific memory
   te_flow_control [ I ] - TE Flow Control Flag
   iqmuxenable     [ I ] - QMUX Control Flag
RETURN VALUE:
   int - 0 for success
            Negative errno for failure
===========================================================================*/
int QMIWDASetDataFormat( sGobiUSBNet * pDev, int te_flow_control , int iqmuxenable)
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 WDAClientID;
   unsigned long flags;
   struct semaphore readSem;
   u16 uTID=1;
   //DBG("\n");

   RETURN_WHEN_DEVICE_ERR(pDev);
   barrier();
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   result = GetClientID( pDev, QMIWDA ,NULL);
   if (result < 0)
   {
      return result;
   }
   WDAClientID = result;

   // QMI WDA Set Data Format Request
   writeBufferSize = QMIWDASetDataFormatReqSize(te_flow_control,iqmuxenable);
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      ReleaseClientID( pDev, WDAClientID );
      return -ENOMEM;
   }

   result = QMIWDASetDataFormatReq( pWriteBuffer,
                                    writeBufferSize,
                                    uTID,
                                    te_flow_control,
                                    pDev->iDataMode,
                                    pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                                    iqmuxenable);
   if (result < 0)
   {
      kfree( pWriteBuffer );
      ReleaseClientID( pDev, WDAClientID );
      return result;
   }

   result = ReadAsync( pDev, WDAClientID, uTID, UpSem, &readSem ,1);
   if(result == 0)
   {
      result = WriteSync( pDev,
                          pWriteBuffer,
                          writeBufferSize,
                          WDAClientID );
   }
   if(pWriteBuffer)
   {
      kfree( pWriteBuffer );
      pWriteBuffer = NULL;
   }
   if (result < 0)
   {
      DBG( "WriteSync Fail\n" );
      ReleaseClientID( pDev, WDAClientID );
      return result;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
       // Enter critical section
         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 WDAClientID,
                                 uTID,
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            u32 *pULDatagram = NULL;
            u32 *pULDatagramSize = NULL;
            if(iqmuxenable)
            {
               pULDatagram = &pDev->ULDatagram;
               pULDatagramSize = &pDev->ULDatagramSize;
            }
            result = QMIWDASetDataFormatResp(pReadBuffer,
                                                  readBufferSize,
                                                  pDev->iDataMode,
                                                  pULDatagram,
                                                  pULDatagramSize);
            // We don't care about the result
            if(pReadBuffer)
            kfree( pReadBuffer );
         }
         else
         {
            // Read mismatch/failure, unlock and continue
            RemoveAndPopNotifyList(pDev,WDAClientID,uTID,eClearAndReleaseCID);
         }
   }
   else
   {
      DBG( "Timeout\n" );
      result = -1;
      // Timeout, remove the async read
      barrier();
      RemoveAndPopNotifyList(pDev,WDAClientID,uTID,eClearAndReleaseCID);
   }
   spin_unlock_irq(&(pDev->notif_lock));
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   if (result < 0)
   {
      DBG( "Data Format Cannot be set\n" );
   }
   if(iqmuxenable!=0)
   {
      /* Set QMAP Aggregation size*/
      result = QMIWDASetQMAP( pDev , WDAClientID);
   }
   ReleaseClientID( pDev, WDAClientID );

   // Success
   return result;
}

#define gobi_mdelay_interruptible(n) (\
   (__builtin_constant_p(n) && (n)<=MAX_UDELAY_MS) ? udelay((n)*1000) : \
   ({unsigned long __ms=(n); \
   while (__ms--){ \
      udelay(1000); \
      if(signal_pending(current)){\
         break;\
      } \
   };\
   }))

void wait_ms(unsigned int ms) {
   if(in_atomic())
   {
      DBG("preempt_ensabled\n");
      return ;
   }
   if (!in_interrupt())
   {
       schedule_timeout_interruptible(1 + ms * HZ / 1000);
   }
   else
   {
      barrier();
      gobi_mdelay_interruptible(ms);
   }
}

void wait_control_msg_semaphore_timeout(struct semaphore *pSem, unsigned int timeout)
{
    unsigned int totaltime = 0;
    if(pSem==NULL)
    {
        return;
    }
    do
    {
        wait_ms(QMI_CONTROL_MSG_DELAY_MS/2);
        if (down_trylock( pSem ) == 0)
        {
            up(pSem);
            return;
        }
        if(signal_pending(current))
        {
            return ;
        }
        totaltime +=QMI_CONTROL_MSG_DELAY_MS/2;
    }while(totaltime<timeout);
    DBG("wait timeout\n");
    return ;
}

/*===========================================================================
METHOD:
   RemoveAndPopNotifyList (Public Method)

DESCRIPTION:
   Remove first Notify entry from this client's notify list
   and Run function

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   bool
===========================================================================*/
int RemoveAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID ,
   int              iClearClientID)
{
   sClientMemList * pClientMem;
   sNotifyList * pDelNotifyList, ** ppNotifyList;
   sClientMemList ** ppDelClientMem;
   sClientMemList * pNextClientMem;
   mb();
   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif
   do
   {
      // Get this client's memory location
      pClientMem = FindClientMem( pDev, clientID );
      if (pClientMem == NULL)
      {
         DBG( "Could not find this client's memory 0x%04X\n", clientID );
         return eNotifyListEmpty;
      }

      ppNotifyList = &(pClientMem->mpReadNotifyList);
      pDelNotifyList = NULL;
      // Remove from list
      CLIENT_READMEM_SNAPSHOT(clientID,pDev);
      while (*ppNotifyList != NULL)
      {
         // Do we care about transaction ID?
         if (transactionID == 0
         ||  (*ppNotifyList)->mTransactionID == 0
         ||  transactionID == (*ppNotifyList)->mTransactionID)
         {
            pDelNotifyList = *ppNotifyList;
            mb();
            DBG( "Remove Notify TID = %x\n", (*ppNotifyList)->mTransactionID );
            break;
         }

         DBG( "skipping data TID = %x\n", (*ppNotifyList)->mTransactionID );

         // next
         ppNotifyList = &(*ppNotifyList)->mpNext;
      }
      mb();
      if (pDelNotifyList != NULL)
      {
         // Remove element
         *ppNotifyList = (*ppNotifyList)->mpNext;
         pDelNotifyList->mpNext = NULL;
         pDelNotifyList->mpNotifyFunct = NULL;
         pDelNotifyList->mpData = NULL;
         pDelNotifyList->mTransactionID = 0;
         // Delete memory
         kfree( pDelNotifyList );
         pDelNotifyList = NULL;
         mb();
      }
      else
      {
         void *pFreeData = NULL;
         u16 FreeDataSize;
         //Remove From memory List
         while(PopFromReadMemList( pDev,
             clientID,
             transactionID,
             &pFreeData,
             &FreeDataSize ) == true )
         {
             DBG( "Remove Mem ClientID: 0x%x, data TID = 0x%x\n", clientID,transactionID);
             kfree( pFreeData );
             pFreeData = NULL;
         }
         DBG( "no one to notify for TID 0x%x\n", transactionID );
         mb();
         break;//return eNotifyListEmpty;
      }
   }while(ppNotifyList!=NULL);


   ppDelClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppDelClientMem != NULL)
   {
      if ((*ppDelClientMem)->mClientID == clientID)
      {
         pNextClientMem = (*ppDelClientMem)->mpNext;
         kfree( *ppDelClientMem );
         *ppDelClientMem = NULL;

         // Overwrite the pointer that was to this client mem
         *ppDelClientMem = pNextClientMem;
      }
      else
      {
         // I now point to (a pointer of ((the node I was at)'s mpNext))
          if(*ppDelClientMem==NULL)
          {
              DBG("ppDelClientMem NULL %d\r\n",__LINE__);
              break;
          }
         ppDelClientMem = &(*ppDelClientMem)->mpNext;
      }
      mb();
   }
   mb();
   return eNotifyListEmpty;
}

/*===========================================================================
METHOD:
   SetPowerSaveMode (Public Method)

DESCRIPTION:
   Set mode in power save mode

PARAMETERS:
   pDev     [ I ] - Device specific memory
   mode     [ I ] - power save mode, 0:wakeup ; 1:suspend

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int SetPowerSaveMode(sGobiUSBNet *pDev,u8 mode)
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u8 transactionID;
   unsigned long flags;
   struct semaphore readSem;

   if (IsDeviceValid( pDev ) == false)
   {
      printk(KERN_ERR "Invalid device!\n" );
      return -ENXIO;
   }

   if(iIsRemoteWakeupSupport(pDev->mpNetDev)==0)
   {
      DBG("remote wakeup not supported\n");
      return 0;
   }
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();
   writeBufferSize = QMICTLSetPowerSaveModeReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   transactionID = QMIXactionIDGet(pDev);
   result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);

    result = QMICTLSetPowerSaveModeReq(pWriteBuffer,
                                     writeBufferSize,
                                     transactionID,
                                     mode );
   if (result < 0)
   {
       kfree( pWriteBuffer );
       return result;
   }
   result = WriteSyncNoResume( pDev,
          pWriteBuffer,
          writeBufferSize,
          QMICTL );
   kfree( pWriteBuffer );

   if (result < 0)
   {
        DBG( "bad write data %d\n", result );
        return result;
   }
   wait_control_msg_semaphore_timeout(&readSem, QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         // Success
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = QMICTLSetPowerSaveModeResp(pReadBuffer,
                                               readBufferSize);

         // We don't care about the result
         if(pReadBuffer)
         kfree( pReadBuffer );
         return result;
      }
      else
      {
         // Read mismatch/failure, unlock and continue
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearAndReleaseCID);
      }
   }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,0,eClearAndReleaseCID);
      // End critical section
      result = -1;
   }
   spin_unlock_irq(&(pDev->notif_lock));
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return result;

}

/*==========================================================================
METHOD:
   WriteSyncNoResume (Public Method)

DESCRIPTION:
   Start synchronous write without resume device

PARAMETERS:
   pDev                 [ I ] - Device specific memory
   pWriteBuffer         [ I ] - Data to be written
   writeBufferSize      [ I ] - Size of data to be written
   clientID             [ I ] - Client ID of requester

RETURN VALUE:
   int - write size (includes QMUX)
         negative errno for failure
============================================================================*/
int WriteSyncNoResume(
   sGobiUSBNet *          pDev,
   char *                 pWriteBuffer,
   int                    writeBufferSize,
   u16                    clientID )
{
   int i;
   int result;
   int iLockRetry =0;
   //DBG("\n");
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "Unloading device!\n" );
      return -ENXIO;
   }
   // Fill writeBuffer with QMUX
   result = FillQMUX( clientID, pWriteBuffer, writeBufferSize );
   if (result < 0)
   {
      return result;
   }

   // Wake device
   #ifdef CONFIG_PM
   UsbAutopmGetInterface( pDev->mpIntf );
   #else
   gobi_usb_autopm_get_interface( pDev->mpIntf );
   #endif

   DBG( "Actual Write:\n" );
   PrintHex( pWriteBuffer, writeBufferSize );

   // Write Control URB, protect with read semaphore to track in-flight USB control writes in case of disconnect
   for(i=0;i<USB_WRITE_RETRY;i++)
   {

      if(isModuleUnload(pDev))
      {
         DBG( "unloaded\n" );
         return -EFAULT;
      }
      pDev->iShutdown_read_sem= __LINE__;
      if(signal_pending(current))
      {
         return -ERESTARTSYS;
      }

      iLockRetry = 0;
      mb();
      while(down_read_trylock(&(pDev->shutdown_rwsem))!=1)
      {
         wait_ms(5);
         mb();
         if(iLockRetry++>100)
         {
            DBG("down_read_trylock timeout");
            return -EFAULT;
         }
         if(pDev==NULL)
         {
            DBG( "NULL\n" );
            return -EFAULT;
         }
         if (pDev->mbUnload >= eStatUnloading)
         {
            DBG( "unloaded\n" );
            return -EFAULT;
         }
      }
      mb();
      result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev, usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
             SEND_ENCAPSULATED_COMMAND,
             USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
             0, pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
             (void*)pWriteBuffer, writeBufferSize,
             USB_WRITE_TIMEOUT );
       up_read(&pDev->shutdown_rwsem);
       if(signal_pending(current))
       {
          return -ERESTARTSYS;
       }
       if (IsDeviceDisconnect(pDev) )
       {
         return -ENXIO;
       }
       if(pDev==NULL)
       {
          return -EFAULT;
       }

       pDev->iShutdown_read_sem=- __LINE__;

       if (pDev->mbUnload >= eStatUnloading)
       {
          DBG( "unloaded\n" );
          return -EFAULT;
       }

       if (signal_pending(current))
       {
           return -ERESTARTSYS;
       }

       if (result < 0)
       {
          printk(KERN_WARNING "usb_control_msg failed (%d)", result);
       }
       // Control write transfer may occasionally timeout with certain HCIs, attempt a second time before reporting an error
       if (result == -ETIMEDOUT)
       {
           pDev->writeTimeoutCnt++;
           printk(KERN_WARNING "Write URB timeout, cnt(%d)\n", pDev->writeTimeoutCnt);
       }
       else if(result < 0 )
       {
          DBG( "%s no device!\n" ,__FUNCTION__);
           return result;
       }
       else
       {
           break;
       }
       if (IsDeviceValid( pDev ) == false)
       {
          DBG( "%s Invalid device!\n" ,__FUNCTION__);
          return -ENXIO;
       }
       if (pDev->mbUnload > eStatUnloading)
       {
         DBG( "unloaded\n" );
         return -EFAULT;
       }
   }

   // Write is done, release device
   UsbAutopmPutInterface(pDev->mpIntf);
   return result;
}

int ReleaseNotifyList(sGobiUSBNet *pDev,u16 clientID,u8 transactionID)
{
   unsigned long flags;
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   // Timeout, remove the async read
   NotifyAndPopNotifyList( pDev, clientID, transactionID );
   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return 0;
}

#ifdef CONFIG_PM
/*===========================================================================
METHOD:
   ConfigPowerSaveSettings (Public Method)

DESCRIPTION:
   Set modem power save mode config

PARAMETERS:
   pDev      [ I ] - Device specific memory
   service   [ I ] - QMI service number
   indication[ I ] - QMI indication number

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int ConfigPowerSaveSettings(sGobiUSBNet *pDev, u8 service, u8 indication)
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u8 transactionID;
   unsigned long flags;
   struct semaphore readSem;

   if (IsDeviceValid(pDev) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   if(iIsRemoteWakeupSupport(pDev->mpNetDev)==0)
   {
      DBG("remote wakeup not supported\n");
      return 0;
   }

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   mb();

   writeBufferSize = QMICTLConfigPowerSaveSettingsReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

    transactionID = QMIXactionIDGet(pDev);
    result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);
    result = QMICTLConfigPowerSaveSettingsReq(pWriteBuffer,
                                              writeBufferSize,
                                              transactionID,
                                              service,
                                              indication);
    if (result < 0)
    {
        kfree( pWriteBuffer );
        return result;
    }

    result = WriteSyncNoResume( pDev,
                      pWriteBuffer,
                      writeBufferSize,
                      QMICTL );
    kfree( pWriteBuffer );

   if (result < 0)
   {
       DBG( "bad write data %d\n", result );
       return result;
   }
   wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
   mb();
   // Enter critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   spin_lock_irq(&(pDev->notif_lock));
   barrier();
   if (down_trylock( &readSem ) == 0)
   {
      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         // Success
         spin_unlock_irq(&(pDev->notif_lock));
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         result = QMICTLConfigPowerSaveSettingsResp(pReadBuffer,
                                               readBufferSize);

         // We don't care about the result
         if(pReadBuffer)
         kfree( pReadBuffer );
         return result;
      }
      else
      {
         // Read mismatch/failure, unlock and continue
         RemoveAndPopNotifyList(pDev,QMICTL,0,eClearAndReleaseCID);
      }
    }
   else
   {
      // Timeout, remove the async read
      RemoveAndPopNotifyList(pDev,QMICTL,0,eClearAndReleaseCID);
      // End critical section
      result = -1;
   }
   spin_unlock_irq(&(pDev->notif_lock));
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return result;
}
#endif

void RemoveProcessFile(sGobiUSBNet *pDev)
{
   char qcqmi_dev_name[10]={0};
   if(pDev->mQMIDev.proc_file != NULL)
   {
      sprintf(qcqmi_dev_name, "qcqmi%d", (int)pDev->mQMIDev.qcqmi);
      remove_proc_entry(qcqmi_dev_name, NULL);
      pDev->mQMIDev.proc_file = NULL;
      DBG("remove:%s",qcqmi_dev_name);
   }
   return;
}

void RemoveCdev(sGobiUSBNet * pDev)
{
   if(pDev->mQMIDev.mbCdevIsInitialized==true)
   {
      pDev->mQMIDev.mbCdevIsInitialized=false;
      if (IS_ERR( pDev->mQMIDev.mpDevClass ) == false)
      {
         device_destroy( pDev->mQMIDev.mpDevClass,
                         pDev->mQMIDev.mDevNum );
         cdev_del( &pDev->mQMIDev.mCdev );
         unregister_chrdev_region( pDev->mQMIDev.mDevNum, 1 );
      }
   }
}

/*===========================================================================
WriteSyncNoRetry
   WriteSync (Public Method)

DESCRIPTION:
   Start synchronous write no retry

PARAMETERS:
   pDev                 [ I ] - Device specific memory
   pWriteBuffer         [ I ] - Data to be written
   writeBufferSize      [ I ] - Size of data to be written
   clientID             [ I ] - Client ID of requester

RETURN VALUE:
   int - write size (includes QMUX)
         negative errno for failure
===========================================================================*/
int WriteSyncNoRetry(
   sGobiUSBNet *          pDev,
   char *                 pWriteBuffer,
   int                    writeBufferSize,
   u16                    clientID )
{
   int result = -1;
   int iLockRetry =0;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   if(IsDeviceDisconnect(pDev))
   {
      DBG( "Device Disconnected!\n" );
      return -ENXIO;
   }
   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "Unloading device!\n" );
      return -ENXIO;
   }
   // Fill writeBuffer with QMUX
   result = FillQMUX( clientID, pWriteBuffer, writeBufferSize );
   if (result < 0)
   {
      return result;
   }

   // Wake device
   result = gobi_usb_autopm_get_interface( pDev->mpIntf );
   if (result < 0)
   {
      DBG( "unable to resume interface: %d\n", result );

      // Likely caused by device going from autosuspend -> full suspend
      if (result == -EPERM)
      {
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
         pDev->mpNetDev->udev->auto_pm = 0;
#endif
         GobiNetSuspend( pDev->mpIntf, PMSG_SUSPEND );
#endif /* CONFIG_PM */
      }
      return result;
   }

   DBG( "Actual Write:\n" );
   PrintHex( pWriteBuffer, writeBufferSize );


  if(isModuleUnload(pDev))
  {
     DBG( "unloaded\n" );
     return -EFAULT;
  }
  pDev->iShutdown_read_sem= __LINE__;

  iLockRetry = 0;
  mb();
  while(down_read_trylock(&(pDev->shutdown_rwsem))!=1)
  {
     wait_ms(5);
     mb();
     if(iLockRetry++>100)
     {
        DBG("down_read_trylock timeout");
        return -EFAULT;
     }
     if(pDev==NULL)
     {
        DBG( "NULL\n" );
        return -EFAULT;
     }
     if (pDev->mbUnload >= eStatUnloading)
     {
        DBG( "unloaded\n" );
        return -EFAULT;
     }
  }
  mb();
  result = Gobi_usb_control_msg(pDev->mpIntf, pDev->mpNetDev->udev, usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
         SEND_ENCAPSULATED_COMMAND,
         USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
         0, pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
         (void*)pWriteBuffer, writeBufferSize,
         50 );//50ms timeout
   up_read(&pDev->shutdown_rwsem);
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   if (IsDeviceDisconnect(pDev) )
   {
      return -ENXIO;
   }
   if(pDev==NULL)
   {
      return -EFAULT;
   }

   pDev->iShutdown_read_sem=- __LINE__;

   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "unloaded\n" );
      return -EFAULT;
   }
   if (result < 0)
   {
      printk(KERN_WARNING "usb_control_msg failed (%d)", result);
   }
   // Control write transfer may occasionally timeout with certain HCIs, attempt a second time before reporting an error
   if (result == -ETIMEDOUT)
   {
       pDev->writeTimeoutCnt++;
       printk(KERN_WARNING "Write URB timeout, cnt(%d)\n", pDev->writeTimeoutCnt);
   }
   else if(result < 0 )
   {
      DBG( "%s no device!\n" ,__FUNCTION__);
      gobi_usb_autopm_put_interface( pDev->mpIntf );
       return result;
   }

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "%s Invalid device!\n" ,__FUNCTION__);
      return -ENXIO;
   }
   if (pDev->mbUnload > eStatUnloading)
   {
     DBG( "unloaded\n" );
     return -EFAULT;
   }

   // Write is done, release device
   gobi_usb_autopm_put_interface( pDev->mpIntf );


   return result;
}

int Gobi_usb_control_msg(struct usb_interface *intf,struct usb_device *dev, unsigned int pipe, __u8 request,
                     __u8 requesttype, __u16 value, __u16 index, void *data,
                      __u16 size, int timeout)
{
   if(dev==NULL)
   return -ENODEV;
   mb();
   if(intf==NULL)
      return -ENODEV;
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,33 ))
   if(intf->resetting_device)
   {
      return -ENXIO;
   }
   #endif
   if (dev->parent->state == USB_STATE_NOTATTACHED )
   {
      return -ENXIO;
   }
   if (dev->state == USB_STATE_NOTATTACHED )
   {
      return -ENXIO;
   }
   return usb_control_msg(dev,pipe,request,requesttype,value,index,data,size,timeout);

}

bool IsDeviceDisconnect(sGobiUSBNet *pDev)
{
   if(!pDev)
      return true;
   mb();
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,33 ))
   if(pDev->mpIntf==NULL)
   {
      return true;
   }
   if(pDev->mpIntf->resetting_device)
   {
      pDev->iUSBState = USB_STATE_NOTATTACHED;
      mb();
      return true;
   }
   #endif
   if(pDev->iUSBState == USB_STATE_NOTATTACHED)
      return true;
   if(!interface_to_usbdev(pDev->mpIntf))
      return true;
   if (interface_to_usbdev(pDev->mpIntf)->state == USB_STATE_NOTATTACHED )
   {
      pDev->iUSBState = USB_STATE_NOTATTACHED;
      mb();
      return true;
   }
   if(pDev->mpIntf->condition == USB_INTERFACE_UNBINDING)
   {
      pDev->iUSBState = USB_STATE_NOTATTACHED;
      mb();
      return true;
   }
   if(pDev->mpIntf->condition == USB_INTERFACE_UNBOUND)
   {
      pDev->iUSBState = USB_STATE_NOTATTACHED;
      mb();
      return true;
   }
   return false;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
void gobi_usb_autopm_enable(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return ;
   }
   usb_autopm_enable( intf );
}
#endif
void gobi_usb_autopm_put_interface(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return ;
   }
   usb_autopm_put_interface(intf);
}

int gobi_usb_autopm_get_interface_async(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return -ENXIO;
   }
   return usb_autopm_get_interface_async(intf);
}

void gobi_usb_autopm_get_interface_no_resume(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return ;
   }
   #if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,0,0 ))
   usb_autopm_get_interface_async(intf);
   return ;
   #else
   usb_autopm_get_interface_no_resume(intf);
   #endif
}

void gobi_usb_autopm_put_interface_no_resume(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return ;
   }
   #if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,0,0 ))
   usb_autopm_put_interface_async(intf);
   return ;
   #else
   usb_autopm_put_interface_no_suspend(intf);
   #endif
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,29 ))
void usb_autopm_put_interface_async(struct usb_interface *intf)
{
   struct usb_device   *udev = interface_to_usbdev(intf);
   int         status = 0;

   if (intf->condition == USB_INTERFACE_UNBOUND) {
      status = -ENODEV;
   } else {
      udev->last_busy = jiffies;
      --intf->pm_usage_cnt;
      if (udev->autosuspend_disabled || udev->autosuspend_delay < 0)
         status = -EPERM;
      else if (intf->pm_usage_cnt <= 0 &&
            !timer_pending(&udev->autosuspend.timer)) {
         queue_delayed_work(ksuspend_usb_wq, &udev->autosuspend,
               round_jiffies_up_relative(
                  udev->autosuspend_delay));
      }
   }
}
#endif
void gobi_usb_autopm_put_interface_async(struct usb_interface *intf)
{
   if(IsInterfacefDisconnected(intf))
   {
      return ;
   }

   usb_autopm_put_interface_async(intf);
}

struct net_device* gobi_qmimux_register_device(struct net_device *real_dev,int iNumber, u8 mux_id)
{
   struct net_device *new_dev;
   struct gobi_qmimux_priv *priv;
   int err;
   char szName[64]={0};
   DBG("Create 0x%x\n",mux_id);
   snprintf(szName,63,"gobi-%d-%d",iNumber,mux_id-MUX_ID_START);
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,18,0 ))
   new_dev = alloc_netdev(sizeof(struct gobi_qmimux_priv),
                szName, NET_NAME_UNKNOWN, ether_setup);
   #else
   new_dev = alloc_netdev(sizeof(struct gobi_qmimux_priv),
                          szName, ether_setup);
   #endif
   if(!new_dev)
   {
      printk(KERN_WARNING "Memory error alloc netdev");
      return NULL;
   }
   new_dev->netdev_ops = &gobi_qmimux_netdev_ops;
   new_dev->flags           = IFF_NOARP | IFF_MULTICAST;
   random_ether_addr(new_dev->dev_addr);
   if (!new_dev)
      return NULL;
   priv = netdev_priv(new_dev);
   priv->mux_id = mux_id;
   priv->real_dev = real_dev;
   new_dev->irq = real_dev->irq;

   err = register_netdevice(new_dev);
   if (err < 0)
      goto out_free_newdev;
   dev_hold(real_dev);
   return new_dev;

out_free_newdev:
   free_netdev(new_dev);
   return NULL;
}

void gobi_qmimux_unregister_device(struct net_device *dev)
{
   struct gobi_qmimux_priv *priv = netdev_priv(dev);
   struct net_device *real_dev = priv->real_dev;
   netif_carrier_off(dev);
   netif_stop_queue(dev);
   netif_dormant_off(dev);
   dev_close(dev);
   netif_device_detach(dev);
   unregister_netdevice(dev);
   /* Get rid of the reference to real_dev */
   dev_put(real_dev);
}

int iIsValidQmuxSKB(struct sk_buff *skb)
{
   if(!skb)
      return 0;
   if(skb->len >= QMUX_HEADER_LENGTH)
   {
      return iNumberOfQmuxPacket(skb,0);
   }
   return 0;
}

int PrintQmuxPacket(struct sk_buff *skb)
{
   unsigned int offset=0;
   return 0;
   NETDBG("SKB->len:%d\n",skb->len);
   if(skb->len >= QMUX_HEADER_LENGTH)
   {
      u32 length = skb->data[2];
      length = (length<<8) + skb->data[3];
      //To Fix Incomming packet larger than expected.
      if(length==(skb->len-QMUX_HEADER_LENGTH))
      {
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            return 1;
         }
         else if((skb->data[0]<0x80)
         &&(skb->data[1]==0))
         {
            return 1;
         }
         else
         {
            PrintHex (skb->data, skb->len);
         }
         return 0;
      }
      else if(length<(skb->len-QMUX_HEADER_LENGTH))//
      {
         int iNumberOfPacket=1;
         offset += length + QMUX_HEADER_LENGTH;
         do
         {
            length = skb->data[offset+2];
            length = (length<<8) + skb->data[offset+3];
            if((skb->data[offset]!=0)&&(skb->data[offset+1]!=0)&&
               (skb->data[offset+2]!=0)&&(skb->data[offset+3]!=0))
            {
               NETDBG("offset:%d, %02X%02X%02X%02X:length:%d\n",
                  offset,
               skb->data[offset],skb->data[offset+1],skb->data[offset+2],skb->data[offset+3]
               ,length);
            }
            if(length==0)
            {
               if((skb->data[offset]<0x80)
               &&(skb->data[offset+1]&0x8F))
               {
                  iNumberOfPacket++;
               }
               else if((skb->data[offset]<0x80)
                  &&(skb->data[offset+1]==0))
               {
                  iNumberOfPacket++;
               }
               else
               {
                  return iNumberOfPacket;
               }
               if((length+offset)==(skb->len-QMUX_HEADER_LENGTH))
               {
                  return iNumberOfPacket;
               }
            }
            offset += length +QMUX_HEADER_LENGTH;
         }while(offset < skb->len);
         NETDBG("\niNumberOfPacket:%d\n",iNumberOfPacket);
         return iNumberOfPacket;
      }
      else
      {
         NETDBG("Length Not matched.\n");
      }
   }
   return 0;
}

int iIsQmuxZeroPacket(struct sk_buff *skb)
{
   if(skb==NULL)
      return 0;
   if(skb->len >= QMUX_HEADER_LENGTH)
   {
      u32 length = skb->data[2];
      length = (length<<8) + skb->data[3];
      if(length==0)
      {
         if((skb->data[0]==0)
         &&(skb->data[1]==0))
         {
            return 1;
         }
      }
   }
   return 0;
}

int iIsQmuxPacketComplete(struct sk_buff *skb)
{
   if(skb==NULL)
      return 0;
   if(skb->len >= QMUX_HEADER_LENGTH)
   {
      u32 length = skb->data[2];
      length = (length<<8) + skb->data[3];
      if(length==(skb->len-QMUX_HEADER_LENGTH))
      {
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            return 1;
         }
      }
      else if(length<=(skb->len-QMUX_HEADER_LENGTH))//
      {
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            return 1;
         }
      }
      else
      {
         DBG("Length Not matched.\n");
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            return 0;
         }
      }
   }
   return 0;
}

int iNumberOfQmuxPacket(struct sk_buff *skb,int iDisplay)
{
   unsigned int offset=0;
   if(!skb)
      return 0;
   if(iDisplay==1)
   {
      DBG("Packet Len: 0x%x\n",skb->len);
      PrintHex(skb->data,skb->len);
   }
   if(skb->len >= QMUX_HEADER_LENGTH)
   {
      //To Fix Incomming packet larger than expected.
      u32 length = skb->data[2];
      length = (length<<8) + skb->data[3];
      if(length==(skb->len-QMUX_HEADER_LENGTH))
      {
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            return 1;
         }
         else
         {
            PrintHex (skb->data, skb->len);
         }
         NETDBG("Single Packet:");
         NetHex(skb->data,4);
         return 0;
      }
      else if(length<(skb->len-QMUX_HEADER_LENGTH))//
      {
         int iNumberOfPacket=1;
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            iNumberOfPacket=1;
            NETDBG("%d. Packet: %02X%02X %02X%02X .. \n",iNumberOfPacket ,
               skb->data[offset],
               skb->data[offset+1],
               skb->data[offset+2],
               skb->data[offset+3]);
         }
         else
         {
            NETDBG("%s:%d Invalid QMAP Packet\n",__FUNCTION__,__LINE__);
               NETDBG("%d. Packet: %02X%02X %02X%02X .. \n",0,
               skb->data[offset],
               skb->data[offset+1],
               skb->data[offset+2],
               skb->data[offset+3]);
            return 0;
         }
         offset += length + QMUX_HEADER_LENGTH;
         do
         {
            NETDBG("%d. Packet: %02X%02X %02X%02X .. \n",iNumberOfPacket+1,
               skb->data[offset],
               skb->data[offset+1],
               skb->data[offset+2],
               skb->data[offset+3]);
            length = skb->data[offset+2];
            length = (length<<8) + skb->data[offset+3];
            NETDBG("iNumberOfPacket:%d, offset:%d/%d len:0x%x\n",
               iNumberOfPacket,offset
                  ,skb->len
                  ,length)
            if(length>skb->len-offset-QMUX_HEADER_LENGTH)
            {
               NETDBG("iNumberOfPacket:%d, offset:%d/%d delta:0x%x\n",
                  iNumberOfPacket,offset
                  ,skb->len
                  ,length-(skb->len-offset-QMUX_HEADER_LENGTH));
               return iNumberOfPacket;
            }
            offset += length +QMUX_HEADER_LENGTH;
            iNumberOfPacket++;
         }while(offset < skb->len);
         return iNumberOfPacket;
      }
      else
      {
         NETDBG("Length Not matched.\n");
         NETDBG("Incomplete QMAP Packet %s:%d\n",__FUNCTION__,__LINE__);
         if(iIsValidQMAPHeaderInSKBData(skb,0)==1)
         {
            NETDBG("%d. Packet: %02X%02X %02X%02X .. \n",0,
               skb->data[offset],
               skb->data[offset+1],
               skb->data[offset+2],
               skb->data[offset+3]);
            return 0;
         }
         NETDBG("Unknown Packet: %02X%02X %02X%02X .. \n",
               skb->data[offset],
               skb->data[offset+1],
               skb->data[offset+2],
               skb->data[offset+3]);
         return -1;
      }
   }
   return -1;
}

/***********************************
         0 - Request, i.e., sender is sending a
             QMAP control command to the receiver.
         1 - Ack, i.e., receiver is acknowledging that
             it received a QMAP control command and that
            it successfully processed the command.
         2 - Unsupported command, i.e., receiver does
             not support this QMAP control command.
         3 - Invalid command, i.e., receiver encountered
             an error while processing the QMAP control command,
             probably because QMAP control command is malformed.
      **************************************/
int iGetQmuxIDFromSKB(struct sk_buff *skb)
{
   if(iIsValidQmuxSKB(skb))
   {
      return (int)skb->data[1];
   }
   return -1;
}

u32 u32GetSKBQMAPPacketLength(struct sk_buff *skb,int iOffset)
{
   if(skb==NULL)
      return 0;
   if(skb->len < QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   else if(iOffset>skb->len)
   {
      return 0;
   }
   else
   {
      u32 length = skb->data[iOffset+2];
      length = (length<<8) + skb->data[iOffset+3];
      return length;
   }
   return 0;
}

int iIsValidQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset)
{
   if(iOffset<0)
      return 0;
   if(!pSKB)
      return 0;
   if(pSKB->len<QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   if(pSKB->len<iOffset +QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   if((pSKB->data[iOffset+1]&0x8F)||
      (pSKB->data[iOffset+1]==0x00))
   {
      //Downlink padding bytes Less or equal 64 Bytes
      if(pSKB->data[iOffset] > QMAP_MAX_PADDING_BYTES)
      {
         return 0;
      }
      else
      {
         return 1;
      }
   }
   return 0;
}

int iIsCMDQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset)
{
   u32 length = 0;
   if(iOffset<0)
      return 0;
   if(!pSKB)
      return 0;
   if(pSKB->len<QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   length = u32GetSKBQMAPPacketLength(pSKB,iOffset);
   if(pSKB->len < length)
   {
      return 0;
   }
   if(pSKB->data[iOffset]==0x80)
   {
      return 1;
   }
   return 0;
}


int iIsZeroQMAPHeaderInSKBData(struct sk_buff *pSKB, int iOffset)
{
   if(iOffset<0)
      return 0;
   if(!pSKB)
      return 0;
   if(pSKB->len<QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   if(pSKB->len!=iOffset +QMUX_HEADER_LENGTH)
   {
      return 0;
   }
   if((pSKB->data[iOffset]==0x00)
         &&(pSKB->data[iOffset+1]==0x00)
         &&(pSKB->data[iOffset+2]==0x00)
         &&(pSKB->data[iOffset+3]==0x00))
   {
      return 1;
   }
   return 0;
}

/*===========================================================================
METHOD:
   NetHex (Public Method)

DESCRIPTION:
   Print Hex data, for QMAP debug purposes

PARAMETERS:
   pBuffer       [ I ] - Data buffer
   bufSize       [ I ] - Size of data buffer

RETURN VALUE:
   None
===========================================================================*/
void NetHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;
   if(!(debug & DEBUG_NETMASK))
      return ;
   if(bufSize==(u16)(-1))
   {
       DBG( "No Data\n" );
   }
   pPrintBuf = kmalloc( bufSize * 3 + 1, GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );
   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)),
                         4,
                         "%02X ",
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         kfree( pPrintBuf );
         return;
      }
   }
   printk( "   : %s\n", pPrintBuf );
   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;
}

/*===========================================================================
METHOD:
   ErrHex (Public Method)

DESCRIPTION:
   Print Hex data, for QMAP debug purposes

PARAMETERS:
   pBuffer       [ I ] - Data buffer
   bufSize       [ I ] - Size of data buffer

RETURN VALUE:
   None
===========================================================================*/
void ErrHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;
   if(bufSize==(u16)(-1))
   {
       DBG( "No Data\n" );
   }
   pPrintBuf = kmalloc( bufSize * 3 + 1, GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );
   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)),
                         4,
                         "%02X ",
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         kfree( pPrintBuf );
         return;
      }
   }
   printk( "   : %s\n", pPrintBuf );
   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;
}

/*===========================================================================
GobiInitWorkQueue
   GobiInitWorkQueue (Private Method)

DESCRIPTION:
   Init and Create workqueue in device specifc memory.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to Device specific memory
RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int GobiInitWorkQueue(sGobiUSBNet *pGobiDev)
{
   char szProcessName[MAX_WQ_PROC_NAME_SIZE]={0};
   int tableindex = -1;
   int interfaceindex = 0;
   if(pGobiDev==NULL)
      return -1;
   interfaceindex = GetPrivateWorkQueuesInterfaceTableIndex(pGobiDev);

   tableindex = AddPrivateWorkQueues(pGobiDev);
   if((tableindex<0) || (tableindex>=MAX_QCQMI))
   {
      return -1;
   }

   memset(&szProcessName,0,sizeof(szProcessName));
   if(GenerateProcessName("gobiprobe",&szProcessName[0],MAX_WQ_PROC_NAME_SIZE-1,pGobiDev)!=0)
   {
      return -1;
   }
   pGobiDev->wqprobe = GetPrivateWorkQueuesWQByTableIndex(tableindex,interfaceindex,eWQ_PROBE);
   if(pGobiDev->wqprobe==NULL)
   {
      pGobiDev->wqprobe = create_workqueue(szProcessName);
      if (!pGobiDev->wqprobe)
      {
        printk("Create Work Queue Probe Failed\n");
        return -1;
      }
      SetPrivateWorkQueuesWQByTableIndex(tableindex,interfaceindex,pGobiDev->wqprobe,eWQ_PROBE);
   }
   memset(&szProcessName,0,sizeof(szProcessName));
   if(GenerateProcessName("gobireadcb",&szProcessName[0],MAX_WQ_PROC_NAME_SIZE-1,pGobiDev)!=0)
   {
      return -1;
   }
   pGobiDev->wqProcessReadCallback = GetPrivateWorkQueuesWQByTableIndex(tableindex,interfaceindex,eWQ_URBCB);
   if(pGobiDev->wqProcessReadCallback==NULL)
   {
      pGobiDev->wqProcessReadCallback = create_workqueue(szProcessName);
      if (!pGobiDev->wqProcessReadCallback)
      {
        printk("Create Work Queue Probe Failed\n");
        return -1;
      }
      SetPrivateWorkQueuesWQByTableIndex(tableindex,interfaceindex,pGobiDev->wqProcessReadCallback,eWQ_URBCB);
   }
   #ifdef CONFIG_ANDROID
   pGobiDev->wqLockSystemSleep = create_workqueue(szProcessName);
   if (!pGobiDev->wqLockSystemSleep)
   {
      printk("Create Work Queue LockSystemSleep Failed\n");
      return -1;
   }
   pGobiDev->wqUnLockSystemSleep = create_workqueue(szProcessName);
   if (!pGobiDev->wqUnLockSystemSleep)
   {
      printk("Create Work Queue UnLockSystemSleep Failed\n");
      return -1;
   }
   #endif
   return 0;
}

/*===========================================================================
GobiDestoryWorkQueue
   GobiDestoryWorkQueue (Private Method)

DESCRIPTION:
   Destory workqueues in device specific memory.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to Device specific memory
RETURN VALUE:
   NULL
===========================================================================*/
void GobiDestoryWorkQueue(sGobiUSBNet *pGobiDev)
{
   int interfaceindex = 0;
   int tableindex = 0;
   if(pGobiDev==NULL)
      return ;

   interfaceindex = GetPrivateWorkQueuesInterfaceTableIndex(pGobiDev);
   tableindex = GetPrivateWorkQueuesIndex(pGobiDev);

   GobiCancelReadCallBackWorkQueue(pGobiDev);

   GobiCancelProbeWorkQueue(pGobiDev);
   #ifdef CONFIG_ANDROID
   GobiCancelLockSystemSleepWorkQueue(pGobiDev);
   GobiCancelUnLockSystemSleepWorkQueue(pGobiDev);
   #endif
   ClearPrivateWorkQueuesProcessByTableIndex(tableindex,
                  interfaceindex);

}

/*===========================================================================
iRemoveQMAPPaddingBytes

   RemoveQMAPPaddingBytes (Private Method)

DESCRIPTION:
   Remove QMAP header and padding in SKB buffer.

PARAMETERS:
   skb                 [ I ] - Pointer to sk_buff pointer
RETURN VALUE:
   int - 0 for success
         Negative errno for error
===========================================================================*/
int iRemoveQMAPPaddingBytes(struct sk_buff *skb)
{
   u8 padding_lenth = 0;
   if(skb==NULL)
   return -1;
   padding_lenth = (u8)skb->data[0];

   if(padding_lenth>0)
   {
      NETDBG("padding_lenth:%d\n",padding_lenth);
      if((padding_lenth<=QMAP_MAX_PADDING_BYTES)&&
          skb->len >= padding_lenth)
      {
         skb_trim(skb,skb->len - padding_lenth);
      }
      else
      {
         return -1;
      }
   }
   if(skb->len>=QMUX_HEADER_LENGTH)
   {
      skb_pull(skb,QMUX_HEADER_LENGTH);
   }
   else
   {
      return -1;
   }
   return 0;
}

int IsOtherTaskUsingFilp(struct file *pFilp)
{
   sQMIFilpStorage * pFilpData = NULL;
   long refcnt = 0;
   if(pFilp ==NULL)
   {
      return 0;
   }

   refcnt = atomic_long_read(&pFilp->f_count);
   if (refcnt == 1)
   {
      return 0;
   }
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      return 0;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      return 0;
   }
   if((pFilpData->pOpenTask!=pFilpData->pReadTask)||
      (pFilpData->pOpenTask!=pFilpData->pWriteTask)||
      (pFilpData->pOpenTask!=pFilpData->pIOCTLTask))
   {
      return 1;
   }
   return 0;
}

int IsOpenTaskIsCurrent(struct file *pFilp)
{
   sQMIFilpStorage * pFilpData = NULL;
   if(pFilp ==NULL)
   {
      return 0;
   }
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      return 0;
   }
   if(IsDeviceDisconnect(pFilpData->mpDev))
   {
      return 0;
   }
   #if 0
   printk(KERN_INFO "c/o/r/w/i %d/%d/%d/%d/%d\n",
   task_pid_nr(current),
   task_pid_nr(pFilpData->pOpenTask),
   task_pid_nr(pFilpData->pReadTask),
   task_pid_nr(pFilpData->pWriteTask),
   task_pid_nr(pFilpData->pIOCTLTask));
   #endif
   if(pFilpData->pOpenTask==current)
   {
      return 1;
   }

   return 0;
}

int IsCurrentTaskExit(void)
{
    if(current->flags & PF_EXITING)
        return 1;
    return 0;
}

/*===========================================================================
gobi_work_busy

   gobi_work_busy (Private Method)

DESCRIPTION:
   Check delayed work is busy.

PARAMETERS:
   dw                 [ I ] - Pointer to delayed_work pointer
RETURN VALUE:
   int - 0 not busy
===========================================================================*/
int gobi_work_busy(struct delayed_work *dw)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,36 ))
   if(work_pending(&dw->work))
      return 1;
   return 0;
#else
   return work_busy(&dw->work);
#endif
}

/*===========================================================================
ClientTransactionIDExist
   ClientTransactionIDExist (Private Method)

DESCRIPTION:
   Check Client Transcation ID already in notify list

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to Device specific memory.
   clientID                 [ I ] - Client ID.
   u16TransactionID         [ I ] - Transaction ID.
RETURN VALUE:
    int - 1 Found Transcation ID in client.
          0 Not found Transcation ID in client.
===========================================================================*/
int ClientTransactionIDExist(sGobiUSBNet *pDev, u16 clientID,u16 u16TransactionID)
{
   sClientMemList * pClientMem;
   sNotifyList ** ppThisNotifyList;
   if(pDev==NULL)
   {
      DBG("NULL");
      return 0;
   }

#ifdef CONFIG_SMP
   // Verify Lock
   #if _IGNORE_DISCONNECT_SPIN_LOCK_CHECK_
   if(!IsDeviceDisconnect(pDev))
   #endif
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return 0;
   }

   // Go to last URBList entry
   ppThisNotifyList = &pClientMem->mpReadNotifyList;
   while (*ppThisNotifyList != NULL)
   {
      if((*ppThisNotifyList)->mTransactionID==u16TransactionID)
      {
         return 1;
      }
      ppThisNotifyList = &(*ppThisNotifyList)->mpNext;
   }
   return 0;
}

/*===========================================================================
ReadCallbackInt

   ReadCallbackInt (Private Method)

DESCRIPTION:
   Queue URB to be process.

PARAMETERS:
   pReadURB                 [ I ] - Pointer to urb pointer
RETURN VALUE:
   none
===========================================================================*/
void ReadCallbackInt( struct urb * pReadURB )
{
   sGobiUSBNet * pDev;
   if (pReadURB == NULL)
   {
      DBG( "bad read URB\n" );
      return;
   }
   pDev = pReadURB->context;
   pDev->pReadURB = pReadURB;
   gobiProcessReadURB(pDev);
   return ;
}

/*===========================================================================
ProcessReadWorkFunction

   ProcessReadWorkFunction (Private Method)

DESCRIPTION:
   Process ReadCallback.

PARAMETERS:
   w                 [ I ] - Pointer to work_struct pointer
RETURN VALUE:
   none
===========================================================================*/
static void ProcessReadWorkFunction(struct work_struct *w)
{
   struct delayed_work *dwork;
   sGobiUSBNet *pGobiDev = NULL;
   dwork = to_delayed_work(w);
   pGobiDev = container_of(dwork, sGobiUSBNet, dwProcessReadCallback);
   if(pGobiDev!=NULL)
   {
      DBG("ResubmitIntURB\n");
      ReadCallback(pGobiDev->pReadURB);
   }
   else
   {
      DBG("pGobiDev NULL\n");
   }
}

/*===========================================================================
gobiProcessReadURB

   gobiProcessReadURB (Private Method)

DESCRIPTION:
   Add delayed work to wqProcessReadCallback.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
   none
===========================================================================*/
static void gobiProcessReadURB(sGobiUSBNet *pGobiDev)
{
   INIT_DELAYED_WORK(&pGobiDev->dwProcessReadCallback,
            ProcessReadWorkFunction);
   queue_delayed_work(pGobiDev->wqProcessReadCallback, &pGobiDev->dwProcessReadCallback, 0);
}
#ifdef CONFIG_ANDROID
/*===========================================================================
ProcessLockSystemSleepFunction

   ProcessLockSystemSleepFunction (Private Method)

DESCRIPTION:
   Process ReadCallback.

PARAMETERS:
   w                 [ I ] - Pointer to work_struct pointer
RETURN VALUE:
   none
===========================================================================*/
static void ProcessLockSystemSleepFunction(struct work_struct *w)
{
   struct delayed_work *dwork;
   sGobiUSBNet *pGobiDev = NULL;
   dwork = to_delayed_work(w);
   pGobiDev = container_of(dwork, sGobiUSBNet, dwLockSystemSleep);
   if(pGobiDev!=NULL)
   {
      gobiStayAwake(pGobiDev);
   }
   else
   {
      DBG("pGobiDev NULL\n");
   }
}

/*===========================================================================
gobiLockSystemSleep


   gobiLockSystemSleep (Private Method)

DESCRIPTION:
   Add delayed work to wqProcessReadCallback.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
   none
===========================================================================*/
void gobiLockSystemSleep(sGobiUSBNet *pGobiDev)
{
   WLDEBUG("gobiLockSystemSleep\n");
   GobiCancelLockSystemSleepWorkQueue(pGobiDev);
   INIT_DELAYED_WORK(&pGobiDev->dwLockSystemSleep,
            ProcessLockSystemSleepFunction);
   queue_delayed_work(pGobiDev->wqLockSystemSleep, &pGobiDev->dwLockSystemSleep, 0);
}

/*===========================================================================
gobiPmRelax

   gobiPmRelax (Private Method)

DESCRIPTION:
   run __pm_relax.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
   none
===========================================================================*/
void gobiPmRelax(sGobiUSBNet *pGobiDev)
{
   if(pGobiDev)
   {
      struct wakeup_source *ws = pGobiDev->ws;
      if(ws)
      {
         PRINT_WS_LOCK(ws);
         WLDEBUG("__pm_relax start\n");
         __pm_relax(ws);
         WLDEBUG("__pm_relax end\n");
         PRINT_WS_LOCK(ws);
      }
   }
}

/*===========================================================================
Gobi_pm_stay_awake

   Gobi_pm_stay_awake (Private Method)

DESCRIPTION:
   run __pm_stay_awake.

PARAMETERS:
   ws                 [ I ] - Pointer to wakeup_source pointer
RETURN VALUE:
   none
===========================================================================*/
void Gobi_pm_stay_awake(struct wakeup_source *ws)
{
   if(ws)
   {
      if(!ws->active)
      {
         WLDEBUG("__pm_stay_awake start\n");
         __pm_stay_awake(ws);
         WLDEBUG( "__pm_stay_awake complete\n");
      }
      PRINT_WS_LOCK(ws);
   }
}

/*===========================================================================
gobiStayAwake

   gobiStayAwake (Private Method)

DESCRIPTION:
   Keep system stay awake not suspend.

PARAMETERS:
   ws                 [ I ] - Pointer to wakeup_source pointer
RETURN VALUE:
   none
===========================================================================*/
void gobiStayAwake(sGobiUSBNet *pGobiDev)
{
   if (GobiTestDownReason( pGobiDev, DRIVER_SUSPENDED ) == false)
   {
      WLDEBUG( "pm_stay_awake\n");
      pm_stay_awake(&pGobiDev->mpIntf->dev);
   }
   if(pGobiDev)
   {
      struct wakeup_source *ws = pGobiDev->ws;
      if(ws)
      {
         Gobi_pm_stay_awake(ws);
      }
   }
}

/*===========================================================================
ProcessUnLockSystemSleepFunction

   ProcessUnLockSystemSleepFunction (Private Method)

DESCRIPTION:
   Work queue handler to release wake lock.

PARAMETERS:
   w                 [ I ] - Pointer to work_struct pointer
RETURN VALUE:
   none
===========================================================================*/
static void ProcessUnLockSystemSleepFunction(struct work_struct *w)
{
   struct delayed_work *dwork;
   sGobiUSBNet *pGobiDev = NULL;
   dwork = to_delayed_work(w);
   pGobiDev = container_of(dwork, sGobiUSBNet, dwUnLockSystemSleep);
   if(pGobiDev!=NULL)
   {
      WLDEBUG( "gobiPmRelax\n");
      gobiPmRelax(pGobiDev);
   }
   else
   {
      DBG("pGobiDev NULL\n");
   }
}

/*===========================================================================
gobiUnLockSystemSleep

   gobiUnLockSystemSleep (Private Method)

DESCRIPTION:
   Add delayed work to wqProcessReadCallback.

PARAMETERS:
   pGobiDev                 [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
   none
===========================================================================*/
void gobiUnLockSystemSleep(sGobiUSBNet *pGobiDev)
{
   WLDEBUG( "gobiUnLockSystemSleep\n");
   GobiCancelUnLockSystemSleepWorkQueue(pGobiDev);
   INIT_DELAYED_WORK(&pGobiDev->dwUnLockSystemSleep,
            ProcessUnLockSystemSleepFunction);
   queue_delayed_work(pGobiDev->wqUnLockSystemSleep, &pGobiDev->dwUnLockSystemSleep, DELAY_MS_DEFAULT);
}
#endif


/*===========================================================================
GenerateProcessName

   GenerateProcessName (Private Method)

DESCRIPTION:
   Generate workqueue name with prefix.

PARAMETERS:
   pPrefix                 [ I ] - work queue name prefix
   szProcessName           [ O ] - strore generated process name
   sizeofName              [ I ] - size of process name
   pGobiDev                [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
   int - 0 Suceesss.
       - -1 Error.
===========================================================================*/
int GenerateProcessName(const char *pPrefix,char *szProcessName,unsigned sizeofName,sGobiUSBNet *pGobiDev )
{
    struct usb_device *dev = NULL;
    if(pGobiDev==NULL)
    {
        return -1;
    }
    dev = interface_to_usbdev(pGobiDev->mUsb_Interface);
    snprintf(szProcessName,sizeofName,"%s-%d-%d-%s:%d.%d",
      pPrefix,
      (int)pGobiDev->mQMIDev.qcqmi,
      dev->bus->busnum, dev->devpath,
      dev->actconfig->desc.bConfigurationValue,
      pGobiDev->mUsb_Interface->cur_altsetting->desc.bInterfaceNumber);
    return 0;
}

/*===========================================================================
GetPrivateWorkQueuesInterfaceTableIndex

   GetPrivateWorkQueuesInterfaceTableIndex (Private Method)

DESCRIPTION:
   Get table interface(GobiPrivateWorkQueues) index by USB interface number.

PARAMETERS:
   pGobiDev                [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
    int - 0 interface 8(default).
        - 1 interface 10.
===========================================================================*/
int GetPrivateWorkQueuesInterfaceTableIndex(sGobiUSBNet *pGobiDev)
{
   if(pGobiDev->mUsb_Interface->cur_altsetting->desc.bInterfaceNumber==10)
      return 1;
   return 0;
}

/*===========================================================================
AddPrivateWorkQueues

   AddPrivateWorkQueues (Private Method)

DESCRIPTION:
   Add pGObiDev USB interface device to GobiPrivateWorkQueues.

PARAMETERS:
   pGobiDev                [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
    int - 0 - MAX_QCQMI-1 GobiPrivateWorkQueues workqueue index.
===========================================================================*/
int AddPrivateWorkQueues(sGobiUSBNet *pGobiDev)
{
   int i=0,j=0;
   char szProcessName[MAX_WQ_PROC_NAME_SIZE];
   memset(&szProcessName,0,sizeof(szProcessName));
   if(pGobiDev->mUsb_Interface->cur_altsetting->desc.bInterfaceNumber==10)
   {
      j = 1;
   }
   if(GenerateProcessName("gobi",&szProcessName[0],MAX_WQ_PROC_NAME_SIZE-1,pGobiDev)!=0)
   {
      return -1;
   }
   i = GetPrivateWorkQueuesIndex(pGobiDev);
   if(i<MAX_QCQMI)
   {
      DBG("%s REUSE %d:%d :%s",__FUNCTION__,i,j,szProcessName);
      return i;
   }
   for(i=0;i<MAX_QCQMI;i++)
   {
      if(strlen(GobiPrivateWorkQueues[i][j].szProcessName)==0)
      {
         strncpy(GobiPrivateWorkQueues[i][j].szProcessName,szProcessName,strlen(szProcessName));
         DBG("%s ADD %d:%d :%s",__FUNCTION__,i,j,szProcessName);
         return i;
      }
   }
   return -1;
}

/*===========================================================================
GetPrivateWorkQueuesIndex

   GetPrivateWorkQueuesIndex (Private Method)

DESCRIPTION:
   Get pGObiDev index in GobiPrivateWorkQueues.

PARAMETERS:
   pGobiDev                [ I ] - Pointer to sGobiUSBNet pointer
RETURN VALUE:
    int - 0 - MAX_QCQMI-1 GobiPrivateWorkQueues workqueue index.
===========================================================================*/
int GetPrivateWorkQueuesIndex(sGobiUSBNet *pGobiDev)
{
   int i=0,j=0;
   char szProcessName[MAX_WQ_PROC_NAME_SIZE];
   memset(&szProcessName,0,sizeof(szProcessName));
   j = GetPrivateWorkQueuesInterfaceTableIndex(pGobiDev);
   if(GenerateProcessName("gobi",&szProcessName[0],MAX_WQ_PROC_NAME_SIZE-1,pGobiDev)!=0)
   {
      return -1;
   }
   for(i=0;i<MAX_QCQMI;i++)
   {
      if(
             (strlen(GobiPrivateWorkQueues[i][j].szProcessName)>0) &&
             (strlen(GobiPrivateWorkQueues[i][j].szProcessName)==strlen(szProcessName)) &&
             (strncmp(GobiPrivateWorkQueues[i][j].szProcessName,szProcessName,strlen(szProcessName))==0)
             )
      {
         DBG("%s FOUND :%s\n",__FUNCTION__,szProcessName);
         return i;
      }
   }
   return i;
}

/*===========================================================================
iClearWorkQueuesByTableIndex

   iClearWorkQueuesByTableIndex (Private Method)

DESCRIPTION:
   Clear index in GobiPrivateWorkQueues device workqueues.

PARAMETERS:
   index                [ I ] - index of GobiPrivateWorkQueues device to be cleared
RETURN VALUE:
    none
===========================================================================*/
int iClearWorkQueuesByTableIndex(int index)
{
   int i=0,j=0;
   i = index;
   if((i>=0)&&(i<MAX_QCQMI))
   {
      for(j=0;j<MAX_QCQMI_PER_INTF;j++)
      {
         if(strlen(GobiPrivateWorkQueues[i][j].szProcessName)>0)
         {
            DBG("%s clear %d:%d :%s",__FUNCTION__,
            i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
         }
         memset(&GobiPrivateWorkQueues[i][j].szProcessName,0,sizeof(GobiPrivateWorkQueues[i][j].szProcessName));
         if(GobiPrivateWorkQueues[i][j].wqprobe!=NULL)
         {
            DBG("%s wqprobe %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            SetPrivateWorkQueuesWQByTableIndex(i,j,NULL,eWQ_PROBE);
         }
         GobiPrivateWorkQueues[i][j].wqprobe=NULL;
         if(GobiPrivateWorkQueues[i][j].wqProcessReadCallback!=NULL)
         {
            DBG("%s wqProcessReadCallback %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            SetPrivateWorkQueuesWQByTableIndex(i,j,NULL,eWQ_URBCB);
         }
         GobiPrivateWorkQueues[i][j].wqProcessReadCallback=NULL;
      }
   }
   return 0;
}

/*===========================================================================
GetPrivateWorkQueuesWQByTableIndex

   GetPrivateWorkQueuesWQByTableIndex (Private Method)

DESCRIPTION:
   Get GobiPrivateWorkQueues workqueue with specific device index(i),
   interface index(j), and work queue type.

PARAMETERS:
   i                 [ I ] - index of GobiPrivateWorkQueues device.
   j                 [ I ] - index of GobiPrivateWorkQueues interface.
   type              [ I ] - work queue type.
RETURN VALUE:
    struct workqueue_struct pointer:
         - NULL fail.
===========================================================================*/
struct workqueue_struct *GetPrivateWorkQueuesWQByTableIndex(int i,int j,int type)
{
   if ((j<0) ||(j>=MAX_QCQMI_PER_INTF))
      return NULL;
   if((i<0)||(i>=MAX_QCQMI))
      return NULL;

   switch(type)
   {
      case eWQ_PROBE:
         if(GobiPrivateWorkQueues[i][j].wqprobe!=NULL)
         {
            return GobiPrivateWorkQueues[i][j].wqprobe;
         }
         break;
      case eWQ_URBCB:
         if(GobiPrivateWorkQueues[i][j].wqProcessReadCallback!=NULL)
         {
            return GobiPrivateWorkQueues[i][j].wqProcessReadCallback;
         }
         break;
      default:
         break;
   }

   return NULL;
}

/*===========================================================================
SetPrivateWorkQueuesWQByTableIndex

   SetPrivateWorkQueuesWQByTableIndex (Private Method)

DESCRIPTION:
   Set GobiPrivateWorkQueues workqueue with specific device index(i),
   interface index(j), and work queue type.

PARAMETERS:
   i                 [ I ] - index of GobiPrivateWorkQueues device.
   j                 [ I ] - index of GobiPrivateWorkQueues interface.
   wq                [ I ] - work queue pointer to be stored in GobiPrivateWorkQueues
   type              [ I ] - work queue type.
RETURN VALUE:
    int - 0 success.
    Negative errno for error
===========================================================================*/
int SetPrivateWorkQueuesWQByTableIndex(int i,int j,struct workqueue_struct *wq, int type)
{
   if ((j<0) ||(j>=MAX_QCQMI_PER_INTF))
      return -1;
   if((i<0)||(i>=MAX_QCQMI))
      return -1;

   switch(type)
   {
      case eWQ_PROBE:
         if(GobiPrivateWorkQueues[i][j].wqprobe!=NULL)
         {
            DBG( "%s wqprobe %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            flush_workqueue(GobiPrivateWorkQueues[i][j].wqprobe);
            DBG( "%s wq wqprobe %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            destroy_workqueue(GobiPrivateWorkQueues[i][j].wqprobe);
         }
         GobiPrivateWorkQueues[i][j].wqprobe=wq;
         break;
      case eWQ_URBCB:
         if(GobiPrivateWorkQueues[i][j].wqProcessReadCallback!=NULL)
         {
            DBG("%s wqProcessReadCallback %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            flush_workqueue(GobiPrivateWorkQueues[i][j].wqProcessReadCallback);
            DBG( "%s wqProcessReadCallback destroy_workqueue %d:%d :%s",
               __FUNCTION__,i,j,
            GobiPrivateWorkQueues[i][j].szProcessName);
            destroy_workqueue(GobiPrivateWorkQueues[i][j].wqProcessReadCallback);
         }
         GobiPrivateWorkQueues[i][j].wqProcessReadCallback=wq;
         break;
      default:
         break;
   }

   return 0;
}

/*===========================================================================
ClearPrivateWorkQueuesProcessByTableIndex

   ClearPrivateWorkQueuesProcessByTableIndex (Private Method)

DESCRIPTION:
   Clear GobiPrivateWorkQueues ProcessName with specific device index(i) and
   interface index(j).

PARAMETERS:
   i                 [ I ] - index of GobiPrivateWorkQueues device.
   j                 [ I ] - index of GobiPrivateWorkQueues interface.
RETURN VALUE:
    int - 0 success.
    Negative errno for error
===========================================================================*/
int ClearPrivateWorkQueuesProcessByTableIndex(int i,int j)
{
    DBG("%s %d:%d ",
               __FUNCTION__,i,j)
   if ((j<0) ||(j>=MAX_QCQMI_PER_INTF))
      return -1;
   if((i<0)||(i>=MAX_QCQMI))
      return -1;
   if( (GobiPrivateWorkQueues[i][j].wqProcessReadCallback==NULL) &&
       (GobiPrivateWorkQueues[i][j].wqprobe==NULL) )
   {
      memset(&GobiPrivateWorkQueues[i][j].szProcessName,0,
         sizeof(GobiPrivateWorkQueues[i][j].szProcessName));
   }
   return 0;
}

/*===========================================================================
GobiCancelReadCallBackWorkQueue

   GobiCancelReadCallBackWorkQueue (Private Method)

DESCRIPTION:
   Cancel device ReadCallBack work queue.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
RETURN VALUE:
    none
===========================================================================*/
void GobiCancelReadCallBackWorkQueue(sGobiUSBNet *pGobiDev)
{
   if( (pGobiDev != NULL) &&
      (pGobiDev->wqProcessReadCallback != NULL))
   {

      DBG("%s\n",__FUNCTION__);
      GobiCancelDelayWorkWorkQueue(pGobiDev,
         pGobiDev->wqProcessReadCallback,
         &pGobiDev->dwProcessReadCallback);
   }
}
#ifdef CONFIG_ANDROID
/*===========================================================================
GobiCancelReadCallBackWorkQueue

   GobiCancelLockSystemSleepWorkQueue (Private Method)

DESCRIPTION:
   Cancel device LockSystemSleep work queue.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
RETURN VALUE:
    none
===========================================================================*/
void GobiCancelLockSystemSleepWorkQueue(sGobiUSBNet *pGobiDev)
{
   if( (pGobiDev != NULL) &&
      (pGobiDev->wqLockSystemSleep != NULL))
   {

      DBG("%s\n",__FUNCTION__);
      GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice(pGobiDev,
         pGobiDev->wqLockSystemSleep,
         &pGobiDev->dwLockSystemSleep);
   }
}

/*===========================================================================
GobiCancelUnLockSystemSleepWorkQueue

   GobiCancelUnLockSystemSleepWorkQueue (Private Method)

DESCRIPTION:
   Cancel device LockSystemSleep work queue.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
RETURN VALUE:
    none
===========================================================================*/
void GobiCancelUnLockSystemSleepWorkQueue(sGobiUSBNet *pGobiDev)
{
   if( (pGobiDev != NULL) &&
      (pGobiDev->wqUnLockSystemSleep != NULL))
   {

      DBG("%s\n",__FUNCTION__);
      GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice(pGobiDev,
         pGobiDev->wqUnLockSystemSleep,
         &pGobiDev->dwUnLockSystemSleep);
   }
}
#endif
/*===========================================================================
METHOD:
   TransceiveReleaseClientID (Private Method)

DESCRIPTION:
   Send/Receive Release QMI client control message.

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   true - 0 success.
   false - on error.
===========================================================================*/
bool TransceiveReleaseClientID(
   sGobiUSBNet *    pDev,
   u16                clientID)
{
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer = NULL;
   u16 readBufferSize;
   u8 transactionID;
   int result;
   unsigned long flags = 0;
   struct semaphore readSem;
   bool bRet = true;
   // Is device is still valid?
   DBG("clientID:0x%x\n",clientID);
   if (pDev->mbUnload > eStatUnloaded)
   {
      DBG( "unloaded\n" );
      return false;
   }
   // Run QMI ReleaseClientID if this isn't QMICTL
   if(IsDeviceDisconnect(pDev)==true)
      return false;
   barrier();
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   if ((clientID != QMICTL) && (pDev->mReleaseClientIDFail==0))
   {
      // Note: all errors are non fatal, as we always want to delete
      //    client memory in latter part of function

      writeBufferSize = QMICTLReleaseClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GOBI_GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         DBG( "memory error\n" );
         return false;
      }
      else
      {
         transactionID = QMIXactionIDGet(pDev);
         result = QMICTLReleaseClientIDReq( pWriteBuffer,
                                            writeBufferSize,
                                            transactionID,
                                            clientID );
         if (result < 0)
         {
            kfree( pWriteBuffer );
            DBG( "error %d filling req buffer\n", result );
         }
         else
         {
            mb();
            result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem ,1);
            if(result == 0)
            {
               result = WriteSyncNoRetry( pDev,
                             pWriteBuffer,
                             writeBufferSize,
                             QMICTL );
               kfree( pWriteBuffer );
               if(result<0)
               {
                  DBG( " WriteSyncNoRetry error %d\n", result );
                  pDev->mReleaseClientIDFail = 1;
                  bRet = false;
                  flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
                  RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
                  LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               }
               else
               {
                  wait_control_msg_semaphore_timeout(&readSem,QMI_CONTROL_MAX_MSG_DELAY_MS);
                  mb();
                  // Enter critical section
                  flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
                  barrier();
                  spin_lock_irq(&(pDev->notif_lock));
                  if (down_trylock( &readSem ) == 0)
                  {
                     // Pop the read data
                     if (PopFromReadMemList( pDev,
                                             QMICTL,
                                             transactionID,
                                             &pReadBuffer,
                                             &readBufferSize ) == true)
                     {
                        // End critical section
                        result = QMICTLReleaseClientIDResp(pReadBuffer,
                                                              readBufferSize);
                        if (result < 0)
                        {
                           DBG( "error %d parsing response\n", result );
                           RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
                        }
                        // We don't care about the result
                        if(pReadBuffer)
                        kfree( pReadBuffer );
                     }
                     else
                     {
                        // Read mismatch/failure, unlock and continue
                        RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
                     }
                  }
                  else
                  {
                     DBG( "Lock Timeout\n" );
                     // Timeout, remove the async read
                     RemoveAndPopNotifyList(pDev,QMICTL,0,eClearCID);
                     // End critical section
                  }
                  spin_unlock_irq(&(pDev->notif_lock));
                  LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               }
            }
            else
            {
                if(pWriteBuffer)
                {
                   kfree( pWriteBuffer );
                   pWriteBuffer = NULL;
                }
            }
         }
      }
   }
   return bRet;
}

/*===========================================================================
GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice


   GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice (Private Method)

DESCRIPTION:
   Cancel work queue and delayed work.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
   wq                [ I ] - pointer to workqueue_struct.
   dw                [ I ] - pointer to delayed_work.

RETURN VALUE:
    none
===========================================================================*/
void GobiCancelDelayWorkWorkQueueWithoutUSBLockDevice(
   sGobiUSBNet *pGobiDev,
   struct workqueue_struct *wq,
   struct delayed_work *dw)
{
   if( (pGobiDev != NULL) &&
      (wq != NULL) &&
      (dw != NULL) )
   {
      struct usb_device *dev = NULL;
      unsigned int flag = 0;
      if( pGobiDev->mUsb_Interface == NULL )
      {
         return ;
      }
      dev = interface_to_usbdev(pGobiDev->mUsb_Interface);
      flag = gobi_work_busy(dw);
      if(flag)
      {
         #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,36 ))
         if(flag & WORK_BUSY_RUNNING)
         {
            DBG("flush_delayed_work %d\n",
                pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
            flush_delayed_work(dw);
         }
         #endif
         if(cancel_delayed_work (dw))
         {
            DBG("flush_workqueue %d\n",
                pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
            flush_workqueue(wq);
         }
      }
      else
      {
         DBG("flush_work \n %d\n",
            pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
         flush_work(&dw->work);
      }
   }
}

/*===========================================================================
GobiCancelDelayWorkWorkQueue

   GobiCancelDelayWorkWorkQueue (Private Method)

DESCRIPTION:
   Cancel work queue and delayed work.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
   wq                [ I ] - pointer to workqueue_struct.
   dw                [ I ] - pointer to delayed_work.

RETURN VALUE:
    none
===========================================================================*/
void GobiCancelDelayWorkWorkQueue(
   sGobiUSBNet *pGobiDev,
   struct workqueue_struct *wq,
   struct delayed_work *dw)
{
   if( (pGobiDev != NULL) &&
      (wq != NULL) &&
      (dw != NULL) )
   {

      int ret = 0;
      struct usb_device *dev = NULL;
      unsigned int flag = 0;
      if( pGobiDev->mUsb_Interface == NULL )
      {
         return ;
      }
      dev = interface_to_usbdev(pGobiDev->mUsb_Interface);
      ret = usb_lock_device_for_reset(dev, NULL);
      if( (ret==0) ||
          (pGobiDev->iIsUSBReset==false) )
      {
         if(ret==0)
         {
            //Prevent Deadlock GobiUSBLockReset
            usb_unlock_device(dev);
         }
         flag = gobi_work_busy(dw);
         if(flag)
         {
            #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,36 ))
            if(flag & WORK_BUSY_RUNNING)
            {
               DBG("flush_delayed_work %d\n",
                   pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
               flush_delayed_work(dw);
            }
            #endif
            if(cancel_delayed_work (dw))
            {
               DBG("flush_workqueue %d\n",
                   pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
               flush_workqueue(wq);
            }
         }
         else
         {
            DBG("flush_work \n %d\n",
               pGobiDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);
            flush_work(&dw->work);
         }
      }
   }
}

/*===========================================================================
GobiCancelProbeWorkQueue

   GobiCancelProbeWorkQueue (Private Method)

DESCRIPTION:
   Cancel device Probe work queue.

PARAMETERS:
   pGobiDev          [ I ] - pointer to sGobiUSBNet.
RETURN VALUE:
    none
===========================================================================*/
void GobiCancelProbeWorkQueue(sGobiUSBNet *pGobiDev)
{
   if( (pGobiDev != NULL) &&
      (pGobiDev->wqprobe != NULL))
   {

      DBG("%s\n",__FUNCTION__);
      GobiCancelDelayWorkWorkQueue(pGobiDev,
         pGobiDev->wqprobe,
         &pGobiDev->dwprobe);
   }
}
#ifdef CONFIG_ANDROID
//Don't do this before SetCurrentSuspendStat
void SetTxRxStat(sGobiUSBNet *pGobiDev,int state)
{
   unsigned long flags = 0;
   spin_lock_irqsave(&pGobiDev->sSuspendLock,flags);
   if(state==RESUME_TX_RX_DISABLE)
   {
      pGobiDev->iSuspendReadWrite = RESUME_TX_RX_DISABLE;
   }
   else
   {
      pGobiDev->iSuspendReadWrite |= state;
      DBG("%s ON\n",(state==RESUME_RX_OKAY)? "Rx" : "Tx");
   }
   spin_unlock_irqrestore(&pGobiDev->sSuspendLock,flags);

}

int GetTxRxStat(sGobiUSBNet *pGobiDev,int Channel)
{
   if(Channel & RESUME_RX_OKAY)
   {
      return (pGobiDev->iSuspendReadWrite & RESUME_RX_OKAY);
   }
   if(Channel & RESUME_TX_OKAY)
   {
      return (pGobiDev->iSuspendReadWrite & RESUME_TX_OKAY);
   }
   DBG( "%s OFF\n",(Channel==RESUME_RX_OKAY)? "Rx" : "Tx");
   return 0;
}
#endif
