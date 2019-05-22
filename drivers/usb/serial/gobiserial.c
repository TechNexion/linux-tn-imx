/*===========================================================================
FILE:
   GobiSerial.c

DESCRIPTION:
   Linux Qualcomm Serial USB driver Implementation

PUBLIC DRIVER FUNCTIONS:
   GobiProbe
   GobiOpen
   GobiClose
   GobiReadBulkCallback
   GobiSerialSuspend
   GobiSerialResume (if kernel is less than 2.6.24)

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
==========================================================================*/
//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/serial.h>

/* Determine if we are in a particular kernel series */
#define KERNEL_SERIES(x, y) \
	((LINUX_VERSION_CODE >> 8) == (KERNEL_VERSION( x,y,0 ) >> 8))

//---------------------------------------------------------------------------
// Global variables and definitions
//---------------------------------------------------------------------------

// Version Information
#define DRIVER_VERSION "2019-05-03/SWI_2.37"
#define DRIVER_AUTHOR "Qualcomm Innovation Center"
#define DRIVER_DESC "GobiSerial"

#define NUM_BULK_EPS         1
#define MAX_BULK_EPS         6

#define SWIMS_USB_REQUEST_SetHostPower	0x09
#define SET_CONTROL_LINE_STATE_REQUEST_TYPE        0x21
#define SET_CONTROL_LINE_STATE_REQUEST             0x22
#define CONTROL_DTR                     0x01
#define CONTROL_RTS                     0x02


// Debug flag
static int debug;
// flow control flag
static int flow_control = 1;
// allow port open to success even when GPS control message failed
static int ignore_gps_start_error = 1;
#define OPEN_GPS_DELAY_IN_SECOND 10
// Open GPS port delay
static int delay_open_gps_port = OPEN_GPS_DELAY_IN_SECOND;
// Number of serial interfaces
static int nNumInterfaces;
// Enable Zero Lenght Payload on USB3 in QDL Mode
static int iusb3_zlp_enable = 1;

// Global pointer to usb_serial_generic_close function
// This function is not exported, which is why we have to use a pointer
// instead of just calling it.
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   void (* gpClose)(
      struct usb_serial_port *,
      struct file * );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   void (* gpClose)(
      struct tty_struct *,
      struct usb_serial_port *,
      struct file * );
#else // > 2.6.30
   void (* gpClose)( struct usb_serial_port * );
#endif

// DBG macro
#define DBG( format, arg... ) \
   if (debug == 1)\
   { \
      printk( KERN_INFO "GobiSerial::%s " format, __FUNCTION__, ## arg ); \
   } \

struct gobi_serial_intf_private {
       spinlock_t susp_lock;
       unsigned int suspended:1;
       struct sierra_port_private *pPortdata;
};

/*=========================================================================*/
// Function Prototypes
/*=========================================================================*/

// Attach to correct interfaces
static int GobiProbe(
   struct usb_serial * pSerial,
   const struct usb_device_id * pID );

// Start GPS if GPS port, run usb_serial_generic_open
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   int GobiOpen(
      struct usb_serial_port *   pPort,
      struct file *              pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
   int GobiOpen(
      struct tty_struct *        pTTY,
      struct usb_serial_port *   pPort,
      struct file *              pFilp );
#else // > 2.6.31
   int GobiOpen(
      struct tty_struct *        pTTY,
      struct usb_serial_port *   pPort );
#endif

// Stop GPS if GPS port, run usb_serial_generic_close
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   void GobiClose(
      struct usb_serial_port *,
      struct file * );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   void GobiClose(
      struct tty_struct *,
      struct usb_serial_port *,
      struct file * );
#else // > 2.6.30
   void GobiClose( struct usb_serial_port * );
#endif

// Read data from USB, push to TTY and user space
static void GobiReadBulkCallback( struct urb * pURB );

// Set reset_resume flag
int GobiSerialSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent );
int GobiUSBSerialSuspend(struct usb_serial *serial, pm_message_t message);
int GobiUSBSerialResume(struct usb_serial *serial);
int GobiUSBSerialResetResume(struct usb_serial *serial);
void GobiUSBSerialDisconnect(struct usb_serial * serial);
static int Gobi_write(struct tty_struct *tty, struct usb_serial_port *port,
               const unsigned char *buf, int count);
bool IsDeviceUnbinding(struct usb_serial *serial);
void GobiUSBSendZeroConfigMsg(struct usb_serial *serial);
int gobi_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
   void *data, int len, int *actual_length,
   int timeout);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))

// Restart URBs killed during usb_serial_suspend
int GobiSerialResume( struct usb_interface * pIntf );

#endif
void stop_read_write_urbs(struct usb_serial *serial);


#define MDM9X15_DEVICE(vend, prod) \
	USB_DEVICE(vend, prod), \
	.driver_info = BIT(1) | BIT(8) | BIT(10) | BIT(11)

#define G3K_DEVICE(vend, prod) \
	USB_DEVICE(vend, prod), \
	.driver_info = BIT(0)

#if ((KERNEL_SERIES( 3,4 ) && LINUX_VERSION_CODE >= KERNEL_VERSION( 3,4,34 )) || \
     (KERNEL_SERIES( 3,2 ) && LINUX_VERSION_CODE >= KERNEL_VERSION( 3,2,0 )) || \
     (KERNEL_SERIES( 3,5 ) && LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 )) || \
     (KERNEL_SERIES( 3,7 ) && LINUX_VERSION_CODE >= KERNEL_VERSION( 3,7,10 )) || \
     (KERNEL_SERIES( 3,8 ) && LINUX_VERSION_CODE >= KERNEL_VERSION( 3,8,1) ) || \
     (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,8,1) ))
/* workaround for upstream commit b2ca699076573c94fee9a73cb0d8645383b602a0 */
#warning "Assuming disc_mutex is locked external to the module"
static inline void Gobi_lock_disc_mutex(struct usb_serial *serial) {
   if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,12,0) )
   {
      WARN_ON(!mutex_is_locked(&serial->disc_mutex));
   }
}
static inline void Gobi_unlock_disc_mutex(struct usb_serial *serial) {}
#else
/* use the legacy method of locking disc_mutex in this driver */
#warning "Using legacy method of locking disc_mutex"
static inline void Gobi_lock_disc_mutex(struct usb_serial *serial) {
   mutex_lock(&serial->disc_mutex);
}
static inline void Gobi_unlock_disc_mutex(struct usb_serial *serial) {
   mutex_unlock(&serial->disc_mutex);
}
#endif

/*=========================================================================*/
// Qualcomm Gobi 3000 VID/PIDs
/*=========================================================================*/
static struct usb_device_id GobiVIDPIDTable[] =
{
   { USB_DEVICE(0x05c6, 0x920c) },   // Gobi 3000 QDL
   { USB_DEVICE(0x05c6, 0x920d) },   // Gobi 3000 Composite
   /* Sierra Wireless QMI VID/PID */
   { USB_DEVICE(0x1199, 0x68A2),
      .driver_info = BIT(8) | BIT(19) | BIT(20) |
                     BIT(10) | BIT(11) /* in case a MDM9x15 switched to 0x68a2 */
   },
   /* Sierra Wireless QMI MC78/WP7/AR7 */
   { USB_DEVICE(0x1199, 0x68C0),
      /* blacklist the interface */
      /* whitelist interfaces 5 for raw data */
      .driver_info = BIT(1) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI MC74xx/EM74xx */
   { USB_DEVICE(0x1199, 0x9071),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(4) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI MC74xx/EM74xx */
   { USB_DEVICE(0x1199, 0x9070),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(4) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI AR759x */
   { USB_DEVICE(0x1199, 0x9100),
      /* blacklist the interface */
      /* whitelist interfaces 5 & 6 for raw data and open sim resp. */
      .driver_info = BIT(1) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI AR758x */
   { USB_DEVICE(0x1199, 0x9102),
      /* blacklist the interface */
      /* whitelist interfaces 5 & 6 for raw data and open sim resp. */
      .driver_info = BIT(1) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI AR759x */
   { USB_DEVICE(0x1199, 0x9110),
      /* blacklist the interface */
      /* whitelist interfaces 5 & 6 for raw data and open sim resp. */
      .driver_info = BIT(1) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   /* Sierra Wireless QMI MC75xx/EM75xx BOOT*/
   { USB_DEVICE(0x1199, 0x9090),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },
   { USB_DEVICE(0x1199, 0x9091),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },
   { USB_DEVICE(0x1199, 0x90b0),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },
   { USB_DEVICE(0x1199, 0x90b1),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },
   { USB_DEVICE(0x1199, 0x90c0),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },
   { USB_DEVICE(0x1199, 0x90c1),
      /* blacklist the interface */
      .driver_info = BIT(1) | BIT(5) | BIT(6) | BIT(8) | BIT(10) | BIT(11) | BIT(12) | BIT(13)
   },

   {G3K_DEVICE(0x1199, 0x9010)},
   {G3K_DEVICE(0x1199, 0x9011)},
   {G3K_DEVICE(0x1199, 0x9012)},
   {G3K_DEVICE(0x1199, 0x9013)},
   {G3K_DEVICE(0x1199, 0x9014)},
   {G3K_DEVICE(0x1199, 0x9015)},
   {G3K_DEVICE(0x1199, 0x9018)},
   {G3K_DEVICE(0x1199, 0x9019)},
   {G3K_DEVICE(0x03F0, 0x361D)},
   {G3K_DEVICE(0x03F0, 0x371D)},

   {MDM9X15_DEVICE(0x1199, 0x9040)},
   {MDM9X15_DEVICE(0x1199, 0x9041)},
   {MDM9X15_DEVICE(0x1199, 0x9051)},
   {MDM9X15_DEVICE(0x1199, 0x9053)},
   {MDM9X15_DEVICE(0x1199, 0x9054)},
   {MDM9X15_DEVICE(0x1199, 0x9055)},
   {MDM9X15_DEVICE(0x1199, 0x9056)},
   {MDM9X15_DEVICE(0x1199, 0x9060)},
   {MDM9X15_DEVICE(0x1199, 0x9061)},

   { }  // Terminating entry
};
MODULE_DEVICE_TABLE( usb, GobiVIDPIDTable );

enum {
    eSendUnknown=-1,
    eSendStart=0,
    eSendEnd=1,
};
/* per port private data */
struct sierra_port_private {
   /* Settings for the port */
   int rts_state;    /* Handshaking pins (outputs) */
   int dtr_state;
   int isClosing;
   int iGPSStartState;
   unsigned long ulExpires;
   int iUsb3ZlpEnable;
};

int gobi_usb_serial_generic_resume(struct usb_interface *intf)
{
   struct usb_serial *serial = usb_get_intfdata(intf);
   return usb_serial_generic_resume(serial);
}

/*=========================================================================*/
// Struct usb_serial_driver
// Driver structure we register with the USB core
/*=========================================================================*/
static struct usb_driver GobiDriver =
{
   .name       = "GobiSerial",
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,5,0 ))
   .probe      = usb_serial_probe,
   .disconnect = usb_serial_disconnect,
#endif
   .id_table   = GobiVIDPIDTable,
#ifdef CONFIG_PM
   .suspend    = usb_serial_suspend,
   .resume = gobi_usb_serial_generic_resume,
   .reset_resume = gobi_usb_serial_generic_resume,
   .supports_autosuspend = true,
#else
   .suspend    = NULL,
   .resume     = NULL,
   .supports_autosuspend = false,
#endif
};

static int Gobi_calc_interface(struct usb_serial *serial)
{
   int interface;
   struct usb_interface *p_interface;
   struct usb_host_interface *p_host_interface;
   dev_dbg(&serial->dev->dev, "%s\n", __func__);

   /* Get the interface structure pointer from the serial struct */
   p_interface = serial->interface;

   /* Get a pointer to the host interface structure */
   p_host_interface = p_interface->cur_altsetting;

   /* read the interface descriptor for this active altsetting
    * to find out the interface number we are on
    */
   interface = p_host_interface->desc.bInterfaceNumber;

   return interface;
}

static int Gobi_send_setup(struct usb_serial_port *port)
{
   struct usb_serial *serial = port->serial;
   struct sierra_port_private *portdata;
   __u16 interface = 0;
   int val = 0;
   int retval;

   dev_dbg(&port->dev, "%s\n", __func__);

   portdata = usb_get_serial_port_data(port);

   if (portdata->dtr_state)
      val |= CONTROL_DTR;
   if (portdata->rts_state)
      val |= CONTROL_RTS;

   /* obtain interface for usb control message below */
   if (serial->num_ports == 1) {
      interface = Gobi_calc_interface(serial);
   }
   else {
      dev_err(&port->dev,
            "flow control is not supported for %d serial port\n",
            serial->num_ports);
      return -ENODEV;
   }

   retval = usb_autopm_get_interface(serial->interface);
   if (retval < 0)
   {
      return retval;
   }

   retval = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
         SET_CONTROL_LINE_STATE_REQUEST,
         SET_CONTROL_LINE_STATE_REQUEST_TYPE,
         val, interface, NULL, 0, USB_CTRL_SET_TIMEOUT);
   usb_autopm_put_interface(serial->interface);

   return retval;
}

static void Gobi_dtr_rts(struct usb_serial_port *port, int on)
{
   struct usb_serial *serial = port->serial;
   struct sierra_port_private *portdata;

   portdata = usb_get_serial_port_data(port);
   portdata->rts_state = on;
   portdata->dtr_state = on;

   /* only send down the usb control message if enabled */
   if (serial->dev && flow_control) {
      Gobi_lock_disc_mutex(serial);
      if (!serial->disconnected)
      {
         Gobi_send_setup(port);
      }
      Gobi_unlock_disc_mutex(serial);
   }
}

/*
 * swiState
 * 0x0000   Host device is awake
 * 0x0001   Host device is suspended
 */
static int set_power_state(struct usb_device *udev, __u16 swiState)
{
    int result;
    dev_dbg(&udev->dev, "%s\n", __func__);
    result = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
            SWIMS_USB_REQUEST_SetHostPower, /* __u8 request      */
            USB_TYPE_VENDOR,                /* __u8 request type */
            swiState,                       /* __u16 value       */
            0,                              /* __u16 index       */
            NULL,                           /* void *data        */
            0,                              /* __u16 size        */
            USB_CTRL_SET_TIMEOUT);          /* int timeout       */
    return result;
}

static int Gobi_startup(struct usb_serial *serial)
{
   struct usb_serial_port *port = NULL;
   struct sierra_port_private *portdata = NULL;
   struct gobi_serial_intf_private *intfdata = NULL;
   int i;

   dev_dbg(&serial->dev->dev, "%s\n", __func__);

   if (serial->num_ports) {
      /* Note: One big piece of memory is allocated for all ports
       * private data in one shot. This memory is split into equal
       * pieces for each port.
       */
      portdata = (struct sierra_port_private *)kzalloc
         (sizeof(*portdata) * serial->num_ports, GFP_KERNEL);
      if (!portdata) {
         dev_dbg(&serial->dev->dev, "%s: No memory!\n", __func__);
         return -ENOMEM;
      }
   }
   intfdata = usb_get_serial_data(serial);
   if(intfdata)
   {
      intfdata->pPortdata = portdata;
   }
   else
   {
      kfree(portdata);
      return -ENOMEM;
   }
   /* Now setup per port private data */
   for (i = 0; i < serial->num_ports; i++, portdata++) {
      struct sierra_port_private *privatedata;
      privatedata = portdata;
      privatedata->iGPSStartState = eSendUnknown;
      privatedata->ulExpires = jiffies + msecs_to_jiffies(delay_open_gps_port*1000);
      port = serial->port[i];
      privatedata->iUsb3ZlpEnable = iusb3_zlp_enable;
      /* Set the port private data pointer */
      usb_set_serial_port_data(port, portdata);
   }

   return 0;
}

int Gobi_portremove(struct usb_serial_port *port)
{

   struct sierra_port_private *portdata = usb_get_serial_port_data(port);
   struct gobi_serial_intf_private *intfdata = NULL;
   u8 port_number;
   usb_set_serial_port_data(port, NULL);
   intfdata = usb_get_serial_data(port->serial);
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,11,0 ))
   port_number = (u8)port->port_number;
   #else
   port_number = (u8)port->number;
   #endif
   if(port_number==0)
   {
      if(intfdata)
      {
         intfdata->pPortdata = NULL;
      }
      if(portdata)
      kfree(portdata);
   }
   return 0;
}

void GobiUSBSendZeroConfigMsg(struct usb_serial *serial)
{
   int ret = 0;
   DBG("\n");
   if(!serial)
   {
      return ;
   }
   if(!serial->dev)
   {
      return ;
   }
   ret = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
   USB_REQ_SET_CONFIGURATION, 0,
   serial->dev->actconfig->desc.bConfigurationValue, 0,
   NULL, 0, USB_CTRL_SET_TIMEOUT);
   if (ret < 0)
   {
      printk( KERN_INFO "Send ZERO CONF FAIL!\n" );
   }
}

bool IsDeviceUnbinding(struct usb_serial *serial)
{
   struct usb_device *udev = NULL;
   struct usb_interface *interface = NULL;
   DBG("\n");
   if((!serial)||
      (!serial->interface) ||
      (!serial->interface->cur_altsetting))
   {
      return false;
   }
   #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,33 ))
   if(serial->interface->resetting_device)
   {
      return false;
   }
   #endif
   if (serial->num_ports < 1)
   {
      return false;
   }
   interface = serial->interface;
   if(!serial->dev)
   {
      return false;
   }
   udev = serial->dev;
   if((!udev)||
      (!udev->parent))
   {
      return false;
   }
   if ((udev->parent->state == USB_STATE_NOTATTACHED )||
      (udev->state == USB_STATE_NOTATTACHED ))
   {
      return false;
   }
   if(interface->cur_altsetting->desc.bInterfaceNumber!=0)
   {
      return false;
   }
   if(interface->condition == USB_INTERFACE_UNBINDING)
   {
      return true;
   }
   return false;
}

static void Gobi_release(struct usb_serial *serial)
{
   int i;
   struct usb_serial_port *port;
   struct sierra_port_private *portdata;
   struct gobi_serial_intf_private *intfdata = NULL;
   if(!serial)
      return ;
   if(!serial->dev)
      return ;
   dev_dbg(&serial->dev->dev, "%s\n", __func__);
   //Set USB CONFIGURATION to ZERO
   if(IsDeviceUnbinding(serial))
   {
      GobiUSBSendZeroConfigMsg(serial);
   }
   intfdata = usb_get_serial_data(serial);
   usb_set_serial_data(serial,NULL);
   stop_read_write_urbs(serial);
   if (serial->num_ports > 0) {
      port = serial->port[0];
      if (port)
      {
         /* Note: The entire piece of memory that was allocated
          * in the startup routine can be released by passing
          * a pointer to the beginning of the piece.
          * This address corresponds to the address of the chunk
          * that was given to port 0.
          */
         portdata = usb_get_serial_port_data(port);
         usb_set_serial_port_data(port, NULL);
         if(portdata)
         {
            intfdata->pPortdata = NULL;
            kfree(portdata);
         }
         else
         {
            if(intfdata)
            {
               portdata = intfdata->pPortdata;
               if(portdata)
               {
                  kfree(portdata);
               }
               intfdata->pPortdata = NULL;
            }
         }
      }
   }

   for (i = 0; i < serial->num_ports; ++i) {
      port = serial->port[i];
      if (!port)
      {
         continue;
      }
      usb_set_serial_port_data(port, NULL);
   }
   if(intfdata)
   {
      kfree(intfdata);
   }
}

/*=========================================================================*/
// Struct usb_serial_driver
/*=========================================================================*/
static struct usb_serial_driver gGobiDevice =
{
   .driver =
   {
      .owner     = THIS_MODULE,
      .name      = "GobiSerial driver",
   },
   .description         = "GobiSerial",
   .id_table            = GobiVIDPIDTable,
   .usb_driver          = &GobiDriver,
   .num_ports           = NUM_BULK_EPS,
   .probe               = GobiProbe,
   .open                = GobiOpen,
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))
   .num_interrupt_in    = NUM_DONT_CARE,
   .num_bulk_in         = 1,
   .num_bulk_out        = 1,
#endif
   .write                 = Gobi_write,

   /* TODO PowerPC RD1020DB kernel 3.0 support
    */
   /* register read_bulk_callback in order to resume upon -EPROTO
    */
   .read_bulk_callback  = GobiReadBulkCallback,
   .dtr_rts             = Gobi_dtr_rts,
   .attach              = Gobi_startup,
   .port_remove         = Gobi_portremove,
   .release             = Gobi_release,
   .disconnect           = GobiUSBSerialDisconnect,
#ifdef CONFIG_PM
    .suspend    = GobiUSBSerialSuspend,
    .resume     = GobiUSBSerialResume,
     #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 ))
    .reset_resume = GobiUSBSerialResetResume,
    #endif
 #endif
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,4,0 ))
static struct usb_serial_driver * const serial_drivers[] = {
   &gGobiDevice, NULL
};
#endif

//---------------------------------------------------------------------------
// USB serial core overridding Methods
//---------------------------------------------------------------------------
/*===========================================================================
METHOD:
   GobiProbe (Free Method)

DESCRIPTION:
   Attach to correct interfaces

PARAMETERS:
   pSerial    [ I ] - Serial structure
   pID        [ I ] - VID PID table

RETURN VALUE:
   int - negative error code on failure
         zero on success
===========================================================================*/
static int GobiProbe(
   struct usb_serial * pSerial,
   const struct usb_device_id * pID )
{
   // Assume failure
   int nRetval = -ENODEV;
   int nInterfaceNum;
   struct usb_host_endpoint * pEndpoint;
   int endpointIndex;
   int numEndpoints;
   struct gobi_serial_intf_private *intfdata;

   DBG( "\n" );

   // Test parameters
   if ( (pSerial == NULL)
   ||   (pSerial->dev == NULL)
   ||   (pSerial->dev->actconfig == NULL)
   ||   (pSerial->interface == NULL)
   ||   (pSerial->interface->cur_altsetting == NULL)
   ||   (pSerial->type == NULL) )
   {
      DBG( "invalid parameter\n" );
      return -EINVAL;
   }


   intfdata = kzalloc(sizeof(*intfdata), GFP_KERNEL);
   if (!intfdata)
           return -ENOMEM;

   spin_lock_init(&intfdata->susp_lock);

   usb_set_serial_data(pSerial, intfdata);

   nNumInterfaces = pSerial->dev->actconfig->desc.bNumInterfaces;
   DBG( "Num Interfaces = %d\n", nNumInterfaces );
   nInterfaceNum = pSerial->interface->cur_altsetting->desc.bInterfaceNumber;
   DBG( "This Interface = %d\n", nInterfaceNum );
   #ifdef CONFIG_PM
   pSerial->dev->reset_resume = 0;
   #endif

   if (nNumInterfaces == 1)
   {
      // QDL mode?
      if ((nInterfaceNum == 0) || (nInterfaceNum == 1))
      {
         DBG( "QDL port found\n" );
         nRetval = usb_set_interface( pSerial->dev,
                                      nInterfaceNum,
                                      0 );
         if (nRetval < 0)
         {
            DBG( "Could not set interface, error %d\n", nRetval );
         }
      }
      else
      {
         DBG( "Incorrect QDL interface number\n" );
      }
   }
   else if (nNumInterfaces > 1)
   {
      /* Composite mode */
      if (pSerial->interface->cur_altsetting->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC )
      {
         DBG( "Ignoring non vendor class interface #%d\n", nInterfaceNum );
         usb_set_serial_data(pSerial, NULL);
         kfree(intfdata);
         return -ENODEV;
      }
      else if (pID->driver_info &&
             test_bit(nInterfaceNum, &pID->driver_info)) {
         DBG( "Ignoring blacklisted interface #%d\n", nInterfaceNum );
         usb_set_serial_data(pSerial, NULL);
         kfree(intfdata);
         return -ENODEV;
      }
      else
      {
         nRetval = usb_set_interface( pSerial->dev,
                                      nInterfaceNum,
                                      0 );
         if (nRetval < 0)
         {
            DBG( "Could not set interface, error %d\n", nRetval );
         }

         // Check for recursion
         if (pSerial->type->close != GobiClose)
         {
            // Store usb_serial_generic_close in gpClose
            gpClose = pSerial->type->close;
            pSerial->type->close = GobiClose;
         }
      }
   }
   if (nRetval == 0 && nNumInterfaces > 1 )
   {
      // Clearing endpoint halt is a magic handshake that brings
      // the device out of low power (airplane) mode
      // NOTE: FCC verification should be done before this, if required
      numEndpoints = pSerial->interface->cur_altsetting
                         ->desc.bInterfaceNumber;

      for (endpointIndex = 0; endpointIndex < numEndpoints; endpointIndex++)
      {
         pEndpoint = pSerial->interface->cur_altsetting->endpoint
                   + endpointIndex;

         if (pEndpoint != NULL
         &&  usb_endpoint_dir_out( &pEndpoint->desc ) == true)
         {
            int pipe = usb_sndbulkpipe( pSerial->dev,
                                        pEndpoint->desc.bEndpointAddress );
            nRetval = usb_clear_halt( pSerial->dev, pipe );

            // Should only be one
            break;
         }
      }
   }

   return nRetval;
}

/*===========================================================================
METHOD:
   IsGPSPort (Free Method)

DESCRIPTION:
   Determines whether the interface is GPS port

PARAMETERS:
   pPort   [ I ] - USB serial port structure

RETURN VALUE:
   bool- true if this is a GPS port
       - false otherwise
===========================================================================*/
bool IsGPSPort(struct usb_serial_port *   pPort )
{
   DBG( "Product=0x%x, Interface=0x%x\n",
        cpu_to_le16(pPort->serial->dev->descriptor.idProduct),
        pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber);

   switch (cpu_to_le16(pPort->serial->dev->descriptor.idProduct))
   {
      case 0x68A2:  /* Sierra Wireless QMI */
      case 0x68C0:
      case 0x9041:
      case 0x9071:
      case 0x9070:
      //9x50
      case 0x9090:
      //9x50
      case 0x9091:
      case 0x90b0:
      case 0x90b1:
      case 0x9100:
      case 0x9102:
      case 0x9110:
         if (pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber == 2)
            return true;
         break;

      case 0x9011:  /* Sierra Wireless G3K */
      case 0x9013:  /* Sierra Wireless G3K */
      case 0x9015:  /* Sierra Wireless G3K */
      case 0x9019:  /* Sierra Wireless G3K */
      case 0x371D:  /* G3K */
         if (pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber == 3)
            return true;
         break;

      default:
         return false;
         break;
  }
  return false;
}

/*===========================================================================
METHOD:
   GobiOpen (Free Method)

DESCRIPTION:
   Start GPS if GPS port, run usb_serial_generic_open

PARAMETERS:
   pTTY    [ I ] - TTY structure (only on kernels <= 2.6.26)
   pPort   [ I ] - USB serial port structure
   pFilp   [ I ] - File structure (only on kernels <= 2.6.31)

RETURN VALUE:
   int - zero for success
       - negative errno on error
===========================================================================*/
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
int GobiOpen(
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
int GobiOpen(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#else // > 2.6.31
int GobiOpen(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort )
#endif
{
   struct sierra_port_private *portdata;
   const char startMessage[] = "$GPS_START";
   int nResult;
   int bytesWrote;

   DBG( "\n" );

   portdata = usb_get_serial_port_data(pPort);
   portdata->isClosing = 0;

   // Test parameters
   if ( (pPort == NULL)
   ||   (pPort->serial == NULL)
   ||   (pPort->serial->dev == NULL)
   ||   (pPort->serial->interface == NULL)
   ||   (pPort->serial->interface->cur_altsetting == NULL) )
   {
      DBG( "invalid parameter\n" );
      return -EINVAL;
   }

   // Is this the GPS port?
   if ((IsGPSPort(pPort)) == true)
   {
      int count = 0;
      if(portdata->iGPSStartState == eSendUnknown)
      {
        unsigned long now = jiffies;
        if(now < portdata->ulExpires)
        {
           unsigned long diff = jiffies_to_msecs(portdata->ulExpires-now);
           //Not wait more then delay_open_gps_port Seconds.
           if( diff > (delay_open_gps_port*1000))
           {
              DBG("Overwrite DELAY %lu msec\n",diff);
              diff = delay_open_gps_port*1000;
           }
           DBG("DELAY %lu msec\n",diff);
           if(diff > 0)
           msleep(diff);
           DBG("DELAY FINISH\n");
        }
        else
        {
          DBG("NON DELAY OPEN!\n" );
        }
      }
      portdata->iGPSStartState = eSendStart;
      DBG( "GPS Port detected! send GPS_START!\n" );
      // Send startMessage, USB_CTRL_SET_TIMEOUT timeout
      do
      {
        nResult = gobi_usb_bulk_msg( pPort->serial->dev,
                              usb_sndbulkpipe( pPort->serial->dev,
                                               pPort->bulk_out_endpointAddress ),
                              (void *)&startMessage[0],
                              sizeof( startMessage ),
                              &bytesWrote,
                              USB_CTRL_SET_TIMEOUT );
        if(nResult!=0)
        {
           if(count++>6)
           {
              printk( KERN_INFO "Send GPS_START Timeout!\n" );
              break;
           }
        }
      }while(-ETIMEDOUT==nResult);
      DBG( "send GPS_START done\n");
      if (nResult != 0)
      {
         DBG( "error %d sending startMessage\n", nResult );
         if (!ignore_gps_start_error)
         {
            return nResult;
         }
      }
      if (bytesWrote != sizeof( startMessage ))
      {
         DBG( "invalid write size %d, %lu\n",
              bytesWrote,
              (unsigned long)sizeof( startMessage ) );
         if (!ignore_gps_start_error)
         {
            return -EIO;
         }
      }
      portdata->iGPSStartState = eSendEnd;
   }

   // Clear endpoint halt condition
   if( nNumInterfaces > 1 )
   {
      nResult = usb_clear_halt(pPort->serial->dev,
                               usb_sndbulkpipe(pPort->serial->dev,
                               pPort->bulk_in_endpointAddress) | USB_DIR_IN );
      if (nResult != 0)
      {
         DBG( "usb_clear_halt return value = %d\n", nResult );
      }
   }

   /* host device is awake */
   set_power_state(pPort->serial->dev, 0x0000);

   // Pass to usb_serial_generic_open
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   return usb_serial_generic_open( pPort, pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
   return usb_serial_generic_open( pTTY, pPort, pFilp );
#else // > 2.6.31
   return usb_serial_generic_open( pTTY, pPort );
#endif
}

/*===========================================================================
METHOD:
   GobiClose (Free Method)

DESCRIPTION:
   Stop GPS if GPS port, run usb_serial_generic_close

PARAMETERS:
   pTTY    [ I ] - TTY structure (only if kernel > 2.6.26 and <= 2.6.29)
   pPort   [ I ] - USB serial port structure
   pFilp   [ I ] - File structure (only on kernel <= 2.6.30)
===========================================================================*/
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
void GobiClose(
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
void GobiClose(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#else // > 2.6.30
void GobiClose( struct usb_serial_port * pPort )
#endif
{
   struct sierra_port_private *portdata;
   const char stopMessage[] = "$GPS_STOP";
   int nResult;
   int bytesWrote;

   DBG( "\n" );

   portdata = usb_get_serial_port_data(pPort);
   portdata->isClosing = 1;

   // Test parameters
   if ( (pPort == NULL)
   ||   (pPort->serial == NULL)
   ||   (pPort->serial->dev == NULL)
   ||   (pPort->serial->interface == NULL)
   ||   (pPort->serial->interface->cur_altsetting == NULL) )
   {
      DBG( "invalid parameter\n" );
      return;
   }

   // Is this the GPS port?
   if ((IsGPSPort(pPort)) == true)
   {
      DBG( "GPS Port detected! send GPS_STOP!\n" );
      // Send stopMessage, 1s timeout
      nResult = gobi_usb_bulk_msg( pPort->serial->dev,
                              usb_sndbulkpipe( pPort->serial->dev,
                                               pPort->bulk_out_endpointAddress ),
                              (void *)&stopMessage[0],
                              sizeof( stopMessage ),
                              &bytesWrote,
                              100 );
      if (nResult != 0)
      {
         DBG( "error %d sending stopMessage\n", nResult );
      }
      if (bytesWrote != sizeof( stopMessage ))
      {
         DBG( "invalid write size %d, %lu\n",
              bytesWrote,
              (unsigned long)sizeof( stopMessage ) );
      }
   }

   // Pass to usb_serial_generic_close
   if (gpClose == NULL)
   {
      DBG( "NULL gpClose\n" );
      return;
   }

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   gpClose( pPort, pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   gpClose( pTTY, pPort, pFilp );
#else // > 2.6.30
   gpClose( pPort );
#endif
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,35 ))
/* Push data to tty layer and resubmit the bulk read URB */
static void flush_and_resubmit_read_urb(struct usb_serial_port *port)
{
    struct urb *urb = port->read_urb;
    struct tty_struct *tty = tty_port_tty_get(&port->port);
    char *ch = (char *)urb->transfer_buffer;
    int i;

    if (!tty || urb->status)
    {
        tty_kref_put(tty);
        goto done;
    }

    /* The per character mucking around with sysrq path it too slow for
       stuff like 3G modems, so shortcircuit it in the 99.9999999% of cases
       where the USB serial is not a console anyway */
    if (!port->console || !port->sysrq)
        tty_insert_flip_string(tty, ch, urb->actual_length);
    else {
        /* Push data to tty */
        for (i = 0; i < urb->actual_length; i++, ch++) {
            if (!usb_serial_handle_sysrq_char(tty, port, *ch))
                tty_insert_flip_char(tty, *ch, TTY_NORMAL);
        }
    }
    tty_flip_buffer_push(tty);
    tty_kref_put(tty);
done:
    usb_serial_generic_resubmit_read_urb(port, GFP_ATOMIC);
}
#endif //#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,35 ))

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,3,0 ))
static int usb_serial_generic_submit_read_urb(struct usb_serial_port *port,
                        int index, gfp_t mem_flags)
{
    int res;

    if (!test_and_clear_bit(index, &port->read_urbs_free))
        return 0;

    dev_dbg(&port->dev, "%s - urb %d\n", __func__, index);

    res = usb_submit_urb(port->read_urbs[index], mem_flags);
    if (res) {
        if (res != -EPERM) {
            dev_err(&port->dev,
                    "%s - usb_submit_urb failed: %d\n",
                    __func__, res);
        }
        set_bit(index, &port->read_urbs_free);
        return res;
    }

    return 0;
}
#endif //#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,3,0 ))

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))

/*===========================================================================
METHOD:
   GobiReadBulkCallback (Free Method)

DESCRIPTION:
   Read data from USB, push to TTY and user space

PARAMETERS:
   pURB  [ I ] - USB Request Block (urb) that called us

RETURN VALUE:
===========================================================================*/
static void GobiReadBulkCallback( struct urb * pURB )
{
   struct sierra_port_private *portdata;
   struct usb_serial_port * pPort = pURB->context;
   struct tty_struct * pTTY = pPort->tty;
   int nResult;
   int nRoom = 0;
   unsigned int pipeEP;
   int status = urb->status;

   DBG( "port %d\n", pPort->number );

   portdata = usb_get_serial_port_data(pPort);
   if (portdata->isClosing)
   {
       /* ignore bulk callback when port is closing */
       return;
   }

   if (status != 0)
   {
      if (status == -ESHUTDOWN || status == -ENOENT || status == -ENODEV) {
      {
          return;
      }
      DBG("nonzero read bulk status received: %d\n", pURB->status );
   }

   usb_serial_debug_data( debug,
                          &pPort->dev,
                          __FUNCTION__,
                          pURB->actual_length,
                          pURB->transfer_buffer );

   // We do no port throttling

   // Push data to tty layer and user space read function
   if ( (pTTY != 0) && (status == 0) && (pURB->actual_length) )
   {
      nRoom = tty_buffer_request_room( pTTY, pURB->actual_length );
      DBG( "room size %d %d\n", nRoom, 512 );
      if (nRoom != 0)
      {
         tty_insert_flip_string( pTTY, pURB->transfer_buffer, nRoom );
         tty_flip_buffer_push( pTTY );
      }
   }

   pipeEP = usb_rcvbulkpipe( pPort->serial->dev,
                             pPort->bulk_in_endpointAddress );

   // For continuous reading
   usb_fill_bulk_urb( pPort->read_urb,
                      pPort->serial->dev,
                      pipeEP,
                      pPort->read_urb->transfer_buffer,
                      pPort->read_urb->transfer_buffer_length,
                      GobiReadBulkCallback,
                      pPort );

   nResult = usb_submit_urb( pPort->read_urb, GFP_ATOMIC );
   if (nResult != 0)
   {
      DBG( "failed resubmitting read urb, error %d\n", nResult );
   }
}
#else
void GobiReadBulkCallback(struct urb *urb)
{
    struct sierra_port_private *portdata;
    struct usb_serial_port *port = urb->context;
    unsigned char *data = urb->transfer_buffer;
    unsigned long flags;
    int status = urb->status;
    int i;

    #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,3,0 ))
    for (i = 0; i < ARRAY_SIZE(port->read_urbs); ++i) {
        if (urb == port->read_urbs[i])
            break;
    }
    if (i < ARRAY_SIZE(port->read_urbs))
        set_bit(i, &port->read_urbs_free);
    #endif

    /* only ignore bulk callback after the bit of read_urbs_free was set,
       so that the port can be accessed later on */
    portdata = usb_get_serial_port_data(port);
    if (portdata->isClosing)
    {
        /* ignore bulk callback when port is closing */
        return;
    }

    dev_dbg(&port->dev, "%s - urb %d, len %d\n", __func__, i,
                            urb->actual_length);

    if (urb->status) {
        if (status == -ESHUTDOWN || status == -ENOENT || status == -ENODEV) {
            return;
        }
        DBG("%s - non-zero urb status: %d\n",
            __func__, urb->status);
    }
    else
    {
        #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,7,0 ))
            usb_serial_debug_data(&port->dev, __func__, urb->actual_length, data);
        #else
            usb_serial_debug_data(debug, &port->dev, __func__, urb->actual_length, data);
        #endif

        #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,35 ))
        port->serial->type->process_read_urb(urb);
        #endif
    }

    /* Throttle the device if requested by tty */
    spin_lock_irqsave(&port->lock, flags);
    port->throttled = port->throttle_req;
    if (!port->throttled)
    {
        spin_unlock_irqrestore(&port->lock, flags);
        #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,3,0 ))
        if (i < ARRAY_SIZE(port->read_urbs))
            usb_serial_generic_submit_read_urb(port, i, GFP_ATOMIC);
        #elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,35 ))
        usb_serial_generic_submit_read_urb(port, GFP_ATOMIC);
        #else
        flush_and_resubmit_read_urb(port);
        #endif
    }
    else
    {
        spin_unlock_irqrestore(&port->lock, flags);
    }
}

#endif //#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))

#ifdef CONFIG_PM
/*===========================================================================
METHOD:
   GobiSerialSuspend (Public Method)

DESCRIPTION:
   Set reset_resume flag

PARAMETERS
   pIntf          [ I ] - Pointer to interface
   powerEvent     [ I ] - Power management event

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiSerialSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent )
{
   struct usb_serial * pDev;
   DBG( "\n" );
   if (pIntf == 0)
   {
      return -ENOMEM;
   }

   pDev = usb_get_intfdata( pIntf );
   if (pDev == NULL)
   {
      return -ENXIO;
   }

   // Unless this is PM_EVENT_SUSPEND, make sure device gets rescanned
   if ((powerEvent.event & PM_EVENT_SUSPEND) == 0)
   {
      pDev->dev->reset_resume = 1;
   }

   // Run usb_serial's suspend function
   return usb_serial_suspend( pIntf, powerEvent );
}


#endif /* CONFIG_PM*/


static void gobi_stop_rx_urbs(struct usb_serial_port *port)
{
        usb_kill_urb(port->interrupt_in_urb);
}

void stop_read_write_urbs(struct usb_serial *serial)
{
        int i;
        struct usb_serial_port *port;

        /* Stop reading/writing urbs */
        for (i = 0; i < serial->num_ports; ++i) {
                port = serial->port[i];
                gobi_stop_rx_urbs(port);
        }
}



#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))

/*===========================================================================
METHOD:
   GobiSerialResume (Free Method)

DESCRIPTION:
   Restart URBs killed during usb_serial_suspend

   Fixes 2 bugs in 2.6.23 kernel
      1. pSerial->type->resume was NULL and unchecked, caused crash.
      2. set_to_generic_if_null was not run for resume.

PARAMETERS:
   pIntf  [ I ] - Pointer to interface

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiSerialResume( struct usb_interface * pIntf )
{
   struct usb_serial * pSerial = usb_get_intfdata( pIntf );
   struct usb_serial_port * pPort;
   int portIndex, errors, nResult;

   if (pSerial == NULL)
   {
      DBG( "no pSerial\n" );
      return -ENOMEM;
   }
   if (pSerial->type == NULL)
   {
      DBG( "no pSerial->type\n" );
      return ENOMEM;
   }
   if (pSerial->type->resume == NULL)
   {
      // Expected behaviour in 2.6.23, in later kernels this was handled
      // by the usb-serial driver and usb_serial_generic_resume
      errors = 0;
      for (portIndex = 0; portIndex < pSerial->num_ports; portIndex++)
      {
         pPort = pSerial->port[portIndex];
         if (pPort->open_count > 0 && pPort->read_urb != NULL)
         {
            nResult = usb_submit_urb( pPort->read_urb, GFP_NOIO );
            if (nResult < 0)
            {
               // Return first error we see
               DBG( "error %d\n", nResult );
               return nResult;
            }
         }
      }

      // Success
      return 0;
   }

   // Execution would only reach this point if user has
   // patched version of usb-serial driver.
   return usb_serial_resume( pIntf );
}

#endif
#endif /* CONFIG_PM*/

#ifdef CONFIG_PM
int GobiUSBSerialResetResume(struct usb_serial *serial)
{
    DBG( "\n" );
    return GobiUSBSerialResume(serial);
}

int GobiUSBSerialResume(struct usb_serial *serial)
{
   struct gobi_serial_intf_private *intfdata = usb_get_serial_data(serial);
   DBG( "\n" );
   spin_lock_irq(&intfdata->susp_lock);
   intfdata->suspended = 0;
   spin_unlock_irq(&intfdata->susp_lock);

   return 0;
}

int GobiUSBSerialSuspend(struct usb_serial *pDev, pm_message_t powerEvent)
{
   struct gobi_serial_intf_private *intfdata = usb_get_serial_data(pDev);
   DBG( "\n" );
   spin_lock_irq(&intfdata->susp_lock);
   intfdata->suspended = 1;
   spin_unlock_irq(&intfdata->susp_lock);
   stop_read_write_urbs(pDev);
   return 0;
}
#endif

void GobiUSBSerialDisconnect(struct usb_serial *serial)
{
   DBG( "\n" );
   stop_read_write_urbs(serial);
   return ;
}

/*===========================================================================
METHOD:
   GobiInit (Free Method)

DESCRIPTION:
   Register the driver and device

PARAMETERS:

RETURN VALUE:
   int - negative error code on failure
         zero on success
===========================================================================*/
static int __init GobiInit( void )
{
   int nRetval = 0;
   gpClose = NULL;

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,4,0 ))
   // Registering driver to USB serial core layer
   nRetval = usb_serial_register( &gGobiDevice );
   if (nRetval != 0)
   {
      return nRetval;
   }

   // Registering driver to USB core layer
   nRetval = usb_register( &GobiDriver );
   if (nRetval != 0)
   {
      usb_serial_deregister( &gGobiDevice );
      return nRetval;
   }

   // This will be shown whenever driver is loaded
   printk( KERN_INFO "%s: %s\n", DRIVER_DESC, DRIVER_VERSION );

#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 ))
   nRetval = usb_serial_register_drivers(serial_drivers, KBUILD_MODNAME, GobiVIDPIDTable);
#else
   nRetval = usb_serial_register_drivers(&GobiDriver, serial_drivers);
#endif
   if (nRetval == 0)
   {
      printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
            DRIVER_DESC "\n");
   }
#endif


   return nRetval;
}

/*===========================================================================
METHOD:
   GobiExit (Free Method)

DESCRIPTION:
   Deregister the driver and device

PARAMETERS:

RETURN VALUE:
===========================================================================*/
static void __exit GobiExit( void )
{
   gpClose = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,4,0 ))
   usb_deregister( &GobiDriver );
   usb_serial_deregister( &gGobiDevice );
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 ))
   usb_serial_deregister_drivers(serial_drivers);
#else
   usb_serial_deregister_drivers(&GobiDriver, serial_drivers);
#endif
#endif
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,2,0 ))
static inline int usb_endpoint_maxp(const struct usb_endpoint_descriptor *epd)
{
   return le16_to_cpu(epd->wMaxPacketSize);
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,35 ))
static inline struct usb_host_endpoint *
usb_pipe_endpoint(struct usb_device *dev, unsigned int pipe)
{
   struct usb_host_endpoint **eps;
   eps = usb_pipein(pipe) ? dev->ep_in : dev->ep_out;
   return eps[usb_pipeendpoint(pipe)];
}
#endif

/*===========================================================================
METHOD:
   Gobi_write (Free Method)

DESCRIPTION:
   call usb_serial_generic_write and conditionally add zero lenght payload.

PARAMETERS:
   pTTY    [ I ] - TTY structure
   pPort   [ I ] - USB serial port structure
   buf     [ I ] - write buffer
   count   [ I ] - number of buffer need to be writen.

RETURN VALUE:
   int - usb_serial_generic_write return value
===========================================================================*/
static int Gobi_write(struct tty_struct *pTTY, struct usb_serial_port *pPort,
               const unsigned char *buf, int count)
{
   int result = usb_serial_generic_write(pTTY, pPort, buf, count);
   struct usb_serial *pSerial;
   struct usb_host_endpoint *ep;
   struct sierra_port_private *portdata;
   if( (pPort) &&
       (pPort->serial) &&
       (pPort->serial->interface) )
   {
      if(pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber!=0)
      {
         return result;
      }
   }
   else
   {
      return -ENODEV;
   }
   pSerial = pPort->serial;
   if( (pSerial) &&
       (pSerial->dev) &&
       (pSerial->dev->actconfig) )
   {
      if(pSerial->dev->actconfig->desc.bNumInterfaces!=1)
      {
         return result;
      }
   }
   else
   {
      return -ENODEV;
   }
   portdata = usb_get_serial_port_data(pPort);
   if(portdata)
   {
      if((result ==count)&&(portdata->iUsb3ZlpEnable))
      {
         if(pSerial->dev->speed >= USB_SPEED_SUPER)
         {
            int pipe=0, len=0, size=0;
            ep = usb_pipe_endpoint(pSerial->dev, pipe);
            if( (ep) &&
                (!(count % usb_endpoint_maxp(&ep->desc))) )
            {
               pipe = usb_sndbulkpipe(pSerial->dev, pPort->bulk_out_endpointAddress);
               gobi_usb_bulk_msg(pSerial->dev, pipe, NULL, size,
                             &len, 3000);
            }
         }
      }
   }
   return result;
}

/*===========================================================================
METHOD:
   gobi_usb_bulk_msg (private Method)

DESCRIPTION:
   call usb_bulk_msg and alloc buffer if a data address is within the vmalloc range.

PARAMETERS:
   usb_dev [ I ] - pointer to the usb device to send the message to
   pipe    [ I ] - endpoint "pipe" to send the message to
   data    [ I ] - pointer to the data to send
   len     [ I ] - length in bytes of the data to send
   actual_length     [ O ] - pointer to a location to put the actual
   length transferred in bytes
   timeout [ I ] - time in msecs to wait for the message to complete
   before timing out (if 0 the wait is forever)

RETURN VALUE:
   int - usb_bulk_msg return value
===========================================================================*/
int gobi_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
   void *data, int len, int *actual_length,
   int timeout)
{
   void *pBuf = NULL;
   int ret = 0;

   pBuf = kzalloc(len,GFP_KERNEL);
   if(!pBuf)
   {
      DBG("MEM ERROR!\n");
      return -ENOMEM;
   }
   memcpy(pBuf,data,len);

   ret = usb_bulk_msg(usb_dev,pipe,pBuf,len,actual_length,timeout);
   if(pBuf!=data)
   {
      kfree(pBuf);
      pBuf = NULL;
   }
   return ret;
}

// Calling kernel module to init our driver
module_init( GobiInit );
module_exit( GobiExit );

MODULE_VERSION( DRIVER_VERSION );
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE( "Dual BSD/GPL" );

module_param( debug, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( debug, "Debug enabled or not" );
module_param( flow_control, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( flow_control, "flow control enabled or not" );
module_param( ignore_gps_start_error, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( ignore_gps_start_error,
   "allow port open to success even when GPS control message failed");
module_param( delay_open_gps_port, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( delay_open_gps_port, "Delay Open GPS Port, after device ready" );
module_param( iusb3_zlp_enable, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( iusb3_zlp_enable, "0 = Disable , 1 (default) ZLP on USB3 in QDL mode" );
