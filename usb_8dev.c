/*
 * CAN driver for "8 devices" USB2CAN converter
 *
 * Copyright (C) 2012 Bernd Krumboeck (krumboeck@universalnet.at)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * This driver is inspired by the 3.2.0 version of drivers/net/can/usb/ems_usb.c
 * and drivers/net/can/usb/esd_usb2.c
 *
 * Many thanks to Gerhard Bertelsmann (info@gerhard-bertelsmann.de)
 * for testing and fixing this driver. Also many thanks to "8 devices",
 * who were very cooperative and answered my questions.
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

/* driver constants */
#define MAX_RX_URBS			10
#define MAX_TX_URBS			10
#define RX_BUFFER_SIZE			64

/* vendor and product id */
#define USB_8DEV_VENDOR_ID		0x0483
#define USB_8DEV_PRODUCT_ID		0x1234

/* endpoints */
enum usb_8dev_endpoint {
	USB_8DEV_ENDP_DATA_RX = 1,
	USB_8DEV_ENDP_DATA_TX,
	USB_8DEV_ENDP_CMD_RX,
	USB_8DEV_ENDP_CMD_TX
};

/* bittiming constants */
#define USB_8DEV_ABP_CLOCK		32000000
#define USB_8DEV_BAUD_MANUAL		0x09
#define USB_8DEV_TSEG1_MIN		1
#define USB_8DEV_TSEG1_MAX		16
#define USB_8DEV_TSEG2_MIN		1
#define USB_8DEV_TSEG2_MAX		8
#define USB_8DEV_SJW_MAX		4
#define USB_8DEV_BRP_MIN		1
#define USB_8DEV_BRP_MAX		1024
#define USB_8DEV_BRP_INC		1

/* setup flags */
#define USB_8DEV_SILENT			0x01
#define USB_8DEV_LOOPBACK		0x02
#define USB_8DEV_DISABLE_AUTO_RESTRANS	0x04
#define USB_8DEV_STATUS_FRAME		0x08

/* commands */
enum usb_8dev_cmd {
	USB_8DEV_RESET = 1,
	USB_8DEV_OPEN,
	USB_8DEV_CLOSE,
	USB_8DEV_SET_SPEED,
	USB_8DEV_SET_MASK_FILTER,
	USB_8DEV_GET_STATUS,
	USB_8DEV_GET_STATISTICS,
	USB_8DEV_GET_SERIAL,
	USB_8DEV_GET_SOFTW_VER,
	USB_8DEV_GET_HARDW_VER,
	USB_8DEV_RESET_TIMESTAMP,
	USB_8DEV_GET_SOFTW_HARDW_VER
};

#define USB_8DEV_CMD_START		0x11
#define USB_8DEV_CMD_END		0x22

#define USB_8DEV_CMD_SUCCESS		0
#define USB_8DEV_CMD_ERROR		255

#define USB_8DEV_CMD_TIMEOUT		1000

/* frames */
#define USB_8DEV_DATA_START		0x55
#define USB_8DEV_DATA_END		0xAA

#define USB_8DEV_TYPE_CAN_FRAME		0
#define USB_8DEV_TYPE_ERROR_FRAME	3

#define USB_8DEV_EXTID			0x01
#define USB_8DEV_RTR			0x02
#define USB_8DEV_ERR_FLAG		0x04

/* status */
#define USB_8DEV_STATUSMSG_OK		0x00  /* Normal condition. */
#define USB_8DEV_STATUSMSG_OVERRUN	0x01  /* Overrun occured when sending */
#define USB_8DEV_STATUSMSG_BUSLIGHT	0x02  /* Error counter has reached 96 */
#define USB_8DEV_STATUSMSG_BUSHEAVY	0x03  /* Error count. has reached 128 */
#define USB_8DEV_STATUSMSG_BUSOFF	0x04  /* Device is in BUSOFF */
#define USB_8DEV_STATUSMSG_STUFF	0x20  /* Stuff Error */
#define USB_8DEV_STATUSMSG_FORM		0x21  /* Form Error */
#define USB_8DEV_STATUSMSG_ACK		0x23  /* Ack Error */
#define USB_8DEV_STATUSMSG_BIT0		0x24  /* Bit1 Error */
#define USB_8DEV_STATUSMSG_BIT1		0x25  /* Bit0 Error */
#define USB_8DEV_STATUSMSG_CRC		0x26  /* CRC Error */

#define USB_8DEV_RP_MASK		0x7F  /* Mask for Receive Error Bit */


/* table of devices that work with this driver */
static const struct usb_device_id usb_8dev_table[] = {
	{ USB_DEVICE(USB_8DEV_VENDOR_ID, USB_8DEV_PRODUCT_ID) },
	{ }					/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, usb_8dev_table);

struct usb_8dev_tx_urb_context {
	struct usb_8dev *dev;

	u32 echo_index;
	u8 dlc;
};

/* Structure to hold all of our device specific stuff */
struct usb_8dev {
	struct can_priv can; /* must be the first member */

	struct sk_buff *echo_skb[MAX_TX_URBS];

	struct usb_device *udev;
	struct net_device *netdev;

	atomic_t active_tx_urbs;
	struct usb_anchor tx_submitted;
	struct usb_8dev_tx_urb_context tx_contexts[MAX_TX_URBS];

	struct usb_anchor rx_submitted;

	struct can_berr_counter bec;

	u8 *cmd_msg_buffer;

	unsigned int free_slots; /* remember number of available slots */

	struct mutex usb_8dev_cmd_lock;

};

/* tx frame */
struct __packed usb_8dev_tx_msg {
	u8 begin;
	u8 flags;	/* RTR and EXT_ID flag */
	__be32 id;	/* upper 3 bits not used */
	u8 dlc;		/* data length code 0-8 bytes */
	u8 data[8];	/* 64-bit data */
	u8 end;
};

/* rx frame */
struct __packed usb_8dev_rx_msg {
	u8 begin;
	u8 type;		/* frame type */
	u8 flags;		/* RTR and EXT_ID flag */
	__be32 id;		/* upper 3 bits not used */
	u8 dlc;			/* data length code 0-8 bytes */
	u8 data[8];		/* 64-bit data */
	__be32 timestamp;	/* 32-bit timestamp */
	u8 end;
};

/* command frame */
struct __packed usb_8dev_cmd_msg {
	u8 begin;
	u8 channel;	/* unkown - always 0 */
	u8 command;	/* command to execute */
	u8 opt1;	/* optional parameter / return value */
	u8 opt2;	/* optional parameter 2 */
	u8 data[10];	/* optional parameter and data */
	u8 end;
};

static struct usb_driver usb_8dev_driver;

static int usb_8dev_send_cmd_msg(struct usb_8dev *dev, u8 *msg, int size)
{
	int actual_length;

	return usb_bulk_msg(dev->udev,
			    usb_sndbulkpipe(dev->udev, USB_8DEV_ENDP_CMD_TX),
			    msg, size, &actual_length, USB_8DEV_CMD_TIMEOUT);
}

static int usb_8dev_wait_cmd_msg(struct usb_8dev *dev, u8 *msg, int size,
				int *actual_length)
{
	return usb_bulk_msg(dev->udev,
			    usb_rcvbulkpipe(dev->udev, USB_8DEV_ENDP_CMD_RX),
			    msg, size, actual_length, USB_8DEV_CMD_TIMEOUT);
}

/*
 * Send command to device and receive result.
 * Command was successful when opt1 = 0.
 */
static int usb_8dev_send_cmd(struct usb_8dev *dev, struct usb_8dev_cmd_msg *out,
			    struct usb_8dev_cmd_msg *in)
{
	int err;
	int num_bytes_read;
	struct net_device *netdev;

	netdev = dev->netdev;

	out->begin = USB_8DEV_CMD_START;
	out->end = USB_8DEV_CMD_END;

	mutex_lock(&dev->usb_8dev_cmd_lock);

	memcpy(dev->cmd_msg_buffer, out,
		sizeof(struct usb_8dev_cmd_msg));

	err = usb_8dev_send_cmd_msg(dev, dev->cmd_msg_buffer,
				    sizeof(struct usb_8dev_cmd_msg));
	if (err < 0) {
		netdev_err(netdev, "sending command message failed\n");
		return err;
	}

	err = usb_8dev_wait_cmd_msg(dev, dev->cmd_msg_buffer,
				    sizeof(struct usb_8dev_cmd_msg),
				    &num_bytes_read);
	if (err < 0) {
		netdev_err(netdev, "no command message answer\n");
		return err;
	}

	memcpy(in, dev->cmd_msg_buffer, sizeof(struct usb_8dev_cmd_msg));

	mutex_unlock(&dev->usb_8dev_cmd_lock);

	if (in->begin != USB_8DEV_CMD_START || in->end != USB_8DEV_CMD_END ||
			num_bytes_read != 16 || in->opt1 != 0)
		return -EPROTO;

	return 0;
}

/* Send open command to device */
static int usb_8dev_cmd_open(struct usb_8dev *dev)
{
	struct can_bittiming *bt = &dev->can.bittiming;
	struct usb_8dev_cmd_msg outmsg;
	struct usb_8dev_cmd_msg inmsg;
	u32 flags = 0;
	u32 beflags;
	u16 bebrp;
	u32 ctrlmode = dev->can.ctrlmode;

	if (ctrlmode & CAN_CTRLMODE_LOOPBACK)
		flags |= USB_8DEV_LOOPBACK;
	if (ctrlmode & CAN_CTRLMODE_LISTENONLY)
		flags |= USB_8DEV_SILENT;
	if (ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		flags |= USB_8DEV_DISABLE_AUTO_RESTRANS;

	flags |= USB_8DEV_STATUS_FRAME;

	memset(&outmsg, 0, sizeof(struct usb_8dev_cmd_msg));
	outmsg.command = USB_8DEV_OPEN;
	outmsg.opt1 = USB_8DEV_BAUD_MANUAL;
	outmsg.data[0] = (bt->prop_seg + bt->phase_seg1);
	outmsg.data[1] = bt->phase_seg2;
	outmsg.data[2] = bt->sjw;

	/* BRP */
	bebrp = cpu_to_be16((u16) bt->brp);
	memcpy(&outmsg.data[3], &bebrp, sizeof(bebrp));

	/* flags */
	beflags = cpu_to_be32(flags);
	memcpy(&outmsg.data[5], &beflags, sizeof(beflags));

	return usb_8dev_send_cmd(dev, &outmsg, &inmsg);
}

/* Send close command to device */
static int usb_8dev_cmd_close(struct usb_8dev *dev)
{
	struct usb_8dev_cmd_msg inmsg;
	struct usb_8dev_cmd_msg outmsg = {
		.channel = 0,
		.command = USB_8DEV_CLOSE,
		.opt1 = 0,
		.opt2 = 0
		};

	return usb_8dev_send_cmd(dev, &outmsg, &inmsg);
}

/* Get firmware and hardware version */
static int usb_8dev_cmd_version(struct usb_8dev *dev, u32 *res)
{
	struct usb_8dev_cmd_msg	inmsg;
	struct usb_8dev_cmd_msg	outmsg = {
		.channel = 0,
		.command = USB_8DEV_GET_SOFTW_HARDW_VER,
		.opt1 = 0,
		.opt2 = 0
		};

	int err = usb_8dev_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return err;

	*res = be32_to_cpu(*(u32 *)inmsg.data);

	return err;
}

/* Get firmware version */
static ssize_t show_firmware(struct device *d, struct device_attribute *attr,
			     char *buf)
{
	struct usb_interface *intf = to_usb_interface(d);
	struct usb_8dev *dev = usb_get_intfdata(intf);
	u16 result;
	struct usb_8dev_cmd_msg inmsg;
	struct usb_8dev_cmd_msg outmsg = {
		.channel = 0,
		.command = USB_8DEV_GET_SOFTW_VER,
		.opt1 = 0,
		.opt2 = 0
		};

	int err = usb_8dev_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return -EIO;

	result = be16_to_cpu(*(u16 *)inmsg.data);

	return sprintf(buf, "%d.%d\n", (u8)(result>>8), (u8)result);
}

/* Get hardware version */
static ssize_t show_hardware(struct device *d, struct device_attribute *attr,
			     char *buf)
{
	struct usb_interface *intf = to_usb_interface(d);
	struct usb_8dev *dev = usb_get_intfdata(intf);
	u16 result;
	struct usb_8dev_cmd_msg inmsg;
	struct usb_8dev_cmd_msg outmsg = {
		.channel = 0,
		.command = USB_8DEV_GET_HARDW_VER,
		.opt1 = 0,
		.opt2 = 0
		};

	int err = usb_8dev_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return -EIO;

	result = be16_to_cpu(*(u16 *)inmsg.data);

	return sprintf(buf, "%d.%d\n", (u8)(result>>8), (u8)result);
}

static DEVICE_ATTR(firmware, S_IRUGO, show_firmware, NULL);
static DEVICE_ATTR(hardware, S_IRUGO, show_hardware, NULL);

/*
 * Set network device mode
 *
 * Maybe we should leave this function empty, because the device
 * set mode variable with open command.
 */
static int usb_8dev_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct usb_8dev *dev = netdev_priv(netdev);
	int err = 0;

	switch (mode) {
	case CAN_MODE_START:
		err = usb_8dev_cmd_open(dev);
		if (err)
			netdev_warn(netdev, "couldn't start device");
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* Read data and status frames */
static void usb_8dev_rx_can_msg(struct usb_8dev *dev,
				struct usb_8dev_rx_msg *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &dev->netdev->stats;

	if (msg->type == USB_8DEV_TYPE_CAN_FRAME) {
		skb = alloc_can_skb(dev->netdev, &cf);
		if (!skb)
			return;

		cf->can_id = be32_to_cpu(msg->id);
		cf->can_dlc = get_can_dlc(msg->dlc & 0xF);

		if (msg->flags & USB_8DEV_EXTID)
			cf->can_id |= CAN_EFF_FLAG;

		if (msg->flags & USB_8DEV_RTR)
			cf->can_id |= CAN_RTR_FLAG;
		else
			memcpy(cf->data, msg->data, cf->can_dlc);
	} else if (msg->type == USB_8DEV_TYPE_ERROR_FRAME &&
		   msg->flags == USB_8DEV_ERR_FLAG) {

		/*
		 * Error message:
		 * byte 0: Status
		 * byte 1: bit   7: Receive Passive
		 * byte 1: bit 0-6: Receive Error Counter
		 * byte 2: Transmit Error Counter
		 * byte 3: Always 0 (maybe reserved for future use)
		 */

		u8 state = msg->data[0];
		u8 rxerr = msg->data[1] & USB_8DEV_RP_MASK;
		u8 txerr = msg->data[2];
		int rx_errors = 0;
		int tx_errors = 0;

		skb = alloc_can_err_skb(dev->netdev, &cf);
		if (!skb)
			return;

		switch (state) {
		case USB_8DEV_STATUSMSG_OK:
			dev->can.state = CAN_STATE_ERROR_ACTIVE;
			cf->can_id |= CAN_ERR_PROT;
			cf->data[2] = CAN_ERR_PROT_ACTIVE;
			break;
		case USB_8DEV_STATUSMSG_BUSOFF:
			dev->can.state = CAN_STATE_BUS_OFF;
			cf->can_id |= CAN_ERR_BUSOFF;
			can_bus_off(dev->netdev);
			break;
		case USB_8DEV_STATUSMSG_OVERRUN:
		case USB_8DEV_STATUSMSG_BUSLIGHT:
		case USB_8DEV_STATUSMSG_BUSHEAVY:
			dev->can.state = CAN_STATE_ERROR_WARNING;
			cf->can_id |= CAN_ERR_CRTL;
			break;
		default:
			dev->can.state = CAN_STATE_ERROR_WARNING;
			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
			dev->can.can_stats.bus_error++;
			break;
		}

		switch (state) {
		case USB_8DEV_STATUSMSG_ACK:
			cf->can_id |= CAN_ERR_ACK;
			tx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_CRC:
			cf->data[2] |= CAN_ERR_PROT_BIT;
			rx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_BIT0:
			cf->data[2] |= CAN_ERR_PROT_BIT0;
			tx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_BIT1:
			cf->data[2] |= CAN_ERR_PROT_BIT1;
			tx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_FORM:
			cf->data[2] |= CAN_ERR_PROT_FORM;
			rx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_STUFF:
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			rx_errors = 1;
			break;
		case USB_8DEV_STATUSMSG_OVERRUN:
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_OVERFLOW :
				CAN_ERR_CRTL_RX_OVERFLOW;
			cf->data[2] |= CAN_ERR_PROT_OVERLOAD;
			stats->rx_over_errors++;
			break;
		case USB_8DEV_STATUSMSG_BUSLIGHT:
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
			dev->can.can_stats.error_warning++;
			break;
		case USB_8DEV_STATUSMSG_BUSHEAVY:
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
			dev->can.can_stats.error_passive++;
			break;
		}

		if (tx_errors) {
			cf->data[2] |= CAN_ERR_PROT_TX;
			stats->tx_errors++;
		}

		if (rx_errors)
			stats->rx_errors++;

		cf->data[6] = txerr;
		cf->data[7] = rxerr;

		dev->bec.txerr = txerr;
		dev->bec.rxerr = rxerr;

	} else {
		netdev_warn(dev->netdev, "frame type %d unknown",
			 msg->type);
		return;
	}

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
}

/*
 * Callback for reading data from device
 *
 * Check urb status, call read function and resubmit urb read operation.
 */
static void usb_8dev_read_bulk_callback(struct urb *urb)
{
	struct usb_8dev *dev = urb->context;
	struct net_device *netdev;
	int retval;
	int pos = 0;

	netdev = dev->netdev;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
	case -ESHUTDOWN:
		return;

	default:
		netdev_info(netdev, "Rx URB aborted (%d)\n",
			 urb->status);
		goto resubmit_urb;
	}

	while (pos < urb->actual_length) {
		struct usb_8dev_rx_msg *msg;

		if (pos + sizeof(struct usb_8dev_rx_msg) > urb->actual_length) {
			netdev_err(dev->netdev, "format error\n");
			break;
		}

		msg = (struct usb_8dev_rx_msg *)(urb->transfer_buffer + pos);
		usb_8dev_rx_can_msg(dev, msg);

		pos += sizeof(struct usb_8dev_rx_msg);
	}

resubmit_urb:
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_rcvbulkpipe(dev->udev, USB_8DEV_ENDP_DATA_RX),
			  urb->transfer_buffer, RX_BUFFER_SIZE,
			  usb_8dev_read_bulk_callback, dev);

	retval = usb_submit_urb(urb, GFP_ATOMIC);

	if (retval == -ENODEV)
		netif_device_detach(netdev);
	else if (retval)
		netdev_err(netdev,
			"failed resubmitting read bulk urb: %d\n", retval);
}

/*
 * Callback handler for write operations
 *
 * Free allocated buffers, check transmit status and
 * calculate statistic.
 */
static void usb_8dev_write_bulk_callback(struct urb *urb)
{
	struct usb_8dev_tx_urb_context *context = urb->context;
	struct usb_8dev *dev;
	struct net_device *netdev;

	BUG_ON(!context);

	dev = context->dev;
	netdev = dev->netdev;

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	atomic_dec(&dev->active_tx_urbs);

	if (!netif_device_present(netdev))
		return;

	if (urb->status)
		netdev_info(netdev, "Tx URB aborted (%d)\n",
			 urb->status);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += context->dlc;

	can_get_echo_skb(netdev, context->echo_index);

	/* Release context */
	context->echo_index = MAX_TX_URBS;

	netif_wake_queue(netdev);
}

/* Send data to device */
static netdev_tx_t usb_8dev_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	struct usb_8dev *dev = netdev_priv(netdev);
	struct net_device_stats *stats = &netdev->stats;
	struct can_frame *cf = (struct can_frame *) skb->data;
	struct usb_8dev_tx_msg *msg;
	struct urb *urb;
	struct usb_8dev_tx_urb_context *context = NULL;
	u8 *buf;
	int i, err;
	size_t size = sizeof(struct usb_8dev_tx_msg);

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	/* create a URB, and a buffer for it, and copy the data to the URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		netdev_err(netdev, "No memory left for URBs\n");
		goto nomem;
	}

	buf = usb_alloc_coherent(dev->udev, size, GFP_ATOMIC,
				 &urb->transfer_dma);
	if (!buf) {
		netdev_err(netdev, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	memset(buf, 0, size);

	msg = (struct usb_8dev_tx_msg *)buf;
	msg->begin = USB_8DEV_DATA_START;
	msg->flags = 0x00;

	if (cf->can_id & CAN_RTR_FLAG)
		msg->flags |= USB_8DEV_RTR;

	if (cf->can_id & CAN_EFF_FLAG)
		msg->flags |= USB_8DEV_EXTID;

	msg->id = cpu_to_be32(cf->can_id & CAN_ERR_MASK);
	msg->dlc = cf->can_dlc;
	memcpy(msg->data, cf->data, cf->can_dlc);
	msg->end = USB_8DEV_DATA_END;

	for (i = 0; i < MAX_TX_URBS; i++) {
		if (dev->tx_contexts[i].echo_index == MAX_TX_URBS) {
			context = &dev->tx_contexts[i];
			break;
		}
	}

	/*
	 * May never happen! When this happens we'd more URBs in flight as
	 * allowed (MAX_TX_URBS).
	 */
	if (!context) {
		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, buf, urb->transfer_dma);

		netdev_warn(netdev, "couldn't find free context");

		return NETDEV_TX_BUSY;
	}

	context->dev = dev;
	context->echo_index = i;
	context->dlc = cf->can_dlc;

	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, USB_8DEV_ENDP_DATA_TX),
			  buf, size, usb_8dev_write_bulk_callback, context);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->tx_submitted);

	can_put_echo_skb(skb, netdev, context->echo_index);

	atomic_inc(&dev->active_tx_urbs);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(err)) {
		can_free_echo_skb(netdev, context->echo_index);

		usb_unanchor_urb(urb);
		usb_free_coherent(dev->udev, size, buf, urb->transfer_dma);
		dev_kfree_skb(skb);

		atomic_dec(&dev->active_tx_urbs);

		if (err == -ENODEV) {
			netif_device_detach(netdev);
		} else {
			netdev_warn(netdev, "failed tx_urb %d\n", err);

			stats->tx_dropped++;
		}
	} else {
		/* Slow down tx path */
		if (atomic_read(&dev->active_tx_urbs) >= MAX_TX_URBS ||
		    dev->free_slots < 5) {
			netif_stop_queue(netdev);
		}
	}

	/*
	 * Release our reference to this URB, the USB core will eventually free
	 * it entirely.
	 */
	usb_free_urb(urb);

	return NETDEV_TX_OK;

nomem:
	dev_kfree_skb(skb);
	stats->tx_dropped++;

	return NETDEV_TX_OK;
}

static int usb_8dev_get_berr_counter(const struct net_device *netdev,
				     struct can_berr_counter *bec)
{
	struct usb_8dev *dev = netdev_priv(netdev);

	bec->txerr = dev->bec.txerr;
	bec->rxerr = dev->bec.rxerr;

	return 0;
}

/* Start USB device */
static int usb_8dev_start(struct usb_8dev *dev)
{
	struct net_device *netdev = dev->netdev;
	int err, i;

	dev->free_slots = 15; /* initial size */

	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf = NULL;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			netdev_err(netdev, "No memory left for URBs\n");
			return -ENOMEM;
		}

		buf = usb_alloc_coherent(dev->udev, RX_BUFFER_SIZE, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			netdev_err(netdev, "No memory left for USB buffer\n");
			usb_free_urb(urb);
			return -ENOMEM;
		}

		usb_fill_bulk_urb(urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev,
						  USB_8DEV_ENDP_DATA_RX),
				  buf, RX_BUFFER_SIZE,
				  usb_8dev_read_bulk_callback, dev);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &dev->rx_submitted);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			if (err == -ENODEV)
				netif_device_detach(dev->netdev);

			usb_unanchor_urb(urb);
			usb_free_coherent(dev->udev, RX_BUFFER_SIZE, buf,
					  urb->transfer_dma);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}

	/* Did we submit any URBs */
	if (i == 0) {
		netdev_warn(netdev, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't transmit all the URBs */
	if (i < MAX_RX_URBS)
		netdev_warn(netdev, "rx performance may be slow\n");

	err = usb_8dev_cmd_open(dev);
	if (err)
		goto failed;

	dev->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;

failed:
	if (err == -ENODEV)
		netif_device_detach(dev->netdev);

	netdev_warn(netdev, "couldn't submit control: %d\n", err);

	return err;
}

/* Open USB device */
static int usb_8dev_open(struct net_device *netdev)
{
	struct usb_8dev *dev = netdev_priv(netdev);
	int err;

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	/* finally start device */
	err = usb_8dev_start(dev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(dev->netdev);

		netdev_warn(netdev, "couldn't start device: %d\n",
			 err);

		close_candev(netdev);

		return err;
	}

	netif_start_queue(netdev);

	return 0;
}

static void unlink_all_urbs(struct usb_8dev *dev)
{
	int i;

	usb_kill_anchored_urbs(&dev->rx_submitted);

	usb_kill_anchored_urbs(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		dev->tx_contexts[i].echo_index = MAX_TX_URBS;
}

/* Close USB device */
static int usb_8dev_close(struct net_device *netdev)
{
	struct usb_8dev *dev = netdev_priv(netdev);
	int err = 0;

	/* Send CLOSE command to CAN controller */
	err = usb_8dev_cmd_close(dev);
	if (err)
		netdev_warn(netdev, "couldn't stop device");

	dev->can.state = CAN_STATE_STOPPED;

	/* Stop polling */
	unlink_all_urbs(dev);

	netif_stop_queue(netdev);

	close_candev(netdev);

	return err;
}

static const struct net_device_ops usb_8dev_netdev_ops = {
	.ndo_open = usb_8dev_open,
	.ndo_stop = usb_8dev_close,
	.ndo_start_xmit = usb_8dev_start_xmit,
};

static const struct can_bittiming_const usb_8dev_bittiming_const = {
	.name = "usb_8dev",
	.tseg1_min = USB_8DEV_TSEG1_MIN,
	.tseg1_max = USB_8DEV_TSEG1_MAX,
	.tseg2_min = USB_8DEV_TSEG2_MIN,
	.tseg2_max = USB_8DEV_TSEG2_MAX,
	.sjw_max = USB_8DEV_SJW_MAX,
	.brp_min = USB_8DEV_BRP_MIN,
	.brp_max = USB_8DEV_BRP_MAX,
	.brp_inc = USB_8DEV_BRP_INC,
};

/*
 * Probe USB device
 *
 * Check device and firmware.
 * Set supported modes and bittiming constants.
 * Allocate some memory.
 */
static int usb_8dev_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct net_device *netdev;
	struct usb_8dev *dev;
	int i, err = -ENOMEM;
	u32 version;
	char buf[18];
	struct usb_device *usbdev = interface_to_usbdev(intf);

	/* product id looks strange, better we also check iProdukt string */
	if (usb_string(usbdev, usbdev->descriptor.iProduct, buf,
		       sizeof(buf)) > 0 && strcmp(buf, "USB2CAN converter")) {
		dev_info(&usbdev->dev, "ignoring: not an USB2CAN converter\n");
		return -ENODEV;
	}

	netdev = alloc_candev(sizeof(struct usb_8dev), MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	dev = netdev_priv(netdev);

	dev->udev = usbdev;
	dev->netdev = netdev;

	dev->can.state = CAN_STATE_STOPPED;
	dev->can.clock.freq = USB_8DEV_ABP_CLOCK;
	dev->can.bittiming_const = &usb_8dev_bittiming_const;
	dev->can.do_set_mode = usb_8dev_set_mode;
	dev->can.do_get_berr_counter = usb_8dev_get_berr_counter;
	dev->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				      CAN_CTRLMODE_LISTENONLY |
				      CAN_CTRLMODE_ONE_SHOT;

	netdev->netdev_ops = &usb_8dev_netdev_ops;

	netdev->flags |= IFF_ECHO; /* we support local echo */

	init_usb_anchor(&dev->rx_submitted);

	init_usb_anchor(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		dev->tx_contexts[i].echo_index = MAX_TX_URBS;

	dev->cmd_msg_buffer = kzalloc(sizeof(struct usb_8dev_cmd_msg),
				      GFP_KERNEL);
	if (!dev->cmd_msg_buffer) {
		netdev_err(netdev, "Couldn't alloc Tx buffer\n");
		goto cleanup_candev;
	}

	usb_set_intfdata(intf, dev);

	SET_NETDEV_DEV(netdev, &intf->dev);

	mutex_init(&dev->usb_8dev_cmd_lock);

	err = usb_8dev_cmd_version(dev, &version);
	if (err) {
		netdev_err(netdev, "can't get firmware version\n");
		goto cleanup_cmd_msg_buffer;
	} else {
		netdev_info(netdev,
			 "firmware: %d.%d, hardware: %d.%d\n",
			 (version>>24) & 0xff, (version>>16) & 0xff,
			 (version>>8) & 0xff, version & 0xff);
	}

	err = register_candev(netdev);
	if (err) {
		netdev_err(netdev,
			"couldn't register CAN device: %d\n", err);
		goto cleanup_cmd_msg_buffer;
	}

	if (device_create_file(&intf->dev, &dev_attr_firmware))
		netdev_err(netdev,
			"Couldn't create device file for firmware\n");

	if (device_create_file(&intf->dev, &dev_attr_hardware))
		netdev_err(netdev,
			"Couldn't create device file for hardware\n");

	/* let the user know what node this device is now attached to */
	netdev_info(netdev, "device registered as %s\n", netdev->name);
	return 0;

cleanup_cmd_msg_buffer:
	kfree(dev->cmd_msg_buffer);

cleanup_candev:
	free_candev(netdev);

	return err;

}

/* Called by the usb core when driver is unloaded or device is removed */
static void usb_8dev_disconnect(struct usb_interface *intf)
{
	struct usb_8dev *dev = usb_get_intfdata(intf);

	device_remove_file(&intf->dev, &dev_attr_firmware);
	device_remove_file(&intf->dev, &dev_attr_hardware);

	usb_set_intfdata(intf, NULL);

	if (dev) {
		netdev_info(dev->netdev, "disconnect %s\n", dev->netdev->name);

		unregister_netdev(dev->netdev);
		free_candev(dev->netdev);

		unlink_all_urbs(dev);
	}

}

static struct usb_driver usb_8dev_driver = {
	.name =		"usb_8dev",
	.probe =	usb_8dev_probe,
	.disconnect =	usb_8dev_disconnect,
	.id_table =	usb_8dev_table,
};

module_usb_driver(usb_8dev_driver);

MODULE_AUTHOR("Bernd Krumboeck <krumboeck@universalnet.at>");
MODULE_DESCRIPTION("CAN driver for 8 devices USB2CAN interfaces");
MODULE_LICENSE("GPL v2");
