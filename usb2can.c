/*
 * CAN driver for UAB "8 devices" USB2CAN converter
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
 * This driver is based on the 3.2.0 version of drivers/net/can/usb/ems_usb.c
 * and drivers/net/can/usb/esd_usb2.c
 *
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
#define INTR_IN_BUFFER_SIZE		4

/* vendor and product id */
#define USB2CAN_VENDOR_ID		0x0483
#define USB2CAN_PRODUCT_ID		0x1234

/* bittiming constants */
#define USB2CAN_ABP_CLOCK		32000000
#define USB2CAN_BAUD_MANUAL		0x09
#define USB2CAN_TSEG1_MIN		1
#define USB2CAN_TSEG1_MAX		8
#define USB2CAN_TSEG2_MIN		1
#define USB2CAN_TSEG2_MAX		8
#define USB2CAN_SJW_MAX			4
#define USB2CAN_BRP_MIN			1
#define USB2CAN_BRP_MAX			32
#define USB2CAN_BRP_INC			1

/* setup flags */
#define USB2CAN_SILENT			0x00000001
#define USB2CAN_LOOPBACK		0x00000002
#define USB2CAN_DAR_DISABLE		0x00000004
#define USB2CAN_STATUS_FRAME		0x00000008

/* commands */
#define USB2CAN_RESET			1
#define USB2CAN_OPEN			2
#define USB2CAN_CLOSE			3
#define USB2CAN_SET_SPEED		4
#define USB2CAN_SET_MASK_FILTER		5
#define USB2CAN_GET_STATUS		6
#define USB2CAN_GET_STATISTICS		7
#define USB2CAN_GET_SERIAL		8
#define USB2CAN_GET_SOFTW_VER		9
#define USB2CAN_GET_HARDW_VER		10
#define USB2CAN_RESET_TIMESTAMP		11
#define USB2CAN_GET_SOFTW_HARDW_VER	12

#define USB2CAN_CMD_START		0x11
#define USB2CAN_CMD_END			0x22

#define USB2CAN_CMD_SUCCESS		0
#define USB2CAN_CMD_ERROR		255

/* statistics */
#define USB2CAN_STAT_RX_FRAMES		0
#define USB2CAN_STAT_RX_BYTES		1
#define USB2CAN_STAT_TX_FRAMES		2
#define USB2CAN_STAT_TX_BYTES		3
#define USB2CAN_STAT_OVERRUNS		4
#define USB2CAN_STAT_WARNINGS		5
#define USB2CAN_STAT_BUS_OFF		6
#define USB2CAN_STAT_RESET_STAT		7

/* frames */
#define USB2CAN_DATA_START		0x55
#define USB2CAN_DATA_END		0xAA

#define USB2CAN_TYPE_CAN_FRAME		0
#define USB2CAN_TYPE_ERROR_FRAME	3

#define USB2CAN_EXTID			0x01
#define USB2CAN_RTR			0x02
#define USB2CAN_ERR_FLAG		0x04

/* status */
#define USB2CAN_STATUSMSG_OK		0x00  /* Normal condition. */
#define USB2CAN_STATUSMSG_OVERRUN	0x01  /* Overrun occured when sending */
#define USB2CAN_STATUSMSG_BUSLIGHT	0x02  /* Error counter has reached 96 */
#define USB2CAN_STATUSMSG_BUSHEAVY	0x03  /* Error count. has reached 128 */
#define USB2CAN_STATUSMSG_BUSOFF	0x04  /* Device is in BUSOFF */
#define USB2CAN_STATUSMSG_STUFF		0x20  /* Stuff Error */
#define USB2CAN_STATUSMSG_FORM		0x21  /* Form Error */
#define USB2CAN_STATUSMSG_ACK		0x23  /* Ack Error */
#define USB2CAN_STATUSMSG_BIT0		0x24  /* Bit1 Error */
#define USB2CAN_STATUSMSG_BIT1		0x25  /* Bit0 Error */
#define USB2CAN_STATUSMSG_CRC		0x26  /* CRC Error */

#define USB2CAN_RP_MASK			0x7F  /* Mask for Receive Error Bit */


/* table of devices that work with this driver */
static struct usb_device_id usb2can_table [] = {
	{ USB_DEVICE(USB2CAN_VENDOR_ID, USB2CAN_PRODUCT_ID) },
	{ }					/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, usb2can_table);

struct usb2can_tx_urb_context {
	struct usb2can *dev;

	u32 echo_index;
	u8 dlc;
};

/* Structure to hold all of our device specific stuff */
struct usb2can {
	struct can_priv can; /* must be the first member */
	int open_time;

	struct sk_buff *echo_skb[MAX_TX_URBS];

	struct usb_device *udev;
	struct net_device *netdev;

	atomic_t active_tx_urbs;
	struct usb_anchor tx_submitted;
	struct usb2can_tx_urb_context tx_contexts[MAX_TX_URBS];

	struct usb_anchor rx_submitted;

	struct urb *intr_urb;

	u8 *cmd_msg_buffer;

	u8 *intr_in_buffer;
	unsigned int free_slots; /* remember number of available slots */
};

/* tx frame */
struct __packed usb2can_tx_msg {
	u8 begin;
	u8 flags;	/* RTR and EXT_ID flag */
	__be32 id;	/* upper 3 bits not used */
	u8 dlc;		/* data length code 0-8 bytes */
	u8 data[8];	/* 64-bit data */
	u8 end;
};

/* rx frame */
struct __packed usb2can_rx_msg {
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
struct __packed usb2can_cmd_msg {
	u8 begin;
	u8 channel;	/* unkown - always 0 */
	u8 command;	/* command to execute */
	u8 opt1;	/* optional parameter / return value */
	u8 opt2;	/* optional parameter 2 */
	u8 data[10];	/* optional parameter and data */
	u8 end;
};

static struct usb_driver usb2can_driver;

static int usb2can_send_cmd_msg(struct usb2can *dev, u8 *msg, int size)
{
	int actual_length;

	return usb_bulk_msg(dev->udev,
			    usb_sndbulkpipe(dev->udev, 4),
			    msg,
			    size,
			    &actual_length,
			    1000);
}

static int usb2can_wait_cmd_msg(struct usb2can *dev, u8 *msg, int size,
			        int *actual_length)
{
	return usb_bulk_msg(dev->udev,
			    usb_rcvbulkpipe(dev->udev, 3),
			    msg,
			    size,
			    actual_length,
			    1000);
}

/*
 * Send command to device and receive result.
 * Command was successful When opt1 = 0.
 */
static int usb2can_send_cmd(struct usb2can *dev, struct usb2can_cmd_msg *out,
			    struct usb2can_cmd_msg *in)
{
	int	err;
	int	nBytesRead;
	struct net_device *netdev;

	netdev = dev->netdev;

	out->begin = USB2CAN_CMD_START;
	out->end = USB2CAN_CMD_END;

	memcpy(&dev->cmd_msg_buffer[0], out,
		sizeof(struct usb2can_cmd_msg));

	err = usb2can_send_cmd_msg(dev, &dev->cmd_msg_buffer[0],
				   sizeof(struct usb2can_cmd_msg));
	if (err < 0) {
		dev_err(netdev->dev.parent, "sending command message failed\n");
		return err;
	}

	err = usb2can_wait_cmd_msg(dev, &dev->cmd_msg_buffer[0],
				   sizeof(struct usb2can_cmd_msg), &nBytesRead);
	if (err < 0) {
		dev_err(netdev->dev.parent, "no command message answer\n");
		return err;
	}

	memcpy(in, &dev->cmd_msg_buffer[0],
		sizeof(struct usb2can_cmd_msg));

	if (in->begin != USB2CAN_CMD_START || in->end != USB2CAN_CMD_END ||
			nBytesRead != 16 || in->opt1 != 0)
		return -EPROTO;

	return 0;
}

/*
 * Send open command to device
 */
static int usb2can_cmd_open(struct usb2can *dev, u8 speed, u8 tseg1, u8 tseg2,
			    u8 sjw, u16 brp, u32 ctrlmode)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	u32 flags = 0x00000000;
	u32 beflags;
	u16 bebrp;

	if (ctrlmode & CAN_CTRLMODE_LOOPBACK)
		flags |= USB2CAN_LOOPBACK;
	if (ctrlmode & CAN_CTRLMODE_LISTENONLY)
		flags |= USB2CAN_SILENT;

	flags |= USB2CAN_STATUS_FRAME;

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_OPEN;
	outmsg.opt1    = speed;
	outmsg.data[0] = tseg1;
	outmsg.data[1] = tseg2;
	outmsg.data[2] = sjw;

	// BRP
	bebrp = cpu_to_be16(brp);
	memcpy(&outmsg.data[3], &beflags, sizeof(bebrp));

	// flags
	beflags = cpu_to_be32(flags);
	memcpy(&outmsg.data[5], &beflags, sizeof(beflags));

	return usb2can_send_cmd(dev, &outmsg, &inmsg);
}

/*
 * Send close command to device
 */
static int usb2can_cmd_close(struct usb2can *dev)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_CLOSE;

	return usb2can_send_cmd(dev, &outmsg, &inmsg);
}

/*
 * Get firmware and hardware version
 */
static int usb2can_cmd_version(struct usb2can *dev, u32 *res)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	int err = 0;
	u32 *value;

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_GET_SOFTW_HARDW_VER;

	err = usb2can_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return err;

	value = (u32 *) inmsg.data;
	*res = be32_to_cpu(*value);

	return err;
}

/*
 * Get firmware version
 */
static ssize_t show_firmware(struct device *d, struct device_attribute *attr, char *buf)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	int err = 0;
	u16 *value;
	u16 result;
	struct usb_interface *intf = to_usb_interface(d);
	struct usb2can *dev = usb_get_intfdata(intf);

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_GET_SOFTW_VER;

	err = usb2can_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return -EIO;

	value = (u16 *) inmsg.data;
	result = be16_to_cpu(*value);

	return sprintf(buf, "%d.%d\n", (u8)(result>>8), (u8)result);
}

/*
 * Get hardware version
 */
static ssize_t show_hardware(struct device *d, struct device_attribute *attr, char *buf)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	int err = 0;
	u16 *value;
	u16 result;
	struct usb_interface *intf = to_usb_interface(d);
	struct usb2can *dev = usb_get_intfdata(intf);

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_GET_HARDW_VER;

	err = usb2can_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return -EIO;

	value = (u16 *) inmsg.data;
	result = be16_to_cpu(*value);

	return sprintf(buf, "%d.%d\n", (u8)(result>>8), (u8)result);
}

/*
 * Get statistic values
 */
static ssize_t show_statistics(struct device *d, struct device_attribute *attr, u8 statistic, char *buf)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	int err = 0;
	u32 *value;
	u32 result;
	struct usb_interface *intf = to_usb_interface(d);
	struct usb2can *dev = usb_get_intfdata(intf);

	memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
	outmsg.command = USB2CAN_GET_STATISTICS;
	outmsg.opt1 = statistic;

	err = usb2can_send_cmd(dev, &outmsg, &inmsg);
	if (err)
		return -EIO;

	value = (u32 *) inmsg.data;
	result = be32_to_cpu(*value);

	return sprintf(buf, "%d\n", result);
}

static ssize_t show_rx_frames(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_RX_FRAMES, buf);
}

static ssize_t show_rx_bytes(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_RX_BYTES, buf);
}

static ssize_t show_tx_frames(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_TX_FRAMES, buf);
}

static ssize_t show_tx_bytes(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_RX_BYTES, buf);
}

static ssize_t show_overruns(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_OVERRUNS, buf);
}

static ssize_t show_warnings(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_WARNINGS, buf);
}

static ssize_t show_bus_off(struct device *d, struct device_attribute *attr, char *buf)
{
	return show_statistics(d, attr, USB2CAN_STAT_BUS_OFF, buf);
}

/*
 * Reset statistics
 */
static ssize_t reset_statistics(struct device *d, struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb2can_cmd_msg	outmsg;
	struct usb2can_cmd_msg	inmsg;
	int err = 0;
	struct usb_interface *intf = to_usb_interface(d);
	struct usb2can *dev = usb_get_intfdata(intf);

	if (buf[0] == '1') {
		memset(&outmsg, 0, sizeof(struct usb2can_cmd_msg));
		outmsg.command = USB2CAN_GET_STATISTICS;
		outmsg.opt1 = USB2CAN_STAT_RESET_STAT;

		err = usb2can_send_cmd(dev, &outmsg, &inmsg);
		if (err)
			return -EIO;
	}

	return count;
}

static DEVICE_ATTR(firmware, S_IRUGO, show_firmware, NULL);
static DEVICE_ATTR(hardware, S_IRUGO, show_hardware, NULL);
static DEVICE_ATTR(can_rx_frames, S_IRUGO, show_rx_frames, NULL);
static DEVICE_ATTR(can_rx_bytes, S_IRUGO, show_rx_bytes, NULL);
static DEVICE_ATTR(can_tx_frames, S_IRUGO, show_tx_frames, NULL);
static DEVICE_ATTR(can_tx_bytes, S_IRUGO, show_tx_bytes, NULL);
static DEVICE_ATTR(can_overruns, S_IRUGO, show_overruns, NULL);
static DEVICE_ATTR(can_warnings, S_IRUGO, show_warnings, NULL);
static DEVICE_ATTR(can_bus_off_counter, S_IRUGO, show_bus_off, NULL);

static DEVICE_ATTR(can_reset_statistics, S_IWUSR, NULL, reset_statistics);

/*
 * Set network device mode
 *
 * Maybe we should leave this function empty, because the device
 * set mode variable with open command.
 */
static int usb2can_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct usb2can *dev = netdev_priv(netdev);
	struct can_bittiming *bt = &dev->can.bittiming;
	int err = 0;

	if (!dev->open_time)
		return -EINVAL;

	switch (mode) {
	case CAN_MODE_START:
		err = usb2can_cmd_open(dev, USB2CAN_BAUD_MANUAL,
				bt->phase_seg1, bt->phase_seg2,
				bt->sjw, bt->brp, dev->can.ctrlmode);
		if (err)
			dev_warn(netdev->dev.parent, "couldn't start device");

		if (netif_queue_stopped(netdev))
			netif_wake_queue(netdev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/*
 * Set network device bittiming
 *
 * Maybe we should leave this function empty, because the device
 * set bittiming variable with open command.
 */
static int usb2can_set_bittiming(struct net_device *netdev)
{
	struct usb2can *dev = netdev_priv(netdev);
	struct can_bittiming *bt = &dev->can.bittiming;
	int err = 0;

       	err = usb2can_cmd_close(dev);
	if (err)
		dev_warn(netdev->dev.parent, "couldn't stop device");

       	err = usb2can_cmd_open(dev, USB2CAN_BAUD_MANUAL,
			bt->phase_seg1, bt->phase_seg2,
			bt->sjw, bt->brp, dev->can.ctrlmode);
	if (err)
		dev_warn(netdev->dev.parent, "couldn't start device");
	return err;
}

static void usb2can_read_interrupt_callback(struct urb *urb)
{
	struct usb2can *dev = urb->context;
	struct net_device *netdev = dev->netdev;
	int err;

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0:
		dev->free_slots = dev->intr_in_buffer[1];
		break;

	case -ECONNRESET: /* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		return;

	default:
		dev_info(netdev->dev.parent, "Rx interrupt aborted %d\n",
			 urb->status);
		break;
	}

	err = usb_submit_urb(urb, GFP_ATOMIC);

	if (err == -ENODEV)
		netif_device_detach(netdev);
	else if (err)
		dev_err(netdev->dev.parent,
			"failed resubmitting intr urb: %d\n", err);
}

/*
 * Read data and status frames
 */
static void usb2can_rx_can_msg(struct usb2can *dev, struct usb2can_rx_msg *msg)
{
	struct can_frame *cf;
	struct sk_buff *skb;
	int i;
	struct net_device_stats *stats = &dev->netdev->stats;

        skb = alloc_can_skb(dev->netdev, &cf);
	if (skb == NULL)
		return;

	if (msg->type == USB2CAN_TYPE_CAN_FRAME) {
		cf->can_id = be32_to_cpu(msg->id);
		cf->can_dlc = get_can_dlc(msg->dlc & 0xF);

		if (msg->flags & USB2CAN_EXTID)
		cf->can_id |= CAN_EFF_FLAG;

		if (msg->flags & USB2CAN_RTR) {
			cf->can_id |= CAN_RTR_FLAG;
		} else {
			for (i = 0; i < cf->can_dlc; i++)
				cf->data[i] = msg->data[i];
		}
	} else if (msg->type == USB2CAN_TYPE_ERROR_FRAME &&
		   msg->flags == USB2CAN_ERR_FLAG) {

		/*
		 * Error message
		 *
		 * byte 0: Status
		 * byte 1: bit   7: Receive Passive
		 * byte 1: bit 0-6: Receive Error Counter
		 * byte 2: Transmit Error Counter
		 * byte 3: Always 0 (maybe reserved for future use)
		 */

		u8 state = msg->data[0];
		u8 rxerr = msg->data[1] & USB2CAN_RP_MASK;
		u8 txerr = msg->data[2];
		int rx_errors = 0;
		int tx_errors = 0;

		dev->can.can_stats.bus_error++;

		cf->can_id = CAN_ERR_FLAG;
		cf->can_dlc = CAN_ERR_DLC;

		switch (state) {
			case USB2CAN_STATUSMSG_OK:
				dev->can.state = CAN_STATE_ERROR_ACTIVE;
		                cf->can_id |= CAN_ERR_PROT;
				cf->data[2] = CAN_ERR_PROT_ACTIVE;
				break;
			case USB2CAN_STATUSMSG_BUSOFF:
				dev->can.state = CAN_STATE_BUS_OFF;
				cf->can_id |= CAN_ERR_BUSOFF;
				can_bus_off(dev->netdev);
				break;
			default:
				dev->can.state = CAN_STATE_ERROR_WARNING;
				cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
				break;
		}

		switch (state) {
			case USB2CAN_STATUSMSG_ACK:
				cf->can_id |= CAN_ERR_ACK;
				tx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_CRC:
				cf->data[2] |= CAN_ERR_PROT_BIT;
				rx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_BIT0:
				cf->data[2] |= CAN_ERR_PROT_BIT0;
				tx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_BIT1:
				cf->data[2] |= CAN_ERR_PROT_BIT1;
				tx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_FORM:
				cf->data[2] |= CAN_ERR_PROT_FORM;
				rx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_STUFF:
				cf->data[2] |= CAN_ERR_PROT_STUFF;
				rx_errors = 1;
				break;
			case USB2CAN_STATUSMSG_OVERRUN:
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_OVERFLOW :
					CAN_ERR_CRTL_RX_OVERFLOW;
				cf->data[2] |= CAN_ERR_PROT_OVERLOAD;
				stats->rx_over_errors++;
				break;
			case USB2CAN_STATUSMSG_BUSLIGHT:
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
				dev->can.can_stats.error_warning++;
				break;
			case USB2CAN_STATUSMSG_BUSHEAVY:
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_PASSIVE :
					CAN_ERR_CRTL_RX_PASSIVE;
				dev->can.can_stats.error_passive++;
				break;
			default:
				cf->data[2] |= CAN_ERR_PROT_UNSPEC;
				break;
		}

		if (tx_errors) {
			cf->data[2] |= CAN_ERR_PROT_TX;
			stats->tx_errors++;
		}
		if (rx_errors) {
			stats->rx_errors++;
		}

		cf->data[6] = txerr;
		cf->data[7] = rxerr;
		
	} else {
		dev_warn(dev->udev->dev.parent, "frame type %d unknown",
		         msg->type);
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
static void usb2can_read_bulk_callback(struct urb *urb)
{
	struct usb2can *dev = urb->context;
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
		dev_info(netdev->dev.parent, "Rx URB aborted (%d)\n",
			 urb->status);
		goto resubmit_urb;
	}

	while (pos < urb->actual_length) {
		struct usb2can_rx_msg *msg;

		msg = (struct usb2can_rx_msg *)(urb->transfer_buffer + pos);

		usb2can_rx_can_msg(dev, msg);

		pos += sizeof(struct usb2can_rx_msg);

		if (pos > urb->actual_length) {
			dev_err(dev->udev->dev.parent, "format error\n");
			break;
		}
	}

resubmit_urb:
	usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 1),
			  urb->transfer_buffer, RX_BUFFER_SIZE,
			  usb2can_read_bulk_callback, dev);

	retval = usb_submit_urb(urb, GFP_ATOMIC);

	if (retval == -ENODEV)
		netif_device_detach(netdev);
	else if (retval)
		dev_err(netdev->dev.parent,
			"failed resubmitting read bulk urb: %d\n", retval);
}

/*
 * Callback handler for write operations
 *
 * Free allocated buffers, check transmit status and
 * calculate statistic.
 */
static void usb2can_write_bulk_callback(struct urb *urb)
{
	struct usb2can_tx_urb_context *context = urb->context;
	struct usb2can *dev;
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
		dev_info(netdev->dev.parent, "Tx URB aborted (%d)\n",
			 urb->status);

	netdev->trans_start = jiffies;

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += context->dlc;

	can_get_echo_skb(netdev, context->echo_index);

	/* Release context */
	context->echo_index = MAX_TX_URBS;

	if (netif_queue_stopped(netdev))
		netif_wake_queue(netdev);
}

/*
 * Send data to device
 */
static netdev_tx_t usb2can_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	struct usb2can *dev = netdev_priv(netdev);
	struct net_device_stats *stats = &netdev->stats;
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct usb2can_tx_msg *msg;
	struct urb *urb;
	struct usb2can_tx_urb_context *context = NULL;
	u8 *buf;
	int i, err;
	size_t size = sizeof(struct usb2can_tx_msg);

	if (can_dropped_invalid_skb(netdev, skb))
		return NETDEV_TX_OK;

	/* create a URB, and a buffer for it, and copy the data to the URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		dev_err(netdev->dev.parent, "No memory left for URBs\n");
		goto nomem;
	}

	buf = usb_alloc_coherent(dev->udev, size, GFP_ATOMIC,
				 &urb->transfer_dma);
	if (!buf) {
		dev_err(netdev->dev.parent, "No memory left for USB buffer\n");
		usb_free_urb(urb);
		goto nomem;
	}

	memset(buf, 0, size);

	msg = (struct usb2can_tx_msg *)buf;
	msg->begin = USB2CAN_DATA_START;
	msg->flags = 0x00;

	if (cf->can_id & CAN_RTR_FLAG)
		msg->flags |= USB2CAN_RTR;

	if (cf->can_id & CAN_EFF_FLAG)
		msg->flags |= USB2CAN_EXTID;

	msg->id = cpu_to_be32(cf->can_id & CAN_ERR_MASK);

	msg->dlc = cf->can_dlc;

	for (i = 0; i < cf->can_dlc; i++)
		msg->data[i] = cf->data[i];

	msg->end = USB2CAN_DATA_END;


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

		dev_warn(netdev->dev.parent, "couldn't find free context");

		return NETDEV_TX_BUSY;
	}

	context->dev = dev;
	context->echo_index = i;
	context->dlc = cf->can_dlc;

	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, 2), buf,
			  size, usb2can_write_bulk_callback, context);
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
			dev_warn(netdev->dev.parent, "failed tx_urb %d\n", err);

			stats->tx_dropped++;
		}
	} else {
		netdev->trans_start = jiffies;

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


/*
 * Start USB device
 */
static int usb2can_start(struct usb2can *dev)
{
	struct net_device *netdev = dev->netdev;
	int err, i;
	struct can_bittiming *bt = &dev->can.bittiming;

	dev->intr_in_buffer[0] = 0;
	dev->free_slots = 15; /* initial size */

	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf = NULL;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			dev_err(netdev->dev.parent,
				"No memory left for URBs\n");
			return -ENOMEM;
		}

		buf = usb_alloc_coherent(dev->udev, RX_BUFFER_SIZE, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			dev_err(netdev->dev.parent,
				"No memory left for USB buffer\n");
			usb_free_urb(urb);
			return -ENOMEM;
		}

		usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 1),
				  buf, RX_BUFFER_SIZE,
				  usb2can_read_bulk_callback, dev);
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
		dev_warn(netdev->dev.parent, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't transmit all the URBs */
	if (i < MAX_RX_URBS)
		dev_warn(netdev->dev.parent, "rx performance may be slow\n");

	/* Setup and start interrupt URB */
	usb_fill_int_urb(dev->intr_urb, dev->udev,
			 usb_rcvintpipe(dev->udev, 1),
			 dev->intr_in_buffer,
			 INTR_IN_BUFFER_SIZE,
			 usb2can_read_interrupt_callback, dev, 1);

	err = usb_submit_urb(dev->intr_urb, GFP_KERNEL);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(dev->netdev);

		dev_warn(netdev->dev.parent, "intr URB submit failed: %d\n",
			 err);

		return err;
	}

	err = usb2can_cmd_open(dev, USB2CAN_BAUD_MANUAL, bt->phase_seg1,
			       bt->phase_seg2, bt->sjw, bt->brp,
			       dev->can.ctrlmode);
	if (err)
		goto failed;

	dev->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;

failed:
	if (err == -ENODEV)
		netif_device_detach(dev->netdev);

	dev_warn(netdev->dev.parent, "couldn't submit control: %d\n", err);

	return err;
}

/*
 * Open USB device
 */
static int usb2can_open(struct net_device *netdev)
{
	struct usb2can *dev = netdev_priv(netdev);
	int err;

	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	/* finally start device */
	err = usb2can_start(dev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(dev->netdev);

		dev_warn(netdev->dev.parent, "couldn't start device: %d\n",
			 err);

		close_candev(netdev);

		return err;
	}

	dev->open_time = jiffies;

	netif_start_queue(netdev);

	return 0;
}

static void unlink_all_urbs(struct usb2can *dev)
{
	int i;

	usb_unlink_urb(dev->intr_urb);

	usb_kill_anchored_urbs(&dev->rx_submitted);

	usb_kill_anchored_urbs(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		dev->tx_contexts[i].echo_index = MAX_TX_URBS;
}

/*
 * Close USB device
 */
static int usb2can_close(struct net_device *netdev)
{
	struct usb2can *dev = netdev_priv(netdev);
	int err = 0;

	/* Send CLOSE command to CAN controller */
	err = usb2can_cmd_close(dev);
	if (err)
		dev_warn(netdev->dev.parent, "couldn't stop device");

	dev->can.state = CAN_STATE_STOPPED;

	/* Stop polling */
	unlink_all_urbs(dev);

	netif_stop_queue(netdev);

	close_candev(netdev);

	dev->open_time = 0;

	return err;
}

static const struct net_device_ops usb2can_netdev_ops = {
	.ndo_open = usb2can_open,
	.ndo_stop = usb2can_close,
	.ndo_start_xmit = usb2can_start_xmit,
};

static struct can_bittiming_const usb2can_bittiming_const = {
	.name = "usb2can",
	.tseg1_min = USB2CAN_TSEG1_MIN,
	.tseg1_max = USB2CAN_TSEG1_MAX,
	.tseg2_min = USB2CAN_TSEG2_MIN,
	.tseg2_max = USB2CAN_TSEG2_MAX,
	.sjw_max = USB2CAN_SJW_MAX,
	.brp_min = USB2CAN_BRP_MIN,
	.brp_max = USB2CAN_BRP_MAX,
	.brp_inc = USB2CAN_BRP_INC,
};

/*
 * Probe USB device
 *
 * Check device and firmware.
 * Set supported modes and bittiming constants.
 * Allocate some memory.
 */
static int usb2can_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct net_device *netdev;
	struct usb2can *dev;
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


	netdev = alloc_candev(sizeof(struct usb2can), MAX_TX_URBS);
	if (!netdev) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	dev = netdev_priv(netdev);

	dev->udev = usbdev;
	dev->netdev = netdev;

	dev->can.state = CAN_STATE_STOPPED;
	dev->can.clock.freq = USB2CAN_ABP_CLOCK;
	dev->can.bittiming_const = &usb2can_bittiming_const;
	dev->can.do_set_bittiming = usb2can_set_bittiming;
	dev->can.do_set_mode = usb2can_set_mode;
	dev->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				      CAN_CTRLMODE_LISTENONLY;

	netdev->netdev_ops = &usb2can_netdev_ops;

	netdev->flags |= IFF_ECHO; /* we support local echo */

	init_usb_anchor(&dev->rx_submitted);

	init_usb_anchor(&dev->tx_submitted);
	atomic_set(&dev->active_tx_urbs, 0);

	for (i = 0; i < MAX_TX_URBS; i++)
		dev->tx_contexts[i].echo_index = MAX_TX_URBS;

	dev->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->intr_urb) {
		dev_err(&intf->dev, "Couldn't alloc intr URB\n");
		goto cleanup_candev;
	}

	dev->intr_in_buffer = kzalloc(INTR_IN_BUFFER_SIZE, GFP_KERNEL);
	if (!dev->intr_in_buffer) {
		dev_err(&intf->dev, "Couldn't alloc Intr buffer\n");
		goto cleanup_intr_urb;
	}

	dev->cmd_msg_buffer = kzalloc(sizeof(struct usb2can_cmd_msg),
				      GFP_KERNEL);
	if (!dev->cmd_msg_buffer) {
		dev_err(&intf->dev, "Couldn't alloc Tx buffer\n");
		goto cleanup_intr_in_buffer;
	}

	usb_set_intfdata(intf, dev);

	SET_NETDEV_DEV(netdev, &intf->dev);

	err = usb2can_cmd_version(dev, &version);
	if (err) {
		dev_err(netdev->dev.parent, "can't get firmware version\n");
		goto cleanup_cmd_msg_buffer;
	} else {
		dev_info(netdev->dev.parent,
			 "firmware: %d.%d, hardware: %d.%d\n",
			 (u8)(version>>24),(u8)(version>>16),
			 (u8)(version>>8), (u8)version);
	}

	err = register_candev(netdev);
	if (err) {
		dev_err(netdev->dev.parent,
			"couldn't register CAN device: %d\n", err);
		goto cleanup_cmd_msg_buffer;
	}

	if (device_create_file(&intf->dev, &dev_attr_firmware))
		dev_err(&intf->dev,
			"Couldn't create device file for firmware\n");

	if (device_create_file(&intf->dev, &dev_attr_hardware))
		dev_err(&intf->dev,
			"Couldn't create device file for hardware\n");

	if (device_create_file(&intf->dev, &dev_attr_can_rx_frames))
		dev_err(&intf->dev,
			"Couldn't create device file for can_rx_frames\n");

	if (device_create_file(&intf->dev, &dev_attr_can_rx_bytes))
		dev_err(&intf->dev,
			"Couldn't create device file for can_rx_bytes\n");

	if (device_create_file(&intf->dev, &dev_attr_can_tx_frames))
		dev_err(&intf->dev,
			"Couldn't create device file for can_tx_frames\n");

	if (device_create_file(&intf->dev, &dev_attr_can_tx_bytes))
		dev_err(&intf->dev,
			"Couldn't create device file for can_tx_bytes\n");

	if (device_create_file(&intf->dev, &dev_attr_can_overruns))
		dev_err(&intf->dev,
			"Couldn't create device file for can_overruns\n");

	if (device_create_file(&intf->dev, &dev_attr_can_warnings))
		dev_err(&intf->dev,
			"Couldn't create device file for can_warnings\n");

	if (device_create_file(&intf->dev, &dev_attr_can_bus_off_counter))
		dev_err(&intf->dev,
			"Couldn't create device file for can_bus_off_counter\n");

	if (device_create_file(&intf->dev, &dev_attr_can_reset_statistics))
		dev_err(&intf->dev,
			"Couldn't create device file for can_reset_statistics\n");

	/* let the user know what node this device is now attached to */
	dev_info(netdev->dev.parent, "device registered as %s\n", netdev->name);
	return 0;

cleanup_cmd_msg_buffer:
	kfree(dev->cmd_msg_buffer);

cleanup_intr_in_buffer:
	kfree(dev->intr_in_buffer);

cleanup_intr_urb:
	usb_free_urb(dev->intr_urb);

cleanup_candev:
	free_candev(netdev);

	return err;

}

/*
 * Called by the usb core when driver is unloaded or device is removed
 */
static void usb2can_disconnect(struct usb_interface *intf)
{
	struct usb2can *dev = usb_get_intfdata(intf);

	device_remove_file(&intf->dev, &dev_attr_firmware);
	device_remove_file(&intf->dev, &dev_attr_hardware);
	device_remove_file(&intf->dev, &dev_attr_can_rx_frames);
	device_remove_file(&intf->dev, &dev_attr_can_rx_bytes);
	device_remove_file(&intf->dev, &dev_attr_can_tx_frames);
	device_remove_file(&intf->dev, &dev_attr_can_tx_bytes);
	device_remove_file(&intf->dev, &dev_attr_can_overruns);
	device_remove_file(&intf->dev, &dev_attr_can_warnings);
	device_remove_file(&intf->dev, &dev_attr_can_bus_off_counter);
	device_remove_file(&intf->dev, &dev_attr_can_reset_statistics);

	usb_set_intfdata(intf, NULL);

	if (dev) {
		dev_info(&intf->dev, "disconnect %s\n", dev->netdev->name);

		unregister_netdev(dev->netdev);
		free_candev(dev->netdev);

		unlink_all_urbs(dev);

		usb_free_urb(dev->intr_urb);

		kfree(dev->intr_in_buffer);
	}

}

static struct usb_driver usb2can_driver = {
	.name =		"usb2can",
	.probe =	usb2can_probe,
	.disconnect =	usb2can_disconnect,
	.id_table =	usb2can_table,
};

static int __init usb2can_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&usb2can_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb2can_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&usb2can_driver);
}

module_init(usb2can_init);
module_exit(usb2can_exit);

MODULE_AUTHOR("Bernd Krumboeck <krumboeck@universalnet.at>");
MODULE_DESCRIPTION("CAN driver for UAB 8 devices USB2CAN interfaces");
MODULE_LICENSE("GPL v2");

