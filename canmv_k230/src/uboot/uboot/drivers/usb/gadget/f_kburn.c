#include <command.h>
#include <config.h>
#include <common.h>
#include <env.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <part.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linux/compiler.h>
#include <g_dnl.h>
#include <stddef.h>

#include "kburn.h"

static void tx_done_handler(struct usb_ep *ep, struct usb_request *req);
static void rx_command_handler(struct usb_ep *ep, struct usb_request *req);

enum kburn_pkt_cmd {
    KBURN_CMD_NONE = 0,

    KBURN_CMD_DEV_PROBE = 0x10,
	KBURN_CMD_DEV_GET_INFO = 0x11,

	KBURN_CMD_WRITE_LBA = 0x20,
	KBURN_CMD_ERASE_LBA = 0x21,

    KBURN_CMD_MAX,
};

enum kburn_pkt_result {
    KBURN_RESULT_NONE = 0,

    KBURN_RESULT_OK = 1,
    KBURN_RESULT_ERROR = 2,

    KBURN_RESULT_ERROR_MSG = 0xFF,

    KBURN_RESULT_MAX,
};

#define KBUNR_USB_PKT_SIZE	(64)

struct kburn_usb_pkt {
    uint16_t cmd;
    uint16_t result; /* only valid in csw */
    uint8_t data_size;
    uint8_t data[0];
};

struct kburn_pkt_handler_t {
	enum kburn_pkt_cmd cmd;
	/* call back function to handle rockusb command */
	void (*cb)(struct usb_ep *ep, struct usb_request *req);
};

struct kburn_usb_t {
    struct usb_function usb_function;
	struct usb_ep *in_ep, *out_ep;
	struct usb_request *in_req, *out_req;

	struct kburn *burner;

	u64 offset;
	u64 dl_size;
	u64 dl_bytes;
	u64 ul_size;
	u64 ul_bytes;

    void *buf;
	void *buf_head;
};

static struct usb_endpoint_descriptor hs_ep_in = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN | 1,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_ep_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT | 2,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor fs_ep_in = {
	.bLength            = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType    = USB_DT_ENDPOINT,
	.bEndpointAddress   = USB_DIR_IN | 3,
	.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize     = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor fs_ep_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT | 4,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(64),
};

static struct usb_interface_descriptor interface_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= 0x01,
	.bAlternateSetting	= 0x00,
	.bNumEndpoints		= 0x02,
	.bInterfaceClass	= KBURN_USB_INTF_CLASS,
	.bInterfaceSubClass	= KBURN_USB_INTF_SUBCLASS,
	.bInterfaceProtocol	= KBURN_USB_INTF_PROTOCOL,
};

static struct usb_descriptor_header *kburn_fs_function[] = {
	(struct usb_descriptor_header *)&interface_desc,
	(struct usb_descriptor_header *)&fs_ep_in,
	(struct usb_descriptor_header *)&fs_ep_out,
};

static struct usb_descriptor_header *kburn_hs_function[] = {
	(struct usb_descriptor_header *)&interface_desc,
	(struct usb_descriptor_header *)&hs_ep_in,
	(struct usb_descriptor_header *)&hs_ep_out,
	NULL,
};

static const char kburn_name[] = "Canaan KBURN";

static struct usb_string kburn_string_defs[] = {
	[0].s = kburn_name,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_kburn = {
	.language	= 0x0409,	/* en-us */
	.strings	= kburn_string_defs,
};

static struct usb_gadget_strings *kburn_strings[] = {
	&stringtab_kburn,
	NULL,
};

static struct kburn_usb_t *s_kburn = NULL;

static inline struct kburn_usb_t *func_to_kburn(struct usb_function *f)
{
	return container_of(f, struct kburn_usb_t, usb_function);
}

struct kburn_usb_t *get_kburn_usb(void)
{
	struct kburn_usb_t *kburn_usb = s_kburn;

	if (!kburn_usb) {
		kburn_usb = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*kburn_usb));
		if (!kburn_usb)
			return NULL;

		s_kburn = kburn_usb;
		memset(kburn_usb, 0, sizeof(*kburn_usb));
	}

	if (!kburn_usb->buf_head) {
		kburn_usb->buf_head = memalign(CONFIG_SYS_CACHELINE_SIZE, KBURN_USB_BUFFER_SIZE);
		if (!kburn_usb->buf_head)
			return NULL;

		kburn_usb->buf = kburn_usb->buf_head;
		memset(kburn_usb->buf_head, 0, KBURN_USB_BUFFER_SIZE);
	}

	return kburn_usb;
}

static struct usb_endpoint_descriptor *kburn_ep_desc(
struct usb_gadget *g,
struct usb_endpoint_descriptor *fs,
struct usb_endpoint_descriptor *hs)
{
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return hs;
	return fs;
}

static int kburn_bind(struct usb_configuration *c, struct usb_function *f)
{
	int id;
	struct usb_gadget *gadget = c->cdev->gadget;
	struct kburn_usb_t *f_kburn = func_to_kburn(f);

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	interface_desc.bInterfaceNumber = id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;

	kburn_string_defs[0].id = id;
	interface_desc.iInterface = id;

    usb_gadget_vbus_draw(gadget, 500);

	f_kburn->in_ep = usb_ep_autoconfig(gadget, &fs_ep_in);
	if (!f_kburn->in_ep)
		return -ENODEV;
	f_kburn->in_ep->driver_data = c->cdev;

	f_kburn->out_ep = usb_ep_autoconfig(gadget, &fs_ep_out);
	if (!f_kburn->out_ep)
		return -ENODEV;
	f_kburn->out_ep->driver_data = c->cdev;

    return 0;
}

static void kburn_unbind(struct usb_configuration *c, struct usb_function *f)
{
	/* clear the configuration*/
	memset(s_kburn, 0, sizeof(*s_kburn));
}

static int kburn_handle_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_gadget *gadget = f->config->cdev->gadget;
	struct usb_request *req = f->config->cdev->req;

	u16		w_index = get_unaligned_le16(&ctrl->wIndex);
	u16		w_value = get_unaligned_le16(&ctrl->wValue);
	u16		w_length = get_unaligned_le16(&ctrl->wLength);
	u8		req_type = ctrl->bRequestType & USB_TYPE_MASK;

	int value = 0;

	printf("w_index 0x%x, w_value 0x%x, w_length %d, req_type %x\n", \
		w_index, w_value, w_length, req_type);

	if (USB_TYPE_VENDOR == (req_type & USB_TYPE_VENDOR)) {
		if ((0x00 == w_index) && (0x000 == w_value)) {
			const char *mark = "Uboot Stage for K230";

			strncpy(req->buf, mark, w_length);
			value = strlen(mark);
		}
	}

	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue(gadget->ep0, req, 0);
		if (value < 0) {
			debug("ep_queue --> %d\n", value);
			req->status = 0;
		}
	}

	return value;
}

static struct usb_request *kburn_start_ep(struct usb_ep *ep)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, 0);
	if (!req)
		return NULL;

	req->length = KBURN_USB_EP_BUFFER_SZIE;
	req->buf = memalign(CONFIG_SYS_CACHELINE_SIZE, KBURN_USB_EP_BUFFER_SZIE);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	memset(req->buf, 0, req->length);

	return req;
}

static void kburn_disable(struct usb_function *f)
{
	struct kburn_usb_t *f_kburn = func_to_kburn(f);

	usb_ep_disable(f_kburn->out_ep);
	usb_ep_disable(f_kburn->in_ep);

	if (f_kburn->out_req) {
		free(f_kburn->out_req->buf);
		usb_ep_free_request(f_kburn->out_ep, f_kburn->out_req);
		f_kburn->out_req = NULL;
	}

	if (f_kburn->in_req) {
		free(f_kburn->in_req->buf);
		usb_ep_free_request(f_kburn->in_ep, f_kburn->in_req);
		f_kburn->in_req = NULL;
	}

	if (f_kburn->buf_head) {
		free(f_kburn->buf_head);
		f_kburn->buf_head = NULL;
		f_kburn->buf = NULL;
	}

	if(f_kburn->burner) {
		kburn_destory(f_kburn->burner);
		f_kburn->burner = NULL;
	}
}

static int kburn_set_alt(struct usb_function *f, unsigned interface, unsigned alt)
{
	int ret;
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct kburn_usb_t *f_kburn = func_to_kburn(f);
	const struct usb_endpoint_descriptor *d;

	printf("%s: func: %s intf: %d alt: %d\n",
	      __func__, f->name, interface, alt);

	d = kburn_ep_desc(gadget, &fs_ep_out, &hs_ep_out);
	ret = usb_ep_enable(f_kburn->out_ep, d);
	if (ret) {
		printf("failed to enable out ep\n");
		return ret;
	}

	f_kburn->out_req = kburn_start_ep(f_kburn->out_ep);
	if (!f_kburn->out_req) {
		printf("failed to alloc out req\n");
		ret = -EINVAL;
		goto err;
	}
	f_kburn->out_req->complete = rx_command_handler;

	d = kburn_ep_desc(gadget, &fs_ep_in, &hs_ep_in);
	ret = usb_ep_enable(f_kburn->in_ep, d);
	if (ret) {
		printf("failed to enable in ep\n");
		goto err;
	}

	f_kburn->in_req = kburn_start_ep(f_kburn->in_ep);
	if (!f_kburn->in_req) {
		printf("failed alloc req in\n");
		ret = -EINVAL;
		goto err;
	}
	f_kburn->in_req->complete = tx_done_handler;

	ret = usb_ep_queue(f_kburn->out_ep, f_kburn->out_req, 0);
	if (ret)
		goto err;

	return 0;

err:
	kburn_disable(f);

	return ret;
}

static int kburn_add(struct usb_configuration *c)
{
	int status;

	struct kburn_usb_t *kburn_usb = get_kburn_usb();

	kburn_usb->usb_function.name = "f_kburn";
	kburn_usb->usb_function.strings = kburn_strings;
	kburn_usb->usb_function.descriptors = kburn_fs_function;
	kburn_usb->usb_function.hs_descriptors = kburn_hs_function;
	kburn_usb->usb_function.ss_descriptors = NULL;

	kburn_usb->usb_function.bind = kburn_bind;
	kburn_usb->usb_function.unbind = kburn_unbind;
	kburn_usb->usb_function.set_alt = kburn_set_alt;
	kburn_usb->usb_function.disable = kburn_disable;

	kburn_usb->usb_function.setup = kburn_handle_setup;

	// kburn_usb->usb_function.get_alt = kburn_get_alt;
	// kburn_usb->usb_function.suspend = kburn_suspend;
	// kburn_usb->usb_function.resume = kburn_resume;

	status = usb_add_function(c, &kburn_usb->usb_function);
	if (status) {
		free(kburn_usb->buf_head);
		free(kburn_usb);
		s_kburn = NULL;
	}

	return status;
}

DECLARE_GADGET_BIND_CALLBACK(usb_dnl_kburn, kburn_add);

static void kburn_print_pkt(struct kburn_usb_pkt *pkt)
{
    printf("cmd 0x%04x, result 0x%04x, size %d, data: ", pkt->cmd, pkt->result, pkt->data_size);
    for(int i = 0; i < pkt->data_size; i++) {
        printf("%02x ", pkt->data[i]);
    }
    printf("\n");
}

// User can not direct call this.
static int kburn_tx_write(const char *buffer, unsigned int buffer_size)
{
	struct usb_request *in_req = s_kburn->in_req;
	int ret;

	memcpy(in_req->buf, buffer, buffer_size);
	in_req->length = buffer_size;
	debug("Transferring 0x%x bytes\n", buffer_size);
	usb_ep_dequeue(s_kburn->in_ep, in_req);
	ret = usb_ep_queue(s_kburn->in_ep, in_req, 0);
	if (ret)
		printf("Error %d on queue\n", ret);
	return 0;
}

static int kburn_tx_result(uint16_t cmd, uint16_t result, uint8_t *data, uint8_t data_size)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, csw, KBUNR_USB_PKT_SIZE);

    if(data_size > (KBUNR_USB_PKT_SIZE - sizeof(struct kburn_usb_pkt))) {
        printf("tx too many data\n");
        return -1;
    }

    csw->cmd = 0x8000 | cmd;
    csw->result = result;
    csw->data_size = data_size;
	if(data_size) {
	    memcpy(csw->data, data, data_size);
	}

    return kburn_tx_write((char *)csw, KBUNR_USB_PKT_SIZE);
}

#define kburn_tx_string_result(c, r, d) kburn_tx_result(c, r, d, strlen(d))

static int kburn_tx_error_string(char *msg)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, csw, KBUNR_USB_PKT_SIZE);

    int msg_len = strlen(msg);

    if(msg_len > (KBUNR_USB_PKT_SIZE - sizeof(struct kburn_usb_pkt))) {
        printf("tx err msg too long\n");
        return -1;
    }

    csw->cmd = 0x8000 | KBURN_CMD_NONE;
    csw->result = KBURN_RESULT_ERROR_MSG;
    csw->data_size = msg_len;
    memcpy(csw->data, msg, msg_len);

    return kburn_tx_write((char *)csw, KBUNR_USB_PKT_SIZE);
}

static void tx_done_handler(struct usb_ep *ep, struct usb_request *req)
{
	int status = req->status;

	if (!status)
		return;

	printf("status: %d ep '%s' trans: %d\n", status, ep->name, req->actual);
}

static const struct kburn_pkt_handler_t pkt_handlers[];

static void rx_command_handler(struct usb_ep *ep, struct usb_request *req)
{
	void (*func_cb)(struct usb_ep *ep, struct usb_request *req) = NULL;

	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, cbw, KBUNR_USB_PKT_SIZE);
	char *cmdbuf = req->buf;
	int i;

	if (req->status || req->length == 0)
		return;

	memcpy((char *)cbw, req->buf, KBUNR_USB_PKT_SIZE);

    kburn_print_pkt(cbw);

	for (i = 0; pkt_handlers[i].cb; i++) {
		if (pkt_handlers[i].cmd == cbw->cmd) {
			func_cb = pkt_handlers[i].cb;
			break;
		}
	}

	if (!func_cb) {
		printf("unknown command: %s\n", (char *)req->buf);
		kburn_tx_error_string("FAILunknown command");
	} else {
		if (req->actual < req->length) {
			u8 *buf = (u8 *)req->buf;

			buf[req->actual] = 0;
			func_cb(ep, req);
		} else {
			puts("buffer overflow\n");
			kburn_tx_error_string("FAILbuffer overflow");
		}
	}

	*cmdbuf = '\0';
	req->actual = 0;
	usb_ep_queue(ep, req, 0);
}

static void cb_probe_device(struct usb_ep *ep, struct usb_request *req)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, cbw, KBUNR_USB_PKT_SIZE);

	struct kburn_usb_t *kburn_usb = get_kburn_usb();

	uint8_t index = 0;
	enum KBURN_MEDIA_TYPE type = KBURN_MEDIA_NONE;
	uint64_t result[1] = {KBURN_USB_EP_BUFFER_SZIE};

	memcpy((char *)cbw, req->buf, KBUNR_USB_PKT_SIZE);

	if(cbw->data_size != 2) {
		kburn_tx_string_result(KBURN_CMD_DEV_PROBE, KBURN_RESULT_ERROR_MSG, "ERROR DATA SIZE");
		return;
	}

	type = cbw->data[0];
	index = cbw->data[1];
	if(kburn_usb->burner) {
		kburn_destory(kburn_usb->burner);
		kburn_usb->burner = NULL;
	}
	kburn_usb->burner = kburn_probe_media(type);

	if (kburn_usb->burner) {
		kburn_tx_result(KBURN_CMD_DEV_PROBE, KBURN_RESULT_OK, (uint8_t *)&result[0], sizeof(result));
	} else {
		kburn_tx_string_result(KBURN_CMD_DEV_PROBE, KBURN_RESULT_ERROR_MSG, "PROBE FAILED");
	}
}

static void cb_get_device_info(struct usb_ep *ep, struct usb_request *req)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, cbw, KBUNR_USB_PKT_SIZE);
	memcpy((char *)cbw, req->buf, KBUNR_USB_PKT_SIZE);

	struct kburn_usb_t *kburn_usb = get_kburn_usb();
	int result = -1;

	if(kburn_usb->burner) {
		result = kburn_get_medium_info(kburn_usb->burner);

		kburn_tx_result(KBURN_CMD_DEV_GET_INFO, 0x00 == result ? KBURN_RESULT_OK : KBURN_RESULT_ERROR, \
			(uint8_t *)&kburn_usb->burner->medium_info, sizeof(kburn_usb->burner->medium_info));
	} else {
		kburn_tx_string_result(KBURN_CMD_DEV_GET_INFO, KBURN_RESULT_ERROR_MSG, "RUNTIME ERROR");
	}
}

static unsigned int rx_bytes_expected(struct usb_ep *ep)
{
	struct kburn_usb_t *kburn_usb = get_kburn_usb();
	u64 rx_remain = kburn_usb->dl_size - kburn_usb->dl_bytes;
	u64 rem;
	u64 maxpacket = ep->maxpacket;

	if (rx_remain <= 0) {
		return 0;
	} else if (rx_remain > KBURN_USB_EP_BUFFER_SZIE) {
		return KBURN_USB_EP_BUFFER_SZIE;
	}

	rem = rx_remain % maxpacket;
	if (rem > 0) {
		rx_remain = rx_remain + (maxpacket - rem);
	}

	return (unsigned int)rx_remain;
}

static void rx_write_lba_handler(struct usb_ep *ep, struct usb_request *req)
{
	struct kburn_usb_t *kburn_usb = get_kburn_usb();
	unsigned int transfer_size = 0;
	const unsigned char *buffer = req->buf;
	unsigned int buffer_size = req->actual;

	transfer_size = kburn_usb->dl_size - kburn_usb->dl_bytes;

	if (req->status != 0) {
		printf("Bad status: %d\n", req->status);
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "ERROR STATUS");
		return;
	}

	if (buffer_size < transfer_size)
		transfer_size = buffer_size;

	memcpy((void *)kburn_usb->buf, buffer, transfer_size);
	kburn_usb->dl_bytes += transfer_size;

	u64 xfer_size = transfer_size;
	int result = kburn_write_medium(kburn_usb->burner, kburn_usb->offset, kburn_usb->buf, &xfer_size);
	if((0x00 != result) || (xfer_size != transfer_size)) {
		printf("write failed %d, %lld != %d\n", result, xfer_size, transfer_size);
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "WRITE ERROR");
		return;
	}

	kburn_usb->offset += transfer_size;

	if (kburn_usb->dl_bytes >= kburn_usb->dl_size) {
		req->complete = rx_command_handler;
		req->length = KBURN_USB_EP_BUFFER_SZIE;
		kburn_usb->buf = kburn_usb->buf_head;

		printf("write 0x%llx bytes done\n", kburn_usb->dl_size);
		kburn_usb->dl_size = 0;

		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_OK, "WRITE DONE");
	} else {
		req->length = rx_bytes_expected(ep);
		if (kburn_usb->buf == kburn_usb->buf_head)
			kburn_usb->buf = kburn_usb->buf_head + KBURN_USB_EP_BUFFER_SZIE;
		else
			kburn_usb->buf = kburn_usb->buf_head;

		// printf("remain %x bytes\n", req->length);
	}

	req->actual = 0;
	usb_ep_queue(ep, req, 0);
}

static void cb_write_lba(struct usb_ep *ep, struct usb_request *req)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, cbw, KBUNR_USB_PKT_SIZE);

	struct kburn_usb_t *kburn_usb = get_kburn_usb();

	u64 offset, size;

	memcpy((char *)cbw, req->buf, KBUNR_USB_PKT_SIZE);

	if(cbw->data_size != 16) {
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "ERROR DATA SIZE");
		return;
	}

	offset = get_unaligned_le64(&cbw->data[0]);
	size = get_unaligned_le64(&cbw->data[8]);

	if(0x01 != kburn_usb->burner->medium_info.valid) {
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "MEDIUM INFO INVALID");
		return;
	}

	if(0x00 == size) {
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "DATA SIZE INVALID");
		return;
	}

	if ((offset + size) > kburn_usb->burner->medium_info.capacity) {
		kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_ERROR_MSG, "DATA SIZE EXCEED");
		return;
	}

	kburn_usb->offset = offset;
	kburn_usb->dl_size = size;
	kburn_usb->dl_bytes = 0;

	printf("require write %llx bytes to offset %llx\n", kburn_usb->dl_size, kburn_usb->offset);

	kburn_tx_string_result(KBURN_CMD_WRITE_LBA, KBURN_RESULT_OK, "START DL");

	req->complete = rx_write_lba_handler;
	req->length = rx_bytes_expected(ep);
}

static void cb_erase_lba(struct usb_ep *ep, struct usb_request *req)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct kburn_usb_pkt, cbw, KBUNR_USB_PKT_SIZE);

	struct kburn_usb_t *kburn_usb = get_kburn_usb();

	int result = -1;
	uint8_t data[16];

	u64 offset, size;

	memcpy((char *)cbw, req->buf, KBUNR_USB_PKT_SIZE);

	if(cbw->data_size != 16) {
		kburn_tx_string_result(KBURN_CMD_ERASE_LBA, KBURN_RESULT_ERROR_MSG, "ERROR DATA SIZE");
		return;
	}

	offset = get_unaligned_le64(&cbw->data[0]);
	size = get_unaligned_le64(&cbw->data[8]);

	if(kburn_usb->burner) {
		if(0x00 != size) {
			result = kburn_erase_medium(kburn_usb->burner, offset, &size);
		}
		put_unaligned_le64(offset, &data[0]);
		put_unaligned_le64(size, &data[8]);

		kburn_tx_result(KBURN_CMD_ERASE_LBA, 0x00 == result ? KBURN_RESULT_OK : KBURN_RESULT_ERROR, data, sizeof(data));
	} else {
		kburn_tx_string_result(KBURN_CMD_ERASE_LBA, KBURN_RESULT_ERROR_MSG, "RUNTIME ERROR");
	}
}

static void cb_not_support(struct usb_ep *ep, struct usb_request *req)
{
    kburn_tx_error_string("NOT SUPPORT FUNC");
}

static const struct kburn_pkt_handler_t pkt_handlers[] = {
    {
        .cmd = KBURN_CMD_NONE,
        .cb = cb_not_support,
    },
    {
        .cmd = KBURN_CMD_MAX,
        .cb = cb_not_support,
    },
    {
        .cmd = KBURN_CMD_DEV_PROBE,
        .cb = cb_probe_device,
    },
	{
		.cmd = KBURN_CMD_DEV_GET_INFO,
		.cb = cb_get_device_info,
	},
	{
		.cmd = KBURN_CMD_WRITE_LBA,
		.cb = cb_write_lba,
	},
	{
		.cmd = KBURN_CMD_ERASE_LBA,
		.cb = cb_erase_lba,
	},
    {
        // end of table
        .cb = NULL,
    }
};
