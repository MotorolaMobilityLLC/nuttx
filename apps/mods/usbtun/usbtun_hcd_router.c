/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <debug.h>
#include <errno.h>
#include <pthread.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/usb.h>
#include <nuttx/usb/usb.h>
#include <nuttx/bufram.h>
#include <nuttx/time.h>
#include <nuttx/util.h>

#include "nuttx/usbtun/usbtun_hcd_router.h"

#include "usbtun.h"

int mhb_send_hsic_status_not(uint8_t status);

/* HCD local defs */
#define SET_PORT_FEATURE 0x2303
#define CLR_PORT_FEATURE 0x2301
#define GET_PORT_STATUS  0xa300
#define PORT_POWERING_DELAY_IN_MS   100
#define PORT_RESETING_DELAY_IN_MS   100
#define PORT_RESET      0x4
#define PORT_POWER      0x8
#define ROOT_HUB_PORT   1

#define USB_SPEED_HIGH  3

#define SET_FEATURE           0x0003
#define SET_ADDRESS           0x0005
#define SET_CONFIGURATION     0x0009

#define USBHUB_CONFIG_ID       1
#define USBHUB_ADDRESS         2

#ifdef CONFIG_MODS_USB_HCD_TUN_PORT
#define USBHUB_EXTERNAL_PORT CONFIG_MODS_USB_HCD_TUN_PORT
#else
#define USBHUB_EXTERNAL_PORT 1
#endif

#define URB_MAX_PACKET  0x40

#define PORT_CONNECTED_MASK  (1 << 0)
#define PORT_LOW_SPEED_MASK  (1 << 9)
#define PORT_HIGH_SPEED_MASK (1 << 10)

#define PORT_CHANGE_CONNECTION  (1 << 0)
#define PORT_CHANGE_ENABLE      (1 << 1)
#define PORT_CHANGE_SUSPEND     (1 << 2)
#define PORT_CHANGE_OVERCURRENT (1 << 3)
#define PORT_CHANGE_RESET       (1 << 4)

#define C_PORT_CONNECTION  16
#define C_PORT_ENABLE      17
#define C_PORT_SUSPEND     18
#define C_PORT_OVERCURRENT 19
#define C_PORT_RESET       20

#define PCD_HANDSHAKE_WAIT_NS 100000000LL

#define NUM_CTRL_REQS 5
#define NUM_INT_IN_REQS 10
#define NUM_IN_REQS 50
#define NUM_INT_OUT_REQS 10
#define NUM_OUT_REQS 50

#define SETUP_REQ_SIZE 8
#define MAX_NUM_IFACES 256

#define EP_MAP_IN  (1 << 0)
#define EP_MAP_OUT (1 << 1)

typedef struct {
    uint16_t port_status;
    uint16_t status_change;
} port_status_t;

typedef struct {
    sq_entry_t entry;
    bool sync;
    bool tun;
    struct urb urb;
    usbtun_buf_t setup;
    usbtun_buf_t data;
} hcd_req_t;

typedef struct {
    uint8_t e2a[MAX_ENDPOINTS];
    uint8_t a2e[MAX_ENDPOINTS];
    uint8_t a2e_if[MAX_ENDPOINTS];
    uint8_t a2e_valid[MAX_ENDPOINTS];
} ep_map_t;

typedef struct {
    void *config_desc;
    size_t config_size;
    ep_map_t ep_map;
    uint8_t iface_settings[MAX_NUM_IFACES];
} hcd_config_desc_t;

struct hcd_router_data_s {
    struct device *dev;
    bool do_tunnel;
    bool hcd_ready;
    bool pcd_ready;
    hcd_config_desc_t *configs;
    uint8_t num_configs;
    uint8_t selected_config;
    struct usb_epdesc_s ep_list[MAX_ENDPOINTS];
    ep_map_t ep_map;
    /* This pointer points queue head for a host channel
     * assigned to the EP. Need to use same value across
     * all the request */
    void *hcpriv_ep[MAX_ENDPOINTS];
};

struct hcd_router_data_s s_data;

static pthread_t hcd_thread;
static pthread_mutex_t run_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t run_cond = PTHREAD_COND_INITIALIZER;
static bool do_run = false;
static bool port_change = false;

static bool init_ep_req(uint8_t local_eq, hcd_req_t *ep_urb, struct usb_epdesc_s *eq_info);
static void prep_tun_setup(hcd_req_t *req, usbtun_buf_t *setup, void *dout, uint16_t length);
static void prep_tun_data_out(hcd_req_t *req, usbtun_buf_t *buf);
static int usb_control_transfer_tun(struct device *dev, hcd_req_t *req, int direction, int addr);

static int send_to_unipro(hcd_req_t *req, size_t actual_length, uint8_t type, int16_t code) {
    uint8_t ep = req->urb.pipe.endpoint;
    uint8_t mapped = s_data.ep_map.e2a[ep];

    int ret = unipro_send_tunnel_cmd(mapped, type, code, req->data.ptr, actual_length);

    if (ret)
        lldbg("Failed to send data to mapped EP %d. (%d)\n", mapped, ret);

    return ret;
}

static usbtun_hdr_res_t handle_hdr(uint8_t ep, uint8_t type, int16_t code, size_t len) {
    if (len == 0) {
        if (ep == 0) {
            switch (type) {
            case PCD_ROUTER_READY:
                s_data.pcd_ready = true;
                break;
            default:
                lldbg("Unexpected hdr. type=%d\n", type);
                break;
            }
        } else {
            lldbg("Unexpected len = 0 hdr. ep=%d, type=%d\n", ep, type);
        }
        return USBTUN_NO_DATA;
    }

    if (!s_data.do_tunnel)
        return USBTUN_NO_DATA;

    return USBTUN_WAIT_DATA;
}

static void clean_hcd_req(hcd_req_t *req) {
    if (req) {
        usbtun_clean_mem(&req->setup);
        usbtun_clean_mem(&req->data);
    }
}

static usbtun_data_res_t handle_data_body(usbtun_buf_t *buf, uint8_t ep, uint8_t type) {
    bool free_buf = true;
    hcd_req_t *req;

    if (!s_data.do_tunnel)
        return USBTUN_FREE_BUF;

    if (ep == 0) {
        switch (type) {
        case PCD_SETUP:
        {
            struct usb_ctrlreq_s *ctrl = (struct usb_ctrlreq_s *)buf->ptr;
            uint16_t dlen = GETUINT16(ctrl->len);
            void *dout = NULL;
            if (dlen && USB_REQ_ISOUT(ctrl->type)) {
                dout = (char *)buf->ptr + sizeof(*ctrl);
            }

#ifdef USBTUN_DEBUG
            if (USBTUN_DEBUG_EP == 0) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                uint16_t value = GETUINT16(ctrl->value);
                uint16_t idx = GETUINT16(ctrl->index);
                lldbg("[%u:%ld] setup: type %02x, req %02x, value %04x, idx %04x, len = %d\n",
                      ts.tv_sec, ts.tv_nsec / 1000000,
                      ctrl->type, ctrl->req, value, idx, dlen);
            }
#endif
            int dir = USB_REQ_ISOUT(ctrl->type) ? USB_HOST_DIR_OUT : USB_HOST_DIR_IN;

            req = (hcd_req_t *)usbtun_req_dq(0);
            if (req) {
                free_buf = false;
                prep_tun_setup(req, buf, dout, dlen);

                usbtun_req_to_usb(0, &req->entry);
                if (usb_control_transfer_tun(s_data.dev, req, dir, 0)) {
                    lldbg("Failed to pass setup data\n");
                    clean_hcd_req(req);
                    usbtun_req_from_usb(0, &req->entry);
                    usbtun_req_q(0, &req->entry);
                }
            } else {
                lldbg("No more hcd_req to handle SETUP\n");
            }
            break;
        }
        default:
            lldbg("Unknow data type to handle\n");
        }
    } else {
        /* Non-control OUT EP */
        uint8_t local_ep = s_data.ep_map.a2e[ep];
        req = (hcd_req_t *)usbtun_req_dq(local_ep);

#ifdef USBTUN_DEBUG
        if (USBTUN_DEBUG_EP == local_ep) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            lldbg("[%u:%03ld] OUT EP %d, mapped (%d), len = %d, req %p\n",
                  ts.tv_sec, ts.tv_nsec / 1000000, local_ep, ep, buf->size, req);
        }
#endif

        if (req) {
            free_buf = false;
            prep_tun_data_out(req, buf);
            if (s_data.hcpriv_ep[ep])
                req->urb.hcpriv_ep = s_data.hcpriv_ep[ep];

            usbtun_req_to_usb(local_ep, &req->entry);
            if (device_usb_hcd_urb_enqueue(s_data.dev, &req->urb)) {
                lldbg("urb enqueue failed\n");
                clean_hcd_req(req);
                usbtun_req_from_usb(local_ep, &req->entry);
                usbtun_req_q(local_ep, &req->entry);
            } else {
                if (s_data.hcpriv_ep[ep] == 0)
                    s_data.hcpriv_ep[ep] = req->urb.hcpriv_ep;

                sem_wait(&req->urb.semaphore);
            }

        } else {
            lldbg("EP %d : no ep_urb available\n", local_ep);
        }
    }
    return free_buf ? USBTUN_FREE_BUF : USBTUN_KEEP_BUF;
}

static unsigned int get_hcd_ep_dir(uint8_t addr) {
    if (addr & 0x80)
        return USB_HOST_DIR_IN;

    return USB_HOST_DIR_OUT;
}
static unsigned int get_hcd_ep_type(uint8_t attr) {
    uint8_t type = attr & 0x03;
    if (type == 0)
        return  USB_HOST_PIPE_CONTROL;
    else if (type == 1)
        return  USB_HOST_PIPE_ISOCHRONOUS;
    else if (type == 2)
        return USB_HOST_PIPE_BULK;

    return USB_HOST_PIPE_INTERRUPT;
}

static unsigned int get_hcd_ep_interval(uint8_t ival) {
    if (ival == 0)
        return 0;

    /* TODO: The logic below is high speed only. Need to handle other speeds */
    return 1 << (ival - 1);
}

static bool set_one_mapping(uint8_t iface, struct usb_epdesc_s *desc, ep_map_t *ep_map) {

    /* APBridge has hard-coded EP config. */
    static const uint8_t hwcfg[] = { 0,
                                     EP_MAP_IN,  /* EP 1 - IN */
                                     EP_MAP_OUT, /* EP 2 - OUT */
                                     EP_MAP_IN,  /* EP 3 - IN */
                                     EP_MAP_OUT, /* EP 4 - OUT */
                                     EP_MAP_IN,  /* EP 5 - IN */
                                     EP_MAP_OUT, /* EP 6 - OUT */
                                     EP_MAP_IN,  /* EP 7 - IN */
                                     EP_MAP_OUT, /* EP 8 - OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 9  - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 10 - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 11 - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 12 - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 13 - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 14 - IN/OUT */
                                     EP_MAP_IN | EP_MAP_OUT, /* EP 15 - IN/OUT */
    };

    bool found = true;
    uint8_t ep_num = desc->addr & USB_EPNO_MASK;

    /* check if original EP number can be used as is */
    if ((USB_ISEPIN(desc->addr) && (hwcfg[ep_num] & EP_MAP_IN)) ||
        (USB_ISEPOUT(desc->addr) && (hwcfg[ep_num] & EP_MAP_OUT))) {
        if (ep_map->a2e_valid[ep_num]) {
            if (ep_map->a2e[ep_num] == ep_num && ep_map->a2e_if[ep_num] == iface) {
                found = true;
                goto found;
            }
        } else {
            ep_map->a2e_valid[ep_num] = 1;
            ep_map->a2e_if[ep_num] = iface;
            ep_map->a2e[ep_num] = ep_num;
            ep_map->e2a[ep_num] = ep_num;
            found = true;
            goto found;
        }
    }

    int idx;
    for (idx = 0; idx < MAX_ENDPOINTS; idx++) {
        if (USB_ISEPIN(desc->addr)) {
            if (!(hwcfg[idx] & EP_MAP_IN))
                continue;
        } else {
            if (!(hwcfg[idx] & EP_MAP_OUT))
                continue;
        }
        if (ep_map->a2e_valid[idx]) {
            if (ep_map->a2e[idx] == ep_num && ep_map->a2e_if[idx] == iface) {
                found = true;
                break;
            }
        } else {
            ep_map->a2e_valid[idx] = 1;
            ep_map->a2e_if[idx] = iface;
            ep_map->a2e[idx] = ep_num;
            ep_map->e2a[ep_num] = idx;
            found = true;
            break;
        }
    }

found:
    if (found)
        lldbg("IF %d: APBE[%s](%d) ==> APBA(%d)\n",
              iface, USB_ISEPIN(desc->addr) ? "IN" : "OUT", ep_num, idx);
    else
        lldbg("Mapping for EP %d cannot be set\n", ep_num);

    return found;
}

static bool update_ep_map(void *buf, size_t len, ep_map_t *ep_map) {
    if (len <= sizeof(struct usb_cfgdesc_s)) {
        return false;
    }

    memset(ep_map, 0, sizeof(*ep_map));

    uint8_t *ptr = (uint8_t *)buf;
    size_t total = 0;
    uint8_t iface = 0;
    while (total < len) {
        struct usb_desc_s *hdr = (struct usb_desc_s *)ptr;
        /* look for IFACE and EP descriptor and map EP number available on APBA */
        if (hdr->type == USB_DESC_TYPE_INTERFACE) {
            struct usb_ifdesc_s *desc = (struct usb_ifdesc_s *)ptr;
            iface = desc->ifno;
        } else if (hdr->type == USB_DESC_TYPE_ENDPOINT) {
            struct usb_epdesc_s *desc = (struct usb_epdesc_s *)ptr;
            if (set_one_mapping(iface, desc, ep_map)) {
                uint8_t ep_num = desc->addr & USB_EPNO_MASK;
                desc->addr &= ~USB_EPNO_MASK;
                desc->addr += ep_map->e2a[ep_num];
            }
        }
        ptr += hdr->len;
        total += hdr->len;
    }

    return true;
}

static bool prepare_config_list(void *buf, size_t len) {
    struct usb_devdesc_s *desc;

    if (len != sizeof(*desc)) {
        lldbg("Invalid device descriptor size\n");
        return false;
    }

    desc = buf;

    if (!desc->nconfigs) {
        lldbg("No config in device descriptor\n");
        return false;
    }

    if (s_data.configs)
        USBTUN_FREE(s_data.configs);

    s_data.configs = USBTUN_ALLOC(desc->nconfigs * sizeof(hcd_config_desc_t));

    if (!s_data.configs) {
        lldbg("Failed to allocate config_desc list\n");
        return false;
    }
    s_data.num_configs = desc->nconfigs;

    return true;
}

static bool prepare_config_item(uint8_t idx, void *buf, size_t len) {
    if (!s_data.configs) {
        lldbg("No device descriptor read yet\n");
        return false;
    }

    if (len <= sizeof(struct usb_cfgdesc_s)) {
        lldbg("Invalid config descriptor\n");
        return false;
    }

    struct usb_cfgdesc_s *desc = buf;

    uint16_t total = GETUINT16(desc->totallen);
    if (total != len) {
        lldbg("Imcomplete config. Not saved\n");
        return false;
    }

    /* look for GET_DESCRIPTOR for a Configuration descriptor */
    /* currently only assuming one */
    if (s_data.configs[idx].config_desc)
        USBTUN_FREE(s_data.configs[idx].config_desc);

    void *ptr = USBTUN_ALLOC(len);
    if (!ptr) {
        lldbg("Failed to allocate config descriptor storage\n");
        return false;
    }

    /* This function modifies all endpoint descriptors to match up
       with what APBridge supports. */
    update_ep_map(buf, len, &s_data.configs[idx].ep_map);
    memcpy(ptr, buf, len);
    s_data.configs[idx].config_desc = ptr;
    s_data.configs[idx].config_size = len;

    lldbg("Cached configuration at idx = %d\n", idx);

    return true;
}

static hcd_config_desc_t *find_selected_config(void) {
    /* non-zero config must be selected */
    if (!s_data.selected_config) {
        lldbg("Config not selected yet\n");
        return NULL;
    }

    int i;
    hcd_config_desc_t *cdesc = NULL;
    for (i = 0; i < s_data.num_configs; i++) {
        if (s_data.configs[i].config_desc) {
            struct usb_cfgdesc_s *desc = s_data.configs[i].config_desc;
            if (desc->cfgvalue == s_data.selected_config) {
                cdesc = &s_data.configs[i];
                break;
            }
        }
    }
    return cdesc;
}

static void select_interface(uint8_t iface, uint8_t alt_setting) {

    lldbg("iface=%d, alt=%d\n", iface, alt_setting);

    hcd_config_desc_t *cdesc = find_selected_config();

    if (!cdesc) {
        lldbg("Selected config not found\n");
        return;
    }

    cdesc->iface_settings[iface] = alt_setting;
}

static bool prepare_config(ep_map_t *ep_map, struct usb_epdesc_s *ep_list) {

    hcd_config_desc_t *cdesc = find_selected_config();

    if (!cdesc) {
        lldbg("Selected config not found\n");
        return false;
    }

    uint8_t *ptr = (uint8_t *)cdesc->config_desc;
    size_t total = 0;
    size_t ep_parse_count = 0;
    memcpy(ep_map, &cdesc->ep_map, sizeof(*ep_map));
    memset(ep_list, 0, sizeof(struct usb_epdesc_s) * MAX_ENDPOINTS);
    while (total < cdesc->config_size) {
        struct usb_desc_s *hdr = (struct usb_desc_s *)ptr;

        switch (hdr->type) {
        case USB_DESC_TYPE_CONFIG: {
            struct usb_cfgdesc_s *desc = (struct usb_cfgdesc_s *)ptr;
            lldbg("config desc: value=%d, num iface=%d, total=%d\n",
                  desc->cfgvalue, desc->ninterfaces, GETUINT16(desc->totallen));
            break;
        }
        case USB_DESC_TYPE_INTERFACE: {
            struct usb_ifdesc_s *desc = (struct usb_ifdesc_s *)ptr;
            lldbg("iface desc: num=%d, alt=%d num ep=%d\n", desc->ifno, desc->alt, desc->neps);
            /* Check to see if this is selected iface */
            if (cdesc->iface_settings[desc->ifno] == desc->alt) {
                lldbg("This is selectd interface\n");
                ep_parse_count = desc->neps;
            }
            break;
        }
        case USB_DESC_TYPE_ENDPOINT: {
            struct usb_epdesc_s *desc = (struct usb_epdesc_s *)ptr;
            lldbg("ep desc: addr=%02x, attr=%02x, pkt=%d, ival=%d\n",
                  desc->addr, desc->attr,
                  GETUINT16(desc->mxpacketsize), desc->interval);

            /* Only parse endpoint descriptors for selected interface */
            if (ep_parse_count) {
                uint8_t ep_num = desc->addr & USB_EPNO_MASK;
                ep_list[ep_num] = *desc;
                ep_parse_count--;
            }
            break;
        }
        default:
            break;
        }
        ptr += hdr->len;
        total += hdr->len;
    }

    lldbg("done parsing\n");
    return true;
}

static void config_endpoint(uint8_t mapped, uint8_t local, struct usb_epdesc_s *ep_desc) {
    int bufnum;
    void *hcpriv_ep = 0;

    /* queue urb if ep type is IN */
    if ((ep_desc->addr & 0x80) == USB_HOST_DIR_IN) {
        /* queue urb if ep type is IN */
        if ((ep_desc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT) {
            bufnum = NUM_INT_IN_REQS;
        } else {
            bufnum = NUM_IN_REQS;
        }

        int i;
        for (i = 0; i < bufnum; i++) {
            hcd_req_t *req = USBTUN_ALLOC(sizeof(*req));
            if (!req)
                continue;

            if (!init_ep_req(local, req, ep_desc)) {
                USBTUN_FREE(req);
                continue;
            }

            if (hcpriv_ep)
                req->urb.hcpriv_ep = hcpriv_ep;

            if (device_usb_hcd_urb_enqueue(s_data.dev, &req->urb)) {
                lldbg("Failed to queue urb for ep=%d\n", local);
                USBTUN_FREE(req);
                continue;
            }

            usbtun_req_to_usb(local, &req->entry);

            if (hcpriv_ep == 0) {
                hcpriv_ep = req->urb.hcpriv_ep;
            }
        }
        s_data.hcpriv_ep[mapped] = hcpriv_ep;
    } else {
        if ((ep_desc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT) {
            bufnum = NUM_INT_OUT_REQS;
        } else {
            bufnum = NUM_OUT_REQS;
        }
        int i;
        for (i = 0; i < bufnum; i++) {
            hcd_req_t *req = USBTUN_ALLOC(sizeof(*req));
            if (!req)
                continue;

            if (!init_ep_req(local, req, ep_desc)) {
                USBTUN_FREE(req);
                continue;
            }

            usbtun_req_q(local, &req->entry);
        }
    }
    lldbg("EP %d (mapped %d) configured\n", local, mapped);
}

static void unconfig_endpoint(uint8_t mapped, uint8_t local, struct usb_epdesc_s *ep_desc) {
    hcd_req_t *req;

    while ((req = (hcd_req_t *)usbtun_req_dq_usb(local)) != NULL) {
        device_usb_hcd_urb_dequeue(s_data.dev, &req->urb);
        usbtun_req_q(local, &req->entry);
    }
    while ((req = (hcd_req_t *)usbtun_req_dq(local)) != NULL) {
        clean_hcd_req(req);
        USBTUN_FREE(req);
    }
    s_data.hcpriv_ep[mapped] = NULL;
    memset(ep_desc, 0, sizeof(*ep_desc));
}

static void prepare_endpoints(ep_map_t *new_map, struct usb_epdesc_s *new_list) {
    int i;
    uint8_t local_ep;

    for (i = 0; i < MAX_ENDPOINTS; i++) {
        if (!s_data.ep_list[i].len) {
            if (!new_list[i].len)
                continue;

            local_ep = new_map->a2e[i];
            config_endpoint(i, local_ep, &new_list[i]);
        } else {
            if (new_list[i].len) {
                /* TODO: check if EP config is the same */
                continue;
            }
            local_ep = s_data.ep_map.a2e[i];
            unconfig_endpoint(i, local_ep, &s_data.ep_list[i]);
            continue;
        }
    }
    memcpy(&s_data.ep_map, new_map, sizeof(s_data.ep_map));
    memcpy(s_data.ep_list, new_list, sizeof(s_data.ep_list));
}

static void unprepare_endpint(void) {
    int i;
    uint8_t local_ep;

    for (i = 1; i < MAX_ENDPOINTS; i++) {
        if (s_data.ep_list[i].len) {
            local_ep = s_data.ep_map.a2e[i];
            unconfig_endpoint(i, local_ep, &s_data.ep_list[i]);
        }
    }
}

static void usb_control_transfer_complete(struct urb *urb) {

    hcd_req_t *req = CONTAINER_OF(urb, hcd_req_t, urb);
    bool config_prepared = false;
    struct usb_epdesc_s new_ep_list[MAX_ENDPOINTS];
    ep_map_t new_ep_map;

    struct usb_ctrlreq_s *ctrl = (struct usb_ctrlreq_s *)urb->setup_packet;
    uint16_t value = GETUINT16(ctrl->value);
    uint16_t index = GETUINT16(ctrl->index);
    uint8_t get_req = USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE;
    uint8_t std_dev_req = USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE;
    uint8_t std_iface_req = USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_INTERFACE;

#ifdef USBTUN_DEBUG
    if (urb->status) {
        lldbg("urb %p status %d, act_len = %d, type=%02x, req=%02x, value=%04x, index=%04x\n",
              urb, urb->status, urb->actual_length, ctrl->type, ctrl->req, value, index);
    }
#endif

    if (ctrl->type == get_req && ctrl->req == USB_REQ_GETDESCRIPTOR) {
        uint8_t desc_type = (value >> 8) & 0xff;
        uint8_t desc_idx = value & 0xff;
        if (desc_type == USB_DESC_TYPE_DEVICE) {
            /* figure out number of configs */
            prepare_config_list(urb->buffer, urb->actual_length);
        } else if (desc_type == USB_DESC_TYPE_CONFIG) {
            /* cache config data for later parsing */
            prepare_config_item(desc_idx, urb->buffer, urb->actual_length);
        }
    } else if (ctrl->type == std_iface_req && ctrl->req == USB_REQ_SETINTERFACE) {
            /* TODO: Need to evaluate config list again ? */
        select_interface(index, value);
        config_prepared = prepare_config(&new_ep_map, new_ep_list);
    } else if (ctrl->type == std_dev_req && ctrl->req == USB_REQ_SETCONFIGURATION) {
        if (s_data.do_tunnel) {
            lldbg("set config idx %d\n", value);
            s_data.selected_config = value;
            config_prepared = prepare_config(&new_ep_map, new_ep_list);
        }
    }

    if (req->tun) {
        /* In case of OUT request for EP0, response is already sent by PCD
         * as soon as data phase is complete. Do not send extra response.
         */
        if (USB_ISEPIN(ctrl->type) || (GETUINT16(ctrl->len) == 0)) {
            send_to_unipro(req, urb->actual_length, HCD_SETUP_RESP, urb->status);
        }
    }

    if (req->sync)
        sem_post(&urb->semaphore);

    clean_hcd_req(req);
    usbtun_req_from_usb(0, &req->entry);
    usbtun_req_q(0, &req->entry);

    if (config_prepared) {
        req = (hcd_req_t *)usbtun_req_dq(0);
        if (!req)
            return;

        void *ptr = bufram_alloc(sizeof(s_data.ep_list));
        if (ptr) {
            memcpy(ptr, new_ep_list, sizeof(s_data.ep_list));
            req->setup.type = USBTUN_MEM_NONE;
            req->data.type = USBTUN_MEM_BUFRAM;
            req->data.ptr = ptr;
            req->data.size = sizeof(s_data.ep_list);
            send_to_unipro(req, sizeof(s_data.ep_list), HCD_ENDPOINTS, 0);
            prepare_endpoints(&new_ep_map, new_ep_list);
        } else {
            lldbg("No men for ENDPOINT List\n");
        }
        clean_hcd_req(req);
        usbtun_req_q(0, &req->entry);
    }
}

static void prep_internal_setup(hcd_req_t *req, uint16_t typeReq, uint16_t wValue,
                                uint16_t wIndex, uint16_t wLength) {

    req->setup.type = USBTUN_MEM_BUFRAM;
    req->setup.ptr = bufram_alloc(SETUP_REQ_SIZE);
    req->setup.size = SETUP_REQ_SIZE;
    req->data.type = USBTUN_MEM_NONE;;
    req->data.ptr = NULL;
    req->data.size = 0;

    uint8_t *setup = req->setup.ptr;
    setup[0] = typeReq >> 8;
    setup[1] = typeReq & 0xff;
    setup[2] = wValue & 0xff;
    setup[3] = wValue >> 8;
    setup[4] = wIndex & 0xff;
    setup[5] = wIndex >> 8;
    setup[6] = wLength & 0xff;
    setup[7] = wLength >> 8;
}

static void prep_tun_setup(hcd_req_t *req, usbtun_buf_t *setup, void *dout, uint16_t dlen) {
    req->setup = *setup;
    if (dlen) {
        req->data.type = USBTUN_MEM_BUFRAM;
        req->data.ptr = bufram_alloc(dlen);
        req->data.size = dlen;
        if (dout) {
            memcpy(req->data.ptr, dout, dlen);
        }
    } else {
        req->data.type = USBTUN_MEM_NONE;;
        req->data.ptr = NULL;
        req->data.size = 0;
    }
}

static void prep_tun_data_out(hcd_req_t *req, usbtun_buf_t *data) {
    req->setup.type = USBTUN_MEM_NONE;
    req->setup.ptr = NULL;
    req->setup.size = 0;
    req->data = *data;
    req->urb.buffer = data->ptr;
    req->urb.length = data->size;
    req->urb.actual_length = 0;
}

static void init_ctrl_req(hcd_req_t *req)
{
    memset(req, 0, sizeof(*req));
    sem_init(&req->urb.semaphore, 0, 0);

    req->urb.complete = usb_control_transfer_complete;
    req->urb.pipe.type = USB_HOST_PIPE_CONTROL;
    req->urb.dev_speed = USB_SPEED_HIGH;
    req->urb.maxpacket = URB_MAX_PACKET;
}

static int usb_control_transfer(struct device *dev, hcd_req_t *req,
                                int direction, int addr, bool to_port) {
    if (!req) {
        lldbg("Null ctrl req\n");
        return -1;
    }

    req->urb.pipe.direction = direction;
    req->urb.pipe.device = addr;
    req->urb.buffer = req->data.ptr;
    req->urb.length = req->data.size;
    req->urb.setup_packet = req->setup.ptr;
    req->urb.actual_length = 0;
    req->urb.status = 0;

    if (to_port) {
        req->urb.devnum = USBHUB_ADDRESS;
        req->urb.dev_ttport = 1;
    }

    return device_usb_hcd_urb_enqueue(dev, &req->urb);
}

static int usb_control_transfer_sync(struct device *dev, hcd_req_t *req,
                                     int direction, int addr, bool to_port) {
    int ret;

    req->sync = true;
    ret = usb_control_transfer(dev, req, direction, addr, to_port);

    if (ret) {
        req->sync = false;
        return ret;
    }

    sem_wait(&req->urb.semaphore);
    req->sync = false;

    return req->urb.status;
}

static int usb_control_transfer_tun(struct device *dev, hcd_req_t *req,
                                    int direction, int addr) {
    int ret;

    req->tun = true;
    ret = usb_control_transfer_sync(dev, req, direction, addr, true);
    req->tun = false;

    return ret;
}

/* Called when IN endpoint have data to be consumed. */
static void usb_ep_in_transfer_complete(struct urb *urb) {
    hcd_req_t *req = CONTAINER_OF(urb, hcd_req_t , urb);
#ifdef USBTUN_DEBUG
    uint8_t ep = urb->pipe.endpoint;
    if (USBTUN_DEBUG_EP == ep) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        lldbg("[%u:%ld] IN EP %d comp. len=%d, urb=%p\n",
              ts.tv_sec, ts.tv_nsec / 1000000, ep, urb->actual_length, urb);
    }
#endif
    if (!s_data.do_tunnel) {
        /* USB device is not present. The request queue will be
         * cleaned up in unconfig_endpoint().*/
        return;
    }

    send_to_unipro(req, urb->actual_length, 0, urb->status);

    if (device_usb_hcd_urb_enqueue(s_data.dev, &req->urb)) {
        lldbg("Failed to enqueue urb for ep=%d\n", urb->pipe.endpoint);
        /* queue to free list so this item is cleaned up later */
        usbtun_req_from_usb(urb->pipe.endpoint, &req->entry);
        usbtun_req_q(urb->pipe.endpoint, &req->entry);
    }
}

/* Called after data is sent to OUT endpoint. */
static void usb_ep_out_transfer_complete(struct urb *urb) {
    uint8_t ep = urb->pipe.endpoint;
    hcd_req_t *req = CONTAINER_OF(urb, hcd_req_t , urb);
#ifdef USBTUN_DEBUG
    if (USBTUN_DEBUG_EP == ep) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        lldbg("[%u:%ld] OUT EP %d comp. len=%d, req=%p\n",
              ts.tv_sec, ts.tv_nsec / 1000000, ep, urb->actual_length, req);
    }
#endif
    clean_hcd_req(req);
    usbtun_req_from_usb(ep, &req->entry);
    usbtun_req_q(ep, &req->entry);

    sem_post(&urb->semaphore);
}

static bool init_ep_req(uint8_t local_ep, hcd_req_t *req, struct usb_epdesc_s *ep_info) {

    memset(req, 0, sizeof(*req));
    sem_init(&req->urb.semaphore, 0, 0);

    uint32_t buff_size = 0;
    void *buffer = NULL;
    if (get_hcd_ep_dir(ep_info->addr) == USB_HOST_DIR_IN) {
        buff_size = GETUINT16(ep_info->mxpacketsize);
        buffer = bufram_alloc(buff_size);
        if (!buffer) {
            return false;
        }
        req->urb.complete = usb_ep_in_transfer_complete;
    } else {
        req->urb.complete = usb_ep_out_transfer_complete;
    }
    req->data.type = USBTUN_MEM_BUFRAM;
    req->data.ptr = buffer;
    req->data.size = buff_size;

    /* ep_info is pointing APBA descriptor. Converto back to APBE eq number */
    req->urb.pipe.endpoint = local_ep;
    req->urb.pipe.type = get_hcd_ep_type(ep_info->attr);
    req->urb.pipe.device = 0;
    req->urb.pipe.direction = get_hcd_ep_dir(ep_info->addr);
    req->urb.dev_speed = USB_SPEED_HIGH;
    req->urb.maxpacket = GETUINT16(ep_info->mxpacketsize);
    req->urb.length = buff_size;
    req->urb.buffer = buffer;
    req->urb.setup_packet = NULL;
    req->urb.interval = get_hcd_ep_interval(ep_info->interval);
    req->urb.devnum = USBHUB_ADDRESS;
    req->urb.dev_ttport = 1;

    return true;
}

static port_status_t get_port_status(void) {
    port_status_t rstatus;

    memset(&rstatus, 0, sizeof(rstatus));

    /* Get port status */
    hcd_req_t *req = (hcd_req_t *)usbtun_req_dq(0);
    if (!req) {
        lldbg("no more req\n");
        return rstatus;
    }

    port_status_t *status = bufram_alloc(sizeof(port_status_t));
    if (!status) {
        lldbg("no more memory\n");
        return rstatus;
    }

    prep_internal_setup(req, GET_PORT_STATUS, 0, USBHUB_EXTERNAL_PORT, sizeof(port_status_t));
    req->data.type = USBTUN_MEM_NONE;
    req->data.ptr = status;
    req->data.size = sizeof(*status);
    int ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_IN, USBHUB_ADDRESS, false);
    if (ret) {
        lldbg("Failed to get port 1 status on 3813 hub.\n");
    } else {
        uint8_t *tmp = (uint8_t *)&status->port_status;
        rstatus.port_status = GETUINT16(tmp);
        tmp = (uint8_t *)&status->status_change;
        rstatus.status_change = GETUINT16(tmp);
        lldbg("Port Status: %04x, Port Changed: %04x\n",
              rstatus.port_status, rstatus.status_change);
    }
    bufram_free(status);

    return rstatus;
}

static void clear_feature(uint16_t feature) {
    /* Get port status */
    hcd_req_t *req = (hcd_req_t *)usbtun_req_dq(0);
    if (!req) {
        lldbg("no more req\n");
        return;
    }
    prep_internal_setup(req, CLR_PORT_FEATURE, feature, USBHUB_EXTERNAL_PORT, 0);
    int ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_OUT, USBHUB_ADDRESS, false);
    if (ret) {
        lldbg("Failed to clear port 1 feature on 3813 hub.\n");
    }
}

static void reset_port(void) {
    hcd_req_t *req = (hcd_req_t *)usbtun_req_dq(0);
    prep_internal_setup(req, SET_PORT_FEATURE, PORT_RESET, USBHUB_EXTERNAL_PORT, 0);
    int ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_OUT, USBHUB_ADDRESS, false);
    if (ret) {
        lldbg("Failed to reset port 1 on 3813 hub\n");
        return;
    }

    up_mdelay(PORT_RESETING_DELAY_IN_MS);
}

static void usb_hub_intr(struct urb *urb) {
    uint8_t status = 0;

    if (urb->status) {
        lldbg("urb %p status %d\n", urb, urb->status);
    } else {
        if (urb->buffer)
            status = *((uint8_t *)urb->buffer);

        lldbg("state change, val=%02x\n", status);
    }

    port_change = true;

    /* Notify HCD thread to handle state change.
     * Due to nuttx limitation, Mutex_lock cannot be held here.
     * Change interrupt will be kept asserted until changed port
     * feature is cleared so this code is assuming the change
     * will be eventually consumed by HCD thread.
     */
    pthread_cond_signal(&run_cond);
}

static void handle_port_change(hcd_req_t *req) {
    port_status_t status;
    bool reset_done = false;

    status = get_port_status();

    if (status.status_change & PORT_CHANGE_CONNECTION)
        clear_feature(C_PORT_CONNECTION);

    if (status.status_change & PORT_CHANGE_ENABLE)
        clear_feature(C_PORT_ENABLE);

    if (status.status_change & PORT_CHANGE_SUSPEND)
        clear_feature(C_PORT_SUSPEND);

    if (status.status_change & PORT_CHANGE_OVERCURRENT)
        clear_feature(C_PORT_OVERCURRENT);

    if (status.status_change & PORT_CHANGE_RESET) {
        reset_done = true;
        clear_feature(C_PORT_RESET);
    }

    if (status.port_status & PORT_CONNECTED_MASK) {
        if (!s_data.do_tunnel) {
            if (!reset_done) {
                lldbg("USB Device Connected - Retting port\n");
                reset_port();
            } else {
                lldbg("USB Device Connected\n");

                mhb_send_hsic_status_not(1);

                s_data.do_tunnel = true;
                int ret = unipro_send_tunnel_cmd(0, HCD_USB_CONNECTED, 0, NULL, 0);
                if (ret) {
                    lldbg("Failed to send ROUTER READY\n");
                } else {
                    lldbg("USB CONNECTED SENT\n");
                }
            }
        }
    } else {
        if (s_data.do_tunnel) {
            lldbg("USB Device Disconnected\n");

            mhb_send_hsic_status_not(0);

            s_data.do_tunnel = false;
            int ret = unipro_send_tunnel_cmd(0, HCD_USB_DISCONNECTED, 0, NULL, 0);
            if (ret) {
                lldbg("Failed to send ROUTER OFF\n");
            } else {
                lldbg("USB DISCONNECTED SENT\n");
            }
            unprepare_endpint();
        }
    }

    req->urb.status = 0;
    req->urb.actual_length = 0;
    if (device_usb_hcd_urb_enqueue(s_data.dev, &req->urb)) {
        lldbg("Failed to queue urb for hub int\n");
    }
}

static bool start_port_monitor(hcd_req_t *req) {

    memset(req, 0, sizeof(hcd_req_t));

    /* TODO: Currently this code is assuming USB3813 like hub (< 4 port hub).
     *       The logic needs to be refied to read out the config descripto
     *       of the hub in future.
     */
    void *buffer = bufram_alloc(1);
    if (!buffer)
        return false;

    req->urb.complete = usb_hub_intr;
    req->data.type = USBTUN_MEM_BUFRAM;
    req->data.ptr = buffer;
    req->data.size = 1;

    /* ep_info is pointing APBA descriptor. Converto back to APBE eq number */
    req->urb.pipe.endpoint = 1;
    req->urb.pipe.type = USB_HOST_PIPE_INTERRUPT;
    req->urb.pipe.device = USBHUB_ADDRESS;
    req->urb.pipe.direction = USB_HOST_DIR_IN;
    req->urb.dev_speed = USB_SPEED_HIGH;
    req->urb.maxpacket = 1;
    req->urb.length = 1;
    req->urb.buffer = buffer;
    req->urb.setup_packet = NULL;
    req->urb.interval = get_hcd_ep_interval(0x0c);

    lldbg("starting port monitor\n");
    if (device_usb_hcd_urb_enqueue(s_data.dev, &req->urb)) {
        lldbg("Failed to start port monitor\n");
    }

    return true;
}

static void stop_port_monitor(hcd_req_t *req) {
    device_usb_hcd_urb_dequeue(s_data.dev, &req->urb);
    lldbg("stopped port monitor\n");
}

static void *hcd_router_startup(void *arg) {
    int i;
    hcd_req_t *req;
    hcd_req_t *hub_req = NULL;;

    /* TODO: temporary memory debug - to be removed */
    usbtun_print_mem_info();

    int ret;
    uint32_t *status = bufram_alloc(sizeof(uint32_t));

    if (!status) {
        lldbg("Failed to allocate status memory\n");
        return NULL;
    }

    memset(&s_data, 0, sizeof(s_data));

    /* init pooled resources */
    if (init_router_common(&handle_hdr, &handle_data_body))
        goto bufram_clean;

    /* pre-allocate ep_urb for control endpoint */
    for (i = 0; i < NUM_CTRL_REQS; i++) {
        req = USBTUN_ALLOC(sizeof(*req));
        if (req) {
            init_ctrl_req(req);
            usbtun_req_q(0, &req->entry);
        }
    }

    s_data.dev = device_open(DEVICE_TYPE_USB_HCD, 0);
    if (!s_data.dev) {
        lldbg("Failed to open HCD device\n");
        goto errout;
    }

    ret = device_usb_hcd_start(s_data.dev);
    if (ret) {
        lldbg("Failed to start up HCD device\n");
        goto errout;
    }
    lldbg("HCD device started\n");

    /*
     * Enable port 1 on root hub
     */
    device_usb_hcd_hub_control(s_data.dev, SET_PORT_FEATURE, PORT_POWER, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_POWERING_DELAY_IN_MS);

    device_usb_hcd_hub_control(s_data.dev, SET_PORT_FEATURE, PORT_RESET, ROOT_HUB_PORT,
                               NULL, 0);
    up_mdelay(PORT_RESETING_DELAY_IN_MS);

    /* Get port status of Port 1. Expecting a 4 bytes response. */
    device_usb_hcd_hub_control(s_data.dev, GET_PORT_STATUS, 0, 1, (char *)status, sizeof(*status));

    lldbg("Status on root hub port %08x\n", *status);
    lldbg("HCD Router started\n");

    /* Configure usb3813 hub */
    req = (hcd_req_t *)usbtun_req_dq(0);
    prep_internal_setup(req, SET_ADDRESS, USBHUB_ADDRESS, 0, 0);
    ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_OUT, 0, false);
    if (ret) {
        lldbg("Failed to set address to 3813 hub\n");
        goto stop_hcd;
    }

    req = (hcd_req_t *)usbtun_req_dq(0);
    prep_internal_setup(req, SET_CONFIGURATION, USBHUB_CONFIG_ID, 0, 0);
    ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_OUT, USBHUB_ADDRESS, false);
    if (ret) {
        lldbg("Failed to set configuration to 3813 hub\n");
        goto stop_hcd;
    }

    while(!s_data.pcd_ready) {
        if (!do_run)
            goto stop_hcd;

        struct timespec expires;
        clock_gettime(CLOCK_REALTIME, &expires);
        uint64_t new_ns = timespec_to_nsec(&expires);
        new_ns += PCD_HANDSHAKE_WAIT_NS;
        nsec_to_timespec(new_ns, &expires);

        pthread_mutex_lock(&run_lock);
        pthread_cond_timedwait(&run_cond, &run_lock, &expires);
        pthread_mutex_unlock(&run_lock);
    }
    lldbg("PCD router ready\n");

    ret = unipro_send_tunnel_cmd(0, HCD_ROUTER_READY, 0, NULL, 0);
    if (ret) {
        lldbg("Failed to send ROUTER READY\n");
        goto stop_hcd;
    } else {
        lldbg("HCD ROUTER READY SENT\n");
    }

    /* Enable port 1 on usb3813 hub */
    req = (hcd_req_t *)usbtun_req_dq(0);
    prep_internal_setup(req, SET_PORT_FEATURE, PORT_POWER, USBHUB_EXTERNAL_PORT, 0);
    ret = usb_control_transfer_sync(s_data.dev, req, USB_HOST_DIR_OUT, USBHUB_ADDRESS, false);
    if (ret) {
        lldbg("Failed to enable port 1 on 3813 hub\n");
        goto stop_hcd;
    }

    /* start monitor port state change */
    hub_req = USBTUN_ALLOC(sizeof(hcd_req_t));
    if (!hub_req) {
        lldbg("Failed to allocate hcd req\n");
        goto stop_hcd;
    }

    if (start_port_monitor(hub_req) == false) {
        lldbg("Failed to start port monitor\n");
        goto stop_hcd;
    }

    while (true) {
        if (port_change) {
            handle_port_change(hub_req);
            port_change = false;
        }
        pthread_mutex_lock(&run_lock);
        if (!do_run) {
            pthread_mutex_unlock(&run_lock);
            break;
        }
        /* Wait until uninit request is received */
        pthread_cond_wait(&run_cond, &run_lock);
        pthread_mutex_unlock(&run_lock);
    }

    stop_port_monitor(hub_req);

stop_hcd:
    s_data.do_tunnel = false;
    lldbg("Stopping HCD router\n");

    if (hub_req)
        clean_hcd_req(hub_req);

errout:
    /* clean up non-ctrl endpoints */
    unprepare_endpint();

    /* clean up ctrl */
    while ((req = (hcd_req_t *)usbtun_req_dq_usb(0)) != NULL) {
        device_usb_hcd_urb_dequeue(s_data.dev, &req->urb);
        usbtun_req_q(0, &req->entry);
    }
    while ((req = (hcd_req_t *)usbtun_req_dq(0)) != NULL) {
        clean_hcd_req(req);
        USBTUN_FREE(req);
    }

    device_usb_hcd_stop(s_data.dev);

    /* init pooled resources */
    uninit_router_common();

    device_close(s_data.dev);
    s_data.dev = NULL;

bufram_clean:
    bufram_free(status);

    lldbg("HCD router Stopped\n");

    /* TODO: temporary memory debug - to be removed */
    usbtun_print_mem_info();

    return NULL;
}

int usbtun_hcd_router_init(void) {
    pthread_mutex_lock(&run_lock);
    if (do_run) {
        /* already running. Just return */
        pthread_mutex_unlock(&run_lock);
        return 0;
    }
    do_run = true;
    pthread_mutex_unlock(&run_lock);

    int ret =  pthread_create(&hcd_thread, NULL, hcd_router_startup, NULL);
    if (ret) {
        lldbg("HCD router init failed\n");
        return ret;
    }

    return 0;
}

void usbtun_hcd_router_uninit(void) {
    pthread_mutex_lock(&run_lock);
    if (!do_run) {
        /* Not running. Just return */
        pthread_mutex_unlock(&run_lock);
        return;
    }
    do_run = false;
    pthread_cond_signal(&run_cond);
    pthread_mutex_unlock(&run_lock);

    pthread_join(hcd_thread, NULL);

    return;
}
