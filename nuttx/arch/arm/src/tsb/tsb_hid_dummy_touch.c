/*
 * Copyright (c) 2015 Google, Inc.
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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/gpio.h>
#include <nuttx/clock.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>
#include "clock/clock.h"

#define HID_DEVICE_FLAG_PROBE           BIT(0)  /* device probed */
#define HID_DEVICE_FLAG_OPEN            BIT(1)  /* device opened */
#define HID_DEVICE_FLAG_POWERON         BIT(2)  /* device powered on */

#define TIP_SWITCH                      BIT(0)  /* finger touched */
#define IN_RANGE                        BIT(1)  /* touch is valid */

#define VENDORID                        0xABCD
#define PRODUCTID                       0x1234

#define REPORT_ID_ZERO                  0   /* Report ID zero is reserved and
                                             * should not be used. */
#define REPORT_ID_MULTITOUCH            1   /* Report ID: Multitouch */
#define REPORT_ID_MAX_CONTACTS          2   /* Report ID: Max fingers config */

#define MAX_VALID_CONTACTS              3   /* max fingers count */
#define TOUCH_DEV_WIDTH                 3840 /* default touch panel width */
#define TOUCH_DEV_HEIGHT                2400 /* default touch panel height */

#define TESTCASE_ONE_FINGER_CLICK       0   /* test case 0: 1-finger click */
#define TESTCASE_ONE_FINGER_SWIPE       1   /* test case 1: 1-finger swipe */
#define TESTCASE_TWO_FINGER_SWIPE       2   /* test case 2: 2-finger swipe */
#define TESTCASE_THREE_FINGER_SWIPE     3   /* test case 3: 3-finger swipe */
#define NUM_OF_TESTCASE                 4

#define MIN_MT_COUNT                    10
#define MAX_MT_COUNT                    100
#define MT_MOVE_STEP                    2
#define CLICK_PRESSURE                  20
#define SWIPE_PRESSURE                  30

#define GPIO_TRIGGER                    0   /* Trigger GPIO pin for testing */
#define DEFAULT_DEBOUNCE_TIME           25  /* 250ms (1 SysTick = 10ms) */
#define TOUCH_SAMPLE_RATE               20000 /* 20ms */

static int tsb_hid_get_report_length(struct device *dev, uint8_t report_type,
                                     uint8_t report_id);

static struct device *hid_dev = NULL;

/**
 * HID report length structure
 */
struct report_len {
    /** input report length */
    uint16_t input;
    /** output report length */
    uint16_t output;
    /** feature report length */
    uint16_t feature;
} __packed;

/**
 * HID Report Size Structure. Type define for a report item size
 * information structure, to retain the size of a device's reports by ID.
 */
struct hid_size_info {
    /** Report ID */
    uint8_t id;
    /** HID Report length array */
    union {
        uint16_t size[3];
        struct report_len len;
    } reports;
};

/**
 * Touch finger data
 */
struct finger {
    /** the finger status:  BIT(0): Tip Switch,  BIT(1): In Range */
    uint8_t state;
    /** contact id */
    uint8_t cid;
    /** x coordinate */
    uint16_t x;
    /** y coordinate */
    uint16_t y;
    /** pressure of the touch contact */
    uint16_t pressure;
} __packed;

/**
 * Touch report data
 */
struct hid_touch_data {
    /** report id */
    uint8_t report_id;
    /** touch finger data array */
    struct finger points[MAX_VALID_CONTACTS];
    /** actual finger count */
    uint8_t actual;
} __packed;

/**
 * wait queue
 */
struct hid_waitq {
    /** thread exit flag */
    int abort;
    /** condition object for wait event */
    pthread_cond_t cond;
    /** mutex object for wait event */
    pthread_mutex_t mutex;
};

/**
 * Private HID device information
 */
struct tsb_hid_info {
    /** Driver model representation of the device */
    struct device *dev;

    /** HID device descriptor */
    struct hid_descriptor *hdesc;
    /** HID report descriptor */
    uint8_t *rdesc;
    /** number of HID Report structure */
    int num_ids;
    /** report length of each HID Reports */
    struct hid_size_info *sinfo;

    /** number of fingers supported */
    uint8_t maximum_contacts;
    /** HID device state*/
    int state;
    /** hid input event callback function */
    hid_event_callback event_callback;
    /** Exclusive access for operation */
    sem_t lock;

    /** store latest valid keyboard interrupt time */
    uint32_t last_activetime;
    /** store latest valid keyboard state */
    uint8_t last_gpiostate;
    /**
     * GPIO debounce time count
     *
     * Since current SysTick resolution is 10 milliseconds, so the
     * debounce_time unit is 10 milliseconds.
     */
    int debounce_time;

    /** thread handle */
    pthread_t hid_thread;
    /** wait queue for gpio trigger event*/
    struct hid_waitq wq;

    /** multitouch data. for testing */
    struct hid_touch_data data;
    /** start to generate multitouch data. for testing */
    int mt_generate;
    /** multitouch data count. for testing */
    int mt_count;
};

#define LOGICAL_FINGER_COLLECTION \
    0xa1, 0x02,                    /*   COLLECTION (Logical) */\
    0x05, 0x0d,                    /*     USAGE_PAGE (Digitizers) */\
    0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */\
    0x25, 0x01,                    /*     LOGICAL_MAXIMUM (1) */\
    0x75, 0x01,                    /*     REPORT_SIZE (1) */\
    0x95, 0x02,                    /*     REPORT_COUNT (2) */\
    0x09, 0x42,                    /*     USAGE (Tip Switch) */\
    0x09, 0x32,                    /*     USAGE (In Range) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x95, 0x06,                    /*     REPORT_COUNT (6) */\
    0x81, 0x01,                    /*     INPUT (Cnst,Ary,Abs) */\
    0x75, 0x08,                    /*     REPORT_SIZE (8) */\
    0x09, 0x51,                    /*     USAGE (Contact identifier) */\
    0x95, 0x01,                    /*     REPORT_COUNT (1) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x05, 0x01,                    /*     USAGE_PAGE (Generic Desktop) */\
    0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */\
    0x75, 0x10,                    /*     REPORT_SIZE (16) */\
    0x09, 0x30,                    /*     USAGE (X) */\
    0x26, 0x00, 0x0f,              /*     LOGICAL_MAXIMUM (3840) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x26, 0x60, 0x09,              /*     LOGICAL_MAXIMUM (2400) */\
    0x09, 0x31,                    /*     USAGE (Y) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x05, 0x0d,                    /*     USAGE_PAGE (Digitizers) */\
    0x09, 0x30,                    /*     USAGE (Tip Pressure) */\
    0x26, 0xFF, 0x00,              /*     LOGICAL_MAXIMUM (255) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0xc0                           /*   END_COLLECTION */

/**
 * Multitouch HID report descriptor
 */
uint8_t hid_report_desc[] = {
    0x05, 0x0d,                    /* USAGE_PAGE (Digitizers) */
    0x09, 0x04,                    /* USAGE (Touch Screen) */
    0xa1, 0x01,                    /* COLLECTION (Application) */
    0x85, REPORT_ID_MULTITOUCH,    /*   REPORT_ID (multitouch) */
    0x09, 0x22,                    /*   USAGE (Finger) */
    LOGICAL_FINGER_COLLECTION,     /*   1st finger */
    LOGICAL_FINGER_COLLECTION,     /*   2nd finger */
    LOGICAL_FINGER_COLLECTION,     /*   3rd finger */
    0x05, 0x0d,                    /*   USAGE_PAGE (Digitizers) */
    0x09, 0x54,                    /*   USAGE (Contact count) */
    0x95, 0x01,                    /*   REPORT_COUNT (1) */
    0x75, 0x08,                    /*   REPORT_SIZE (8) */
    0x25, MAX_VALID_CONTACTS,      /*   LOGICAL_MAXIMUM (MAX_VALID_CONTACTS) */
    0x81, 0x02,                    /*   INPUT (Data,Var,Abs) */
    0x85, REPORT_ID_MAX_CONTACTS,  /*   REPORT_ID (max_contacts) */
    0x09, 0x55,                    /*   USAGE (Contact count maximum) */
    0xb1, 0x02,                    /*   FEATURE (Data,Var,Abs) */
    0xc0                           /* END_COLLECTION */
};

/**
 * Keyboard HID Device Descriptor
 */
struct hid_descriptor hid_dev_desc = {
    sizeof(struct hid_descriptor), /* hid_descriptor size */
    sizeof(hid_report_desc), /* hid_report_desc size */
    0x0111, /* HID v1.11 compliant */
    PRODUCTID,
    VENDORID,
    0x00, /* no country code */
};

/**
 * report length of each HID Reports in HID Report Descriptor
 *
 * all value is parsed from HID Report Descriptor manually
 */
struct hid_size_info hid_sizeinfo[] =
{
    { .id = REPORT_ID_MULTITOUCH,
      .reports = {
          .size = { 25, 0, 0 }
      }
    },
    { .id = REPORT_ID_MAX_CONTACTS,
      .reports = {
          .size = { 0, 0, 1 }
      }
    },
};

/**
 * @brief generate fake test data for multitouch
 *
 * @param info - pointer to structure of HID private data
 * @param testcase - type of multitouch test case
 */
int update_touch_data(struct tsb_hid_info *info, uint8_t testcase)
{
    int i = 0, ret = 0;

    if (!info) {
        return -EINVAL;
    }

    if (!info->mt_generate) {
        /* start to generate fake multitouch data */

        /* reset, set all points to null-values */
        memset(&info->data, 0, sizeof(struct hid_touch_data));

        /* first touch packet */
        info->mt_generate = 1;
        info->data.report_id = REPORT_ID_MULTITOUCH;
        switch (testcase) {
            case TESTCASE_ONE_FINGER_CLICK:
                info->data.actual = 1;
                info->data.points[0].state = TIP_SWITCH | IN_RANGE;
                info->data.points[0].cid = 1;
                info->data.points[0].x = rand() % TOUCH_DEV_WIDTH;
                info->data.points[0].y = rand() % TOUCH_DEV_HEIGHT;
                info->data.points[0].pressure = CLICK_PRESSURE;
                info->mt_count = 1;
            break;
            case TESTCASE_ONE_FINGER_SWIPE:
            case TESTCASE_TWO_FINGER_SWIPE:
            case TESTCASE_THREE_FINGER_SWIPE:
                info->data.actual = testcase;
                if (info->data.actual > info->maximum_contacts) {
                    info->data.actual = info->maximum_contacts;
                }
                for (i = 0; i < info->data.actual; i++) {
                    info->data.points[i].state = TIP_SWITCH | IN_RANGE;
                    info->data.points[i].cid = i + 1;
                    /* x = 0 ~ (TOUCH_DEV_WIDTH - 1) */
                    info->data.points[i].x = rand() % TOUCH_DEV_WIDTH;
                    /* y = 0 ~ (TOUCH_DEV_HEIGHT -
                     *         (MAX_MT_COUNT * MT_MOVE_STEP) - 1) */
                    info->data.points[i].y = rand() % (TOUCH_DEV_HEIGHT -
                                             (MAX_MT_COUNT * MT_MOVE_STEP));
                    info->data.points[i].pressure = SWIPE_PRESSURE;
                }
                /* mt_count = MIN_MT_COUNT ~ MAX_MT_COUNT */
                info->mt_count = (rand() % (MAX_MT_COUNT - MIN_MT_COUNT + 1))
                                   + MIN_MT_COUNT;
            break;
            default:
                info->mt_generate = 0;
                ret = -EINVAL;
            break;
        }
    } else {
        /* next touch packet */
        info->mt_count--;
        for (i = 0; i < info->data.actual; i++) {
            if (info->mt_count) {
                info->data.points[i].state = TIP_SWITCH | IN_RANGE;
                info->data.points[i].y += MT_MOVE_STEP;
            } else {
                /* stop to generate touch packet */
                info->data.points[i].state &= ~TIP_SWITCH;
                info->mt_generate = 0;
            }
        }
    }
    return ret;
}

/**
 * @brief HID demo thread function
 *
 * @param context - pointer to structure of device data
 */
void tsb_hid_thread_func(void *context)
{
    struct device *dev = context;
    struct tsb_hid_info *info = NULL;
    uint8_t testcase = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    pthread_mutex_lock(&info->wq.mutex);

    /* Initialize random number generator for Multitouch HID demo case */
    srand(time(NULL));

    while (1) {
        /* wait for gpio trigger event */
        pthread_cond_wait(&info->wq.cond, &info->wq.mutex);
        if (info->wq.abort) {
            /* exit tsb_hid_thread_func loop */
            break;
        }

        /* send demo touch packet to host */
        do {
            /* prepare touch packet */
            update_touch_data(info, testcase);

            if (info->event_callback) {
                /* send touch packet to the caller */
                info->event_callback(dev, HID_INPUT_REPORT,
                                     (uint8_t*)&info->data,
                                     sizeof(struct hid_touch_data));
            }
            usleep(TOUCH_SAMPLE_RATE);
        } while (info->mt_count > 0);

        /* switch to next testcase */
        testcase = (testcase + 1) % NUM_OF_TESTCASE;
    }
    pthread_mutex_unlock(&info->wq.mutex);
}

/**
 * @brief Get HID Input report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int get_input_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (report_id != REPORT_ID_MULTITOUCH) {
        /* For current case, only supports REPORT_ID_MULTITOUCH input report,
         * if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }
    if (len < sizeof(struct hid_touch_data)) {
        /* no enough buffer space to receive touch data */
        return -ENOBUFS;
    }
    memcpy(data, &info->data, sizeof(struct hid_touch_data));
    return 0;
}

/**
 * @brief Get HID Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int get_feature_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int rptlen = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (report_id != REPORT_ID_MAX_CONTACTS) {
        /* For current case, only supports REPORT_ID_MAX_CONTACTS feature
         * report, if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }

    rptlen = tsb_hid_get_report_length(dev, HID_FEATURE_REPORT, report_id);
    if (report_id != REPORT_ID_ZERO) {
        /* if report id isn't zero, return data need to contain 1-Byte report
         * id value */
        rptlen++;
    }
    if (len < rptlen) {
        /* no enough buffer space to receive feature data */
        return -ENOBUFS;
    }

    /* Feature report format (2Bytes): [Report ID][Max Contacts] */
    data[0] = report_id;
    data[1] = info->maximum_contacts;
    return 0;
}

/**
 * @brief Set HID Output report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int set_output_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    /* current report descriptor doesn't contain a output report, so just
     * returns an error code to caller. */
    return -EIO;
}

/**
 * @brief Set HID Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int set_feature_report(struct device *dev, uint8_t report_id,
                            uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int rptlen = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (report_id != REPORT_ID_MAX_CONTACTS) {
        /* For current case, only supports REPORT_ID_MAX_CONTACTS feature
         * report, if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }

    rptlen = tsb_hid_get_report_length(dev, HID_FEATURE_REPORT, report_id);

    if (len < rptlen) {
        /* no enough buffer space to receive feature data */
        return -ENOBUFS;
    }

    if (data[0] > MAX_VALID_CONTACTS) {
        /* the new value can't larger than maximum contacts which defined in
         * HID report descriptor */
        return -EIO;
    }

    /* Because Greybus HID protocol passed the feature data without report id,
     * so the first byte is maximum contacts field. */
    info->maximum_contacts = data[0];
    return 0;
}

/**
 * @brief HID device interrupt routine
 *
 * @param context - pointer to structure of device data
 */
int tsb_hid_irq_event(int irq, FAR void *context)
{
    struct device *dev = hid_dev;
    struct tsb_hid_info *info = NULL;
    uint8_t new_gpiostate = 0;
    int elapsed = 0;

    if (!dev || !device_get_private(dev)) {
        return ERROR;
    }

    info = device_get_private(hid_dev);

    elapsed = clock_systimer() - info->last_activetime;

    if (elapsed >= info->debounce_time) {
        gpio_mask_irq(irq);
        new_gpiostate = gpio_get_value(GPIO_TRIGGER);

        /* check whether the key state change or not */
        if (info->last_gpiostate != new_gpiostate) {
            info->last_gpiostate = new_gpiostate;
            info->last_activetime = clock_systimer();

            /* notify thread function to send demo report data*/
            if (new_gpiostate) {
                pthread_cond_signal(&info->wq.cond);
            }
        }
        gpio_unmask_irq(irq);
    }
    return OK;
}

/**
 * @brief Power-on the HID device.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_power_on(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweron;
    }
    if (!(info->state & HID_DEVICE_FLAG_POWERON)) {
        info->state |= HID_DEVICE_FLAG_POWERON;
        /* enable interrupt */
        gpio_unmask_irq(GPIO_TRIGGER);
    } else {
        ret = -EBUSY;
    }
err_poweron:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Power-off the HID device.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_power_off(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sem_wait(&info->lock);
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweroff;
    }
    if (info->state & HID_DEVICE_FLAG_POWERON) {
        /* changed power-on state */
        info->state &= ~HID_DEVICE_FLAG_POWERON;
        /* disable interrupt */
        gpio_mask_irq(GPIO_TRIGGER);
    } else {
        ret = -EIO;
    }
err_poweroff:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Descriptor
 *
 * @param dev - pointer to structure of device data
 * @param desc - pointer to structure of HID device descriptor
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_get_desc(struct device *dev, struct hid_descriptor *desc)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !desc) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    /* get HID device descriptor */
    memcpy(desc, info->hdesc, sizeof(struct hid_descriptor));
    return 0;
}

/**
 * @brief Get HID Report Descriptor
 *
 * @param dev - pointer to structure of device data
 * @param desc - pointer to HID report descriptor
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_get_report_desc(struct device *dev, uint8_t *desc)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !desc) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* get HID report descriptor */
    memcpy(desc, info->rdesc, info->hdesc->report_desc_length);
    return 0;
}

/**
 * @brief Get HID report length
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @return the report size on success, negative errno on error
 */
static int tsb_hid_get_report_length(struct device *dev, uint8_t report_type,
                                     uint8_t report_id)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0, i;

    /* check input parameters */
    if (!dev || !device_get_private(dev) ||
        (report_type > HID_FEATURE_REPORT)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* lookup the hid_size_info table to find the report size */
    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].id == report_id) {
            ret = info->sinfo[i].reports.size[report_type];
            break;
        }
    }
    return ret;
}

/**
 * @brief Get HID maximum report size in all Report ID for each Report type
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @return the report size on success, negative errno on error
 */
static int tsb_hid_get_maximum_report_length(struct device *dev,
                                             uint8_t report_type)
{
    struct tsb_hid_info *info = NULL;
    int i = 0, maxlen = 0, id = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) ||
        (report_type > HID_FEATURE_REPORT)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    /* lookup the hid_size_info table to find the max report size
     * in specific Report type  */

    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].reports.size[report_type] > maxlen) {
            id = info->sinfo[i].id;
            maxlen = info->sinfo[i].reports.size[report_type];
        }
    }
    /* If the Report ID isn't zero, add a extra 1-byte space to save
     * the Report ID.*/
    if (id) {
        maxlen++;
    }
    return maxlen;
}

/**
 * @brief Get HID Input / Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_get_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !data || !len) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_getreport;
    }

    switch (report_type) {
        case HID_INPUT_REPORT:
            ret = get_input_report(dev, report_id, data, len);
        break;
        case HID_FEATURE_REPORT:
            ret = get_feature_report(dev, report_id, data, len);
        break;
        default:
            /* only support input and feature report */
            ret = -EINVAL;
        break;
    }
err_getreport:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Set HID Output / Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_set_report(struct device *dev, uint8_t report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !data || !len) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);
    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_setreport;
    }

    switch (report_type) {
        case HID_OUTPUT_REPORT:
            ret = set_output_report(dev, report_id, data, len);
        break;
        case HID_FEATURE_REPORT:
            ret = set_feature_report(dev, report_id, data, len);
        break;
        default:
            /* only support output and feature report */
            ret = -EINVAL;
        break;
    }
err_setreport:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Register HID Report notify event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify event
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_register_callback(struct device *dev,
                                     hid_event_callback callback)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    info->event_callback = callback;
    return 0;
}

/**
 * @brief Remove HID Report notify event
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_unregister_callback(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }
    info->event_callback = NULL;
    return 0;
}

/**
 * @brief Open HID device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_dev_open(struct device *dev)
{
    struct tsb_hid_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_PROBE)) {
        ret = -EIO;
        goto err_open;
    }

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        /* device has been opened, return error */
        ret = -EBUSY;
        goto err_open;
    }

    /* calculate report descriptor length dynamically */
    hid_dev_desc.report_desc_length = sizeof(hid_report_desc);

    info->hdesc = &hid_dev_desc;
    info->rdesc = hid_report_desc;
    info->sinfo = hid_sizeinfo;
    info->num_ids = ARRAY_SIZE(hid_sizeinfo);

    info->event_callback = NULL;
    info->state |= HID_DEVICE_FLAG_OPEN;

    /* set device default value */
    info->last_activetime = 0;
    info->last_gpiostate = 0xFF;
    info->debounce_time = DEFAULT_DEBOUNCE_TIME;
    info->maximum_contacts = MAX_VALID_CONTACTS;

    /* initialize testcase variable */
    memset(&info->data, 0, sizeof(struct hid_touch_data));
    info->mt_count = 0;
    info->mt_generate = 0;
    info->data.report_id = REPORT_ID_MULTITOUCH;

    /* initialize GPIO pin */
    if (GPIO_TRIGGER >= gpio_line_count()) {
        info->state &= ~HID_DEVICE_FLAG_OPEN;
        ret = -EIO;
        goto err_open;
    }
    gpio_activate(GPIO_TRIGGER);
    gpio_direction_in(GPIO_TRIGGER);
    gpio_mask_irq(GPIO_TRIGGER);
    set_gpio_triggering(GPIO_TRIGGER, IRQ_TYPE_EDGE_BOTH);
    gpio_irqattach(GPIO_TRIGGER, tsb_hid_irq_event);

    /* initialize waitqueue */
    info->wq.abort = 0;
    pthread_mutex_init(&info->wq.mutex, NULL);
    pthread_cond_init(&info->wq.cond, NULL);

    /* create thread to send demo report data */
    if (pthread_create(&info->hid_thread, NULL, (void*)tsb_hid_thread_func,
                       (void*)dev) != 0) {
        ret = -EIO;
    }
err_open:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close HID device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev - pointer to structure of device data
 */
static void tsb_hid_dev_close(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        goto err_close;
    }

    if (info->state & HID_DEVICE_FLAG_POWERON) {
        tsb_hid_power_off(dev);
    }

    /* uninitialize GPIO pin */
    gpio_unmask_irq(GPIO_TRIGGER);
    gpio_deactivate(GPIO_TRIGGER);

    if (info->hid_thread != (pthread_t)0) {
        info->wq.abort = 1;
        pthread_cond_signal(&info->wq.cond);
        /* wait for thread completed */
        pthread_join(info->hid_thread, NULL);
    }

    pthread_cond_destroy(&info->wq.cond);
    pthread_mutex_destroy(&info->wq.mutex);

    /* clear open state */
    info->state &= ~HID_DEVICE_FLAG_OPEN;

    info->hdesc = NULL;
    info->rdesc = NULL;
    info->sinfo = NULL;
    info->num_ids = 0;

    info->last_activetime = 0;
    info->last_gpiostate = 0xFF;
    info->debounce_time = 0;
    info->maximum_contacts = 0;

    info->event_callback = NULL;

err_close:
    sem_post(&info->lock);
}

/**
 * @brief Probe HID device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private HID device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_hid_dev_probe(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    hid_dev = dev;
    info->dev = dev;
    info->state = HID_DEVICE_FLAG_PROBE;
    device_set_private(dev, info);

    sem_init(&info->lock, 0, 1);
    return 0;
}

/**
 * @brief Remove HID device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev - pointer to structure of device data
 */
static void tsb_hid_dev_remove(struct device *dev)
{
    struct tsb_hid_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        tsb_hid_dev_close(dev);
    }
    info->state = 0;
    sem_destroy(&info->lock);

    device_set_private(dev, NULL);
    hid_dev = NULL;
    free(info);
}

static struct device_hid_type_ops tsb_hid_type_ops = {
    .power_on = tsb_hid_power_on,
    .power_off = tsb_hid_power_off,
    .get_descriptor = tsb_hid_get_desc,
    .get_report_descriptor = tsb_hid_get_report_desc,
    .get_report_length = tsb_hid_get_report_length,
    .get_maximum_report_length = tsb_hid_get_maximum_report_length,
    .get_report = tsb_hid_get_report,
    .set_report = tsb_hid_set_report,
    .register_callback = tsb_hid_register_callback,
    .unregister_callback = tsb_hid_unregister_callback,
};

static struct device_driver_ops tsb_hid_driver_ops = {
    .probe          = tsb_hid_dev_probe,
    .remove         = tsb_hid_dev_remove,
    .open           = tsb_hid_dev_open,
    .close          = tsb_hid_dev_close,
    .type_ops       = &tsb_hid_type_ops,
};

struct device_driver tsb_hid_driver = {
    .type       = DEVICE_TYPE_HID_HW,
    .name       = "tsb_hid",
    .desc       = "TSB HID Driver",
    .ops        = &tsb_hid_driver_ops,
};
