/*
 * Copyright (c) 2015 Motorola Mobility, LLC.
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

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/camera/camera_ext.h>

struct camera_ext_db {
    const struct camera_ext_format_db *format_db;
    struct camera_ext_format_user_config user_cfg;
    struct camera_ext_ctrl_db *ctrl_db;
};

static uint16_t s_phone_ver; /* major << 8 | minor */

static size_t get_local_ctrl_val_size(
        const struct camera_ext_ctrl_val_cfg *val_cfg);

/* packet control value to greybus payload.
 * val_cfg - control value config
 * local_val - control value
 * gb_val - greybus payload
 * gb_val_size - greybus payload size
 * is_cfg_pkt - whether pack value to the config
 */
static int ctrl_val_local_to_greybus(
        const struct camera_ext_ctrl_val_cfg *val_cfg,
        const camera_ext_ctrl_val_t *local_val, uint8_t *gb_val,
        const uint32_t gb_val_size, int is_cfg_pkt);

/*
 * TODO: Need to support multiple instances of format_cfg below if more than
 *       one camera_ext protocol is configured in a single manifest.
*/
static struct camera_ext_db g_camera_ext_db;

camera_ext_event_cb_t s_event_cb;
static struct device *s_dev;

int camera_ext_register_event_cb(struct device *dev, camera_ext_event_cb_t cb)
{
    s_event_cb = cb;
    s_dev = dev;

    return 0;
}

int camera_ext_send_metadata(const uint8_t *desc, uint32_t length)
{
    if (s_event_cb == NULL) {
        CAM_ERR("event callback not exists\n");
        return -EINVAL;
    }

    if (length > CAMERA_EXT_EVENT_METADATA_DESC_LEN) {
        CAM_ERR("metadata too long\n");
        return -EINVAL;
    }

    return s_event_cb(s_dev, CAMERA_EXT_REPORT_METADATA, (uint8_t *)desc, length);
}

int camera_ext_send_error(uint32_t err)
{
    if (s_event_cb == NULL) {
        CAM_ERR("event callback not exists\n");
        return -EINVAL;
    }

    struct camera_ext_event_error err_msg;
    memset(&err_msg, 0, sizeof(err_msg));
    err_msg.error_code = cpu_to_le32(err);
    return s_event_cb(s_dev, CAMERA_EXT_REPORT_ERROR, (uint8_t *)&err_msg, sizeof(err_msg));
}

void camera_ext_register_format_db(const struct camera_ext_format_db *db)
{
    if (db == NULL)
        return;

    g_camera_ext_db.format_db = db;
}

const struct camera_ext_format_db *camera_ext_get_format_db(void)
{
    return g_camera_ext_db.format_db;
}

struct camera_ext_format_user_config *camera_ext_get_user_config(void)
{
    return &g_camera_ext_db.user_cfg;
}

void camera_ext_register_control_db(struct camera_ext_ctrl_db *ctrl_db)
{
    if (ctrl_db == NULL)
        return;

    g_camera_ext_db.ctrl_db = ctrl_db;
}

static struct camera_ext_ctrl_db *camera_ext_get_control_db(void)
{
    return g_camera_ext_db.ctrl_db;
}

//Find the format node (by fourcc) which is a sub node of a input node
//return index in the input->format_nodes array, -1 not found.
static int find_format_node(struct camera_ext_input_node const *input_node,
    __le32 pixelformat)
{
    int index;
    struct camera_ext_format_node const *format_node;

    for (index = 0; index < input_node->num_formats; index++) {
        format_node = &input_node->format_nodes[index];
        if (format_node->fourcc == le32_to_cpu(pixelformat)) {
            break;
        }
    }

    if (index >= input_node->num_formats) index = -1;
    return index;
}

/* Find the frmsize node (by dimention) which is a sub node of a format node.
 * return the index in the array. -1 not found
 */
static int find_frmsize_node(struct camera_ext_format_node const *format_node,
    __le32 width, __le32 height)
{
    int index;
    struct camera_ext_frmsize_node const *frmsize_node;

    for (index = 0; index < format_node->num_frmsizes; index++) {
        frmsize_node = &format_node->frmsize_nodes[index];
        if (frmsize_node->width == le32_to_cpu(width)
            && frmsize_node->height == le32_to_cpu(height)) {
            break;
        }
    }

    if (index >= format_node->num_frmsizes) index = -1;
    return index;
}

/* Functions to set user configuration */
int cam_ext_set_current_format(struct camera_ext_format_db const *db,
                               struct camera_ext_format_user_config *cfg,
                               struct camera_ext_format *format)
{
    int retval;
    int index_format;
    int index_frmsize;
    struct camera_ext_input_node const *input_node;
    struct camera_ext_format_node const *format_node;

    input_node = get_current_input_node(db, cfg);
    if (input_node == NULL)
        return -EINVAL;

    retval = -EINVAL;
    //find format node
    index_format = find_format_node(input_node, format->pixelformat);
    if (index_format != -1) {
        format_node = &input_node->format_nodes[index_format];
        //find the frmsize node which meets the requirement
        index_frmsize = find_frmsize_node(format_node, format->width,
                                format->height);
        if (index_frmsize != -1) {
            //store user config
            cfg->frmival = 0; //reset to first frame rate
            cfg->format = index_format;
            cfg->frmsize = index_frmsize;
            retval = 0;
        }
    }

    CAM_DBG("set format result %d\n", retval);
    return retval;
}

//Update the stream frame rate. If success, return the frmival_node in use
int cam_ext_frmival_set(struct camera_ext_format_db const *db,
                        struct camera_ext_format_user_config *cfg,
                        struct camera_ext_streamparm *parm)
{
    int index;
    struct camera_ext_frmsize_node const *frmsize_node;
    struct camera_ext_frmival_node const *frmival_node;

    frmsize_node = get_current_frmsize_node(db, cfg);
    if (frmsize_node == NULL) {
        CAM_ERR("invalid user config\n");
        return -EINVAL;
    }

    frmival_node = NULL;
    for (index = 0; index < frmsize_node->num_frmivals; index++) {
        frmival_node = &frmsize_node->frmival_nodes[index];
        if (camera_ext_fract_equal(frmival_node,
                                   &parm->capture.timeperframe)) {
            //accept
            cfg->frmival = index;
            break;
        } else {
            frmival_node = NULL;
        }
    }

    return 0;
}

/* Utility Function to access Format DB structure */
bool is_input_valid(struct camera_ext_format_db const *db, uint32_t input)
{
    return (input >= 0 && input < db->num_inputs) ? true : false;
}

bool is_format_valid(struct camera_ext_format_db const *db,
                                   uint32_t input, uint32_t format)
{
    if (is_input_valid(db, input)) {
        return (format >= 0 &&
                format < db->input_nodes[input].num_formats) ? true : false;
    }
    return false;
}

bool is_frmsize_valid(struct camera_ext_format_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize)
{
    if (is_format_valid(db, input, format)) {
        return (frmsize >= 0 &&
                frmsize < db->input_nodes[input].format_nodes[format].num_frmsizes) ?
            true : false;
    }
    return false;
}

bool is_frmival_valid(struct camera_ext_format_db const *db,
                      uint32_t input, uint32_t format, uint32_t frmsize, uint32_t frmival)
{
    if (is_frmsize_valid(db, input, format, frmsize)) {
        return (frmival >= 0 &&
                frmival < db->input_nodes[input].format_nodes[format].
                frmsize_nodes[frmsize].num_frmivals) ? true : false;
    }
    return false;
}

static struct camera_ext_input_node const *get_input_node(
    struct camera_ext_format_db const *db, uint32_t input)
{
    if (!is_input_valid(db, input))
        return NULL;

    return &db->input_nodes[input];
}

static struct camera_ext_format_node const *get_format_node(
    struct camera_ext_format_db const *db, uint32_t input, uint32_t format)
{
    if (!is_format_valid(db, input, format))
        return NULL;

    return &db->input_nodes[input].format_nodes[format];
}

static struct camera_ext_frmsize_node const *get_frmsize_node(
    struct camera_ext_format_db const *db, uint32_t input, uint32_t format, uint32_t frmsize)
{
    if (!is_frmsize_valid(db, input, format, frmsize))
        return NULL;

    return &db->input_nodes[input].format_nodes[format].frmsize_nodes[frmsize];
}

static struct camera_ext_frmival_node const *get_frmival_node(
    struct camera_ext_format_db const *db, uint32_t input, uint32_t format,
    uint32_t frmsize, uint32_t frmival)
{
    if (!is_frmival_valid(db, input, format, frmsize, frmival))
        return NULL;

    return &db->input_nodes[input].format_nodes[format].
        frmsize_nodes[frmsize].frmival_nodes[frmival];
}

struct camera_ext_input_node const *get_current_input_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg)
{
    return get_input_node(db, cfg->input);
}

struct camera_ext_format_node const *get_current_format_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg)
{
    return get_format_node(db, cfg->input, cfg->format);
}

struct camera_ext_frmsize_node const *get_current_frmsize_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg)
{
    return get_frmsize_node(db, cfg->input, cfg->format, cfg->frmsize);
}

struct camera_ext_frmival_node const *get_current_frmival_node(
    struct camera_ext_format_db const *db,
    struct camera_ext_format_user_config const *cfg)
{
    return get_frmival_node(db, cfg->input, cfg->format, cfg->frmsize, cfg->frmival);
}

/* Function to set GB data structure from Format DB structure */
int camera_ext_fill_gb_input(struct camera_ext_format_db const *db, uint32_t index,
                             struct camera_ext_input *input)
{
    const struct camera_ext_input_node *inode = get_input_node(db, index);

    if (inode == NULL)
        return -EINVAL;

    input->index = cpu_to_le32(index);
    strncat(input->name, inode->name, 31);
    input->type = cpu_to_le32(inode->type);
    input->status = cpu_to_le32(inode->status);
    input->capabilities = cpu_to_le32(inode->capabilities);
    return 0;
}

int camera_ext_fill_gb_fmtdesc(struct camera_ext_format_db const *db, uint32_t input,
                               uint32_t format, struct camera_ext_fmtdesc *fmt)
{
    const struct camera_ext_format_node *fnode = get_format_node(db, input, format);

    if (fnode == NULL)
        return -EINVAL;

    fmt->index = cpu_to_le32(format);
    strncat(fmt->name, fnode->name, 31);
    fmt->fourcc = cpu_to_le32(fnode->fourcc);
    fmt->depth = cpu_to_le32(fnode->depth);
    return 0;
}

int cam_ext_fill_gb_format(struct camera_ext_format_db const *db,
                           uint32_t input, uint32_t format, uint32_t frmsize,
                           struct camera_ext_format *fmt)
{
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;

    format_node = get_format_node(db, input, format);
    frmsize_node = get_frmsize_node(db, input, format, frmsize);

    if (format_node == NULL || frmsize_node == NULL) {
        CAM_ERR("invalid user config\n");
        return -EINVAL;
    }

    uint32_t w, h;
    w = frmsize_node->width;
    h = frmsize_node->height;
    uint32_t byte_count = (w * format_node->depth) >> 3;

    //considering 'step' or 'continuous' frame size,
    //calc other fields at run time.
    fmt->width = cpu_to_le32(w);
    fmt->height = cpu_to_le32(h);
    fmt->pixelformat = cpu_to_le32(format_node->fourcc);
    fmt->bytesperline = cpu_to_le32(byte_count);
    fmt->sizeimage = cpu_to_le32(byte_count * h);

    return 0;
}

int cam_ext_fill_gb_frmsize(struct camera_ext_format_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmsize* frmsize)
{
    int index_format;
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;
    struct camera_ext_input_node const *input_node = get_input_node(db, input);

    if (input_node == NULL)
        return -EINVAL;

    //find format node
    index_format = find_format_node(input_node, frmsize->pixelformat);
    if (index_format != -1) {
        format_node = &input_node->format_nodes[index_format];
        if (index >=0 && index < format_node->num_frmsizes) {
            frmsize_node = &format_node->frmsize_nodes[index];
            frmsize->index = cpu_to_le32(index);
            frmsize->type = cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_DISCRETE);
            frmsize->discrete.width = cpu_to_le32(frmsize_node->width);
            frmsize->discrete.height = cpu_to_le32(frmsize_node->height);
            return 0;
        }
    }

    return -EINVAL;
}

int cam_ext_fill_gb_frmival(struct camera_ext_format_db const *db, uint32_t input,
                            uint32_t index, struct camera_ext_frmival* frmival)
{
    int index_format;
    int index_frmsize;
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;
    struct camera_ext_frmival_node const *frmival_node;
    struct camera_ext_input_node const *input_node = get_input_node(db, input);

    if (input_node == NULL)
        return -EINVAL;

    //find format node
    index_format = find_format_node(input_node, frmival->pixelformat);
    if (index_format != -1) {
        format_node = &input_node->format_nodes[index_format];
        //find frmsize node
        index_frmsize = find_frmsize_node(format_node,
                                          le32_to_cpu(frmival->width),
                                          le32_to_cpu(frmival->height));
        if (index_frmsize != -1) {
            frmsize_node = &format_node->frmsize_nodes[index_frmsize];
            if (index >= 0  && index < frmsize_node->num_frmivals) {
                frmival_node = &frmsize_node->frmival_nodes[index];
                //copy raw data
                frmival->index = cpu_to_le32(index);
                frmival->type = cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_DISCRETE);
                frmival->discrete.numerator = cpu_to_le32(frmival_node->numerator);
                frmival->discrete.denominator = cpu_to_le32(frmival_node->denominator);
                return 0;
            }
        }
    }

    return -EINVAL;
}

int cam_ext_fill_gb_streamparm(struct camera_ext_format_db const *db,
                               struct camera_ext_format_user_config const *cfg,
                               uint32_t capability, uint32_t capturemode,
                               struct camera_ext_streamparm *parm)
{
    struct camera_ext_frmival_node const *frmival_node;

    frmival_node = get_current_frmival_node(db, cfg);
    if (frmival_node == NULL) {
        CAM_ERR("failed to get current frmival node\n");
        return -EINVAL;
    }

    memset(parm, 0, sizeof(*parm));
    parm->type = cpu_to_le32(CAMERA_EXT_BUFFER_TYPE_VIDEO_CAPTURE);
    parm->capture.timeperframe.numerator= cpu_to_le32(frmival_node->numerator);
    parm->capture.timeperframe.denominator = cpu_to_le32(frmival_node->denominator);
    parm->capture.capability = cpu_to_le32(capability);
    parm->capture.capturemode = cpu_to_le32(capturemode);

    return 0;
}

/* Common format db access functions each drivers can pick up */
int camera_ext_input_enum(struct device *dev, struct camera_ext_input *input)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    int index = le32_to_cpu(input->index);

    if(camera_ext_fill_gb_input(db, index, input) != 0) {
        CAM_DBG("no such input: %d\n", index);
        return -EFAULT;
    }
    return 0;
}

int camera_ext_input_get(struct device *dev, int *input)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    if (!is_input_valid(db, cfg->input)) {
        CAM_ERR("invalid current config\n");
        return -EFAULT;
    }

    *input = cfg->input;
    return 0;
}

int camera_ext_input_set(struct device *dev, int index)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    if (!is_input_valid(db, le32_to_cpu(index))) {
        CAM_DBG("v4l input index %d out of range\n", index);
        return -EFAULT;
    }

    cfg->input = index;

    return 0;
}

int camera_ext_format_enum(struct device *dev, struct camera_ext_fmtdesc *format)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    int index = le32_to_cpu(format->index);

    if (camera_ext_fill_gb_fmtdesc(db, cfg->input, index, format) != 0) {
        CAM_DBG("no such format: %d\n", index);
        return -EFAULT;
    }

    return 0;
}

int camera_ext_format_get(struct device *dev, struct camera_ext_format *format)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    return cam_ext_fill_gb_format(db, cfg->input, cfg->format, cfg->frmsize, format);
}

int camera_ext_format_set(struct device *dev, struct camera_ext_format* format)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    return cam_ext_set_current_format(db, cfg, format);
}

int camera_ext_frmsize_enum(struct device *dev, struct camera_ext_frmsize* frmsize)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    int index = le32_to_cpu(frmsize->index);

    return cam_ext_fill_gb_frmsize(db, cfg->input, index, frmsize);
}

int camera_ext_frmival_enum(struct device *dev, struct camera_ext_frmival* frmival)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();
    int index = le32_to_cpu(frmival->index);

    return cam_ext_fill_gb_frmival(db, cfg->input, index, frmival);
}

int camera_ext_stream_set_parm(struct device *dev, struct camera_ext_streamparm *parm)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    if (cam_ext_frmival_set(db, cfg, parm) != 0) {
        CAM_ERR("failed to apply stream parm\n");
        return -EINVAL;
    }

    return 0;
}

int camera_ext_stream_get_parm(struct device *dev, struct camera_ext_streamparm *parm)
{
    const struct camera_ext_format_db *db = camera_ext_get_format_db();
    const struct camera_ext_format_user_config *cfg = camera_ext_get_user_config();

    if (cam_ext_fill_gb_streamparm(db, cfg, 0, 0, parm) != 0) {
        CAM_ERR("failed to get current stream param\n");
        return -EINVAL;
    }

    return 0;
}

static inline void camera_ext_ctrl_float_set(camera_ext_ctrl_float *ff, float f)
{
    snprintf(*ff, sizeof(camera_ext_ctrl_float), "%.6e", f);
}

static inline float camera_ext_ctrl_float_get(camera_ext_ctrl_float *ff)
{
    return atof(*ff);
}

static inline void camera_ext_ctrl_double_set(camera_ext_ctrl_double *dd, double d)
{
    snprintf(*dd, sizeof(camera_ext_ctrl_double), "%.12e", d);
}

static inline double camera_ext_ctrl_double_get(camera_ext_ctrl_double *dd)
{
    return atof(*dd);
}

static int put_s64(uint8_t **p, uint32_t *size, uint32_t flag,
    const int64_t d)
{
    if (*size < sizeof(int64_t) + sizeof(uint32_t))
        return -EINVAL;
    *(__le32 *)*p = cpu_to_le32(flag);
    *p += sizeof(uint32_t);
    *(__le64 *)*p = cpu_to_le64(d);
    *p += sizeof(int64_t);
    *size -= sizeof(int64_t) + sizeof(uint32_t);
    return 0;
}

static int put_s64s(uint8_t **p, uint32_t *size, uint32_t flag,
        const int64_t *d, size_t d_num)
{
    size_t i;

    if (*size < sizeof(uint32_t) + d_num * sizeof(int64_t))
        return -EINVAL;
    *(__le32 *)*p = cpu_to_le32(flag);
    *p += sizeof(uint32_t);
    for (i = 0; i < d_num; i++) {
        *(__le64 *)*p = cpu_to_le64(*d++);
        *p += sizeof(int64_t);
    }
    *size -= sizeof(uint32_t) + d_num * sizeof(int64_t);
    return 0;
}

static int put_u64(uint8_t **p, uint32_t *size, uint32_t flag,
        const uint64_t d)
{
    if (*size < sizeof(uint64_t) + sizeof(uint32_t))
        return -EINVAL;

    *(__le32 *)*p = cpu_to_le32(flag);
    *p += sizeof(uint32_t);
    *(__le64 *)*p = cpu_to_le64(d);
    *p += sizeof(uint64_t);
    *size -= sizeof(uint64_t) + sizeof(uint32_t);
    return 0;
}

static int put_u32s(uint8_t **p, uint32_t *size, uint32_t flag,
        const uint32_t *d, uint32_t d_num)
{
    size_t i;

    if (*size < sizeof(uint32_t) + d_num * sizeof(uint32_t))
        return -EINVAL;

    *(__le32 *)*p = cpu_to_le32(flag);
    *p += sizeof(uint32_t);
    for (i = 0; i < d_num; i++) {
        *(__le32 *)*p = cpu_to_le32(*d++);
        *p += sizeof(uint32_t);
    }
    *size -= sizeof(uint32_t) + d_num * sizeof(uint32_t);
    return 0;
}

static int put_camera_ext_ctrl_floats(uint8_t **p, uint32_t *size,
            uint32_t flag, const float *d, uint32_t d_num)
{
    size_t i;

    if (*size < sizeof(uint32_t) + sizeof(camera_ext_ctrl_float) * d_num)
        return -EINVAL;

    *(__le32 *)*p = cpu_to_le32(flag);
    *p += sizeof(uint32_t);
    for (i = 0; i < d_num; i++) {
        camera_ext_ctrl_float_set((camera_ext_ctrl_float *)*p, *d++);
        *p += sizeof(camera_ext_ctrl_float);
    }
    *size -= sizeof(uint32_t) + sizeof(camera_ext_ctrl_float) * d_num;
    return 0;
}

static int put_def(uint8_t **p, uint32_t *size,
        const struct camera_ext_ctrl_val_cfg *val_cfg,
        const camera_ext_ctrl_val_t *def)
{
    int retval;
    size_t val_size;

    val_size = get_local_ctrl_val_size(val_cfg);
    if (*size < sizeof(uint32_t) + val_size) //flag:val
        return -EINVAL;

    *(__le32 *)*p = cpu_to_le32(CAMERA_EXT_CTRL_FLAG_NEED_DEF);
    *p += sizeof(uint32_t);
    *size -= sizeof(uint32_t);

    retval = ctrl_val_local_to_greybus(val_cfg, def, *p, *size, 1 /*cfg pkt*/);
    *p += val_size;
    *size -= val_size;

    return retval;
}
static int cfg_local_to_greybus(const struct camera_ext_ctrl_cfg *local,
            uint8_t *cfg, uint32_t cfg_size)
{
    int retval = 0;
    if (local->flags & CAMERA_EXT_CTRL_FLAG_NEED_MIN)
        retval = put_s64(&cfg, &cfg_size, CAMERA_EXT_CTRL_FLAG_NEED_MIN,
                    local->min);

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_MAX)
        retval = put_s64(&cfg, &cfg_size, CAMERA_EXT_CTRL_FLAG_NEED_MAX,
                    local->max);

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_STEP)
        retval = put_u64(&cfg, &cfg_size, CAMERA_EXT_CTRL_FLAG_NEED_STEP,
                    local->step);

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_DEF) {
        retval = put_def(&cfg, &cfg_size, &local->val_cfg, &local->def);
    }

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_DIMS) {
        if (local->array_size > CAMERA_EXT_MAX_DIM) {
            CAM_ERR("dim items %d exceed limitation %d\n",
                local->array_size, CAMERA_EXT_MAX_DIM);
            retval = -EINVAL;
        } else {
            retval = put_u32s(&cfg, &cfg_size, CAMERA_EXT_CTRL_FLAG_NEED_DIMS,
                    local->dims, local->array_size);
        }
    }

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK)
        retval = put_u64(&cfg, &cfg_size, CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK,
                    local->menu_skip_mask);

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT) {
        if (local->array_size > CAMERA_EXT_MAX_MENU_NUM) {
            CAM_ERR("menu items %d exceed limitation %d\n",
                local->array_size, CAMERA_EXT_MAX_MENU_NUM);
            retval = -EINVAL;
        } else {
            retval = put_s64s(&cfg, &cfg_size,
                    CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT,
                    local->menu_int, local->array_size);
        }
    }

    if (!retval && local->flags & CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT) {
        if (local->array_size > CAMERA_EXT_MAX_MENU_NUM) {
            CAM_ERR("menu items %d exceed limitation %d\n",
                local->array_size, CAMERA_EXT_MAX_MENU_NUM);
            retval = -EINVAL;
        } else {
            retval = put_camera_ext_ctrl_floats(&cfg, &cfg_size,
                CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT, local->menu_float,
                    local->array_size);
        }
    }

    if (cfg_size != 0 || retval != 0) {
        CAM_ERR("pack result %d, config data %d bytes left\n",
            retval, cfg_size);
        retval = -EINVAL;
    }

    return retval;
}

/* let the phone side know how to calculate the next GET_CFG response size */
static void cam_ext_ctrl_get_size_info(struct camera_ext_ctrl_db *ctrl_db,
    int idx, struct camera_ext_predefined_ctrl_mod_cfg *cfg)
{
    if (ctrl_db == NULL || idx >= ctrl_db->num_ctrls) {
        /* no more next */
        cfg->next.id = (uint32_t) -1;
    } else {
        uint16_t ctrl_ver = ctrl_db->ctrls[idx]->ver;
        if (ctrl_ver == 0) /* default as 1.0 if not set */
            ctrl_ver = 0x100;

        if (ctrl_ver <= s_phone_ver) {
            cfg->next.id = cpu_to_le32(ctrl_db->ctrls[idx]->id);
            cfg->next.array_size = cpu_to_le32(
                ctrl_db->ctrls[idx]->array_size);
            cfg->next.val_size = cpu_to_le32((uint32_t)
                get_local_ctrl_val_size(&ctrl_db->ctrls[idx]->val_cfg));
        } else
            cfg->next.id = (uint32_t) -1;
    }
}

int cam_ext_ctrl_get_cfg(struct camera_ext_ctrl_db *ctrl_db,
        uint32_t idx, struct camera_ext_predefined_ctrl_mod_cfg *cfg,
        uint32_t cfg_size)
{
    int retval = 0;

    cfg->next.array_size = 0;
    if (idx == (uint32_t) -1) { /* enumeration start */
        cfg->id = (uint32_t) -1;
        cam_ext_ctrl_get_size_info(ctrl_db, 0, cfg);
    } else if (idx < ctrl_db->num_ctrls) {
        //pack this control's config
        cfg->id = cpu_to_le32(ctrl_db->ctrls[idx]->id);
        retval = cfg_local_to_greybus(ctrl_db->ctrls[idx],
                cfg->data, cfg_size);

        if (retval == 0) {
            //fill next control size info
            cam_ext_ctrl_get_size_info(ctrl_db, idx + 1, cfg);
        }
    } else
        retval = -EINVAL;

    if (retval != 0) {
        CAM_ERR("failed to convert control %ld cfg to greybus. res %d\n",
            idx, retval);
    }

    return retval;
}

/* packet size to transfer this value over greybus */
static size_t get_local_ctrl_val_size(
        const struct camera_ext_ctrl_val_cfg *val_cfg)
{
    size_t elem_size;
    switch (val_cfg->elem_type) {
    case CAM_EXT_CTRL_DATA_TYPE_INT:
    case CAM_EXT_CTRL_DATA_TYPE_BOOL:
        elem_size = sizeof(uint32_t);
        break;
    case CAM_EXT_CTRL_DATA_TYPE_INT64:
        elem_size = sizeof(uint64_t);
        break;
    case CAM_EXT_CTRL_DATA_TYPE_FLOAT:
        elem_size = sizeof(camera_ext_ctrl_float);
        break;
    case CAM_EXT_CTRL_DATA_TYPE_DOUBLE:
        elem_size = sizeof(camera_ext_ctrl_double);
        break;
    case CAM_EXT_CTRL_DATA_TYPE_STRING:
        elem_size = sizeof(camera_ext_ctrl_string);
        break;
    default:
        CAM_ERR("unsupported val type %d\n", val_cfg->elem_type);
        elem_size = 0;
        break;
    }

    return elem_size * val_cfg->nr_of_elem;
}

static int ctrl_val_local_to_greybus(
        const struct camera_ext_ctrl_val_cfg *val_cfg,
        const camera_ext_ctrl_val_t *local_val, uint8_t *gb_val,
        const uint32_t gb_val_size, int is_cfg_pkt)
{
    size_t local_val_size = get_local_ctrl_val_size(val_cfg);

    if (is_cfg_pkt && gb_val_size < local_val_size) {
        /* get config process will check if the total packet size matches */
        CAM_ERR("not enough space, require %ld, provide %ld\n",
            gb_val_size, local_val_size);
    } else if (!is_cfg_pkt && gb_val_size != local_val_size) {
        CAM_ERR("control value mismatch, require %ld, provide %ld\n",
            gb_val_size, local_val_size);
        return -EINVAL;
    }

    size_t i;
    int is_array = val_cfg->nr_of_elem > 1;

    for (i = 0; i< val_cfg->nr_of_elem; i++) {
        switch (val_cfg->elem_type) {
        case CAM_EXT_CTRL_DATA_TYPE_INT:
        case CAM_EXT_CTRL_DATA_TYPE_BOOL:
            *(__le32 *)gb_val = cpu_to_le32(
                    is_array? local_val->p_val[i] : local_val->val);
            gb_val += sizeof(__le32);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_INT64:
            *(__le64 *)gb_val = cpu_to_le64(
                is_array? local_val->p_val_64[i] : local_val->val_64);
            gb_val += sizeof(__le64);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_FLOAT:
            camera_ext_ctrl_float_set((camera_ext_ctrl_float *)gb_val,
                is_array? local_val->p_val_f[i] : local_val->val_f);
            gb_val += sizeof(camera_ext_ctrl_float);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_DOUBLE:
            camera_ext_ctrl_double_set((camera_ext_ctrl_double *)gb_val,
                is_array? local_val->p_val_d[i] : local_val->val_d);
            gb_val += sizeof(camera_ext_ctrl_double);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_STRING:
            memcpy(gb_val, &(local_val->p_val_8[i * sizeof(camera_ext_ctrl_string)]), sizeof(camera_ext_ctrl_string));
            gb_val += sizeof(camera_ext_ctrl_string);
            break;
        default:
            CAM_ERR("unsupported val type %d\n", val_cfg->elem_type);
            return -EINVAL;
        }
    }
    return 0;
}

static int cam_ext_ctrl_get(struct device *dev,
        struct camera_ext_ctrl_db *ctrl_db, uint32_t idx, uint8_t *ctrl_val,
        uint32_t ctrl_val_size)
{
    int retval = -EINVAL;
    camera_ext_ctrl_val_t local_val;
    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->get_volatile_ctrl != NULL) {
        retval = ctrl_db->ctrls[idx]->get_volatile_ctrl(dev,
            ctrl_db->ctrls[idx], &local_val);

        if (retval == 0)
            retval = ctrl_val_local_to_greybus(&ctrl_db->ctrls[idx]->val_cfg,
                        &local_val, ctrl_val, ctrl_val_size, 0 /*not cfg pkt*/);
    }
    return retval;
}

/* For array data, we re-use the greybus message memory to avoid
 * allocate/free memory.
 * ctrl_val: from greybus
 * local_val: to driver (elem_type and nr_of_elem are set)
 */
static int ctrl_val_greybus_to_local(uint8_t *ctrl_val, size_t ctrl_val_size,
            const struct camera_ext_ctrl_val_cfg *val_cfg,
            camera_ext_ctrl_val_t *local_val)
{
    size_t local_val_size = get_local_ctrl_val_size(val_cfg);
    if (local_val_size != ctrl_val_size) {
        CAM_ERR("ctrl value from greybus has unexpected size %lu  "
            "expected %lu\n", ctrl_val_size, local_val_size);
        return -EINVAL;
    }

    int is_array = val_cfg->nr_of_elem > 1;
    size_t i;
    /* in-place data conversion for array. If the source and target data
     * has same length, p will be used to point to both.
     * Otherwise (float and double data) , p points to source data, f or d
     * points to target
     */
    uint8_t *p = ctrl_val;
    float *f = (float *)ctrl_val;
    double *d = (double *)ctrl_val;

    for (i = 0; i < val_cfg->nr_of_elem; i++) {
    switch (val_cfg->elem_type) {
        case CAM_EXT_CTRL_DATA_TYPE_INT:
        case CAM_EXT_CTRL_DATA_TYPE_BOOL:
            if (is_array) {
                if (i == 0) local_val->p_val = (uint32_t *)p;
                *(uint32_t *)p = le32_to_cpu(*(__le32 *)p);
                p += sizeof(uint32_t);
            } else
                local_val->val = le32_to_cpu(*(__le32 *)p);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_INT64:
            if (is_array) {
                if (i == 0) local_val->p_val_64 = (uint64_t *)p;
                *(uint64_t *)p = le64_to_cpu(*(__le64 *)p);
                p += sizeof(uint64_t);
            } else
                local_val->val_64 = le32_to_cpu(*(__le64 *)p);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_FLOAT:
            if (is_array) {
                if (i == 0)local_val->p_val_f = f;
                *f++ = camera_ext_ctrl_float_get((camera_ext_ctrl_float *) p);
                p += sizeof(camera_ext_ctrl_float);
            } else
                local_val->val_f = camera_ext_ctrl_float_get(
                    (camera_ext_ctrl_float *)p);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_DOUBLE:
            if (is_array) {
                if (i == 0) local_val->p_val_d = d;
                *d++ = camera_ext_ctrl_double_get((camera_ext_ctrl_double *)p);
                p += sizeof(camera_ext_ctrl_double);
            } else
                local_val->val_d = camera_ext_ctrl_double_get(
                    (camera_ext_ctrl_double *)p);
            break;
        case CAM_EXT_CTRL_DATA_TYPE_STRING:
            local_val->p_val_8 = p;
            break;
        default:
            CAM_ERR("unsupported data type %d\n", val_cfg->elem_type);
            return -EINVAL;
        }
    }

    return 0;
}

static int cam_ext_ctrl_set(struct device *dev,
        struct camera_ext_ctrl_db *ctrl_db, uint32_t idx, uint8_t *ctrl_val,
        size_t ctrl_val_size)
{
    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    int retval = -EINVAL;
    if (ctrl_db->ctrls[idx]->set_ctrl != NULL) {
        camera_ext_ctrl_val_t local_val;
        //convert to local struct camera_ext_ctrl_val
        retval = ctrl_val_greybus_to_local(ctrl_val, ctrl_val_size,
                    &ctrl_db->ctrls[idx]->val_cfg, &local_val);
        if (retval == 0)
            retval = ctrl_db->ctrls[idx]->set_ctrl(dev,
                ctrl_db->ctrls[idx], &local_val);
        else
            CAM_ERR("failed to set value for control %x\n",
                ctrl_db->ctrls[idx]->id);
    }
    return retval;
}

static int cam_ext_ctrl_try(struct device *dev,
        struct camera_ext_ctrl_db *ctrl_db, uint32_t idx,
        uint8_t *ctrl_val, size_t ctrl_val_size)
{
    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    int retval = -EINVAL;
    if (ctrl_db->ctrls[idx]->try_ctrl != NULL) {
        camera_ext_ctrl_val_t local_val;
        //convert to local struct camera_ext_ctrl_val
        retval = ctrl_val_greybus_to_local(ctrl_val, ctrl_val_size,
                    &ctrl_db->ctrls[idx]->val_cfg, &local_val);
        if (retval == 0)
            retval = ctrl_db->ctrls[idx]->try_ctrl(dev,
                ctrl_db->ctrls[idx], &local_val);
        return retval;
    }

    /* if try does not exist, no try needed, return ok */
    return 0;
}

int camera_ext_ctrl_get_cfg(struct device *dev, uint32_t idx,
        struct camera_ext_predefined_ctrl_mod_cfg *cfg, uint32_t cfg_size)
{
    struct camera_ext_ctrl_db *ctrl_db = camera_ext_get_control_db();

    return cam_ext_ctrl_get_cfg(ctrl_db, idx, cfg, cfg_size);
}

int camera_ext_ctrl_get(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size)
{
    struct camera_ext_ctrl_db *ctrl_db = camera_ext_get_control_db();

    return cam_ext_ctrl_get(dev, ctrl_db, idx, ctrl_val, ctrl_val_size);
}

int camera_ext_ctrl_set(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size)
{
    struct camera_ext_ctrl_db *ctrl_db = camera_ext_get_control_db();

    return cam_ext_ctrl_set(dev, ctrl_db, idx, ctrl_val, ctrl_val_size);
}

int camera_ext_ctrl_try(struct device *dev, uint32_t idx, uint8_t *ctrl_val,
    uint32_t ctrl_val_size)
{
    struct camera_ext_ctrl_db *ctrl_db = camera_ext_get_control_db();

    return cam_ext_ctrl_try(dev, ctrl_db, idx, ctrl_val, ctrl_val_size);
}

int camera_ext_set_phone_ver(struct device *dev, uint8_t major, uint8_t minor)
{
    s_phone_ver = (major << 8) | minor;

    return 0;
}

uint16_t camera_ext_get_phone_ver(void)
{
    return s_phone_ver;
}
