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
#include "camera_ext.h"

struct format_cfg {
    const struct camera_ext_format_db *format_db;
    struct camera_ext_format_user_config user_cfg;
};

/*
 * TODO: Need to support multiple instances of format_cfg below if more than
 *       one camera_ext protocol is configured in a single manifest.
*/
static struct format_cfg g_format_cfg;

void camera_ext_register_format_db(const struct camera_ext_format_db *db)
{
    if (db == NULL)
        return;

    memset(&g_format_cfg, 0, sizeof(g_format_cfg));
    g_format_cfg.format_db = db;
}

const struct camera_ext_format_db *camera_ext_get_format_db(void)
{
    return g_format_cfg.format_db;
}

struct camera_ext_format_user_config *camera_ext_get_user_config(void)
{
    return &g_format_cfg.user_cfg;
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

/* Functions to set GB data structure from Control DB structure */
int cam_ext_ctrl_get_cfg(struct camera_ext_ctrl_db *ctrl_db, uint32_t idx,
        struct camera_ext_predefined_ctrl_mod_cfg *mod_ctrl_cfg)
{
    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    memcpy(mod_ctrl_cfg, &ctrl_db->ctrls[idx]->ctrl_cfg,
        sizeof(*mod_ctrl_cfg));
    return 0;
}

int cam_ext_ctrl_get(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->g_volatile_ctrl != NULL)
        return ctrl_db->ctrls[idx]->g_volatile_ctrl(ctrl_val);
    return -EINVAL;
}

int cam_ext_ctrl_set(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->s_ctrl != NULL)
        return ctrl_db->ctrls[idx]->s_ctrl(ctrl_val);
    return -EINVAL;
}

int cam_ext_ctrl_try(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->try_ctrl != NULL)
        return ctrl_db->ctrls[idx]->try_ctrl(ctrl_val);
    /* if try does not exist, no try needed, return ok */
    return 0;
}


int cam_ext_ctrl_array_get(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_array_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->g_volatile_ctrl != NULL)
        return ctrl_db->ctrls[idx]->g_volatile_ctrl(ctrl_val);
    return -EINVAL;
}

int cam_ext_ctrl_array_set(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_array_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->s_ctrl != NULL)
        return ctrl_db->ctrls[idx]->s_ctrl(ctrl_val);
    return -EINVAL;
}

int cam_ext_ctrl_array_try(struct camera_ext_ctrl_db *ctrl_db,
        struct camera_ext_ctrl_array_val *ctrl_val)
{
    uint32_t idx = cam_ext_get_ctrl_val_idx(ctrl_val);

    if (idx >= ctrl_db->num_ctrls) return -EINVAL;

    if (ctrl_db->ctrls[idx]->try_ctrl != NULL)
        return ctrl_db->ctrls[idx]->try_ctrl(ctrl_val);
    /* no need to try, always ok */
    return 0;
}
