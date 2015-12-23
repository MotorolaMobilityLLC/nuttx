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

//fill format with current user config
int cam_ext_fill_current_format(struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_format *format)
{
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;

    format_node = get_current_format_node(db, cfg);
    frmsize_node = get_current_frmsize_node(db, cfg);

    if (format_node == NULL || frmsize_node == NULL) {
        CAM_ERR("invalid user config\n");
        return -EINVAL;
    }

    if (frmsize_node->raw_data.type ==
            cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_DISCRETE)) {
        format->width = frmsize_node->raw_data.discrete.width;
        format->height = frmsize_node->raw_data.discrete.height;
    } else {
        //step or continuous
        format->width = cfg->frmsize.width;
        format->height = cfg->frmsize.height;
    }

    //considering 'step' or 'continuous' frame size,
    //calc other fields at run time.
    format->pixelformat = format_node->raw_data.fourcc;
    format->bytesperline = (format->width * format_node->raw_data.depth) >> 3;
    format->sizeimage = format->bytesperline * format->height;
    return 0;

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
        if (format_node->raw_data.fourcc == pixelformat) {
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
        if (frmsize_node->raw_data.type == CAM_EXT_FRMSIZE_TYPE_DISCRETE) {
            if (frmsize_node->raw_data.discrete.width == width
                && frmsize_node->raw_data.discrete.height == height) {
                break;
            }
        } else if (frmsize_node->raw_data.type
            == CAM_EXT_FRMSIZE_TYPE_CONTINUOUS) {
            //TODO: store w/h to frmsize_user_config
            break;
        } else if (frmsize_node->raw_data.type
            == CAM_EXT_FRMSIZE_TYPE_STEPWISE) {
            //TODO: store w/h to frmsize_user_config
            //check if the width/height meet the step requirement
            break;
        }
    }

    if (index >= format_node->num_frmsizes) index = -1;
    return index;
}

int cam_ext_set_current_format(struct camera_ext_input_node const *input_node,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_format *format)
{
    int retval;
    int index_format;
    int index_frmsize;
    struct camera_ext_format_node const *format_node;

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
            cfg->frmival.idx_frmival = 0; //reset to first frame rate
            cfg->format = index_format;
            cfg->frmsize.idx_frmsize = index_frmsize;
            retval = 0;
        }
    }

    CAM_DBG("set format result %d\n", retval);
    return retval;
}

int cam_ext_frmsize_enum(struct camera_ext_input_node const *input_node,
    int index, struct camera_ext_frmsize* frmsize)
{
    int index_format;
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;

    //find format node
    index_format = find_format_node(input_node, frmsize->pixelformat);
    if (index_format != -1) {
        format_node = &input_node->format_nodes[index_format];
        if (index >=0 && index < format_node->num_frmsizes) {
            frmsize_node = &format_node->frmsize_nodes[index];
            frmsize->index = index;
            frmsize->type = frmsize_node->raw_data.type;
            if (frmsize->type == cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_DISCRETE)) {
                frmsize->discrete = frmsize_node->raw_data.discrete;
            } else if (frmsize->type
                        == cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_STEPWISE)) {
                frmsize->stepwise = frmsize_node->raw_data.stepwise;
            }
            return 0;
        }
    }

    return -EINVAL;
}

int cam_ext_frmival_enum(struct camera_ext_input_node const *input_node,
    int index, struct camera_ext_frmival* frmival)
{
    int index_format;
    int index_frmsize;
    struct camera_ext_format_node const *format_node;
    struct camera_ext_frmsize_node const *frmsize_node;
    struct camera_ext_frmival_node const *frmival_node;

    //find format node
    index_format = find_format_node(input_node, frmival->pixelformat);
    if (index_format != -1) {
        format_node = &input_node->format_nodes[index_format];
        //find frmsize node
        index_frmsize = find_frmsize_node(format_node,
                                frmival->width, frmival->height);
        if (index_frmsize != -1) {
            frmsize_node = &format_node->frmsize_nodes[index_frmsize];
            if (index >= 0  && index < frmsize_node->num_frmivals) {
                frmival_node = &frmsize_node->frmival_nodes[index];
                //copy raw data
                frmival->index = index;
                frmival->type = frmival_node->raw_data.type;
                if (frmival->type == cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_DISCRETE)) {
                    frmival->discrete = frmival_node->raw_data.discrete;
                } else if (frmival->type
                            == cpu_to_le32(CAM_EXT_FRMSIZE_TYPE_STEPWISE)) {
                    frmival->stepwise = frmival_node->raw_data.stepwise;
                }
                return 0;
            }
        }
    }

    return -EINVAL;
}

//Update the stream frame rate. If success, return the frmival_node in use
struct camera_ext_frmival_node const *cam_ext_frmival_set(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_streamparm *parm)
{
    int index;
    struct camera_ext_frmsize_node const *frmsize_node;
    struct camera_ext_frmival_node const *frmival_node;

    frmsize_node = get_current_frmsize_node(db, cfg);
    if (frmsize_node == NULL) {
        CAM_ERR("invalid user config\n");
        return NULL;
    }

    frmival_node = NULL;
    for (index = 0; index < frmsize_node->num_frmivals; index++) {
        frmival_node = &frmsize_node->frmival_nodes[index];
        if (frmival_node->raw_data.type
            == cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_DISCRETE)) {
            if (camera_ext_fract_equal(&frmival_node->raw_data.discrete,
                    &parm->capture.timeperframe)) {
                //accept
                cfg->frmival.idx_frmival = index;
                break;
            } else {
                frmival_node = NULL;
            }
        } else if (frmival_node->raw_data.type
            == cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_STEPWISE)) {
            //TODO: not support for now
            frmival_node = NULL;
        } else if (frmival_node->raw_data.type
            == cpu_to_le32(CAM_EXT_FRMIVAL_TYPE_CONTINUOUS)) {
            //TODO: not support for now
            frmival_node = NULL;
        }
    }

    return frmival_node;
}

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
