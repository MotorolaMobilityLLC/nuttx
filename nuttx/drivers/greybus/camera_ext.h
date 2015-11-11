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

#ifndef CAMERA_EXT_H
#define CAMERA_EXT_H
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <arch/byteorder.h>
#include <nuttx/greybus/types.h>
#include <nuttx/camera_ext_defs.h>

#define DEBUG
#include "camera_ext_dbg.h"

#define CAMERA_UNIPRO_CPORT 16
#define IPC_TIMEOUT 1000

#define BW_MEGA_BIT(w, h, depth, fps) \
    ((((uint32_t)w*(uint32_t)h*(uint32_t)depth*(uint32_t)fps)>>20) + 1)

/*
 * Structures used to store static sensor v4l2 info data in a hierachy way.
 * Each node has a raw data and points to its subnodes array.
 *
 */
struct camera_ext_frmival_raw_data {
    const __le32 type;
    union {
        const struct camera_ext_fract discrete;
        const struct camera_ext_frmival_stepwise stepwise;
    };
};

struct camera_ext_frmsize_raw_data {
    const __le32 type;
    union {
        const struct camera_ext_frmsize_discrete discrete;
        const struct camera_ext_frmsize_stepwise stepwise;
    };
};

/* Frame interval for a specific frame size and format.
 * It's a leaf node and may store some user data per
 * sensor type (csi camera or usb camera).
 */
struct camera_ext_frmival_node {
    const struct camera_ext_frmival_raw_data raw_data;
    void const *user_data;
};

//frame size for a specific format
struct camera_ext_frmsize_node {
    const struct camera_ext_frmsize_raw_data raw_data;

    const int num_frmivals;
    const struct camera_ext_frmival_node *frmival_nodes;
};

//format for a specific input
struct camera_ext_format_node {
    const struct camera_ext_fmtdesc raw_data;

    const int num_frmsizes;
    struct camera_ext_frmsize_node const *frmsize_nodes;
};

//input of this sensor
struct camera_ext_input_node {
    const struct camera_ext_input raw_data;

    const int num_formats;
    struct camera_ext_format_node const *format_nodes;
};

//root node
struct gb_camera_ext_sensor_db {
    const int num_inputs;
    struct camera_ext_input_node const *input_nodes;
};

struct frmsize_user_config {
    uint32_t idx_frmsize;
    //actual frame size if idx points to a step/continuous type
    uint32_t width;
    uint32_t height;
};

struct frmival_user_config {
    uint32_t idx_frmival;
    //actual frame interval if idx points to a step/continuous type
    uint32_t numerator;
    uint32_t denominator;
};

struct gb_camera_ext_sensor_user_config {
    uint32_t input;//index of current input
    uint32_t format; //index of current format
    struct frmsize_user_config frmsize;
    struct frmival_user_config frmival;
};

//helper functions
static inline struct camera_ext_input_node const *get_input_node(
    struct gb_camera_ext_sensor_db const *db, int index)
{
    if (index >= 0 && index < db->num_inputs) {
        return &db->input_nodes[index];
    }
    return NULL;
}

static inline struct camera_ext_format_node const *get_format_node(
    struct gb_camera_ext_sensor_db const *db, int input, int format)
{
    struct camera_ext_input_node const *input_node = get_input_node(db, input);
    if (input_node != NULL) {
        if (format >= 0 && format < input_node->num_formats) {
            return &input_node->format_nodes[format];
        }
    }
    return NULL;
}

static inline struct camera_ext_frmsize_node const *get_frmsize_node(
    struct gb_camera_ext_sensor_db const *db, int input, int format,
    struct frmsize_user_config *frmsize)
{
    struct camera_ext_format_node const *format_node =
                get_format_node(db, input, format);

    if (format_node != NULL) {
        if (frmsize->idx_frmsize >= 0
            && frmsize->idx_frmsize < format_node->num_frmsizes) {
            return &format_node->frmsize_nodes[frmsize->idx_frmsize];
        }
    }
    return NULL;
}

static inline struct camera_ext_frmival_node const *get_frmival_node(
    struct gb_camera_ext_sensor_db const *db, int input, int format,
    struct frmsize_user_config *frmsize, struct frmival_user_config *frmival)
{
    struct camera_ext_frmsize_node const *frmsize_node =
                get_frmsize_node(db, input, format, frmsize);

    if (frmsize_node != NULL) {
        if (frmival->idx_frmival >= 0
            && frmival->idx_frmival < frmsize_node->num_frmivals) {
            return &frmsize_node->frmival_nodes[frmival->idx_frmival];
        }
    }
    return NULL;
}

static inline struct camera_ext_input_node const *get_current_input_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg)
{
    return get_input_node(db, cfg->input);
}

static inline struct camera_ext_format_node const *get_current_format_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg)
{
    return get_format_node(db, cfg->input, cfg->format);
}

static inline struct camera_ext_frmsize_node const *get_current_frmsize_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg)
{
    return get_frmsize_node(db, cfg->input, cfg->format, &cfg->frmsize);
}

static inline struct camera_ext_frmival_node const *get_current_frmival_node(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg)
{
    return get_frmival_node(db,
            cfg->input,
            cfg->format,
            &cfg->frmsize,
            &cfg->frmival);
}

int cam_ext_fill_current_format(struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_format *format);

int cam_ext_set_current_format(struct camera_ext_input_node const *input_node,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_format *format);

int cam_ext_frmsize_enum(struct camera_ext_input_node const *input_node,
    int index, struct camera_ext_frmsize* fmsize);

int cam_ext_frmival_enum(struct camera_ext_input_node const *input_node,
    int index, struct camera_ext_frmival* frmival);

struct camera_ext_frmival_node const *cam_ext_frmival_set(
    struct gb_camera_ext_sensor_db const *db,
    struct gb_camera_ext_sensor_user_config *cfg,
    struct camera_ext_streamparm *parm);

#endif
