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

#ifndef __CAMERA_EXT_DEFS_H
#define __CAMERA_EXT_DEFS_H

/*
 * These structs defined here are transferred via greybus.
 * This file is used by AP and MOD. Modify both at the same time and keep them same.
 */

#define CAM_EXT_INPUT_TYPE_TUNER  1
#define CAM_EXT_INPUT_TYPE_CAMERA 2

/* field 'status' - sensor orientation */
//from V4L2_IN_ST_HFLIP
#define CAM_EXT_IN_ST_HFLIP 0x00000010 /* Frames are flipped horizontally */
//from V4L2_IN_ST_VFLIP
#define CAM_EXT_IN_ST_VFLIP 0x00000020 /* Frames are flipped vertically */

#define __packed    __attribute__((packed))

/* from v4l2_input */
struct camera_ext_input {
    __le32 index;
    char name[32];
    __le32 type;
    __le32 status;
} __packed;

/* from v4l2_fmtdesc */
struct camera_ext_fmtdesc {
    __le32 index;
    char name[32];
    __le32 fourcc;
    __le32 depth;
} __packed;

/* from v4l2_format.v4l2_pix_format (V4L2_BUF_TYPE_VIDEO_CAPTURE) */
struct camera_ext_format {
    __le32 width;
    __le32 height;
    __le32 pixelformat;
    __le32 bytesperline;
    __le32 sizeimage;
} __packed;

#define CAM_EXT_FRMSIZE_TYPE_DISCRETE   1
#define CAM_EXT_FRMSIZE_TYPE_CONTINUOUS 2
#define CAM_EXT_FRMSIZE_TYPE_STEPWISE   3

/* from v4l2_frmsize_discrete */
struct camera_ext_frmsize_discrete {
    __le32 width;
    __le32 height;
} __packed;

/* from v4l2_frmsize_stepwise */
struct camera_ext_frmsize_stepwise {
    __le32 min_width;
    __le32 max_width;
    __le32 step_width;
    __le32 min_height;
    __le32 max_height;
    __le32 step_height;
} __packed;

/* v4l2_frmsizeenum */
struct camera_ext_frmsize {
    __le32 index;
    __le32 pixelformat; //four cc
    __le32 type; //see CSI_FRMSIZE_TYPE_XYZ

    union {
        struct camera_ext_frmsize_discrete discrete;
        struct camera_ext_frmsize_stepwise stepwise;
    };
} __packed;

#define CAM_EXT_FRMIVAL_TYPE_DISCRETE   1
#define CAM_EXT_FRMIVAL_TYPE_CONTINUOUS 2
#define CAM_EXT_FRMIVAL_TYPE_STEPWISE   3

/* from v4l2_fract */
struct camera_ext_fract {
    __le32 numerator;
    __le32 denominator;
} __packed;

static inline int camera_ext_fract_equal(struct camera_ext_fract const *l,
        struct camera_ext_fract const *r)
{
    //suppose they are irreducible fraction
    return l->numerator == r->numerator
        && l->denominator == r->denominator;
}

/* from v4l2_frmival_stepwise */
struct camera_ext_frmival_stepwise {
    struct camera_ext_fract min;
    struct camera_ext_fract max;
    struct camera_ext_fract step;
} __packed;

/* from v4l2_frmivalenum */
struct camera_ext_frmival {
    __le32 index;
    __le32 pixelformat; //four cc
    __le32 width;
    __le32 height;
    __le32 type;

    union {
        struct camera_ext_fract discrete;
        struct camera_ext_frmival_stepwise stepwise;
    };
} __packed;

/* from v4l2_captureparm */
struct camera_ext_captureparm {
    struct camera_ext_fract timeperframe;
} __packed;

/* from v4l2_streamparm */
struct camera_ext_streamparm {
    __le32 type;
    union {
        struct camera_ext_captureparm capture;
        uint8_t raw_data[200]; /*user defined */
    };
} __packed;

#endif
