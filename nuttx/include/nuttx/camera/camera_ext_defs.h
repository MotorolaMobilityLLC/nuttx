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

#define CAMERA_EXT_STREAM_CAP_PREVIEW   (1 << 0)
#define CAMERA_EXT_STREAM_CAP_VIDEO     (1 << 1)
#define CAMERA_EXT_STREAM_CAP_SNAPSHOT  (1 << 2)

/* from v4l2_input */
struct camera_ext_input {
    __le32 index;
    char name[32];
    __le32 type;
    __le32 status;
    __le32 capabilities;
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

/*  Flags for 'capability' and 'capturemode' fields */
#define CAMERA_EXT_MODE_HIGHQUALITY 0x0001 /* V4L2_MODE_HIGHQUALITY */
#define CAMERA_EXT_CAP_TIMEPERFRAME 0x1000 /* V4L2_CAP_TIMEPERFRAME */

/* from v4l2_captureparm */
struct camera_ext_captureparm {
    __le32 capability;
    __le32 capturemode;
    struct camera_ext_fract timeperframe;
    __le32 extendedmode;
    __le32 readbuffers;
} __packed;

/* V4L2_BUF_TYPE_VIDEO_CAPTURE */
#define CAMERA_EXT_BUFFER_TYPE_VIDEO_CAPTURE 1

/* from v4l2_streamparm */
struct camera_ext_streamparm {
    __le32 type; /* CAMERA_EXT_BUFFER_TYPE_XXX */
    union {
        struct camera_ext_captureparm capture;
        uint8_t raw_data[200]; /*user defined */
    };
} __packed;

#define CAM_EXT_CTRL_ID_MASK		(0x0fffffff)

/*
 * Phone side predfines controls and indicating which fields are
 * needed from MOD.
 * MOD will send missing config in the binary format "FLAG0 data0 FLAG1
 * data1 ...
 * FLAG takes a __le32 and data is in the format in each flag's comment.
 */
#define CAMERA_EXT_CTRL_FLAG_NEED_MIN		0x00010000 /* s64 */
#define CAMERA_EXT_CTRL_FLAG_NEED_MAX		0x00020000 /* s64 */
#define CAMERA_EXT_CTRL_FLAG_NEED_STEP		0x00040000 /* u64 */
#define CAMERA_EXT_CTRL_FLAG_NEED_DEF		0x00080000 /* s64 */
#define CAMERA_EXT_CTRL_FLAG_NEED_DIMS		0x00100000 /* u32 [4] */
#define CAMERA_EXT_CTRL_FLAG_NEED_MENU_MASK	0x00200000 /* u64 */
#define CAMERA_EXT_CTRL_FLAG_NEED_MENU_INT	0x00400000 /* s64 [64] */
/* camera_ext_ctrl_float[64]: uint8_t [64][13] */
#define CAMERA_EXT_CTRL_FLAG_NEED_MENU_FLOAT	0x00800000

#define CAMERA_EXT_MAX_MENU_NUM     64
#define CAMERA_EXT_MAX_DIM	4

/* If this flag is set, make sure the default value from MOD is properly
 * passed to control (by set_ctrl).
 */
#define CAMERA_EXT_CTRL_FLAG_STRING_AS_NUMBER	0x80000000

/* Y.XXXXXXE-ZZZ plus '\0': len is 13 */
#define CAM_EXT_CTRL_FLOAT_STR_LEN  13
/* Y.XXXXXXXXXXXXE-ZZZ plus '\0': len 19 */
#define CAM_EXT_CTRL_DOUBLE_STR_LEN 19
typedef char camera_ext_ctrl_float[CAM_EXT_CTRL_FLOAT_STR_LEN];
typedef char camera_ext_ctrl_double[CAM_EXT_CTRL_DOUBLE_STR_LEN];

#define CAM_EXT_CTRL_STRING_MAX_LEN 64
typedef char camera_ext_ctrl_string[CAM_EXT_CTRL_STRING_MAX_LEN];

struct camera_ext_predefined_ctrl_mod_req {
	/* Phone access MOD control by index (0, 1, ...).
	 */
	__le32 idx;
	/* required response size
	 * Expected responding config data size (including header) for
	 * GB_CAMERA_EXT_TYPE_CTRL_GET_CFG.
	 * Expected control value size for GB_CAMERA_EXT_TYPE_CTRL_GET.
	 * Size of control value to set/try for GB_CAMERA_EXT_TYPE_CTRL_SET/TRY.
	 */
	__le32 data_size;
	/* control value to set for GB_CAMERA_EXT_TYPE_CTRL_SET/TRY
	 */
	uint8_t data[0];
};

/* max control value size */
#define CAMERA_EXT_CTRL_MAX_VAL_SIZE (16 * 1024)

/* next available control info (to calc expected gb response size) */
struct camera_ext_ctrl_size_info {
	/* id, -1 if no more */
	__le32 id;
	/* if next control has menu/dims from MOD, indicating the array
	 * item number */
	__le32 array_size;
	/* next control's value size */
	__le32 val_size;
} __packed;

/* ctrl config from MOD, playload is decided by
 * CAMERA_EXT_CTRL_FLAG_NEED_XXX. MOD side must provide all fields
 * each field is tagged by its NEED_XXX flag.
 */
struct camera_ext_predefined_ctrl_mod_cfg {
	/* control id at position idx (camera_ext_predefined_ctrl_mod_req) */
	__le32 id;
	struct camera_ext_ctrl_size_info next;
	/* FLAG0 DATA0 FLAG1 DATA1 ... */
	uint8_t data[0];
} __packed;

#define CAMERA_EXT_REPORT_ERROR 0x00
#define CAMERA_EXT_REPORT_METADATA 0x01

/* use 1024 as metadata length*/
#define CAMERA_EXT_EVENT_METADATA_DESC_LEN 1024

/* event send from MOD to AP */
struct camera_ext_event_hdr {
    __le32 type;
    uint8_t data[0];
} __packed;

/* error code for CAMERA_EXT_REPORT_ERROR */
#define CAMERA_EXT_ERROR_FATAL      0x00
#define CAMERA_EXT_ERROR_POWER_ON   0x01
#define CAMERA_EXT_ERROR_STREAM_ON  0x02
#define CAMERA_EXT_ERROR_CSI_RESET  0x03
#define CAMERA_EXT_ERROR_POWER_OFF  0x04
#define CAMERA_EXT_ERROR_STREAM_OFF 0x05
#define CAMERA_EXT_ERROR_TIMEOUT    0x06
#define CAMERA_EXT_ERROR_SOC_CRASH  0x07

struct camera_ext_event_error {
    __le32 error_code;
} __packed;

/* open mode hint value sent along with power up request */
#define CAMERA_EXT_BOOTMODE_NORMAL  0x00
#define CAMERA_EXT_BOOTMODE_PREVIEW 0x01
#define CAMERA_EXT_BOOTMODE_DFU     0x02

#endif /* __CAMERA_EXT_DEFS_H */
