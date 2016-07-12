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

#ifndef CAMERA_EXT_META_H
#define CAMERA_EXT_META_H

#ifdef CONFIG_PM
#include <nuttx/power/pm.h>
#endif

/* USAGE: meta data apis are used by mhb camera device driver.
 * - init_meta_data_task in device probe
 * - start_meta_data_task when streaming on
 * - stop_metadata_task when streaming off
 * - During streaming, call one or more set_xyz_metadata functions, then call
 *   send_metadata_oneshot. If send_metadata_oneshot is not called in
 *   METADATA_MAX_IDLE_TIME_MS, meta data task will send out a default/empty
 *   meta data.
 */
typedef enum {
    CAM_METADATA_AUTOFOCUS_REQ_NOT_VALID = 0,
    CAM_METADATA_AUTOFOCUS_REQ_SUCCESS = 1,
    CAM_METADATA_AUTOFOCUS_REQ_FAILURE = 2,
} cam_metadata_autofocus_request_result_e;

typedef enum {
    CAM_METADATA_AUTOFOCUS_STATUS_NOT_VALID = 0,
    CAM_METADATA_AUTOFOCUS_STATUS_STARTED = 1,
    CAM_METADATA_AUTOFOCUS_STATUS_STOPPED = 2,
    CAM_METADATA_AUTOFOCUS_STATUS_UPDATING = 3,
} cam_metadata_autofocus_status_e;

typedef struct {
    bool valid;
    cam_metadata_autofocus_request_result_e request_result;
    cam_metadata_autofocus_status_e status;
} cam_metadata_autofocus_t;


typedef enum {
    CAM_METADATA_ZOOM_ACTIVITY_NOT_VALID = 0,
    CAM_METADATA_ZOOM_ACTIVITY_MOVING = 1,
    CAM_METADATA_ZOOM_ACTIVITY_STOPPED = 2,
} cam_metadata_zoom_activity_e;

typedef struct {
    bool valid;
    cam_metadata_zoom_activity_e activity;
    uint8_t zoom_numerator;
    uint8_t zoom_denominator;
    uint8_t reserved;
} cam_metadata_zoom_t;

typedef uint16_t cam_metadata_postview_shutter_frame_marker_t;

typedef enum {
    CAM_METADATA_XENONFLASH_NOT_VALID = 0,
    CAM_METADATA_XENONFLASH_CHARGED = 1,
    CAM_METADATA_XENONFLASH_NOT_CHARGED = 2,
} cam_metadata_xenonflash_charged_e;

typedef enum {
    CAM_METADATA_XENONFLASH_AUTOFLASH_NOT_VALID = 0,
    CAM_METADATA_XENONFLASH_AUTO = 1,
} cam_metadata_xenonflash_autoflash_e;

typedef struct {
    bool valid;
    cam_metadata_xenonflash_charged_e charged_value;
    cam_metadata_xenonflash_autoflash_e autoflash_value;
} cam_metadata_xenonflash_t;

typedef struct {
    bool valid;
    uint64_t exposure_time;  // unit is in nano seconds
    uint16_t lux_numerator;
    uint16_t lux_denominator;
    int16_t  iso_gain;     //signed integer, unit in dBm
    uint8_t  aperture_numerator;
    uint8_t  aperture_denominator;
} cam_metadata_ae_status_t;

typedef struct {
    bool valid;
    cam_metadata_postview_shutter_frame_marker_t marker;
} cam_metadata_postview;

typedef struct {
    int8_t id;
    int8_t score;
    int16_t face_bound_bottom;
    int16_t face_bound_left;
    int16_t face_bound_right;
    int16_t face_bound_top;
    int16_t left_eye_x;
    int16_t left_eye_y;
    int16_t right_eye_x;
    int16_t right_eye_y;
    int16_t mouth_x;
    int16_t mouth_y;
} camera_face_t;

#define MHB_CAM_FACE_MAX_COUNT 8

typedef struct {
    bool valid;
    uint16_t num_faces;
    camera_face_t face[MHB_CAM_FACE_MAX_COUNT];
} cam_metadata_faces_t;

typedef struct {
    bool valid;
    uint16_t near_focus_numerator;
    uint16_t near_focus_denominator;
    uint16_t optimal_focus_numerator;
    uint16_t optimal_focus_denominator;
    uint16_t far_focus_numerator;
    uint16_t far_focus_denominator;
} cam_metadata_focus_distance_t;

int init_metadata_task(void);
int start_metadata_task(void);
void stop_metadata_task(void);

void send_metadata_oneshot(void);

void set_autofocus_metadata(cam_metadata_autofocus_request_result_e result,
        cam_metadata_autofocus_status_e status);
void set_zoomchange_metadata(cam_metadata_zoom_activity_e activity,
        uint8_t zoom_numerator, uint8_t zoom_denominator);
void set_postview_metadata(cam_metadata_postview_shutter_frame_marker_t marker);
void set_xenon_metadata(cam_metadata_xenonflash_charged_e charged,
        cam_metadata_xenonflash_autoflash_e autoflash);
void set_ae_metadata( uint64_t exposure_time, uint16_t lux_numerator,
        uint16_t lux_denominator, int16_t  iso_gain,
        uint8_t  aperture_numerator, uint8_t  aperture_denominator);
void init_faces_metadata(void);
bool add_face(int8_t id, int8_t score, int16_t f_b_b, int16_t f_b_l, int16_t f_b_r,
              int16_t f_b_t, int16_t l_e_x, int16_t l_e_y, int16_t r_e_x,
              int16_t r_e_y, int16_t m_x, int16_t m_y);
void set_focusdistance_metadata(uint16_t near_focus_numerator,
        uint16_t near_focus_denominator, uint16_t optimal_focus_numerator,
        uint16_t optimal_focus_denominator, uint16_t far_focus_numerator,
        uint16_t far_focus_denominator);

#endif
