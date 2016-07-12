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
#include <errno.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nuttx/camera/camera_ext.h>
#include <nuttx/camera/camera_ext_dbg.h>
#include <nuttx/camera/camera_ext_meta.h>
#include <nuttx/time.h>

#define MY_TIMER_SIGNAL 10
#define SIGVALUE_INT  20
#define METADATA_HEADER 4
#define METADATA_MAX_IDLE_TIME_MS 5000LL
#define META_PAYLOAD_LENGTH 1024

/*
================================================================================
* Byte Order:

If a field specified in this document has more than one byte width, little
endian format will be used to put the value in the bit stream.

================================================================================

the line shoud be marked with
special signature (0xAA01) at first 2 byte. Next 2 bytes will indicate total
size of metadata block in the line. See the figure below.

byte    0      1      2      3      4       5
     +------+------+------+------+------+------
line | 0xAA | 0x01 |    length   | metadata ...
     +------+------+------+------+------+------

================================================================================
* Metadata Structure:

Each metadata will have own header in front of the payload. Multiple metadata
shall be concatenated back to back like the figure below.

 +----------+----------+----------+---------+----------+-----
 | metadata | payload  | metadata | payload | metadata |
 |  header  |          |  header  |         |  header  | continues ..
 |   (0)    |   (0)    |   (1)    |   (1)   |   (2)    |
 +----------+----------+----------+---------+----------+-----

================================================================================
* Metadata header

Metadata header shall consists of "tag" and "payload length". Tag will be
a unique value defined for each metadata. Length is the size of payload
following the metadata header.

             (byte)   0       1         2       3
                 +--------+--------+--------+--------+
                 |       tag       |     payload     |
                 |                 |      length     |
                 +--------+--------+--------+--------+

================================================================================
* Metadata types summary

The following is the list metadata and it's assigned tags.

  Tag: 0  - Auto Focus Status
  Tag: 1  - Zoom Status
  Tag: 2  - Postview frame identification
  Tag: 3  - Xenon Flash
  Tag: 4  - AE Status
  Tag: 5  - Faces
  Tag: 6  - Focus Distances

================================================================================
* "Auto Focus Status" (0x00)

               (byte)   0             1
                 +-------------+-------------+
                 |   Request   |   Status    |
                 |   Complete  |             |
                 +-------------+-------------+

   Request Complet Values:
   0 - not valid
   1 - success
   2 - failure

   Status Values:
   0 - not valid
   1 - started
   2 - stopped
   3 - updating

================================================================================
* "Zoom Status" (0x01)

  (byte)    0               1            3              4
       +-------------+-------------+-------------+-------------+
       |    Zoom     |  Zoom Value |  Zoom Value |  Reserved   |
       |   Activity  |  Numerator  | Denominator |             |
       +-------------+-------------+-------------+-------------+

   Zoom Activity Values:
   0 - not valid
   1 - moving
   2 - stopped

   Zoom Value :
   Represents current zoon value.

   example - Zoom Numerator = 75, Zoom Denominator = 10
             Current Zoom = 75 / 10 ==> 7.5X

================================================================================
* "Postview Frame Indication" (0x02)

               (byte)   0             1
                 +-------------+-------------+
                 |       Shutter Frame       |
                 |           Marker          |
                 +-------------+-------------+

   Shutter Frame Maker value (N):
   0 .. (greater than zero)
   N indicats N frames earlier of tis frame was the time shutter event was
   triggered. N = 2 means shutter was triggered 2 frames before this frame.

================================================================================
* "Xenon Flash Status" (0x03)

               (byte)   0             1
                 +-------------+-------------+
                 |   Charged   |    Auto     |
                 |             |    Flash    |
                 +-------------+-------------+

   Charged Values:
   0 - not valid
   1 - charged
   2 - not charged

   Auto Flash Values :
   0 - not valid
   1 - Fire the flash if FlashMode is set to "Auto"

================================================================================
* "AE Status" (0x04)

  (byte)    0               1            3              4
       +-------------+-------------+-------------+-------------+
       |                        Exposure                       |
       |                          Time                         |
       +-------------+-------------+-------------+-------------+
       |                        Exposure                       |
       |                          Time                         |
       +-------------+-------------+-------------+-------------+
       |            Lux            |            Lux            |
       |          Numerator        |        Denominator        |
       +-------------+-------------+-------------+-------------+
       |            ISO            |  Aperture   | Aperture    |
       |            Gain           |  Numerator  | Denominator |
       +-------------+-------------+-------------+-------------+

   Exposure Time :
   8 bytes field. Unit is in nano seconds

   Lux Numerator and Denoninator :
   Represents Lux value in float.

   example - Lux Numerator = 125, Denominator = 1000
             Then, Lux = 125 / 1000 = 0.125

   ISO Gain :
   Signed integer. Unit in dBm.

   Aperture Numerator and Denominator :
   Represents f/N value in float.

   example - Aperture Numerator = 22, Denominator = 10
             Then Aperture = 22/10 ==> f/2.2

================================================================================
* "Faces" (0x05)

               (byte)   0             1
                 +-------------+-------------+
                 |            Num            |
                 |           Faces           |
                 +-------------+-------------+
                 |     Id      |     Score   |
                 |             |             |
                 +-------------+-------------+
                 |         Face Bound        |
                 |           bottom          |
                 +-------------+-------------+
                 |         Face Bound        |
                 |           left            |
                 +-------------+-------------+
                 |         Face Bound        |
                 |           right           |
                 +-------------+-------------+
                 |         Face Bound        |
                 |            top            |
                 +-------------+-------------+
                 |         Left Eye          |
                 |             x             |
                 +-------------+-------------+
                 |         Left Eye          |
                 |             y             |
                 +-------------+-------------+
                 |         Right Eye         |
                 |             x             |
                 +-------------+-------------+
                 |         Right Eye         |
                 |             y             |
                 +-------------+-------------+
                 |           Mouth           |
                 |             x             |
                 +-------------+-------------+
                 |           Mouth           |
                 |             y             |
                 +-------------+-------------+
                 |  Repeats for Num Faces    |
                 |           ....            |

   Num Faces :
   Unsigned Integer. Number of Faces in the metadata.

   Id :
   Signed Integer. -1 if Id is not supported.

   Score :
   Range 1 - 100. -1 if Score is not supported.

   Face Bound (bottom, left, right, top) :
   Signed Integer. Must be supported.

   Left Eye Point (x, y) :
   Signed Integer. Set to -1 if not supported.

   Right Eye Point (x, y) :
   Signed Integer. Set to -1 if not supported.

   Mouth Point (x, y) :
   Signed Integer. Set to -1 if not supported.

================================================================================
* "Focus Distances" (0x06)

  (byte)    0               1            3              4
       +-------------+-------------+-------------+-------------+
       |         Near Focus        |         Near Focus        |
       |          Numerator        |        Denominator        |
       +-------------+-------------+-------------+-------------+
       |      Optimal Focus        |      Optimal Focus        |
       |          Numerator        |        Denominator        |
       +-------------+-------------+-------------+-------------+
       |          Far Focus        |          Far Focus        |
       |          Numerator        |        Denominator        |
       +-------------+-------------+-------------+-------------+

   Focus Distance Numerator and Denoninator :
   Optimal Focus Distance is the distance with sharpest image. The subject distance may be within
   near and far focus distance.
   Unit in meters. Denominator=0 means infinite focus

   example - Focu Distance Numerator = 155, Denominator = 10
             Then, Focus Distance = 15.5 meters

================================================================================
*/

typedef struct cam_metadata {
    cam_metadata_autofocus_t autofocus;
    cam_metadata_zoom_t zoomchange;
    cam_metadata_postview postview;
    cam_metadata_xenonflash_t xenon;
    cam_metadata_ae_status_t ae;
    cam_metadata_faces_t faces;
    cam_metadata_focus_distance_t focusdistance;
} cam_metadata_t;

static sem_t sem;

static uint8_t meta[META_PAYLOAD_LENGTH];
static cam_metadata_t metadata_value;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/* this empty_meta buffer will read by meta task */
static uint8_t empty_meta[META_PAYLOAD_LENGTH];

static pthread_t thread;

static pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool task_running;

void set_autofocus_metadata(cam_metadata_autofocus_request_result_e result,
        cam_metadata_autofocus_status_e af_status)
{
    pthread_mutex_lock(&mutex);

    metadata_value.autofocus.valid = true;
    metadata_value.autofocus.request_result = result;
    metadata_value.autofocus.status = af_status;

    pthread_mutex_unlock(&mutex);
}

void set_zoomchange_metadata(cam_metadata_zoom_activity_e activity,
        uint8_t zoom_numerator, uint8_t zoom_denominator)
{
    pthread_mutex_lock(&mutex);

    metadata_value.zoomchange.valid = true;
    metadata_value.zoomchange.activity = activity;
    metadata_value.zoomchange.zoom_numerator = zoom_numerator;
    metadata_value.zoomchange.zoom_denominator = zoom_denominator;
    metadata_value.zoomchange.reserved = 0;

    pthread_mutex_unlock(&mutex);
}

void set_postview_metadata(cam_metadata_postview_shutter_frame_marker_t marker)
{
    pthread_mutex_lock(&mutex);

    metadata_value.postview.valid = true;
    metadata_value.postview.marker = marker;

    pthread_mutex_unlock(&mutex);
}

void set_xenon_metadata(cam_metadata_xenonflash_charged_e charged,
        cam_metadata_xenonflash_autoflash_e autoflash)
{
    pthread_mutex_lock(&mutex);

    metadata_value.xenon.valid = true;
    metadata_value.xenon.charged_value = charged;
    metadata_value.xenon.autoflash_value = autoflash;

    pthread_mutex_unlock(&mutex);
}

void set_ae_metadata( uint64_t exposure_time, uint16_t lux_numerator,
        uint16_t lux_denominator, int16_t  iso_gain,
        uint8_t  aperture_numerator, uint8_t  aperture_denominator)
{
    pthread_mutex_lock(&mutex);

    metadata_value.ae.valid = true;
    metadata_value.ae.exposure_time = exposure_time;  // unit is in nano seconds
    metadata_value.ae.lux_numerator = lux_numerator;
    metadata_value.ae.lux_denominator = lux_denominator;
    metadata_value.ae.iso_gain = iso_gain;     //signed integer, unit in dBm
    metadata_value.ae.aperture_numerator = aperture_numerator;
    metadata_value.ae.aperture_denominator = aperture_denominator;

    pthread_mutex_unlock(&mutex);
}

bool add_face(int8_t id, int8_t score, int16_t f_b_b, int16_t f_b_l, int16_t f_b_r,
              int16_t f_b_t, int16_t l_e_x, int16_t l_e_y, int16_t r_e_x,
              int16_t r_e_y, int16_t m_x, int16_t m_y)
{
    int num;

    pthread_mutex_lock(&mutex);
    num = metadata_value.faces.num_faces;
    if (num >= MHB_CAM_FACE_MAX_COUNT) {
        CAM_ERR("%s: face data already full\n", __func__);
        pthread_mutex_unlock(&mutex);
        return false;
    }

    if (num == 0)
        metadata_value.faces.valid = true;

    metadata_value.faces.face[num].id = id;
    metadata_value.faces.face[num].score = score;
    metadata_value.faces.face[num].face_bound_bottom = f_b_b;
    metadata_value.faces.face[num].face_bound_left = f_b_l;
    metadata_value.faces.face[num].face_bound_right = f_b_r;
    metadata_value.faces.face[num].face_bound_top = f_b_t;
    metadata_value.faces.face[num].left_eye_x = l_e_x;
    metadata_value.faces.face[num].left_eye_y = l_e_y;
    metadata_value.faces.face[num].right_eye_x = r_e_x;
    metadata_value.faces.face[num].right_eye_y = r_e_y;
    metadata_value.faces.face[num].mouth_x = m_x;
    metadata_value.faces.face[num].mouth_y = m_y;
    metadata_value.faces.num_faces++;
    pthread_mutex_unlock(&mutex);

    return true;
}

void set_focusdistance_metadata(uint16_t near_focus_numerator,
        uint16_t near_focus_denominator, uint16_t optimal_focus_numerator,
        uint16_t optimal_focus_denominator, uint16_t far_focus_numerator,
        uint16_t far_focus_denominator)
{
    pthread_mutex_lock(&mutex);

    metadata_value.focusdistance.valid = true;
    metadata_value.focusdistance.near_focus_numerator = near_focus_numerator;
    metadata_value.focusdistance.near_focus_denominator = near_focus_denominator;
    metadata_value.focusdistance.optimal_focus_numerator = optimal_focus_numerator;
    metadata_value.focusdistance.optimal_focus_denominator = optimal_focus_denominator;
    metadata_value.focusdistance.far_focus_numerator = far_focus_numerator;
    metadata_value.focusdistance.far_focus_denominator = far_focus_denominator;

    pthread_mutex_unlock(&mutex);
}

static uint16_t populate_metadata_autofocus(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.autofocus.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);
    uint8_t temp_value;

    temp_value = (uint8_t) metadata_value.autofocus.request_result;
    memcpy((uint8_t *)(point + offset), &temp_value, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    temp_value = metadata_value.autofocus.status;
    memcpy((uint8_t *)(point + offset), &temp_value, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_zoom(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.zoomchange.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);
    uint8_t temp_value;

    temp_value = (uint8_t) metadata_value.zoomchange.activity;
    memcpy((uint8_t *)(point + offset), &temp_value, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    memcpy((uint8_t *)(point + offset), &metadata_value.zoomchange.zoom_numerator, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy((uint8_t *)(point + offset), &metadata_value.zoomchange.zoom_denominator, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy((uint8_t *)(point + offset), &metadata_value.zoomchange.reserved, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_postview(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.postview.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);

    memcpy((uint16_t *)(point + offset), &metadata_value.postview.marker, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_xenon(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.xenon.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);

    uint8_t temp_value;
    temp_value = (uint8_t)metadata_value.xenon.charged_value;
    memcpy((uint8_t *)(point + offset), &temp_value, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    temp_value = (uint8_t)metadata_value.xenon.autoflash_value;
    memcpy((uint8_t *)(point + offset), &temp_value, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_ae(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.ae.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);

    memcpy((uint64_t *)(point + offset), &metadata_value.ae.exposure_time, sizeof(uint64_t));
    offset += sizeof(uint64_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.ae.lux_numerator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.ae.lux_denominator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((int16_t *)(point + offset), &metadata_value.ae.iso_gain, sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy((uint8_t *)(point + offset), &metadata_value.ae.aperture_numerator, sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy((uint8_t *)(point + offset), &metadata_value.ae.aperture_denominator, sizeof(uint8_t));
    offset += sizeof(uint8_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_faces(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.faces.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);

    memcpy((uint16_t *)(point + offset), &metadata_value.faces.num_faces, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    int i;
    for (i = 0; i < metadata_value.faces.num_faces; i++)
    {
        memcpy((int8_t *)(point + offset), &metadata_value.faces.face[i].id, sizeof(int8_t));
        offset += sizeof(int8_t);
        memcpy((int8_t *)(point + offset), &metadata_value.faces.face[i].score, sizeof(int8_t));
        offset += sizeof(int8_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].face_bound_bottom, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].face_bound_left, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].face_bound_right, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].face_bound_top, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].left_eye_x, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].left_eye_y, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].right_eye_x, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].right_eye_y, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].mouth_x, sizeof(int16_t));
        offset += sizeof(int16_t);
        memcpy((int16_t *)(point + offset), &metadata_value.faces.face[i].mouth_y, sizeof(int16_t));
        offset += sizeof(int16_t);
    }

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static uint16_t populate_metadata_focusdistance(uint8_t *point, uint16_t tag)
{
    if (!metadata_value.focusdistance.valid)
        return 0;

    uint16_t offset = 0;
    memcpy(point+offset, &tag, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    offset += sizeof(uint16_t);

    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.near_focus_numerator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.near_focus_denominator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.optimal_focus_numerator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.optimal_focus_denominator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.far_focus_denominator, sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy((uint16_t *)(point + offset), &metadata_value.focusdistance.far_focus_denominator, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    uint16_t size =  offset - METADATA_HEADER;
    memcpy((uint16_t *)(point + sizeof(uint16_t)), &size, sizeof(uint16_t));

    return offset;
}

static void populate_metadata_stream(void)
{
    uint8_t *pos;
    uint16_t payload_length;

    memset(meta, 0, META_PAYLOAD_LENGTH);
    meta[0] = 0xaa;
    meta[1] = 0x01;
    pos = meta + METADATA_HEADER;

    pthread_mutex_lock(&mutex);

    payload_length = populate_metadata_autofocus(pos, 0);
    pos += payload_length;
    payload_length = populate_metadata_zoom(pos, 1);
    pos += payload_length;
    payload_length = populate_metadata_postview(pos, 2);
    pos += payload_length;
    payload_length = populate_metadata_xenon(pos, 3);
    pos += payload_length;
    payload_length = populate_metadata_ae(pos, 4);
    pos += payload_length;
    payload_length = populate_metadata_faces(pos, 5);
    pos += payload_length;
    payload_length = populate_metadata_focusdistance(pos, 6);
    pos += payload_length;

    // With current structure of cam_metadata, the payload size will be 246 in maxium case.
    // will not exceed the 1024 size.

    uint16_t total_length = (uint16_t)(pos - meta);
    memcpy((uint16_t *)(meta + sizeof(uint16_t)), &total_length, sizeof(uint16_t));
    memset(&metadata_value, 0, sizeof(metadata_value));
    pthread_mutex_unlock(&mutex);
}

static void *metadata_task(void* arg)
{
    struct timespec now_ts;
    uint64_t now_ns;
    struct timespec next_cp_ts;

    sem_init(&sem, 0, 0);
    CAM_DBG("metadata task enter\n");
    while (true) {
        clock_gettime(CLOCK_REALTIME, &now_ts);
        now_ns = timespec_to_nsec(&now_ts);
        nsec_to_timespec(now_ns + METADATA_MAX_IDLE_TIME_MS * NSEC_PER_MSEC,
                &next_cp_ts);

        if (sem_timedwait(&sem, &next_cp_ts) != 0) {
            if (errno != ETIMEDOUT) {
                CAM_ERR("sem wait errno %d\n", errno);
                break;
            } else {
                camera_ext_send_metadata(empty_meta, META_PAYLOAD_LENGTH);
            }
        } else if (task_running) {
            populate_metadata_stream();
            camera_ext_send_metadata(meta, META_PAYLOAD_LENGTH);
        } else
            break;
    }
    CAM_DBG("metadata task exit\n");

    return NULL;
}

#ifdef CONFIG_PM
static int mhb_camera_metadata_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
    return task_running ? -EBUSY : 0;
}

static struct pm_callback_s pm_callback =
{
    .prepare = mhb_camera_metadata_pm_prepare,
};
#endif

int init_metadata_task(void)
{
#ifdef CONFIG_PM
    pm_register(&pm_callback);
#endif
    memset(&metadata_value, 0, sizeof(metadata_value));
    populate_metadata_stream();
    memcpy(empty_meta, meta, META_PAYLOAD_LENGTH);

    return 0;
}

int start_metadata_task(void)
{
    pthread_mutex_lock(&task_mutex);
    if (task_running) {
        CAM_ERR("meta data task already running\n");
        pthread_mutex_unlock(&task_mutex);
        return 0; /* re-use */
    }

    task_running = true; /* metadata_task loop condition */
    if (pthread_create(&thread, NULL, &metadata_task, NULL) != 0) {
        CAM_ERR("Failed to start metadata thread\n");
        task_running = false;
        pthread_mutex_unlock(&task_mutex);
        return -1;
    }

    pthread_mutex_unlock(&task_mutex);
    return 0;
}

void stop_metadata_task(void)
{
    pthread_mutex_lock(&task_mutex);
    task_running = false; /* meta task loop condition */
    sem_post(&sem);
    pthread_join(thread, NULL);
    pthread_mutex_unlock(&task_mutex);
}

void send_metadata_oneshot(void)
{
    sem_post(&sem);
}
