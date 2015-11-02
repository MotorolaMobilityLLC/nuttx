#ifndef CSI_CAMERA_H
#define CSI_CAMERA_H

#define TEST_PATTERN

#include <nuttx/i2c.h>

typedef enum {
    OFF = 0,
    ON,
    STREAMING
} camera_status_t;

struct camera_dev_s
{
    camera_status_t status;
    struct i2c_dev_s *i2c;
    struct sensor_info *sensor;
};

struct gb_csi_camera_res {
    int x_output;
    int y_output;
    int line_length;
    int frame_length;
    int op_pixel_clk; //vfe pixel clk required
};

struct gb_csi_camera_info {
    int lane_cnt;
    int settle_cnt;
    int format_fourcc; //cid0 for now
    int res_size;
    struct gb_csi_camera_res res[4];
};

int cam_setup(struct camera_dev_s *cam_dev);
int cam_query_info(struct camera_dev_s *cam_dev,
		struct gb_csi_camera_info *cam_info);
int cam_power_on(struct camera_dev_s *cam_dev);
int cam_power_off(struct camera_dev_s *cam_dev);
int cam_stream_on(struct camera_dev_s *cam_dev);
int cam_stream_off(struct camera_dev_s *cam_dev);
int cam_set_resolution(struct camera_dev_s *cam_dev, int res);

#ifdef TEST_PATTERN
int cam_set_test_pattern(struct camera_dev_s *cam_dev, int test);
#endif

struct cam_i2c_reg_array {
    uint16_t reg_addr;
    uint16_t data;
};

struct cam_i2c_reg_setting {
    uint16_t size;
    struct cam_i2c_reg_array *regs;
};

struct cam_group_settings {
    uint16_t size;
    struct cam_i2c_reg_setting *settings;
};

struct sensor_info {
    uint16_t i2c_addr;
    struct gb_csi_camera_info cam_info;
    struct cam_i2c_reg_setting init;
    struct cam_i2c_reg_setting start;
    struct cam_i2c_reg_setting stop;
    struct cam_group_settings resolutions;
#ifdef TEST_PATTERN
    struct cam_group_settings test;
#endif
};

struct sensor_info *get_board_sensor_info(void);

#endif
