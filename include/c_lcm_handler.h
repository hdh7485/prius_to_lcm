#ifndef C_LCM_HANDLER_H
#define C_LCM_HANDLER_H

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "eurecar_lcmtypes/eurecar/cam.hpp"
#include "eurecar_lcmtypes/eurecar/cam_save.hpp"
#include "eurecar_lcmtypes/eurecar/can_t.hpp"
#include "eurecar_lcmtypes/eurecar/Ai_steer.hpp"
#include "eurecar_lcmtypes/eurecar/vision_imgframe.hpp"
#include "eurecar_lcmtypes/eurecar/velo_raw.hpp"

#include "eurecar_lcmtypes/eurecar/dw_object_detection.hpp"
#include "eurecar_lcmtypes/eurecar/dw_lane_detection.hpp"

class C_LCM_CAM_SAVE
{
public:
    ~C_LCM_CAM_SAVE () {}

    int64_t utime = 0;
    int32_t img_size = 0;
    //byte* img_bytes;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam_save* msg);
};


class C_LCM_CAM
{
public:
    ~C_LCM_CAM () {}

    int64_t utime = 0;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::cam* msg);
};

class C_LCM_CAN_T
{
public:
    ~C_LCM_CAN_T () {}

    int64_t    utime;
    double     time;
    double     yaw_rate;
    double     mdps_torque;
    double     mdps_str_ang;
    double     VS_CAN;
    double     lat_accel;
    double     mcp;
    double     accel_pedal_value;
    double     tps;
    double     odometer;
    double     battery_voltage;
    double     WHL_SPD_RR;
    double     WHL_SPD_RL;
    double     WHL_SPD_FR;
    double     WHL_SPD_FL;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::can_t * msg);
};

class C_LCM_VELO_RAW
{
public:
    ~C_LCM_VELO_RAW () {}

    int64_t utime = 0;
    uint8_t raw[1206];

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::velo_raw * msg);

};

class C_LCM_DW_OBJ
{
public:
    ~C_LCM_DW_OBJ () {}

    int64_t utime = 0;
    int32_t num_of_objects = 0;
    std::vector< int32_t > type;
    std::vector< int32_t > lefttop_x;
    std::vector< int32_t > lefttop_y;
    std::vector< int32_t > obj_width;
    std::vector< int32_t > obj_height;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::dw_object_detection * msg)
    {
        this->utime = msg->timestamp;
        this->num_of_objects = msg->num_of_objects;
        this->type = msg->type;
        this->lefttop_x = msg->lefttop_x;
        this->lefttop_y = msg->lefttop_y;
        this->obj_width = msg->obj_width;
        this->obj_height = msg->obj_height;

        std::cout << "num of objects : " << this->num_of_objects << std::endl;
    }
};

class C_LCM_DW_LANE
{
public:
    ~C_LCM_DW_LANE () {}

    int64_t utime = 0;
    int32_t num_of_lanes;
    std::vector< std::string > lane_pos;
    std::vector< int32_t > max_y;
    std::vector< int32_t > min_y;
    std::vector< int32_t > coef_3;
    std::vector< int32_t > coef_2;
    std::vector< int32_t > coef_1;
    std::vector< int32_t > coef_0;

    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::dw_lane_detection * msg)
    {
        this->utime = msg->timestamp;
        this->num_of_lanes = msg->num_of_lanes;
        this->lane_pos = msg->lane_pos;
        this->max_y = msg->max_y;
        this->min_y = msg->min_y;
        this->coef_3 = msg->coef_3;
        this->coef_2 = msg->coef_2;
        this->coef_1 = msg->coef_1;
        this->coef_0 = msg->coef_0;
    }
};

#endif // C_LCM_HANDLER_H

