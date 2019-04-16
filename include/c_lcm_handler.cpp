#include "c_lcm_handler.h"

void C_LCM_CAM_SAVE::handleMessage(const lcm::ReceiveBuffer *rbuf, const string &chan, const eurecar::cam_save *msg)
{
//    utime = msg->timestamp;
//    img_size = msg->img_size;
//    img_bytes = new byte[img_size];
//    std::memcpy(img_bytes,&(msg->img),img_size * sizeof(byte));
}

void C_LCM_CAM::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::__cxx11::string &chan, const eurecar::cam *msg)
{
    utime = msg->timestamp;
}

void C_LCM_CAN_T::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const eurecar::can_t * msg)
{
    utime = msg->utime;
    time= msg->time;
    yaw_rate = msg->yaw_rate;
    mdps_str_ang = msg->mdps_str_ang;
    VS_CAN = msg->VS_CAN;
    lat_accel = msg->lat_accel;
    mcp = msg->mcp;
    accel_pedal_value = msg->accel_pedal_value;
    tps = msg->tps;
    odometer = msg->odometer;
    battery_voltage = msg->battery_voltage;
    WHL_SPD_RR = msg->WHL_SPD_RR;
    WHL_SPD_RL = msg->WHL_SPD_RL;
    WHL_SPD_FR = msg->WHL_SPD_FR;
    WHL_SPD_FL = msg->WHL_SPD_FL;
}

void C_LCM_VELO_RAW::handleMessage(const lcm::ReceiveBuffer *rbuf, const string &chan, const eurecar::velo_raw *msg)
{
    utime = msg->utime;
    memcpy(raw,msg->raw,1206);
}

void C_LCM_DW_OBJ::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::__cxx11::string &chan, const eurecar::dw_object_detection *msg)
{
    this->utime = msg->timestamp;
    this->num_of_objects = msg->num_of_objects;
    this->type = msg->type;
    this->lefttop_x = msg->lefttop_x;
    this->lefttop_y = msg->lefttop_y;
    this->obj_width = msg->obj_width;
    this->obj_height = msg->obj_height;
}
