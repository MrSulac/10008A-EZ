#pragma once
#include "bot/config.hpp"
namespace ez {
class Drive { public:
 double left=0,right=0,heading=0,x=0,y=0; int leftCommand=0,rightCommand=0; unsigned corrections=0;
 double odom_x_get(){return x;} double odom_y_get(){return y;} double odom_theta_get(){return heading;}
 double drive_sensor_left(){return left;} double drive_sensor_right(){return right;} double drive_imu_get(){return heading;}
 void odom_xy_set(double px,double py){x=px;y=py;++corrections;}
 void drive_set(int l,int r){leftCommand=l;rightCommand=r;}
};
inline std::string lastScreen;
inline void screen_print(const std::string& text,int){lastScreen=text;}
}
inline ez::Drive chassis;
