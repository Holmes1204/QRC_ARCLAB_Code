#ifndef BASIC_ARCLAB_TYPE_
#define BASIC_ARCLAB_TYPE_
struct gamepad_t
{
    double vx;
    double vy;
    double yaw;
};


struct joint_t{
    double pos[12];
    double vel[12];
};


struct imu_t{
    double vx;
    double vy;
    double vz;
    double ax;
    double ay;
    double az;
    double w;
    double x;
    double y;
    double z;
};

struct proprioSense_t
{
joint_t joint_;
imu_t imu_;
};

struct joint_cmd_t{
    double pos[12];
    double vel[12];
    double torque[12];
    double kp[12];
    double kd[12];
};
#endif