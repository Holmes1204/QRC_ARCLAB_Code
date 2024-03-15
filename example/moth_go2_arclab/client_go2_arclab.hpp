#ifndef __TG_MOTH_CLIENT_HPP__
#define __TG_MOTH_CLIENT_HPP__


#include <cmath>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <chrono>
#include "json.hpp"
#include <string>
#include <mutex>



#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>


#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include <unitree/robot/go2/robot_state/robot_state_client.hpp>


#include <thread>
#include <functional>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <stdint.h>

#include <arclab_control/arclab_controller.h>


#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"
#define TOPIC_LOWCMD "rt/lowcmd"

using json = nlohmann::json;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
using namespace unitree::robot;
using namespace unitree::common;

typedef websocketpp::client<websocketpp::config::asio_client> client;
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

class Custom {
public:
	Custom(client* c, websocketpp::connection_hdl hdl);

	void GetInitState();
	void HighStateHandler(const void *message);
	void LowStateMessageHandler(const void* message);
	void MessageSend(client* c, websocketpp::connection_hdl hdl);

	//our code
	void InitLowCmd();                     
	void RoboControl();
	void createControlThread(){
		/*loop publishing thread*/
    	lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 4000, &Custom::RoboControl, this);
	}

	float dt = 0.005;      // 控制步长0.001~0.01
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;

private:
	unitree_go::msg::dds_::SportModeState_ state;
	unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

	double px0, py0, yaw0; // 初始状态的位置和偏航
	double ct = 0;         // 运行时间
	int flag = 0;          // 特殊动作执行标志
//	float dt = 0.005;      // 控制步长0.001~0.01
	ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
	/*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
	
    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
	unitree_go::msg::dds_::LowState_ low_state{};

	client* c;
	websocketpp::connection_hdl hdl;
	json j;
	std::mutex mtx;
	/*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
	//
	//Joy command
	double lastPos[12];
	int swapJointIndices_[12]={3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
	rl_controller controller;
	gamepad_t mygamepad;
	proprioSense_t feedback;
	joint_cmd_t jcmd;
	int motiontime = 0;
	int Button_A_count = 0;
	int mode=0;
	int mode_1_count= 0;
};

void on_open(client* c, websocketpp::connection_hdl hdl);
void on_fail(client* c, websocketpp::connection_hdl hdl);
void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg);
void on_close(client* c, websocketpp::connection_hdl hdl);

//GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data);
//std::future<void> asyncVideo(client*c, websocketpp::connection_hdl hdl);

#endif
