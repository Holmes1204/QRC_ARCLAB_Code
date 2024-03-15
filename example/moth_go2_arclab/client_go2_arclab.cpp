/********************************************************************** Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "client_go2_arclab.hpp"
#include <gst/gst.h>
#include <gst/app/app.h>

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

GstSample* global_sample;
uint16_t key = 0;
float lx = 0.0;
float ly = 0.0;
float rx = 0.0;
float ry = 0.0;
auto start = std::chrono::high_resolution_clock::now();
bool control_flag = true;
bool videoflag = false;
bool cp_control = false;
bool gst_flag_ = true;

std::string url;
json globalData;

std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

std::chrono::system_clock::time_point control_time = std::chrono::system_clock::now();

Custom::Custom(client* c, websocketpp::connection_hdl hdl) : c(c), hdl(hdl)
{
	suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
	suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

	InitLowCmd();
    /*create subscriber*/
	lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
	lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

	/*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

};

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::GetInitState()
{
	px0 = state.position()[0];
	py0 = state.position()[1];
	yaw0 = state.imu_state().rpy()[2];
	std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
};

void Custom::MessageSend(client* c, websocketpp::connection_hdl hdl)
{
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> now_time = now - start_time;
	std::chrono::duration<double, std::milli> control_ = now - control_time;
	if (control_.count()/100 > 0.3){
		lx = 0.0;
		ly = 0.0;
		rx = 0.0;
		ry = 0.0;
		key = 0;
	}
	j["data"]["timestamp"] = now_time.count() / 1000;
	j["type"] = "lab";
	j["data"]["RPY"]["Roll"] = std::round(state.imu_state().rpy()[0]*180/3.1415926535 * 10)/10;
	j["data"]["RPY"]["Pitch"] = std::round(state.imu_state().rpy()[1]*180/3.1415926535 * 10)/10;
	j["data"]["RPY"]["Yaw"] = std::round(state.imu_state().rpy()[2]*180/3.1415926535 * 10)/10;
	j["data"]["control"] = globalData;
	j["data"]["motionParameters"]["Vx"] = state.velocity()[0];
	j["data"]["motionParameters"]["Vy"] = state.velocity()[1];
	j["data"]["motionParameters"]["Wz"] = state.velocity()[2];
	j["data"]["motionParameters"]["bodyHeight"] = state.body_height();
	j["data"]["motionParameters"]["footRaise"] = state.foot_raise_height();
	j["data"]["capture"] = cp_control;
	j["data"]["timestamp"] = now_time.count() / 1000;
	j["data"]["legAngles"]["RF"]["Hip"] = std::round(low_state.motor_state()[0].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["RF"]["Thigh"] = std::round(low_state.motor_state()[1].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["RF"]["Calf"] = std::round(low_state.motor_state()[2].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LF"]["Hip"] = std::round(low_state.motor_state()[3].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LF"]["Thigh"] = std::round(low_state.motor_state()[4].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LF"]["Calf"] = std::round(low_state.motor_state()[5].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["RR"]["Hip"] = std::round(low_state.motor_state()[6].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["RR"]["Thigh"] = std::round(low_state.motor_state()[7].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["RR"]["Calf"] = std::round(low_state.motor_state()[8].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LR"]["Hip"] = std::round(low_state.motor_state()[9].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LR"]["Thigh"] = std::round(low_state.motor_state()[10].q()*180/3.1415926535 * 10)/10;
	j["data"]["legAngles"]["LR"]["Calf"] = std::round(low_state.motor_state()[11].q()*180/3.1415926535 * 10)/10;
	std::string jsonString = j.dump();
	std::vector<uint8_t> qdata(jsonString.begin(), jsonString.end());
	c->send(hdl,qdata.data(),qdata.size(), websocketpp::frame::opcode::binary);
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
	if (cp_control == true){
		cp_control = false;
	}
	//we will append our control algorithm in here


}


void Custom::RoboControl()
{
  motiontime++;
  //robot state
//   udp.GetRecv(state);
  //joy msg
//   memcpy(&_keyData, &state.wirelessRemote[0], 40);
    for (size_t i = 0; i < 12; i++)
    {
        int swap_i = swapJointIndices_[i];
        feedback.joint_.pos[i] = low_state.motor_state()[swap_i].q();
        feedback.joint_.vel[i] = low_state.motor_state()[swap_i].dq();
    }

    feedback.imu_.vx = low_state.imu_state().gyroscope()[0];
    feedback.imu_.vy = low_state.imu_state().gyroscope()[1];
    feedback.imu_.vz = low_state.imu_state().gyroscope()[2];

    feedback.imu_.ax = low_state.imu_state().accelerometer()[0];
    feedback.imu_.ay = low_state.imu_state().accelerometer()[1];
    feedback.imu_.az = low_state.imu_state().accelerometer()[2];

    feedback.imu_.w  = low_state.imu_state().quaternion()[0];
    feedback.imu_.x  = low_state.imu_state().quaternion()[1];
    feedback.imu_.y  = low_state.imu_state().quaternion()[2];
    feedback.imu_.z  = low_state.imu_state().quaternion()[3];

    mygamepad.vx  = -ry ;
    mygamepad.vy  = -rx;
    mygamepad.yaw = -lx;

    // printf("%8f %8f %8f\n",mygamepad.vx ,mygamepad.vy ,mygamepad.yaw);
        // outer.go;
    //arclab_controoler instanisation class
    //1. find weight document;
    //2. get all data for Moth and Go2_SDK
    controller.getInput(feedback,mygamepad);
    //3. calculate the output
    controller.calculate();
    //4. send the output to low level controller
    if (key >= 16)
    {
      Button_A_count++;
    }else
    {
      if (Button_A_count>20)
      {
        printf("from mode %2d to",mode);
        if (mode==0)
        {
          mode = 1;
        }else if(mode ==1)
        {
          mode = 2;
        }else
        {
          mode = 0;
        }
        printf(" mode %2d\n",mode);
      }
      Button_A_count = 0;
    }
    
    if (mode == 1 )
    {
      mode_1_count++;
	//   std::cout<<"flag 1"<<std::endl;
      controller.getDefault(jcmd);
      double percent;
      if (mode_1_count==1)
      {
        for(int j=0; j<12; j++) lastPos[j] = feedback.joint_.pos[j];
      }
      percent = (double)mode_1_count/2000;
      if (percent >= 1.0) { percent = 1.0; }
	  else{
		printf("standing now\n");
	  }
      // std::cout << "update" << mode_1_count << std::endl;
      for(int j=0; j<12; j++){
        jcmd.pos[j] = lastPos[j]*(1-percent) + jcmd.pos[j]*percent;
        // printf("%f ",jcmd.pos[j]);
      }
	//   std::cout<<"flag 2"<<std::endl;

      // printf("\n");
      for (size_t i = 0; i < 12; i++)
      {
        int swap_i = swapJointIndices_[i];
        // printf("%f ",jcmd.pos[i]);
		low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q()   =  jcmd.pos[swap_i];
        low_cmd.motor_cmd()[i].dq()   = 0.0;
        low_cmd.motor_cmd()[i].tau()  = 0.0;
        low_cmd.motor_cmd()[i].kp()   = 30;
        low_cmd.motor_cmd()[i].kd()   = 0.8;
      }
	  	// std::cout<<"flag 3"<<std::endl;
		low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
	  	// std::cout<<"flag 4"<<std::endl;
		
		lowcmd_publisher->Write(low_cmd);
	  	// std::cout<<"flag 5"<<std::endl;	

    }
    else if(mode==2)
    {
      controller.getOutput(jcmd);
      for (size_t i = 0; i < 12; i++)
      {
          int swap_i = swapJointIndices_[i];
          // printf("%f ",jcmd.pos[i]);
		low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q()   =  jcmd.pos[swap_i];
        low_cmd.motor_cmd()[i].dq()   = 0.0;
        low_cmd.motor_cmd()[i].tau()  = 0.0;
        low_cmd.motor_cmd()[i].kp()   = 30;
        low_cmd.motor_cmd()[i].kd()   = 0.8;
      }
		low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
		lowcmd_publisher->Write(low_cmd);
    }

    // printf("\n");
}

void Custom::HighStateHandler(const void *message)
{
	state = *(unitree_go::msg::dds_::SportModeState_ *)message;
};

void Custom::LowStateMessageHandler(const void* message)
{
	low_state = *(unitree_go::msg::dds_::LowState_*)message;
}


void on_open(client* c, websocketpp::connection_hdl hdl){
	std::string msg = "";
	msg = "video/h264;width=1280;height=720;framerate=15;codecs=avc1.42002A";
	c->send(hdl, msg, websocketpp::frame::opcode::text);
	c->get_alog().write(websocketpp::log::alevel::app, "Sent Message: " + msg);
}

void on_fail(client* c, websocketpp::connection_hdl hdl){
	c->get_alog().write(websocketpp::log::alevel::app, "Connection Failed");
}

void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg) {
	control_time = std::chrono::system_clock::now();
	c->get_alog().write(websocketpp::log::alevel::app, "Received Reply: " + msg->get_payload());
	try{
		start = std::chrono::high_resolution_clock::now();
		json jsonData = json::parse(msg->get_payload());
		globalData = jsonData;
		std::string type = jsonData["type"];

		if (type == "capture"){
			cp_control = true;
		}

		lx = jsonData["direction"]["lx"];
		ly = jsonData["direction"]["ly"];
		rx = jsonData["direction"]["rx"];
		ry = jsonData["direction"]["ry"];
		key = jsonData["direction"]["mode"];

	} catch (...){
		std::cerr << "Not JSON" << std::endl;
	}
}

void on_close(client* c, websocketpp::connection_hdl hdl) {
	c->get_alog().write(websocketpp::log::alevel::app, "Connection Closed");
}

GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data){
	global_sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
	if (global_sample != NULL){
		GstBuffer *buffer = gst_sample_get_buffer(global_sample);
		videoflag = true;
	}
	return GST_FLOW_OK;
}

std::future<void> asyncVideo(client* c, websocketpp::connection_hdl hdl) {
	return std::async(std::launch::async, [c, hdl]() {
		if (c->get_con_from_hdl(hdl)){
			start_time = std::chrono::system_clock::now();
			while (true) {
				if (videoflag){
					try {
						GstBuffer *buffer = gst_sample_get_buffer(global_sample);
						GstMapInfo map;
						if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
							c->send(hdl, map.data, map.size, websocketpp::frame::opcode::binary);
						}
					}catch (const websocketpp::exception& e) {
						std::cerr << "Err: " << e.what() << std::endl;
					}
					videoflag = false;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
	});
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
		exit(-1);
	}
	rl_controller test;
	unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
	ChannelPublisher<unitree_go::msg::dds_::WirelessController_> publisher(TOPIC_JOYSTICK);
	publisher.InitChannel();

	gst_init(&argc, &argv);
	const char *pipeline_cmd = "";
	pipeline_cmd = "udpsrc address=230.1.1.1 port=1720 multicast-iface=enp130s0 ! queue ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse config-interval=1 ! video/x-h264, stream-format=byte-stream, alignment=au ! queue leaky=2 ! appsink name=moth sync=false max-buffers=2 emit-signals=true drop=true";
	GstElement *pipeline = gst_parse_launch(pipeline_cmd, NULL);
	GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "moth");
	g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), NULL);
	gst_element_set_state(pipeline, GST_STATE_PLAYING);

	std::cout << "arg2: " << argv[2] << std::endl;
	std::ostringstream oss;
	oss << "ws://ieee-qrc.agilertc.com:8276/pang/ws/pub?channel=" << argv[2] << "&track=video&mode=bundle";
	url = oss.str();
	client c;
	try{
		c.clear_access_channels(websocketpp::log::alevel::frame_header);
		c.clear_access_channels(websocketpp::log::alevel::frame_payload);

		c.init_asio();

		c.set_open_handler(bind(&on_open, &c, ::_1));
		c.set_fail_handler(bind(&on_fail, &c, ::_1));
		c.set_message_handler(bind(&on_message, &c, ::_1, ::_2));
		c.set_close_handler(bind(&on_close, &c, ::_1));

		websocketpp::lib::error_code ec;
		client::connection_ptr con = c.get_connection(url, ec);
		std::cout << "url: " << url << std::endl;
		url.clear();
		if (!con) {
			std::cerr << "Failed to create connection" << std::endl;
		}
		try{
			c.connect(con);
		}catch (const websocketpp::exception& e){
			std::cerr << "Websocket exception: " << e.what() << std::endl;
		}catch (const std::exception& e){
			std::cerr << "standard exception: " << e.what() << std::endl;
		}

		std::thread asyncThreadVideo([&c, con]() {
			auto asyncResultVideo = asyncVideo(&c, con);
		});

		std::thread asyncC([&c]() {
			c.run();
		});

		sleep(1); // Wait for 1 second to obtain a stable state

		Custom custom(&c, con);

		sleep(1); // Wait for 1 second to obtain a stable state

		custom.GetInitState(); // Get initial position

		unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::MessageSend, &custom, &c, con));
		// here start our
		custom.createControlThread();

		while (true)
		{
			// auto end = std::chrono::high_resolution_clock::now();
			// std::chrono::duration<double> elapsed = end - start;
			// if(elapsed.count() < 1.0){
			// 	unitree_go::msg::dds_::WirelessController_ msg;
			// 	msg.lx() = lx;
			// 	msg.ly() = ly;
			// 	msg.rx() = rx;
			// 	msg.ry() = ry;
			// 	msg.keys() = key;
			// 	publisher.Write(msg);
			// 	if (control_flag == false){
			// 		control_flag = true;
			// 	}
			// 	sleep(0.1);
			// }
			// else{
			// 	if (control_flag == true){
			// 		unitree_go::msg::dds_::WirelessController_ msg;
			// 		msg.lx() = 0.0;
			// 		msg.ly() = 0.0;
			// 		msg.rx() = 0.0;
			// 		msg.ry() = 0.0;
			// 		msg.keys() = 0;
			// 		publisher.Write(msg);
			// 		control_flag = false;
			// 	}
				sleep(0.01);
			// }
		}

	} catch (const std::exception & e) {
		std::cout << e.what() << std::endl;
	} catch (websocketpp::lib::error_code e) {
		std::cout << e.message() << std::endl;
	} catch (...) {
		std::cout << "Other Exception!!" << std::endl;
	}


	return 0;
}


