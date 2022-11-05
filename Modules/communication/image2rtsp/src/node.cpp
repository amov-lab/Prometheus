#include <iostream>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <image2rtsp/encode.hpp>
#include <image2rtsp/rtspServer.hpp>
#include <image2rtsp/buffer.hpp>

class Image2RTSP_sample{
public:
	Image2RTSP_sample() {
		m_nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

		m_nh->getParam("port", m_serverPort);
		m_nh->getParam("proxyServerPort", m_proxyServerPort);
		m_nh->getParam("sessionNumber", m_sessionNumber);

		{
			m_sessionImageInfo = std::vector<i2r::enc::SessionImageInfo>(m_sessionNumber);
			m_streamUrl = std::vector<std::string>(m_sessionNumber);
			m_subTopic = std::vector<std::string>(m_sessionNumber);
		}

		for(auto i = 0 ; i < m_sessionNumber ; ++i){
			std::string index = std::to_string(i);
			
			m_nh->getParam("subTopic_" 	+ index, m_subTopic[i]);
			m_nh->getParam("srcWidth_" 	+ index, m_sessionImageInfo[i].srcWidth);
			m_nh->getParam("srcHeight_" + index, m_sessionImageInfo[i].srcHeight);
			m_nh->getParam("encode_" 	+ index, m_sessionImageInfo[i].fomat);
        	m_nh->getParam("fps_" 		+ index, m_sessionImageInfo[i].fps);
			m_nh->getParam("streamUrl_" + index, m_streamUrl[i]);
		}
	}

	~Image2RTSP_sample() {
		if(m_rtspServer.joinable())
			m_rtspServer.join();
	}

	bool Run(){
		// initialize x264 handle & rtsp stream
		{	
			m_rtsp = std::unique_ptr<i2r::net::RosRtspServer>(new i2r::net::RosRtspServer(m_serverPort, m_proxyServerPort, m_sessionNumber));

			if(!m_rtsp->Init(m_sessionImageInfo))
				return false;

			for(auto i = 0 ; i < m_sessionNumber ; ++i){
				m_rtsp->AddSession(m_streamUrl[i], i);
			}
		}

		// regist ros subscribe callback
		// regist your callback as many as your sessions
		{
			m_sub.push_back(m_nh->subscribe(m_subTopic[0], 10, &Image2RTSP_sample::Session_0_callback, this));
			m_sub.push_back(m_nh->subscribe(m_subTopic[1], 10, &Image2RTSP_sample::Session_1_callback, this));
		}
		
		// run media & server thread
		{	
			for(auto i = 0 ; i < m_sessionNumber ; ++i)
				m_rtsp->Play(i);

			m_rtspServer = std::thread(&Image2RTSP_sample::ServerRun, this);
		}

		return true;
	}

	void Join(){
		if(m_rtspServer.joinable())
			m_rtspServer.join();
	}

private:
    // write your callback as many as your sessions
	void Session_0_callback(const sensor_msgs::Image::ConstPtr &msg){
		m_rtsp->StreamImage(&(msg->data[0]), 0);
	}

	void Session_1_callback(const sensor_msgs::Image::ConstPtr &msg){
		m_rtsp->StreamImage(&(msg->data[0]), 1);
	}

	void ServerRun(){
		m_rtsp->DoEvent();
	}

private:
    // ros
	std::unique_ptr<ros::NodeHandle> m_nh;
	std::vector<ros::Subscriber> m_sub;
    
    // encoder
	std::unique_ptr<i2r::net::RosRtspServer> m_rtsp;

	//thread
	std::thread m_rtspServer;
    
	// param
	int m_serverPort;
	int m_proxyServerPort;
	int m_sessionNumber;

	std::vector<i2r::enc::SessionImageInfo> m_sessionImageInfo;
	std::vector<std::string> m_streamUrl;
	std::vector<std::string> m_subTopic;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "image2rtsp");

	Image2RTSP_sample sample;
	
	if(!sample.Run())
		ros::shutdown();

	ros::spin();
	
	ros::shutdown();
}