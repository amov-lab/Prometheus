#ifndef IMAGE2RTSP_RTSP
#define IMAGE2RTSP_RTSP

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <live555/BasicUsageEnvironment.hh>
#include <live555/GroupsockHelper.hh>
#include <live555/liveMedia.hh>

#include <image2rtsp/rosImageSource.hpp>

extern "C"{
#include <x264.h>
}

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace net{

class RosRtspServer{
public:
    RosRtspServer(const int port,const int proxyPort, const int sessionNum) :  
            m_rtpPortNum(18888), m_rtcpPortNum(m_rtpPortNum+1), m_ttl(255),
            m_rtpPayloadFormat(96), m_estimatedSessionBandwidth(500),
            m_rtspPort(port), m_rtspProxyPort(proxyPort), m_sessionNum(sessionNum){
        
        m_scheduler = std::unique_ptr<TaskScheduler>(BasicTaskScheduler::createNew());
        m_env = BasicUsageEnvironment::createNew(*m_scheduler);
        m_authDB = std::unique_ptr<UserAuthenticationDatabase>(nullptr);

        m_videoSink = std::vector<RTPSink*>(m_sessionNum, nullptr);
        m_rtcp = std::vector<RTCPInstance*>(m_sessionNum, nullptr);
    }

    ~RosRtspServer() {}

    bool Init(const std::vector<i2r::enc::SessionImageInfo>& sessionImageInfo){
        
        m_sessionImageInfo = std::vector<i2r::enc::SessionImageInfo>(sessionImageInfo);

        OutPacketBuffer::maxSize = 100000;

        // get hostname
        int cNameLen = 100;
        m_cName.resize(cNameLen + 1, 0);
        gethostname((char*)&(m_cName[0]), cNameLen);
            
        // server handle create
        m_rtspServer = RTSPServer::createNew(*m_env, m_rtspPort, m_authDB.get());
        if(m_rtspServer == nullptr){
            ROS_ERROR("Failed to create RTSP server: %s", m_env->getResultMsg());
            return false;
        }

        m_proxyRtspServer = RTSPServerWithREGISTERProxying::createNew(*m_env, m_rtspProxyPort, m_authDB.get(), NULL, 65, true, 2, NULL, NULL);
        if(m_rtspServer == nullptr){
            ROS_ERROR("Failed to create Proxy RTSP server: %s", m_env->getResultMsg());
            return false;
        }

        // x264 encoder open from RosImageSource
        m_rosImageSource = i2r::net::RosImageSource::createNew(*m_env, 0, 0, m_sessionNum);
        for(auto i = 0 ; i < m_sessionNum ; ++i){
            if(!m_rosImageSource[i]->OpenEncoder(m_sessionImageInfo[i])){
                ROS_ERROR("Failed to [%d] X264 Encoder Open", i);
                return false;
            }
        }

        return true;
    }

    void AddSession(const std::string& streamName, const int index){
        // create rtp, rtcp handle
        {
            struct in_addr destinationAddress;
            destinationAddress.s_addr = chooseRandomIPv4SSMAddress(*m_env);

            const Port rtpPort(m_rtpPortNum + (index * 2));
            const Port rtcpPort(m_rtcpPortNum + (index * 2));

            auto rtpGroupSock = new Groupsock(*m_env, destinationAddress, rtpPort, m_ttl);
            m_rtpGroupSock->multicastSendOnly();
            
            auto rtcpGroupSock =new Groupsock(*m_env, destinationAddress, rtcpPort, m_ttl);
            m_rtpGroupSock->multicastSendOnly();

            m_videoSink[index] = H264VideoRTPSink::createNew(*m_env, rtpGroupSock, m_rtpPayloadFormat);

            m_rtcp[index] =  RTCPInstance::createNew(*m_env, rtcpGroupSock, m_estimatedSessionBandwidth, &(m_cName[0]), m_videoSink[index], NULL, True);
        }

        // create media session & regist session to rtsp server
        {
            auto sms = ServerMediaSession::createNew(*m_env, streamName.c_str(), "ROS_IMAGE", "Session streamed ROS IMAGE", True );

            sms->addSubsession(PassiveServerMediaSubsession::createNew(*m_videoSink[index], m_rtcp[index]));

            auto proxySms = ProxyServerMediaSession::createNew(*m_env, m_proxyRtspServer, 
                            m_rtspServer->rtspURL(sms), streamName.c_str(), 
                            NULL, NULL, 0, 1);

            m_rtspServer->addServerMediaSession(sms);
            m_proxyRtspServer->addServerMediaSession(proxySms);
                        
            ROS_INFO("Play this stream using the Local URL %s",  m_rtspServer->rtspURL(sms));
            ROS_INFO("Play this stream using the Proxy URL %s", m_proxyRtspServer->rtspURL(proxySms));
        }
    }

    inline void Play(const int index){
        m_videoES = m_rosImageSource[index];

        m_videoSource = H264VideoStreamFramer::createNew(*m_env, m_videoES);
        
        m_videoSink[index]->startPlaying(*m_videoSource, NULL, NULL);
    }

    inline void DoEvent(){
        m_env->taskScheduler().doEventLoop();
    }

    inline void StreamImage(const uint8_t* src, const int index){
        m_rosImageSource[index]->Encode(src);
    }

private:
    // live555
    UsageEnvironment* m_env;
    std::unique_ptr<TaskScheduler> m_scheduler;
    std::unique_ptr<UserAuthenticationDatabase> m_authDB;

    RTSPServer* m_rtspServer;
    RTSPServer* m_proxyRtspServer;
    
    std::vector<unsigned char> m_cName;
    const unsigned int m_rtpPortNum;
    const unsigned int m_rtcpPortNum;
    const unsigned char m_ttl;    
    std::unique_ptr<Groupsock> m_rtpGroupSock;
    std::unique_ptr<Groupsock> m_rtcpGroupSock;
    const unsigned m_estimatedSessionBandwidth;

    std::vector<RTPSink*> m_videoSink;
    std::vector<RTCPInstance*> m_rtcp;
    ServerMediaSession* m_sms;
    FramedSource* m_videoES;
    H264VideoStreamFramer* m_videoSource;

    std::vector<i2r::net::RosImageSource*> m_rosImageSource;

    // param
    const int m_rtspPort;
    const int m_rtspProxyPort;
    const unsigned char m_rtpPayloadFormat;
    const int m_sessionNum;
    std::vector<i2r::enc::SessionImageInfo> m_sessionImageInfo;
};

} // net
} // i2r

#endif
