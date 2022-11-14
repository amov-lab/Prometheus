#ifndef IMAGE2RTSP_ROSIMAGESOURCE
#define IMAGE2RTSP_ROSIMAGESOURCE

#include <live555/FramedSource.hh>

extern "C"{
#include <x264.h>
}

#include <image2rtsp/buffer.hpp>

namespace i2r{
namespace net{

class RosImageSource : public FramedSource{
public:
    static std::vector<RosImageSource*> createNew(UsageEnvironment& env, unsigned preferredFrameSize, unsigned playTimePerFrame, const int sessionNum){

        std::vector<RosImageSource*> rosImageSourceArray;
        for(auto i = 0 ; i < sessionNum ; ++i)
            rosImageSourceArray.push_back(new RosImageSource(env, preferredFrameSize, playTimePerFrame));

        return rosImageSourceArray;
    }
    
    bool OpenEncoder(const i2r::enc::SessionImageInfo& imageInfo){   
        if(!m_encoder.Open(imageInfo))
            return false;
        else
            return true;
    }

    void Encode(const uint8_t* src){
        m_encoder.Encoding(src);

        envir().taskScheduler().triggerEvent(m_eventTriggerId, this);
    }

protected:
    RosImageSource(UsageEnvironment& env, unsigned preferredFrameSize, unsigned playTimePerFrame) 
        :   FramedSource(env),
            fPreferredFrameSize(fMaxSize),
            fPlayTimePerFrame(playTimePerFrame),
            fLastPlayTime(0),
            m_encoder(i2r::enc::Encoder(m_buffer)),
            m_buffer() {
                
        ++m_referenceCount;
        
        if (m_eventTriggerId == 0) {
            m_eventTriggerId = envir().taskScheduler().createEventTrigger(deliverFrame0);
        }
    }
    
    virtual ~RosImageSource(void){
        --m_referenceCount;
        envir().taskScheduler().deleteEventTrigger(m_eventTriggerId);
        m_eventTriggerId = 0;
    }

private:
    virtual void doGetNextFrame(){
        deliverFrame();    
    }
    
    static void deliverFrame0(void* clientData){
        ((RosImageSource*)clientData)->deliverFrame();
    }
    
    void deliverFrame(){
        if(!isCurrentlyAwaitingData()) return;
              
        if (fPlayTimePerFrame > 0 && fPreferredFrameSize > 0) {
            if (fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0) {
                gettimeofday(&fPresentationTime, NULL);
            } 
            else {
                unsigned uSeconds	= fPresentationTime.tv_usec + fLastPlayTime;
                fPresentationTime.tv_sec += uSeconds / 1000000;
                fPresentationTime.tv_usec = uSeconds % 1000000;
            }

            fLastPlayTime = (fPlayTimePerFrame*fFrameSize)/fPreferredFrameSize;
            fDurationInMicroseconds = fLastPlayTime;
        } 
        else {
            gettimeofday(&fPresentationTime, NULL);
        }
        
        if(!m_buffer.Empty()){
            m_buffer.Pop(m_nalToDeliver);
        
            unsigned newFrameSize = m_nalToDeliver.i_payload;

            if (newFrameSize > fMaxSize) {
                fFrameSize = fMaxSize;
                fNumTruncatedBytes = newFrameSize - fMaxSize;
            }
            else {
                fFrameSize = newFrameSize;
            }
            
            auto res = std::copy(m_nalToDeliver.p_payload, m_nalToDeliver.p_payload + m_nalToDeliver.i_payload, fTo);    
            
            FramedSource::afterGetting(this);
        }
    }

private:
    unsigned fPreferredFrameSize;
    unsigned fPlayTimePerFrame;
    unsigned fNumSources;
    unsigned fCurrentlyReadSourceNumber;
    unsigned fLastPlayTime;

    i2r::enc::Encoder m_encoder;

    x264_nal_t m_nalToDeliver;
    i2r::util::Buffer<x264_nal_t> m_buffer;

    unsigned m_referenceCount;
    timeval m_currentTime;

    EventTriggerId m_eventTriggerId;
};

} // net
} // i2r

#endif