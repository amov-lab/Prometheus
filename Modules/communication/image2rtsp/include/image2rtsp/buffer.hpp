#ifndef IMAGE2RTSP_BUFFER
#define IMAGE2RTSP_BUFFER

#include <queue>
#include <mutex>
#include <condition_variable>

namespace i2r{
namespace util{

template<typename Data>
class Buffer{
public:
    Buffer() : m_mutex(), m_cv() {};

    virtual ~Buffer() {};

    void Push(const Data& data){
        std::unique_lock<std::mutex> lock(m_mutex);

        m_queue.push(data);
        
        m_cv.notify_one();
    }

    bool Empty() {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    void Pop(Data& data){
        std::unique_lock<std::mutex> lock(m_mutex);
        
        while(m_queue.empty()){
            m_cv.wait(lock);
        }
                
        data = m_queue.front();
        m_queue.pop();
    }

    const size_t Size(){
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    
private:
    std::queue<Data> m_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

} // util
} // i2r

#endif
