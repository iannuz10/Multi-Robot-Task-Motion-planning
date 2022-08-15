#include <mutex>
#include <condition_variable>

class Semaphore {
public:
    Semaphore (int count_);

    inline void notify();
    inline void wait();

private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};