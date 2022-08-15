#include <mutex>
#include <condition_variable>

class Semaphore {
public:
    Semaphore ();
    Semaphore (int count_);

    void notify();
    void wait();
    int getCount();
    void setCount(int count_);

private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};