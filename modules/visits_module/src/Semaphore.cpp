#ifndef semaphore_h_
#define semaphore_h_

#include <mutex>
#include <condition_variable>
#include "Semaphore.h"

Semaphore::Semaphore(){}

Semaphore::Semaphore(int count_){
    this->count = count_;
}

void Semaphore::notify(){
    std::unique_lock<std::mutex> lock(mtx);
    count++;
    cv.notify_one();
}

void Semaphore::wait(){
    std::unique_lock<std::mutex> lock(mtx);

    while(count == 0){
        cv.wait(lock);
    }
    count--;
}

int Semaphore::getCount(){
    return count;
}

void Semaphore::setCount(int count_){
    this->count = count_;
}

#endif