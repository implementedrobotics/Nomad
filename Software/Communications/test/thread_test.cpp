#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <iostream>

using namespace std::chrono_literals;

std::queue<int> q[5];
std::mutex m[5];
std::mutex m2;
std::condition_variable cv;
auto timeout = std::chrono::milliseconds(1000);
void read_task(int id)
{
    std::unique_lock<std::mutex> lock(m2);
    
    while(1)
    {
        bool got = cv.wait_for(lock, 1000ms, [&] {return ::q[id].size() >0; });
        std::cout << "GOT DATA: " << got << std::endl;
        if(got)
        {
            int i = q[id].front();
            q[id].pop();
            std::cout << "i : " << i << std::endl;
        }
        //int i = q[id].front();
        //std::cout << "[ " << id << "]" << "i: " << i << " TIMEOUT: " << got << std::endl;
        //q[id].pop();
        //lock.unlock();
        //if(i == 99)
          //  return;

        //lock.lock();
    
    }
}

int main()
{
    std::thread t1(read_task, 0);
    //std::thread t2(read_task, 1);


    std::cout << " SHOULD GET 6 TIMEOUTS " << std::endl;
    std::this_thread::sleep_for(6*1000ms);
    m2.lock();
    q[0].push(10);
    m2.unlock();


    std::this_thread::sleep_for(1500ms);
    m2.lock();
    q[0].push(100);
    m2.unlock();

    for(int i = 0; i < 100; i++)
    {
        std::this_thread::sleep_for(500ms);
        m2.lock();
        q[0].push(i+100);
        m2.unlock();
        cv.notify_all();
    }

    // for(int i = 0; i < 100; i++)
    // {
    //     m2.lock();
    //     q[0].push(i);
    //     //m[0].unlock();

    //     //m[1].lock();
    //     q[1].push(i);
    //     m2.unlock();
    //     cv.notify_all();
    // }
    t1.join();
   // t2.join();
    return 0;
}
