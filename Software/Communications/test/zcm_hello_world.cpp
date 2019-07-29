
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <zcm/zcm-cpp.hpp>
#include <Communications/Messages/msg_t.hpp>
#include <sys/types.h>
#include <iostream>
#include <pthread.h>
using namespace std;

class Handler
{
    public:
        ~Handler() {}

        void handleMessage(const zcm::ReceiveBuffer* rbuf,
                           const string& chan,
                           const msg_t *msg)
        {
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  Message   = %s\n", msg->str.c_str());
            std::cout << "PID: " << pthread_self() << std::endl;
        }
};

//zcm::ZCM zcm_context {"inproc"};
const char *CHANNEL = "EXAMPLE";

// TODO: Pass the global zmq context
void *pub_func(void *arg) {

    zcm::ZCM zcm_context {"ipc"};
    std::cout << "PUB PID: " << pthread_self() << std::endl;
    //std::cout << "PUB PID: " << gettid() << std::endl;
    msg_t my_data {};
    my_data.str = (char*)"HELLO, WORLD!\n";

    //zcm_context.start();

    while (1) {
        zcm_context.publish(CHANNEL, &my_data);
        zcm_context.publish("FOOBAR", &my_data);
        usleep(1e6);
    }

   // zcm_context.stop();

}

void *sub_func(void *arg) {

    zcm::ZCM zcm_context {"ipc"};
    std::cout << "SUB PID1: " << pthread_self() << std::endl;
    //std::cout << "SUB PID: " << gettid() << std::endl;
    Handler handlerObject;
    auto subs = zcm_context.subscribe(".*", &Handler::handleMessage, &handlerObject);

    zcm_context.run();
}

void *sub_func2(void *arg) {

zcm::ZCM zcm_context {"ipc"};
    std::cout << "SUB PID3: " << pthread_self() << std::endl;
    //std::cout << "SUB PID: " << gettid() << std::endl;
    Handler handlerObject;
    auto subs = zcm_context.subscribe(".*", &Handler::handleMessage, &handlerObject);

    zcm_context.run();
}



int main(int argc, char *argv[])
{
    std::cout << "MAIN PID: " << getpid() << std::endl;
    //if (!zcm_context.good())
   //     return 1;

    std::cout << "Starting Subscriber Thread:" << std::endl;

    pthread_t sub_thread;
    pthread_t sub_thread2;
    pthread_t pub_thread;

    if (pthread_create(&pub_thread, NULL, pub_func, NULL))
    {
        std::cout << "Error Creating Publisher Thread!" << std::endl;
        return 1;
    }

    if (pthread_create(&sub_thread, NULL, sub_func2, NULL))
    {
        std::cout << "Error Creating Subscriber Thread!" << std::endl;
        return 1;
    }

        if (pthread_create(&sub_thread2, NULL, sub_func, NULL))
    {
        std::cout << "Error Creating Subscriber2 Thread!" << std::endl;
        return 1;
    }

    pthread_join(sub_thread2, NULL);
    pthread_join(sub_thread, NULL);
    
    pthread_join(pub_thread, NULL);

    //zcm_context.unsubscribe(subs);

    return 0;
}