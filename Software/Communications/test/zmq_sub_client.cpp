#include <iostream>
#include <sstream>
#include <string.h>
#include <string>
#include <unistd.h>
#include <thread>
#include <zmq.hpp>

int main(int argc, char *argv[]) {
    // Create Context
    zmq::context_t context(1);

    // Create Subscriber
    zmq::socket_t subscriber(context, ZMQ_SUB);
    //zmq::socket_t subscriber(context, ZMQ_DISH);
    subscriber.setsockopt(ZMQ_CONFLATE, 1);

    // Create Transport
    //std::string transport("udp://localhost:5555");
    std::string transport("tcp://localhost:5555");
    //std::string transport(inproc://mytransport);
    //std::string transport("ipc:///tmp/tests/0");

    // Connect To Publisher
    subscriber.connect(transport);

    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    int num_rx = 0;

    while(true) {
        zmq::message_t rx_msg;

        subscriber.recv(&rx_msg);
        std::string rx_str;
        rx_str.assign(static_cast<char *>(rx_msg.data()), rx_msg.size());
        std::cout << "Received: " << rx_str << " : " << num_rx << std::endl;
        ++num_rx;

        usleep(2000000);
    }


}