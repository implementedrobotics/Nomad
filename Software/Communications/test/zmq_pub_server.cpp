#include <iostream>
#include <sstream>
#include <string.h>
#include <string>
#include <unistd.h>
#include <thread>
#include <zmq.hpp>

int main(int argc, char *argv[]) {
    // Create the ZMQ Context
    zmq::context_t context(1); // 0 for INPROC Connections, or atleast 1 for others

    // Create Socket
    zmq::socket_t publisher(context, ZMQ_PUB);

    // If using inproc you MUST bind before connecting any subs
    std::string transport("tcp://*:5555");
    // std::string transport("inproc://mytransport");
    //std::string transport("ipc:///tmp/tests/0");
    publisher.bind(transport);

    int num_message = 10000000;

    // Publish Some Test Messages
    for(int i = 0; i < num_message; ++i) {

        // Create a message
        std::stringstream s;
        s << "Hello " << i;
        auto msg = s.str();
        zmq::message_t message(msg.length());

        // Copy Intol Buffer
        memcpy(message.data(), msg.c_str(), msg.length());

        // Send It
        publisher.send(message);

        usleep(1000);
    }

}