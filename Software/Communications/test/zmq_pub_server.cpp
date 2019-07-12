#include <iostream>
#include <sstream>
#include <string.h>
#include <string>
#include <unistd.h>
#include <thread>
#include <zmq.hpp>

#include <ctime>


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

    std::cout << "Publisher Running!" << std::endl;
    int num_message = 10000000;

    time_t elapsed = 0;
    // Publish Some Test Messages
    for(int i = 0; i < num_message; ++i) {

        auto start = std::chrono::high_resolution_clock::now();
        // Get Time Stamp:
        std::time_t result = std::time(nullptr);

        // Local Time:
        char *local_time = std::asctime(std::localtime(&result));
        local_time[strlen(local_time)-1] = '\0';

        // Create a message
        std::stringstream s;
        s << "[" << local_time <<  "]: " << "MSG: " << i;
        auto msg = s.str();
        zmq::message_t message(msg.length());

        // Copy Into Buffer
        memcpy(message.data(), msg.c_str(), msg.length());


        // Send It
        publisher.send(message);

        std::cout << "Sending: " << s.str() << std::endl;
        
        
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        //std::cout << "Loop Time: "<< microseconds;
        usleep(2000);
    }

}