
// #include <stdio.h>
// #include <unistd.h>
// #include <string>
// #include <zcm/zcm-cpp.hpp>
// #include <Communications/Messages/msg_t.hpp>

// #include <iostream>

// using namespace std;

// class Handler
// {
//     public:
//         ~Handler() {}

//         void handleMessage(const zcm::ReceiveBuffer* rbuf,
//                            const string& chan,
//                            const msg_t *msg)
//         {
//             printf("Received message on channel \"%s\":\n", chan.c_str());
//             printf("  Message   = %s\n", msg->str.c_str());
//         }
// };

// void *sub_func(void *arg)
// {

// printf("IN SUB\n");
//     zcm::ZCM zcm {"nonblock-inproc"};
//     if (!zcm.good())
//     {
//         printf("NOT GOOD\n");
//         return NULL;
//     }
        

//     Handler handlerObject;
//     zcm.subscribe("CHANNEL", &Handler::handleMessage, &handlerObject);

//     printf("Start RUN SUB\n");
//     while(1)
//     {
//         int test = zcm.handleNonblock();
//         std::cout << "Handle: " << test << std::endl;
//         usleep(1e6);
//     }

//     printf("HEre\n");
//     //    zmq::context_t *ctx = (zmq::context_t *)arg;

//     //    zmq::socket_t subscriber(*ctx, ZMQ_SUB);
//     //    std::string transport("inproc://mytransport");

//     //   subscriber.setsockopt(ZMQ_CONFLATE, 1);

//     //   // 2. we need to connect to the transport
//     //   subscriber.connect(transport);
//     //   // 3. set the socket options such that we receive all messages. we can set
//     //   // filters here. this "filter" ("" and 0) subscribes to all messages.
//     //   subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

//     //   int num_rx = 0;
//     //   std::cout << "Subscriber Running" << std::endl;
//     // while(true) {

//     //         zmq::message_t rx_msg;

//     //         subscriber.recv(&rx_msg);
//     //         std::string rx_str;
//     //         rx_str.assign(static_cast<char *>(rx_msg.data()), rx_msg.size());
//     //         std::cout << "Received: " << rx_str << " : " << num_rx << std::endl;
//     //         ++num_rx;

//     //         usleep(50000);
//     //     }
// }

// // TODO: Pass the global zmq context
// void *pub_func(void *arg)
// {
//     printf("HERE\n");
//     zcm::ZCM zcm{"nonblock-inproc"};
//     if (!zcm.good())
//     {
//         printf("NOT GOOD\n");
//         return NULL;
//     }
        

//     msg_t msg;
//     msg.str = (char*)"Hello, World!";

//     while(1) {
//         printf("PUBLISHING!\n");
//         zcm.publish("CHANNEL", &msg);
//         usleep(1e6);

//     }
//     //   zmq::context_t *ctx = (zmq::context_t *)arg;

//     //   zmq::socket_t publisher(*ctx, ZMQ_PUB);
//     //   std::string transport("inproc://mytransport");
//     //   publisher.bind(transport);

//     //   std::cout << "Publisher Running!" << std::endl;

//     //   for (size_t i = 0; i < 1000000; ++i) {
//     //     // create a message
//     //     std::stringstream s;
//     //     s << "Hello " << i;
//     //     auto msg = s.str();
//     //     zmq::message_t message(msg.length());
//     //     memcpy(message.data(), msg.c_str(), msg.length());
//     //     publisher.send(message);

//     //     usleep(5000);
//     //   }
// }

// int main(int argc, char *argv[])
// {

//     std::cout << "Starting Publisher Thread:" << std::endl;

//     pthread_t sub_thread;
//    // pthread_t pub_thread;

//     // if (pthread_create(&pub_thread, NULL, pub_func, NULL))
//     // {
//     //     std::cout << "Error Creating Publisher Thread!" << std::endl;
//     //     return 1;
//     // }

//      std::cout << "Starting Subscriber Thread:" << std::endl;
//     if (pthread_create(&sub_thread, NULL, sub_func, NULL))
//     {
//         std::cout << "Error Creating Subscriber Thread!" << std::endl;
//         return 1;
//     }

//    // pthread_join(pub_thread, NULL);
//   //  printf("JOINING\n");

//         printf("HERE\n");
//     zcm::ZCM zcm{"inproc"};
//     if (!zcm.good())
//     {
//         printf("NOT GOOD\n");
//         return 1;
//     }
        

//     msg_t msg;
//     msg.str = (char*)"Hello, World!";

//     while(1) {
//         printf("PUBLISHING!\n");
//         zcm.publish("CHANNEL", &msg);
//         usleep(1e6);

//     }

//     pthread_join(sub_thread, NULL);
//     return 0;

//     //   zmq::context_t context(0);
//     //   std::cout << "Starting Subscriber Thread:" << std::endl;

//     //   pthread_t sub_thread;
//     //   pthread_t pub_thread;

//     //   if(pthread_create(&pub_thread, NULL, pub_func, &context)) {
//     //     std::cout << "Error Creating Publisher Thread!" << std::endl;
//     //     return 1;
//     //   }

//     //   if(pthread_create(&sub_thread, NULL, sub_func, &context)) {
//     //     std::cout << "Error Creating Subscriber Thread!" << std::endl;
//     //     return 1;
//     //   }

//     //   pthread_join(sub_thread, NULL);
//     //   pthread_join(pub_thread, NULL);
//     //   return 0;

//     std::cout << "Hello in Main" << std::endl;
//     return 0;
// }


#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <zcm/zcm.h>
#include <zcm/transport.h>
#include <zcm/transport_registrar.h>
#include <Communications/Messages/msg_t.h>

static void my_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const msg_t *msg, void *user)
{
    /* /\* if (quiet) */
    /*     return; */

    printf("Received message on channel \"%s\":\n", channel);
    // printf("  timestamp   = %"PRId64"\n", msg->timestamp);
    // printf("  position    = (%f, %f, %f)\n",
    //         msg->position[0], msg->position[1], msg->position[2]);
    // printf("  orientation = (%f, %f, %f, %f)\n",
    //         msg->orientation[0], msg->orientation[1], msg->orientation[2],
    //         msg->orientation[3]);
    // printf("  ranges:");
    // for(int i = 0; i < msg->num_ranges; i++)
    //     printf(" %d", msg->ranges[i]);
    // printf("\n");
    // printf("  name        = '%s'\n", msg->name);
    // printf("  enabled     = %d\n", msg->enabled);
}

int main(int argc, char *argv[])
{
    if (argc > 1) {
        if (strcmp(argv[1], "-h") == 0) {
            zcm_transport_help(stdout);
            return 0;
        }
    }

    zcm_t zcm;
    if (zcm_init(&zcm, "nonblock-inproc") != 0) return 1;

    msg_t_subscription_t* exSub = msg_t_subscribe(&zcm, "EXAMPLE", &my_handler, NULL);


     msg_t msg;
     msg.str = (char*)"Hello, World!";


    // msg_t my_data = {};

    // my_data.timestamp = 0,
    // my_data.position[0] = 1;
    // my_data.position[1] = 2;
    // my_data.position[2] = 3;
    // my_data.orientation[0] = 1;
    // my_data.orientation[1] = 0;
    // my_data.orientation[2] = 0;
    // my_data.orientation[3] = 0;

    // my_data.num_ranges = 15;
    // my_data.ranges = calloc(my_data.num_ranges, sizeof(int16_t));
    // my_data.name = "example string";
    // my_data.enabled = 1;

    size_t i;
    for (i = 0; i < 10; ++i) {
        //my_data.timestamp = i;
        msg_t_publish(&zcm, "EXAMPLE", &msg);
        usleep(100000);
        zcm_handle_nonblock(&zcm);
    }

   // free(my_data.ranges);

    msg_t_unsubscribe(&zcm, exSub);

    zcm_cleanup(&zcm);
    return 0;
}