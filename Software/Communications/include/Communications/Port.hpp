/*
 * Port.hpp
 *
 *  Created on: July 25, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef NOMAD_REALTIME_PORT_H_
#define NOMAD_REALTIME_PORT_H_

// C Includes

// C++ Includes
#include <deque>
#include <memory>
#include <map>
#include <mutex>
#include <queue>
#include <chrono>
#include <condition_variable>
#include <iostream>

// Project Includes
#include <Common/Time.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Communications/Messages/msg_helpers.hpp>

// Third Party Includes
#include <zcm/zcm-cpp.hpp>


template <typename T>
struct conv;

namespace Communications
{
    class PortInterface
    {

    public:
        // Transport Type Enum
        enum TransportType
        {
            INPROC = 0,
            IPC,
            UDP,
            NATIVE,
            SERIAL
        };

        // Data Type Enum
        enum DataType
        {
            BYTE = 0,
            INT8,
            INT16,
            INT32,
            INT64,
            FLOAT,
            DOUBLE
        };

        // Port Type Enum
        enum Direction
        {
            INPUT = 0,
            OUTPUT
        };

        PortInterface(const std::string &name, Direction direction, DataType data_type, int dimension, int period);
        PortInterface(const std::string &name, Direction direction, int period);
        virtual ~PortInterface() {}

        // Connect
        virtual bool Connect(){return false;}// = 0;

        // Bind Port
        virtual bool Bind(){return false;}// = 0;

        // Map
        virtual bool Map(std::shared_ptr<PortInterface> output){return false;}// = 0;

        // Map Ports
        static bool Map(std::shared_ptr<PortInterface> input, std::shared_ptr<PortInterface> output);

        // Send message type data on port
        template <typename PortType>
        bool Send(PortType &msg);

        // Receive message type data on port
        template <typename PortType>
        bool Receive(PortType &msg, std::chrono::microseconds timeout = std::chrono::microseconds(0));

        // Demux/Mux Helpers
        virtual std::vector<std::shared_ptr<PortInterface>> Demux()
        {
            std::cout << "Demux Main" << std::endl;
            return std::vector<std::shared_ptr<PortInterface>>();
        }

        // // Map Ports
        // static bool Map(std::shared_ptr<PortInterface> input, std::shared_ptr<PortInterface> output);

        // Transport
        // For INPROC/IPC transport URL should depend on block/noblock?  Not technically necessary to set.
        void SetTransport(const TransportType transport, const std::string &transport_url, const std::string &channel)
        {
            transport_type_ = transport;
            transport_url_ = transport_url;
            channel_ = channel;
        }

        // Signal Labels
        void SetSignalLabel(const int signal_idx, const std::string &label);

        PortInterface::DataType GetDataType();
        int GetDimension() { return dimension_; }

        const std::string &GetName() const { return name_; }

        static const int MAX_LISTENERS = 16;

    protected:
        // Port Name
        std::string name_;

        // Update Period
        unsigned int update_period_;

        // Size of the queue bufferr
        int queue_size_;

        // Port Dimension
        int dimension_;

        // Port Type
       // DataType data_type_;

        // Port Direction
        Direction direction_;

        // Port Labels
        std::map<int, std::string> signal_labels_;

        // Transport Type
        TransportType transport_type_;

        // Transport URL
        std::string transport_url_;

        // Channel
        std::string channel_;

        // Using ZeroCM for thread sync and message passing
        // Context
        std::shared_ptr<zcm::ZCM> context_;

        // Has port been connected?
        bool started_;

        // Sequence Number:
        uint64_t sequence_num_;

        // Pointer to Handler
        void *handler_;

        // Thread Synchronization and Handling for Native Communications
        // Current Listener Ports
        std::vector<std::shared_ptr<PortInterface>> listeners_;
    };

    template <typename PortType = double_vec_t>
    class Port : public PortInterface
    {

    public:
        Port(const std::string &name, Direction direction, DataType data_type, int dimension, int period);
        Port(const std::string &name, Direction direction, int period);
        ~Port();

        // Port Type
        typedef PortType port_type_t;

        static std::shared_ptr<PortInterface> CreateInput(const std::string &name, int period = -1);
        static std::shared_ptr<PortInterface> CreateOutput(const std::string &name, int period = -1);

        // Connect Port
        bool Connect();

        // Bind Port
        bool Bind();

        // Map Input Output
        virtual bool Map(std::shared_ptr<PortInterface> output);

        // auto getSelf() -> Port<PortType> & override
        // {
        //     std::cout << "SUB" << typeid(port_type_t).name() << std::endl;

        //     return *this;
        // }

        virtual std::vector<std::shared_ptr<PortInterface>> Demux()
        {
            std::cout << "Demux Sub" << std::endl;
            conv<port_type_t>::Demux();
            return std::vector<std::shared_ptr<PortInterface>>();
        }

    // auto getType() -> port_type_t& override {
    //     port_type_t var;
    //     return var;
    // }


    protected:


    private:

        void _CreateHandler();

    };

    template <class T>
    class PortHandler
    {
        friend class PortInterface;

    public:
        // Base class for RealTimeTaskNode PortHandler
        // name = Port Name
        // ctx = ZCM Context
        // transport = ZCM Message Transport Location String
        // period = Update period (Does not matter for Input Ports)
        PortHandler(int queue_size = 20);
        ~PortHandler();

        // Message Handling Callback
        void HandleMessage(const zcm::ReceiveBuffer *rbuf,
                           const std::string &chan,
                           const T *msg);

    protected:
        // Get Message Buffer
        inline std::deque<T> &GetMessageQueue() { return msg_buffer_; }

        // Read Available Messages
        // TODO: Read Backward In Time
        const inline bool Read(T &rx_msg, std::chrono::duration<double> timeout = std::chrono::milliseconds(0));

        void HandleMessage(T &rx_msg);

        // Condition Signaling Variable for Thread Comms Sync
        std::condition_variable cond_;

        // Message Buffer
        std::deque<T> msg_buffer_;

        T data_;

        // Thread mutex
        std::mutex mutex_;

        // Queue Size to Buffer
        int queue_size_;

        // Number of unread messages on port
        int num_unread_;
    };

    class PortManager
    {

    public:
        // Base Class Real Time Task Manager
        PortManager();

        // STATIC Singleton Instance
        static PortManager *Instance();

        // Singleton ZCM Context for INPROC messaging
        std::shared_ptr<zcm::ZCM> GetInprocContext() const { return inproc_context_; }

    protected:
        // Using ZMQ for thread sync and message passing
        // ZMQ Context
        // Thread inproc messaging must share a single context.  We put it in the singleton here to keep it unique;
        //zcm::ZCM *inproc_context_;

        std::shared_ptr<zcm::ZCM> inproc_context_;

    private:
        // Singleton Instance
        static PortManager *manager_instance_;
    };

} // namespace Communications

#include <Communications/Port-impl.hpp>
#endif // NOMAD_REALTIME_PORT_H_
