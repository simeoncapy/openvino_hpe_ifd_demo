#pragma once

#include <iostream>
#include <string.h>
#include <zmq.h>
#include <vector>

class NepSubscriber 
{
    public:
        NepSubscriber();
        NepSubscriber(const std::string topic_, const std::string port);
        ~NepSubscriber();

        void createDirect(std::string topic, std::string port, std::string masterIp = "127.0.0.1");

        std::string explode(const std::string& s, const char& c);
        std::string s_recv(void* socket);
        std::string listen_string();

        //  Sends string as 0MQ string, as multipart non-terminal
        void close();

    private:    
        void* m_context;
        void* m_sub;
        std::string m_topic;
        bool isInitialised;
};