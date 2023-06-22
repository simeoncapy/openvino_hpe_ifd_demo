#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <zmq.hpp>
#include <iostream>
#include "json.hpp"

// for convenience
using json = nlohmann::json;

class NepPublisher 
{
  public:
  /* CONSTRUCTORS - DESTRUCTORS */
    NepPublisher();
    //NepPublisher(std::string topic, std::string port, std::string ip = "*", bool one2Many = true);
    NepPublisher(std::string topic, std::string nodeName);
	  NepPublisher(std::string topic, std::string nodeName, std::string masterIp);
	  NepPublisher(std::string topic, std::string ip, std::string port, bool one2many);

    ~NepPublisher();

  /* METHODS */
    // Create a NEP node with the parameters
    void createDirect(std::string topic, std::string port, std::string ip = "*", bool one2Many = true);
	  void createHybrid(std::string topic, std::string nodeName, std::string masterIp = "127.0.0.1");

    //  Sends string as 0MQ string, as multipart non-terminal
    void publish(json j);

    // close the connection
    void close();

 private:
    void *m_context;
    void *m_pub;
	  std::string m_topic;
    std::string m_masterIp;

    bool isInitialised;
};