#pragma once

#include <string>
#include <zmq.h>
#include <iostream>
#include "json.hpp"

using json = nlohmann::json;

class NepClient
{
	public:

		NepClient(std::string ip, std::string port);
    	~NepClient();


		void close(void);
		json listen_info(void);
		void send_info(json jobjec);	

	private:
		void *m_requester;
		void *m_context;


		std::string s_recv(void *socket);

};

