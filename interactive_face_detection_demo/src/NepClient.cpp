//#include "pch.h"
#include <string>
#include <zmq.h>
#include <iostream>
#include "NepClient.hpp"
#include "json.hpp"
using json = nlohmann::json;

NepClient::NepClient(std::string ip, std::string port) 
{
	m_context = zmq_ctx_new();
	m_requester = zmq_socket(m_context, ZMQ_REQ);
	std::string endpoint = "tcp://" + ip + ":" + port;
	zmq_connect(m_requester, endpoint.c_str());
}


NepClient::~NepClient()
{
    close();
}

void NepClient::close() 
{
	zmq_close(m_requester);
	zmq_ctx_destroy(m_context);
}

json NepClient::listen_info()
{
	std::string str = s_recv(m_requester);
	auto value = json::parse(str);
	std::cout << value << std::endl;
	return json::parse(str);
}

void NepClient::send_info(json jobjec)
{
	std::string json_str = jobjec.dump();
	char *cstr = (char *)malloc(json_str.length() + 1);
	strcpy(cstr, json_str.c_str());
	zmq_send(m_requester, cstr, strlen(cstr), 0);
	free(cstr);
}


std::string NepClient::s_recv(void *socket)
{
	zmq_msg_t message;
	zmq_msg_init(&message);
	int size = zmq_msg_recv(&message, socket, 0);
	if (size == -1)
		return NULL;
	char *str = (char *)malloc(size + 1);
	memcpy(str, zmq_msg_data(&message), size);
	zmq_msg_close(&message);
	str[size] = 0;
	std::string msg = str;
	free(str);
	return msg;
}
