#include "NepPublisher.hpp"
#include "NepClient.hpp"

NepPublisher::NepPublisher()
{
    isInitialised = false;
}

NepPublisher::NepPublisher(std::string topic, std::string nodeName)
{
	createHybrid(topic, nodeName, "127.0.0.1");
}

NepPublisher::NepPublisher(std::string topic, std::string nodeName, std::string masterIp)
{
    createHybrid(topic, nodeName, masterIp);
}


NepPublisher::NepPublisher(std::string topic, std::string ip, std::string port, bool one2Many)
{
	createDirect(topic, port, ip, one2Many);
}

NepPublisher::~NepPublisher()
{
    close();
}

// METHODS

void NepPublisher::createDirect(std::string topic, std::string port, std::string ip /*= "*"*/, bool one2Many /*= true*/)
{
    m_context = zmq_ctx_new();
    m_pub = zmq_socket(m_context, ZMQ_PUB);
    std::string url = "tcp://" + ip + ":" + port;
    char *cstr = new char[url.length() + 1];
    strcpy(cstr, url.c_str());
    m_topic = topic;
    if (one2Many)
    {
        zmq_bind(m_pub, cstr);
        std::cout << "PUB: direct one2many " << m_topic << " bind " << url << std::endl;
    }
    else
    {
        zmq_connect(m_pub, cstr);
        std::cout << "PUB: direct many2one" << m_topic << " connect " << url << std::endl;
    }
    isInitialised = true;    
}

void NepPublisher::createHybrid(std::string topic, std::string node_name, std::string masterIp)
{
    m_topic = topic;
	m_masterIp = masterIp;
	m_context = zmq_ctx_new();
	m_pub = zmq_socket(m_context, ZMQ_PUB);

	std::cout << "PUB: " << topic << " waiting NEP master ..." << std::endl;
	NepClient client(masterIp, "7000");
	json message = { {"node", node_name.c_str()} , {"topic", topic.c_str()}, {"mode", "many2many"}, {"socket", "publisher"}, {"pid", "none"}, {"msg_type", "json"}, {"language", "C++"} };

	client.send_info(message);
	json response = client.listen_info();

	std::string ip = m_masterIp;
	std::string port = response["port"];

	std::string url = "tcp://" + ip + ":" + port;
	char *cstr = new char[url.length() + 1];

	strcpy(cstr, url.c_str());
	zmq_connect(m_pub, cstr);
	std::cout << "PUB: " << topic << " connect " << url << std::endl;
    isInitialised = true;
}

//  Sends string as 0MQ string, as multipart non-terminal
void NepPublisher::publish(json j)
{
    if(!isInitialised)
        throw "The connection has not been initialised, impossible to send any messages.";
    std::string json_str = j.dump();
    std::string str = m_topic + " " + json_str;
    char *cstr = (char *)malloc(str.length() + 1);
    strcpy(cstr, str.c_str());
    zmq_send(m_pub, cstr, strlen(cstr), 0);
    free(cstr);
}

//  Sends string as 0MQ string, as multipart non-terminal
void NepPublisher::close()
{
    zmq_close(m_pub);
    zmq_ctx_destroy(m_context);
    m_topic = "";
    m_masterIp = "";
    isInitialised = false;
}

