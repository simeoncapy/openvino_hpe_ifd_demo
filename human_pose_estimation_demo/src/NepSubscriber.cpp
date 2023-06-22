#include "NepSubscriber.hpp"
#include "NepClient.hpp"

NepSubscriber::NepSubscriber()
{
    isInitialised = false;
}

NepSubscriber::NepSubscriber(const std::string topic_, const std::string port)
{
    createDirect(topic_, port);
}

NepSubscriber::~NepSubscriber()
{
    close();
}

void NepSubscriber::createDirect(std::string topic, std::string port, std::string masterIp)
{
    m_context = zmq_ctx_new();
    m_sub = zmq_socket(m_context, ZMQ_SUB);
    std::string ip = "tcp://" + masterIp + ":" + port;
    char* cstr = new char[ip.length() + 1];
    strcpy(cstr, ip.c_str());

    char* ttopic = new char[topic.length() + 1];
    strcpy(ttopic, topic.c_str());


    zmq_connect(m_sub, cstr);
    zmq_setsockopt(m_sub, ZMQ_SUBSCRIBE, ttopic, 1);
    int conflate = 1;
    zmq_setsockopt(m_sub, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    m_topic = topic;
    isInitialised = true;
}

std::string NepSubscriber::explode(const std::string& s, const char& c)
{
    std::string buff = ("");
    std::vector<std::string> messages;

    for (auto n : s)
    {
        if (n != c)
            buff += n;
        else if (n == c && buff != "")
        {
            messages.push_back(buff);
            buff = "";
        }
    }
    if (buff != "")
        messages.push_back(buff);
    return messages[1];
}

std::string NepSubscriber::s_recv(void* socket)
{
    zmq_msg_t message;
    zmq_msg_init(&message);
    int size = zmq_msg_recv(&message, socket, 0);
    if (size == -1)
        return NULL;
    char* str = (char*)malloc(size + 1);
    memcpy(str, zmq_msg_data(&message), size);
    zmq_msg_close(&message);
    str[size] = 0;
    std::string msg = explode(str, ' ');
    free(str);
    return msg;
}

std::string NepSubscriber::listen_string()
{
    return s_recv(m_sub);
}

//  Sends string as 0MQ string, as multipart non-terminal
void NepSubscriber::close()
{
    zmq_close(m_sub);
    zmq_ctx_destroy(m_context);
}