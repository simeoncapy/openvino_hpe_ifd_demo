#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <gflags/gflags.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "json.hpp"
#include "NepPublisher.hpp"
#include "NepSubscriber.hpp"

class ModelDemoAbstract
{
    public:
        ModelDemoAbstract(int argc, char *argv[], std::string file = "constants.conf");
        void loadConstants(std::string file);
        void modifyImage(cv::Mat &src, cv::Mat &dest);
        bool ParseAndCheckCommandLine(int argc, char *argv[]);
        void setPublisher()

        virtual void loop() = 0;
        
    private:
        std::map<std::string, std::string> constantes;
        NepPublisher pub;
        int delay;
        int flipParam;
        std::string FLAGS_m, FLAGS_d, FLAGS_pc;
};