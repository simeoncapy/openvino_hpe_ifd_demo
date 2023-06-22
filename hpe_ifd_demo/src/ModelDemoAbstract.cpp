#include "ModelDemoAbstract.hpp"

ModelDemoAbstract::ModelDemoAbstract(int argc, char *argv[], std::string file)
{
    this->loadConstants(file)
    setPublisher()
    if (!ParseAndCheckCommandLine(argc, argv))
        return EXIT_SUCCESS;
}

void ModelDemoAbstract::loadConstants(std::string file)
{
    std::ifstream myfile;
    std::string line;
    myfile.open(file);
    char delimiter = '=';

    if (myfile.is_open())
    {
        while (std::getline(myfile,line))
        {
            std::istringstream tokenStream(line);
            std::string key, value;
            std::getline(tokenStream, key, delimiter);
            std::getline(tokenStream, value, delimiter);
            constantes[key] = value;
        }
        myfile.close();
    }
    else
        std::cout << "Cannot open the constants file" << std::endl;
}

void ModelDemoAbstract::modifyImage(cv::Mat &src, cv::Mat &dest)
{
    //cv::resize(src, dest, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
    if(flipParam >= 0)
        cv::rotate(src, dest, flipParam);
    else
        dest = src;    
}

bool ModelDemoAbstract::ParseAndCheckCommandLine(int argc, char *argv[]) {
    // ---------------------------Parsing and validating input arguments--------------------------------------
    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        showAvailableDevices();
        return false;
    }
    slog::info << "Parsing input parameters" << slog::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m.empty()) {
        throw std::logic_error("Parameter -m is not set");
    }

    if (FLAGS_n_ag < 1) {
        throw std::logic_error("Parameter -n_ag cannot be 0");
    }

    if (FLAGS_n_hp < 1) {
        throw std::logic_error("Parameter -n_hp cannot be 0");
    }

    // no need to wait for a key press from a user if an output image/video file is not shown.
    FLAGS_no_wait |= FLAGS_no_show;

    if (FLAGS_f == "90")
        flipParam = cv::ROTATE_90_CLOCKWISE;
    else if (FLAGS_f == "180")
        flipParam = cv::ROTATE_180;
    else if (FLAGS_f == "270")
        flipParam = cv::ROTATE_90_COUNTERCLOCKWISE;
    else
        flipParam = -1;

    

    return true;
}

void ModelDemoAbstract::setPublisher()
{
   if(constantes["IS_2_PI"] == "True")
    {
        pub.createHybrid(constantes["NEP_TOPIC"], "cppnodepub", constantes["IP_OTHER"]);
    }
    else
    {
        pub.createHybrid(constantes["NEP_TOPIC"], "cppnodepub");
    } 
}