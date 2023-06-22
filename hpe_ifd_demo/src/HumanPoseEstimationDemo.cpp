#include "HumanPoseEstimationDemo.hpp"

HumanPoseEstimationDemo::HumanPoseEstimationDemo(int argc, char *argv[], std::string file) : 
    ModelDemoAbstract::ModelDemoAbstract(argc, argv, string)
{
    this->delay = 33;

    std::cout << "InferenceEngine (HPE): " << GetInferenceEngineVersion() << std::endl;

    estimator(FLAGS_m, FLAGS_d, FLAGS_pc);
}