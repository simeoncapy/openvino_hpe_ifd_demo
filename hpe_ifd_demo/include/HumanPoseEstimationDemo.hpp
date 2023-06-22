#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <pthread.h>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>

#include <zmq.hpp>

#include <inference_engine.hpp>

#include <monitors/presenter.h>
#include <samples/ocv_common.hpp>

#include "human_pose_estimation_demo.hpp"
#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "json.hpp"
#include "NepPublisher.hpp"
#include "NepSubscriber.hpp"
#include "ModelDemoAbstract.hpp"

using namespace InferenceEngine;
using namespace human_pose_estimation;
using json = nlohmann::json;

const std::string NEP_TOPIC             = "body_position";
const std::string NEP_DATA              = "bodyPositions";
const std::string NEP_DATA_PERSPECTIVE  = "PERSPECTIVE";
const std::string NEP_PORT              = "8080";
const std::string NEP_READY_TOPIC       = "cam_ready_topic";
const std::string NEP_READY_DATA        = "cam_ready";

class HumanPoseEstimationDemo : public ModelDemoAbstract
{
    public:
        HumanPoseEstimaionDemo(int argc, char *argv[], std::string file);

        void loop();

    private:
        HumanPoseEstimator estimator;
};