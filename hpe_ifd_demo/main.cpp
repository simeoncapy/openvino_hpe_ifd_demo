// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

// Merging of human_pose_estimation demo with interactive_face_detection demo

/**
* \brief The entry point for the Inference Engine Human Pose Estimation demo application
* \file human_pose_estimation_demo/main.cpp
* \example human_pose_estimation_demo/main.cpp
*/

#include <gflags/gflags.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <pthread.h>
#include <utility>
#include <chrono>
#include <thread>
#include <functional>
#include <random>
#include <memory>
#include <algorithm>
#include <iterator>
#include <list>
#include <set>

#include <zmq.hpp>

#include <inference_engine.hpp>

#include <monitors/presenter.h>
#include <samples/ocv_common.hpp>
#include <samples/slog.hpp>

#include "human_pose_estimation_demo.hpp"
#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"

#include "interactive_face_detection.hpp"
#include "detectors.hpp"
#include "face.hpp"
#include "visualizer.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <ie_iextension.h>

#include "json.hpp"
#include "NepPublisher.hpp"
#include "NepSubscriber.hpp"

using namespace InferenceEngine;
using namespace human_pose_estimation;
using json = nlohmann::json;

const std::string NEP_TOPIC_HPE         = "body_position";
const std::string NEP_DATA_HPE          = "bodyPositions";
const std::string NEP_DATA_PERSPECTIVE  = "PERSPECTIVE";

const std::string NEP_PORT              = "8080";
const std::string NEP_READY_TOPIC       = "cam_ready_topic";
const std::string NEP_READY_DATA        = "cam_ready";

const std::string NEP_TOPIC_IFD         = "user_emotions";
const std::string NEP_DATA_IFD          = "userEmotions";

std::map<std::string, std::string> constantes_hpe, constantes_ifd;

void hpe_loop(cv::Mat);
void ifd_loop(cv::Mat);

std::string getEmotion(std::map<std::string, float> emotions) {
    std::string emotion = "neutral";
    float max = 0.0;
    for (auto const& x : emotions) {
        if (x.second > max) {
            max = x.second;
            emotion = x.first;
        }
    }
    return emotion;
}

void loadConstants(Bool hpe)
{
    std::ifstream myfile;
    std::string line;
    if(hpe)
        myfile.open("constants.conf");
    else
        myfile.open("constants_emo.conf");
    char delimiter = '=';

    if (myfile.is_open())
    {
        while (std::getline(myfile,line))
        {
            std::istringstream tokenStream(line);
            std::string key, value;
            std::getline(tokenStream, key, delimiter);
            std::getline(tokenStream, value, delimiter);
            if(hpe)
                constantes_hpe[key] = value;
            else
                constantes_ifd[key] = value;
        }
        myfile.close();
    }
    else
        std::cout << "Cannot open the constants file" << std::endl;
}

bool ParseAndCheckCommandLine(int argc, char *argv[]) {
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

    return true;
}

void modifyImage(cv::Mat &src, cv::Mat &dest, int flipParam)
{
    //cv::resize(src, dest, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
    if(flipParam >= 0)
        cv::rotate(src, dest, flipParam);
    else
        dest = src;    
}

int main(int argc, char *argv[]) {
    try {
        loadConstants(true); // load HPE constants
        loadConstants(false); // load IFD constants
        //NepPublisher pub (NEP_TOPIC, NEP_PORT);
        NepPublisher pub_hpe, pub_ifd;
        if(constantes["IS_2_PI"] == "True")
        {
            //pub.createDirect(constantes["NEP_TOPIC"], constantes["NEP_PORT"], constantes["IP_OTHER"]);
            pub_hpe.createHybrid(constantes_hpe["NEP_TOPIC"], "cppnodepub", constantes_hpe["IP_OTHER"]);
            pub_ifd.createHybrid(constantes_ifd["NEP_TOPIC"], "cppnodepub", constantes_ifd["IP_OTHER"]);
        }
        else
        {
            //pub.createDirect(constantes["NEP_TOPIC"], constantes["NEP_PORT"]);
            pub_hpe.createHybrid(constantes_hpe["NEP_TOPIC"], "cppnodepub");
            pub_ifd.createHybrid(constantes_ifd["NEP_TOPIC"], "cppnodepub");
        }
        std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

        // ------------------------------ Parsing and validating of input arguments --------------------------
        if (!ParseAndCheckCommandLine(argc, argv)) {
            return EXIT_SUCCESS;
        }

        int flipVideo = -1;
        if (FLAGS_f == "90")
            flipVideo = cv::ROTATE_90_CLOCKWISE;
        else if (FLAGS_f == "180")
            flipVideo = cv::ROTATE_180;
        else if (FLAGS_f == "270")
            flipVideo = cv::ROTATE_90_COUNTERCLOCKWISE;
        else
            flipVideo = -1;


        HumanPoseEstimator estimator(FLAGS_m, FLAGS_d, FLAGS_pc);

        slog::info << "Reading input" << slog::endl;
        cv::VideoCapture cap;
        if (!(FLAGS_i == "cam" ? cap.open(0) : cap.open(FLAGS_i))) {
            throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
        }

        Timer timer;
        int delay_hpe = 33;

        cv::Mat curr_frame, temp_frame;
        cap >> temp_frame;
        modifyImage(temp_frame, curr_frame, flipVideo);

        estimator.reshape(curr_frame); // hpe

        const size_t width  = static_cast<size_t>(curr_frame.cols); // ifd
        const size_t height = static_cast<size_t>(curr_frame.rows); // ifd

        std::cout << "To close the application, press 'CTRL+C' here";
        if (!FLAGS_no_show) {
            std::cout << " or switch to the output window and press ESC key" << std::endl;
            std::cout << "To pause execution, switch to the output window and press 'p' key" << std::endl;
        }
        std::cout << std::endl;

        cv::VideoWriter videoWriter;
        if (!FLAGS_o.empty()) {
            videoWriter.open(FLAGS_o, cv::VideoWriter::fourcc('I', 'Y', 'U', 'V'), 25, cv::Size(width, height));
        }

        cv::Size graphSize{static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH) / 4), 60};
        Presenter presenter(FLAGS_u, static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)) - graphSize.height - 10, graphSize);
        std::vector<HumanPose> poses;
        bool isLastFrame = false;
        bool isAsyncMode = true;
        bool isModeChanged = true;
        if(FLAGS_s == "1")
        {
            isAsyncMode = false;// execution is always started in SYNC mode (demo default)
            isModeChanged = false; // set to true when execution mode is changed (SYNC<->ASYNC)
        }
        bool blackBackground = FLAGS_black;

        typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
        auto total_t0 = std::chrono::high_resolution_clock::now();
        auto wallclock = std::chrono::high_resolution_clock::now();
        double render_time = 0;

        // ---------------------------------------------------------------------------------------------------
        // --------------------------- 1. Loading Inference Engine IFD -----------------------------

        Core ie;

        std::set<std::string> loadedDevices;
        std::pair<std::string, std::string> cmdOptions[] = {
            {FLAGS_d_f, FLAGS_m_f},
            {FLAGS_d_ag, FLAGS_m_ag},
            {FLAGS_d_hp, FLAGS_m_hp},
            {FLAGS_d_em, FLAGS_m_em},
            {FLAGS_d_lm, FLAGS_m_lm}
        };
        FaceDetection faceDetector(FLAGS_mf, FLAGS_d_f, 1, false, FLAGS_async, FLAGS_t, FLAGS_r,
                                   static_cast<float>(FLAGS_bb_enlarge_coef), static_cast<float>(FLAGS_dx_coef), static_cast<float>(FLAGS_dy_coef));
        AgeGenderDetection ageGenderDetector(FLAGS_m_ag, FLAGS_d_ag, FLAGS_n_ag, FLAGS_dyn_ag, FLAGS_async, FLAGS_r);
        HeadPoseDetection headPoseDetector(FLAGS_m_hp, FLAGS_d_hp, FLAGS_n_hp, FLAGS_dyn_hp, FLAGS_async, FLAGS_r);
        EmotionsDetection emotionsDetector(FLAGS_m_em, FLAGS_d_em, FLAGS_n_em, FLAGS_dyn_em, FLAGS_async, FLAGS_r);
        FacialLandmarksDetection facialLandmarksDetector(FLAGS_m_lm, FLAGS_d_lm, FLAGS_n_lm, FLAGS_dyn_lm, FLAGS_async, FLAGS_r);

        for (auto && option : cmdOptions) {
            auto deviceName = option.first;
            auto networkName = option.second;

            if (deviceName.empty() || networkName.empty()) {
                continue;
            }

            if (loadedDevices.find(deviceName) != loadedDevices.end()) {
                continue;
            }
            slog::info << "Loading device " << deviceName << slog::endl;
            std::cout << ie.GetVersions(deviceName) << std::endl;

            /** Loading extensions for the CPU device **/
            if ((deviceName.find("CPU") != std::string::npos)) {

                if (!FLAGS_l.empty()) {
                    // CPU(MKLDNN) extensions are loaded as a shared library and passed as a pointer to base extension
                    auto extension_ptr = make_so_pointer<IExtension>(FLAGS_l);
                    ie.AddExtension(extension_ptr, "CPU");
                    slog::info << "CPU Extension loaded: " << FLAGS_l << slog::endl;
                }
            } else if (!FLAGS_c.empty()) {
                // Loading extensions for GPU
                ie.SetConfig({{PluginConfigParams::KEY_CONFIG_FILE, FLAGS_c}}, "GPU");
            }

            loadedDevices.insert(deviceName);
        }

        /** Per-layer metrics **/
        if (FLAGS_pc) {
            ie.SetConfig({{PluginConfigParams::KEY_PERF_COUNT, PluginConfigParams::YES}});
        }

        // ---------------------------------------------------------------------------------------------------

        // --------------------------- 2. Reading IR models and loading them to plugins ----------------------
        // Disable dynamic batching for face detector as it processes one image at a time
        Load(faceDetector).into(ie, FLAGS_d, false);
        Load(ageGenderDetector).into(ie, FLAGS_d_ag, FLAGS_dyn_ag);
        Load(headPoseDetector).into(ie, FLAGS_d_hp, FLAGS_dyn_hp);
        Load(emotionsDetector).into(ie, FLAGS_d_em, FLAGS_dyn_em);
        Load(facialLandmarksDetector).into(ie, FLAGS_d_lm, FLAGS_dyn_lm);
        // ----------------------------------------------------------------------------------------------------

        // --------------------------- 3. Doing inference -----------------------------------------------------
        // Starting inference & calculating performance
        slog::info << "Start inference " << slog::endl;

        bool isFaceAnalyticsEnabled = ageGenderDetector.enabled() || headPoseDetector.enabled() ||
                                      emotionsDetector.enabled() || facialLandmarksDetector.enabled();

        std::ostringstream out;
        size_t framesCounter = 0;
        int delay_ifd = 1;
        double msrate = -1;
        cv::Mat prev_frame, next_frame;
        std::list<Face::Ptr> faces;
        size_t id = 0;

        if (FLAGS_fps > 0) {
            msrate = 1000.f / FLAGS_fps;
        }

        Visualizer::Ptr visualizer;
        if (!FLAGS_no_show || !FLAGS_o.empty()) {
            visualizer = std::make_shared<Visualizer>(cv::Size(width, height));
            if (!FLAGS_no_show_emotion_bar && emotionsDetector.enabled()) {
                visualizer->enableEmotionBar(emotionsDetector.emotionsVec);
            }
        }

        // Detecting all faces on the first frame and reading the next one
        faceDetector.enqueue(frame);
        faceDetector.submitRequest();

        prev_frame = frame.clone();

        // Reading the next frame
        bool frameReadStatus = cap.read(frame);

        std::cout << "To close the application, press 'CTRL+C' here";
        if (!FLAGS_no_show) {
            std::cout << " or switch to the output window and press Q or Esc";
        }
        std::cout << std::endl;

        const cv::Point THROUGHPUT_METRIC_POSITION{10, 45};

        cv::Size graphSize{static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH) / 4), 60};
        Presenter presenter(FLAGS_u, THROUGHPUT_METRIC_POSITION.y + 15, graphSize);

        while (true) {
            temp_frame = cv::Mat();
            if (!cap.read(temp_frame)) {
                if (temp_frame.empty()) {
                    isLastFrame = true; //end of video file
                } else {
                    throw std::logic_error("Failed to get frame from cv::VideoCapture");
                }
            }
            modifyImage(temp_frame, next_frame, flipVideo);

            hpe_loop();
            ifd_loop();

        }
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Execution successful" << std::endl;
    return EXIT_SUCCESS;
}

void hpe_loop(cv::Mat next_frame)
{

}

void ifd_loop()
{

}