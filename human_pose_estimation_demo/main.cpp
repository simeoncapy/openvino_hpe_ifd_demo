// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

/**
* \brief The entry point for the Inference Engine Human Pose Estimation demo application
* \file human_pose_estimation_demo/main.cpp
* \example human_pose_estimation_demo/main.cpp
*/

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

using namespace InferenceEngine;
using namespace human_pose_estimation;
using json = nlohmann::json;

const std::string NEP_TOPIC             = "body_position";
const std::string NEP_DATA              = "bodyPositions";
const std::string NEP_DATA_PERSPECTIVE  = "PERSPECTIVE";
const std::string NEP_PORT              = "8080";
const std::string NEP_READY_TOPIC       = "cam_ready_topic";
const std::string NEP_READY_DATA        = "cam_ready";

std::map<std::string, std::string> constantes;

// Global variables for message storage
std::string msg;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
cv::Mat image;

bool is_base64(char c)
{
    return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_decode(const std::string& base64String)
{
    static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::vector<uint8_t> decodedData;
    int in_len = base64String.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    uint32_t char_array_4[4], char_array_3[3];

    while (in_len-- && (base64String[in_] != '=') &&
        (is_base64(base64String[in_]) || (base64String[in_] == '=')))
    {
        uint8_t c = base64String[in_++];
        char_array_4[i++] = c;
        if (i == 4)
        {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                decodedData.push_back(char_array_3[i]);
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++)
            decodedData.push_back(char_array_3[j]);
    }

    return std::string(decodedData.begin(), decodedData.end());
}

cv::Mat base64ToMat(const std::string& base64String)
{
    std::string decodedString;
    std::vector<uint8_t> imageData;

    // Decode the Base64 string
    cv::Mat image;
    try
    {
        decodedString = base64_decode(base64String);

        // Convert the decoded string to a vector of bytes
        imageData.assign(decodedString.begin(), decodedString.end());

        // Decode the image from the byte data
        image = cv::imdecode(imageData, cv::IMREAD_COLOR);
    }
    catch (const std::exception& e)
    {
        std::cout << "Error decoding Base64 string: " << e.what() << std::endl;
    }

    return image;
}

void* subscriber_thread(void* arg) 
{
    //NepSubscriber sub("testImage", "8585");
    while (1) 
    {	
        msg = arg->listen_string();
        pthread_mutex_lock(&mutex);
        image = base64ToMat(msg);
        pthread_mutex_unlock(&mutex);
        pthread_cond_signal(&cond);
    }
    return NULL;
}

void loadConstants()
{
    std::ifstream myfile;
    std::string line;
    myfile.open("constants.conf");
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

bool ParseAndCheckCommandLine(int argc, char* argv[]) {
    // ---------------------------Parsing and validation of input args--------------------------------------

    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        showAvailableDevices();
        return false;
    }

    std::cout << "Parsing input parameters" << std::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m.empty()) {
        throw std::logic_error("Parameter -m is not set");
    }

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

int main(int argc, char* argv[]) {
    try {
        loadConstants();
        //NepPublisher pub (NEP_TOPIC, NEP_PORT);
        NepPublisher pub;
        NepSubscriber sub;
        if(constantes["IS_2_PI"] == "True")
        {
            //pub.createDirect(constantes["NEP_TOPIC"], constantes["NEP_PORT"], constantes["IP_OTHER"]);
            pub.createHybrid(constantes["NEP_TOPIC"], "cppnodepub", constantes["IP_OTHER"]);
            sub.createDirect(constantes["CAMERA_NEP_TOPIC"], constantes["CAMERA_NEP_PORT"]);
        }
        else
        {
            //pub.createDirect(constantes["NEP_TOPIC"], constantes["NEP_PORT"]);
            pub.createHybrid(constantes["NEP_TOPIC"], "cppnodepub");
            sub.createDirect(constantes["CAMERA_NEP_TOPIC"], constantes["CAMERA_NEP_PORT"]);
        }

        pthread_t thread;
        pthread_create(&thread, NULL, subscriber_thread, &sub);

        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_unlock(&mutex);

        std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

        // ------------------------------ Parsing and validation of input args ---------------------------------
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
        cv::VideoCapture cap;
        //cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);//360
        /*if (!(FLAGS_i == "cam" ? cap.open(0) : cap.open(FLAGS_i))) {
            throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
        }*/

        int delay = 33;

        // read input (video) frame
        cv::Mat curr_frame, temp_frame; 
        //cap >> temp_frame;
        //temp_frame = base64ToMat(sub.listen_string());
        // Get image from global variable

    	pthread_mutex_lock(&mutex);
        image.copyTo(temp_frame);
        pthread_mutex_unlock(&mutex);

        modifyImage(temp_frame, curr_frame, flipVideo);
        cv::Mat next_frame;
        /*if (!cap.grab()) {
            throw std::logic_error("Failed to get frame from cv::VideoCapture");
        }*/

        estimator.reshape(curr_frame);  // Do not measure network reshape, if it happened

        std::cout << "To close the application, press 'CTRL+C' here";
        if (!FLAGS_no_show) {
            std::cout << " or switch to the output window and press ESC key" << std::endl;
            std::cout << "To pause execution, switch to the output window and press 'p' key" << std::endl;
        }
        std::cout << std::endl;

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

        while (true) {
            auto t0 = std::chrono::high_resolution_clock::now();
            //here is the first asynchronus point:
            //in the async mode we capture frame to populate the NEXT infer request
            //in the regular mode we capture frame to the current infer request

            //---- checking if the camera should be paused
            //do{

            //}while();
            //----
            temp_frame = cv::Mat();
            /*if (!cap.read(temp_frame)) {
                if (temp_frame.empty()) {
                    isLastFrame = true; //end of video file
                } else {
                    throw std::logic_error("Failed to get frame from cv::VideoCapture");
                }
            }*/
            //temp_frame = base64ToMat(sub.listen_string());
            pthread_mutex_lock(&mutex);
            image.copyTo(temp_frame);
            pthread_mutex_unlock(&mutex);
            modifyImage(temp_frame, next_frame, flipVideo);
            if (isAsyncMode) {
                if (isModeChanged) {
                    estimator.frameToBlobCurr(curr_frame);
                }
                if (!isLastFrame) {
                    estimator.frameToBlobNext(next_frame);
                }
            } else if (!isModeChanged) {
                estimator.frameToBlobCurr(curr_frame);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            double decode_time = std::chrono::duration_cast<ms>(t1 - t0).count();

            t0 = std::chrono::high_resolution_clock::now();
            // Main sync point:
            // in the trully Async mode we start the NEXT infer request, while waiting for the CURRENT to complete
            // in the regular mode we start the CURRENT request and immediately wait for it's completion
            if (isAsyncMode) {
                if (isModeChanged) {
                    estimator.startCurr();
                }
                if (!isLastFrame) {
                    estimator.startNext();
                }
            } else if (!isModeChanged) {
                estimator.startCurr();
            }

            if (estimator.readyCurr()) {
                t1 = std::chrono::high_resolution_clock::now();
                ms detection = std::chrono::duration_cast<ms>(t1 - t0);
                t0 = std::chrono::high_resolution_clock::now();
                ms wall = std::chrono::duration_cast<ms>(t0 - wallclock);
                wallclock = t0;

                t0 = std::chrono::high_resolution_clock::now();

                if (!FLAGS_no_show) {
                    if (blackBackground) {
                        curr_frame = cv::Mat::zeros(curr_frame.size(), curr_frame.type());
                    }
                    std::ostringstream out;
                    out << "OpenCV cap/render time: " << std::fixed << std::setprecision(2)
                        << (decode_time + render_time) << " ms";

                    cv::putText(curr_frame, out.str(), cv::Point2f(0, 25),
                                cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 0));
                    out.str("");
                    out << "Wallclock time " << (isAsyncMode ? "(TRUE ASYNC):      " : "(SYNC, press Tab): ");
                    out << std::fixed << std::setprecision(2) << wall.count()
                        << " ms (" << 1000.f / wall.count() << " fps)";
                    cv::putText(curr_frame, out.str(), cv::Point2f(0, 50),
                                cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 255));
                    if (!isAsyncMode) {  // In the true async mode, there is no way to measure detection time directly
                        out.str("");
                        out << "Detection time  : " << std::fixed << std::setprecision(2) << detection.count()
                        << " ms ("
                        << 1000.f / detection.count() << " fps)";
                        cv::putText(curr_frame, out.str(), cv::Point2f(0, 75), cv::FONT_HERSHEY_TRIPLEX, 0.6,
                            cv::Scalar(255, 0, 0));
                    }
                }

                poses = estimator.postprocessCurr();

                if (FLAGS_r) {
                    if (!poses.empty()) {
                        std::time_t result = std::time(nullptr);
                        char timeString[sizeof("2020-01-01 00:00:00: ")];
                        std::strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S: ", std::localtime(&result));
                        std::cout << timeString;
                     }

                    int idPerson = 0;
                    std::string isClose = "Far";
                    json msg, msg2;
                    
                    for (HumanPose const& pose : poses) {
                        if(pose.keypoints[8].x == -1 && pose.keypoints[8].y == -1 && pose.keypoints[11].x == -1 && pose.keypoints[11].y == -1)
                            break; // if not detecting the hips, we discard the data

                        std::stringstream rawPose;
                        rawPose << std::fixed << std::setprecision(0);
                        for (auto const& keypoint : pose.keypoints) {
                            rawPose << keypoint.x << ";" << keypoint.y << ",";
                        }
                        rawPose << pose.score << ";";
                        // If the noze is not visible, the person is close
                        if(pose.keypoints[0].x < 0 || pose.keypoints[10].x < 0 || pose.keypoints[13].x < 0)
                        { 
                            rawPose << "1";
                            if (idPerson == 0)
                                isClose = "Close";
                        }
                        else
                        {
                            rawPose << "0";
                            if (idPerson == 0)
                                isClose = "Far";
                        }
                        std::cout << rawPose.str() << std::endl;

                        // Creating data for NEP
                        std::string tempData = rawPose.str();
                        //std::replace(tempData.begin(), tempData.end(), ' ', ';'); // Replace " " by ";" to have the data like that: x1;y1,x2;y2...                  
                        msg[std::to_string(idPerson)] = tempData;
                        idPerson++;
                        break; // We use only the first person
                    }

                    if(idPerson != 0) // If the data were discarded, then not sending
                    {
                        msg2[constantes["NEP_DATA"]] = msg;
                        msg2[constantes["NEP_DATA_PERSPECTIVE"]] = isClose;
                        // Sending to NEP
                        pub.publish(msg2);
                    }
                }

                if (!FLAGS_no_show) {
                    presenter.drawGraphs(curr_frame);
                    renderHumanPose(poses, curr_frame);
                    cv::imshow("Human Pose Estimation on " + FLAGS_d, curr_frame);
                    t1 = std::chrono::high_resolution_clock::now();
                    render_time = std::chrono::duration_cast<ms>(t1 - t0).count();
                }
            }

            if (isLastFrame) {
                break;
            }

            if (isModeChanged) {
                isModeChanged = false;
            }

            // Final point:
            // in the truly Async mode we swap the NEXT and CURRENT requests for the next iteration
            curr_frame = next_frame;
            next_frame = cv::Mat();
            if (isAsyncMode) {
                estimator.swapRequest();
            }

            const int key = cv::waitKey(delay) & 255;
            if (key == 'p') {
                delay = (delay == 0) ? 33 : 0;
            } else if (27 == key) { // Esc
                break;
            } else if (9 == key) { // Tab
                isAsyncMode ^= true;
                isModeChanged = true;
            } else if (32 == key) { // Space
                blackBackground ^= true;
            }
            presenter.handleKey(key);
        }

        auto total_t1 = std::chrono::high_resolution_clock::now();
        ms total = std::chrono::duration_cast<ms>(total_t1 - total_t0);
        std::cout << "Total Inference time: " << total.count() << std::endl;
        std::cout << presenter.reportMeans() << '\n';
        pthread_join(thread, NULL);
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
