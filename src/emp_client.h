#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <emp-tool/io/net_io_channel.h>

bool estimatePoseSecure(std::vector<cv::Point3f> &objPoints, std::vector<cv::Point2f> &imPoints,
                        cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                        cv::InputOutputArray _rvec, cv::InputOutputArray _tvec, bool useExtrinsicGuess,
                        emp::NetIO *aliceio, emp::NetIO *bobio);
