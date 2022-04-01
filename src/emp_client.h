#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <emp-tool/io/net_io_channel.h>

bool estimatePoseCharucoBoard_Secure(cv::InputArray _charucoCorners, cv::InputArray _charucoIds,
                              const cv::Ptr<cv::aruco::CharucoBoard> &_board, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                              cv::InputOutputArray _rvec, cv::InputOutputArray _tvec, bool useExtrinsicGuess,
                              emp::NetIO* aliceio, emp::NetIO* bobio);
