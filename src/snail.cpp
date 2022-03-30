#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>

namespace {
const char* about = "ChArUco-based snail localization";
const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to run the camera calibrate process;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}

const std::string calibFile="/home/pi/calib.txt";

void createBoard();
void detectCharucoBoardWithCalibrationPose();

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

static bool saveCameraParams(const std::string &filename, cv::Size imageSize, float aspectRatio, int flags,
                             const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, double totalAvgErr) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

void createBoard() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}

void calibrateCharucoBoard() {
    int calibrationFlags = 0;
    //calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
    //calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    //calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

    float aspectRatio = 1;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create(); // can also read from file

    cv::VideoCapture inputVideo;
    inputVideo.open(-1); // -1 is autodetect, otherwise set to /dev/video*

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

    // collect data from each frame
    std::vector< std::vector< std::vector< cv::Point2f > > > allCorners;
    std::vector< std::vector< int > > allIds;
    std::vector< cv::Mat > allImgs;
    cv::Size imgSize;

    while(inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        std::vector< int > ids;
        std::vector< std::vector< cv::Point2f > > corners, rejected;

        // detect markers
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        //if(refindStrategy) cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // interpolate charuco corners
        cv::Mat currentCharucoCorners, currentCharucoIds;
        if(ids.size() > 0)
            cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                                             currentCharucoIds);

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy, corners);

        if(currentCharucoCorners.total() > 0)
            cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
        char key = (char)cv::waitKey(30);
        if(key == 27) break;
        if(key == 'c' && ids.size() > 0) {
            std::cout << "Frame captured" << std::endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
        }
    }

    if(allIds.size() < 1) {
        std::cerr << "Not enough captures for calibration" << std::endl;
        return;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector< cv::Mat > rvecs, tvecs;
    double repError;

    //if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
    //    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    //    cameraMatrix.at< double >(0, 0) = aspectRatio;
    //}

    // prepare data for calibration
    std::vector< std::vector< cv::Point2f > > allCornersConcatenated;
    std::vector< int > allIdsConcatenated;
    std::vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, cv::noArray(), cv::noArray(), calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    std::vector< cv::Mat > allCharucoCorners;
    std::vector< cv::Mat > allCharucoIds;
    std::vector< cv::Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if(allCharucoCorners.size() < 4) {
        std::cerr << "Not enough corners for calibration" << std::endl;
        return;
    }

    // calibrate camera using charuco
    repError =
        cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    bool saveOk =  saveCameraParams(calibFile, imgSize, aspectRatio, calibrationFlags,
                                    cameraMatrix, distCoeffs, repError);
    if(!saveOk) {
        std::cerr << "Cannot save output file" << std::endl;
        return;
    }

    std::cout << "Rep Error: " << repError << std::endl;
    std::cout << "Rep Error Aruco: " << arucoRepErr << std::endl;
    std::cout << "Calibration saved to " << calibFile << std::endl;

    // show interpolated charuco corners for debugging
    //if(showChessboardCorners) {
    //    for(unsigned int frame = 0; frame < filteredImages.size(); frame++) {
    //        Mat imageCopy = filteredImages[frame].clone();
    //        if(allIds[frame].size() > 0) {

    //            if(allCharucoCorners[frame].total() > 0) {
    //                cv::aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame],
    //                                                   allCharucoIds[frame]);
    //            }
    //        }

    //        imshow("out", imageCopy);
    //        char key = (char)waitKey(0);
    //        if(key == 27) break;
    //    }
    //}
}

void detectCharucoBoardWithCalibrationPose() {
    cv::VideoCapture inputVideo;
    inputVideo.open(1); // set to /dev/video*

    cv::Mat cameraMatrix, distCoeffs;
    bool readOk = readCameraParameters(calibFile, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera calibration file: " << calibFile << std::endl;
        return;
    }
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    while (inputVideo.grab()) {
        cv::Mat image;
        cv::Mat imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0) {
                cv::Scalar color = cv::Scalar(255, 0, 0);
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                cv::Vec3d rvec, tvec;
                bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                // if charuco pose is valid
                if (valid)
                    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
            }
        }
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }
}

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (argc < 2) {
        parser.printMessage();
        return 0;
    }
    int choose = parser.get<int>("c");
    switch (choose) {
    case 1:
        createBoard();
        std::cout << "BoardImg.jpg generated" << std::endl;
        break;
    case 2:
        calibrateCharucoBoard();
		break;
    case 3:
        detectCharucoBoardWithCalibrationPose();
        break;
    default:
        break;
    }
    return 0;
}
