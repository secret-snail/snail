//! \example mbot-apriltag-pbvs.cpp
#include <signal.h>

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <emp_client.h>
#include <emp-tool/io/net_io_channel.h>
#include <emp-tool/utils/constants.h>

#include <jlog.h>
#include <cleartext-ref/lmlocalization.hpp>

//#define OFFLOAD_IP "68.74.215.161"
#define OFFLOAD_IP "192.168.11.9"

template<typename T>
void Hoffs3DPoints(std::vector<cv::Point3_<T>> &points) {
    points.push_back(cv::Point3_<T>(0,10,6));
    points.push_back(cv::Point3_<T>(0,2,6));
    points.push_back(cv::Point3_<T>(2,0,6));
    points.push_back(cv::Point3_<T>(0,10,2));
    points.push_back(cv::Point3_<T>(0,2,2));
    points.push_back(cv::Point3_<T>(2,0,2));
}

template<typename T>
void Hoffs2DPoints(std::vector<cv::Point_<T>> &points) {
    points.push_back(cv::Point_<T>(183,147));
    points.push_back(cv::Point_<T>(350,133));
    points.push_back(cv::Point_<T>(454,144));
    points.push_back(cv::Point_<T>(176,258));
    points.push_back(cv::Point_<T>(339,275));
    points.push_back(cv::Point_<T>(444,286));
}

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

bool signal_stop = false;
int signal_count = 0;
void signal_callback_handler(int signum) {
  std::cout << "caught signal, exiting\n";
  signal_stop = true;
  signal_count ++;
  if (signal_count >= 3)
    exit(1);
}

int main(int argc, const char **argv)
{
  bool secure = false;
  bool opencv = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--secure") {
      secure = true;
    } else if (std::string(argv[i]) == "--opencv") {
      opencv = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--secure]";
      return EXIT_SUCCESS;
    }
  }

  if (opencv && secure) {
    std::cout << "Cannot combine secure and opencv\n";
  }

  float f=715;
  float cx=354;
  float cy=245;
  float _cM[] = {f, 0, cx,
                 0, f, cy,
                 0, 0, 1};
  cv::Mat cameraMatrix = cv::Mat(3, 3, cv::DataType<float>::type, _cM);
  cv::Mat distCoeffs = cv::Mat::zeros(4,1,cv::DataType<float>::type);

  vector<cv::Point2f> imagePoints;
  vector<cv::Point3f> objectPoints;
  Hoffs2DPoints(imagePoints);
  Hoffs3DPoints(objectPoints);

  emp::NetIO *aliceio;
  emp::NetIO *bobio;
  if (secure) {
    std::cout << "Ensure the server side code runs at the same speed as cleartext code.\n"
	    << "Since the crypto is actually much slower than plaintext,\n"
            << "it must be emulated with a sleep instead of the servers acutally computing localization.\n";

    std::cout << "\n\nWARNING: Did you modify the server side code with (PPL_FLOW_POWER_TESTING) to sleep instead of compute?\n";
    sleep(5);

    int baseport = 8080;
    std::cout << "Connecting to alice and bob..." << std::endl;
    aliceio = new emp::NetIO(OFFLOAD_IP, baseport+emp::ALICE*17);
    bobio = new emp::NetIO(OFFLOAD_IP, baseport+emp::BOB*17);
    std::cout << "Connected to alice port: " << baseport+emp::ALICE*17
        << " bob port: " << baseport+emp::BOB*17 << std::endl;
  }

  // quit signal handler
  signal(SIGINT, signal_callback_handler);  

  for (;;) {
    WALL_CLOCK(power_tester_clock);
    WALL_TIC(power_tester_clock);
    if (secure) {
      cout << "testing lm\n";
      cv::Mat rvec = cv::Mat::zeros(3,1,cv::DataType<float>::type);
      cv::Mat tvec = cv::Mat::zeros(3,1,cv::DataType<float>::type);
      bool res = estimatePoseSecure(objectPoints, imagePoints, cameraMatrix,
            	                distCoeffs, rvec, tvec, true,
            			aliceio, bobio);
      std::cout << "secure pose:\n";
      std::cout << rvec << '\n';
      std::cout << tvec << '\n';

    } else if (opencv) {
      cout << "\nopencv\n";
      cv::Mat rvec = cv::Mat::zeros(3,1,cv::DataType<float>::type);
      cv::Mat tvec = cv::Mat::zeros(3,1,cv::DataType<float>::type);
      CLOCK(opencv);
      TIC(opencv);
      // OpenCV PnP method
      cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
      TOC(opencv);
      std::cout << "opencv pose:\n";
      std::cout << rvec << '\n';
      std::cout << tvec << '\n';

    } else {
      cout << "testing lm\n";
      float rt[6] = {0, 0, 0, 0, 0, 0}; // initial guess
      lm<float>(objectPoints, imagePoints, f, cx, cy, rt);
      cv::Mat rvec = cv::Mat(3,1,cv::DataType<float>::type,rt);
      cv::Mat tvec = cv::Mat(3,1,cv::DataType<float>::type,&rt[3]);
      std::cout << "cleartext pose:\n";
      std::cout << rvec << '\n';
      std::cout << tvec << '\n';
    }
    WALL_TOC(power_tester_clock);
  }

  return EXIT_SUCCESS;
}
