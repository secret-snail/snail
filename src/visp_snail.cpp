//! \example mbot-apriltag-pbvs.cpp
#include <signal.h>
#include <chrono>
#include <thread>
#include <mutex>

#include <visp3/core/vpSerial.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/vs/vpServo.h>

#include <pigpio.h>

#include <emp_client.h>

#include <emp-tool/io/net_io_channel.h>
#include <emp-tool/utils/constants.h>

const std::string calibFile="/home/pi/snail/calib.txt";
//#define OFFLOAD_IP "68.74.215.161"
#define OFFLOAD_IP "192.168.11.9"

static constexpr const float kInputConditioningScalar = 2.f;
static const constexpr double Z_d = 0.4f; // Desired distance to the target


static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

#define in1 25
#define in2 23

#define in3 24
#define in4 26

#define en1 19
#define en2 13

int servo_setup() {
  if (gpioInitialise() < 0) {
    return 1; 
  }
  gpioSetMode(in1, PI_OUTPUT);
  gpioSetMode(in2, PI_OUTPUT);
  gpioSetMode(in3, PI_OUTPUT);
  gpioSetMode(in4, PI_OUTPUT);
  
  gpioSetMode(en1, PI_OUTPUT);
  gpioSetMode(en2, PI_OUTPUT);
  gpioSetPWMfrequency(en1, 400);
  gpioSetPWMfrequency(en2, 400);
  gpioPWM(en1, 0);
  gpioPWM(en2, 0);
  return 0;
}

void servo_move(float l, float r) {
  if (l > 0) {
    gpioWrite(in1, PI_LOW);
    gpioWrite(in2, PI_HIGH);
  } else {
    gpioWrite(in1, PI_HIGH);
    gpioWrite(in2, PI_LOW);
  }
  if (r > 0) {
    gpioWrite(in3, PI_LOW);
    gpioWrite(in4, PI_HIGH);
  } else {
    gpioWrite(in3, PI_HIGH);
    gpioWrite(in4, PI_LOW);
  }

  l = abs(l);
  r = abs(r);
  float largest = std::max(l,r);
  if (largest > 10) {
    l = l/largest*250;
    r = r/largest*250;
  } else {
    l = l*10;
    r = r*10;
  }
  gpioPWM(en1, l);
  gpioPWM(en2, r);

  std::cout << "wrote pwms to " << l << " " << r << "\n";
}

void servo_stop() {
   gpioWrite(in1, PI_HIGH);
   gpioWrite(in2, PI_HIGH);
   gpioWrite(in3, PI_HIGH);
   gpioWrite(in4, PI_HIGH);
   gpioPWM(en1, 0);
   gpioPWM(en2, 0);
   gpioTerminate(); 
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

//sudo groupadd gpio
//sudo usermod -a -G gpio pi
//sudo grep gpio /etc/group
//sudo chown root.gpio /dev/gpiomem
//sudo chmod g+rw /dev/gpiomem

std::mutex pose_mutex;  // protects shared global vars below
std::chrono::steady_clock::time_point last_updated;
double X = 0, Y = 0, Z = Z_d;

void control_loop(bool secure, bool display_on) {
  double target_Z = Z_d * (secure ? kInputConditioningScalar : 1);

  vpServo task;
  vpAdaptiveGain lambda;
  if (display_on)
    lambda.initStandard(2.5, 0.4, 30); // lambda(0)=2.5, lambda(oo)=0.4 and lambda'(0)=30
  else
    lambda.initStandard(1, 0.1, 10); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
    //lambda.initStandard(4, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30

  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
  task.setLambda(lambda);
  vpRotationMatrix cRe;
  cRe[0][0] = 0;
  cRe[0][1] = -1;
  cRe[0][2] = 0;
  cRe[1][0] = 0;
  cRe[1][1] = 0;
  cRe[1][2] = -1;
  cRe[2][0] = 1;
  cRe[2][1] = 0;
  cRe[2][2] = 0;

  vpHomogeneousMatrix cMe(vpTranslationVector(), cRe);
  vpVelocityTwistMatrix cVe(cMe);
  task.set_cVe(cVe);

  vpMatrix eJe(6, 2, 0);
  eJe[0][0] = eJe[5][1] = 1.0;
  task.set_eJe(eJe);

  // Create X_3D visual features for visp task.
  vpFeaturePoint3D s_XZ, s_XZ_d;
  s_XZ.buildFrom(0, 0, target_Z);
  s_XZ_d.buildFrom(0, 0, target_Z);
  task.addFeature(s_XZ, s_XZ_d, vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectZ());

  std::chrono::steady_clock::time_point last_read;
  std::chrono::steady_clock::time_point last_estimated;
  double estimated_X=0, estimated_Y=0, estimated_Z=0;
  vpColVector velocity(2);

  while (!signal_stop) {
    // Read global state.
    {
      const std::lock_guard<std::mutex> lock(pose_mutex);
      if (last_read != last_updated) {
        // Got a new pose, overwrite our estimates.
	      std::cout << "Got a new pose, overwrite our estimates.\n";
        last_estimated = last_updated;
	last_read = last_updated;
        estimated_X = X;
        estimated_Y = Y;
        estimated_Z = Z;
      }
    }

    // Check for staleness i.e. we do not know current pose.
    auto now = std::chrono::steady_clock::now();
    auto msSinceRead = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count();
    if (msSinceRead > 750) { // Very stale global pose, stop moving
      servo_move(0,0);
      sleep(0.1);
      continue;
    }

    double z_tol = 0.1 * target_Z;
    if (estimated_Z > target_Z - z_tol && estimated_Z < target_Z + z_tol) { // z close enough
      servo_move(0,0);
      sleep(0.1);
      continue;
    }

    // Estimate current pose based on prev velocity and time passed.
    auto usSinceEstimated = std::chrono::duration_cast<std::chrono::microseconds>(now - last_estimated).count();
    if (usSinceEstimated > 1000) { // Stale estimated pose.
      //estimated_X += static_cast<double>(velocity[0]) * usSinceEstimated * 1.e-6; // forget about x for now
      estimated_Z += static_cast<double>(velocity[0]) * usSinceEstimated * 1.e-6;
      last_estimated = now;
    }

    // Compute the control law. Velocities are computed in the mobile robot reference frame
    s_XZ.set_XYZ(estimated_X, estimated_Y, estimated_Z);
    velocity = task.computeControlLaw();
    std::cout << "Send velocity to the mbot: " << velocity[0] << " m/s " << vpMath::deg(velocity[1]) << " deg/s" << std::endl;
    
    task.print();
    const static constexpr double radius = 0.0325;
    const static constexpr double L = 0.155 / 2;
    //const static constexpr double L = 0.04; // reduce impact of turns, doesn't work well
    double motor_left = (velocity[0] + (L * velocity[1])) / radius;
    double motor_right = (velocity[0] - (L * velocity[1])) / radius;

    std::cout << "motor velocity left: " << motor_left << ", right: " << motor_right << '\n';
    double rpm_left = motor_left * 30. / M_PI;
    double rpm_right = motor_right * 30. / M_PI;
    std::cout << "motor rpm left: " << vpMath::round(rpm_left) << ", right: " << vpMath::round(rpm_right) << '\n';
    servo_move(rpm_left, rpm_right);
  }
  servo_move(0,0);
  servo_stop();
}

int main(int argc, const char **argv)
{
  if (servo_setup()) {
    return 1;
  }

#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_V4L2)
  int device = 0;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.065;
  float quad_decimate = 4.0;
  int nThreads = 2;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  bool display_on = false;
  bool serial_off = false;
  bool save_image = false; // Only possible if display_on = true
  bool secure = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = std::atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      device = std::atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = std::atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
#if defined(VISP_HAVE_X11)
    } else if (std::string(argv[i]) == "--display_on") {
      display_on = true;
    } else if (std::string(argv[i]) == "--save_image") {
      save_image = true;
#endif
    } else if (std::string(argv[i]) == "--serial_off") {
      serial_off = true;
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--secure") {
      secure = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <camera input>] [--tag_size <tag_size in m>]"
                   " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                   " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                   " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: TAG_36ARTOOLKIT,"
                   " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
                   " [--display_tag]";
#if defined(VISP_HAVE_X11)
      std::cout << " [--display_on] [--save_image]";
#endif
      std::cout << " [--serial_off] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  try {
    vpImage<unsigned char> I;
    vpImage<unsigned char> I2;

    vpV4l2Grabber g;
    std::ostringstream device_name;
    device_name << "/dev/video" << device;
    g.setDevice(device_name.str());
    g.setScale(1);
    g.setNBuffers(1);
    g.acquire(I);
    g.acquire(I2);

    vpDisplay *d = NULL;
    vpImage<vpRGBa> O;
#ifdef VISP_HAVE_X11
    if (display_on) {
      d = new vpDisplayX(I2);
    }
#endif

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty())
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);

    std::cout << "cam:\n" << cam << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;

    vpDetectorAprilTag detector(tagFamily);

    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag);

    cv::Mat cameraMatrix, distCoeffs;
    cameraMatrix = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
    cameraMatrix.at<double>(0,0) = 615.1674805;
    cameraMatrix.at<double>(1,1) = 615.1674805;
    cameraMatrix.at<double>(0,2) = (I2.getWidth() / 2.);
    cameraMatrix.at<double>(1,2) = (I2.getHeight() / 2.);
    cameraMatrix.at<double>(2,2) = 1.0f;

    cv::Vec3d s_rvec={0,0,0}, s_tvec={0,0,1};

    emp::NetIO *aliceio;
    emp::NetIO *bobio;
    if (secure) {
      int baseport = 8080;
      std::cout << "Connecting to alice and bob..." << std::endl;
      aliceio = new emp::NetIO(OFFLOAD_IP, baseport+emp::ALICE*17);
      bobio = new emp::NetIO(OFFLOAD_IP, baseport+emp::BOB*17);
      std::cout << "Connected to alice port: " << baseport+emp::ALICE*17
          << " bob port: " << baseport+emp::BOB*17 << std::endl;
    }

    // override pigpio signal handler
    signal(SIGINT, signal_callback_handler);  

    // start control loop
    std::thread cl(control_loop, secure, display_on);

    std::vector<double> time_vec;
    while (!signal_stop) {
      // visp buffers images, clear out the buffer
      double imageT;
      do {
        imageT = vpTime::measureTimeMs();
      	g.acquire(I);
      } while (vpTime::measureTimeMs() - imageT < 10);

      // rotate image
      vpMatrix M(2, 3);
      M.eye();
      const double theta = vpMath::rad(180);
      M[0][0] = cos(theta);   M[0][1] = -sin(theta);   M[0][2] = I.getWidth();
      M[1][0] = sin(theta);   M[1][1] = cos(theta);    M[1][2] = I.getHeight();
      //M[0][0] = cos(theta);   M[0][1] = -sin(theta);   M[0][2] = 0;
      //M[1][0] = sin(theta);   M[1][1] = cos(theta);    M[1][2] = 0;
      vpImageTools::warpImage(I, M, I2);

      if (display_on) {
        vpDisplay::display(I2);
      }

      double t = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec;

      // calling detect with cam and cMo does pose estimation
      //detector.detect(I2, tagSize, cam, cMo_vec);
      //std::cout << "cMo_vec from detect:\n" << cMo_vec[0] << '\n';

      // should be equivalent to detect(I2) (just image) then getPose
      detector.detect(I2);
      cMo_vec.push_back({});
      if (detector.getNbObjects() == 1) {
	//if (!secure) {
      	  bool ret = detector.getPose(0, tagSize, cam, cMo_vec[0]);
	  if (!ret) {
              std::cout << "pose detection failed\n";
	      continue;
	  }
          //std::cout << "cMo_vec from cleartext getPose:\n" << cMo_vec[0] << '\n';
          std::cout << "cleartext pose:\n" << cMo_vec[0].getThetaUVector().t() <<
		  ' ' << cMo_vec[0].getTranslationVector().t() << '\n';
          if (display_on) {
            vpDisplay::displayFrame(I2, cMo_vec[0], cam, tagSize / 2, vpColor::none, 1);
	  }

	//}
	if (secure) {
	  std::vector<vpImagePoint> p = detector.getPolygon(0);

          std::vector<cv::Point3f> obPoints;
          static constexpr const float modTagSize = .065*kInputConditioningScalar;
	  for (auto p : std::initializer_list<std::pair<int,int>>{{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}) {
	  //for (auto p : std::initializer_list<std::pair<int,int>>{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
	  //for (auto p : std::initializer_list<std::pair<int,int>>{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}) {
	    obPoints.push_back({modTagSize/2*p.first, modTagSize/2*p.second, 0});
          }

          std::vector<cv::Point2f> imPoints;
	  for (int j=0; j<p.size(); ++j) { // should be 4
            imPoints.push_back({(float)p[j].get_u(), (float)p[j].get_v()});
	  }

	  //for (int i=0; i<obPoints.size(); ++i) {
	  //  std::cout << "obPonts " << obPoints[i] << '\n';
	  //}
	  //for (int i=0; i<imPoints.size(); ++i) {
	  //  std::cout << "imPonts " << imPoints[i] << '\n';
	  //}
	  //std::cout << "cameraMatrix: " << cameraMatrix << '\n';
          bool res = estimatePoseSecure(obPoints, imPoints, cameraMatrix,
			                distCoeffs, s_rvec, s_tvec, true,
					aliceio, bobio);
	  if (!res) {
            std::cout << "pose estimation failed\n";
            continue;
	  }
          std::cout << "secure pose:\n" << s_rvec << s_tvec << std::endl;

          // convert to visp cmo_vec
	  vpPoseVector pose {s_tvec[0], s_tvec[1], s_tvec[2], s_rvec[0], s_rvec[1], s_rvec[2]};
	  cMo_vec[0].buildFrom(pose);

          if (display_on) {
            vpDisplay::displayFrame(I2, cMo_vec[0], cam, tagSize / 2 * kInputConditioningScalar, vpColor::none, 3);
	  }
        }

        t = vpTime::measureTimeMs() - t;
        time_vec.push_back(t);

        if (display_on) {
          vpHomogeneousMatrix cdMo(0, 0, Z_d * (secure ? kInputConditioningScalar : 1), 0, 0, 0);
          vpDisplay::displayFrame(I2, cdMo, cam, tagSize / 3, vpColor::red, 3);
          std::stringstream ss;
          ss << "Detection time: " << t << " ms";
          vpDisplay::displayText(I2, 40, 20, ss.str(), vpColor::red);
	}

        // Update Point 3D feature for control loop to use later
        {
          const std::lock_guard<std::mutex> lock(pose_mutex);
          X = cMo_vec[0][0][3];
          Y = cMo_vec[0][1][3];
          Z = cMo_vec[0][2][3];
          last_updated = std::chrono::steady_clock::now();
	}
        //std::cout << "X: " << X << " Z: " << Z << std::endl;
      }

      if (display_on) {
        vpDisplay::displayText(I2, 20, 20, "Click to quit.", vpColor::red);
        vpDisplay::flush(I2);
        if (display_on && save_image) {
          vpDisplay::getImage(I2, O);
          vpImageIo::write(O, "image.png");
        }
        if (vpDisplay::getClick(I2, false))
          break;
      }
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
              << " ; " << vpMath::getMedian(time_vec) << " ms"
              << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    // Let the control loop finish.
    cl.join();

    if (display_on)
      delete d;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }


  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#ifndef VISP_HAVE_V4L2
  std::cout << "ViSP is not build with v4l2 support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
  return EXIT_SUCCESS;
#endif
}
