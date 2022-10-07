//! \example mbot-apriltag-pbvs.cpp
#include <visp3/core/vpSerial.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/vs/vpServo.h>

#include <emp_client.h>

#include <emp-tool/io/net_io_channel.h>
#include <emp-tool/utils/constants.h>

const std::string calibFile="/home/pi/snail/calib.txt";
#define OFFLOAD_IP "68.74.215.161"

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

int main(int argc, const char **argv)
{
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

    vpServo task;
    vpAdaptiveGain lambda;
    if (display_on)
      lambda.initStandard(2.5, 0.4, 30); // lambda(0)=2.5, lambda(oo)=0.4 and lambda'(0)=30
    else
      lambda.initStandard(4, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30

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

    std::cout << "eJe: \n" << eJe << std::endl;

    // opencv's cameraMatrix does not work here
    cv::Mat cameraMatrix, distCoeffs;
    bool readOk = readCameraParameters(calibFile, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera calibration file: " << calibFile << std::endl;
        return 1;
    }
    //cameraMatrix = cv::Mat::zeros(3, 3, cv::DataType<float>::type);
    //cameraMatrix.at<float>(0,0) = 615.1674805;
    //cameraMatrix.at<float>(1,1) = 615.1674805;
    //cameraMatrix.at<float>(0,2) = (I2.getWidth() / 2.);
    //cameraMatrix.at<float>(1,2) = (I2.getHeight() / 2.);
    //cameraMatrix.at<float>(2,2) = 1.0f;

    // Desired distance to the target
    double Z_d = 0.4;
    double X = 0, Y = 0, Z = Z_d;

    cv::Vec3d s_rvec={0,0,0}, s_tvec={0,0,1};

    // Create X_3D visual features
    vpFeaturePoint3D s_XZ, s_XZ_d;
    s_XZ.buildFrom(0, 0, Z_d);
    s_XZ_d.buildFrom(0, 0, Z_d);

    // Create Point 3D X, Z coordinates visual features
    s_XZ.buildFrom(X, Y, Z);
    s_XZ_d.buildFrom(0, 0, Z_d); // The value of s* is X=Y=0 and Z=Z_d meter

    // Add the features
    task.addFeature(s_XZ, s_XZ_d, vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectZ());

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

    std::vector<double> time_vec;
    for (;;) {
      g.acquire(I);

      // rotate image
      vpMatrix M(2, 3);
      M.eye();
      const double theta = vpMath::rad(180);
      M[0][0] = cos(theta);   M[0][1] = -sin(theta);   M[0][2] = I.getWidth();
      M[1][0] = sin(theta);   M[1][1] = cos(theta);    M[1][2] = I.getHeight();
      //M[0][0] = cos(theta);   M[0][1] = -sin(theta);   M[0][2] = 0;
      //M[1][0] = sin(theta);   M[1][1] = cos(theta);    M[1][2] = 0;
      vpImageTools::warpImage(I, M, I2);

      vpDisplay::display(I2);

      double t = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec; // JIM: vector of tags, each with 4(?) points

      // calling detect with cam and cMo does pose estimation
      //detector.detect(I2, tagSize, cam, cMo_vec);
      //std::cout << "cMo_vec from detect:\n";
      //for (unsigned int i = 0; i < cMo_vec[0].getRows(); i++) {
      //  for (unsigned int j = 0; j < cMo_vec[0].getCols(); j++) {
      //    std::cout << cMo_vec[0][i][j] << " ";
      //  }
      //  std::cout << std::endl;
      //}
      //std::cout << std::endl;

      // should be equivalent to detect(I2) (just image) then getPose
      detector.detect(I2);
      cMo_vec.push_back({});
      if (detector.getNbObjects() == 1) {
      	bool ret = detector.getPose(0, tagSize, cam, cMo_vec[0]);
	if (!ret) {
            std::cout << "pose detection failed\n";
	    continue;
	}
        std::cout << "cMo_vec from getPose:\n";
        for (unsigned int i = 0; i < cMo_vec[0].getRows(); i++) {
          for (unsigned int j = 0; j < cMo_vec[0].getCols(); j++) {
            std::cout << cMo_vec[0][i][j] << " ";
          }
          std::cout << std::endl;
        }
        std::cout << std::endl;
      }

      //std::vector<vpImagePoint> p = detector.getPolygon(i); // todo do i need this?

      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      {
        std::stringstream ss;
        ss << "Detection time: " << t << " ms";
        vpDisplay::displayText(I2, 40, 20, ss.str(), vpColor::red);
      }

      if (detector.getNbObjects() == 1) {
        vpDisplay::displayFrame(I2, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);

	if (secure) {
	  std::vector<vpImagePoint> p = detector.getPolygon(0);

          std::vector<cv::Point3f> obPoints;
          static constexpr const float modTagSize = .055;
	  obPoints.push_back({0, 0, 0});
	  obPoints.push_back({modTagSize, 0, 0});
	  obPoints.push_back({modTagSize, modTagSize, 0});
	  obPoints.push_back({0, modTagSize, 0});
	  //obPoints.push_back({modTagSize/2, modTagSize/2, 0}); // fake center point

          std::vector<cv::Point2f> imPoints;
          //float usum=0.0f, vsum=0.0f;
	  for (int j=0; j<p.size(); ++j) { // should be 4
            imPoints.push_back({(float)p[j].get_i(), (float)p[j].get_j()}); // TODO should these be i,j vs u,v?
            //usum+= p[j].get_u();
            //vsum+= p[j].get_v();
	  }
          //imPoints.push_back({usum/4, vsum/4}); // fake center point

	  for (int i=0; i<obPoints.size(); ++i) {
	    std::cout << "obPonts " << obPoints[i] << '\n';
	  }
	  for (int i=0; i<imPoints.size(); ++i) {
	    std::cout << "imPonts " << imPoints[i] << '\n';
	  }
	  std::cout << "cameraMatrix: " << cameraMatrix << '\n';
          bool res = estimatePoseSecure(obPoints, imPoints, cameraMatrix,
			                distCoeffs, s_rvec, s_tvec, true,
					aliceio, bobio);
	  if (!res) {
            std::cout << "pose estimation failed\n";
            continue;
	  }
          std::cout << "secure pose:\n"
              << s_rvec << s_tvec
              << std::endl;

          // convert rot vector to visp cmo_vec
          vpRxyzVector rxyz{s_rvec[0], s_rvec[1], s_rvec[2]};
          vpRotationMatrix R(rxyz);
          for (int i=0; i<3; ++i) {
            for (int j=0; j<3; ++j) {
              cMo_vec[0][i][j] = R[i][j];
            }
            cMo_vec[0][i][3] = s_tvec[i];
          }
          std::cout << "cMo_vec from SECURE getPose:\n";
          for (unsigned int i = 0; i < cMo_vec[0].getRows(); i++) {
            for (unsigned int j = 0; j < cMo_vec[0].getCols(); j++) {
              std::cout << cMo_vec[0][i][j] << " ";
            }
            std::cout << std::endl;
          }
          std::cout << std::endl;
	}

        // Display visual features
        vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
        vpDisplay::displayFrame(I2, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);
        vpDisplay::displayFrame(I2, cdMo, cam, tagSize / 3, vpColor::red, 3);

	// JIM: why last index 3 here? what are 1 and 2?
	// is 3 the pose estimate after localization?
        X = cMo_vec[0][0][3];
        Y = cMo_vec[0][1][3];
        Z = cMo_vec[0][2][3];

        // Update Point 3D feature
        s_XZ.set_XYZ(X, Y, Z);

        std::cout << "X: " << X << " Z: " << Z << std::endl;

        std::cout << "updated\n";
        task.set_cVe(cVe);
        task.set_eJe(eJe);

        // Compute the control law. Velocities are computed in the mobile robot reference frame
        vpColVector v = task.computeControlLaw();

        std::cout << "Send velocity to the mbot: " << v[0] << " m/s " << vpMath::deg(v[1]) << " deg/s" << std::endl;

        task.print();
        double radius = 0.0325;
        double L = 0.0725;
        double motor_left = (-v[0] - L * v[1]) / radius;
        double motor_right = (v[0] - L * v[1]) / radius;
        std::cout << "motor left vel: " << motor_left << " motor right vel: " << motor_right << std::endl;
        std::stringstream ss;
        double rpm_left = motor_left * 30. / M_PI;
        double rpm_right = motor_right * 30. / M_PI;
        ss << "MOTOR_RPM=" << vpMath::round(rpm_left) << "," << vpMath::round(rpm_right) << "\n";
        std::cout << "Send: " << ss.str() << std::endl;
      } else {
        // stop the robot
	// TODO
      }

      vpDisplay::displayText(I2, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I2);
      if (display_on && save_image) {
        vpDisplay::getImage(I2, O);
        vpImageIo::write(O, "image.png");
      }
      if (vpDisplay::getClick(I2, false))
        break;
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
              << " ; " << vpMath::getMedian(time_vec) << " ms"
              << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

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
