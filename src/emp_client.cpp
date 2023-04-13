#include <emp_client.h>

#include <privacyconf.h>

//#include <emp-tool/io/io_channel.h>
#include <emp-tool/io/net_io_channel.h>
#include <emp-tool/utils/block.h>
//#include <emp-tool/utils/constants.h>
//#include <emp-tool/utils/hash.h>
#include <emp-tool/utils/prg.h>
//#include <emp-tool/utils/aes.h>
#include <emp-tool/utils/utils.h>

//#include <emp-tool/emp-tool.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/aruco.hpp"
#include <opencv2/calib3d.hpp>

emp::PRG prg;
emp::block delta;

// copied from charuco.cpp
/**
  * Check if a set of 3d points are enough for calibration. Z coordinate is ignored.
  * Only axis parallel lines are considered
  */
static bool arePointsEnoughForPoseEstimation(const std::vector< cv::Point3f > &points) {

    if(points.size() < 4) return false;

    std::vector< double > sameXValue; // different x values in points
    std::vector< int > sameXCounter;  // number of points with the x value in sameXValue
    for(unsigned int i = 0; i < points.size(); i++) {
        bool found = false;
        for(unsigned int j = 0; j < sameXValue.size(); j++) {
            if(sameXValue[j] == points[i].x) {
                found = true;
                sameXCounter[j]++;
            }
        }
        if(!found) {
            sameXValue.push_back(points[i].x);
            sameXCounter.push_back(1);
        }
    }

    // count how many x values has more than 2 points
    int moreThan2 = 0;
    for(unsigned int i = 0; i < sameXCounter.size(); i++) {
        if(sameXCounter[i] >= 2) moreThan2++;
    }

    // if we have more than 1 two xvalues with more than 2 points, calibration is ok
    if(moreThan2 > 1)
        return true;
    else
        return false;
}


void p128_hex_u32(__m128i in) {
    alignas(16) uint32_t v[4];
    _mm_store_si128((__m128i*)v, in);
    printf("v4_u32: %x %x %x %x\n", v[0], v[1], v[2], v[3]);
}

void sendFloatBlock(emp::NetIO* io, float f) {
    int *in = (int*)&f;
    bool* b = new bool[32];
    emp::int_to_bool<int>(b, *in, 32);

    emp::block label[32];
    prg.random_block(label, 32);
    uint32_t mask = 1;
    for (int i = 0; i < 32; ++i) {
        if(b[i]) {
            label[i] = label[i] ^ delta;
        }
    }
    io->send_block(label, 32);
    //p128_hex_u32(label[0]);
    io->flush();
}

void recvFloatBlock(emp::NetIO* aliceio, emp::NetIO* bobio, float* f) {
    bool alicelsb = 0;
    bool boblsb = 0;
    uint32_t res = 0;
    for (int i = 0; i < 32; ++i) {
        aliceio->recv_data(&alicelsb, 1);
        bobio->recv_data(&boblsb, 1);
        uint32_t tmp = alicelsb ^ boblsb;
        res |= (tmp<<i);
    }
    float *fp = (float*)(&res);
    *f = *fp;
}

bool estimatePoseSecure(std::vector<cv::Point3f> &objPoints, std::vector<cv::Point2f> &imPoints,
                        cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                        cv::InputOutputArray _rvec, cv::InputOutputArray _tvec, bool useExtrinsicGuess,
                        emp::NetIO *aliceio, emp::NetIO *bobio) {


    // points need to be in different lines, check if detected points are enough
    if(!arePointsEnoughForPoseEstimation(objPoints)) return false;

    // the cleartext solution
    //solvePnP(objPoints, imPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);

    uint32_t numPts = objPoints.size();

#if PPL_FLOW == PPL_FLOW_SiSL
    bool done = false;
    float lambda = LM_LAMBDA_INIT;
    float err = std::numeric_limits<float>::max();
    while (!done) {
#endif
      std::cout << std::endl;

      // Setup
      aliceio->send_data(&numPts, sizeof(uint32_t));
      bobio->send_data(&numPts, sizeof(uint32_t));
      aliceio->flush();
      bobio->flush();
      std::cout << "sent alice and bob numpoints="<<numPts<<"\n";

      emp::block seed;
      prg.random_block(&seed, 1); // used for alice's inputs only
      prg.reseed(&seed);

      aliceio->recv_block(&delta, 1);
      std::cout << "got delta from alice\n";
      //p128_hex_u32(delta);


      // Distribute secret input data via 3-way OT
      // send prg seed directly to bob so he can generate his own labels
      bobio->send_block(&seed, 1);
      bobio->flush();
      std::cout << "sent bob seed\n";
      //p128_hex_u32(seed);

      // must send all labels ^ gc->delta to alice, she cannot learn seed
      for(int i=0; i<numPts; i++) {
          sendFloatBlock(aliceio, objPoints[i].x);
          sendFloatBlock(aliceio, objPoints[i].y);
          sendFloatBlock(aliceio, objPoints[i].z);
          //sendFloatBlock(aliceio, 1.0f); // fix homog coords div by zero

          sendFloatBlock(aliceio, imPoints[i].x);
          sendFloatBlock(aliceio, imPoints[i].y);
      }
      sendFloatBlock(aliceio, _cameraMatrix.getMat().at<double>(0,0));
      sendFloatBlock(aliceio, _cameraMatrix.getMat().at<double>(0,2));
      sendFloatBlock(aliceio, _cameraMatrix.getMat().at<double>(1,2));
      for (int i=0; i<3; i++) {
          sendFloatBlock(aliceio, _rvec.getMat().at<double>(i));
      }
      for (int i=0; i<3; i++) {
          sendFloatBlock(aliceio, _tvec.getMat().at<double>(i));
      }

#if PPL_FLOW == PPL_FLOW_SiSL
      sendFloatBlock(aliceio, lambda);
#endif

      aliceio->flush();
      std::cout << "sent alice labels\n";

      // Offload servers run computation...
      std::cout << "Waiting for computation to finish...\n";

      // Collect result
      for (int i=0; i<3; i++) {
          float tmp;
          recvFloatBlock(aliceio, bobio, &tmp);
          _rvec.getMat().at<double>(i) = tmp;
          //std::cout << _rvec.getMat().at<double>(i) << " ";
      }
      std::cout << "got rvec, waiting on tvec\n";
      for (int i=0; i<3; i++) {
          float tmp;
          recvFloatBlock(aliceio, bobio, &tmp);
          _tvec.getMat().at<double>(i) = tmp;
          //std::cout << _tvec.getMat().at<double>(i) << " ";
      }

#if PPL_FLOW == PPL_FLOW_SiSL
      std::cout << "wating on extra err to be returned\n";
      float newerr;
      recvFloatBlock(aliceio, bobio, &newerr);
      if (err < MIN_ER) {
        done = true;
      }
      if( newerr > err ) lambda *= 10;
      else lambda /= 10;
      lambda = MIN(lambda, LM_LAMBDA_MAX);
      lambda = MAX(lambda, LM_LAMBDA_MIN);
      err = newerr;

      std::cout << "intermediate pose:\n" << _rvec.getMat().t() << _tvec.getMat().t() << '\n';
      std::cout << "intermediate err:\n" << err << '\n';
    }
#endif

    return true;
}
