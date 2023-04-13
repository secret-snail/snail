#pragma once
#include <stdint.h>

// Algorithm control flow
//#define PPL_FLOW_NATIVE     1
#define PPL_FLOW_DO         2
#define PPL_FLOW_LOOP_LEAK  3
#define PPL_FLOW_SiSL     4

//#define PPL_FLOW PPL_FLOW_DO
#define PPL_FLOW PPL_FLOW_SiSL

// maximum number of iterations
//calibration.cpp - cvFindExtrinsicCameraParams2() = 20
const int GN_MAX_ITR=30;

//const float JACOB_EPSILON=1e-5; //hoff's points
//const float JACOB_EPSILON=0x0.00005fp0; //or 0x0.000040p0 for ETH3D dataset
//const float JACOB_EPSILON=0x0.000007p0; // works with charuco snail
//const float MIN_ER=1e-2; // pose L2 norm changed by less than this considered converged (charuco snail)

// Works for snail and hoffs test points for Loop Leak and SiSL
const float JACOB_EPSILON=0x0.0000bfp0;
const float MIN_ER=1e-2;

const float GT_MIN_ER=1; // (pose - ground truth) L2 norm less than this considered correct

const int LM_MAX_ITR=30;
const float LM_LAMBDA_INIT=1e-3;
//const float LM_LAMBDA_MAX=1e5; // opencv default, works with hoff and ETH3D
const float LM_LAMBDA_MAX=1e2; // snail with aruco markers
const float LM_LAMBDA_MIN=1e-5;

// Debugging tools
#define DBG_FLOW 0x01
#define DBG_PROJECT 0x02
#define DBG_JACOB 0x04
#define DBG_POSE_UPDATE 0x08
#define DBG_ER 0x10
#define DBG_ARGS 0x20
#define DBG_LAMBDA 0x40
const uint32_t _verbosity=0x00;
const bool printints=false; // used for ABY, their debug prints all floats as ints

