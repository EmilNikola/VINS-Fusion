/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include "../estimator/estimator.h"
#include "../estimator/parameters.h"

void registerPub();
void pubOdometry(const Estimator &estimator);

// UDP payloads intended to carry the same *content* loop_fusion consumed from
// ROS topics:
//   - keyframe_pose (pose at WINDOW_SIZE-2)
//   - keyframe_point (world XYZ + per-point [nx, ny, u, v, id])
//   - extrinsic (tic/qic)
//
// They are sent only if a UDP "debug client" has registered itself by pinging
// the estimator's UDP port (see main.cpp which populates pub_addr).
void pubKeyframeUdp(const Estimator &estimator);
void pubExtrinsicUdp(const Estimator &estimator, double stamp);
// UDP payloads intended to carry the same *content* loop_fusion consumed from
// ROS topics:
void pubKeyframeUdp(const Estimator &estimator);
void pubExtrinsicUdp(const Estimator &estimator, double stamp);