/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <stdint.h>
#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include "udp_packet.h"

/* Initialise UNIX and UDP sockets used for publishing. */
void registerPub();

/* Publish current VIO odometry as a UDP_TYPE_ODOMETRY datagram. */
void pubOdometry(const Estimator &estimator);

/* Publish keyframe pose (UDP_TYPE_KF_POSE) for frame WINDOW_SIZE-2.
 * Should be called when solver_flag==NON_LINEAR && marginalization_flag==MARGIN_OLD. */
void pubKeyframePose(const Estimator &estimator);

/* Publish keyframe 3-D points with 2-D observations (UDP_TYPE_KF_POINT).
 * Should be called under the same condition as pubKeyframePose. */
void pubKeyframePoint(const Estimator &estimator);

/* Publish camera-to-IMU extrinsic calibration (UDP_TYPE_EXTRINSIC). */
void pubExtrinsic(const Estimator &estimator);

/* Publish the marginalised point cloud (UDP_TYPE_MARGIN_CLOUD, visualisation only).
 * Should be called when solver_flag==NON_LINEAR && marginalization_flag==MARGIN_OLD. */
void pubMarginCloud(const Estimator &estimator);

/* Store the latest received image frame in a thread-safe buffer.
 * Called from main.cpp whenever a new image arrives on /tmp/chobits_image.
 *   stamp    – image timestamp (seconds)
 *   width    – image width in pixels
 *   height   – image height in pixels
 *   encoding – 0 = mono8, 1 = jpeg
 *   data     – pointer to payload bytes
 *   bytes    – payload byte count */
void storeLatestImage(double stamp, uint32_t width, uint32_t height,
                      uint32_t encoding, const uint8_t *data, uint32_t bytes);

/* Send the stored image that is closest in time to `kf_stamp` as chunked
 * UDP_TYPE_IMAGE_CHUNK datagrams (tolerance 5 ms).  No-op if no image is
 * buffered or if no UDP client has registered yet.
 * Should be called under the same condition as pubKeyframePose. */
void pubKeyframeImage(double kf_stamp);
