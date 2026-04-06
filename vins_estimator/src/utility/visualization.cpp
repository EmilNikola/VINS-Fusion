/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <cstdint>
#include <vector>
#include <cstring>

static struct sockaddr_un chobits_addr, chobits_local_addr;
static int chobits_sock;
extern int pub_sock;
extern struct sockaddr_in pub_addr;

// -----------------------
// UDP message definitions
// -----------------------
// Goal: send the same *content* loop_fusion expects from ROS, but over UDP.
// (Loop_fusion will be modified later to decode these packets.)
//
// Packet = [VinsUdpHeader][payload]
// All numeric values are host-endian for now (same-host assumption).

#pragma pack(push, 1)
struct VinsUdpHeader {
    uint32_t magic;     // 'VINS' = 0x56494E53
    uint16_t version;   // 1
    uint16_t type;      // enum below
    double stamp;       // seconds
    uint32_t count;     // element count (points), or 0
};

struct VinsUdpKeyframePoint {
    float X, Y, Z;      // world point
    float nx, ny;       // normalized coords at keyframe
    float u, v;         // pixel coords at keyframe
    int32_t id;         // feature_id
};
#pragma pack(pop)

static constexpr uint32_t VINS_UDP_MAGIC = 0x56494E53; // 'VINS'
static constexpr uint16_t VINS_UDP_VERSION = 1;
enum : uint16_t {
    VINS_UDP_ODOM          = 1,
    VINS_UDP_KEYFRAME_POSE = 2,
    VINS_UDP_KEYFRAME_PTS  = 3,
    VINS_UDP_EXTRINSIC     = 4,
};

static inline bool udpClientReady()
{
    return pub_addr.sin_family == AF_INET;
}

static void udpSendPacket(uint16_t type, double stamp, uint32_t count,
                          const void *payload, size_t payload_bytes)
{
    if (!udpClientReady())
        return;

    VinsUdpHeader hdr{};
    hdr.magic = VINS_UDP_MAGIC;
    hdr.version = VINS_UDP_VERSION;
    hdr.type = type;
    hdr.stamp = stamp;
    hdr.count = count;

    std::vector<uint8_t> pkt;
    pkt.resize(sizeof(hdr) + payload_bytes);
    std::memcpy(pkt.data(), &hdr, sizeof(hdr));
    if (payload_bytes && payload) {
        std::memcpy(pkt.data() + sizeof(hdr), payload, payload_bytes);
    }

    if (sendto(pub_sock, pkt.data(), pkt.size(), 0,
               (struct sockaddr *)&pub_addr, sizeof(pub_addr)) < 0) {
        perror("udp sendto failed");
    }
}

void registerPub()
{
    memset(&chobits_addr, 0, sizeof(struct sockaddr_un));
    chobits_addr.sun_family = AF_UNIX;
    strcpy(chobits_addr.sun_path, "/tmp/chobits_server");
    memset(&chobits_local_addr, 0, sizeof(struct sockaddr_un));
    chobits_local_addr.sun_family = AF_UNIX;
    strcpy(chobits_local_addr.sun_path, "/tmp/chobits_1234");
    chobits_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    unlink("/tmp/chobits_1234");
    bind(chobits_sock, (struct sockaddr*)&chobits_local_addr, sizeof(chobits_local_addr));
}

void pubOdometry(const Estimator &estimator)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        double px = estimator.Ps[WINDOW_SIZE].x();
        double py = estimator.Ps[WINDOW_SIZE].y();
        double pz = estimator.Ps[WINDOW_SIZE].z();
        double vx = estimator.Vs[WINDOW_SIZE].x();
        double vy = estimator.Vs[WINDOW_SIZE].y();
        double vz = estimator.Vs[WINDOW_SIZE].z();

        Eigen::Quaterniond q = Eigen::Quaterniond(estimator.Rs[WINDOW_SIZE]);
        double qx = q.x();
        double qy = q.y();
        double qz = q.z();
        double qw = q.w();

        float chobits_msg[10] = { (float)qw, (float)qx, (float)qy, (float)qz, (float)px, (float)py, (float)pz, (float)vx, (float)vy, (float)vz };
        sendto(chobits_sock, chobits_msg, sizeof(chobits_msg), 0, (struct sockaddr*)&chobits_addr, sizeof(chobits_addr));

        // UDP: loop-fusion-friendly odometry payload (pose+vel).
        // payload = [px,py,pz,qx,qy,qz,qw,vx,vy,vz]
        double payload[10] = {px, py, pz, qx, qy, qz, qw, vx, vy, vz};
        // Use estimator header stamp if available
        double stamp = 0;
        stamp = estimator.Headers[WINDOW_SIZE];
        udpSendPacket(VINS_UDP_ODOM, stamp, 0, payload, sizeof(payload));
    }
}

void pubKeyframeUdp(const Estimator &estimator)
{
    // Mirror original ROS pubKeyframe() trigger:
    // publish only on keyframe (marginalization_flag == 0) after initialization.
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    if (estimator.marginalization_flag != 0)
        return;

    const int kf_idx = WINDOW_SIZE - 2;
    if (kf_idx < 0)
        return;

    const double ts = estimator.Headers[kf_idx];

    // KEYFRAME_POSE UDP
    {
        Eigen::Vector3d P = estimator.Ps[kf_idx];
        Eigen::Quaterniond Q = Eigen::Quaterniond(estimator.Rs[kf_idx]);
        double payload_pose[7] = {P.x(), P.y(), P.z(), Q.x(), Q.y(), Q.z(), Q.w()};
        udpSendPacket(VINS_UDP_KEYFRAME_POSE, ts, 0, payload_pose, sizeof(payload_pose));
    }

    // KEYFRAME_POINTS UDP
    std::vector<VinsUdpKeyframePoint> out;
    out.reserve(1024);
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        const int frame_size = (int)it_per_id.feature_per_frame.size();
        if (it_per_id.solve_flag != 1)
            continue;
        if (!(it_per_id.start_frame < kf_idx &&
              it_per_id.start_frame + frame_size - 1 >= kf_idx))
            continue;

        const int imu_i = it_per_id.start_frame;
        const int imu_j = kf_idx - it_per_id.start_frame;
        const double depth = it_per_id.estimated_depth;
        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * depth;
        Eigen::Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        VinsUdpKeyframePoint p{};
        p.X = (float)w_pts_i.x();
        p.Y = (float)w_pts_i.y();
        p.Z = (float)w_pts_i.z();
        p.nx = (float)it_per_id.feature_per_frame[imu_j].point.x();
        p.ny = (float)it_per_id.feature_per_frame[imu_j].point.y();
        p.u  = (float)it_per_id.feature_per_frame[imu_j].uv.x();
        p.v  = (float)it_per_id.feature_per_frame[imu_j].uv.y();
        p.id = (int32_t)it_per_id.feature_id;
        out.push_back(p);
    }

    udpSendPacket(VINS_UDP_KEYFRAME_PTS, ts, (uint32_t)out.size(), out.empty() ? nullptr : out.data(), out.size() * sizeof(VinsUdpKeyframePoint));
}

void pubExtrinsicUdp(const Estimator &estimator, double stamp)
{
    Eigen::Vector3d t = estimator.tic[0];
    Eigen::Quaterniond q(estimator.ric[0]);
    double payload[7] = {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
    udpSendPacket(VINS_UDP_EXTRINSIC, stamp, 0, payload, sizeof(payload));
}
