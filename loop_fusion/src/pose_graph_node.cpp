/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <sys/un.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "keyframe.h"
#include "pose_graph.h"
#include "parameters.h"

#define SKIP_FIRST_CNT 10

using namespace std;

// -----------------------
// Global algorithm state
// -----------------------
static std::mutex m_buf;
static std::mutex m_process;
static int frame_index  = 0;
static int sequence = 1;
static PoseGraph posegraph;
static int skip_first_cnt = 0;
static int SKIP_CNT = 0;
static int skip_cnt = 0;
static double SKIP_DIS = 0;
static Eigen::Vector3d last_t(-100, -100, -100);

// parameters.h globals (definitions)
int VISUALIZATION_SHIFT_X = 0;
int VISUALIZATION_SHIFT_Y = 0;
int ROW = 0;
int COL = 0;
int DEBUG_IMAGE = 0;
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;

// -----------------------
// UDP message definitions
// (must match vins_estimator/src/utility/visualization.cpp)
// -----------------------
#pragma pack(push, 1)
struct VinsUdpHeader {
    uint32_t magic;     // 'VINS' = 0x56494E53
    uint16_t version;   // 1
    uint16_t type;      // enum
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

// Optional: VFRM image packet (your earlier format)
#pragma pack(push, 1)
struct VfrmHeader {
    uint32_t magic;   // 'VFRM' = 0x5646524D
    uint16_t version; // 1
    uint16_t flags;   // 0
    double stamp;     // seconds
    uint32_t width;
    uint32_t height;
    uint32_t format;  // 0=mono8
    uint32_t data_len;
};
#pragma pack(pop)
static constexpr uint32_t VFRM_MAGIC = 0x5646524D; // 'VFRM'

// -----------------------
// Incoming buffers (UDP)
// -----------------------
struct PoseMsg {
    double stamp = 0;
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};
struct PointsMsg {
    double stamp = 0;
    std::vector<VinsUdpKeyframePoint> pts;
};
struct ImageMsg {
    double stamp = 0;
    cv::Mat mono; // CV_8UC1
};

static std::queue<PoseMsg> pose_buf;
static std::queue<PointsMsg> points_buf;
static std::queue<ImageMsg> image_buf;

static std::atomic<bool> running{true};
static std::atomic<bool> got_first_vins_packet{false};

static void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        fprintf(stderr, "only support 5 sequences.\n");
        std::abort();
    }
    posegraph.posegraph_visualization->reset();

    std::lock_guard<std::mutex> lk(m_buf);
    while(!image_buf.empty()) image_buf.pop();
    while(!points_buf.empty()) points_buf.pop();
    while(!pose_buf.empty()) pose_buf.pop();
}

static int createUdpSocketBound(int port)
{
    int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket");
        return -1;
    }
    int reuse = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }
    return s;
}

static void sendPingRegister(int sock, const std::string& estimator_ip, int estimator_port)
{
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)estimator_port);
    if (inet_aton(estimator_ip.c_str(), &dst.sin_addr) == 0) {
        fprintf(stderr, "inet_aton failed for %s\n", estimator_ip.c_str());
        return;
    }
    const char ping[4] = {'P','I','N','G'};
    if (sendto(sock, ping, sizeof(ping), 0, (sockaddr*)&dst, sizeof(dst)) < 0) {
        perror("sendto ping");
    } else {
        printf("sent ping-register to %s:%d from local udp socket\n", estimator_ip.c_str(), estimator_port);
    }
}

static void udpReceiverVins(int sock)
{
    std::vector<uint8_t> buf;
    buf.resize(65536);

    while (running.load())
    {
        ssize_t n = recv(sock, buf.data(), buf.size(), 0);
        if (n <= 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if ((size_t)n < sizeof(VinsUdpHeader))
            continue;

        VinsUdpHeader hdr{};
        std::memcpy(&hdr, buf.data(), sizeof(hdr));
        if (hdr.magic != VINS_UDP_MAGIC || hdr.version != VINS_UDP_VERSION)
            continue;
        got_first_vins_packet.store(true);

        const uint8_t* payload = buf.data() + sizeof(VinsUdpHeader);
        const size_t payload_len = (size_t)n - sizeof(VinsUdpHeader);

        if (hdr.type == VINS_UDP_EXTRINSIC) {
            if (payload_len < 7 * sizeof(double))
                continue;
            double v[7];
            std::memcpy(v, payload, sizeof(v));
            // [tx,ty,tz,qx,qy,qz,qw]
            std::lock_guard<std::mutex> lk(m_process);
            tic = Eigen::Vector3d(v[0], v[1], v[2]);
            Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
            qic = q.toRotationMatrix();
        }
        else if (hdr.type == VINS_UDP_KEYFRAME_POSE) {
            if (payload_len < 7 * sizeof(double))
                continue;
            double v[7];
            std::memcpy(v, payload, sizeof(v));
            PoseMsg pm;
            pm.stamp = hdr.stamp;
            pm.t = Eigen::Vector3d(v[0], v[1], v[2]);
            pm.q = Eigen::Quaterniond(v[6], v[3], v[4], v[5]); // w, x, y, z

            std::lock_guard<std::mutex> lk(m_buf);
            pose_buf.push(std::move(pm));
        }
        else if (hdr.type == VINS_UDP_KEYFRAME_PTS) {
            const size_t want = (size_t)hdr.count * sizeof(VinsUdpKeyframePoint);
            if (payload_len < want)
                continue;

            PointsMsg pts;
            pts.stamp = hdr.stamp;
            pts.pts.resize(hdr.count);
            if (hdr.count > 0)
                std::memcpy(pts.pts.data(), payload, want);

            std::lock_guard<std::mutex> lk(m_buf);
            points_buf.push(std::move(pts));
        }
        // ODOM currently ignored by loop_fusion (old node used /vins_estimator/odometry for viz only)
    }
}

static void unixReceiverVfrm(int sock)
{
    std::vector<uint8_t> buf;
    buf.resize(65536);
    while (running.load())
    {
        ssize_t n = recv(sock, buf.data(), buf.size(), 0);
        if (n <= 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if ((size_t)n < sizeof(VfrmHeader))
            continue;

        VfrmHeader hdr{};
        std::memcpy(&hdr, buf.data(), sizeof(hdr));
        if (hdr.magic != VFRM_MAGIC)
            continue;
        if (hdr.data_len + sizeof(VfrmHeader) > (size_t)n)
            continue;
        if (hdr.format != 0) // mono8 only
            continue;

        const uint8_t* payload = buf.data() + sizeof(VfrmHeader);
        cv::Mat mono((int)hdr.height, (int)hdr.width, CV_8UC1);
        std::memcpy(mono.data, payload, hdr.data_len);

        ImageMsg im;
        im.stamp = hdr.stamp;
        im.mono = mono;

        std::lock_guard<std::mutex> lk(m_buf);
        image_buf.push(std::move(im));
    }
}

static int createUnixDgramSocketBound(const char* path)
{
    int s = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (s < 0) {
        perror("socket(AF_UNIX)");
        return -1;
    }

    // Remove old socket file if present
    ::unlink(path);

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind(AF_UNIX)");
        close(s);
        return -1;
    }
    return s;
}

static bool tryPopSynced(double &stamp, PoseMsg &pose, PointsMsg &pts, ImageMsg &img, bool require_image)
{
    std::lock_guard<std::mutex> lk(m_buf);
    if (pose_buf.empty() || points_buf.empty())
        return false;
    if (require_image && image_buf.empty())
        return false;

    // We assume the estimator uses the same timestamp for KEYFRAME_POSE and KEYFRAME_PTS (it does in your code).
    const double ts_pose = pose_buf.front().stamp;
    const double ts_pts  = points_buf.front().stamp;
    if (std::fabs(ts_pose - ts_pts) > 1e-6) {
        // drop older one
        if (ts_pose < ts_pts) pose_buf.pop();
        else points_buf.pop();
        return false;
    }

    if (require_image) {
        // Align image buffer to timestamp (best-effort).
        while (!image_buf.empty() && image_buf.front().stamp + 1e-6 < ts_pose)
            image_buf.pop();
        if (image_buf.empty())
            return false;
        if (std::fabs(image_buf.front().stamp - ts_pose) > 0.02) {
            // no close image yet
            return false;
        }
        img = std::move(image_buf.front());
        image_buf.pop();
    } else {
        img = ImageMsg{};
        img.stamp = ts_pose;
        img.mono = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    }

    pose = std::move(pose_buf.front()); pose_buf.pop();
    pts  = std::move(points_buf.front()); points_buf.pop();
    stamp = ts_pose;
    return true;
}

static void processLoop(bool require_image)
{
    while (running.load())
    {
        double stamp = 0;
        PoseMsg pose;
        PointsMsg pts;
        ImageMsg img;
        if (!tryPopSynced(stamp, pose, pts, img, require_image)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // skip first few
        if (skip_first_cnt < SKIP_FIRST_CNT) {
            skip_first_cnt++;
            continue;
        }

        if (skip_cnt < SKIP_CNT) {
            {
                std::lock_guard<std::mutex> lk(m_buf);
                if (pose_buf.empty() || points_buf.empty())
                    return false;
                if (image_buf.empty())
                    return false;
        Eigen::Vector3d T = pose.t;
        Eigen::Matrix3d R = pose.q.toRotationMatrix();

        if ((T - last_t).norm() <= SKIP_DIS)
            continue;

        std::vector<cv::Point3f> point_3d;
        std::vector<cv::Point2f> point_2d_uv;
        std::vector<cv::Point2f> point_2d_normal;
        std::vector<double> point_id;

        point_3d.reserve(pts.pts.size());
        point_2d_uv.reserve(pts.pts.size());
        point_2d_normal.reserve(pts.pts.size());
        point_id.reserve(pts.pts.size());

        for (const auto &p : pts.pts)
        {
            point_3d.emplace_back(p.X, p.Y, p.Z);
            point_2d_normal.emplace_back(p.nx, p.ny);
            point_2d_uv.emplace_back(p.u, p.v);
            point_id.push_back((double)p.id);
        }

        cv::Mat image = img.mono;
        if (image.empty()) {
            image = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
        }

        KeyFrame* keyframe = new KeyFrame(stamp, frame_index, T, R, image,
                                          point_3d, point_2d_uv, point_2d_normal, point_id, sequence);

        m_process.lock();
        posegraph.addKeyFrame(keyframe, /*detect_loop=*/1);
        m_process.unlock();

        frame_index++;
        last_t = T;

        // Optional: drain visualization markers (now ROS-less).
        // If you have a UDP visualizer, send these out here.
        auto markers = posegraph.consumeVisualizationMarkers();
        (void)markers;
    }
}

static void commandThread()
{
    while (running.load())
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\n");
            printf("program shutting down...\n");
            running.store(false);
        }
        if (c == 'n')
            new_sequence();

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: loop_fusion_node <config_file.yaml>\n");
        return 0;
    }

    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return 1;
    }

    ROW = (int)fsSettings["image_height"];
    COL = (int)fsSettings["image_width"];

    // Find support_files relative to repo layout:
    // config/.../xxx.yaml -> repo_root/config/.../xxx.yaml
    // so repo_root = path prefix up to /config
    std::string repo_root;
    {
        auto pos = config_file.find("/config/");
        if (pos != std::string::npos) repo_root = config_file.substr(0, pos);
        else {
            // fallback: assume current working directory is repo root
            repo_root = ".";
        }
    }
    std::string vocabulary_file = repo_root + "/support_files/brief_k10L6.bin";
    posegraph.loadVocabulary(vocabulary_file);
    BRIEF_PATTERN_FILE = repo_root + "/support_files/brief_pattern.yml";

    int pn = (int)config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;
    int LOAD_PREVIOUS_POSE_GRAPH = (int)fsSettings["load_previous_pose_graph"];
    int USE_IMU = (int)fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
    }
    else
    {
        printf("no previous pose graph\n");
    }

    // UDP setup
    const std::string estimator_ip = "127.0.0.1";
    const int estimator_port = 8800;  // estimator bind port
    const int local_vins_port = 8801; // loop_fusion receives VINS packets here
    const char* local_vfrm_path = "/tmp/chobits_frames"; // VFRM frames from camera program (AF_UNIX DGRAM)

    int vins_sock = createUdpSocketBound(local_vins_port);
    if (vins_sock < 0) return 1;

    // ping-register from the same socket we will receive on.
    // Robustness: keep pinging until we receive the first valid VINS packet
    // (otherwise "start loop_fusion before estimator" can stall indefinitely).
    std::thread t_ping([&](){
        while (running.load() && !got_first_vins_packet.load()) {
            sendPingRegister(vins_sock, estimator_ip, estimator_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    // Image receiver is mandatory
    int vfrm_sock = createUnixDgramSocketBound(local_vfrm_path);
    if (vfrm_sock < 0) {
        fprintf(stderr, "failed to bind VFRM unix socket %s\n", local_vfrm_path);
        return 1;
    }
    printf("listening VFRM on unix dgram %s\n", local_vfrm_path);
    printf("listening VINS on udp %d\n", local_vins_port);

    std::thread t_vins(udpReceiverVins, vins_sock);
    std::thread t_vfrm(unixReceiverVfrm, vfrm_sock);

    std::thread t_process(processLoop);
    std::thread t_cmd(commandThread);

    t_cmd.join();
    running.store(false);

    if (t_ping.joinable()) t_ping.join();

    // allow receiver threads to exit
    shutdown(vins_sock, SHUT_RDWR);
    close(vins_sock);
    if (vfrm_sock >= 0) {
        // For AF_UNIX DGRAM, shutdown is optional; close is enough.
        close(vfrm_sock);
        // Clean up socket file so the next run can bind without issues.
        ::unlink(local_vfrm_path);
    }

    if (t_vins.joinable()) t_vins.join();
    if (t_vfrm.joinable()) t_vfrm.join();
    if (t_process.joinable()) t_process.join();

    return 0;
}
