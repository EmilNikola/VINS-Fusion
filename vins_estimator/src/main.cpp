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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#define IMU_SOCK_PATH      "/tmp/chobits_imu"
#define FEATURES_SOCK_PATH "/tmp/chobits_features"
#define IMAGE_SOCK_PATH    "/tmp/chobits_image"

/*
 * Image IPC frame format (received on IMAGE_SOCK_PATH):
 *   header  (24 bytes):
 *     double   timestamp_sec   [8]
 *     uint32_t width           [4]
 *     uint32_t height          [4]
 *     uint32_t encoding        [4]   0=mono8  1=jpeg
 *     uint32_t payload_bytes   [4]   byte count of the image data
 *   followed by payload_bytes of image data
 *
 * Maximum supported payload: IMAGE_MAX_PAYLOAD bytes.
 * Datagrams larger than IMAGE_SOCK_BUF_SZ are silently discarded.
 */
#define IMAGE_HDR_SZ     24u                          /* bytes: 8+4+4+4+4 */
#define IMAGE_MAX_PAYLOAD (4u * 1024u * 1024u)        /* 4 MB  */
#define IMAGE_SOCK_BUF_SZ (IMAGE_HDR_SZ + IMAGE_MAX_PAYLOAD)

#define BUF_SZ (12*1024)

bool gogogo = true;
double buf[BUF_SZ/sizeof(double)];
int pub_sock = 0;
struct sockaddr_in pub_addr;

/* Scratch buffer for the image socket.  Allocated once on the heap to avoid
 * placing a 4 MB array on the stack. */
static uint8_t *image_buf = nullptr;

void sig_func(int sig) {}

int main(int argc, char **argv)
{
    Estimator estimator;

    /* ── Allocate image receive buffer ────────────────────────────────── */
    image_buf = new uint8_t[IMAGE_SOCK_BUF_SZ];

    /* ── UNIX dgram sockets: IMU, features, image ─────────────────────── */
    struct pollfd pfds[4];
    int imu_sock, features_sock, image_sock;
    struct sockaddr_un ipc_addr;

    if ((imu_sock = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, IMU_SOCK_PATH);
    unlink(IMU_SOCK_PATH);
    if (bind(imu_sock, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    if ((features_sock = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, FEATURES_SOCK_PATH);
    unlink(FEATURES_SOCK_PATH);
    if (bind(features_sock, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    /* Image socket – absence of a sender is not fatal; handle gracefully. */
    image_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (image_sock >= 0) {
        memset(&ipc_addr, 0, sizeof(ipc_addr));
        ipc_addr.sun_family = AF_UNIX;
        strcpy(ipc_addr.sun_path, IMAGE_SOCK_PATH);
        unlink(IMAGE_SOCK_PATH);
        if (bind(image_sock, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
            perror("image socket bind failed (image input disabled)");
            close(image_sock);
            image_sock = -1;
        }
    }

    /* ── UDP socket for publishing to loop_fusion client ─────────────── */
    struct sockaddr_in srv_addr;
    pub_sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    srv_addr.sin_port = htons(8800);
    if (bind(pub_sock, (const struct sockaddr *)&srv_addr, sizeof(srv_addr)) < 0 ) {
        perror("bind failed");
    }
    memset(&pub_addr, 0, sizeof(pub_addr));

    /* ── poll set: IMU, features, UDP control, image ─────────────────── */
    pfds[0].fd     = imu_sock;
    pfds[0].events = POLLIN;
    pfds[1].fd     = features_sock;
    pfds[1].events = POLLIN;
    pfds[2].fd     = pub_sock;
    pfds[2].events = POLLIN;
    pfds[3].fd     = (image_sock >= 0) ? image_sock : -1;
    pfds[3].events = POLLIN;

    signal(SIGINT, sig_func);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub();

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    while (true) {
        if (poll(pfds, 4, -1) > 0) {
            if (pfds[0].revents & POLLIN) {
                if (recv(imu_sock, buf, BUF_SZ, 0) > 0) {
                    Vector3d acc(buf[1], buf[2], buf[3]);
                    Vector3d gyr(buf[4], buf[5], buf[6]);
                    estimator.inputIMU(buf[0], acc, gyr);
                }
            }
            if (pfds[1].revents & POLLIN) {
                int len = recv(features_sock, buf, BUF_SZ, 0);
                if (len > 0) {
                    double* features_data = buf;
                    int num = features_data[0];
                    //printf("rcv %d bytes, %d features\n", len, num);
                    double t = features_data[1];
                    features_data += 2;
                    featureFrame.clear();
                    for (int i = 0; i < num; ++i) {
                        int id = features_data[0];
                        xyz_uv_velocity << features_data[1], features_data[2], 1, features_data[3], features_data[4], features_data[5], features_data[6];
                        featureFrame[id].emplace_back(0,  xyz_uv_velocity);
                        xyz_uv_velocity << features_data[7], features_data[8], 1, features_data[9], features_data[10], features_data[11], features_data[12];
                        featureFrame[id].emplace_back(1,  xyz_uv_velocity);
                        features_data += 13;
                    }
                    estimator.inputFeature(t, featureFrame);
                }
            }
            if (pfds[2].revents & POLLIN) {
                socklen_t addrlen = sizeof(struct sockaddr_in);
                if (recvfrom(pub_sock, buf, 8, 0, (struct sockaddr*)&pub_addr, &addrlen) > 0) {
                    printf("debug client inc\n");
                }
            }
            /* ── Image input ──────────────────────────────────────────── */
            if ((pfds[3].revents & POLLIN) && image_sock >= 0) {
                ssize_t len = recv(image_sock, image_buf, IMAGE_SOCK_BUF_SZ, 0);
                if (len >= (ssize_t)IMAGE_HDR_SZ) {
                    /* Parse header fields at their fixed offsets:
                     *   [0..7]   double   timestamp_sec
                     *   [8..11]  uint32_t width
                     *  [12..15]  uint32_t height
                     *  [16..19]  uint32_t encoding  (0=mono8, 1=jpeg)
                     *  [20..23]  uint32_t payload_bytes
                     */
                    double   stamp;
                    uint32_t width, height, encoding, payload_bytes;
                    const uint8_t *hp = image_buf;
                    memcpy(&stamp,         hp,      sizeof(double));   hp += 8;
                    memcpy(&width,         hp,      sizeof(uint32_t)); hp += 4;
                    memcpy(&height,        hp,      sizeof(uint32_t)); hp += 4;
                    memcpy(&encoding,      hp,      sizeof(uint32_t)); hp += 4;
                    memcpy(&payload_bytes, hp,      sizeof(uint32_t));
                    ssize_t expected = (ssize_t)(IMAGE_HDR_SZ + payload_bytes);
                    if (len >= expected && payload_bytes > 0) {
                        storeLatestImage(stamp, width, height, encoding,
                                         image_buf + IMAGE_HDR_SZ, payload_bytes);
                    }
                }
            }
        } else break;
    }

    gogogo = false;

    unlink(IMU_SOCK_PATH);
    unlink(FEATURES_SOCK_PATH);
    if (image_sock >= 0) {
        close(image_sock);
        unlink(IMAGE_SOCK_PATH);
    }
    delete[] image_buf;

    printf("bye\n");

    return 0;
}
