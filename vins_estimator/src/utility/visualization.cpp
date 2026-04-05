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
#include <mutex>
#include <vector>
#include <cstring>
#include <cmath>

/* ── external state from main.cpp ───────────────────────────────────────── */
extern int pub_sock;
extern struct sockaddr_in pub_addr;

/* ── UNIX dgram socket used for the legacy chobits VIO consumer ─────────── */
static struct sockaddr_un chobits_addr, chobits_local_addr;
static int chobits_sock;

/* ── per-type UDP sequence counters ─────────────────────────────────────── */
static uint16_t g_seq[6] = {0};

/* ── thread-safe latest-image buffer ────────────────────────────────────── */
struct LatestImage {
    double   stamp    = 0.0;
    uint32_t width    = 0;
    uint32_t height   = 0;
    uint32_t encoding = 0;          /* 0=mono8, 1=jpeg */
    std::vector<uint8_t> data;
    bool     valid    = false;
};
static LatestImage      g_latest_image;
static std::mutex       g_image_mutex;

/* ══════════════════════════════════════════════════════════════════════════
 * Helper: fill and send a fixed-size UDP datagram to the registered client.
 * Returns false if no client has registered yet.
 * ══════════════════════════════════════════════════════════════════════════ */
static bool udp_send(const void *buf, size_t len)
{
    if (pub_addr.sin_family != AF_INET)
        return false;
    if (sendto(pub_sock, buf, len, 0,
               reinterpret_cast<const struct sockaddr *>(&pub_addr),
               sizeof(pub_addr)) < 0) {
        perror("udp_send");
        return false;
    }
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Helper: build UdpHeader in-place.
 * ══════════════════════════════════════════════════════════════════════════ */
static void fill_header(UdpHeader *h, UdpPacketType type, double stamp,
                        uint32_t payload_sz)
{
    h->magic      = UDP_MAGIC;
    h->version    = UDP_VERSION;
    h->type       = static_cast<uint8_t>(type);
    h->sequence   = g_seq[type]++;
    h->stamp      = stamp;
    h->payload_sz = payload_sz;
}

/* ══════════════════════════════════════════════════════════════════════════
 * registerPub – open sockets used for publishing.
 * ══════════════════════════════════════════════════════════════════════════ */
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
    bind(chobits_sock,
         reinterpret_cast<struct sockaddr *>(&chobits_local_addr),
         sizeof(chobits_local_addr));
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubOdometry
 *
 * UDP_TYPE_ODOMETRY payload (80 bytes):
 *   double px, py, pz       – position (world frame)
 *   double qx, qy, qz, qw   – orientation quaternion
 *   double vx, vy, vz        – velocity (world frame)
 * ══════════════════════════════════════════════════════════════════════════ */
void pubOdometry(const Estimator &estimator)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    double px = estimator.Ps[WINDOW_SIZE].x();
    double py = estimator.Ps[WINDOW_SIZE].y();
    double pz = estimator.Ps[WINDOW_SIZE].z();
    double vx = estimator.Vs[WINDOW_SIZE].x();
    double vy = estimator.Vs[WINDOW_SIZE].y();
    double vz = estimator.Vs[WINDOW_SIZE].z();

    Eigen::Quaterniond q(estimator.Rs[WINDOW_SIZE]);
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();
    double qw = q.w();

    /* Legacy chobits consumer (float array, unchanged). */
    float chobits_msg[10] = {
        (float)qw, (float)qx, (float)qy, (float)qz,
        (float)px, (float)py, (float)pz,
        (float)vx, (float)vy, (float)vz
    };
    sendto(chobits_sock, chobits_msg, sizeof(chobits_msg), 0,
           reinterpret_cast<struct sockaddr *>(&chobits_addr),
           sizeof(chobits_addr));

    if (pub_addr.sin_family != AF_INET)
        return;

    /* Structured UDP packet – build into a flat byte array to avoid any
     * compiler padding between UdpHeader and the payload doubles. */
    double stamp = estimator.Headers[WINDOW_SIZE];
    double payload[10] = { px, py, pz, qx, qy, qz, qw, vx, vy, vz };

    uint8_t pkt[sizeof(UdpHeader) + sizeof(payload)];
    fill_header(reinterpret_cast<UdpHeader *>(pkt),
                UDP_TYPE_ODOMETRY, stamp, sizeof(payload));
    memcpy(pkt + sizeof(UdpHeader), payload, sizeof(payload));
    udp_send(pkt, sizeof(pkt));
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubKeyframePose
 *
 * Publishes the pose of the frame at window index WINDOW_SIZE-2, which is
 * the keyframe being committed when marginalization_flag == MARGIN_OLD.
 *
 * UDP_TYPE_KF_POSE payload (80 bytes) – same layout as ODOMETRY:
 *   double px, py, pz       – position (world frame)
 *   double qx, qy, qz, qw   – orientation quaternion
 *   double vx, vy, vz        – zeros (not meaningful for a keyframe)
 * ══════════════════════════════════════════════════════════════════════════ */
void pubKeyframePose(const Estimator &estimator)
{
    if (pub_addr.sin_family != AF_INET)
        return;

    int i = WINDOW_SIZE - 2;
    double stamp = estimator.Headers[i];

    Eigen::Vector3d     P = estimator.Ps[i];
    Eigen::Quaterniond  R(estimator.Rs[i]);

    double payload[10] = {
        P.x(), P.y(), P.z(),
        R.x(), R.y(), R.z(), R.w(),
        0.0, 0.0, 0.0      /* velocity not used for keyframe pose */
    };

    uint8_t pkt[sizeof(UdpHeader) + sizeof(payload)];
    fill_header(reinterpret_cast<UdpHeader *>(pkt),
                UDP_TYPE_KF_POSE, stamp, sizeof(payload));
    memcpy(pkt + sizeof(UdpHeader), payload, sizeof(payload));
    udp_send(pkt, sizeof(pkt));
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubKeyframePoint
 *
 * Publishes the 3-D points observed in the keyframe (WINDOW_SIZE-2).
 * A point is included when:
 *   – its start_frame < WINDOW_SIZE-2   (observed in an earlier frame too)
 *   – it is still tracked at WINDOW_SIZE-2
 *   – solve_flag == 1  (depth successfully triangulated)
 *
 * UDP_TYPE_KF_POINT payload (4 + N*32 bytes):
 *   uint32_t n_points
 *   For each point:
 *     float x, y, z   – 3-D world position
 *     float nx, ny    – normalised image coords in cam0 at WINDOW_SIZE-2
 *     float u, v      – pixel coords  in cam0 at WINDOW_SIZE-2
 *     float id        – feature track id (cast from int)
 *
 * Multiple datagrams are sent if the payload would exceed ~60 KB (rare).
 * ══════════════════════════════════════════════════════════════════════════ */
void pubKeyframePoint(const Estimator &estimator)
{
    if (pub_addr.sin_family != AF_INET)
        return;

    int kf = WINDOW_SIZE - 2;
    double stamp = estimator.Headers[kf];

    /* Collect all qualifying points into a contiguous float buffer.
     * Each point: x,y,z,nx,ny,u,v,id  →  8 floats = 32 bytes. */
    std::vector<float> pts;
    pts.reserve(256 * 8);

    for (const auto &it_per_id : estimator.f_manager.feature)
    {
        int frame_size = static_cast<int>(it_per_id.feature_per_frame.size());
        if (it_per_id.start_frame >= kf)
            continue;   /* not visible before kf */
        if (it_per_id.start_frame + frame_size - 1 < kf)
            continue;   /* tracking lost before kf */
        if (it_per_id.solve_flag != 1)
            continue;   /* depth not triangulated */

        /* 3-D world position. */
        int imu_i = it_per_id.start_frame;
        Eigen::Vector3d pts_c =
            it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Eigen::Vector3d w_pts =
            estimator.Rs[imu_i] * (estimator.ric[0] * pts_c + estimator.tic[0])
            + estimator.Ps[imu_i];

        /* 2-D observation at frame kf (relative index). */
        int rel = kf - it_per_id.start_frame;
        const auto &obs = it_per_id.feature_per_frame[rel];

        pts.push_back((float)w_pts.x());
        pts.push_back((float)w_pts.y());
        pts.push_back((float)w_pts.z());
        pts.push_back((float)obs.point.x());   /* nx */
        pts.push_back((float)obs.point.y());   /* ny */
        pts.push_back((float)obs.uv.x());      /* u  */
        pts.push_back((float)obs.uv.y());      /* v  */
        pts.push_back((float)it_per_id.feature_id); /* id */
    }

    uint32_t n = static_cast<uint32_t>(pts.size() / 8);
    uint32_t payload_sz = sizeof(uint32_t) + n * 8 * sizeof(float);

    /* Build and send a single datagram (typical point counts are small). */
    std::vector<uint8_t> buf(sizeof(UdpHeader) + payload_sz);
    UdpHeader *hdr = reinterpret_cast<UdpHeader *>(buf.data());
    fill_header(hdr, UDP_TYPE_KF_POINT, stamp, payload_sz);

    uint8_t *p = buf.data() + sizeof(UdpHeader);
    memcpy(p, &n, sizeof(uint32_t));
    p += sizeof(uint32_t);
    if (n > 0)
        memcpy(p, pts.data(), n * 8 * sizeof(float));

    udp_send(buf.data(), buf.size());
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubExtrinsic
 *
 * UDP_TYPE_EXTRINSIC payload (56 bytes):
 *   double tx, ty, tz    – camera translation in IMU frame
 *   double qx, qy, qz, qw – camera-to-IMU rotation quaternion
 * ══════════════════════════════════════════════════════════════════════════ */
void pubExtrinsic(const Estimator &estimator)
{
    if (pub_addr.sin_family != AF_INET)
        return;

    double stamp = estimator.Headers[WINDOW_SIZE];
    Eigen::Quaterniond q(estimator.ric[0]);
    const Eigen::Vector3d &t = estimator.tic[0];

    double payload[7] = {
        t.x(), t.y(), t.z(),
        q.x(), q.y(), q.z(), q.w()
    };

    uint8_t pkt[sizeof(UdpHeader) + sizeof(payload)];
    fill_header(reinterpret_cast<UdpHeader *>(pkt),
                UDP_TYPE_EXTRINSIC, stamp, sizeof(payload));
    memcpy(pkt + sizeof(UdpHeader), payload, sizeof(payload));
    udp_send(pkt, sizeof(pkt));
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubMarginCloud
 *
 * Publishes 3-D points that are about to be marginalised (visualisation
 * only).  Included: features with start_frame==0 and only 1-2 observations.
 *
 * UDP_TYPE_MARGIN_CLOUD payload (4 + N*12 bytes):
 *   uint32_t n_points
 *   For each point:
 *     float x, y, z   – 3-D world position
 * ══════════════════════════════════════════════════════════════════════════ */
void pubMarginCloud(const Estimator &estimator)
{
    if (pub_addr.sin_family != AF_INET)
        return;

    double stamp = estimator.Headers[WINDOW_SIZE - 2];

    std::vector<float> pts;
    pts.reserve(128 * 3);

    for (const auto &it_per_id : estimator.f_manager.feature)
    {
        if (it_per_id.start_frame == 0
            && it_per_id.feature_per_frame.size() <= 2
            && it_per_id.solve_flag == 1)
        {
            Eigen::Vector3d pts_c =
                it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Eigen::Vector3d w_pts =
                estimator.Rs[0] * (estimator.ric[0] * pts_c + estimator.tic[0])
                + estimator.Ps[0];
            pts.push_back((float)w_pts.x());
            pts.push_back((float)w_pts.y());
            pts.push_back((float)w_pts.z());
        }
    }

    uint32_t n = static_cast<uint32_t>(pts.size() / 3);
    uint32_t payload_sz = sizeof(uint32_t) + n * 3 * sizeof(float);

    std::vector<uint8_t> buf(sizeof(UdpHeader) + payload_sz);
    UdpHeader *hdr = reinterpret_cast<UdpHeader *>(buf.data());
    fill_header(hdr, UDP_TYPE_MARGIN_CLOUD, stamp, payload_sz);

    uint8_t *p = buf.data() + sizeof(UdpHeader);
    memcpy(p, &n, sizeof(uint32_t));
    p += sizeof(uint32_t);
    if (n > 0)
        memcpy(p, pts.data(), n * 3 * sizeof(float));

    udp_send(buf.data(), buf.size());
}

/* ══════════════════════════════════════════════════════════════════════════
 * storeLatestImage – called from main.cpp when an image arrives on the
 *                    /tmp/chobits_image UNIX socket.
 * ══════════════════════════════════════════════════════════════════════════ */
void storeLatestImage(double stamp, uint32_t width, uint32_t height,
                      uint32_t encoding, const uint8_t *data, uint32_t bytes)
{
    std::lock_guard<std::mutex> lk(g_image_mutex);
    g_latest_image.stamp    = stamp;
    g_latest_image.width    = width;
    g_latest_image.height   = height;
    g_latest_image.encoding = encoding;
    g_latest_image.data.assign(data, data + bytes);
    g_latest_image.valid    = true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * pubKeyframeImage
 *
 * Sends the buffered image whose timestamp is closest to kf_stamp (within
 * 5 ms tolerance) as one or more UDP_TYPE_IMAGE_CHUNK datagrams.
 *
 * Each datagram:
 *   UdpHeader           (20 bytes)  – type=IMAGE_CHUNK, stamp=kf_stamp
 *   UdpImageChunkHeader (24 bytes)  – chunking metadata + image dimensions
 *   payload             (≤ UDP_IMG_CHUNK_PAYLOAD bytes) – raw pixel/JPEG bytes
 *
 * The sequence counter in UdpHeader is shared across all chunks of the
 * same image so the receiver can group them.
 * ══════════════════════════════════════════════════════════════════════════ */
void pubKeyframeImage(double kf_stamp)
{
    if (pub_addr.sin_family != AF_INET)
        return;

    /* Take a snapshot of the image buffer under lock. */
    LatestImage img;
    {
        std::lock_guard<std::mutex> lk(g_image_mutex);
        if (!g_latest_image.valid)
            return;
        if (std::fabs(g_latest_image.stamp - kf_stamp) > 0.005)
            return;   /* timestamp difference > 5 ms, skip */
        img = g_latest_image;
    }

    uint32_t total_bytes  = static_cast<uint32_t>(img.data.size());
    uint32_t total_chunks = (total_bytes + UDP_IMG_CHUNK_PAYLOAD - 1)
                            / UDP_IMG_CHUNK_PAYLOAD;
    if (total_chunks == 0) total_chunks = 1;

    /* All chunks of this image share the same sequence number so the
     * receiver can reassemble them. */
    uint16_t img_seq = g_seq[UDP_TYPE_IMAGE_CHUNK]++;

    /* Scratch buffer large enough for one full packet. */
    static uint8_t pkt_buf[sizeof(UdpHeader) + sizeof(UdpImageChunkHeader)
                            + UDP_IMG_CHUNK_PAYLOAD];

    for (uint32_t ci = 0; ci < total_chunks; ci++)
    {
        uint32_t offset      = ci * UDP_IMG_CHUNK_PAYLOAD;
        uint32_t chunk_bytes = total_bytes - offset;
        if (chunk_bytes > UDP_IMG_CHUNK_PAYLOAD)
            chunk_bytes = UDP_IMG_CHUNK_PAYLOAD;

        uint32_t payload_sz = sizeof(UdpImageChunkHeader) + chunk_bytes;

        UdpHeader *hdr = reinterpret_cast<UdpHeader *>(pkt_buf);
        hdr->magic      = UDP_MAGIC;
        hdr->version    = UDP_VERSION;
        hdr->type       = static_cast<uint8_t>(UDP_TYPE_IMAGE_CHUNK);
        hdr->sequence   = img_seq;   /* same for all chunks of this image */
        hdr->stamp      = kf_stamp;
        hdr->payload_sz = payload_sz;

        UdpImageChunkHeader *chk =
            reinterpret_cast<UdpImageChunkHeader *>(pkt_buf + sizeof(UdpHeader));
        chk->total_bytes  = total_bytes;
        chk->chunk_index  = ci;
        chk->total_chunks = total_chunks;
        chk->width        = img.width;
        chk->height       = img.height;
        chk->encoding     = img.encoding;

        uint8_t *data_dst = pkt_buf + sizeof(UdpHeader) + sizeof(UdpImageChunkHeader);
        memcpy(data_dst, img.data.data() + offset, chunk_bytes);

        udp_send(pkt_buf, sizeof(UdpHeader) + payload_sz);
    }
}
