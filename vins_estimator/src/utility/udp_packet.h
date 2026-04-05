/*******************************************************
 * UDP packet definitions for vins_estimator → loop_fusion IPC.
 *
 * All packets begin with UdpHeader (20 bytes), followed by a
 * type-specific payload.  Image packets additionally contain an
 * UdpImageChunkHeader immediately after UdpHeader.
 *
 * Byte order: native (little-endian on all supported platforms).
 *******************************************************/
#pragma once
#include <stdint.h>

/* Magic number identifying every loop-fusion UDP packet ("LFUS"). */
#define UDP_MAGIC   0x5355464Cu  /* 'L','F','U','S' in little-endian */
#define UDP_VERSION 1u

/* Maximum safe payload for a single image chunk (~1200 bytes keeps
 * the total datagram well below the 1500-byte Ethernet MTU). */
#define UDP_IMG_CHUNK_PAYLOAD 1200u

/*
 * Packet type codes (stored in UdpHeader::type).
 *
 *  0  ODOMETRY      – current VIO pose + velocity
 *  1  KF_POSE       – keyframe pose (published at marginalization)
 *  2  KF_POINT      – keyframe 3-D points with 2-D observations
 *  3  EXTRINSIC     – camera-to-IMU extrinsic calibration
 *  4  MARGIN_CLOUD  – marginalized point cloud (visualization only)
 *  5  IMAGE_CHUNK   – one chunk of a grayscale/JPEG keyframe image
 */
typedef enum UdpPacketType : uint8_t {
    UDP_TYPE_ODOMETRY     = 0,
    UDP_TYPE_KF_POSE      = 1,
    UDP_TYPE_KF_POINT     = 2,
    UDP_TYPE_EXTRINSIC    = 3,
    UDP_TYPE_MARGIN_CLOUD = 4,
    UDP_TYPE_IMAGE_CHUNK  = 5,
} UdpPacketType;

/*
 * Common packet header – present in every UDP datagram (20 bytes).
 *
 *  Offset  Size  Field
 *  ------  ----  -----
 *    0      4    magic      – must equal UDP_MAGIC (0x5355464C)
 *    4      1    version    – must equal UDP_VERSION (1)
 *    5      1    type       – UdpPacketType
 *    6      2    sequence   – per-type rolling counter (wraps at 65535)
 *    8      8    stamp      – timestamp in seconds (double)
 *   16      4    payload_sz – byte count of data following this header
 */
#pragma pack(push, 1)
struct UdpHeader {
    uint32_t magic;
    uint8_t  version;
    uint8_t  type;        /* UdpPacketType */
    uint16_t sequence;
    double   stamp;
    uint32_t payload_sz;
};
#pragma pack(pop)

/*
 * Image chunk sub-header – immediately follows UdpHeader when
 * type == UDP_TYPE_IMAGE_CHUNK (24 bytes).
 *
 *  Offset  Size  Field
 *  ------  ----  -----
 *    0      4    total_bytes  – total uncompressed/compressed image size
 *    4      4    chunk_index  – 0-based index of this chunk
 *    8      4    total_chunks – total number of chunks for this image
 *   12      4    width        – image width in pixels
 *   16      4    height       – image height in pixels
 *   20      4    encoding     – 0 = mono8, 1 = jpeg
 */
#pragma pack(push, 1)
struct UdpImageChunkHeader {
    uint32_t total_bytes;
    uint32_t chunk_index;
    uint32_t total_chunks;
    uint32_t width;
    uint32_t height;
    uint32_t encoding;   /* 0 = mono8, 1 = jpeg */
};
#pragma pack(pop)

/*
 * Payload layouts (all values are native-endian doubles or floats
 * unless noted otherwise):
 *
 * UDP_TYPE_ODOMETRY / UDP_TYPE_KF_POSE  (80 bytes):
 *   double px, py, pz          – position (world frame)
 *   double qx, qy, qz, qw      – orientation quaternion
 *   double vx, vy, vz          – velocity (world frame)  [zero for KF_POSE]
 *
 * UDP_TYPE_EXTRINSIC  (56 bytes):
 *   double tx, ty, tz          – translation (camera in IMU frame)
 *   double qx, qy, qz, qw      – rotation quaternion
 *
 * UDP_TYPE_KF_POINT  (4 + N*32 bytes):
 *   uint32_t n_points
 *   For each point:
 *     float x, y, z            – 3-D world position
 *     float nx, ny             – normalised image coordinates (cam0)
 *     float u, v               – pixel coordinates (cam0)
 *     float id                 – feature track id (cast from int)
 *
 * UDP_TYPE_MARGIN_CLOUD  (4 + N*12 bytes):
 *   uint32_t n_points
 *   For each point:
 *     float x, y, z            – 3-D world position
 *
 * UDP_TYPE_IMAGE_CHUNK:
 *   UdpHeader + UdpImageChunkHeader + raw pixel (or JPEG) bytes
 */
