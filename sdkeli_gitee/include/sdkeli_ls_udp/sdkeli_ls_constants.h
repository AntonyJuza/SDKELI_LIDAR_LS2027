#ifndef SDKELI_LS_CONSTANTS_HPP_
#define SDKELI_LS_CONSTANTS_HPP_

#include <cstdint>
#include <cstddef>

/* =========================================================
 * SDKELI UDP command frames (BINARY PROTOCOL)
 * ========================================================= */

/* Stop data streaming */
static constexpr uint8_t CMD_STOP_STREAM_DATA[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x02, 0x02
};

/* Enter maintenance access mode */
static constexpr uint8_t CMD_SET_MAINTENANCE_ACCESS_MODE[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01
};

/* Reboot device */
static constexpr uint8_t CMD_REBOOT[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x03, 0x03
};

/* Read device identify */
static constexpr uint8_t CMD_READ_IDENTIFY[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01
};

/* Read serial number */
static constexpr uint8_t CMD_READ_SERIAL_NUMBER[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x05, 0x05
};

/* Read firmware version */
static constexpr uint8_t CMD_READ_FIRMWARE_VERSION[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01
};

/* Read device state */
static constexpr uint8_t CMD_READ_DEVICE_STATE[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x04, 0x04
};

/* Start data streaming */
static constexpr uint8_t CMD_START_STREAM_DATA[] = {
  0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01
};

/* Command length (ALL commands are fixed-length) */
static constexpr std::size_t SIZE_OF_CMD =
  sizeof(CMD_START_STREAM_DATA);

/* =========================================================
 * Frame markers
 * ========================================================= */

static constexpr uint8_t STX = 0xAA;
static constexpr uint8_t ETX = 0x66;

/* =========================================================
 * Frame size
 * ========================================================= */

static constexpr uint32_t FRAME_LENGTH = 1622;

/* Fixed received buffer size */
#define RECV_BUFFER_SIZE 65536

/* =========================================================
 * Frame header offsets
 * ========================================================= */

#define CMD_FRAME_HEADER_START 0
#define CMD_FRAME_HEADER_LENGTH_H 4
#define CMD_FRAME_HEADER_LENGTH_L 5
#define CMD_FRAME_HEADER_CHECK_SUM 6
#define CMD_FRAME_HEADER_TYPE 7
#define CMD_FRAME_HEADER_TOTAL_INDEX_H 8
#define CMD_FRAME_HEADER_TOTAL_INDEX_L 9
#define CMD_FRAME_HEADER_SUB_PKG_NUM 10
#define CMD_FRAME_HEADER_SUB_INDEX 11
#define CMD_FRAME_DATA_START 12

/* =========================================================
 * Range indices
 * ========================================================= */

#define INDEX_RANGE_MAX 65415
#define INDEX_RANGE_MIN (20 * 60)

/* =========================================================
 * Frame limits
 * ========================================================= */

#define CMD_FRAME_MAX_LEN 1500
#define CMD_FRAME_MAX_SUB_PKG_NUM 4
#define CMD_FRAME_MIN_SUB_PKG_NUM 2

/* =========================================================
 * Endian helpers
 * ========================================================= */

static inline uint16_t switch_uint16(uint16_t value)
{
  return static_cast<uint16_t>((value << 8) | (value >> 8));
}

/* Backward compatibility with existing code */
#define SWITCH_UINT16(x) switch_uint16(x)

#endif  // SDKELI_LS_CONSTANTS_HPP_
