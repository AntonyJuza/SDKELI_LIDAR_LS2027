#ifndef SDKELI_LS_SENSOR_FRAME_HPP_
#define SDKELI_LS_SENSOR_FRAME_HPP_

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace sdkeli_ls_udp
{

class CSDKeliLsSensFrame
{
  struct SensData
  {
    uint8_t  header;
    uint8_t  cmd_id;
    uint16_t range_start;
    uint16_t range_end;
    uint8_t  check_value;
    uint16_t sens_data[0];
  } __attribute__((packed));

public:
  CSDKeliLsSensFrame();
  ~CSDKeliLsSensFrame();

  bool InitFromSensBuff(const char * buff, int length);

  /* Get Frame Header */
  uint8_t GetFrameHeader() const;

  /* Get command Id */
  uint8_t GetCommandId() const;

  /* Get Range Start and Range End */
  uint16_t GetRangeStart() const;
  uint16_t GetRangeEnd() const;

  /* Get sensor data count */
  int GetSensDataCount() const;

  /* Get sensor data of index */
  uint16_t GetSensDataOfIndex(int index) const;

  /* Get sensor intensity of index */
  uint16_t GetSensIntensityOfIndex(int index) const;

  /* Debug helpers */
  void DumpFrameHeader() const;
  void DumpFrameData() const;

private:
  SensData * sens_data_ {nullptr};
  int sens_data_length_ {0};

  bool CheckFrame(
    const char * buff,
    int length,
    uint8_t value) const;
};

}  // namespace sdkeli_ls_udp

#endif  // SDKELI_LS_SENSOR_FRAME_HPP_
