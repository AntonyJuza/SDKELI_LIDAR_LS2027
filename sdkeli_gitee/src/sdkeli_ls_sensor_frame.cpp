#include "sdkeli_ls_sensor_frame.h"
#include "sdkeli_ls_constants.h"

#include <cstring>
#include <cstdio>

namespace sdkeli_ls_udp
{

CSDKeliLsSensFrame::CSDKeliLsSensFrame()
: sens_data_(nullptr),
  sens_data_length_(0)
{
}

CSDKeliLsSensFrame::~CSDKeliLsSensFrame()
{
  if (sens_data_)
  {
    delete[] reinterpret_cast<uint8_t *>(sens_data_);
    sens_data_ = nullptr;
  }
}

uint8_t CSDKeliLsSensFrame::GetFrameHeader() const
{
  return sens_data_->header;
}

uint8_t CSDKeliLsSensFrame::GetCommandId() const
{
  return sens_data_->cmd_id;
}

uint16_t CSDKeliLsSensFrame::GetRangeStart() const
{
  return sens_data_->range_start;
}

uint16_t CSDKeliLsSensFrame::GetRangeEnd() const
{
  return sens_data_->range_end;
}

int CSDKeliLsSensFrame::GetSensDataCount() const
{
  return sens_data_->range_end - sens_data_->range_start + 1;
}

uint16_t CSDKeliLsSensFrame::GetSensDataOfIndex(int index) const
{
  if (!sens_data_ ||
      index < 0 ||
      index >= GetSensDataCount())
  {
    return 0;
  }

  return sens_data_->sens_data[index];
}

uint16_t CSDKeliLsSensFrame::GetSensIntensityOfIndex(int index) const
{
  const int data_count = GetSensDataCount();
  const int offset = data_count + index;

  if (!sens_data_ ||
      index < 0 ||
      index >= data_count)
  {
    return 0;
  }

  return sens_data_->sens_data[offset];
}

bool CSDKeliLsSensFrame::CheckFrame(
  const char * buff,
  int length,
  uint8_t value) const
{
  if (!buff || length <= 0)
  {
    return false;
  }

  uint8_t checksum = 0;
  for (int i = 0; i < length; ++i)
  {
    checksum += static_cast<uint8_t>(buff[i]);
  }

  return checksum == value;
}

bool CSDKeliLsSensFrame::InitFromSensBuff(
  const char * buff,
  int length)
{
  if (!buff || length <= 0)
  {
    return false;
  }

  /* Allocate contiguous buffer */
  uint8_t * raw = new uint8_t[length];
  std::memcpy(raw, buff, length);

  sens_data_ = reinterpret_cast<SensData *>(raw);
  sens_data_length_ = length;

  /* NOTE:
   * Device uses little-endian encoding.
   * Header fields are fixed for this model.
   */
  sens_data_->range_start = 0;
  sens_data_->range_end   = 810;

  int data_count = GetSensDataCount();

  if (length == data_count * 4)
  {
    data_count *= 2;  // distance + intensity
  }

  const uint16_t * src = reinterpret_cast<const uint16_t *>(buff);

  for (int i = 0; i < data_count; ++i)
  {
    sens_data_->sens_data[i] = SWITCH_UINT16(src[i]);
  }

  return true;
}

void CSDKeliLsSensFrame::DumpFrameHeader() const
{
  if (!sens_data_)
  {
    return;
  }

  printf("Frame Header: 0x%02X\n", GetFrameHeader());
  printf("Command   ID: 0x%02X\n", GetCommandId());
  printf("Angle START: 0x%04X\n", GetRangeStart());
  printf("Angle   END: 0x%04X\n", GetRangeEnd());
}

void CSDKeliLsSensFrame::DumpFrameData() const
{
  if (!sens_data_)
  {
    return;
  }

  const int data_count = GetSensDataCount();
  printf("Data Count: %d\n", data_count);

  for (int i = 0; i < data_count; ++i)
  {
    printf("%u ", static_cast<unsigned>(GetSensDataOfIndex(i)));
    if ((i + 1) % 48 == 0)
    {
      printf("\n");
    }
  }
  printf("\n");
}

}  // namespace sdkeli_ls_udp
