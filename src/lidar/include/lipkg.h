/**
 * @file lipkg.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LIPKG_H
#define __LIPKG_H

#include <chrono>

#include "pointdata.h"
#include "sl_transform.h"
#include "cmd_interface_linux.h"
#include "slbf.h"

namespace ldlidar {

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  LiPkg();
  ~LiPkg();

  std::string GetSdkVersionNumber(void);
  void SetProductType(LDType type_number);
  /**
   * @brief Set laser scan dir
   * @param dir
   *       value is true, counterclockwise;
   *       value is false, clockwise
  */
  void SetLaserScanDir(bool dir);
  // Lidar spin speed (Hz)
  double GetSpeed(void);
  // get lidar spind speed (degree per second) origin
  uint16_t GetSpeedOrigin(void);
  // time stamp of the packet In milliseconds
  uint16_t GetTimestamp(void);
  // Lidar data frame is ready
  bool IsFrameReady(void);
  void ResetFrameReady(void);
  void SetFrameReady(void);
  void CommReadCallback(const char *byte, size_t len);
  Points2D GetLaserScanData(void);
 
 private:
  LDType ld_product_type_;
  std::string sdk_pack_verison_;
  bool laser_scan_dir_;
  bool is_frame_ready_;
  uint16_t timestamp_;
  double speed_;
  int point_frequence_;

  LiDARFrameTypeDef pkg;
  Points2D frame_tmp_;
  Points2D laser_scan_data_;
  std::mutex  mutex_lock1_;
  std::mutex  mutex_lock2_;
  
   // parse single packet
  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t *data, long len);
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();
  void SetLaserScanData(Points2D& src);
};

} // namespace ldlidar 

#endif  // __LIPKG_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/