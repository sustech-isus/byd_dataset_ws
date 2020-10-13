
#ifndef SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_
#define SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_

#include <cstdint>

struct PclRadarPointType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
  float vz;
  std::uint8_t id;
  std::uint16_t peak;
  std::uint8_t car_speed;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclRadarPointType,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, vx, vx)
                                 (float, vy, vy)
                                 (float, vz, vz)
                                 (std::uint8_t, id, id)
                                 (std::uint16_t, peak, peak)
                                 (std::uint8_t, car_speed, car_speed)
)
struct PclRadarTrackType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
  float vz;
  std::uint8_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclRadarTrackType,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, vx, vx)
                                 (float, vy, vy)
                                 (float, vz, vz)
                                 (std::uint8_t, id, id))

inline PclRadarPointType CanMsgToRadarPointMsg(const unsigned char *data) {
  PclRadarPointType pcl_radar_point;

  pcl_radar_point.id   = (uint8_t)(data[0]);
  pcl_radar_point.peak = (uint16_t)(data[1] + ((data[2] & 0x1f) << 8));
  pcl_radar_point.car_speed = (uint8_t)data[4];

  auto raw_target_distance = (float)((data[2] >> 5) + (data[3] << 3));
  float target_distance = raw_target_distance / 10.0f;

  auto raw_target_angle     = (float)(data[5] + ((data[6] & 0x0f) << 8));
  float target_angle    = (raw_target_angle * 0.1f) - 180.0f;

  pcl_radar_point.x = std::sin(target_angle / 180.0 * M_PI) * target_distance;
  pcl_radar_point.y = std::cos(target_angle / 180.0 * M_PI) * target_distance;

  auto raw_target_speed     = (float)((data[6] >> 4) + (data[7] << 4));
  float target_speed    = (raw_target_speed * 0.025f) - 50.0f;
  pcl_radar_point.vx = std::sin(target_angle / 180.0 * M_PI) * target_speed;
  pcl_radar_point.vy = std::cos(target_angle / 180.0 * M_PI) * target_speed;

  return pcl_radar_point;
}

inline PclRadarTrackType CanMsgToRadarTrackMsg(const unsigned char *data) {
  PclRadarTrackType pcl_radar_track;
  pcl_radar_track.id = (uint8_t)(data[0] & 0x3f);
  auto raw_v_x = (float)(((data[2] & 0x07) << 10) + (data[1] << 2) + (data[0] >> 6));
  auto raw_v_y = (float)((data[3] << 5) + (data[2] >> 3));
  auto raw_r_x = (float)((data[5] << 8) + data[4]);
  auto raw_r_y = (float)((data[7] << 8) + data[6]);
  pcl_radar_track.vx = raw_v_x / 50.0f - 50.0f;
  pcl_radar_track.vy = raw_v_y / 50.0f - 50.0f;
  pcl_radar_track.x = raw_r_x / 50.0f - 200.0f;
  pcl_radar_track.y = raw_r_y / 50.0f - 10.0f;
  return pcl_radar_track;
}


#endif //SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_
