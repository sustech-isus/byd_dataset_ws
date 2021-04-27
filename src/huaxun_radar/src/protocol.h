
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


/*

 // human  readable interpretation
  Name:       ESR_Track34
  Id:         0x521
  Length:     8 bytes
  Cycle time: - ms
  Senders:    ESR
  Layout:

                          Bit

             7   6   5   4   3   2   1   0
           +---+---+---+---+---+---+---+---+
         0 |<---------------------x|<-x|<-x|
           +---+---+---+---+---+---+---+---+
                                 |   |   +-- CAN_TX_TRACK_ONCOMING
                                 |   +-- CAN_TX_TRACK_GROUPING_CHANGED
                                 +-- CAN_TX_TRACK_LAT_RATE
           +---+---+---+---+---+---+---+---+
         1 |<---------x|<------------------|
           +---+---+---+---+---+---+---+---+
                     +-- CAN_TX_TRACK_STATUS
           +---+---+---+---+---+---+---+---+
         2 |------------------x|<----------|
           +---+---+---+---+---+---+---+---+
                             +-- CAN_TX_TRACK_ANGLE
           +---+---+---+---+---+---+---+---+
         3 |------------------------------x|
     B     +---+---+---+---+---+---+---+---+
     y                                   +-- CAN_TX_TRACK_RANGE
     t     +---+---+---+---+---+---+---+---+
     e   4 |<-x|<-x|<-------------x|<------|
           +---+---+---+---+---+---+---+---+
             |   |               +-- CAN_TX_TRACK_WIDTH
             |   +-- CAN_TX_TRACK_ROLLING_COUNT
             +-- CAN_TX_TRACK_BRIDGE_OBJECT
           +---+---+---+---+---+---+---+---+
         5 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         +-- CAN_TX_TRACK_RANGE_ACCEL
           +---+---+---+---+---+---+---+---+
         6 |<-----x|<----------------------|
           +---+---+---+---+---+---+---+---+
                 +-- CAN_TX_TRACK_MED_RANGE_MODE
           +---+---+---+---+---+---+---+---+
         7 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         +-- CAN_TX_TRACK_RANGE_RATE

  Signal tree:

    -- {root}
       +-- CAN_TX_TRACK_LAT_RATE
       +-- CAN_TX_TRACK_GROUPING_CHANGED
       +-- CAN_TX_TRACK_ONCOMING
       +-- CAN_TX_TRACK_STATUS
       +-- CAN_TX_TRACK_ANGLE
       +-- CAN_TX_TRACK_RANGE
       +-- CAN_TX_TRACK_BRIDGE_OBJECT
       +-- CAN_TX_TRACK_ROLLING_COUNT
       +-- CAN_TX_TRACK_WIDTH
       +-- CAN_TX_TRACK_RANGE_ACCEL
       +-- CAN_TX_TRACK_MED_RANGE_MODE
       +-- CAN_TX_TRACK_RANGE_RATE

  Signal choices:

    CAN_TX_TRACK_GROUPING_CHANGED
        0 GroupingUnchanged
        1 GroupingChanged

    CAN_TX_TRACK_ONCOMING
        0 NotOncoming
        1 Oncoming

    CAN_TX_TRACK_STATUS
        0 No_Target
        1 New_Target
        2 New_Updated_Target
        3 Updated_Target
        4 Coasted_Target
        5 Merged_Target
        6 Invalid_Coasted_Target
        7 New_Coasted_Target

    CAN_TX_TRACK_BRIDGE_OBJECT
        0 Not_Bridge
        1 Bridge

    CAN_TX_TRACK_MED_RANGE_MODE
        0 No_MR_LR_Update
        1 MR_Update_Only
        2 LR_Update_Only
        3 Both_MR_LR_Update
*/

/* dbc
BO_ 1313 ESR_Track34: 8 ESR

 SG_ CAN_TX_TRACK_GROUPING_CHANGED : 1|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_ONCOMING : 0|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_LAT_RATE : 7|6@0- (0.25,0) [-8|7.75] "" Vector__XXX
 SG_ CAN_TX_TRACK_BRIDGE_OBJECT : 39|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_WIDTH : 37|4@0+ (0.5,0) [0|7.5] "m" Vector__XXX
 SG_ CAN_TX_TRACK_STATUS : 15|3@0+ (1,0) [0|7] "" Vector__XXX
 SG_ CAN_TX_TRACK_ROLLING_COUNT : 38|1@0+ (1,0) [0|1] "" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE_RATE : 53|14@0- (0.01,0) [-81.92|81.91] "m/s" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE_ACCEL : 33|10@0- (0.05,0) [-25.6|25.55] "m/s/s" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE : 18|11@0+ (0.1,0) [0|204.7] "m" Vector__XXX
 SG_ CAN_TX_TRACK_MED_RANGE_MODE : 55|2@0+ (1,0) [0|3] "" Vector__XXX
 SG_ CAN_TX_TRACK_ANGLE : 12|10@0- (0.1,0) [-51.2|51.1] "deg" Vector__XXX
------------------------------------------------------------------------------
 sg_  signal
      name
start|end bits
@: 0-bigendian, 1-littleendian, byteorder
 +/-signaed/unsigned
 (scale, offset)
 [min, max]
 unit
 receiver?
*/
struct PclDelphiRadarTrackType{
  PCL_ADD_POINT4D;
  std::uint8_t oncoming;
  std::uint8_t grouping_changed;
  std::uint8_t lat_rate;
  std::uint8_t status;
  
  std::uint16_t angle;
  std::uint16_t range;
  std::uint16_t range_accel;
  std::uint16_t range_rate;
  std::uint8_t  med_range_mode;

  std::uint8_t width;
  std::uint8_t rolling_count;
  std::uint8_t bridge_object;

  float vx;  
  float vy;
  float vz;
  std::uint8_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclDelphiRadarTrackType,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                  (std::uint8_t, oncoming, oncoming)
                                  (std::uint8_t, grouping_changed, grouping_changed)
                                  (std::uint8_t, lat_rate, lat_rate)
                                  (std::uint8_t, status, status)
                                  (std::uint16_t, angle,angle)
                                  (std::uint16_t, range,range)
                                  (std::uint16_t, range_accel,range_accel)
                                  (std::uint16_t, range_rate,range_rate)
                                  (std::uint8_t,  med_range_mode,med_range_mode)
                                  (std::uint8_t, width,width)
                                  (std::uint8_t, rolling_count, rolling_count)
                                  (std::uint8_t, bridge_object, bridge_object)
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
  pcl_radar_point.z = 0;

  auto raw_target_speed     = (float)((data[6] >> 4) + (data[7] << 4));
  float target_speed    = (raw_target_speed * 0.025f) - 50.0f;
  pcl_radar_point.vx = std::sin(target_angle / 180.0 * M_PI) * target_speed;
  pcl_radar_point.vy = std::cos(target_angle / 180.0 * M_PI) * target_speed;
  pcl_radar_point.vz = 0;

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
  pcl_radar_track.vz = 0;
  pcl_radar_track.x = raw_r_x / 50.0f - 200.0f;
  pcl_radar_track.y = raw_r_y / 50.0f - 10.0f;
  pcl_radar_track.z = 0;
  return pcl_radar_track;
}

// tricky impl
inline std::uint64_t readBits(std::uint64_t d, int start, int length)
{
  int byte = start/8;
  int bit = start%8;
  int startInU64 = 8 * (7-byte) + bit - length + 1;

  uint64_t ret = d << (64- startInU64 - length);
  ret = ret >> (64-length);
  return ret;
}

inline std::uint64_t makeUint64(const unsigned char *data, int length)
{
  std::uint64_t d = 0;
  for (int i=0; i < 8 && i < length; i++)
  {
    d = d*0x100 + data[i];
  }
  return d;
}

/*
 SG_ CAN_TX_TRACK_GROUPING_CHANGED : 1|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_ONCOMING : 0|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_LAT_RATE : 7|6@0- (0.25,0) [-8|7.75] "" Vector__XXX
 SG_ CAN_TX_TRACK_BRIDGE_OBJECT : 39|1@0+ (1,0) [0|0] "" Vector__XXX
 SG_ CAN_TX_TRACK_WIDTH : 37|4@0+ (0.5,0) [0|7.5] "m" Vector__XXX
 SG_ CAN_TX_TRACK_STATUS : 15|3@0+ (1,0) [0|7] "" Vector__XXX
 SG_ CAN_TX_TRACK_ROLLING_COUNT : 38|1@0+ (1,0) [0|1] "" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE_RATE : 53|14@0- (0.01,0) [-81.92|81.91] "m/s" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE_ACCEL : 33|10@0- (0.05,0) [-25.6|25.55] "m/s/s" Vector__XXX
 SG_ CAN_TX_TRACK_RANGE : 18|11@0+ (0.1,0) [0|204.7] "m" Vector__XXX
 SG_ CAN_TX_TRACK_MED_RANGE_MODE : 55|2@0+ (1,0) [0|3] "" Vector__XXX
 SG_ CAN_TX_TRACK_ANGLE : 12|10@0- (0.1,0) [-51.2|51.1] "deg" Vector__XXX
 */
inline PclDelphiRadarTrackType CanMsgToDelphiRadarTrackMsg(const unsigned char *data) {

  std::uint64_t  d = makeUint64(data, 8);


  PclDelphiRadarTrackType track;
  
  track.oncoming =          readBits(d, 0,  1);
  track.grouping_changed =  readBits(d, 1,  1);
  track.lat_rate =          readBits(d, 7,  6);
  track.bridge_object =     readBits(d, 39, 1);
  track.width =             readBits(d, 37, 4);
  track.status =            readBits(d, 15, 3);
  track.rolling_count =     readBits(d, 38, 1);
  track.range_rate =        readBits(d, 53, 14);
  track.range_accel =       readBits(d, 33, 10);
  track.range =             readBits(d, 18, 11);
  track.med_range_mode =    readBits(d, 55, 2);
  track.angle =             readBits(d, 12, 10);

  float angle = track.angle * 0.1 - 51.2;
  float range = track.range * 0.1;

  track.x  = range * std::sin(angle / 180.0 * M_PI);
  track.y  = range * std::cos(angle / 180.0 * M_PI);
  
  return track;
}


#endif //SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_
