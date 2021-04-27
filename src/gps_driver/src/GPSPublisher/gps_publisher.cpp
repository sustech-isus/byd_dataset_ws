#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <iostream>
#include <gps_common/GPSFix.h>
#include <nav_msgs/Odometry.h>
#include <gps_common/conversions.h>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/serialization/vector.hpp>
#include "../ASIOLib/Executor.h"
#include "../ASIOLib/SerialPort.h"
#include "../GPSLib/GPSSentenceDecoder.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>
#include <gps_driver/GPSInfo.h>
#include <gps_driver/RotateInfo.h>
#include <gps_driver/SpeedInfo.h>
#include <unistd.h>

boost::mutex cout_lock;
void Log(const std::string &msg) 
{
	boost::mutex::scoped_lock lock(cout_lock);
	std::cout << "[" << boost::this_thread::get_id() << "] " << msg << std::endl;
}

class GPSPublisher :public boost::enable_shared_from_this<GPSPublisher>
{
    // add the INSPVAXA and RAWIMUA type
    enum MSG_TYPE{GGA,RMC,GSV,GLL,GSA,PSAT,PASHR,FPD,IMU,INSPVAXA,RAWIMUA};
    unsigned int _gps_seq_cnt=0;
    unsigned int _imu_seq_cnt=0;
    unsigned int _msg_map_total=0;
    unsigned int _msg_map_tmp=0;
	double last_useful_track=NULL_ANGLE;
	// define the msg data
    gps_common::GPSFix _gps_data;
    sensor_msgs::Imu _imu_data;


    sensor_msgs::Imu _rawimu_info;
    gps_driver::GPSInfo _gps_Info;
    gps_driver::SpeedInfo _speed_Info;
    gps_driver::RotateInfo _rotate_Info;

    boost::shared_ptr<ASIOLib::SerialPort> _serialPort;
    const std::string _portName;
    const unsigned int _baudRate;
    geometry_msgs::Quaternion _orientation;
    // decode the GPS message
    boost::shared_ptr<GPSLib::GPSSentenceDecoder> _decoder; // so shared_from_this() will work  
    // define message publish
    const boost::shared_ptr<ros::Publisher> _gpsinfo_pub_ptr;
    const boost::shared_ptr<ros::Publisher> _speedinfo_pub_ptr;
    const boost::shared_ptr<ros::Publisher> _rotateinfo_pub_ptr;
    const boost::shared_ptr<ros::Publisher> _rawimu_pub_ptr;

    const boost::shared_ptr<ros::Publisher> _gps_pub_ptr;
    boost::shared_ptr<ros::Publisher> _imu_pub_ptr;

    boost::posix_time::ptime _epoch;
    const boost::shared_ptr<std::ostream> _of;

    void initMsgTypeMap(std::vector<std::string> &message_types);
    void setMsgTypeMap(unsigned int &map,unsigned int bit);
    void initGPSHeader(boost::posix_time::time_duration time);
   // void initNovGPSHeader (boost::posix_time::ptime time);
    void initNovGPSHeader (ros::Time time);
    //void initNovIMUHeader(boost::posix_time::ptime time);
    void initNovIMUHeader(ros::Time time);
    void initIMUHeader(boost::posix_time::time_duration time);
    void PublishGPS();
    void OnRead(boost::asio::io_service &ios, const std::vector<unsigned char> &buffer, size_t bytesRead);
    void OnGGA(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, int quality, int numSatellites, double horizontalDilution, double altitude);

    // test by kp
    // receive the GPS info, including latitude 维度, longitude 经度, elevation 高程, updown 起伏,latStaDev 纬度标差，lonStaDev 经度标差, EleStaDev 高程标差
    void OnINS1(boost::asio::io_service &ios, ros::Time time,double latitude, double longitude, double elevation ,double latStaDev,
               double lonStaDev,double eleStaDev);
    // receive the imu speed info, including northAcc 北向速度，eastAcc 东向速度, diuAcc 天向速度, nVelStaDev 北速标差,eVelStaDev 东速标差,dVelStaDev天速标差
    void OnINS2(boost::asio::io_service &ios, ros::Time time,double northAcc, double eastAcc, double diuAcc ,double nVelStaDev,
                double eVelStaDev,double dVelStaDev);
    // receive the imu rotate info, including rollAng 横滚角, pitAng 俯仰角，headAng 航向角, rStaDev 横滚标差,pStaDev 俯仰标差,hStaDev 航向标差
    void OnINS3(boost::asio::io_service &ios, ros::Time time,double rollAng, double pitAng, double headAng ,double rStaDev,
                double pStaDev,double hStaDev);
    // receive the rawimu data
    void OnRAWIMUA(boost::asio::io_service &ios, ros::Time time,double zAcc,double yAcc,double xAcc,double rollVec,double yawVec,double pitchVec);

    void OnRMC(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, double speed, double course, boost::gregorian::date date, const std::string &validity);
    void OnGSV(boost::asio::io_service &ios, int totalMessages, int messageNumber, int totalSatellitesInView, const std::vector<GPSLib::SatelliteInfo> &satelliteInfo);
    void OnGLL(boost::asio::io_service &ios, boost::posix_time::time_duration, double latitude, double longitude, const std::string &validity);
    void OnGSA(boost::asio::io_service &ios, const std::string &mode, int fix, const std::vector<int> &satellitesInView, double pdop, double hdop, double vdop);
    void OnPSAT(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, double pitch, double roll);
    void OnPASHR(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, bool true_heading,double pitch, double roll);
    void OnFPD(boost::asio::io_service &ios, boost::posix_time::time_duration time, const std::vector<double> &pose_vect, const std::vector<double> &geo_vect, double speed,int numSatellites,int status_code);
    void OnIMU(boost::asio::io_service &ios, boost::posix_time::time_duration time, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ);
public:
    GPSPublisher(const std::string &portName, unsigned int baudRate,const boost::shared_ptr<ros::Publisher> &gps_pub,
                 const boost::shared_ptr<ros::Publisher> &_gpsinfo_pub_ptr,
                 const boost::shared_ptr<ros::Publisher> &_speedinfo_pub_ptr,
                 const boost::shared_ptr<ros::Publisher> &_rotateinfo_pub_ptr,
                 const boost::shared_ptr<ros::Publisher> &_rawimu_pub_ptr,
                 std::vector<std::string> &message_types,const boost::shared_ptr<std::ostream> &of);
    void Create(boost::asio::io_service &ios);
    void setGPSPublisher(const boost::shared_ptr<ros::Publisher> &gps_pub);
    void setIMUPublisher(const boost::shared_ptr<ros::Publisher> &imu_pub);
};

GPSPublisher::GPSPublisher(const std::string &portName, unsigned int baudRate,const boost::shared_ptr<ros::Publisher> &gps_pub,
                           const boost::shared_ptr<ros::Publisher> &gpsinfo_pub,
                           const boost::shared_ptr<ros::Publisher> &speedinfo_pub,
                           const boost::shared_ptr<ros::Publisher> &rotateinfo_pub,
                           const boost::shared_ptr<ros::Publisher> &rawimuinfo_pub,
                           std::vector<std::string> &message_types,const boost::shared_ptr<std::ostream> &of):
   _portName(portName),_baudRate(baudRate),_gps_pub_ptr(gps_pub),_gpsinfo_pub_ptr(gpsinfo_pub),
   _speedinfo_pub_ptr(speedinfo_pub),_rotateinfo_pub_ptr(rotateinfo_pub),_rawimu_pub_ptr(rawimuinfo_pub),_of(of)
{
    //_portName=portName;
    //_baudRate=baudRate;
    //_gps_pub_ptr=gps_pub;
    _epoch=(boost::posix_time::ptime )(boost::gregorian::date(1970, 1, 1));
    //_oa=oa;

    _msg_map_total=0;
    _msg_map_tmp=0;

    _decoder=boost::shared_ptr<GPSLib::GPSSentenceDecoder>(new GPSLib::GPSSentenceDecoder);
    initMsgTypeMap(message_types);
    
}

void GPSPublisher::setGPSPublisher(const boost::shared_ptr<ros::Publisher> &gps_pub)
{
}

void GPSPublisher::setIMUPublisher(const boost::shared_ptr<ros::Publisher> &imu_pub)
{
    _imu_pub_ptr=imu_pub;
}

void GPSPublisher::Create(boost::asio::io_service &ios)
{
    try
    {

        std::cout << "START TO CAEATE" << std::endl;
        _decoder->OnGGA=boost::bind(&GPSPublisher::OnGGA,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        // add by kp
        _decoder->OnINS1= boost::bind(&GPSPublisher::OnINS1,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnINS2= boost::bind(&GPSPublisher::OnINS2,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnINS3= boost::bind(&GPSPublisher::OnINS3,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnRAWIMUA= boost::bind(&GPSPublisher::OnRAWIMUA,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);

        _decoder->OnRMC=boost::bind(&GPSPublisher::OnRMC,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnPSAT=boost::bind(&GPSPublisher::OnPSAT,shared_from_this(),_1,_2,_3,_4,_5);
        _decoder->OnPASHR=boost::bind(&GPSPublisher::OnPASHR,shared_from_this(),_1,_2,_3,_4,_5,_6);
        _decoder->OnFPD=boost::bind(&GPSPublisher::OnFPD,shared_from_this(),_1,_2,_3,_4,_5,_6,_7);
        _decoder->OnIMU=boost::bind(&GPSPublisher::OnIMU,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->setRTKHeading(false);
        _serialPort.reset(new ASIOLib::SerialPort(ios, _portName));
        _serialPort->Open(boost::bind(&GPSPublisher::OnRead,shared_from_this(),_1,_2,_3),_baudRate);
    }
    catch(const std::exception &e) 
    {
        std::cout << "GPSPublisher exception (create): " << e.what() << std::endl;	
    }
}

void GPSPublisher::PublishGPS()
{
    //publish gps
    _gps_pub_ptr->publish(_gps_data);
    _msg_map_tmp=0;
}

void GPSPublisher::OnRead(boost::asio::io_service &ios,const std::vector<unsigned char> &buffer,size_t bytesRead)
{
    const std::vector<unsigned char> v(buffer.begin(), buffer.begin()+bytesRead);
    std::cout<<"AddBytes start"<<"\n";
    _decoder->AddBytes(ios,v);

    if (_of) 
    {
       std::copy(v.begin(), v.end(), std::ostream_iterator<unsigned char>(*_of, ""));
    }
}

void GPSPublisher::initMsgTypeMap(std::vector<std::string> &message_types)
{
    // enum MSG_TYPE{GGA,RMC,GSV,GLL,GSA,PSAT,PASHR,FPD,IMU,INSPVAXA};
    for(int i=0;i<message_types.size();i++)
    {
        // need to be added
        if(message_types[i]=="GPGGA")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::GGA);
        }

        //add by kp
        if(message_types[i]=="INSPVAXA")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::INSPVAXA);
        }

        else if(message_types[i]=="RAWIMUA")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::RAWIMUA);
        }

        else if(message_types[i]=="GPRMC")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::RMC);         
        }
        else if(message_types[i]=="PSAT")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::PSAT);
			_decoder->setRTKHeading(true);
        }
        else if(message_types[i]=="GPGSV")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::GSV);
        }
        else if(message_types[i]=="GPGLL")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::GLL);
        }
        else if(message_types[i]=="GPGSA")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::GSA);
        }
        else if(message_types[i]=="PASHR")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::PASHR);
			_decoder->setRTKHeading(true);
        }
        else if(message_types[i]=="GPFPD")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::FPD);
			_decoder->setRTKHeading(true);
        }
                
    }
    std::cout<<"_msg_map_total="<<_msg_map_total<<std::endl;

}

void GPSPublisher:: setMsgTypeMap(unsigned int &map,unsigned int bit)
{
    unsigned int pos=(1<<bit);
    map|=pos;
}

void GPSPublisher::initGPSHeader(boost::posix_time::time_duration time)
{
    ros::Time rostime=ros::Time::fromBoost(time);
    gps_common::GPSFix gps_data;
    _gps_data=gps_data;
    std_msgs::Header header;
    header.stamp = rostime;
    header.seq = _gps_seq_cnt;
    header.frame_id = "map";//map.world.base_link
    _gps_data.header=header;
	_gps_data.track=last_useful_track;

    _gps_seq_cnt++;

}

void GPSPublisher::initNovGPSHeader(ros::Time time)
{
    std :: cout << "begin to init the NovGPSHeader" << std::endl;
    //ros::Time rostime=ros::Time::fromBoost(time);

    gps_driver::GPSInfo gpsInfo;
    gps_driver::SpeedInfo speedInfo;
    gps_driver::RotateInfo rotateInfo;


    _gps_Info = gpsInfo;
    _speed_Info = speedInfo;
    _rotate_Info = rotateInfo;

    std_msgs::Header header;
    header.stamp = time;
    header.seq = _gps_seq_cnt;
    header.frame_id = "gps";//map.world.base_link
    _gps_Info.header = header;
    _speed_Info.header = header;
    _rotate_Info.header = header;
    _gps_seq_cnt++;

}

void GPSPublisher::initNovIMUHeader(ros::Time time)
{
    std :: cout << "begin to init the NovIMUHeader" << std::endl;
    //ros::Time rostime=ros::Time::fromBoost(time);
    sensor_msgs::Imu rawimuInfo;
    _rawimu_info = rawimuInfo;
    std_msgs::Header header;
    header.stamp = time;
    header.seq = _gps_seq_cnt;
    header.frame_id = "imu";//map.world.base_link
    _rawimu_info.header = header;
    _imu_seq_cnt++;
    std::cout << "the _imu cunt is: " << _imu_seq_cnt << std::endl;
}

void GPSPublisher::initIMUHeader(boost::posix_time::time_duration time)
{
    ros::Time rostime=ros::Time::fromBoost(time);
    sensor_msgs::Imu imu_data;
    _imu_data=imu_data;
    std_msgs::Header header;
    header.stamp = rostime;
    header.seq = _imu_seq_cnt;
    header.frame_id = "rslidar";//map.world.base_link
    _imu_data.header=header;
    _imu_seq_cnt++;

}

void GPSPublisher::OnFPD(boost::asio::io_service &ios, boost::posix_time::time_duration time,const std::vector<double> &pose_vect, const std::vector<double> &geo_vect, double speed,int numSatellites,int status_code)
{
    //std::cout<<"OnFPD start"<<"\n";
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
         //std::cout<<"OnFPD initGPSHeader start"<<"\n";
        initGPSHeader(time);
    }

    double Vu=0;
    double Ve=0;
    double Vn=0;

    gps_common::GPSStatus status;
    status.satellites_used=numSatellites;
    status.satellites_visible=numSatellites;
    status.status=status_code;
    _gps_data.status=status;
    _gps_data.latitude=geo_vect[0];
    _gps_data.longitude=geo_vect[1];
    _gps_data.altitude=geo_vect[2];

    //_gps_data.track=heading;//degrees from north
	if(pose_vect[0]>(NULL_ANGLE))
	{
        _gps_data.track=pose_vect[0];//degrees from north
		last_useful_track=pose_vect[0];
	}	
	
	
    _gps_data.pitch=pose_vect[1];
    _gps_data.roll=pose_vect[2];

    _gps_data.err_horz=Ve; //east speed
    _gps_data.err_track=Vn;//nort speed
    _gps_data.err_vert=Vu; //up speed

    _gps_data.speed = speed;

     //convert to 0~2PI
     double radius=M_PI / 180.0;
     tf::Quaternion q;
     q.setRPY(_gps_data.roll*radius, _gps_data.pitch*radius, _gps_data.track*radius);
     _orientation.x = q.x();
     _orientation.y = q.y();
     _orientation.z = q.z();
     _orientation.w = q.w();

    //std::cout<<"setMsgTypeMap start"<<"\n";
    setMsgTypeMap(_msg_map_tmp,MSG_TYPE::FPD);
    //std::cout<<"_msg_map_tmp=%d"<<_msg_map_tmp<<"\n";
    //std::cout<<"_msg_map_total=%d"<<_msg_map_total<<"\n";
    if((_msg_map_tmp&_msg_map_total)==_msg_map_total)
    {
        //std::cout<<"OnFPD PublishGPS start"<<"\n";
        PublishGPS();
    }
    else if(_msg_map_tmp>_msg_map_total)
    {
        //std::cout<<"_msg_map_tmp>_msg_map_total"<<"\n";
        _msg_map_tmp=0;
    }

}

void GPSPublisher::OnIMU(boost::asio::io_service &ios, boost::posix_time::time_duration time, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ)
{

    //std::cout<<"OnIMU start"<<"\n";
    initIMUHeader(time);

    _imu_data.orientation = _orientation;
    _imu_data.angular_velocity.x = gyroX;
    _imu_data.angular_velocity.y = gyroY;
    _imu_data.angular_velocity.z = gyroZ;
    _imu_data.linear_acceleration.x = accX;
    _imu_data.linear_acceleration.y= accY;
    _imu_data.linear_acceleration.z = accZ;

    _imu_pub_ptr->publish(_imu_data);
}

void GPSPublisher::OnINS1(boost::asio::io_service &ios,ros::Time time,double latitude, double longitude, double elevation ,
                          double latStaDev,double lonStaDev,double eleStaDev)
{
    std::cout<<"OnINS1 start"<<"\n";
    initNovGPSHeader(time);

    std::cout << "the latitude of INS1 is: " << latitude << std::endl;

    _gps_Info.latitude = latitude;
    _gps_Info.longitude = longitude;
    _gps_Info.elevation = elevation;
    _gps_Info.latStaDev = latStaDev;
    _gps_Info.lonStaDev = lonStaDev;
    _gps_Info.eleStaDev = eleStaDev;

    _gpsinfo_pub_ptr->publish(_gps_Info);
}

void GPSPublisher::OnINS2(boost::asio::io_service &ios, ros::Time time,double northAcc, double eastAcc, double diuAcc ,double nVelStaDev,
            double eVelStaDev,double dVelStaDev)
{
    std::cout<<"OnINS2 start"<<"\n";

    initNovGPSHeader(time);

    _speed_Info.northAcc = northAcc;
    _speed_Info.eastAcc = eastAcc;
    _speed_Info.diuAcc = diuAcc;
    _speed_Info.nVelStaDev = nVelStaDev;
    _speed_Info.eVelStaDev = eVelStaDev;
    _speed_Info.dVelStaDev = dVelStaDev;

    _speedinfo_pub_ptr->publish(_speed_Info);


}

void GPSPublisher::OnINS3(boost::asio::io_service &ios, ros::Time time,double rollAng, double pitAng, double headAng ,double rStaDev,
            double pStaDev,double hStaDev)
{
    std::cout<<"OnINS3 start"<<"\n";
    initNovGPSHeader(time);

    _rotate_Info.rollAng = rollAng;
    _rotate_Info.pitAng = pitAng;
    _rotate_Info.headAng = headAng;
    _rotate_Info.rStaDev = rStaDev;
    _rotate_Info.pStaDev = pStaDev;
    _rotate_Info.hStaDev = hStaDev;

    _rotateinfo_pub_ptr->publish(_rotate_Info);

}

void GPSPublisher::OnRAWIMUA(boost::asio::io_service &ios, ros::Time time,double zACC,double yACC,double xAcc,double rollVec,double yawVec,double pitchVec)
{
    std::cout<<"OnRAWIMUSA start"<<"\n";
    //unsigned int last_msg_map_tmp=_msg_map_tmp;

    initNovIMUHeader(time);  // need to be modified
    //  }

    _rawimu_info.linear_acceleration.x = xAcc;
    _rawimu_info.linear_acceleration.y = yACC;
    _rawimu_info.linear_acceleration.z = zACC;
    _rawimu_info.angular_velocity.x = pitchVec;
    _rawimu_info.angular_velocity.y = yawVec;
    _rawimu_info.angular_velocity.z = rollVec;

    _rawimu_pub_ptr->publish(_rawimu_info);

}


void GPSPublisher::OnGGA(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, int quality, int numSatellites, double horizontalDilution, double altitude)
{

    //std::cout<<"OnGGA start"<<"\n";
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
         //std::cout<<"OnGGA initGPSHeader start"<<"\n";
        initGPSHeader(time);
    }

    gps_common::GPSStatus status;
    status.satellites_used=numSatellites;
    status.satellites_visible=numSatellites;
    status.status=quality;
    _gps_data.status=status;

    _gps_data.latitude=latitude;
    _gps_data.longitude=longitude;
    _gps_data.altitude=altitude;

    setMsgTypeMap(_msg_map_tmp,MSG_TYPE::GGA);
    //std::cout<<"OnGGA map="<<(_msg_map_tmp&_msg_map_total)<<std::endl;
    if((_msg_map_tmp&_msg_map_total)==_msg_map_total)
    {
        //std::cout<<"OnGGA PublishGPS start"<<"\n";
        PublishGPS();
    }
    else if(_msg_map_tmp>_msg_map_total)
    {
        _msg_map_tmp=0;
    }
    //std::cout<<"OnGGA end"<<"\n";
}

void GPSPublisher::OnRMC(boost::asio::io_service &, boost::posix_time::time_duration time, double latitude, double longitude, double speed, double course, boost::gregorian::date date, const std::string &validity)
{
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
        initGPSHeader(time);
    }
    
    if(_gps_data.latitude==0)
    {
        _gps_data.latitude=latitude;
    }
 
    if(_gps_data.longitude==0)
    {
        _gps_data.longitude=longitude;
    }
	
	if(course>(NULL_ANGLE))
	{
        _gps_data.track=course;//degrees from north
		last_useful_track=course;
	}
	
    _gps_data.speed=speed;

    setMsgTypeMap(_msg_map_tmp,(unsigned int)MSG_TYPE::RMC);
    //std::cout<<"OnRMC map"<<(_msg_map_tmp&_msg_map_total)<<std::endl;
    if((_msg_map_tmp&_msg_map_total)==_msg_map_total)
    {
        PublishGPS();
    }
    else if(_msg_map_tmp>_msg_map_total)
    {
        _msg_map_tmp=0;
    }
}

void GPSPublisher::OnGSV(boost::asio::io_service &, int totalMessages, int messageNumber, int totalSatellitesInView, const std::vector<GPSLib::SatelliteInfo> &satelliteInfo)
{

}

void GPSPublisher::OnGLL(boost::asio::io_service &, boost::posix_time::time_duration, double latitude, double longitude, const std::string &validity)
{

}

void GPSPublisher::OnGSA(boost::asio::io_service &, const std::string &mode, int fix, const std::vector<int> &satellitesInView, double pdop, double hdop, double vdop)
{

    
}

void GPSPublisher::OnPSAT(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, double pitch, double roll)
{
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
        initGPSHeader(time);
    }

    //_gps_data.track=heading;
	if(heading>(NULL_ANGLE))
	{
        _gps_data.track=heading;//degrees from north
		last_useful_track=heading;
	}		
    _gps_data.pitch=pitch;
    _gps_data.roll=roll;

    setMsgTypeMap(_msg_map_tmp,(unsigned int)MSG_TYPE::PSAT);
    //std::cout<<"OnPSAT map="<<(_msg_map_tmp&_msg_map_total)<<std::endl;
    if((_msg_map_tmp&_msg_map_total)==_msg_map_total)
    {
        PublishGPS();
    }
    else if(_msg_map_tmp>_msg_map_total)
    {
        _msg_map_tmp=0;
    }
}

void GPSPublisher::OnPASHR(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, bool true_heading,double pitch, double roll)
{
    //std::cout<<"OnPASHR start"<<"\n";
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
        //std::cout<<"OnPASHR initGPSHeader start"<<"\n";
       //init message header
        initGPSHeader(time);
    }

    //_gps_data.track=heading;
	if(heading>(NULL_ANGLE))
	{
        _gps_data.track=heading;//degrees from north
		last_useful_track=heading;
	}		
    _gps_data.pitch=pitch;
    _gps_data.roll=roll;

    setMsgTypeMap(_msg_map_tmp,(unsigned int)MSG_TYPE::PASHR);
    //std::cout<<"OnPSAT map="<<(_msg_map_tmp&_msg_map_total)<<std::endl;
    if((_msg_map_tmp&_msg_map_total)==_msg_map_total)
    {
        //std::cout<<"OnPASHR PublishGPS start"<<"\n";
        PublishGPS();
    }
    else if(_msg_map_tmp>_msg_map_total)
    {
        _msg_map_tmp=0;
    }
    //std::cout<<"OnPASHR end"<<"\n";
}

int main(int argc,char **argv)
{
    std::cout << "BEGINING AAAA" << std::endl;
    std::string portName="/dev/ttyUSB0";
    int baudRate=115200;
    std::string file;
    bool b_GPGGA=false;
    bool b_GPRMC=false;
    bool b_PSAT=false;
    bool b_PASHR=false;
    bool b_FPD=false;
    bool b_IMU=false;
    bool b_INS=false;
    bool b_RAWIMUA = false;


    ros::init(argc,argv,"gps_publisher",ros::init_options::NoSigintHandler);//一定要添加ros::init_options::NoSigintHandler，禁用ROS的ctl+C,在ASIOLib::Executor里面处理ctl+C

    boost::program_options::options_description desc("Options");
    desc.add_options()
    	("help,h", "help")
    	("port,p", boost::program_options::value<std::string>(&portName), "port name (required)")
    	("baud,b", boost::program_options::value<int>(&baudRate), "baud rate (required)")
        ("GPGGA,g", boost::program_options::value<bool>(&b_GPGGA), "GPGGA message")
        // add by kp
        ("INSPVAXA,n", boost::program_options::value<bool>(&b_INS), "INSPVAXA message")
        ("RAWIMUA,k", boost::program_options::value<bool>(&b_RAWIMUA), "RAWIMUA message")

        ("GPRMC,r", boost::program_options::value<bool>(&b_GPRMC), "GPRMC message")
        ("PSAT,s", boost::program_options::value<bool>(&b_PSAT), "PSAT message")
        ("PASHR,a", boost::program_options::value<bool>(&b_PASHR), "PASHR message")
        ("GPFPD,d", boost::program_options::value<bool>(&b_FPD), "GPFPD message")
        ("IMU,i", boost::program_options::value<bool>(&b_IMU), "GTIMU message")
    	("file,f", boost::program_options::value<std::string>(&file), "file to save to")
    	;
	std::cout << "process the gps*************" << std::endl;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.empty() || vm.count("help")) 
    {

        ros::param::get("~port_name",portName);
        ros::param::get("~baud_rate",baudRate);
        ros::param::get("~save_file",file);
	    std::cout << desc << "\n";
        std::cout <<"default: -p "<<portName<<" -b "<<baudRate<<"\n";
    }
    boost::program_options::notify(vm);  


    ros::NodeHandle node_handle;

    ros::Publisher gps_pub=node_handle.advertise<gps_common::GPSFix>("/fix",10);
    boost::shared_ptr<ros::Publisher> gps_pub_ptr(&gps_pub);

    ros::Publisher gpsinfo_pub=node_handle.advertise<gps_driver::GPSInfo>("/gpsinfo",10);
    boost::shared_ptr<ros::Publisher>gpsinfo_pub_ptr(&gpsinfo_pub);

    ros::Publisher speedinfo_pub=node_handle.advertise<gps_driver::SpeedInfo>("/speedinfo",10);
    boost::shared_ptr<ros::Publisher>speedinfo_pub_ptr(&speedinfo_pub);

    ros::Publisher rotateinfo_pub=node_handle.advertise<gps_driver::RotateInfo>("/rotateinfo",10);
    boost::shared_ptr<ros::Publisher>rotateinfo_pub_ptr(&rotateinfo_pub);

    ros::Publisher rawimuinfo_pub=node_handle.advertise<sensor_msgs::Imu>("/rawimuinfo",10);
    boost::shared_ptr<ros::Publisher>rawimuinfo_pub_ptr(&rawimuinfo_pub);


//    ros::Publisher imu_pub=node_handle.advertise<sensor_msgs::Imu>("/imu",10);
//    boost::shared_ptr<ros::Publisher> imu_pub_ptr(&imu_pub);

    const boost::shared_ptr<std::ostream> out(file.empty() ? 0 : new std::ofstream(file.c_str()));

    std::vector<std::string> types;

    if(b_GPGGA)
    {
        ROS_INFO("set $GPGGA");
        types.push_back("GPGGA");
    }

    if(b_INS)
    {
        ROS_INFO("set INSPVAXA");
        types.push_back("INSPVAXA");
    }

    if(b_RAWIMUA)
    {
        ROS_INFO("set RAWIMUA");
        types.push_back("RAWIMUA");
    }


    if(b_GPRMC)
    {
        ROS_INFO("set $GPRMC");
        types.push_back("GPRMC");
    }

    if(b_PSAT)
    {
        ROS_INFO("set $PSAT");
        types.push_back("PSAT");
    }

    if(b_PASHR)
    {
        ROS_INFO("set $PASHR");
        types.push_back("PASHR");
    }

    if(b_FPD)
    {
        ROS_INFO("set $GPFPD");
        types.push_back("GPFPD");
    }


    ASIOLib::Executor e;
    e.OnWorkerThreadError = [](boost::asio::io_service &, boost::system::error_code ec) { Log(std::string("GPSPublisher error (asio): ") + boost::lexical_cast<std::string>(ec)); };
    e.OnWorkerThreadException = [](boost::asio::io_service &, const std::exception &ex) { Log(std::string("GPSPublisher exception (asio): ") + ex.what()); };



    boost::shared_ptr<GPSPublisher> spd(new GPSPublisher(portName, baudRate, gps_pub_ptr,
                                                         gpsinfo_pub_ptr,
                                                         speedinfo_pub_ptr,
                                                         rotateinfo_pub_ptr,
                                                         rawimuinfo_pub_ptr,
                                                         types,out));  // for shared_from_this() to work inside of Reader, Reader must already be managed by a smart pointer
    //spd->setIMUPublisher(imu_pub_ptr);

    e.OnRun = boost::bind(&GPSPublisher::Create, spd, _1);
    e.Run();  
    sleep(10);  
   // system("pause");
    std::cout<<"exit"<< std::endl;

    return 0;


}
