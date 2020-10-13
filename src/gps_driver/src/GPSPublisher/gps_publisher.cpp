#include "ros/ros.h"
#include <cstdlib>
#include <map>
#include <string>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "../ASIOLib/Executor.h"
#include "../ASIOLib/SerialPort.h"
#include "../GPSLib/GPSSentenceDecoder.h"
#include <sensor_msgs/Imu.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

boost::mutex cout_lock;
void Log(const std::string &msg) 
{
	boost::mutex::scoped_lock lock(cout_lock);
	std::cout << "[" << boost::this_thread::get_id() << "] " << msg << std::endl;
}



class GPSPublisher :public boost::enable_shared_from_this<GPSPublisher>
{

    enum MSG_TYPE{GGA,RMC,GSV,GLL,GSA,PSAT,PASHR,FPD,IMU};
    unsigned int _gps_seq_cnt=0;
    unsigned int _imu_seq_cnt=0;
    unsigned int _msg_map_total=0;
    unsigned int _msg_map_tmp=0;
    gps_common::GPSFix _gps_data;
    sensor_msgs::Imu _imu_data;
    geometry_msgs::Quaternion _orientation;

    boost::shared_ptr<ASIOLib::SerialPort> _serialPort;
    const std::string _portName;
    const unsigned int _baudRate;
    boost::shared_ptr<GPSLib::GPSSentenceDecoder> _decoder; // so shared_from_this() will work  
    const boost::shared_ptr<ros::Publisher> _gps_pub_ptr;
    boost::shared_ptr<ros::Publisher> _imu_pub_ptr;
    boost::posix_time::ptime _epoch;
    const boost::shared_ptr<std::ostream> _of;

    void initMsgTypeMap(std::vector<std::string> &message_types);
    void setMsgTypeMap(unsigned int &map,unsigned int bit);
    void initGPSHeader();
    void initIMUHeader();
    void PublishGPS();
    void OnRead(boost::asio::io_service &ios, const std::vector<unsigned char> &buffer, size_t bytesRead);
    void OnGGA(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, int quality, int numSatellites, double horizontalDilution, double altitude);
    void OnRMC(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, double speed, double course, boost::gregorian::date date, const std::string &validity);
    void OnGSV(boost::asio::io_service &ios, int totalMessages, int messageNumber, int totalSatellitesInView, const std::vector<GPSLib::SatelliteInfo> &satelliteInfo);
    void OnGLL(boost::asio::io_service &ios, boost::posix_time::time_duration, double latitude, double longitude, const std::string &validity);
    void OnGSA(boost::asio::io_service &ios, const std::string &mode, int fix, const std::vector<int> &satellitesInView, double pdop, double hdop, double vdop);
    void OnPSAT(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, double pitch, double roll);
    void OnPASHR(boost::asio::io_service &ios, boost::posix_time::time_duration time, double heading, bool true_heading,double pitch, double roll);
    void OnFPD(boost::asio::io_service &ios, double heading, double pitch, double roll, double latitude, double longitude,double altitude,double speed);
    void OnIMU(boost::asio::io_service &ios, boost::posix_time::time_duration time, double gyroX, double gyroY, double gyroZ, double accX, double accY, double accZ);
public:
    GPSPublisher(const std::string &portName, unsigned int baudRate,const boost::shared_ptr<ros::Publisher> &gps_pub,std::vector<std::string> &message_types,const boost::shared_ptr<std::ostream> &of);
    void Create(boost::asio::io_service &ios);
    void setGPSPublisher(const boost::shared_ptr<ros::Publisher> &gps_pub);
    void setIMUPublisher(const boost::shared_ptr<ros::Publisher> &imu_pub);
};

GPSPublisher::GPSPublisher(const std::string &portName, unsigned int baudRate,const boost::shared_ptr<ros::Publisher> &gps_pub,std::vector<std::string> &message_types,const boost::shared_ptr<std::ostream> &of):
   _portName(portName),_baudRate(baudRate),_gps_pub_ptr(gps_pub),_of(of)
{
    //_portName=portName;
    //_baudRate=baudRate;
    //_gps_pub_ptr=gps_pub;
    _epoch=(boost::posix_time::ptime )(boost::gregorian::date(1970, 1, 1));
    //_oa=oa;

    _msg_map_total=0;
    _msg_map_tmp=0;

    initMsgTypeMap(message_types);


    _decoder=boost::shared_ptr<GPSLib::GPSSentenceDecoder>(new GPSLib::GPSSentenceDecoder);
    
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
        _decoder->OnGGA=boost::bind(&GPSPublisher::OnGGA,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnRMC=boost::bind(&GPSPublisher::OnRMC,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnPSAT=boost::bind(&GPSPublisher::OnPSAT,shared_from_this(),_1,_2,_3,_4,_5);
        _decoder->OnPASHR=boost::bind(&GPSPublisher::OnPASHR,shared_from_this(),_1,_2,_3,_4,_5,_6);
        _decoder->OnFPD=boost::bind(&GPSPublisher::OnFPD,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);
        _decoder->OnIMU=boost::bind(&GPSPublisher::OnIMU,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);

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
    //_decoder->OnGGA=[&](boost::asio::io_service &ios,boost::posix_time::time_duration time,double latitude,double longitude,int quality,int numSatellites, double horizontalDilution, double altitude){
    //   PublishGPS( boost::posix_time::ptime(_epoch + time),latitude,longitude,altitude);
    //};
    //_decoder->OnGGA=boost::bind(&GPSPublisher::OnGGA,shared_from_this(),_1,_2,_3,_4,_5,_6,_7,_8);


    const std::vector<unsigned char> v(buffer.begin(), buffer.begin()+bytesRead);

    //std::cout<<"AddBytes start"<<"\n";

    _decoder->AddBytes(ios,v);

    if (_of) 
    { 	
       //std::copy(v.begin(), v.end(), std::ostream_iterator<unsigned char>(std::cout, ""));
       std::copy(v.begin(), v.end(), std::ostream_iterator<unsigned char>(*_of, ""));	
	//*_of <<v;
    }
}

void GPSPublisher::initMsgTypeMap(std::vector<std::string> &message_types)
{

    for(int i=0;i<message_types.size();i++)
    {
        if(message_types[i]=="GPGGA")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::GGA);
        }
        else if(message_types[i]=="GPRMC")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::RMC);         
        }
        else if(message_types[i]=="PSAT")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::PSAT);
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
        }
        else if(message_types[i]=="GPFPD")
        {
            setMsgTypeMap(_msg_map_total,(unsigned int)MSG_TYPE::FPD);
        }
                
    }
    std::cout<<"_msg_map_total="<<_msg_map_total<<std::endl;

}

void GPSPublisher::setMsgTypeMap(unsigned int &map,unsigned int bit)
{
    unsigned int pos=(1<<bit);
    map|=pos;
}

void GPSPublisher::initGPSHeader()
{
    gps_common::GPSFix gps_data;
    _gps_data=gps_data;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = _gps_seq_cnt;
    header.frame_id = "map";//map.world.base_link
    _gps_data.header=header;

    _gps_seq_cnt++;
   
}

void GPSPublisher::initIMUHeader()
{
    sensor_msgs::Imu imu_data;
    _imu_data=imu_data;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = _imu_seq_cnt;
    header.frame_id = "velodyne";//map.world.base_link
    _imu_data.header=header;

    _imu_seq_cnt++;

}

void GPSPublisher::OnFPD(boost::asio::io_service &ios, double heading, double pitch, double roll, double latitude, double longitude,double altitude,double speed)
{
    //std::cout<<"OnFPD start"<<"\n";
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
         //std::cout<<"OnFPD initGPSHeader start"<<"\n";
        initGPSHeader();
    }

    double Vu=0;
    double Ve=0;
    double Vn=0;

    _gps_data.latitude=latitude;
    _gps_data.longitude=longitude;
    _gps_data.altitude=altitude;

    _gps_data.track=heading;//degrees from north
    _gps_data.pitch=pitch;
    _gps_data.roll=roll;

    _gps_data.err_horz=Ve; //east speed
    _gps_data.err_track=Vn;//nort speed
    _gps_data.err_vert=Vu; //up speed

    _gps_data.speed = speed;

     //convert to 0~2PI
     double radius=M_PI / 180.0;
     tf::Quaternion q;
     q.setRPY(roll*radius, pitch*radius, heading*radius);
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
    initIMUHeader();

    _imu_data.orientation = _orientation;
    _imu_data.angular_velocity.x = gyroX;
    _imu_data.angular_velocity.y = gyroY;
    _imu_data.angular_velocity.z = gyroZ;
    _imu_data.linear_acceleration.x = accX;
    _imu_data.linear_acceleration.y= accY;
    _imu_data.linear_acceleration.z = accZ;

    _imu_pub_ptr->publish(_imu_data);
}

void GPSPublisher::OnGGA(boost::asio::io_service &ios, boost::posix_time::time_duration time, double latitude, double longitude, int quality, int numSatellites, double horizontalDilution, double altitude)
{

    //std::cout<<"OnGGA start"<<"\n";
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
         //std::cout<<"OnGGA initGPSHeader start"<<"\n";
        initGPSHeader();
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


void GPSPublisher::OnRMC(boost::asio::io_service &, boost::posix_time::time_duration, double latitude, double longitude, double speed, double course, boost::gregorian::date date, const std::string &validity)
{
    unsigned int last_msg_map_tmp=_msg_map_tmp;
    if(last_msg_map_tmp==0)
    {
       //init message header
        initGPSHeader();
    }
    
    if(_gps_data.latitude==0)
    {
        _gps_data.latitude=latitude;
    }
 
    if(_gps_data.longitude==0)
    {
        _gps_data.longitude=longitude;
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
        initGPSHeader();
    }

    _gps_data.track=heading;
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
        initGPSHeader();
    }

    _gps_data.track=heading;
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
    std::string portName="/dev/ttyUSB0";
    int baudRate=115200;
    std::string file;
    bool b_GPGGA=false;
    bool b_GPRMC=false;
    bool b_PSAT=false;
    bool b_PASHR=false;
    bool b_FPD=false;

    ros::init(argc,argv,"gps_publisher",ros::init_options::NoSigintHandler);//一定要添加ros::init_options::NoSigintHandler，禁用ROS的ctl+C,在ASIOLib::Executor里面处理ctl+C

    boost::program_options::options_description desc("Options");
    desc.add_options()
    	("help,h", "help")
    	("port,p", boost::program_options::value<std::string>(&portName), "port name (required)")
    	("baud,b", boost::program_options::value<int>(&baudRate), "baud rate (required)")
        ("GPGGA,g", boost::program_options::value<bool>(&b_GPGGA), "GPGGA message")
        ("GPRMC,r", boost::program_options::value<bool>(&b_GPRMC), "GPRMC message")
        ("PSAT,s", boost::program_options::value<bool>(&b_PSAT), "PSAT message")
        ("PASHR,a", boost::program_options::value<bool>(&b_PASHR), "PASHR message")
        ("GPFPD,d", boost::program_options::value<bool>(&b_FPD), "GPFPD message")
 //       ("IMU,i", boost::program_options::value<bool>(&b_IMU), "GTIMU message")
    	("file,f", boost::program_options::value<std::string>(&file), "file to save to")
    	;
	
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

    ros::Publisher imu_pub=node_handle.advertise<sensor_msgs::Imu>("/imu",10);
    boost::shared_ptr<ros::Publisher> imu_pub_ptr(&imu_pub);

    const boost::shared_ptr<std::ostream> out(file.empty() ? 0 : new std::ofstream(file.c_str()));
    //const boost::scoped_ptr<boost::archive::text_oarchive> archive(file.empty() ? 0 : new boost::archive::text_oarchive(*out));

    //ros::Rate loop_rate(10); 
    //while(ros::ok())
    //{
    //    ros::spinOnce();
    //    loop_rate.sleep();
    //}
    
    //std::cout<< file<<"\n";
    std::vector<std::string> types;

    if(b_GPGGA)
    {
        ROS_INFO("set $GPGGA");
        types.push_back("GPGGA");
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
    
    boost::shared_ptr<GPSPublisher> spd(new GPSPublisher(portName, baudRate, gps_pub_ptr,types,out));  // for shared_from_this() to work inside of Reader, Reader must already be managed by a smart pointer
    spd->setIMUPublisher(imu_pub_ptr);

    e.OnRun = boost::bind(&GPSPublisher::Create, spd, _1);
    e.Run();

    return 0;


}
