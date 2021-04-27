#ifndef __GPSSENTENCEDECODER_H__
#define __GPSSENTENCEDECODER_H__

#include <vector>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include "GPSLib_Export.h"
#include "ros/ros.h"

#define NULL_ANGLE -360
#define GYRO_SCALE (0.008/65536)/125
#define ACC_SCALE (0.200/65536)/125

namespace GPSLib {
	class SatelliteInfo {
	public:
	    // elevation 海拔 azimuth 方位角
		int _prn, _elevation, _azimuth, _snr;
		SatelliteInfo(int prn, int elevation, int azimuth, int snr) :
		_prn(prn), _elevation(elevation), _azimuth(azimuth), _snr(snr) {}
	};

	class GPSLib_Export GPSSentenceDecoder : public boost::enable_shared_from_this<GPSSentenceDecoder> {
		std::string _buffer;
		boost::mutex _bufferMutex;
		boost::shared_ptr<boost::asio::strand> _decodeStrand;
		bool _use_rtk_heading;
		void Decode(boost::asio::io_service &ios, const std::string &s);
	public:
		void AddBytes(boost::asio::io_service &ios, const std::vector<unsigned char> &buffer, size_t bufferSize = -1); 
		boost::function<void (boost::asio::io_service &, const std::string &)> OnInvalidSentence;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, double, int, int, double, double)> OnGGA;
		// add by kp
        boost::function<void (boost::asio::io_service &, ros::Time,double, double, double, double, double, double)> OnINS1;
        boost::function<void (boost::asio::io_service &, ros::Time,double, double, double, double, double, double)> OnINS2;
        boost::function<void (boost::asio::io_service &, ros::Time,double, double, double, double, double, double)> OnINS3;
        boost::function<void (boost::asio::io_service &, ros::Time,double, double, double, double, double,double)> OnRAWIMUA;

		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, double, double, double, boost::gregorian::date, const std::string &)> OnRMC;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, double, double)> OnPSAT;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, bool, double,double)> OnPASHR;
		boost::function<void (boost::asio::io_service &, int, int, int, const std::vector<SatelliteInfo> &)> OnGSV;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, double, const std::string &)> OnGLL;
		boost::function<void (boost::asio::io_service &, const std::string &, int, const std::vector<int> &, double, double, double)> OnGSA;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, const std::vector<double> &, const std::vector<double> &,double,int,int)> OnFPD;
		boost::function<void (boost::asio::io_service &, boost::posix_time::time_duration, double, double, double, double, double, double)> OnIMU;
		void setRTKHeading(bool use_rtk_heading);
	};
}
#endif
