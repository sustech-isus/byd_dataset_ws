#include "GPSSentenceDecoder.h"
#include "Util.h"
#include <cctype>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp> 
#include <boost/regex.hpp>



void GPSLib::GPSSentenceDecoder::AddBytes(boost::asio::io_service &ios, const std::vector<unsigned char> &bufferToAdd, size_t bufferSize) {
	boost::mutex::scoped_lock lock(_bufferMutex);

	if (_decodeStrand == 0)
		_decodeStrand = boost::shared_ptr<boost::asio::strand>(new boost::asio::strand(ios));

	// pass bufferSize in case buffer has size greater than the amount of meaningful data in it
	std::for_each(bufferToAdd.begin(), (bufferSize == -1) ? bufferToAdd.end() : (bufferToAdd.begin() + bufferSize), [this](const unsigned char &c) {
		if (std::isprint(c) || (c=='\r') || (c == '\n'))  // a sentence is ASCII plus CR-LF - ignore anything out of that range
			_buffer += c;
		   // std::cout << "the buffer is: "<< _buffer << std::endl;
	});
	std::cout << "the buffer is: "<< _buffer << std::endl;

	size_t pos;
	
	while ( (pos = _buffer.find("\r\n")) != std::string::npos) {  // tokenize on \r\n which ends a string
		std::cout<<"while run"<<std::endl;
		//std::cout << "the buffer is: "<< _buffer << std::endl;
		const std::string s(_buffer.substr(0, pos+2));  // +2 for \r\n
		_buffer.erase(0, pos+2);
		// post this to io_service through a strand to keep order of decode the same as order of arrival (as some messages may decode faster than others)
		_decodeStrand->post(boost::bind(&GPSSentenceDecoder::Decode, shared_from_this(), boost::ref(ios), s));  
	}
	//std::cout<<_buffer<<std::endl;
	std::cout<<"while exit"<<std::endl ;
}


namespace {
//	bool TokenizeSentenceforGGA(const std::string &s, std::vector<std::string> &tokens) {
//		tokens.clear();
//		std::cout << "the s is: " << s << std::endl;
//		const std::string::size_type dollarPos = s.find("$");
//		const std::string::size_type CRLFPos = s.find("\r\n");
//		std::cout << "the CRLFPos is: " << CRLFPos << std::endl;
//		// fail if can't find sentence boundary, or sentence too short
//        // std::string::npos  the length of the string class
//		if ((dollarPos == std::string::npos) ||
//			(CRLFPos == std::string::npos) ||
//			(CRLFPos - dollarPos < 5)) {
//				return false;
//		}
//		std::string::size_type textEndPos = CRLFPos;
//		// 消息结束的标志位
//		const std::string::size_type starPos = s.rfind("*");
//		if (starPos != std::string::npos) {
//			// have a checksum, so validate it
//            std::cout << "the s[starPos+1] is: " << s[starPos+1] << std::endl;
//            std::cout << "the s[starPos+2] is: " << s[starPos+2] << std::endl;
//			const char expected_checksum_bytes[] = { s[starPos+1], s[starPos+2], 0 };  // checksum is just prior to the CRLF
//            std::cout << "the expected_checksum_bytes[0] is: " << expected_checksum_bytes[0] << std::endl;
//            std::cout << "the expected_checksum_bytes[1] is: " << expected_checksum_bytes[1] << std::endl;
//            std::cout << "the expected_checksum_bytes[2] is: " << expected_checksum_bytes[2] << std::endl;
//			const unsigned char expected_checksum = boost::lexical_cast<GPSLib::byte_from_hex>(expected_checksum_bytes);
//            std::cout << "the expected_checksum is: " << expected_checksum << std::endl;
//			unsigned char calculated_checksum = 0;
//			std::for_each(s.begin()+dollarPos+1, s.begin()+starPos, [&calculated_checksum](char c)
//			{ std::cout << calculated_checksum << c << std::endl; calculated_checksum^=c;std::cout << c << std::endl; });  // +1, since only chars between $ *
//
//			// fail if checksum mismatch
//			if (calculated_checksum != expected_checksum)
//				return false;
//
//			textEndPos = starPos;
//		}
//		// only tokenize from char following $ to before *, if it exists 按照逗号切分
//		const boost::char_separator<char> sep(",", 0, boost::keep_empty_tokens);
//		const boost::tokenizer<boost::char_separator<char>> t(s.begin()+dollarPos, s.begin()+textEndPos, sep);
//		std::copy(t.begin(), t.end(), std::back_inserter(tokens));
//		return tokens.size()>0;  // parse is good if there is at least one token
//	}

	// write by kp
    bool TokenizeSentence(const std::string &s, std::vector<std::string> &tokens) {
	    std::cout << "start to process TokenizeSentenceforINS" << std::endl;
        tokens.clear();
        //std::cout << "the s is: " << s << std::endl;
        //#INSPVAXA,COM1,0,73.5,FINESTEERING,1695,309428.000,00000040,4e77,43562;INS_SOLUTION_GOOD,INS_PSRSP,
        // 51.11637873403,-114.03825114994,1063.6093,-16.9000,-0.0845,-0.0464,-0.0127,0.138023492,0.069459386,
        // 90.000923268,0.9428,0.6688,1.4746,0.0430,0.0518,0.0521,0.944295466,0.944567084,1.000131845,000000C3,0*e877c178
        // need to be modified,start with "#" 找到开始的位置和结束的位置 凯品修改
        const std::string::size_type dollarPos = s.find("#");
		std::cout << "the dollarPos is: " <<  dollarPos << std::endl;
        //const std::string::size_type dollarPos = s.find("$");
        const std::string::size_type CRLFPos = s.find("\r\n");
        std::cout << "the CRLFPos is: " << CRLFPos << std::endl;
        // fail if can't find sentence boundary, or sentence too short
        // std::string::npos  the length of the string class
        if ((dollarPos == std::string::npos) ||
            (CRLFPos == std::string::npos) ||
            (CRLFPos - dollarPos < 5)) {
            return false;
        }

        std::string::size_type textEndPos = CRLFPos;
        // 消息结束的标志位
        const std::string::size_type starPos = s.rfind("*");
		std::cout << "the starpos is: " << starPos << std::endl;
        // if (starPos != std::string::npos) {
        //     // have a checksum, so validate it
        //     // maybe need to be modified,b'cause the leng of the *byte
        //     //std::cout << "the s[starPos+1] is: " << s[starPos+1] << std::endl;
        //     //std::cout << "the s[starPos+2] is: " << s[starPos+2] << std::endl;
        //     //?
        //     const char expected_checksum_bytes[] = { s[starPos+1], s[starPos+2], s[starPos+3],s[starPos+4],s[starPos+5],
		// 												s[starPos+6],s[starPos+7],s[starPos+8],0 };  // checksum is just prior to the CRLF
        //     std::cout << "the expected_checksum_bytes is: " << expected_checksum_bytes << std::endl;
		// 	//std::cout << "the expected_checksum_bytes[0] is: " << expected_checksum_bytes[0] << std::endl;
        //     //std::cout << "the expected_checksum_bytes[1] is: " << expected_checksum_bytes[1] << std::endl;
        //     //std::cout << "the expected_checksum_bytes[2] is: " << expected_checksum_bytes[2] << std::endl;
        //     const unsigned char expected_checksum = boost::lexical_cast<GPSLib::byte_from_hex>(expected_checksum_bytes);
        //     std::cout << "the expected_checksum is: " << expected_checksum << std::endl;

        //     unsigned char calculated_checksum = 0;
        //     std::for_each(s.begin()+dollarPos+1, s.begin()+starPos, [&calculated_checksum](char c)
        //     {
		// 		std::cout << "the ori calculated_checksum is: " << static_cast<unsigned>(calculated_checksum) <<std::endl;
		// 		std::cout << "the c is: " <<  c << std::endl; 
		// 		calculated_checksum^=c;
		// 		std::cout << "the reult is : " << calculated_checksum << std::endl;
            	
		// 	});  // +1, since only chars between $ *

        //     // fail if checksum mismatch
			
        //     if (calculated_checksum != expected_checksum)
        //         std::cout << "aaaaaaaaaa" << std::endl;
		// 		return false;

        //     textEndPos = starPos;
        // }
		textEndPos = starPos;
        // only tokenize from char following $ to before *, if it exists 按照逗号切分,其中包含一个分号
        const boost::char_separator<char> sep(",", 0, boost::keep_empty_tokens);
        const boost::tokenizer<boost::char_separator<char>> t(s.begin()+dollarPos, s.begin()+textEndPos, sep);
        //std::cout << "the tokened string is: " <<  << std::endl;
        std::copy(t.begin(), t.end(), std::back_inserter(tokens));
        //std::cout << "the tokened string is: " <<  tokens.<< std::endl;
		
		
		// for(int i = 0;i < tokens.size(); i++)
		// {
		// 	std::cout<< "the tokens[i] is: " << tokens[i] << std::endl;
		// }


        return tokens.size()>0;  // parse is good if there is at least one token
    }


	const boost::regex hms("(\\d{2})(\\d{2})(\\d{2})(?:(.\\d*))?");
	boost::posix_time::time_duration DecodeTime(std::vector<std::string>::iterator &i) {
		boost::smatch m;
		if (boost::regex_match(*i, m, hms)) {
			i++;  // consume the match
			const boost::posix_time::hours hr(boost::lexical_cast<int>(m[1].str()));
			const boost::posix_time::minutes min(boost::lexical_cast<int>(m[2].str()));
			const boost::posix_time::seconds sec(boost::lexical_cast<int>(m[3].str()));
			const boost::posix_time::milliseconds ms(boost::lexical_cast<int>(boost::lexical_cast<double>(m[4].str())*1000));
			return boost::posix_time::time_duration(hr+min+sec+ms);
		}
		else
			throw std::invalid_argument(*i + " is not hms");
	}

	const boost::regex latlng("(\\d{2,3})(\\d{2}.\\d+)");
	double DecodeLatLng(std::vector<std::string>::iterator &i) {
		boost::smatch m;
		if (boost::regex_match(*i, m, latlng)) {
			i++; // consume the match
			double deg = boost::lexical_cast<double>(m[1].str());
			double min = boost::lexical_cast<double>(m[2].str());
			std::string hemisphere = *i++;
			return GPSLib::ToDecimalDegree(deg, min, hemisphere);
		}
		else
			throw std::invalid_argument(*i + " is not lat or lng");
	}

	const boost::regex dmy("(\\d{2})(\\d{2})(\\d{2})");
	boost::gregorian::date DecodeDate(std::vector<std::string>::iterator &i) {
		boost::smatch m;
		if (boost::regex_match(*i, m, dmy)) {
			i++; // consume the match
			int day = boost::lexical_cast<int>(m[1].str());
			int month = boost::lexical_cast<int>(m[2].str());
			int year = boost::lexical_cast<int>(m[3].str()) + 2000;
			return boost::gregorian::date(year, month, day);
		}
		else
			throw std::invalid_argument(*i + " is not dmy");
	}
}

void GPSLib::GPSSentenceDecoder::setRTKHeading(bool use_rtk_heading)
{
	_use_rtk_heading=use_rtk_heading;
}

void GPSLib::GPSSentenceDecoder::Decode(boost::asio::io_service &ios, const std::string &s) {
	try {
                //std::cout<<"Decode"<<"\n";
		std::vector<std::string> v;
		if (!TokenizeSentence(s, v)) {
			if (OnInvalidSentence) 
				OnInvalidSentence(ios, s);
			return;
		}

		std::vector<std::string>::iterator i = v.begin();
		if ((*i == "$GPGGA")||(*i == "$GNGGA")) {
                        //std::cout<<"$GPGGA"<<"\n";
			// GGA = Global Positioning System Fix Data
			// $GPGGA,191630.609,3848.2905,N,09018.4239,W,1,06,1.3,132.0,M,-33.7,M,0.0,0000*48
			i++;  // consume the $GPGGA token

			const boost::posix_time::time_duration time(DecodeTime(i));
			const double lat = DecodeLatLng(i);
			const double lng = DecodeLatLng(i);
			const int quality = lexical_cast_default<int>(*i++, 0);
			const int numSatellites = lexical_cast_default<int>(*i++, 0);
			const double horizontalDilution = lexical_cast_default<double>(*i++, 0);
			const double altitude = lexical_cast_default<double>(*i++, 0);

			if (OnGGA)
				OnGGA(ios, time, lat, lng, quality, numSatellites, horizontalDilution, altitude);
			return;
		}

        if (*i == "#INSPVAXA")
        {
            //#INSPVAXA,COM1,0,73.5,FINESTEERING,1695,309428.000,00000040,4e77,43562;INS_SOLUTION_GOOD,INS_PSRSP,
            // 51.11637873403,-114.03825114994,1063.6093,-16.9000,-0.0845,-0.0464,-0.0127,0.138023492,0.069459386,
            // 90.000923268,0.9428,0.6688,1.4746,0.0430,0.0518,0.0521,0.944295466,0.944567084,1.000131845,000000C3,0*e877c178
            i = i + 11;  // consume the #INSPVAXA token
//            const boost::posix_time::time_duration time(DecodeTime(i));
//            const double lat = DecodeLatLng(i);
//            const double lng = DecodeLatLng(i);
            //current_stamp = ros::Time::now();
            //boost::posix_time::ptime  time_now ( boost::posix_time::second_clock::local_time() ); //机器的本地时间，比如北京时间，与电脑设置的时区有
            ros::Time time_now = ros::Time::now();
            double latitude = lexical_cast_default<double>(*i++, 0);  // 纬度 WGS84坐标系下的纬度
            std::cout << "the lat is: " << latitude << std::endl;
            double longitude = lexical_cast_default<double>(*i++, 0);  // 经度 WGS84坐标系下的经度
            std::cout << "the lng is: " << longitude << std::endl;
            double elevation = lexical_cast_default<double>(*i++, 0); // 高程 WGS84坐标系下的海拔高
            std::cout << "the elevation is: " << elevation << std::endl;
            double updouwn = lexical_cast_default<double>(*i++, 0); // 起伏 海拔高与大地高的差值
            std::cout << "the updouwn is: " << updouwn << std::endl;
            double northAcc = lexical_cast_default<double>(*i++, 0); // 北向速度 m/s
            std::cout << "the northAcc is: " << northAcc << std::endl;
            double eastAcc = lexical_cast_default<double>(*i++, 0); // 东向速度 m/s
            std::cout << "the eastAcc is: " << eastAcc << std::endl;
            double diuAcc = lexical_cast_default<double>(*i++, 0); // 天向速度 m/s
            std::cout << "the diuAcc is: " << diuAcc << std::endl;
            double rollAng = lexical_cast_default<double>(*i++, 0); // 横滚角
            std::cout << "the rollAng is: " << rollAng << std::endl;
            double pitAng = lexical_cast_default<double>(*i++, 0); // 俯仰角
            std::cout << "the pitAng is: " << pitAng << std::endl;
            double headAng = lexical_cast_default<double>(*i++, 0); // 航向角
            std::cout << "the headAng is: " << headAng << std::endl;
            double latStaDev = lexical_cast_default<double>(*i++, 0); // 纬度标差
            std::cout << "the latStaDev is: " << latStaDev << std::endl;
            double lonStaDev = lexical_cast_default<double>(*i++, 0); // 经度标差
            std::cout << "the lonStaDev is: " << lonStaDev << std::endl;
            double eleStaDev = lexical_cast_default<double>(*i++, 0); // 高程标差
            std::cout << "the EleStaDev is: " << eleStaDev << std::endl;
            double nVelStaDev = lexical_cast_default<double>(*i++, 0); // 北速标差
            std::cout << "the nVelStaDev is: " << nVelStaDev << std::endl;
            double eVelStaDev = lexical_cast_default<double>(*i++, 0); // 东速标差
            std::cout << "the eVelStaDev is: " << eVelStaDev << std::endl;
            double dVelStaDev = lexical_cast_default<double>(*i++, 0); // 天速标差
            std::cout << "the dVelStaDev is: " << dVelStaDev << std::endl;
            double rStaDev = lexical_cast_default<double>(*i++, 0); // 横滚标差
            std::cout << "the rStaDev is: " << rStaDev << std::endl;
            double pStaDev = lexical_cast_default<double>(*i++, 0); // 俯仰标差
            std::cout << "the pStaDev is: " << pStaDev << std::endl;
            double hStaDev = lexical_cast_default<double>(*i++, 0); // 航向标差
            std::cout << "the hStaDev is: " << hStaDev << std::endl;

            if (OnINS1)
                OnINS1(ios, time_now,latitude,  longitude,  elevation ,latStaDev, lonStaDev,eleStaDev);
            if (OnINS2)
                OnINS2(ios, time_now,northAcc,  eastAcc,  diuAcc ,nVelStaDev, eVelStaDev,dVelStaDev);
            if (OnINS3)
                OnINS3(ios, time_now,rollAng,  pitAng,  headAng ,rStaDev, pStaDev,hStaDev);




            return;
        }

        if (*i == "#RAWIMUA")
        {
            //#RAWIMUA,COM1,0,68.5,FINESTEERING,1695,309428.000,024c0040,6125,30019;1724,219418.008755000,
            // 00000077,64732,56,298,8,28,-3*7378486f
            i = i + 12;  // consume the #INSPVAXA token
//            const boost::posix_time::time_duration time(DecodeTime(i));
//            const double lat = DecodeLatLng(i);
//            const double lng = DecodeLatLng(i);
            //current_stamp = ros::Time::now();
            //boost::posix_time::ptime  time_now ( boost::posix_time::second_clock::local_time() ); //机器的本地时间，比如北京时间，与电脑设置的时区有
            ros::Time time_now = ros::Time::now();
            double zAcc = lexical_cast_default<long>(*i++, 0) * ACC_SCALE; // the acc of the z axis
           // std::cout << "the zAcc is: " << zAcc << std::endl;
            // the acc of the y axis right hand,the negative value implies the output is along the positive y-axis marked on imu
            double yAcc = -(lexical_cast_default<long>(*i++, 0) * ACC_SCALE);
            //std::cout << "the yAcc is: " << yAcc << std::endl;
            double xAcc = lexical_cast_default<long>(*i++, 0) * ACC_SCALE; // the acc of the x axis
           // std::cout << "the xAcc is: " << xAcc << std::endl;
            double rollVec = lexical_cast_default<long>(*i++, 0) * GYRO_SCALE; // the gyro vec of the z-axis
          //  std::cout << "the rollVec is: " << rollVec << std::endl;
            double yawVec = -(lexical_cast_default<long>(*i++, 0) * GYRO_SCALE); // the gyro vec of the y-axis
          //  std::cout << "the yawVec is: " << yawVec << std::endl;
            double pitchVec = lexical_cast_default<long>(*i++, 0) * GYRO_SCALE; // the gyro vec of the x-axis
         //   std::cout << "the pitchVec is: " << pitchVec << std::endl;

            if (OnRAWIMUA)
                OnRAWIMUA(ios, time_now,zAcc,  yAcc,  xAcc ,rollVec, yawVec,pitchVec);
            return;
        }


		if (*i == "$GPRMC") {
                        //std::cout<<"$GPRMC"<<"\n";
			// RMC = Recommended minimum specific GPS/Transit data 
			// $GPRMC,191632.609,A,3848.3005,N,09018.4051,W,32.523475,55.89,150113,,*14
			i++;  // consume the $GPRMC token
			boost::posix_time::time_duration time(DecodeTime(i));
			std::string validity(*i++);
			double lat = DecodeLatLng(i);
			double lng = DecodeLatLng(i);
			double speed = lexical_cast_default<double>(*i++, 0);  // knots
			double heading = NULL_ANGLE; //fake angle
			if(_use_rtk_heading)
			{
				i++;
				heading = (NULL_ANGLE)*2;
			}
			else
			{
				heading = lexical_cast_default<double>(*i++, NULL_ANGLE);
			}
			
			boost::gregorian::date date(DecodeDate(i));

			if (OnRMC)
				OnRMC(ios, time, lat, lng, speed, heading, date, validity);
			return;
		}


		if(*i == "$PSAT"){
                        i++;  // consume the $PSAT token
                        i++;  // consume the HPR token
			boost::posix_time::time_duration time(DecodeTime(i));
			double heading = lexical_cast_default<double>(*i++, NULL_ANGLE);  
			double pitch = lexical_cast_default<double>(*i++, 0);
			double roll = lexical_cast_default<double>(*i++, 0);

			if (OnPSAT)
				OnPSAT(ios, time, heading, pitch, roll);
			return;
		}


		if(*i == "$PASHR"){
                        i++;  // consume the $PASHR token
			boost::posix_time::time_duration time(DecodeTime(i));
			double heading = lexical_cast_default<double>(*i++, NULL_ANGLE);
 			std::string Theading = *i++;//lexical_cast_default<double>(*i++, 0); 
			double roll = lexical_cast_default<double>(*i++, 0);
			double pitch = lexical_cast_default<double>(*i++, 0);
                        bool true_heading=false;
                        if(Theading.size()>0)
                        {
                             true_heading=true;   
                        }
			if (OnPASHR)
				OnPASHR(ios, time, heading,true_heading,pitch, roll);
			return;
		}

		if (*i == "$GPGLL") {
                        //std::cout<<"$GPGLL"<<"\n";
			// GLL = Geographic Position, Latitude / Longitude and time
			// $GPGLL,3848.2905,N,09018.4239,W,191630.609,A*20
			i++;  // consume the $GPGLL token
			double lat = DecodeLatLng(i);
			double lng = DecodeLatLng(i);
			boost::posix_time::time_duration time(DecodeTime(i));
			std::string validity(*i++);

			if (OnGLL)
				OnGLL(ios, time, lat, lng, validity);
			return;
		}

		if (*i == "$GPGSV") {
                        //std::cout<<"$GPGSV"<<"\n";
			// GSV = GPS Satellites in view
			// $GPGSV,3,1,10,18,62,311,37,15,47,49,40,14,16,218,30,29,11,186,28*4A
			i++;  // consume the $GPGSV token
			int totalMessages = lexical_cast_default<int>(*i++, 0);
			int messageNumber = lexical_cast_default<int>(*i++, 0);
			int totalSatellitesInView = lexical_cast_default<int>(*i++, 0);
			std::vector<SatelliteInfo> satelliteInfo;
			while (i!=v.end()) {
				int prn = lexical_cast_default<int>(*i++, 0);
				int elevation = lexical_cast_default<int>(*i++, 0);
				int azimuth = lexical_cast_default<int>(*i++, 0);
				int snr = lexical_cast_default<int>(*i++, 0);
				satelliteInfo.push_back(SatelliteInfo(prn, elevation, azimuth, snr));
			}

			if (OnGSV)
				OnGSV(ios, totalMessages, messageNumber, totalSatellitesInView, satelliteInfo);
			return;
		}


		if (*i == "$GPGSA") {
                        //std::cout<<"$GPGSA"<<"\n";
			// GSA = GPS DOP and active satellites
			// $GPGSA,A,3,18,15,21,06,09,,,,,,,,3.7,2.8,2.3*3C
			i++;  // consume the $GPGSA token
			std::string mode(*i++);
			int fix = lexical_cast_default<int>(*i++, 1);  // 1=fix not available
			std::vector<int> satellitesInView;
			while (i<v.begin()+15) {
				int sv = lexical_cast_default<int>(*i++, -1);
				if (sv != -1)
					satellitesInView.push_back(sv);
			}
			double pdop = lexical_cast_default<double>(*i++, 0);
			double hdop = lexical_cast_default<double>(*i++, 0);
			double vdop = lexical_cast_default<double>(*i++, 0);

			if (OnGSA)
				OnGSA(ios, mode, fix, satellitesInView, pdop, hdop, vdop);
			return;
		}

		if (*i == "$GPFPD") {
                        //std::cout<<"$GPFPD"<<"\n";
			// 
			// $GPFPD,GPSWeek,GPSTime,Heading,Pitch,Roll,Lattitude,Longgitude,Altitude,Ve,Vn,Vu,Baseline,NSV1,NSV2,Status*cs
			i=i+2;  // consume the $GPFPD,GPSWeek 

			boost::posix_time::time_duration time(DecodeTime(i));
			
			double heading = lexical_cast_default<double>(*i++, NULL_ANGLE); 
			
			double pitch = lexical_cast_default<double>(*i++, 0); 
			double roll = lexical_cast_default<double>(*i++, 0); 
			double latitude = lexical_cast_default<double>(*i++, 0); 
			double longitude = lexical_cast_default<double>(*i++, 0); 
			double altitude = lexical_cast_default<double>(*i++, 0);
 			double Ve = lexical_cast_default<double>(*i++, 0);
			double Vn = lexical_cast_default<double>(*i++, 0);
			double Vu = lexical_cast_default<double>(*i++, 0);
 
                        double speed=sqrt(Ve*Ve+Vn*Vn+Vu*Vu);

			double baseline = lexical_cast_default<double>(*i++, 0);


			std::vector<int> satellitesInView;
			int nsv1 = lexical_cast_default<int>(*i++, 0);
			int nsv2 = lexical_cast_default<int>(*i++, 0);
			satellitesInView.push_back(nsv1);
			satellitesInView.push_back(nsv2);

			std::vector<double> pose_vect;
			pose_vect.push_back(heading);
			pose_vect.push_back(pitch);
			pose_vect.push_back(roll);

			std::vector<double> geo_vect;
			geo_vect.push_back(latitude);
			geo_vect.push_back(longitude);
			geo_vect.push_back(altitude);

			int status = lexical_cast_default<int>(*i++, 0);

			if (OnFPD)
				OnFPD(ios, time,pose_vect, geo_vect,speed,(nsv1+nsv2)/2,status);
			return;
		}

		if (*i == "$GTIMU") {
                        //std::cout<<"$GTIMU"<<"\n";
			//
			// $GTIMU,GPSWeek,GPSTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Tpr*cs
			i=i+2;  // consume the $GTIMU,GPSWeek,GPSTime

			boost::posix_time::time_duration time(DecodeTime(i));

			double gyroX = lexical_cast_default<double>(*i++, 0); 
			double gyroY = lexical_cast_default<double>(*i++, 0); 
			double gyroZ = lexical_cast_default<double>(*i++, 0); 
			double accX = lexical_cast_default<double>(*i++, 0); 
			double accY = lexical_cast_default<double>(*i++, 0); 
			double accZ = lexical_cast_default<double>(*i++, 0);

			if (OnIMU)
				OnIMU(ios, time, gyroX, gyroY, gyroZ, accX, accY, accZ);
			return;
		}

		// didn't parse anything
		if (OnInvalidSentence) 
			OnInvalidSentence(ios, s);

	} catch (const std::exception &e) {
		std::cout << "Error while decoding >" << s << "< : " << e.what() << std::endl;
		throw;
	}
}
