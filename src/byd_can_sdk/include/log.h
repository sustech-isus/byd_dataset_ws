//
// Created by kevinlad on 2020/10/29.
//

#ifndef SRC_BYD_CAN_SDK_INCLUDE_LOG_H_
#define SRC_BYD_CAN_SDK_INCLUDE_LOG_H_

#include <iostream>
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include <fstream>

//#define DLOG_INFO std::cout
//#define DLOG_ERROR std::cout

// GLog Wrapper

#define LOG_INFO LOG(INFO)
#define LOG_WARNING LOG(WARNING)
#define LOG_ERROR LOG(ERROR)
#define LOG_FATAL LOG(FATAL)

#define LOG_INFO_IF(condition) LOG_IF(INFO,condition)
#define LOG_WARNING_IF(condition) LOG_IF(WARNING,condition)
#define LOG_ERROR_IF(condition) LOG_IF(ERROR,condition)
#define LOG_FATAL_IF(condition) LOG_IF(FATAL,condition)

#define LOG_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define LOG_WARNING_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define LOG_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define DLOG_INFO DLOG(INFO)
#define DLOG_WARNING DLOG(WARNING)
#define DLOG_ERROR DLOG(WARNING)

#define LOG_WARNING_FIRST_N(times) LOG_FIRST_N(WARNING, times)

class GLogWrapper {
 public:
  GLogWrapper(const std::string & program) {
    google::InitGoogleLogging(program.c_str());
    FLAGS_stderrthreshold = google::WARNING;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_v = 3;
    google::InstallFailureSignalHandler();
  }

  ~GLogWrapper() {
    google::ShutdownGoogleLogging();
  }
};

#endif //SRC_BYD_CAN_SDK_INCLUDE_LOG_H_
