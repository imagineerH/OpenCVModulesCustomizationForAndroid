////
////  cpp_hepler.hpp
////  TestFlowTrack
////
////  Created by liuRuiLong on 2018/1/4.
////  Copyright © 2018年 baidu. All rights reserved.
////
//
//#ifndef cpp_helper_hpp
//#define cpp_helper_hpp
//
//#include <string>
//#include <sstream>
//
//#define TS(name) int64 t_##name = cv::getTickCount()
//#define TE(name) printf("TIMER_" #name ": %.2fms\n", \
//1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
//
//#define ATS(name) int64 t_##name = cv::getTickCount()
//#define ATE(name) __android_log_print(ANDROID_LOG_DEBUG, "frameTime", "TIMER_" #name ": %.2fms\n", \
//1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
//
//const std::string exception_prefix = "Custom C++ Exception: ";
//
///**
// * custom C++ exception
// */
//struct Exception : public std::exception {
//    std::string message;
//
//    Exception(const char *detail, const char *file, const int line) {
//        std::stringstream ss;
//        ss << exception_prefix << "-Custom Exception- ";
//        ss << "| [in file] -:- [" << file << "] ";
//        ss << "| [on line] -:- [" << line << "] ";
//        ss << "| [detail] -:- [" << detail << "].";
//        message = ss.str();
//    }
//
//    virtual const char *what() const throw() {
//        return message.c_str();
//    }
//};
//
//#define throw_exception(...) {\
//char buffer[1000]; \
//sprintf(buffer, __VA_ARGS__); \
//std::string detail{buffer}; \
//throw Exception(detail.c_str(), __FILE__, __LINE__);\
//}
//
//#endif
