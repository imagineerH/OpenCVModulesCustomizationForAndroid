//
//  Copyright © 2017年 baidu. All rights reserved.
//

#ifndef flow_tracker_hpp
#define flow_tracker_hpp

//#include "cpp_helper.hpp"

//#include <map>
#include <vector>
//#include <cstdio>
#include <opencv2/core.hpp>
//#include <android/log.h>
#include <jni.h>
// 关闭日志
//#define  __loggable
//#ifdef __loggable
//
//#define TAG    "antoTrans"
//
//#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
//#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
//#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
//
//#else
//
//#define LOGD(...)
//#define LOGE(...)
//#define LOGW(...)
//
//#endif


#ifdef __cplusplus
extern "C" {
#endif

typedef enum : int {
    TrackType_unknown,
    TrackType_onlyTrack,
    TrackType_onlyGetStatus,
    TrackType_both,
} TrackType;

struct TrackStatus {
    bool get_status_point_loss = true;
    cv::Point2f offset = cv::Point2f(0.0, 0.0);
    bool isScale = false;
    float scale = 1.0;
};

struct TrackResult {
    TrackType type;
    TrackStatus status;
    std::vector<cv::Point2f> track_points;
};

// Define JNI interface start.
jint native_init_track(JNIEnv *env, jobject obj, jint width, jint height, jfloatArray pointArray,
                       jfloat compress);

void native_stop_track(JNIEnv *env, jobject obj);

jfloatArray
native_update(JNIEnv *env, jobject obj, jbyteArray yuv, jint width, jint height, jfloat compress, jfloatArray offsetAndScale);

void native_start_record_offset(JNIEnv *env, jobject obj);

jfloatArray native_end_and_get_tracked_offset(JNIEnv *env, jobject obj);

void native_start();

void native_stop();
// Define JNI interface  end.

class FlowTracker {
public:

    /**
     * @b instance method of FlowTracker
     *
     * @param width the width of the image you will track
     *
     * @param height the height of the image you will track
     *
     */
    FlowTracker();

    /**
     * @b init a track with the points you want to track
     *
     * @param points the points you want to track
     */
    void init_track(int width, int height, std::vector<cv::Point2f> points);

    /**
     * @b update the new location in new image of points your are tracking
     *
     * @param mat new image which you want to get new points from, need gray mat
     *
     * @return new points in new image of points being tracked
     */
    TrackResult update(cv::Mat mat);

    /**
     * @b end track, you should call this method when whant to end track
     *
     */
    void end_track();

    /**
     * @b the below two method is used to record status changes of a period of time
     *
     */
    void begin_get_status();

    TrackStatus end_get_status();

    void start();

    void stop();

    cv::Point2f getResultTotalOffset();
    float getResultTotalScale();

private:
    int height;
    int width;
    bool has_start = false;
    bool init_sign = false;
    bool in_local = false;
    cv::Point2f offset;
    cv::Mat mask;
    cv::Mat cur_img;
    cv::Mat forw_img;
    std::vector<cv::Point2f> track_points;
    std::vector<cv::Point2f> track_result_points;
    std::vector<cv::Rect> ocr_rect;
    std::vector<int> track_cnt;
    std::vector<cv::Point2f> cur_pts;
    std::vector<cv::Point2f> forw_pts;
    std::vector<std::pair<cv::Point2f, std::vector<std::pair<int, cv::Point2f>>>> points_map_points;

    bool has_began_get_offset = false;
    bool get_status_point_loss = false;
    float base_scale = 1.0;

    cv::Point2f total_offset_object;
    float acc_scale = 1.0;

    void get_good_points();

//    void track_point_map_points();

    bool in_border(const cv::Point2f &pt);

    void set_mask();
};

#endif

#ifdef __cplusplus
}
#endif



