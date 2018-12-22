//
// Created by zheng hang on 2018/8/14.
//

#ifndef GRAPH_GRAPHICS_GENERATOR_H
#define GRAPH_GRAPHICS_GENERATOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <jni.h>
//#include <android/log.h>
#include <android/bitmap.h>
//#include <bits/stdc++.h>

// 日志
//#ifdef __loggable
//
//const string TAG  = "generator"
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

namespace generator {

    static int _max_img_size = 2500 * 1080 * 5;

    // Define JNI interface.
    JNIEXPORT jboolean JNICALL
    Java_com_baidu_graph_tracker_Tracker_yuvBytesToRGBBitmap(JNIEnv *env,
                                                             jclass thiz,
                                                             jbyteArray yuv,
                                                             jint width,
                                                             jint height,
                                                             jfloat scale,
                                                             jobject bitmap

    );

    JNIEXPORT jbyteArray JNICALL
    Java_com_baidu_graph_tracker_Tracker_getTransBitmap(JNIEnv *env,
                                                        jclass thiz,
                                                        jobject bitmap,
                                                        jint width,
                                                        jint height,
                                                        jfloatArray points,
                                                        jintArray params
    );

    void matToBitmap(JNIEnv * env, cv::Mat &src, jobject bitmap);

    bool bitmapToMat(JNIEnv *env, jobject bitmap, cv::Mat &dst);

}

#endif //GRAPH_GRAPHICS_GENERATOR_H

#ifdef __cplusplus
}
#endif
