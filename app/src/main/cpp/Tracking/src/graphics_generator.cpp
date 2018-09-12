//
// Created by zheng hang on 2018/8/14.
//
#include "../include/graphics_generator.h"

namespace generator {
    // Define JNI interface.
    JNIEXPORT void JNICALL
    Java_com_baidu_graph_tracker_Tracker_yuvBytesToRGBBitmap(JNIEnv *env,
                                                             jclass thiz,
                                                             jbyteArray yuv,
                                                             jint width,
                                                             jint height,
                                                             jfloat scale,
                                                             jobject bitmap

    ) {
        jbyte *data = env->GetByteArrayElements(yuv, 0);
        cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (uchar *) data);
        cv::Mat rgbMat(width, height, CV_8UC3);
        cv::cvtColor(yuvMat, rgbMat, CV_YUV420sp2RGB, 3);
        cv::resize(rgbMat, rgbMat, cv::Size(width * scale, height * scale));
        matToBitmap(env, rgbMat, bitmap);
    }

    JNIEXPORT jbyteArray JNICALL
    Java_com_baidu_graph_tracker_Tracker_getTransBitmap(JNIEnv *env,
                                                        jclass thiz,
                                                        jobject bitmap,
                                                        jint width,
                                                        jint height,
                                                        jfloatArray points,
                                                        jintArray params
    ) {
        jfloat *point_ptr = env->GetFloatArrayElements(points, NULL);
        int size = env->GetArrayLength(points);

        // find min value of coordinate
        float xMin = point_ptr[0];
        float xMax = point_ptr[0];
        float yMin = point_ptr[1];
        float yMax = point_ptr[1];
        for (int i = 2; i < 8; i = i + 2) {
            if (point_ptr[i] < xMin) {
                xMin = point_ptr[i];
            }
            if (point_ptr[i] > xMax) {
                xMax = point_ptr[i];
            }
            if (point_ptr[i + 1] < yMin) {
                yMin = point_ptr[i + 1];
            }
            if (point_ptr[i + 1] > yMax) {
                yMax = point_ptr[i + 1];
            }
        }

        // get dst vector
        for (int i = 0; i < 8; i = i + 2) {
            point_ptr[i] -= xMin;
            point_ptr[i + 1] -= yMin;
        }
        yMax -= yMin;
        yMin = 0;
        xMax -= xMin;
        xMin = 0;

        std::vector<cv::Point2f> dstPoints;
        std::vector<cv::Point2f> boundPoints;
        for (int i = 0; i < size; i = i + 2) {
            cv::Point2f point(point_ptr[i], point_ptr[i + 1]);
//            //logd("origin point: %f, %f", point.x, point.y);
            dstPoints.push_back(point);
        }

        // get bound vector
        cv::Point2f leftTop = cv::Point2f(xMin, yMin);
        cv::Point2f rightTop = cv::Point2f(xMax, yMin);
        cv::Point2f rightBottom = cv::Point2f(xMax, yMax);
        cv::Point2f leftBottom = cv::Point2f(xMin, yMax);
        boundPoints.push_back(leftTop);
//        //logd("lefttop point: %f, %f", leftTop.x, leftTop.y);
        boundPoints.push_back(rightTop);
//        //logd("rightTop point: %f, %f", rightTop.x, rightTop.y);
        boundPoints.push_back(rightBottom);
//        //logd("rightBottom point: %f, %f", rightBottom.x, rightBottom.y);
        boundPoints.push_back(leftBottom);
//        //logd("leftBottom point: %f, %f", leftBottom.x, leftBottom.y);

        // find homography matrix
        cv::Mat2f dstMat = cv::Mat(dstPoints, true);
        cv::Mat2f boundMat = cv::Mat(boundPoints, true);
        cv::Mat homographyMat = cv::findHomography(boundMat, dstMat);

        // construct origin bitmap matrix and resize
        cv::Mat originImgMat;
        bitmapToMat(env, bitmap, originImgMat);
        cv::resize(originImgMat, originImgMat, cv::Size2f(xMax, yMax));

        // generate target bitmap matrix
        cv::Mat targetMat;
        cv::warpPerspective(originImgMat, targetMat, homographyMat, cv::Size2f(xMax, yMax));

        jint *params_ptr = env->GetIntArrayElements(params, NULL);
        params_ptr[0] = targetMat.cols;
        params_ptr[1] = targetMat.rows;
        env->ReleaseIntArrayElements(params, params_ptr, 0);

        int resultSize = targetMat.rows * targetMat.cols * targetMat.channels();
        jbyteArray result = env->NewByteArray(resultSize);
        env->SetByteArrayRegion(result, 0, resultSize,
                                reinterpret_cast<const jbyte *>(targetMat.data));
        return result;
    }

    void matToBitmap(JNIEnv *env, cv::Mat &src, jobject bitmap) {
        AndroidBitmapInfo info;
        void *pixels = 0;

        try {
            //logd("nMatToBitmap");
            CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
            CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                      info.format == ANDROID_BITMAP_FORMAT_RGB_565);
            CV_Assert(src.dims == 2 && info.height == (uint32_t) src.rows &&
                      info.width == (uint32_t) src.cols);
            CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3 || src.type() == CV_8UC4);
            CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
            CV_Assert(pixels);
            if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
                cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
                if (src.type() == CV_8UC1) {
                    //logd("nMatToBitmap: CV_8UC1 -> RGBA_8888");
                    cvtColor(src, tmp, CV_GRAY2RGBA);
                } else if (src.type() == CV_8UC3) {
                    //logd("nMatToBitmap: CV_8UC3 -> RGBA_8888");
                    cvtColor(src, tmp, CV_RGB2RGBA);
                } else if (src.type() == CV_8UC4) {
                    //logd("nMatToBitmap: CV_8UC4 -> RGBA_8888");
                    src.copyTo(tmp);
                }
            } else {
                // info.format == ANDROID_BITMAP_FORMAT_RGB_565
                cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
                if (src.type() == CV_8UC1) {
                    //logd("nMatToBitmap: CV_8UC1 -> RGB_565");
                    cvtColor(src, tmp, CV_GRAY2BGR565);
                } else if (src.type() == CV_8UC3) {
                    //logd("nMatToBitmap: CV_8UC3 -> RGB_565");
                    cvtColor(src, tmp, CV_RGB2BGR565);
                } else if (src.type() == CV_8UC4) {
                    //logd("nMatToBitmap: CV_8UC4 -> RGB_565");
                    cvtColor(src, tmp, CV_RGBA2BGR565);
                }
            }
            AndroidBitmap_unlockPixels(env, bitmap);
            return;
        } catch (const cv::Exception &e) {
            AndroidBitmap_unlockPixels(env, bitmap);
            //loge("nMatToBitmap caught cv::Exception: %s", e.what());
            jclass je = env->FindClass("org/opencv/core/CvException");
            if (!je) je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, e.what());
            return;
        } catch (...) {
            AndroidBitmap_unlockPixels(env, bitmap);
            //loge("nMatToBitmap caught unknown exception (...)");
            jclass je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, "Unknown exception in JNI code {nMatToBitmap}");
            return;
        }
    }

    void bitmapToMat (JNIEnv *env, jobject bitmap, cv::Mat &dst) {
        AndroidBitmapInfo info;
        void *pixels = 0;

        try {
            //logd("nBitmapToMat");
            CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
            CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
                      info.format == ANDROID_BITMAP_FORMAT_RGB_565);
            CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
            CV_Assert(pixels);
            dst.create(info.height, info.width, CV_8UC4);
            if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
                //logd("nBitmapToMat: RGBA_8888 -> CV_8UC4");
                cv::Mat tmp(info.height, info.width, CV_8UC4, pixels);
                tmp.copyTo(dst);
            } else {
                // info.format == ANDROID_BITMAP_FORMAT_RGB_565
                //logd("nBitmapToMat: RGB_565 -> CV_8UC4");
                cv::Mat tmp(info.height, info.width, CV_8UC2, pixels);
                cvtColor(tmp, dst, CV_BGR5652RGBA);
            }
            AndroidBitmap_unlockPixels(env, bitmap);
            return;
        } catch (const cv::Exception &e) {
            AndroidBitmap_unlockPixels(env, bitmap);
            //loge("nBitmapToMat caught cv::Exception: %s", e.what());
            jclass je = env->FindClass("org/opencv/core/CvException");
            if (!je) je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, e.what());
            return;
        } catch (...) {
            AndroidBitmap_unlockPixels(env, bitmap);
            //loge("nBitmapToMat caught unknown exception (...)");
            jclass je = env->FindClass("java/lang/Exception");
            env->ThrowNew(je, "Unknown exception in JNI code {nBitmapToMat}");
            return;
        }
    }


}
