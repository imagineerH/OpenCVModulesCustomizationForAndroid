//
// Copyright © 2018 Baidu, Inc. All Rights Reserved.
// Created by Ye,Zhibin on 02/01/2018.
// Email: yezhibin@baidu.com
//
#ifndef color_palette_h
#define color_palette_h

//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
//#include <iostream>

#include <jni.h>
//#include <android/bitmap.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    // generate pure color background.
            PALETTE_TYPE_PURE = 0,
    // extract background from source.
            PALETTE_TYPE_SOURCE = 1,
};

namespace palette {

    /**
     * The graphics extracted, including some image properties.con
     * Note: it could not be pinned in source image.
     */
    struct Graphics {
        // The background of pinned graphics.
        cv::Mat background;

        // The colors of pinned graphics
        // The index-0 ==> background scalar
        // The index-1 ==> foreground scalar.
        cv::Vec3f *colors;
    };

    /**
     * Extracting colors for source image in target rectangle.
     *
     * @param src The BGR Mat image.
     * @param roi The ROI of fragment.
     * @param type The type of background,
     * #TYPE_PURE, generate pure color background.
     * #TYPE_SOURCE, extract background from source.
     * @return The result of extracted colors, see {#Graphics};
     */
    Graphics extract_image(cv::Mat &src, cv::Rect &roi, int type);

    /**
     * Extracting colors for source image in target rectangle.
     *
     * @param nv21 The source image in YUV format.
     * @param width The source image width.
     * @param height The source image height,
     * @param x The left of ROI
     * @param y The top of ROI
     * @param rect_width The width of ROI
     * @param rect_height The height of ROI
     * @return The result of extracted colors in ARGB values.
     */
//    int *extract_colors(uint8_t *nv21, int src_width, int src_height,
//                        int x,
//                        int y,
//                        int rect_width,
//                        int rect_height);

    /**
     *  Reducing color in Mat, and the color(s) depends on Kmeans-K
     *  The max color is K value of Kmeans.
     * @param src The target source image.
     * @return The result of colors reduction, the size depends on K value.
     */
    cv::Vec3f *color_reduction(cv::Mat &src);

    /**
     * Extracting background image from source image.
     * @param img The source image.
     * @return The extracted background image.
     */
    cv::Mat extract_background_image(cv::Mat &img);

    // Convert BGR color to ARGB integer.
    int bgr_to_int(cv::Vec3f &BGR);

    /**
     * 根据图片背景色的亮度判断是深色背景还是浅色背景，决定文字颜色
     * @param vec 背景色
     * @return 深色背景：白色，浅色背景：黑色
     */
    int decide_foreground_color_by_background_brightness(cv::Vec3f vec);


//    int encode_mat_to_jpg_bytes(cv::Mat img, unsigned char *&data, int &len);


    // Define JNI interface.
//    JNIEXPORT jintArray JNICALL
//    Java_com_baidu_graph_tracker_Tracker_extractColors(JNIEnv *env,
//                                                       jclass thiz,
//                                                       jbyteArray yuv,
//                                                       jint width,
//                                                       jint height,
//                                                       jint x, jint y,
//                                                       jint rect_width,
//                                                       jint rect_height,
//                                                       float compress);


    // Define JNI interface.
//    JNIEXPORT jintArray JNICALL
//    Java_com_baidu_graph_tracker_Tracker_yuvToRGB(JNIEnv *env,
//                                                  jclass thiz,
//                                                  jbyteArray yuv,
//                                                  jint width,
//                                                  jint height);

    JNIEXPORT jbyteArray JNICALL
    Java_com_baidu_graph_tracker_Tracker_extractStickerPalette(JNIEnv *env,
                                                               jclass thiz,
                                                               jbyteArray yuv,
                                                               jint width,
                                                               jint height,
                                                               jint x, jint y,
                                                               jint rect_width,
                                                               jint rect_height,
                                                               jintArray params);
}

#endif //COLOR_PALETTE_H

#ifdef __cplusplus
}
#endif
