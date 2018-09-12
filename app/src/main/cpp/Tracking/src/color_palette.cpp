//
// Copyright © 2018 Baidu, Inc. All Rights Reserved.
// Created by Ye,Zhibin on 02/01/2018.
// Email: yezhibin@baidu.com
//

#include "../include/color_palette.hpp"
#include "../include/flow_tracker.hpp"

//#include <unistd.h>
//#include <sched.h>

const int CHANNEL = 3;
const int KMEANS_K = 2;
const int BLUR_K = 15;
const float PINNED_ALPHA = 0.5f;
const int STRUCTURING_ELEMENT_SIZE = 12;
const int IN_PAINT_RADIUS = 3;



//#define CPU_SETSIZE 1024
//#define __NCPUBITS  (8 * sizeof (unsigned long))
//typedef struct
//{
//    unsigned long __bits[CPU_SETSIZE / __NCPUBITS];
//} cpu_set_t;
//
//#define CPU_SET(cpu, cpusetp) \
//  ((cpusetp)->__bits[(cpu)/__NCPUBITS] |= (1UL << ((cpu) % __NCPUBITS)))
//#define CPU_ZERO(cpusetp) \
//  memset((cpusetp), 0, sizeof(cpu_set_t))

namespace palette {

//    static inline int yuv_to_rgb(int y, int u, int v);


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
    Graphics extract_image(cv::Mat &src, cv::Rect &roi, int type) {
        struct Graphics extracted_graphics;

        // Cut image in Rect(x, y, width, height).
        cv::Mat fragment = cv::Mat(src, roi).clone();

        // Color reduction.
        cv::Vec3f *vec3f;
        vec3f = color_reduction(fragment);

        // Convert vec to Scalar.
        auto *scalars = new cv::Scalar[KMEANS_K];
        scalars[0] = cv::Scalar(vec3f[0][0], vec3f[0][1], vec3f[0][2]);
        scalars[1] = cv::Scalar(vec3f[1][0], vec3f[1][1], vec3f[1][2]);
        // Add alpha.
        scalars[0][3] = 255 * PINNED_ALPHA;
        scalars[1][3] = 255 * PINNED_ALPHA;

        cv::Mat background;
        if (type == PALETTE_TYPE_PURE) {
            // Generate background in purely.
            background = cv::Mat(roi.height, roi.width, CV_8UC4, scalars[0]);
        } else if (type == PALETTE_TYPE_SOURCE) {
            background = extract_background_image(fragment);
            // Gaussian Blur, but may not working in pure color.
            cv::GaussianBlur(background, background, cv::Size(BLUR_K, BLUR_K), 0);
        }

        extracted_graphics.background = background;
        extracted_graphics.colors = vec3f;

        delete []scalars;
        return extracted_graphics;
    }

    // Extract argb color from yuv img.
//    int *extract_colors(uint8_t *nv21, int src_width, int src_height,
//                        int x,
//                        int y,
//                        int rect_width,
//                        int rect_height,
//                        float compress) {
//        // yuv --> BGR
//        cv::Mat yuv_img;
//        int size = src_width * src_height * 3 / 2;
//        yuv_img.create(src_height * 3 / 2, src_width, CV_8UC1);
//        memcpy(yuv_img.data, nv21, size * sizeof(uint8_t));
//        cv::Mat img;
//        cv::cvtColor(yuv_img, img, CV_YUV2BGR_NV21);
//        cv::Rect roi(x, y, rect_width, rect_height);
//
//        auto *result_colors = new int[2];
//        result_colors[0] = 0;
//        result_colors[1] = 0;
//
//        // ROI valid checking
//        roi.x = roi.x < 0 ? 0 : roi.x > src_width ? src_width : roi.x;
//        roi.y = roi.y < 0 ? 0 : roi.y > src_height ? src_height : roi.y;
//        roi.width = roi.width + roi.x > src_width ? src_width - roi.x : roi.width;
//        roi.height = roi.height + roi.y > src_height ? src_height - roi.y : roi.height;
//
//        LOGD("ROI==>%d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);
//
//        // Cut image in Rect(x, y, width, height).
//        img = cv::Mat(img, roi).clone();
//
//        if (img.empty() || img.rows <= 0 || img.cols <= 0) {
//            LOGD("ExtractColorImageIsNull");
//            return result_colors;
//        }
//
//        cv::Mat fragment = img.clone();
//
//
//
//        // Color reduction.
//        cv::Vec3f *vec3f;
//        vec3f = color_reduction(fragment);
//        if (vec3f != NULL) {
//            result_colors[0] = bgr_to_int(vec3f[0]);
//            result_colors[1] = bgr_to_int(vec3f[1]);
//        } else {
//            LOGD("ExtractColorFragmentImageIsNull");
//        }
//        return result_colors;
//    }

    // Reducing color in Mat, and the color(s) depends on Kmeans-K
    // The max color is K value of Kmeans.
    cv::Vec3f *color_reduction(cv::Mat &src) {
        if (src.empty()) {
            return NULL;
        }
        int src_size = src.rows * src.cols;

        // Mark as single channel MAT.
        cv::Mat data = src.reshape(1, src_size);
        data.convertTo(data, CV_32F);

        // A Clustering for cluster colors.
        std::vector<int> labels;
        cv::Mat1f colors;
        cv::kmeans(data, KMEANS_K, labels,
                   cv::TermCriteria(), 1, cv::KMEANS_RANDOM_CENTERS, colors);

        // It's a trick!
        // The labels is a array including 0 and 1 while k = 2, such as [0,0,1,0,0,0,1]
        // The sum(labels) is the count of ONE.
        int num_one_count = static_cast<int>(cv::sum(labels)[0]);
        int num_zero_count = src_size - num_one_count;

        cv::Vec3f bg_color_BGR;
        cv::Vec3f fg_color_BGR;
        for (int i = 0; i < CHANNEL; ++i) {
            bg_color_BGR[i] = colors.at<float>(num_one_count > num_zero_count ? 1 : 0, i);
            fg_color_BGR[i] = colors.at<float>(num_one_count > num_zero_count ? 0 : 1, i);
        }
        auto *vec3f = new cv::Vec3f[2];
        vec3f[0] = bg_color_BGR;
        vec3f[1] = fg_color_BGR;

        return vec3f;
    }

    /**
      * Extracting background image from source image.
      * @param img The source image.
      * @return The extracted background image.
      */
    cv::Mat extract_background_image(cv::Mat &img) {
        cvtColor(img, img, CV_BGRA2BGR);


        int left = 0;
        int top = 0;
        int width = img.cols;
        int height = img.rows;

        cv::Rect roi(left, top, width, height);
        cv::Mat line = img.clone()(roi);

        cv::Mat line_gray;
        cv::cvtColor(line.clone(), line_gray, CV_RGB2GRAY);
        cv::Mat thresh;
        cv::Mat thresh_inv;

        // Divide
        // TODO Optimize
        cv::threshold(line_gray, thresh, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        cv::threshold(line_gray, thresh_inv, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

        // Counting
        int nonzero_of_thresh = cv::countNonZero(thresh);
        int nonzero_of_thresh_inv = cv::countNonZero(thresh_inv);

        cv::Mat line_mask;
        if (nonzero_of_thresh > nonzero_of_thresh_inv) {
            cv::bitwise_not(thresh, line_mask);
        } else {
            cv::bitwise_not(thresh_inv, line_mask);
        }

        // Structuring element
        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(STRUCTURING_ELEMENT_SIZE,
                                                         STRUCTURING_ELEMENT_SIZE));
        cv::dilate(line_mask, line_mask, element);
        cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
        line_mask.copyTo(mask(roi));

        cv::Mat dst;
        cv::inpaint(img, mask, dst, IN_PAINT_RADIUS, cv::INPAINT_TELEA);

        cvtColor(dst, dst, CV_BGR2BGRA);

        return dst;
    }

    // Convert BGR color to argb int.
    int bgr_to_int(cv::Vec3f &BGR) {
        if (BGR.rows < 3) {
            return 0;
        }
        return 0xFF000000 |
               (((int) BGR[2] & 0xFF) << 16) | // R
               (((int) BGR[1] & 0xFF) << 8) | // G
               (((int) BGR[0] & 0xFF) << 0); // B
    }

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
//                                                       jfloat compress) {
//
//        LOGD("ExtractColor==>%d, %d, %d, %d", x, y, rect_width, rect_height);
//
//        jintArray array_results = env->NewIntArray(2);
//        if (rect_width > 0 && rect_height > 0) {
//            jint *results_ptr = env->GetIntArrayElements(array_results, NULL);
//            uint8_t *yuv_data = (uint8_t *) (env->GetByteArrayElements(yuv, NULL));
//            int *colorResult = extract_colors(yuv_data, width, height, x, y,
//                                              rect_width, rect_height, compress);
//            results_ptr[0] = colorResult[0];
//            results_ptr[1] = colorResult[1];
//            env->SetIntArrayRegion(array_results, 0, 2, results_ptr);
//        }
//        return array_results;
//    }

    // Define JNI interface.
    JNIEXPORT jbyteArray JNICALL
    Java_com_baidu_graph_tracker_Tracker_extractStickerPalette(JNIEnv *env,
                                                               jclass thiz,
                                                               jbyteArray yuv,
                                                               jint width,
                                                               jint height,
                                                               jint x, jint y,
                                                               jint rect_width,
                                                               jint rect_height,
                                                               jintArray params) {
        // pick frame data
        jbyte *data = env->GetByteArrayElements(yuv, 0);

        // 用刚才的data 搞一个 mat
        cv::Mat yuvImg(height + height / 2, width, CV_8UC1, (uchar *) data);
        cv::Mat img;

        //转换成nv21 的color  这个nv21 也就是之前的format
//        CV_YUV2RGB_NV12 = 90,
//        CV_YUV2BGR_NV12 = 91,
//        CV_YUV2RGB_NV21 = 92,
//        CV_YUV2BGR_NV21 = 93,

        // 这个东西 看起来只影响色彩的正常性
        cvtColor(yuvImg, img, CV_YUV2BGR_NV21);

        //搞了一个roi 的矩阵
        cv::Rect roi(x, y, rect_width, rect_height);

        // ROI valid checking
        roi.x = roi.x < 0 ? 0 : roi.x > width ? width : roi.x;
        roi.y = roi.y < 0 ? 0 : roi.y > height ? height : roi.y;
        roi.width = roi.width + roi.x > width ? width - roi.x : roi.width;
        roi.height = roi.height + roi.y > height ? height - roi.y : roi.height;

//        LOGD("ROI==>%d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

        if (roi.width <= 5 || roi.height <= 5) {
            return env->NewByteArray(1);
        }

        int w = roi.width;
        int h = roi.height;

        float scale = (roi.width < 60 || roi.height < 60) ? 0.8f : 0.4f;
        cv::resize(img, img, cv::Size(), scale, scale);

        roi.x *= scale;
        roi.y *= scale;
        roi.width *= scale;
        roi.height *= scale;

        // 测时间的.
        cv::TickMeter tickMeter;
        tickMeter.start();
        Graphics graphics = extract_image(img, roi, PALETTE_TYPE_SOURCE);
        tickMeter.stop();

        cv::resize(graphics.background, graphics.background, cv::Size(), 1 / scale, 1 / scale);
//        LOGD("extract_image==> %f, %d, %d", tickMeter.getTimeMilli(), w, h);


//        cv::Mat *result = (cv::Mat *) addrMat;
//        *result = graphics.background;
        cv::Mat matResult = graphics.background;
        cv::Mat tmp(matResult.rows, matResult.cols, CV_8UC2);
        cvtColor(matResult, tmp, CV_BGRA2BGR565);

        jint *params_ptr = env->GetIntArrayElements(params, NULL);
        params_ptr[0] = matResult.cols;
        params_ptr[1] = matResult.rows;
        params_ptr[2] = bgr_to_int(graphics.colors[0]);
        params_ptr[3] = bgr_to_int(graphics.colors[1]);
        env->ReleaseIntArrayElements(params, params_ptr, 0);

        int size = tmp.rows * tmp.cols * tmp.channels();
        jbyteArray result = env->NewByteArray(size);
        env->SetByteArrayRegion(result, 0, size, reinterpret_cast<const jbyte *>(tmp.data));
        return result;
    }

    // Define JNI interface.
//    JNIEXPORT jintArray JNICALL
//    Java_com_baidu_graph_tracker_Tracker_yuvToRGB(JNIEnv *env,
//                                                  jclass thiz,
//                                                  jbyteArray yuv,
//                                                  jint width,
//                                                  jint height) {
//
//        cv::TickMeter tm;
//        tm.start();
//
//        uint8_t *yuv_data = (uint8_t *) (env->GetByteArrayElements(yuv, NULL));
//        const uint8_t *yData = yuv_data;
//        const uint8_t *vuData = yuv_data + width * height;
//
//        const int yRowStride = width;
//        const int vuRowStride = width;
//
//        int len = width * height;
//        int *rgb = new int[len];
//
//        int y = 0;
//        for (y = 0; y < height; ++y) {
//            const uint8_t *pY = yData + y * yRowStride;
//            const uint8_t *pVU = vuData + (y >> 1) * vuRowStride;
//
//            for (int x = 0; x < width; ++x) {
//                const int offset = ((x >> 1) << 1);
//
//                rgb[y * width + x] = yuv_to_rgb(pY[x], pVU[offset + 1], pVU[offset]);
//            }
//        }
//
//        tm.stop();
//        LOGD("YUV2RGB==>%f", tm.getTimeMilli());
//
//        jintArray result = env->NewIntArray(len);
//        jint *ptr = env->GetIntArrayElements(result, NULL);
//        for (int idx = 0; idx < len; ++idx) {
//            ptr[idx] = rgb[idx];
//        }
//
//
//        env->SetIntArrayRegion(result, 0, len, ptr);
//        return result;
//    }


//    static inline int yuv_to_rgb(int y, int u, int v) {
//        int r = (int) (y + 1.370705 * (v - 128));
//        int g = (int) (y - 0.698001 * (u - 128) - 0.703125 * (v - 128));
//        int b = (int) (y + 1.732446 * (u - 128));
//
//        r = (int) fminf(255, fmaxf(0, r));
//        g = (int) fminf(255, fmaxf(0, g));
//        b = (int) fminf(255, fmaxf(0, b));
//
//        return 0xff000000 | (r << 16) | (g << 8) | b;
//    }
}