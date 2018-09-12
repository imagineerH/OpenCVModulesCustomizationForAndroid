//
//  Copyright © 2017年 baidu. All rights reserved.
//
#include <map>
#include <iostream>
#include <thread>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <mutex>
#include <opencv2/imgcodecs.hpp>

#include "../include/flow_tracker.hpp"
#include "../include/color_palette.hpp"

const int g_local_max_cnt = 60;
const int g_local_min_dist = 10;
const int g_max_cnt = 30;
const int g_min_dist = 30;
const float g_f_threshold = 1.0f;
const float g_scale_max_threshold = 1.03f;
const float g_scale_min_threshold = 0.97f;

static std::mutex s_shared_mutex;

FlowTracker g_tracker;

FlowTracker::FlowTracker() {
}

std::vector<cv::Rect> get_rect(std::vector<cv::Point2f> &ocr_roi) {
    std::vector<cv::Rect> roi_rect;
    for (int i = 0; i < ocr_roi.size(); i += 4) {
        double minx = std::max(
                std::min(ocr_roi[i].x,
                         std::min(ocr_roi[i + 1].x, std::min(ocr_roi[i + 2].x, ocr_roi[i + 3].x))),
                float(0.0));
        double maxx = std::max(ocr_roi[i].x,
                               std::max(ocr_roi[i + 1].x,
                                        std::max(ocr_roi[i + 2].x, ocr_roi[i + 3].x)));
        double miny = std::max(
                std::min(ocr_roi[i].y,
                         std::min(ocr_roi[i + 1].y, std::min(ocr_roi[i + 2].y, ocr_roi[i + 3].y))),
                float(0.0));
        double maxy = std::max(ocr_roi[i].y,
                               std::max(ocr_roi[i + 1].y,
                                        std::max(ocr_roi[i + 2].y, ocr_roi[i + 3].y)));
        roi_rect.push_back(cv::Rect(minx, miny, maxx - minx, maxy - miny));
    }
    return roi_rect;
}

cv::Rect get_ocr_rect(std::vector<cv::Point2f> &points) {
    double min_x = 9999;
    double max_x = -9999;
    double min_y = 9999;
    double max_y = -9999;
    for (int i = 0; i < points.size(); ++i) {
        cv::Point2f point = points[i];
        if (point.x < min_x) {
            min_x = point.x;
        }
        if (point.x > max_x) {
            max_x = point.x;
        }
        if (point.y < min_y) {
            min_y = point.y;
        }
        if (point.y > max_y) {
            max_y = point.y;
        }
    }
    cv::Rect ocr_rect = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
    return ocr_rect;
}

bool in_rect(cv::Point2f p1, std::vector<cv::Rect> &roi_rect) {
    for (int i = 0; i < roi_rect.size(); i++) {
        if (p1.x >= roi_rect[i].tl().x && p1.y >= roi_rect[i].tl().y &&
            p1.x <= roi_rect[i].tl().x + roi_rect[i].width &&
            p1.y <= roi_rect[i].tl().y + roi_rect[i].height) {
            return true;
        }
    }
    return false;
}

void FlowTracker::init_track(int width, int height, std::vector<cv::Point2f> points) {
    if (points.size() == 0) {
        //LOGD("points.size() == 0");

        return;
    }

    std::lock_guard<std::mutex> lock(s_shared_mutex);

    if (!has_start) {
        //LOGD("has_start false return");

        return;
    }
    this->mask = cv::Mat(height, width, CV_8UC1);
    this->width = width;
    this->height = height;
    //   std::lock_guard<std::mutex> lock(s_shared_mutex);
//    if (has_began_get_offset && this->track_points.size()) {
//        points.insert(points.begin(), this->track_points.front());
//    }
    //   this->track_points = points;

    cv::Rect ocr_rect = get_ocr_rect(points);

    //LOGD("ocr_rect.size weight =  %d", ocr_rect.size().width);

    double area = ocr_rect.width * ocr_rect.height;
    double img_area = width * height;
    if (area / img_area < 0.2) {
        this->in_local = true;
        this->ocr_rect = get_rect(points);
        this->init_sign = true;
    } else {
        this->in_local = false;
    }

    //    cv::Mat img_equa;
    //    //使图像变的清晰  -- 自适应直方图均衡
    //    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    //    clahe->setClipLimit(3);
    //    clahe->apply(img, img_equa);
    //    cv::resize(img_equa,img_equa,cv::Size(width, height),0,0);
    //    forw_img = img_equa;
    //    get_good_points();
    if (has_began_get_offset && this->track_points.size()) {
        points.insert(points.begin(), this->track_points.front());
    }

    this->track_points = points;
    total_offset_object = cv::Point2f(0,0);
    acc_scale = 1.0;


    //LOGD("init  track_points size=  %d", track_points.size());

}

TrackResult FlowTracker::update(cv::Mat img) {
    std::lock_guard<std::mutex> lock(s_shared_mutex);

    // std::cout<<"feature points image size is : "<<img.size()<<" "<<img.type()<<std::endl;

    //LOGD("刚进update: cur_pts: %d", cur_pts.size());

    TrackResult result;
    if (!has_start) {
        //LOGD("未开始");
        return result;
    }
    if (!has_began_get_offset) {
        result.type = TrackType_onlyTrack;
    } else if (has_began_get_offset && track_points.size() == 1) {
        result.type = TrackType_onlyGetStatus;
    } else if (has_began_get_offset && track_points.size() > 1) {
        result.type = TrackType_both;
    } else if (has_began_get_offset && track_points.empty()) {
        result.type = TrackType_unknown;
    }

    if (track_points.empty()) {
        //LOGD("track_points.empty()");
        return result;
    }
//
//    cv::Mat resize_img;
//
//    cv::resize(img,resize_img,cv::Size(width, height),0,0);
//
//    //灰度处理
//    //    TS(update_to_gray);
//    cv::Mat gray;
//    cv::cvtColor(resize_img, gray, CV_RGBA2GRAY);
//    //    TE(update_to_gray);
//
//    //    TS(img_equa);
//    cv::Mat img_equa;
//    //使图像变的清晰  -- 自适应直方图均衡
//    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//    clahe->setClipLimit(3);
//    clahe->apply(gray, img_equa);
    //    TE(img_equa);

    if (forw_img.empty()) {
        cur_img = forw_img = img;
    } else {
        forw_img = img;
    }

    forw_pts.clear();

    //track

//    std::cout<<"feature points number is : "<<cur_pts.size()<<std::endl;

    //LOGD("cur_pts.size()=  %d", cur_pts.size());

    if (cur_pts.size() > 4) {
        std::vector<uchar> status;
        std::vector<float> err;
        //        TS(optical_flow_track);
        //追踪特征点
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err,
                                 cv::Size(21, 21), 3);
        //        TE(optical_flow_track);

        //LOGD("追踪后forw_pts.size()=  %d", forw_pts.size());

        for (int i = 0; i < int(forw_pts.size()); i++) {
            //LOGD("之前status[i] = %d", status[i]);

            if ((status[i] == 1) && !in_border(forw_pts[i])) {
                //                cout << " 越界: " << status[i] << endl;
                status[i] = 0;
            }
        }
        //LOGD("追踪后 status.size() == %d", status.size());

        int j = 0;
        for (int i = 0; i < status.size(); ++i) {
            //LOGD("之后status[i] = %d", status[i]);
            if (status[i]) {
                cur_pts[j] = cur_pts[i];
                forw_pts[j] = forw_pts[i];
                track_cnt[j] = track_cnt[i];
                j++;
            }
        }
        cur_pts.resize(j);
        forw_pts.resize(j);
        track_cnt.resize(j);
        //LOGD("追踪后:cur_pts = %d   forw_pts = %d   track_cnt = %d  ", cur_pts.size(), forw_pts.size(),track_cnt.size());


        //reject outliers
        if (forw_pts.size() >= 8) {
            std::vector<uchar> status;
            //            TS(delete_wrong);
            //去除错误跟踪
            cv::findFundamentalMat(cur_pts, forw_pts, cv::FM_RANSAC, g_f_threshold, 0.99, status);
            //            TE(delete_wrong);
            int z = 0;
            for (int i = 0; i < status.size(); ++i) {
                if (status[i]) {
                    cur_pts[z] = cur_pts[i];
                    forw_pts[z] = forw_pts[i];
                    track_cnt[z] = track_cnt[i];
                    z++;
                }
            }
            cur_pts.resize(z);
            forw_pts.resize(z);
            track_cnt.resize(z);
        }

        //        cout << "祛除完错误点: " << "count of forw_pts: " << forw_pts.size() << endl;

        for (auto &n : track_cnt) {
            n++;
        }

        if (!cur_pts.empty() && !forw_pts.empty()) {
            cv::Point2f first_cur_pts = cur_pts[0];
            cv::Point2f first_forw_pts = forw_pts[0];

            float cur_distance_sum = 0.0;
            float forw_distance_sum = 0.0;

            cv::Point2f point_offset(0,0);

            for (int i = 0; i < cur_pts.size(); ++i) {
                cv::Point2f cur_point = cur_pts[i];
                cv::Point2f forw_point = forw_pts[i];

                point_offset.x += (forw_point.x - cur_point.x);
                point_offset.y += (forw_point.y - cur_point.y);

                float cur_distance = sqrt(
                        (cur_point.x - first_cur_pts.x) * (cur_point.x - first_cur_pts.x) +
                        (cur_point.y - first_cur_pts.y) * (cur_point.y - first_cur_pts.y));
                float forw_distance = sqrt(
                        (forw_point.x - first_forw_pts.x) * (forw_point.x - first_forw_pts.x) +
                        (forw_point.y - first_forw_pts.y) * (forw_point.y - first_forw_pts.y));

                cur_distance_sum += cur_distance;
                forw_distance_sum += forw_distance;
            }
            total_offset_object.x += (point_offset.x/cur_pts.size());
            total_offset_object.y += (point_offset.y/cur_pts.size());

            float scale = forw_distance_sum/cur_distance_sum;

            acc_scale *= scale;

            if (has_began_get_offset) {
                base_scale *= scale;
            }
        }

        int size_of_cur_pts = cur_pts.size();
        //LOGD("track_points.empty() = %d    size_of_cur_pts  = %d", track_points.empty(),size_of_cur_pts);

//      cout << track_points.empty() << "size of cur pts" << size_of_cur_pts << endl;

//      std::cout<<"feature points number is after delete error pts : "<<cur_pts.size()<<std::endl;

        if (!track_points.empty() && cur_pts.size() >= 4) {
            //            TS(get_new_location);
            cv::Mat perspective_transform = cv::findHomography(cur_pts, forw_pts, cv::RANSAC, 1.0);
            if (!perspective_transform.empty()) {
                cv::perspectiveTransform(track_points, track_result_points, perspective_transform);
                track_points = track_result_points;
            } else {
                //LOGD("运算完透视变换矩阵:得到矩阵为空   track_points.clear():");
                in_local = false;
                base_scale = 1.0;
                acc_scale = 1.0;
                track_points.clear();
                //                cur_pts.clear();
                //                forw_pts.clear();
                has_began_get_offset = false;
            }
            //            TE(get_new_location);
        } else {
            in_local = false;
            base_scale = 1.0;
            acc_scale = 1.0;

            //LOGD("shshshsshs =clear");

            track_points.clear();
            //            cur_pts.clear();
            //            forw_pts.clear();
            has_began_get_offset = false;
        }
    }

//    TS(to_get_good_point);
    get_good_points();
//    TE(to_get_good_point);

    cur_img = forw_img;
    cur_pts = forw_pts;
    //    TS(map_points);
    //    track_point_map_points();
    //    TE(map_points);

    if (result.type == TrackType_onlyTrack) {
        result.track_points = track_result_points;
        //        if (in_local && init_sign) {
        //            this->ocr_rect = get_rect(track_result_points);
        //
        //        }
    } else if (result.type == TrackType_both) {
        TrackStatus status;
        status.scale = base_scale;
        if (base_scale > g_scale_max_threshold || base_scale < g_scale_min_threshold) {
            status.isScale = true;
        } else {
            status.isScale = false;
        }

        if (!track_result_points.empty()) {
            auto first = track_result_points.erase(track_result_points.begin());
            status.offset = cv::Point2f(first->x - width / 2, first->y - height / 2);
            result.track_points = track_result_points;
            //            if (in_local && init_sign) {
            //                this->ocr_rect = get_rect(track_result_points);
            //            }
        } else {
            get_status_point_loss = true;
            in_local = false;
        }
    } else if (result.type == TrackType_onlyGetStatus) {
        TrackStatus status;
        status.scale = base_scale;
        if (base_scale > g_scale_max_threshold || base_scale < g_scale_min_threshold) {
            status.isScale = true;
        } else {
            status.isScale = false;
        }

        if (!track_result_points.empty()) {
            auto first = track_result_points.erase(track_result_points.begin());
            status.offset = cv::Point2f(first->x - width / 2, first->y - height / 2);
        } else {
            get_status_point_loss = true;
            in_local = false;
        }
    }

    return result;
}

void FlowTracker::get_good_points() {
    int n_max_cnt = 0;
    bool is_only_get_status = has_began_get_offset && track_points.size() == 1;

    if (in_local && !is_only_get_status) {
        n_max_cnt = g_local_max_cnt - static_cast<int>(forw_pts.size());
    } else {
        n_max_cnt = g_max_cnt - static_cast<int>(forw_pts.size());
    }

    static int get_feature = 5;

    // std::cout<<"forw_pts.size() is : "<<forw_pts.size()<<std::endl;

    if (get_feature == 5 || n_max_cnt == 60 || n_max_cnt == 30) {
        get_feature = 0;
        set_mask();
        std::vector<cv::Point2f> n_pts;
        if (n_max_cnt > 0) {
            //goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
            //n_pts  获取到新的角点
            if (in_local) {
                cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, g_local_min_dist, mask);
            } else {
                cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, g_min_dist, mask);
            }

        }

        //draw 将新获取到的角点添加到 good_pts 中
        for (auto &p : n_pts) {
            if (in_local && !is_only_get_status) {
                if (this->init_sign == true && in_rect(p, this->ocr_rect)) {
                    forw_pts.push_back(p);
                    track_cnt.push_back(1);
                } else if (this->init_sign == false) {
                    forw_pts.push_back(p);
                    track_cnt.push_back(1);
                }
            } else {
                forw_pts.push_back(p);
                track_cnt.push_back(1);
            }
        }
    }
    get_feature++;
}

void FlowTracker::begin_get_status() {
    std::lock_guard<std::mutex> lock(s_shared_mutex);

    if (!has_start) {
        //LOGD("尚未开始");
        return;
    }

    if (!has_began_get_offset) {
        base_scale = 1.0;
        cv::Point2f center_point = cv::Point2f(width / 2, height / 2);
        //LOGD("向 track points 中添加中心点");

        track_points.insert(track_points.begin(), center_point);
        has_began_get_offset = true;
        get_status_point_loss = false;
    } else {
        //LOGD(" 上一次 get_offset 还未结束");
    }
}

TrackStatus FlowTracker::end_get_status() {
    std::lock_guard<std::mutex> lock(s_shared_mutex);
    TrackStatus track_status;
    if (!has_start) {
        return track_status;
    }

    if (has_began_get_offset) {
        //LOGD("end get status");
        if (base_scale > g_scale_max_threshold || base_scale < g_scale_min_threshold) {
            //            cout << "base scale: " << base_scale << endl;
            track_status.isScale = true;
        } else {
            //            cout << "base scale: " << base_scale << endl;
            track_status.isScale = false;
        }
        has_began_get_offset = false;
        cv::Point2f end_point = track_points.front();
        track_points.erase(track_points.begin());
        track_status.offset = cv::Point2f(end_point.x - width / 2, end_point.y - height / 2);
        track_status.get_status_point_loss = get_status_point_loss;
        track_status.scale = base_scale;
        return track_status;
    } else {
        //LOGD("已经停止过了");
    }
    return track_status;
}

void FlowTracker::end_track() {

    //LOGD("end_track!");

    std::lock_guard<std::mutex> lock(s_shared_mutex);
    if (has_began_get_offset) {
        track_points.erase(track_points.begin() + 1, track_points.end());
    } else {
        base_scale = 1.0;
        acc_scale = 1.0;
        track_points.clear();
        has_began_get_offset = false;
    }
}

//排序 是这个几个vector 都按照 track_cnt 值进行排序
void FlowTracker::set_mask() {
    mask.setTo(255);
    std::vector<std::pair<int, cv::Point2f>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts.size(); i++) {
        cnt_pts_id.push_back(std::make_pair(track_cnt[i], forw_pts[i]));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),
         [](const std::pair<int, cv::Point2f> &a, const std::pair<int, cv::Point2f> &b) {
             return a.first > b.first;
         });

    forw_pts.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id) {
        if (mask.at<uchar>(it.second) == 255) {
            forw_pts.push_back(it.second);
            track_cnt.push_back(it.first);
            if (in_local) {
                cv::circle(mask, it.second, g_local_min_dist, 0, -1);
            } else {
                cv::circle(mask, it.second, g_min_dist, 0, -1);
            }
        }
    }
}

bool FlowTracker::in_border(const cv::Point2f &pt) {
    const int board_size = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return board_size <= img_x && img_x < width - board_size && board_size <= img_y &&
           img_y < height - board_size;
}

//void FlowTracker::track_point_map_points() {
//    if (!track_points.size()) {
//        points_map_points.clear();
//        return;
//    }
//
//    std::vector<std::pair<cv::Point2f, std::vector<std::pair<int, cv::Point2f>>>> in_points_map_points;
//    points_map_points.clear();
//    std::vector<std::pair<int, cv::Point2f>> points_pairs;
//
//    for (int i = 0; i < cur_pts.size(); ++i) {
//        points_pairs.push_back(std::make_pair(i, cur_pts[i]));
//    }
//
//    for (int i = 0; i < track_points.size(); ++i) {
//        auto track_point = track_points[i];
//        auto pair = make_pair(track_point, points_pairs);
//        sort(pair.second.begin(), pair.second.end(),
//             [track_point](std::pair<int, cv::Point2f> &a, std::pair<int, cv::Point2f> &b) {
//                 float a_x_diff = a.second.x - track_point.x;
//                 float a_y_diff = a.second.y - track_point.y;
//                 float a_distance = sqrtf(a_x_diff * a_x_diff + a_y_diff * a_y_diff);
//
//                 float b_x_diff = b.second.x - track_point.x;
//                 float b_y_diff = b.second.y - track_point.y;
//                 float b_distance = sqrtf(b_x_diff * b_x_diff + b_y_diff * b_y_diff);
//
//                 return a_distance < b_distance;
//             });
//        in_points_map_points.push_back(pair);
//    }
//    points_map_points = in_points_map_points;
//}

void FlowTracker::start() {
    has_start = true;
}

void FlowTracker::stop() {
    has_start = false;
}

jint native_init_track(JNIEnv *env, jobject obj, jint width, jint height, jfloatArray pointArray,
                       jfloat compress) {

    std::vector<cv::Point2f> points;
    jfloat *point_ptr = env->GetFloatArrayElements(pointArray, NULL);

    int size = env->GetArrayLength(pointArray);
    //LOGD("init Size==> %d", size);

    for (int i = 0; i < size; i = i + 2) {
        cv::Point2f point(point_ptr[i] * compress, point_ptr[i + 1] * compress);
        //LOGD("%f, %f", point.x, point.y);
        points.push_back(point);
    }

    g_tracker.init_track((int) (width * compress), (int) (height * compress), points);
    return 1;
}

void native_stop_track(JNIEnv *env, jobject obj) {
    g_tracker.stop();

    g_tracker.end_track();
}

jfloatArray
native_update(JNIEnv *env, jobject obj, jbyteArray yuv, jint width, jint height, jfloat compress, jfloatArray offsetAndScale) {
    //LOGD("Update==>%d, %d", width, height);
    cv::TickMeter tm;
    tm.start();
    // YUV --> BGR
    uint8_t *yuv_data = (uint8_t *) (env->GetByteArrayElements(yuv, NULL));
    cv::Mat yuv_img;
    int len = width * height * 3 / 2;
    yuv_img.create(height * 3 / 2, width, CV_8UC1);
    memcpy(yuv_img.data, yuv_data, len * sizeof(uint8_t));

    cv::Mat img;
    cv::cvtColor(yuv_img, img, CV_YUV2GRAY_420);
    if (compress < 1) {
        cv::resize(img, img, cv::Size(), compress, compress);
        //LOGD("ResizeUpdate==>%d, %d", img.rows, img.cols);
    }

    tm.stop();
    //LOGD("JNIConvertSpend==>%f", tm.getTimeMilli());
    //LOGD("UpdateYUVSize==>%d", img.rows * img.cols);
    // Fetch the latest points tracked.

    const TrackResult &result = g_tracker.update(img);

    std::vector<cv::Point2f> points_updated = result.track_points;

    if (points_updated.empty()) {
        //LOGe("track no result");
    }

    // Convert points to float array.
    int point_float_size = points_updated.size() * 2;
    jfloatArray pointArray = env->NewFloatArray(point_float_size);
    jfloat *points_ptr = env->GetFloatArrayElements(pointArray, NULL);
    for (int i = 0; i < points_updated.size(); ++i) {
        points_ptr[i * 2] = points_updated[i].x / compress;
        points_ptr[i * 2 + 1] = points_updated[i].y / compress;
    }

    env->SetFloatArrayRegion(pointArray, 0, point_float_size, points_ptr);
    // return accumulated offset and scale
    jfloat *params_ptr = env->GetFloatArrayElements(offsetAndScale, NULL);
    params_ptr[0] = g_tracker.getResultTotalOffset().x;
    params_ptr[1] = g_tracker.getResultTotalOffset().y;
    params_ptr[2] = g_tracker.getResultTotalScale();
    env->ReleaseFloatArrayElements(offsetAndScale, params_ptr, 0);
    return pointArray;
}

void native_start_record_offset(JNIEnv *env, jobject obj) {
    g_tracker.start();
    g_tracker.begin_get_status();
}

jfloatArray native_end_and_get_tracked_offset(JNIEnv *env, jobject obj) {
    cv::TickMeter tm;
    tm.start();
    TrackStatus status = g_tracker.end_get_status();

    tm.stop();
    //LOGD("end_get_status==>%f", tm.getTimeMilli());

    jfloatArray result = env->NewFloatArray(3);
    jfloat *result_ptr = env->GetFloatArrayElements(result, NULL);
    result_ptr[0] = status.offset.x;
    result_ptr[1] = status.offset.y;
    result_ptr[2] = status.isScale ? 1 : 0;
    env->SetFloatArrayRegion(result, 0, 3, result_ptr);
    return result;
}


void native_start() {
    g_tracker.start();
}

void native_stop() {
    g_tracker.stop();
}

cv::Point2f FlowTracker::getResultTotalOffset()
{
    //get total offset
    return total_offset_object;
}

float FlowTracker::getResultTotalScale()
{
    return acc_scale;
}

static const char *jniClassName = "com/baidu/graph/tracker/Tracker";

static JNINativeMethod methods[] = {
        {"jniStart",             "()V",       (void *) native_start},
        {"jniStop",              "()V",       (void *) native_stop},
        {"initTrack",            "(II[FF)I",  (void *) native_init_track},
        {"stopTrack",            "()V",       (void *) native_stop_track},
        {"update",               "([BIIF[F)[F", (void *) native_update},
        {"startRecordOffset",    "()V",       (void *) native_start_record_offset},
        {"endAndGetTrackOffset", "()[F",      (void *) native_end_and_get_tracked_offset}
};


static int registerNatives(JNIEnv *env) {
    jclass clazz = env->FindClass(jniClassName);
    if (clazz == NULL)
        return JNI_FALSE;

    jint methodSize = sizeof(methods) / sizeof(methods[0]);
    if (env->RegisterNatives(clazz, methods, methodSize) < 0)
        return JNI_FALSE;

    return JNI_TRUE;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    JNIEnv *env = NULL;
    jint result = -1;

    if (vm->GetEnv((void **) &env, JNI_VERSION_1_6) != JNI_OK)
        return JNI_ERR;

    if (!registerNatives(env))
        return JNI_ERR;

    result = JNI_VERSION_1_6;
    return result;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM *vm, void *reserved) {
    JNIEnv *env = nullptr;
    jint ret = vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);
    if (ret != JNI_OK) {
        return;
    }
}





