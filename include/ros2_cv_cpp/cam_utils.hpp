#ifndef CAM_UTILS_HPP
#define CAM_UTILS_HPP

#include <opencv2/opencv.hpp>

#include "ros2_cv_cpp/str_utils.hpp"

constexpr size_t FOURCC_ARRAY_SIZE  = 5;
constexpr int    DEFAULT_CAM_ID     = -1;
constexpr double DEFAULT_WIDTH      = 640;
constexpr double DEFAULT_HEIGHT     = 480;
constexpr double DEFAULT_BRIGHTNESS = 0;
constexpr double DEFAULT_CONTRAST   = 32;
constexpr double DEFAULT_SATURATION = 64;
constexpr double DEFAULT_FPS        = 30;
constexpr std::array<char, FOURCC_ARRAY_SIZE> DEFAULT_FOURCC {'M', 'J', 'P', 'G'};

class cv_camera {
public:
    cv_camera();
    ~cv_camera();
    //setters
    void set_cam(int c_id = DEFAULT_CAM_ID);
    void set_fourcc(const std::string& fourcc_str = arr2str(DEFAULT_FOURCC));
    void set_fps(double fps = DEFAULT_FPS);
    void set_width(double w = DEFAULT_WIDTH);
    void set_height(double h = DEFAULT_HEIGHT);
    void set_brightness(double br = DEFAULT_BRIGHTNESS);
    void set_contrast(double ctr = DEFAULT_CONTRAST);
    void set_saturation(double sat = DEFAULT_SATURATION);

    // getters
    int get_cam_id() const;
    std::string get_fourcc() const;
    double get_fps() const;
    double get_width() const;
    double get_height() const;
    double get_brightness() const;
    double get_contrast() const;
    double get_saturation() const;

    //access opencv functions
    bool isopen() const;
    bool read(cv::Mat& frame);

private:
    int cam_id_;
    std::array<char, FOURCC_ARRAY_SIZE> fourcc_arr_;
    double fps_, width_, height_, brightness_, contrast_, saturation_;
    cv::VideoCapture cam_;

    // additional functions
    int encode_fourcc(const std::array<char, FOURCC_ARRAY_SIZE>& fourcc_arr);
};

#endif