#include <stdexcept>

#include "ros2_cv_cpp/cam_utils.hpp"

// constructor
cv_camera::cv_camera() : 
    cam_id_(DEFAULT_CAM_ID),
    fourcc_arr_(DEFAULT_FOURCC),
    fps_(DEFAULT_FPS),
    width_(DEFAULT_WIDTH),
    height_(DEFAULT_HEIGHT),
    brightness_(DEFAULT_BRIGHTNESS),
    contrast_(DEFAULT_CONTRAST),
    saturation_(DEFAULT_SATURATION) {
    set_cam(cam_id_);
    set_fourcc(arr2str(fourcc_arr_));
    set_fps(fps_);
    set_width(width_);
    set_height(height_);
    set_brightness(brightness_);
    set_contrast(contrast_);
    set_saturation(saturation_);
}

//destructor
cv_camera::~cv_camera() {
    if (cam_.isOpened()) {
        cam_.release();
    }
}

//setters
void cv_camera::set_cam(int c_id) {
    cam_id_ = c_id;
    cam_ = cv::VideoCapture(cam_id_);
    if (!cam_.isOpened()) {
        throw std::runtime_error("Failed to open the camera with ID: " + std::to_string(cam_id_));
    }
}

void cv_camera::set_fourcc(const std::string& fourcc_str) {
    if (fourcc_str.size() != FOURCC_ARRAY_SIZE - 1) {
        throw std::invalid_argument("CV Error: FourCC should contain exactly four characters!");
    }
    fourcc_arr_ = str2arr<FOURCC_ARRAY_SIZE>(fourcc_str);
    cam_.set(cv::CAP_PROP_FOURCC, encode_fourcc(fourcc_arr_));
}

void cv_camera::set_fps(double fps) {
    fps_ = fps;
    cam_.set(cv::CAP_PROP_FPS, fps_);
}

void cv_camera::set_width(double w) {
    width_ = w;
    cam_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
}

void cv_camera::set_height(double h) {
    height_ = h;
    cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
}

void cv_camera::set_brightness(double br) {
    brightness_ = br;
    cam_.set(cv::CAP_PROP_BRIGHTNESS, brightness_);
}

void cv_camera::set_contrast(double ctr) {
    contrast_ = ctr;
    cam_.set(cv::CAP_PROP_CONTRAST, contrast_);
}

void cv_camera::set_saturation(double sat) {
    saturation_ = sat;
    cam_.set(cv::CAP_PROP_SATURATION, saturation_);
}

// getters
int cv_camera::get_cam_id() const {
    return cam_id_;
}

std::string cv_camera::get_fourcc() const {
    return arr2str(fourcc_arr_);
}

double cv_camera::get_fps() const {
    return fps_;
}

double cv_camera::get_width() const {
    return width_;
}

double cv_camera::get_height() const {
    return height_;
}

double cv_camera::get_brightness() const {
    return brightness_;
}

double cv_camera::get_contrast() const {
    return contrast_;
}

double cv_camera::get_saturation() const {
    return saturation_;
}

// access opencv functions
bool cv_camera::isopen() const {
    return cam_.isOpened();
}

bool cv_camera::read(cv::Mat& frame) {
    return cam_.read(frame);
}

// additional functions
int cv_camera::encode_fourcc(const std::array<char, FOURCC_ARRAY_SIZE>& fourcc_arr) {
    return cv::VideoWriter::fourcc(
        fourcc_arr[0], fourcc_arr[1], fourcc_arr[2], fourcc_arr[3]
    );
}