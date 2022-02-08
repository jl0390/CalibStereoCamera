// stereo_rectify.h
//
#ifndef STEREO_RECTIFY_H
#define STEREO_RECTIFY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if CV_MAJOR_VERSION >= 3
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#endif
#include "opencv_ext.hpp"

#define STEREO_FRAME_WIDTH      (640*2)    //640
#define STEREO_FRAME_HEIGHT     480        //240
#define STEREO_MAX_DISPARITY    120
#define STEREO_SGBM             0
#define STEREO_3D               0

typedef struct CameraParam {
    CameraParam() {
        FT = 3000;
    }
    cv::Mat M1, M2, D1, D2, R, T, E, F, P1, R1, P2, R2, Q;
    double FT;
    cv::Rect roiRect1, roiRect2;
} CameraParam;

typedef struct BlockMatch {
#if STEREO_SGBM
    cv::Ptr<cv::StereoSGBM> bm;
#else
    cv::Ptr<cv::StereoBM> bm;
#endif
    cv::Mat disp, vdisp;
#if STEREO_3D
    cv::Mat disp_mem;
    cv::Mat img3d;
#endif
} BlockMatch;

typedef struct CalibData {
    CameraParam param;

    cv::Mat img1, img2;
    cv::Size imageSize;
    cv::Mat map11, map12, map21, map22;
    cv::Mat img1r, img2r;
    cv::Mat pair;

    BlockMatch bm;
} CalibData;

inline void getBinocsSubImage(cv::Mat &src, int index, cv::Mat &dst)
{
    cv::Rect rc = cv::Rect(0, 0, src.cols / 2, src.rows);
    if (index == 1)
        rc.x = src.cols / 2;
    dst = src(rc);
}

int getImageNames(const std::string &dirname, const std::string &filename, std::vector<cv::String> &imageNames);
bool readCameraParam(const std::string &fname, CameraParam &param);
bool writeCameraParam(const std::string &fname, CameraParam &param);
bool writeMapParam(const std::string &fname, const std::string &key, const cv::Mat &map);

bool remap(cv::Mat &leftImg, cv::Mat &rightImg, CalibData &cd);
bool remap(cv::Mat &img, CalibData &cd);
bool blockMatch(cv::Mat &img1, cv::Mat &img2, BlockMatch &bm, CameraParam &param);
void createPair(CalibData &cd, bool grid = true);
void drawGrid(cv::Mat &img, cv::Scalar color=cv::Scalar(0,255,0));
bool createStereoBM(BlockMatch &bm, int disparity=STEREO_MAX_DISPARITY);
double calcDisparities(BlockMatch &bm, int x, int y, int width, int height, std::vector<double>* disps);

void drawChessboardCorners(cv::Mat &image, cv::Size patternSize, std::vector< cv::Point2f >& corners, bool patternWasFound);

#endif //STEREO_RECTIFY_H
