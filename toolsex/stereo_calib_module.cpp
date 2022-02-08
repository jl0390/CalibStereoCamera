// stereo_rectify.cpp : Defines the entry point for the console application.
//
#include <fstream>
#include "stereo_calib_module.h"

int getImageNames(const std::string &dirname, const std::string &filename, std::vector<cv::String> &imageNames)
{
    if (filename.empty()) {
        std::string filter = dirname + "*.jpg";
        cv::glob(filter, imageNames, true);
    }
    else {
        std::ifstream ifs(dirname+filename);
        if (ifs.is_open()) {
            std::string line;
            while (std::getline(ifs, line)) {
                if (!line.empty())
                    imageNames.push_back(dirname + line);
            }
        }
    }
    return (int)imageNames.size();
}

cv::Mat load(const std::string& fname, const std::string &key)
{
    cv::Mat res;
    try {
        cv::FileStorage fs(fname, cv::FileStorage::READ);

        if (fs.isOpened()) {
            fs[key] >> res;
        }
    }
    catch (...) {
    }
    return res;
}

bool readCameraParam(const std::string &fname, CameraParam &param)
{
    //load camera parameters
    try {
        cv::FileStorage fs(fname, cv::FileStorage::READ);

        if (fs.isOpened()) {
            fs["M1"] >> param.M1;
            fs["M2"] >> param.M2;
            fs["D1"] >> param.D1;
            fs["D2"] >> param.D2;
            fs["R"] >> param.R;
            fs["T"] >> param.T;
            fs["E"] >> param.E;
            fs["F"] >> param.F;

            fs["FT"] >> param.FT;
#if 0
            fs["P1"] >> param.P1;
            fs["R1"] >> param.R1;
            fs["P2"] >> param.P2;
            fs["R2"] >> param.R2;
            fs["Q"] >> param.Q;

            fs["roiRC1"] >> param.roiRect1;
            fs["roiRC2"] >> param.roiRect2;
#endif
        }
        else {
            std::string dirname = fname + "/";
            param.M1 = load(dirname + "M1.xml", "M1");
            param.D1 = load(dirname + "D1.xml", "D1");
            param.M2 = load(dirname + "M2.xml", "M2");
            param.D2 = load(dirname + "D2.xml", "D2");
            param.P1 = load(dirname + "P1.xml", "P1");
            param.R1 = load(dirname + "R1.xml", "R1");
            param.P2 = load(dirname + "P2.xml", "P2");
            param.R2 = load(dirname + "R2.xml", "R2");
        }
    }
    catch (...) {
        return false;
    }
    return true;
}

bool writeCameraParam(const std::string &fname, CameraParam &param)
{
    //load camera parameters
    try {
        cv::FileStorage fs(fname, cv::FileStorage::WRITE);

        if (!fs.isOpened())
            return false;

        fs << "M1" << param.M1;
        fs << "M2" << param.M2;
        fs << "D1" << param.D1;
        fs << "D2" << param.D2;
        fs << "R" << param.R;
        fs << "T" << param.T;
        fs << "E" << param.E;
        fs << "F" << param.F;

        fs << "FT" << param.FT;

#if 0
        fs << "P1" << param.P1;
        fs << "R1" << param.R1;
        fs << "P2" << param.P2;
        fs << "R2" << param.R2;
        fs << "Q" << param.Q;

        fs << "roiRC1" << param.roiRect1;
        fs << "roiRC2" << param.roiRect2;
#endif
    }
    catch (...) {
        return false;
    }
    return true;
}

bool writeMapParam(const std::string &fname, const std::string &key, const cv::Mat &map)
{
    //load camera parameters
    try {
        cv::FileStorage fs(fname, cv::FileStorage::WRITE);

        if (!fs.isOpened())
            return false;

        fs << key << map;
    }
    catch (...) {
        return false;
    }
    return true;
}

bool remap(cv::Mat &leftImg, cv::Mat &rightImg, CalibData &cd)
{
    if (leftImg.empty() || rightImg.empty())
        return false;
    if (leftImg.size() != rightImg.size())
        return false;

    cd.img1 = leftImg;
    cd.img2 = rightImg;

    if (cd.imageSize.width == 0 || cd.imageSize.height == 0) {
        int flags = cv::CALIB_ZERO_DISPARITY;
        cd.imageSize = cd.img1.size();
        const CameraParam &param = cd.param;
        cv::stereoRectify(cd.param.M1, cd.param.D1, cd.param.M2, cd.param.D2, cd.imageSize,
            cd.param.R, cd.param.T, cd.param.R1, cd.param.R2, cd.param.P1, cd.param.P2, cd.param.Q,
            flags, -1, cd.imageSize, &cd.param.roiRect1, &cd.param.roiRect2);

        int m1type = CV_16SC2;  // CV_32FC1;
        cv::initUndistortRectifyMap(param.M1, param.D1, param.R1, param.P1, cd.imageSize, m1type, cd.map11, cd.map12);
        cv::initUndistortRectifyMap(param.M2, param.D2, param.R2, param.P2, cd.imageSize, m1type, cd.map21, cd.map22);
#if 0
        writeMapParam("map11.xml", "map", cd.map11);
        writeMapParam("map12.xml", "map", cd.map12);
        writeMapParam("map21.xml", "map", cd.map21);
        writeMapParam("map22.xml", "map", cd.map22);
#endif
    }
    if (cd.img1.size() != cd.imageSize)
        cv::resize(cd.img1, cd.img1, cd.imageSize);
    if (cd.img2.size() != cd.imageSize)
        cv::resize(cd.img2, cd.img2, cd.imageSize);

    cv::remap(cd.img1, cd.img1r, cd.map11, cd.map12, cv::INTER_CUBIC);
    cv::remap(cd.img2, cd.img2r, cd.map21, cd.map22, cv::INTER_CUBIC);

    return true;
}

bool remap(cv::Mat &img, CalibData &cd)
{
    if (img.empty())
        return false;

    cv::Mat leftImg, rightImg;
    getBinocsSubImage(img, 0, leftImg);
    getBinocsSubImage(img, 1, rightImg);

    return remap(leftImg, rightImg, cd);
}

template <typename T>
void cvMatToVect(cv::Mat &m, std::vector<T> &vecs)
{
    vecs.resize(m.rows*m.cols);
    for (int y = 0; y < m.rows; y++) {
        T* ptr = m.ptr<T>(y);
        for (int x = 0; x < m.cols; x++)
            vecs[y*m.cols + x] = ptr[x];
    }
}

bool blockMatch(cv::Mat &img1, cv::Mat &img2, BlockMatch &bm, CameraParam &param)
{
    if (!bm.bm || img1.empty() || img2.empty())
        return false;

    cv::Mat gray1, gray2;
    if (img1.channels() == 3)
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    else
        gray1 = img1;
    if (img2.channels() == 3)
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    else
        gray2 = img2;
    bm.bm->compute(gray1, gray2, bm.disp);

#if 0
    std::vector<ushort> disps;
    cvMatToVect<ushort>(bm.disp, disps);
#endif

#if STEREO_3D
    cv::reprojectImageTo3D(bm.disp, bm.img3d, param.Q, true);
#endif
    cv::normalize(bm.disp, bm.vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
    //cv::normalize(bm.disp, bm.vdisp, 0, 256, CV_MINMAX);
    return true;
}

void createPair(CalibData &cd, bool grid)
{
    cd.pair.create(cd.imageSize.height, cd.imageSize.width * 2, CV_8UC3);
    cv::Mat part = cd.pair.colRange(0, cd.imageSize.width);
    if (cd.img1r.channels() == 1)
        cv::cvtColor(cd.img1r, part, cv::COLOR_GRAY2BGR);
    else
        cd.img1r.copyTo(part);

    part = cd.pair.colRange(cd.imageSize.width, cd.imageSize.width * 2);
    if (cd.img2r.channels() == 1)
        cv::cvtColor(cd.img2r, part, cv::COLOR_GRAY2BGR);
    else
        cd.img2r.copyTo(part);

    if (grid) {
        cv::Scalar color(0, 255, 0);
        for (int j = 0; j < cd.pair.rows; j += 40)
            cv::line(cd.pair, cv::Point(0, j), cv::Point(cd.pair.cols, j), color);
        for (int j = 0; j < cd.pair.cols; j += 40)
            cv::line(cd.pair, cv::Point(j, 0), cv::Point(j, cd.pair.rows), color);

        cv::Scalar color1(0, 0, 255);
        if (cd.param.roiRect1.width <= 0 || cd.param.roiRect1.height<=0) {
            cv::rectangle(cd.pair, cd.param.roiRect1, color1);
        }
        if (cd.param.roiRect2.width<0 || cd.param.roiRect2.height<=0) {
            cv::Rect rc = cd.param.roiRect2;
            rc.x += cd.imageSize.width;
            cv::rectangle(cd.pair, rc, color1);
        }
    }
}

void drawGrid(cv::Mat &img, cv::Scalar color)
{
    for (int j = 0; j < img.rows; j += 40)
        cv::line(img, cv::Point(0, j), cv::Point(img.cols, j), color);
    for (int j = 0; j < img.cols; j += 40)
        cv::line(img, cv::Point(j, 0), cv::Point(j, img.rows), color);
}

bool createStereoBM(BlockMatch &bm, int disparity)
{
    int numDisparities = (disparity + 15) & -16;
    int blockSize = 3;
    int cn = 1;

#if STEREO_SGBM
    bm.bm = cv::StereoSGBM::create(
        0,                      //int minDisparity = 0
        numDisparities,    //int numDisparities = 16
        blockSize,              //int blockSize = 3
        8*cn*blockSize*blockSize,//int P1 = 0
        32*cn*blockSize*blockSize,//int P2 = 0
        1,                      //int disp12MaxDiff = 0
        40,                     //int preFilterCap = 0
        10,                     //int uniquenessRatio = 0
        100,                    //int speckleWindowSize = 0
        32,                     //int speckleRange = 0
        //cv::StereoSGBM::MODE_SGBM
        //cv::StereoSGBM::MODE_HH
        cv::StereoSGBM::MODE_SGBM_3WAY
        //cv::StereoSGBM::MODE_HH4
        );
#else
    bm.bm = cv::StereoBM::create(numDisparities, 21);

#endif
    return true;
}

double calcDisparities(BlockMatch &bm, int x, int y, int width, int height, std::vector<double>* disps)
{
    double ds = 0;
    int count = 0;
    for (int row = y; row < y + height; row++) {
        short *ptr = bm.disp.ptr<short>(row);
        for (int col = x; col <= x+width; col++) {
            if (ptr[col] > 0) {
                double disparity = (double)ptr[col] / 16;
                ds += disparity;
                count++;

                if (disps)
                    disps->push_back(disparity);
            }
        }
    }
    if (count>0)
        ds /= count;
    return ds;
}

void drawChessboardCorners(cv::Mat &image, cv::Size patternSize, std::vector< cv::Point2f >& corners, bool patternWasFound)
{
    if (image.channels() == 1)
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::drawChessboardCorners(image, patternSize, corners, patternWasFound);
}
