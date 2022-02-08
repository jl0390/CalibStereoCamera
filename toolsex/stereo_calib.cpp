// StereoCalibrate.cpp : Defines the entry point for the console application.
//

#include <winsdkver.h>

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <direct.h>

#include "pstdthread.h"
#include "stereo_calib_module.h"

using namespace std;

typedef struct Config {
    Config() {
        deviceId = 0;
        width = 1280;
        height = 480;
        imgNum = 0;
        nx = 8;
        ny = 6;
        squareSize = 1;
    }
    std::string cmd;
    int deviceId;
    int width;
    int height;
    std::string dirname;
    std::string filename;
    int imgNum;
    int nx;
    int ny;
    float squareSize;
    std::string cameraParamFile;
} Config;
static Config s_cfg;

static cv::VideoCapture* openCap(int devid = 0, int width= STEREO_FRAME_WIDTH, int height=STEREO_FRAME_HEIGHT)
{
    cv::VideoCapture *cap;
    cap = new cv::VideoCapture(devid);

    if (cap && !cap->isOpened()) {
        delete cap;
        cap = NULL;
    }
    if (cap == NULL)
        return NULL;

    if (width > 0 && height > 0) {
        cap->set(CV_CAP_PROP_FRAME_WIDTH, width);
        cap->set(CV_CAP_PROP_FRAME_HEIGHT, height);
        //cap->set(CV_CAP_PROP_FPS, 25);
    }
    return cap;
}

static void testCamera(const Config &cfg)
{
    cv::VideoCapture *cap = openCap(cfg.deviceId, cfg.width, cfg.height);
    if (!cap) {
        std::cerr << "Couldn't open the camera device" << std::endl;
        return;
    }

    cv::Mat frame;
    int count = 0;
    while (true) {
        if (!cap->read(frame))
            break;

        cv::imshow("Camera Test", frame);
        int key = cv::waitKey(10) & 0x0ff;
        if (key == 27 || key=='q' || key=='Q')
            break;

        if (key == 0x20) {
            std::string fname = cfg.dirname + "image";
            fname += std::to_string(++count) + ".jpg";
            cv::imwrite(fname, frame);
        }
    }

    cap->release();
    delete cap;
}

static void drawText(cv::Mat &img, int x, int y, const char *str)
{
    float scale = 0.7f;
    int fontFace = cv::FONT_HERSHEY_COMPLEX;
    cv::putText(img, str, cv::Point(x, y), fontFace, scale, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, str, cv::Point(x+1, y+1), fontFace, scale, cv::Scalar(0, 0, 0), 1);
}

static bool createStereoImages(const Config &cfg)
{
    const std::string &dirname = cfg.dirname;
    int nx = cfg.nx;
    int ny = cfg.ny;
    int imgcount = cfg.imgNum;
    const std::string prefix = "image";

    std::string title = "Create stereo images";
    PStdTimer timer;

    cv::VideoCapture *cap = openCap(cfg.deviceId, cfg.width, cfg.height);
    if (cap == NULL) {
        std::cerr << "Couldn't open the camera device: " << cfg.deviceId;
        return false;
    }

    float image_sf = 1.0f;
    float delay = 1.f;
    int board_n = nx * ny;
    cv::Size board_sz = cv::Size(nx, ny);

    // Capture corner views; loop until we've got s_boards number of
    // successful captures (meaning: all corners on each board are found).
    double last_captured_timestamp = 0;
    cv::Size image_size;
    cv::Mat bframe0, bframe, image10, image1, image20, image2;

    _mkdir(dirname.c_str());

    std::string newImageList = dirname + prefix + "-list.txt";
    std::ofstream ofs(newImageList);
    if (!ofs.is_open()) {
        delete cap;
        return false;
    }

    timer.tic();

    char buff[16];
    int count = 0;
    while (count < imgcount) {
        cap->read(bframe0);
        bframe0.copyTo(bframe);
        getBinocsSubImage(bframe, 0, image10);
        getBinocsSubImage(bframe, 1, image20);
        image_size = image10.size();
        resize(image10, image1, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);
        resize(image20, image2, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);
        // Find the board
        //
        vector< cv::Point2f > corners1, corners2;
        bool found1 = cv::findChessboardCorners(image1, board_sz, corners1);
        if (found1) {
            bool found2 = cv::findChessboardCorners(image2, board_sz, corners2);
            if (found2) {
                cv::Mat mcorners1(corners1); // do not copy the data
                mcorners1 *= (1. / image_sf); // scale corner coordinates
                cv::drawChessboardCorners(image10, board_sz, corners1, found1);

                cv::Mat mcorners2(corners2); // do not copy the data
                mcorners2 *= (1. / image_sf); // scale corner coordinates
                cv::drawChessboardCorners(image20, board_sz, corners2, found2);

                // If we got a good board, add it to our data
                //
                double timestamp = timer.toc();

                if (timestamp - last_captured_timestamp > 1000) {
                    last_captured_timestamp = timestamp;
                    bframe ^= cv::Scalar::all(255);
                    count++;

                    sprintf(buff, "%02d", count);
                    std::string fname = prefix + buff;
                    fname += ".jpg";
                    cv::imwrite(dirname + fname, bframe0);
                    ofs << fname << std::endl;
                    std::cout << "Collected out " << "count = " << (int)count
                        << " / " << imgcount << " needed chessboard images" << endl;
                }
            }
        }
        // in color if we did collect the image

        sprintf(buff, "%d/%d", count, imgcount);
        drawText(bframe, 0, 20, buff);
        cv::imshow(title, bframe);

        if ((cv::waitKey(30) & 255) == 27)
            break;

    } // end collection while() loop.
    cv::destroyWindow(title);
    ofs.close();

    if (cap) {
        cap->release();
        delete cap;
    }
    return true;
}

static void stereoCalib(const Config &cfg)
{
    const std::string &dirname = cfg.dirname;
    int nx = cfg.nx;
    int ny = cfg.ny;
    int maxCount = cfg.imgNum;

	bool			showUndistorted = true;
	bool			isVerticalStereo = false;	// horiz or vert cams
	const int		maxScale = 1;
    bool useUncalibrated = false;

	int				i, j;
	int				N = nx * ny;
	vector<cv::String>				imageNames;
	vector<cv::Point3f>				boardModel;
	vector< vector<cv::Point3f> >	objectPoints;
	vector< vector<cv::Point2f> >	points[2];
	vector< cv::Point2f >			corners[2];
	bool							found[2] = { false, false };
	cv::Size						imageSize = cv::Size(640, 480);

	cv::Mat							oimage, gimage, left_image, right_image, image1, image2;
	float							image_sf = 1.f;
	cv::Size						board_sz = cv::Size(nx, ny);
	vector<cv::Point2f>				corners1, corners2;
	vector<vector<cv::Point2f>>		points1, points2;

    float squareSize = cfg.squareSize;

	// READ IN THE LIST OF CIRCLE GRIDS:
    if (0 == getImageNames(cfg.dirname, cfg.filename, imageNames)) {
        std::cerr << "There isn't any images." << std::endl;
        return;
    }

    if (maxCount == 0 || maxCount > imageNames.size())
        maxCount = (int)imageNames.size();
    float rate = (float)maxCount / imageNames.size();

#if 0
	for (int y = 0; y < ny; y++)
		for (int x = 0; x < nx; x++)
			boardModel.push_back(cv::Point3f((float)(x * squareSize), (float)(y * squareSize), 0.f));
#else
    for (int y = 0; y < ny; y++)
        for (int x = 0; x < nx; x++)
            boardModel.push_back(cv::Point3f((float)(y * squareSize), (float)(x * squareSize), 0.f));
#endif

    std::string xmlname = dirname + "camera_params";
    xmlname += std::to_string(maxCount);
    xmlname += ".xml";
    std::cout << "filename: " << xmlname;

    int lastIdx = -1;
    for (int i=0;i<(int)imageNames.size();i++) {
        if (lastIdx >= (int)round(rate*i))
            continue;
        lastIdx = (int)round(rate*i);
        std::string imgname = imageNames[i];
		oimage = cv::imread(imgname, 0);
        if (oimage.empty())
            continue;

        if (oimage.channels() == 3)
            cv::cvtColor(oimage, gimage, cv::COLOR_BGR2GRAY);
        else
            oimage.copyTo(gimage);

		getBinocsSubImage(gimage, 0, left_image);
		getBinocsSubImage(gimage, 1, right_image);
        if (fabs(image_sf - 1.0) > 0.01) {
            resize(left_image, image1, cv::Size(), image_sf, image_sf, cv::INTER_CUBIC);
            resize(right_image, image2, cv::Size(), image_sf, image_sf, cv::INTER_CUBIC);
        }
        else {
            left_image.copyTo(image1);
            right_image.copyTo(image2);
        }

		//if (imageSize.width == 0 || imageSize.height == 0)
		imageSize = left_image.size();

		// Find the board
		bool found1 = cv::findChessboardCorners(image1, board_sz, corners1);
		bool found2 = cv::findChessboardCorners(image2, board_sz, corners2);

		if (found1 && found2) {
			cv::cornerSubPix(image1, corners1, cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
			cv::cornerSubPix(image2, corners2, cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));

            cv::Mat mcorners1(corners1); // do not copy the data
			cv::Mat mcorners2(corners2); // do not copy the data
			mcorners1 *= (1. / image_sf); // scale corner coordinates
			mcorners2 *= (1. / image_sf); // scale corner coordinates
			points1.push_back(corners1);
			points2.push_back(corners2);

			objectPoints.push_back(boardModel);
			points[0].push_back(corners1);
			points[1].push_back(corners2);

#if 1
            oimage = cv::imread(imgname, 1);
            getBinocsSubImage(oimage, 0, left_image);
            getBinocsSubImage(oimage, 1, right_image);
            cv::drawChessboardCorners(left_image, board_sz, corners1, found1);
            cv::drawChessboardCorners(right_image, board_sz, corners2, found2);
#endif
        }

		// in color if we did collect the image
		cv::imshow("Corners", oimage);
		if ((cv::waitKey(30) & 255) == 27)
			break;
	}
	cv::destroyWindow("Corners");

	// CALIBRATE THE STEREO CAMERAS
    CalibData cd;
	cout << "\nRunning stereo calibration : " << maxCount << std::endl;
    cd.param.M1 = cv::Mat::zeros(3, 3, CV_64F);
    cd.param.M2 = cv::Mat::zeros(3, 3, CV_64F);
    cd.param.D1 = cv::Mat::zeros(1, 5, CV_64F);
    cd.param.D2 = cv::Mat::zeros(1, 5, CV_64F);
    cd.param.R = cv::Mat::zeros(3, 3, CV_64F);
    cd.param.T = cv::Mat::zeros(3, 1, CV_64F);
    cd.param.E = cv::Mat::zeros(3, 3, CV_64F);
    cd.param.F = cv::Mat::zeros(3, 3, CV_64F);
    cd.param.Q = cv::Mat::zeros(4, 4, CV_64F);

#if 0
    int calibFlags = cv::CALIB_FIX_ASPECT_RATIO
        | cv::CALIB_ZERO_TANGENT_DIST
        | cv::CALIB_SAME_FOCAL_LENGTH
        | cv::CALIB_USE_INTRINSIC_GUESS
        | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3
        ;
#else
    int calibFlags = cv::CALIB_FIX_ASPECT_RATIO
        | cv::CALIB_ZERO_TANGENT_DIST
        | cv::CALIB_SAME_FOCAL_LENGTH
        //| cv::CALIB_USE_INTRINSIC_GUESS
        //| cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_USE_INTRINSIC_GUESS | 
        | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3
        //cv::CALIB_FIX_INTRINSIC
        ;
#endif
    cv::setIdentity(cd.param.M1);
    cv::setIdentity(cd.param.M2);

#if 1
	cv::stereoCalibrate(
		objectPoints,
		points[0],
		points[1],
		cd.param.M1, cd.param.D1, cd.param.M2, cd.param.D2,
		imageSize,
        cd.param.R, cd.param.T, cd.param.E, cd.param.F,
        calibFlags,
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5)
	);
#else
    readCameraParam(xmlname, cd.param);
#endif
	cout << "Done\n\n";

    // CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	vector< cv::Point3f > lines[2];
	double avgErr = 0;
	int nframes = (int)objectPoints.size();
	for (i = 0; i < nframes; i++) {
		vector< cv::Point2f >& pt0 = points[0][i];
		vector< cv::Point2f >& pt1 = points[1][i];
		cv::undistortPoints(pt0, pt0, cd.param.M1, cd.param.D1, cv::Mat(), cd.param.M1);
		cv::undistortPoints(pt1, pt1, cd.param.M2, cd.param.D2, cv::Mat(), cd.param.M2);
		cv::computeCorrespondEpilines(pt0, 1, cd.param.F, lines[0]);
		cv::computeCorrespondEpilines(pt1, 2, cd.param.F, lines[1]);
		for (j = 0; j < N; j++) {
			double err = fabs(pt0[j].x*lines[1][j].x + pt0[j].y*lines[1][j].y + lines[1][j].z) + 
				fabs(pt1[j].x*lines[0][j].x + pt1[j].y*lines[0][j].y + lines[0][j].z);
			avgErr += err;
		}
	}

	cout << "avg err = " << avgErr / (nframes*N) << endl;

	// COMPUTE AND DISPLAY RECTIFICATION
	//
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (!useUncalibrated) {
		cv::stereoRectify(cd.param.M1, cd.param.D1, cd.param.M2, cd.param.D2,imageSize,
            cd.param.R, cd.param.T, cd.param.R1, cd.param.R2, cd.param.P1, cd.param.P2, cd.param.Q,
            cv::CALIB_ZERO_DISPARITY, -1, imageSize, &cd.param.roiRect1, &cd.param.roiRect2);
	}
	// OR ELSE HARTLEY'S METHOD
	//
	else {
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
		vector< cv::Point2f > allpoints[2];
		for (i = 0; i < nframes; i++) {
			//copy(points[0][i].begin(), points[0][i].end(), back_inserter(allpoints[0]));
			//copy(points[1][i].begin(), points[1][i].end(), back_inserter(allpoints[1]));
			for (j = 0; j < points[0][i].size(); j++) {
				//for (j = points[0][i].size() - 1; j >= 0; j--) {
				allpoints[0].push_back(points[0][i][j]);
				allpoints[1].push_back(points[1][i][j]);
			}
		}

		cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
		cv::Mat H1, H2;
		cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, imageSize, H1, H2, 3);
        cd.param.R1 = cd.param.M1.inv() * H1 * cd.param.M1;
        cd.param.R2 = cd.param.M2.inv() * H2 * cd.param.M2;
	}
    writeCameraParam(xmlname, cd.param);

    if (showUndistorted) {
        // Precompute maps for cvRemap()
        initUndistortRectifyMap(cd.param.M1, cd.param.D1, cd.param.R1, cd.param.P1, imageSize, CV_16SC2, cd.map11, cd.map12);
        initUndistortRectifyMap(cd.param.M2, cd.param.D2, cd.param.R2, cd.param.P2, imageSize, CV_16SC2, cd.map21, cd.map22);

		for (i = 0; i < nframes; i++) {
			gimage = cv::imread(imageNames[i], 0);

            remap(gimage, cd);
            createPair(cd);
			cv::imshow("rectified", cd.pair);
			if ((cv::waitKey() & 255) == 27)
				break;
		}
	}
}

int testStereoCalib(const Config &cfg)
{
    const std::string &dirname = cfg.dirname;
    const std::string &params = cfg.cameraParamFile;

    //loading image list
    std::vector<cv::String> imgNames;

    if (!dirname.empty()) {
        std::string filter = dirname;
        cv::glob(filter, imgNames, true);
    }

    CalibData cd;
    cv::Mat orgImage;

    if (!readCameraParam(params, cd.param)) {
        std::cerr << "Couldn't read camera paramemters : " << params << std::endl;
        return -1;
    }

    createStereoBM(cd.bm, STEREO_MAX_DISPARITY);

    if (imgNames.size() > 0) {
        for (int i = 0; i < (int)imgNames.size(); i++) {
            std::string fname = imgNames[i];
            orgImage = cv::imread(fname);
            if (orgImage.empty())
                continue;

            remap(orgImage, cd);
            createPair(cd);
            blockMatch(cd.img1r, cd.img2r, cd.bm, cd.param);
            drawGrid(orgImage, cv::Scalar(0, 255, 0));

            cv::imshow("image", orgImage);
            cv::imshow("disparity", cd.bm.vdisp);
            cv::imshow("rectified", cd.pair);
            if ((cv::waitKey() & 255) == 27)
                break;
        }
    }
    else {
        cv::VideoCapture *cap = openCap(cfg.deviceId, cfg.width, cfg.height);
        if (!cap) {
            std::cerr << "Couldn't open the camera device" << std::endl;
            return -1;
        }

        cv::Mat frame;
        while (true) {
            if (!cap->read(frame))
                break;
            remap(frame, cd);
            createPair(cd);
            blockMatch(cd.img1r, cd.img2r, cd.bm, cd.param);

            cv::imshow("image", frame);
            cv::imshow("disparity", cd.bm.vdisp);
            cv::imshow("rectified", cd.pair);
            int key = cv::waitKey(10) & 0x0ff;
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }
        cap->release();
        delete cap;
    }
	return 0;
}

typedef struct FocusImage{
    std::string imgname;
    float dist;
}FocusImage;

static double calcDeltaX(std::vector< cv::Point2f > &pts1, std::vector< cv::Point2f > &pts2, BlockMatch* bm)
{
    if (pts1.size() != pts2.size() || pts1.size()==0)
        return 0;

    double dx = 0;
    cv::Point2f pt1, pt2;

    for (int i = 0; i < (int)pts1.size(); i++) {
        pt1 = pts1[i];
        pt2 = pts2[i];

        dx += pt1.x - pt2.x;
    }
    dx /= pts1.size();
    std::cout << "dx=" << dx << std::endl;

    if (bm) {
        float x1, x2, y1, y2;
        for (int i = 0; i < (int)pts1.size(); i++) {
            pt1 = pts1[i];
            pt2 = pts2[i];
            if (i == 0) {
                x1 = x2 = pt1.x;
                y1 = y2 = pt1.y;
            }
            else {
                x1 = std::min(x1, pt1.x);
                x2 = std::max(x2, pt1.x);
                y1 = std::min(y1, pt1.y);
                y2 = std::max(y2, pt1.y);
            }
        }

        double ds = calcDisparities(*bm, x1, y1, x2-x1+1,y2-y1+1, NULL);
        std::cout << "ds=" << ds << std::endl;
    }

    return dx;
}

#if STEREO_3D
static double calcDist(std::vector< cv::Point2f > &pts, BlockMatch &bm)
{
    cv::Mat &img3d = bm.img3d;
    double ss = 0;
    int count = 0;
    for (int i = 0; i < (int)pts.size(); i++) {
        cv::Point pt = pts[i];
        float dist = img3d.ptr<float>(pt.y)[pt.x * 3 + 2];
        if (dist < 10000) {
            ss += dist;
            count++;
        }
    }

    if (count > 0)
        ss /= count;
    return ss;
}
#endif

static bool calcFocus(const Config &cfg)
{
    const std::string dirname = cfg.dirname;
    const std::string imageList = cfg.dirname + "image-list.txt";

    CalibData cd;

    std::ifstream ifs(imageList);
    if (!ifs.is_open()) {
        std::cerr << "Couldn't open image-list.txt" << std::endl;
        return false;
    }

    const std::string &params = cfg.cameraParamFile;
    if (!readCameraParam(params, cd.param)) {
        std::cerr << "Couldn't read camera paramemters : " << params << std::endl;
        return false;
    }

    createStereoBM(cd.bm, STEREO_MAX_DISPARITY);

    cv::Size board_sz = cv::Size(cfg.nx, cfg.ny);

    std::string line;
    int key = 0;
    std::vector<double> dxs;
    std::vector<double> dists;
    while (std::getline(ifs, line)) {
        int pos = (int)line.find(',');
        if (pos == std::string::npos)
            continue;
        std::string imgname = line.substr(0, pos);
        double dist = atof(line.substr(pos + 1).c_str());

        cv::Mat img = cv::imread(dirname + imgname, 0);

        remap(img, cd);
        blockMatch(cd.img1r, cd.img2r, cd.bm, cd.param);

        vector< cv::Point2f > corners1, corners2;
        // Find the board
        bool found1 = cv::findChessboardCorners(cd.img1r, board_sz, corners1);
        bool found2 = cv::findChessboardCorners(cd.img2r, board_sz, corners2);
        if (!found1 || !found2)
            continue;
#if 1
        cv::cornerSubPix(cd.img1r, corners1, cvSize(11, 11), cvSize(-1, -1),
            cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
        cv::cornerSubPix(cd.img2r, corners2, cvSize(11, 11), cvSize(-1, -1),
            cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
#endif
        drawChessboardCorners(cd.img1r, board_sz, corners1, found1);
        drawChessboardCorners(cd.img2r, board_sz, corners2, found2);

#if !STEREO_3D
        double dx = calcDeltaX(corners1, corners2, &cd.bm);
#else
        double dx = calcDist(corners1, cd.bm);
#endif
        if (fabs(dx) > 0.0001) {
            dxs.push_back(dx);
            dists.push_back(dist);
        }
        createPair(cd);
        cv::imshow("disparity", cd.bm.vdisp);
        cv::imshow("rectified", cd.pair);
        key = cv::waitKey() &0x0ff;
        if (key == 27)
            break;
    }
    if (key == 27)
        return false;
    if (dxs.size() < 2)
        return false;

#if !STEREO_3D
    double dx = dxs[0] - dxs[1];
    if (fabs(dx) < 0.0001)
        return false;

    cd.param.FT = (dists[1] - dists[0]) *dxs[0] * dxs[1] / dx;
#else
    cd.param.FT = (dists[1] - dists[0]) / (dxs[1] - dxs[0]);
#endif
    std::cout << "FT=" << cd.param.FT << std::endl;
    writeCameraParam(params, cd.param);
    return true;
}

void parseParams(cv::CommandLineParser &parser, Config &cfg)
{
    cfg.deviceId = parser.get<int>("c");
    cfg.width = parser.get<int>("w");
    cfg.height = parser.get<int>("h");
    cfg.dirname = parser.get<std::string>("dir");
    cfg.filename = parser.get<std::string>("f");
    cfg.imgNum = parser.get<int>("n");
    cfg.nx = parser.get<int>("nx");
    cfg.ny = parser.get<int>("ny");
    cfg.cameraParamFile = parser.get<std::string>("p");
    cfg.squareSize = parser.get<float>("s");

    //check dirname
    if (cfg.dirname.size() > 0) {
        char tag = cfg.dirname[cfg.dirname.size() - 1];
        if (tag != '\\' & tag != '/')
            cfg.dirname += '/';
    }
}

const char *s_keys =
{
    "{ c        camera      | 0         | set cmaera device     }"
    "{ w        width       | 1280      | set cmaera width     }"
    "{ h        height      | 480       | set cmaera height    }"
    "{ d        dir         |           | set image directory     }"
    "{ n        number      | 0         | set image number        }"
    "{ f        file        |           | set file name for image list    }"
    "{ nx       nx          | 8         | x points in chess board }"
    "{ ny       ny          | 6         | y points in chess board }"
    "{ s        square_size | 1         | set the square size of the chess board}"
    "{ p        parameter   |           | set the camera parameters   }"
};

void usage(cv::CommandLineParser &parser)
{
    std::cout << "stereo_calib.exe command (camera | image | calib | test | focus) [OPTIONS] \nOption:\n";
    parser.printMessage();
    std::cout << "parameter example : camera -c=0 -w=1280 -h=480" << std::endl;
    std::cout << "                  : image -d=calib -n=60" << std::endl;
    std::cout << "                  : calib -d=calib -f=image-list.txt -n=60" << std::endl;
    std::cout << "                  : test  -d=calib -p=calib/camera_params60.xml" << std::endl;
    std::cout << "                  : focus -d=focus -p=calib/camera_params60.xml" << std::endl;
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, s_keys);
    if (argc < 2) {
        std::cout << "Please refer to usage" << std::endl;
        usage(parser);
        return -1;
    }

    parser.printMessage();

    s_cfg.cmd = argv[1];
    parseParams(parser, s_cfg);

#if 0 && defined(_DEBUG)
    s_cfg.cmd = "test";
    s_cfg.dirname = "new_calib3";
    s_cfg.cameraParamFile = "new_calib3/camera_params.xml";
#endif
    if (s_cfg.cmd == "camera") {
        testCamera(s_cfg);
    }
    else if (s_cfg.cmd == "image") {
        createStereoImages(s_cfg);
    }
    else if (s_cfg.cmd == "calib") {
        stereoCalib(s_cfg);
    }
    else if (s_cfg.cmd == "test") {
        testStereoCalib(s_cfg);
    }
    else if (s_cfg.cmd == "focus") {
        calcFocus(s_cfg);
    }

	return 0;
}