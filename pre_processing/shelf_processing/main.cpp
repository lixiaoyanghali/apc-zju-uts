#include <iostream>
#include <stdio.h>
#include <vector>
#include <list>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include "include/edge_detection.hpp"
#include "include/select_bin.hpp"

using namespace std;
using namespace pcl;

int main( int argc, char ** argv ) {
    if ( argc < 2 ) {
        printf("Command:\n");
        printf("\t%s <point_cloud.pcd> <image> <depth image>\n", argv[0]);
        return 1;
    }
    if ( string(argv[1]) == "--help" || string(argv[1]) == "-h"  ) {
        printf("Command:\n");
        printf("\t%s <point_cloud.pcd> <image> <depth image>\n", argv[0]);
        return 1;
    }

    // load point cloud and image file
    string cloudName, imgName, depthImgName;
    cloudName   = string(argv[1]);
    imgName     = string(argv[2]);
    depthImgName= string(argv[3]);
    PointCloud<PointXYZRGB>::Ptr cloudPtr( new PointCloud<PointXYZRGB> () );
    io::loadPCDFile( cloudName, *cloudPtr );
    cv::Mat img = cv::imread( imgName, CV_LOAD_IMAGE_COLOR );
    cv::Mat depthImgOri = cv::imread( depthImgName, CV_LOAD_IMAGE_ANYDEPTH );
    cv::Mat depthImg( 480, 640, CV_8UC1 );
    cv::convertScaleAbs( depthImgOri, depthImg, 255/4096.0 );
    cv::Mat grayImg;
    cv::cvtColor( img, grayImg, CV_RGB2GRAY );

    PinPicking<PointXYZRGB> pp(cloudPtr);
    pp.initGUI();
    pp.compute();
}

