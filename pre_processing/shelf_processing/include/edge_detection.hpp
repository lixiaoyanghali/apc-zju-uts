#ifndef EDGE_DETECTION_HPP
#define EDGE_DETECTION_HPP
#include <stdio.h>
#include <vector>
#include <list>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class EdgeDetection{
public:
    cv::Mat img, dstImg, detectedEdge;
    int lowThreshold;
    int ratio;
    string windowName;
    int kernelSize;

    int houghMethod;
    int edgeThresh;
    int maxLowThreshold;


    // constructor
    EdgeDetection( cv::Mat img_, int houghMethod_ ) {
        edgeThresh = 1;
        maxLowThreshold = 100;
        ratio = 3;
        kernelSize = 3;
        windowName = "Edge Map";
        img = img_.clone();
        houghMethod = houghMethod_;     // houghMethod = 0, no hough
                                        // houghMethod = 1, regular hough
                                        // houghMethod = 2, probabilistic hough
    }

    // trackbar callback
    static void cannyThreshold( int , void * cookie ) {
        EdgeDetection* obj = static_cast<EdgeDetection*> (cookie);
        cv::blur( obj->img, obj->detectedEdge, cv::Size(3,3) );
        cv::Canny( obj->detectedEdge, obj->detectedEdge, obj->lowThreshold, obj->lowThreshold*obj->ratio, obj->kernelSize );
        obj->dstImg = cv::Scalar::all(0);
        obj->img.copyTo( obj->dstImg, obj->detectedEdge );

        if ( obj->houghMethod != 0 ) {
            if ( obj->houghMethod == 1 ) {
                vector<cv::Vec2f> lines;
                cv::HoughLines( obj->dstImg, lines, 1, CV_PI/180, 100, 0, 0 );
                for( size_t i = 0; i < lines.size(); i++ ) {
                    float rho = lines[i][0], theta = lines[i][1];
                    cv::Point pt1, pt2;
                    double a = cos(theta), b = sin(theta);
                    double x0 = a*rho, y0 = b*rho;
                    pt1.x = cvRound(x0 + 1000*(-b));
                    pt1.y = cvRound(y0 + 1000*(a));
                    pt2.x = cvRound(x0 - 1000*(-b));
                    pt2.y = cvRound(y0 - 1000*(a));
                    cv::line( obj->dstImg, pt1, pt2, cv::Scalar::all(255), 3, CV_AA);
                }
            }
            else if ( obj->houghMethod == 2 ) {
                vector<cv::Vec4i> lines;
                cv::HoughLinesP( obj->dstImg, lines, 1, CV_PI/180, 50, 50, 10 );
                for( size_t i = 0; i < lines.size(); i++ ) {
                    cv::Vec4i l = lines[i];
                    cv::line( obj->dstImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar::all(255), 3, CV_AA );
                }
            }
        }

        cv::imshow( obj->windowName, obj->dstImg );
    }

    void run() {
        cv::namedWindow( windowName, CV_WINDOW_AUTOSIZE );
        cv::createTrackbar( "Min Threshold:", windowName, &lowThreshold, maxLowThreshold, &EdgeDetection::cannyThreshold, this );
        cv::waitKey(0);
    }
};

#endif // EDGE_DETECTION_HPP
