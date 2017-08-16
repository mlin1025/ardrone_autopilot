#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>

const int circleRadius = 50;

double getAverageColor(cv::Mat& gray, cv::Point center) {
    double av = 0;
    int sz = 2;
    for (int x = -sz; x != sz; ++x)
        for (int y = -sz; y != sz; ++y)
            av += gray.at<int>(center.x + x, center.y + y);
    return av / (sz * sz);
}

std::pair<cv::Point, cv::Point> getTargetVector(cv::Mat& gray, std::vector<cv::Vec3f> circles) {
    cv::Vec2f ret;
    std::vector<std::pair<double, std::pair<int, int>>> colors;
    for (auto crc : circles) {
        cv::Point center(cvRound(crc[0]), cvRound(crc[1]));
        colors.push_back(std::make_pair(getAverageColor(gray, center), std::make_pair(center.x, center.y)));
    }
    std::sort(colors.begin(), colors.end());
    std::reverse(colors.begin(), colors.end());

    cv::Point vecHead = cv::Point(colors[0].second.first, colors[0].second.second);
    cv::Point vecTail = cv::Point((colors[1].second.first + colors[2].second.first) / 2,
        (colors[1].second.second + colors[2].second.second) / 2);
    return std::make_pair(vecTail, vecHead);
}

void processImage(cv::Mat& img) {
  

  cv::Mat imgHSV;
  cv::medianBlur(img, img, 3);
  cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  
  cv::Mat green_hue_range;
  cv::Mat red_hue_range;
  cv::Mat blue_hue_range;
  // Refuse pixels only in red green and blue ranges
  cv::inRange(imgHSV, cv::Scalar(134, 26, 199), cv::Scalar(172, 255, 255), red_hue_range);
  cv::inRange(imgHSV, cv::Scalar(108, 162, 72), cv::Scalar(143, 255, 255), blue_hue_range);
  cv::inRange(imgHSV, cv::Scalar(48, 162, 72), cv::Scalar(103, 255, 255), green_hue_range);
  
  // Summ all ranges
  cv::Mat summ;
  cv::addWeighted(red_hue_range, 1.0, green_hue_range, 1.0, 0.0, summ);
  cv::addWeighted(blue_hue_range, 1.0, summ, 1.0, 0.0, summ);
    
  // smooth it, otherwise a lot of false circles may be detected
  cv::GaussianBlur( summ, summ, cv::Size(9, 9), 2, 2 );
  std::vector<cv::Vec3f> circles;
    HoughCircles(summ, circles, CV_HOUGH_GRADIENT, 1, img.rows/8, 100, 30);
    //if (circles.size() != 3) 
       // return;
    for(size_t i = 0; i < circles.size(); i++ ) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        cv::Point x_p(cvRound(circles[i][0]), cvRound(circles[i][1]) + 23);
        cv::Point rad_p(cvRound(circles[i][0]), cvRound(circles[i][1]) + 46);
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         cv::circle(img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         cv::circle(img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

         // draw coords and radius
         std::string x_coord = std::to_string((int)circles[i][0]);
         std::string y_coord = std::to_string((int)circles[i][1]);
         std::string rad = std::to_string(radius);
         cv::Scalar color(1, 1, 1, 0.5);
         cv::putText(img, "x: " + x_coord, center, cv::FONT_HERSHEY_DUPLEX, 0.8, color);
         cv::putText(img, "y: " + y_coord, x_p, cv::FONT_HERSHEY_DUPLEX, 0.8, color);
         cv::putText(img, "r: " + rad, rad_p, cv::FONT_HERSHEY_DUPLEX, 0.8, color);

    }
 /*        if (circles.size() == 2) {
             float x_between = (circles[0][0] + circles[1][0]) / 2;
             float y_between = (circles[0][1] + circles[1][1]) / 2;
	     return cv::Point(x_between, y_between);
         } else return cv::Point(circles[0][0], circles[0][1]);
*/
//    auto vc = getTargetVector(gray, circles);
//    cv::arrowedLine(img, vc.first, vc.second, cv::Scalar(255, 0, 0), 4);
	return;
}
