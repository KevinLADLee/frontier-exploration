
#ifndef PROJECT_FRONTIER_DETECTION_OPENCV_H
#define PROJECT_FRONTIER_DETECTION_OPENCV_H

#include <vector>
#include <opencv2/opencv.hpp>

inline std::vector<std::vector<cv::Point>> FrontierDetection(const cv::Mat &img){

  cv::Mat out(img.rows, img.cols, CV_8UC1);

//  cv::imshow("0", img);
//  cv::waitKey(0);
  // 滤出所有占据部分（障碍物），存到out里
  cv::inRange(img, cv::Scalar(0), cv::Scalar(1), out);
//  cv::imshow("1", out);
//  cv::waitKey(0);

  // 原图做边缘检测
  cv::Mat edges;
  cv::Canny(img, edges, 0, 255);
//  cv::imshow("2", edges);
//  cv::waitKey(0);

  std::vector<std::vector<cv::Point>> contours ;
  cv::Mat hierarchy;
  cv::findContours(out, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//  cv::Mat img2 = cv::Mat(out);

  cv::drawContours(out, contours, -1, (255,255,255), 5);
//  cv::imshow("3", out);
//  cv::waitKey(0);

  cv::bitwise_not(out, out);
  cv::Mat res;
  cv::bitwise_and(out, edges, res);
//  cv::imshow("4", out);
//  cv::waitKey(0);

  auto frontier = cv::Mat(res);
  cv::findContours(frontier, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  cv::drawContours(frontier, contours, -1, (255,255,255), 2);
  cv::findContours(frontier, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//  cv::imshow("frontier", frontier);
//  cv::waitKey(0);

  return contours;

}


#endif //FRONTIER_DETECTION_OPENCV_H
