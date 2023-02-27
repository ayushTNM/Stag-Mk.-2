#ifndef DRAWER_H
#define DRAWER_H

#include <string>
#include "opencv2/opencv.hpp"

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"
#include "QuadDetector.h"
#include "Marker.h"

using std::string;


class Drawer
{
	void colorAPixel(cv::Mat& img, int x, int y, cv::Scalar color, int dotWidth);

public:
	// draws edge segments
	cv::Mat drawEdgeMap(cv::Mat image, EdgeMap* edgeMap);

	// draws line segments
	cv::Mat drawLines(cv::Mat image, EDLines* edLines);

	// draws corners (intersections of line segments)
	cv::Mat drawCorners(cv::Mat image, const vector<vector<Corner>> &cornerGroups);

	// draws quads
	cv::Mat drawQuads(cv::Mat &image, const vector<Quad> &quads);

	// draws markers
	cv::Mat drawMarkers(cv::Mat image, const vector<Marker> &markers);

	// draws refined markers and their ellipses
	cv::Mat drawEllipses(cv::Mat image, const vector<Marker> &markers);

	// save drawing
	void save(const string& path, cv::Mat image);
};

#endif