#ifndef STAG2_H
#define STAG2_H

#include "EDInterface.h"
#include "QuadDetector.h"
#include "Drawer.h"
#include "Marker.h"
#include "Decoder.h"
#include "PoseRefiner.h"
#include "utility.h"

class Stag2
{
	// if keepLogs is true, keep the intermediate results of the detection algorithm in the memory, to be dumped when asked (default: false)
	bool keepLogs = false;
	int errorCorrection;
	EDInterface edInterface;
	QuadDetector quadDetector;
	Drawer drawer;
	Decoder decoder;
	PoseRefiner poseRefiner;

	vector<cv::Mat> codeLocs;
	vector<cv::Mat> innerLocs;
	vector<cv::Mat> outerLocs;

	cv::Mat image;
	vector<Marker> markers;
	vector<vector<Marker>> groupedMarkers;
	vector<Quad> falseCandidates;

	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	// thresholds and converts to binary code
	Codeword readCode(const Quad &q);
	void fillCodeLocations();
	cv::Mat createMatFromPolarCoords(double radius, double radians, double circleRadius);
public:
	Stag2(int libraryHD = 15, int errorCorrection = 7, bool inKeepLogs = false);
	void detectMarkers(cv::Mat inImage);

	// show is string and shows all specified results, this can be:
	// 		edges, lines, corners, quads, markers, ellipses, distorted quads, false quads
	// if save = true, saves all results (keeping keepLogs in mind)
	void logResults(cv::Mat image, string show = "", bool save = false, string path = "");
};


#endif