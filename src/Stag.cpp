#include "Stag.h"
#include "Ellipse.h"
#include <numeric>
#include "markerStats.h"

#define HALF_PI 1.570796326794897

using cv::Mat;
using cv::Point2d;

Stag::Stag(int libraryHD, int inErrorCorrection, bool inKeepLogs)
{
	keepLogs = inKeepLogs;
	errorCorrection = inErrorCorrection;
	fillCodeLocations();
	quadDetector = QuadDetector(keepLogs);
	decoder = Decoder(libraryHD);
}

void Stag::detectMarkers(Mat inImage)
{
	// clear vectors
	groupedMarkers.clear();
	falseCandidates.clear();

	// convert image to gray if not already
	if (inImage.channels() == 1)
		image = inImage;
	else
		cv::cvtColor(inImage, image, cv::COLOR_RGB2GRAY);

	quadDetector.detectQuads(image, &edInterface);
	vector<Quad> quads = quadDetector.getQuads();

	// groupedMarkers.push_back(vector<Marker>());
	for (int indQuad = 0; indQuad < quads.size(); indQuad++)
	{
		quads[indQuad].estimateHomography();

		// Check if black or white square
		quads[indQuad].check_color(image, blackLocs);

		// Scale out corners of white square to include black square
		quads[indQuad].fix_white();

		Codeword c = readCode(quads[indQuad]);
		int shift;
		int id;
		if (decoder.decode(c, errorCorrection, id, shift))
		{
			// define and shift marker
			Marker marker(quads[indQuad], id);
			marker.shiftCorners2(shift);

			// refine pose of marker
			poseRefiner.refineMarkerPose(&edInterface, marker);

			// group duplicate markers
			if (groupedMarkers.size() == 0)
			{
				groupedMarkers.push_back(vector<Marker>());
				groupedMarkers.back().push_back(marker);
			}
			else
			{
				Marker prev_marker = groupedMarkers.back().back();

				// group if matching id and distance between corners is less than 15% of prev_marker length
				if (prev_marker.id == marker.id && prev_marker.avgMarkerDistRatio(marker) < 0.15)
					groupedMarkers.back().push_back(marker);
				else
				{
					groupedMarkers.push_back(vector<Marker>());
					groupedMarkers.back().push_back(marker);
				}
			}
		}

		else if (keepLogs)
			falseCandidates.push_back(quads[indQuad]);
	}

	markers.clear();
	averageGroupedMarkers();
}

void Stag::averageGroupedMarkers()
{
	vector<cv::Point2d> avgCorners;
	for (int i = 0; i < groupedMarkers.size(); i++)
	{
		if (groupedMarkers[i].size() > 0)
		{
			avgCorners = calcAvgCorners(groupedMarkers[i]);
			Quad test(avgCorners);
			markers.push_back(Marker(test, groupedMarkers[i][0].id));
			markers.back().estimateHomography();
			poseRefiner.refineMarkerPose(&edInterface, markers.back());
		}
	}
}

vector<cv::Point2d> Stag::calcAvgCorners(vector<Marker> markers)
{
	vector<cv::Point2d> avgCorners(4);

	avgCorners = markers[0].corners;

	for (int i = 1; i < markers.size(); i++)
	{
		avgCorners[0] += markers[i].corners[0];
		avgCorners[1] += markers[i].corners[1];
		avgCorners[2] += markers[i].corners[2];
		avgCorners[3] += markers[i].corners[3];
	}

	for (int i = 0; i < avgCorners.size(); i++)
	{
		avgCorners[i].x /= markers.size();
		avgCorners[i].y /= markers.size();
	}

	return avgCorners;
}

void Stag::logResults(Mat image, string show, bool save, string path)
{
	vector<std::pair<string, Mat>> drawnImages;

	if (save == true || show.find("edges") != std::string::npos)
		drawnImages.push_back(std::make_pair("edges", drawer.drawEdgeMap(image, edInterface.getEdgeMap())));

	if (save == true || show.find("lines") != std::string::npos)
		drawnImages.push_back(std::make_pair("lines", drawer.drawLines(image, edInterface.getEDLines())));

	if (save == true || show.find("corners") != std::string::npos)
		drawnImages.push_back(std::make_pair("corners", drawer.drawCorners(image, quadDetector.getCornerGroups())));

	if (save == true || show.find("quads") != std::string::npos)
		drawnImages.push_back(std::make_pair("quads", drawer.drawQuads(image, quadDetector.getQuads())));

	if (save == true || show.find("markers") != std::string::npos)
		drawnImages.push_back(std::make_pair("markers", drawer.drawMarkers(image, markers)));

	if (save == true || show.find("ellipses") != std::string::npos)
		drawnImages.push_back(std::make_pair("ellipses", drawer.drawEllipses(image, markers)));

	if (save == true || show.find("distorted quads") != std::string::npos)
		drawnImages.push_back(std::make_pair("distorted quads", drawer.drawQuads(image, quadDetector.getDistortedQuads())));

	if (save == true || show.find("false quads") != std::string::npos)
		drawnImages.push_back(std::make_pair("false quads", drawer.drawQuads(image, falseCandidates)));

	for (int i = 0; i < drawnImages.size(); i++)
	{
		if (show.find(drawnImages[i].first) != std::string::npos)
			cv::imshow(drawnImages[i].first, drawnImages[i].second);
	}

	if (save)
	{
		// remove distorted quads and flase quads if keeplog is false
		if (!keepLogs)
		{
			drawnImages.pop_back();
			drawnImages.pop_back();
		}
		for (int i = 0; i < drawnImages.size(); i++)
		{
			drawer.save(path + drawnImages[i].first + ".png", drawnImages[i].second);
		}
	}
}

vector<double> test1;
Codeword Stag::readCode(const Quad &q)
{
	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	vector<unsigned char> samples(72);

	// a better idea may be creating a list of points to be sampled and let the OpenCV's interpolation function handle the sampling
	for (int i = 0; i < 48; i++)
	{
		Mat projectedPoint = q.H * codeLocs[i];

		samples[i] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * whiteLocs[i];

		samples[i + 48] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * blackLocs[i];

		samples[i + 60] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}

	// threshold the readings using Otsu's method
	cv::threshold(samples, samples, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);

	// create a codeword using the thresholded readings
	Codeword c;
	for (int i = 0; i < 48; i++)
		c[i] = samples[i] / 255;

	return c;
}

void Stag::fillCodeLocations()
{
	blackLocs = vector<Mat>(12);
	whiteLocs = vector<Mat>(12);
	codeLocs = vector<Mat>(48);
	
	// each quadrant is rotated by HALF_PI
	// these part is left as is for self-documenting purposes
	for (int i = 0; i < 4; i++)
	{
		codeLocs[0 + i * 12] = createMatFromPolarCoords(0.088363142525988, 0.785398163397448 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[1 + i * 12] = createMatFromPolarCoords(0.206935928182607, 0.459275804122858 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[2 + i * 12] = createMatFromPolarCoords(0.206935928182607, HALF_PI - 0.459275804122858 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[3 + i * 12] = createMatFromPolarCoords(0.313672146827381, 0.200579720495241 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[4 + i * 12] = createMatFromPolarCoords(0.327493143484516, 0.591687617505840 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[5 + i * 12] = createMatFromPolarCoords(0.327493143484516, HALF_PI - 0.591687617505840 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[6 + i * 12] = createMatFromPolarCoords(0.313672146827381, HALF_PI - 0.200579720495241 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[7 + i * 12] = createMatFromPolarCoords(0.437421957035861, 0.145724938287167 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[8 + i * 12] = createMatFromPolarCoords(0.437226762361658, 0.433363129825345 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[9 + i * 12] = createMatFromPolarCoords(0.430628029742607, 0.785398163397448 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[10 + i * 12] = createMatFromPolarCoords(0.437226762361658, HALF_PI - 0.433363129825345 + i * HALF_PI, markerStats::innerCircleRadius);
		codeLocs[11 + i * 12] = createMatFromPolarCoords(0.437421957035861, HALF_PI - 0.145724938287167 + i * HALF_PI, markerStats::innerCircleRadius);
	}

	for (int i = 0; i < 12; i++)
	{
		blackLocs[i] = Mat(3, 1, CV_64FC1);
		whiteLocs[i] = Mat(3, 1, CV_64FC1);
	}

	blackLocs[0].at<double>(0) = markerStats::borderRatio;
	blackLocs[0].at<double>(1) = markerStats::borderRatio * 3;
	blackLocs[0].at<double>(2) = 1;

	blackLocs[1].at<double>(0) = markerStats::borderRatio;
	blackLocs[1].at<double>(1) = markerStats::borderRatio;
	blackLocs[1].at<double>(2) = 1;

	blackLocs[2].at<double>(0) = markerStats::borderRatio * 3;
	blackLocs[2].at<double>(1) = markerStats::borderRatio;
	blackLocs[2].at<double>(2) = 1;

	blackLocs[3].at<double>(0) = 1 - 3 * markerStats::borderRatio;
	blackLocs[3].at<double>(1) = markerStats::borderRatio;
	blackLocs[3].at<double>(2) = 1;

	blackLocs[4].at<double>(0) = 1 - markerStats::borderRatio;
	blackLocs[4].at<double>(1) = markerStats::borderRatio;
	blackLocs[4].at<double>(2) = 1;

	blackLocs[5].at<double>(0) = 1 - markerStats::borderRatio;
	blackLocs[5].at<double>(1) = markerStats::borderRatio * 3;
	blackLocs[5].at<double>(2) = 1;

	blackLocs[6].at<double>(0) = 1 - markerStats::borderRatio;
	blackLocs[6].at<double>(1) = 1 - 3 * markerStats::borderRatio;
	blackLocs[6].at<double>(2) = 1;

	blackLocs[7].at<double>(0) = 1 - markerStats::borderRatio;
	blackLocs[7].at<double>(1) = 1 - markerStats::borderRatio;
	blackLocs[7].at<double>(2) = 1;

	blackLocs[8].at<double>(0) = 1 - 3 * markerStats::borderRatio;
	blackLocs[8].at<double>(1) = 1 - markerStats::borderRatio;
	blackLocs[8].at<double>(2) = 1;

	blackLocs[9].at<double>(0) = markerStats::borderRatio * 3;
	blackLocs[9].at<double>(1) = 1 - markerStats::borderRatio;
	blackLocs[9].at<double>(2) = 1;

	blackLocs[10].at<double>(0) = markerStats::borderRatio;
	blackLocs[10].at<double>(1) = 1 - markerStats::borderRatio;
	blackLocs[10].at<double>(2) = 1;

	blackLocs[11].at<double>(0) = markerStats::borderRatio;
	blackLocs[11].at<double>(1) = 1 - 3 * markerStats::borderRatio;
	blackLocs[11].at<double>(2) = 1;

	whiteLocs[0].at<double>(0) = 0.25;
	whiteLocs[0].at<double>(1) = -markerStats::borderRatio;
	whiteLocs[0].at<double>(2) = 1;

	whiteLocs[1].at<double>(0) = 0.5;
	whiteLocs[1].at<double>(1) = -markerStats::borderRatio;
	whiteLocs[1].at<double>(2) = 1;

	whiteLocs[2].at<double>(0) = 0.75;
	whiteLocs[2].at<double>(1) = -markerStats::borderRatio;
	whiteLocs[2].at<double>(2) = 1;

	whiteLocs[3].at<double>(0) = 1 + markerStats::borderRatio;
	whiteLocs[3].at<double>(1) = 0.25;
	whiteLocs[3].at<double>(2) = 1;

	whiteLocs[4].at<double>(0) = 1 + markerStats::borderRatio;
	whiteLocs[4].at<double>(1) = 0.5;
	whiteLocs[4].at<double>(2) = 1;

	whiteLocs[5].at<double>(0) = 1 + markerStats::borderRatio;
	whiteLocs[5].at<double>(1) = 0.75;
	whiteLocs[5].at<double>(2) = 1;

	whiteLocs[6].at<double>(0) = 0.75;
	whiteLocs[6].at<double>(1) = 1 + markerStats::borderRatio;
	whiteLocs[6].at<double>(2) = 1;

	whiteLocs[7].at<double>(0) = 0.5;
	whiteLocs[7].at<double>(1) = 1 + markerStats::borderRatio;
	whiteLocs[7].at<double>(2) = 1;

	whiteLocs[8].at<double>(0) = 0.25;
	whiteLocs[8].at<double>(1) = 1 + markerStats::borderRatio;
	whiteLocs[8].at<double>(2) = 1;

	whiteLocs[9].at<double>(0) = -markerStats::borderRatio;
	whiteLocs[9].at<double>(1) = 0.75;
	whiteLocs[9].at<double>(2) = 1;

	whiteLocs[10].at<double>(0) = -markerStats::borderRatio;
	whiteLocs[10].at<double>(1) = 0.5;
	whiteLocs[10].at<double>(2) = 1;

	whiteLocs[11].at<double>(0) = -markerStats::borderRatio;
	whiteLocs[11].at<double>(1) = 0.25;
	whiteLocs[11].at<double>(2) = 1;
}

Mat Stag::createMatFromPolarCoords(double radius, double radians, double circleRadius)
{
	Mat point(3, 1, CV_64FC1);
	test1.push_back(radius);
	point.at<double>(0) = 0.5 + cos(radians) * radius * (circleRadius / 0.5);
	point.at<double>(1) = 0.5 - sin(radians) * radius * (circleRadius / 0.5);
	point.at<double>(2) = 1;
	return point;
}