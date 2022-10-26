#include "Stag.h"
#include "Ellipse.h"
#include "utility.h"
#include <numeric>

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

cv::Mat image2;
void Stag::detectMarkers(Mat inImage)
{
	markers.clear();
	falseCandidates.clear();
	image = inImage;
	cv::cvtColor(image,image2,cv::COLOR_GRAY2RGB);
	quadDetector.detectQuads(image, &edInterface);
	vector<Quad> quads = quadDetector.getQuads();

	// vector<bool> dark_check(12);
	
	for (int indQuad = 0; indQuad < quads.size(); indQuad++)
	{
		

		cv::String dark_inside;
		if (quads[indQuad].dark_inside) {
			dark_inside = "dark";
		}
		else{
			dark_inside = "white";
		}
		// cv::putText(image,dark_inside,quads[indQuad].corners[0],cv::FONT_HERSHEY_PLAIN,5,cv::Scalar(255,0,0));
		// cv::imshow("test4",image);
		quads[indQuad].estimateHomography();
		// quads[indQuad].check_color(image);
		quads[indQuad].fix_white();
		
		
		// cv::putText(image3,std::to_string(percBlack),warped_corners[1],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));

		// cv::imshow("warp",image3);
		// cv::line(image2,quads[indQuad].corners[1],quads[indQuad].corners[2],cv::Scalar(255,0,255));
		// for (int i = 0; i < 12; i++)
		// {
		// 	Mat projectedPointBl = quads[indQuad].H * blackLocs[i];
		// 	Mat projectedPointWh = quads[indQuad].H * whiteLocs[i];
		// 	Mat image1;
		// 	cv::threshold(image, image1, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY_INV);
		// 	if ((quads[indQuad].dark_inside == true) && (image1.at<double>(projectedPointBl.at<double>(0) / projectedPointBl.at<double>(2), projectedPointBl.at<double>(1) / projectedPointBl.at<double>(2)) != 0)) {
		// 		dark_check[i] = false;
		// 	}
		// 	else if ((quads[indQuad].dark_inside == true) && (image1.at<double>(projectedPointWh.at<double>(0) / projectedPointWh.at<double>(2), projectedPointWh.at<double>(1) / projectedPointWh.at<double>(2)) != 255)) {
		// 		dark_check[i] = false;
		// 	}
		// 	else if ((quads[indQuad].dark_inside == false) && (image1.at<double>(projectedPointBl.at<double>(0) / projectedPointBl.at<double>(2), projectedPointBl.at<double>(1) / projectedPointBl.at<double>(2)) != 255)) {
		// 		dark_check[i] = false;
		// 	}
		// 	else if ((quads[indQuad].dark_inside == false) && (image1.at<double>(projectedPointWh.at<double>(0) / projectedPointWh.at<double>(2), projectedPointWh.at<double>(1) / projectedPointWh.at<double>(2)) != 0)) {
		// 		dark_check[i] = false;
		// 	}
		// 	else {
		// 		dark_check[i] = true;
		// 	}
		// }
		// if (std::accumulate(dark_check.begin(),dark_check.end(),0) > .75 * dark_check.size()) {
			
		// 	quads[indQuad].swapDark();
		// 	// std::cout << "1 " << q.dark_inside << std::endl;
			
		// 	// std::cout << "2 " << q.dark_inside << std::endl;
		// }
		
		Codeword c = readCode(quads[indQuad]);
		int shift;
		int id;
		// std::cout << quads[indQuad].dark_inside << std::endl;	
		if (decoder.decode(c, errorCorrection, id, shift))
		{
			Marker marker(quads[indQuad], id);
			marker.shiftCorners2(shift);
			markers.push_back(marker);
		}
		
		
		else if (keepLogs)
			falseCandidates.push_back(quads[indQuad]);
		// std::cout << "out" << std::endl;		
	}
	string path = "";
	std::cout << "in1" << std::endl;	
	// drawer.drawQuads(path,image2,falseCandidates);
	std::cout << "in2" << std::endl;	
	cv::imshow("test4",image2);
	for (int indMarker = 0; indMarker < markers.size(); indMarker++)
		poseRefiner.refineMarkerPose(&edInterface, markers[indMarker]);
	// std::cout << "out1" << std::endl;	
}


void Stag::logResults(Mat image, string path)
{
	// drawer.drawEdgeMap(path + "1 edges.png", image, edInterface.getEdgeMap());
	// drawer.drawLines(path + "2 lines.png", image, edInterface.getEDLines());
	// drawer.drawCorners(path + "3 corners.png", image, quadDetector.getCornerGroups());
	// drawer.drawQuads(path + "4 quads.png", image, quadDetector.getQuads());
	// if (keepLogs)
	// 	drawer.drawQuads(path + "5 distorted quads.png", image, quadDetector.getDistortedQuads());
	drawer.drawMarkers(path + "6 markers.png", image, markers);
	// if (keepLogs)
	// 	drawer.drawQuads(path + "7 false quads.png", image, falseCandidates);
	// drawer.drawEllipses(path + "8 ellipses.png", image, markers);
}

vector<double> test1;
Codeword Stag::readCode(const Quad &q)
{
	// take readings from 48 code locations, 12 black border locations, and 12 white border locations
	vector<unsigned char> samples(72);
	// cv::Mat cl;
	vector<Mat> bl,wl;
	// bool dark_inside = q.dark_inside;
	
	// std::cout << "3 " << q.dark_inside << std::endl;
	if (q.dark_inside == true) {
		bl = blackLocs;
		wl = whiteLocs;
	}
	else {
		bl = whiteLocs;
		wl = blackLocs;
	}

	// a better idea may be creating a list of points to be sampled and let the OpenCV's interpolation function handle the sampling
	for (int i = 0; i < 48; i++)
	{
		Mat projectedPoint = q.H * codeLocs[i];
		
		// std::cout << "2 " << codeLocs[i] << std::endl;
		// std::cout << cl << std::endl;
		// if (q.dark_inside == true)
		cv::circle(image2,Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)),2,cv::Scalar(0,0,255));
		samples[i] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * bl[i];
		samples[i + 48] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
		// if (q.dark_inside == false) {
		// 	samples[i+48] = 0;
		// }
		// std:: cout << samples[i+48] /255 << std::endl;
	}
	for (int i = 0; i < 12; i++)
	{
		Mat projectedPoint = q.H * wl[i];
		samples[i + 60] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
		// if (q.dark_inside == false) {
		// samples[i+60] = 255;
		// }
		// std:: cout << samples[i+60] /255 << std::endl;
	}

	// threshold the readings using Otsu's method
	// cv::imshow("test3",samples);
	cv::threshold(samples, samples, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY_INV);

	// create a codeword using the thresholded readings
	Codeword c;
	for (int i = 0; i < 48; i++)
		c[i] = samples[i] / 255;

	return c;
}


void Stag::fillCodeLocations()
{
	// fill coordinates to be sampled
	// codeLocsOut = vector<Mat>(48);
	blackLocs = vector<Mat>(12);
	whiteLocs = vector<Mat>(12);
	// codeLocsIn = vector<Mat>(48);
	// blackLocsIn = vector<Mat>(12);
	// blackLocsOut = vector<Mat>(12);
	// whiteLocsIn = vector<Mat>(12);
	// whiteLocsOut = vector<Mat>(12);
	double innerCircleRadius, borderDist;
	// double innerCircleRadiusOut, borderDistOut;
	// double borderDist, extra_rotation;

	// code circles are located in a circle with radius outerCircleRadius
	// std::cout << "here " << q.dark_inside << std::endl;
	borderDist= 0.045;
	double innerSquare = 0.8; 
	innerCircleRadius = (0.95 * ((sqrt(2) * innerSquare) / 2))/2;

		codeLocs = vector<Mat>(48);
		// each quadrant is rotated by HALF_PI
		// these part is left as is for self-documenting purposes
		for (int i = 0; i < 4; i++)
		{
			codeLocs[0 + i * 12] = createMatFromPolarCoords(0.088363142525988, 0.785398163397448 + i * HALF_PI, innerCircleRadius);

			codeLocs[1 + i * 12] = createMatFromPolarCoords(0.206935928182607, 0.459275804122858 + i * HALF_PI, innerCircleRadius);
			// codeLocs[2 + i * 12] = (cv::Mat_<double>(3,1) << 0,0,1);
			codeLocs[2 + i * 12] = createMatFromPolarCoords(0.206935928182607, HALF_PI - 0.459275804122858 + i * HALF_PI, innerCircleRadius);

			// codeLocs[2 + i * 12] = createMatFromPolarCoords(0.088363142525988, (angle2) - 0.785398163397448 + i * angle, innerCircleRadius);

			codeLocs[3 + i * 12] = createMatFromPolarCoords(0.313672146827381, 0.200579720495241 + i * HALF_PI, innerCircleRadius);
			codeLocs[4 + i * 12] = createMatFromPolarCoords(0.327493143484516, 0.591687617505840 + i * HALF_PI, innerCircleRadius);
			codeLocs[5 + i * 12] = createMatFromPolarCoords(0.327493143484516, HALF_PI - 0.591687617505840 + i * HALF_PI, innerCircleRadius);
			// codeLocs[5 + i * 12] = (cv::Mat_<double>(3,1) << 0,0,1);
			codeLocs[6 + i * 12] = createMatFromPolarCoords(0.313672146827381, HALF_PI - 0.200579720495241 + i * HALF_PI, innerCircleRadius);
			// codeLocs[6 + i * 12] = (cv::Mat_<double>(3,1) << 0,0,1);
			codeLocs[7 + i * 12] = createMatFromPolarCoords(0.437421957035861, 0.145724938287167 + i * HALF_PI, innerCircleRadius);
			codeLocs[8 + i * 12] = createMatFromPolarCoords(0.437226762361658, 0.433363129825345 + i * HALF_PI, innerCircleRadius);
			codeLocs[9 + i * 12] = createMatFromPolarCoords(0.430628029742607, 0.785398163397448 + i * HALF_PI, innerCircleRadius);
			codeLocs[10 + i * 12] = createMatFromPolarCoords(0.437226762361658, HALF_PI - 0.433363129825345 + i * HALF_PI, innerCircleRadius);
			// codeLocs[10 + i * 12] = (cv::Mat_<double>(3,1) << 0,0,1);
			codeLocs[11 + i * 12] = createMatFromPolarCoords(0.437421957035861, HALF_PI - 0.145724938287167 + i * HALF_PI, innerCircleRadius);
			// codeLocs[11 + i * 12] = (cv::Mat_<double>(3,1) << 0,0,1);
			// cv::Mat test,test1;
		// for (int j =0; j < 4;j++) {
		// 	for (int i =0; i < 11;i++) {
		// 		cv::hconcat(codeLocs[(j * 12) + i],codeLocs[((j * 12) + i)+1],test1);
		// 	}
		// 	test.push_back(test1);
		// }
		// cv::imshow("test5",test);
		// for i in 
		// quadDetector.image1
		
		// if (!q.dark_inside) {
		// 	for (int i = 0; i < codeLocs.size();i++) {
		// 		cv::Point2f center(codeLocs[i].cols/2.0F, codeLocs[i].rows/2.0F);
		// 		Mat rot_mat = cv::getRotationMatrix2D(center, 45, 1.0);
		// 		warpAffine(codeLocs[i], codeLocs[i], rot_mat, codeLocs[i].size());
		// 	}
		}
		

		for (int i = 0; i < 12; i++)
			blackLocs[i] = Mat(3, 1, CV_64FC1);
		for (int i = 0; i < 12; i++)
			whiteLocs[i] = Mat(3, 1, CV_64FC1);

		blackLocs[0].at<double>(0) = borderDist;
		blackLocs[0].at<double>(1) = borderDist * 3;
		blackLocs[0].at<double>(2) = 1;

		blackLocs[1].at<double>(0) = borderDist * 2;
		blackLocs[1].at<double>(1) = borderDist * 2;
		blackLocs[1].at<double>(2) = 1;

		blackLocs[2].at<double>(0) = borderDist * 3;
		blackLocs[2].at<double>(1) = borderDist;
		blackLocs[2].at<double>(2) = 1;

		blackLocs[3].at<double>(0) = 1 - 3 * borderDist;
		blackLocs[3].at<double>(1) = borderDist;
		blackLocs[3].at<double>(2) = 1;

		blackLocs[4].at<double>(0) = 1 - 2 * borderDist;
		blackLocs[4].at<double>(1) = borderDist * 2;
		blackLocs[4].at<double>(2) = 1;

		blackLocs[5].at<double>(0) = 1 - borderDist;
		blackLocs[5].at<double>(1) = borderDist * 3;
		blackLocs[5].at<double>(2) = 1;

		blackLocs[6].at<double>(0) = 1 - borderDist;
		blackLocs[6].at<double>(1) = 1 - 3 * borderDist;
		blackLocs[6].at<double>(2) = 1;

		blackLocs[7].at<double>(0) = 1 - 2 * borderDist;
		blackLocs[7].at<double>(1) = 1 - 2 * borderDist;
		blackLocs[7].at<double>(2) = 1;

		blackLocs[8].at<double>(0) = 1 - 3 * borderDist;
		blackLocs[8].at<double>(1) = 1 - borderDist;
		blackLocs[8].at<double>(2) = 1;

		blackLocs[9].at<double>(0) = borderDist * 3;
		blackLocs[9].at<double>(1) = 1 - borderDist;
		blackLocs[9].at<double>(2) = 1;

		blackLocs[10].at<double>(0) = borderDist * 2;
		blackLocs[10].at<double>(1) = 1 - 2 * borderDist;
		blackLocs[10].at<double>(2) = 1;

		blackLocs[11].at<double>(0) = borderDist;
		blackLocs[11].at<double>(1) = 1 - 3 * borderDist;
		blackLocs[11].at<double>(2) = 1;


		whiteLocs[0].at<double>(0) = 0.25;
		whiteLocs[0].at<double>(1) = -borderDist;
		whiteLocs[0].at<double>(2) = 1;

		whiteLocs[1].at<double>(0) = 0.5;
		whiteLocs[1].at<double>(1) = -borderDist;
		whiteLocs[1].at<double>(2) = 1;

		whiteLocs[2].at<double>(0) = 0.75;
		whiteLocs[2].at<double>(1) = -borderDist;
		whiteLocs[2].at<double>(2) = 1;

		whiteLocs[3].at<double>(0) = 1 + borderDist;
		whiteLocs[3].at<double>(1) = 0.25;
		whiteLocs[3].at<double>(2) = 1;

		whiteLocs[4].at<double>(0) = 1 + borderDist;
		whiteLocs[4].at<double>(1) = 0.5;
		whiteLocs[4].at<double>(2) = 1;

		whiteLocs[5].at<double>(0) = 1 + borderDist;
		whiteLocs[5].at<double>(1) = 0.75;
		whiteLocs[5].at<double>(2) = 1;

		whiteLocs[6].at<double>(0) = 0.75;
		whiteLocs[6].at<double>(1) = 1 + borderDist;
		whiteLocs[6].at<double>(2) = 1;

		whiteLocs[7].at<double>(0) = 0.5;
		whiteLocs[7].at<double>(1) = 1 + borderDist;
		whiteLocs[7].at<double>(2) = 1;

		whiteLocs[8].at<double>(0) = 0.25;
		whiteLocs[8].at<double>(1) = 1 + borderDist;
		whiteLocs[8].at<double>(2) = 1;

		whiteLocs[9].at<double>(0) = -borderDist;
		whiteLocs[9].at<double>(1) = 0.75;
		whiteLocs[9].at<double>(2) = 1;

		whiteLocs[10].at<double>(0) = -borderDist;
		whiteLocs[10].at<double>(1) = 0.5;
		whiteLocs[10].at<double>(2) = 1;

		whiteLocs[11].at<double>(0) = -borderDist;
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