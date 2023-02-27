#include "Marker.h"
#include "utility.h"

using cv::Point2d;

Marker::Marker(const Quad &q, int inId)
{
	corners = q.corners;
	lineInf = q.lineInf;
	projectiveDistortion = q.projectiveDistortion;
	H = q.H.clone();
	center = q.center;

	id = inId;
	C = cv::Mat(1, 1, CV_64FC1);
}


Marker::Marker(const Marker &m)
{
	corners = m.corners;
	lineInf = m.lineInf;
	projectiveDistortion = m.projectiveDistortion;
	H = m.H.clone();
	center = m.center;

	id = m.id;
	C = m.C;
}


void Marker::shiftCorners2(int shift)
{
	if (shift == 1)
	{
		Point2d t = corners[0];
		corners[0] = corners[1];
		corners[1] = corners[2];
		corners[2] = corners[3];
		corners[3] = t;
	}
	else if (shift == 2)
	{
		Point2d t1 = corners[0];
		Point2d t2 = corners[1];
		corners[0] = corners[2];
		corners[1] = corners[3];
		corners[2] = t1;
		corners[3] = t2;
	}
	else if (shift == 3)
	{
		Point2d t = corners[0];
		corners[0] = corners[3];
		corners[3] = corners[2];
		corners[2] = corners[1];
		corners[1] = t;
	}
	else
		return;

	// have to recalculate homography after shift
	estimateHomography();
}

float Marker::avgMarkerDistRatio (Marker marker2) {
	float sum = 0;
	float markerLen = sqrt(squaredDistance(corners[0],corners[1]));

	for (int c=0; c < corners.size();c++)
		sum += sqrt(squaredDistance(corners[c],marker2.corners[c]));

	float mean = sum/8; // 2x4 corners
	
	// average distance between markers as ratio of current marker
	return mean/markerLen;
}