#include "Drawer.h"
#include "colors.h"
#include <opencv2/core/core_c.h>

using cv::Mat;
using cv::Point2d;

void Drawer::colorAPixel(cv::Mat &img, int x, int y, cv::Scalar color, int dotWidth)
{
	for (int i = y - dotWidth; i < y + dotWidth + 1; i++)
	{
		for (int j = x - dotWidth; j < x + dotWidth + 1; j++)
		{
			if ((i >= 0) && (i < img.rows) && (j >= 0) && (j < img.cols))
			{
				img.at<cv::Vec3b>(i, j)[0] = color.val[0];
				img.at<cv::Vec3b>(i, j)[1] = color.val[1];
				img.at<cv::Vec3b>(i, j)[2] = color.val[2];
			}
		}
	}
}

Mat Drawer::drawEdgeMap(Mat image, EdgeMap *edgeMap)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	int dotWidth = 1;
	int whiteDotWidth = 2;

	for (int i = 0; i < edgeMap->noSegments; i++)
	{
		for (int j = 0; j < edgeMap->segments[i].noPixels; j++)
		{
			colorAPixel(bgrMat, edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r, cv::Scalar(255, 255, 255), whiteDotWidth);
		}
	}

	for (int i = 0; i < edgeMap->noSegments; i++)
	{
		for (int j = 0; j < edgeMap->segments[i].noPixels; j++)
		{
			colorAPixel(bgrMat, edgeMap->segments[i].pixels[j].c, edgeMap->segments[i].pixels[j].r, colors[i % colors.size()], dotWidth);
		}
	}
	return bgrMat;
}

Mat Drawer::drawLines(Mat image, EDLines *edLines)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	int currSegment = -1;
	for (int i = 0; i < edLines->noLines; i++)
	{
		if (edLines->lines[i].segmentNo != currSegment)
			currSegment = edLines->lines[i].segmentNo;

		cv::line(bgrMat, cv::Point(edLines->lines[i].sx, edLines->lines[i].sy), cv::Point(edLines->lines[i].ex, edLines->lines[i].ey), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
		cv::line(bgrMat, cv::Point(edLines->lines[i].sx, edLines->lines[i].sy), cv::Point(edLines->lines[i].ex, edLines->lines[i].ey), colors[i % colors.size()], 2, cv::LINE_AA);
	}
	return bgrMat;
}

Mat Drawer::drawCorners(Mat image, const vector<vector<Corner>> &cornerGroups)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	for (int i = 0; i < cornerGroups.size(); i++)
	{
		for (int j = 0; j < cornerGroups[i].size(); j++)
		{
			cv::circle(bgrMat, cv::Point(cornerGroups[i][j].loc.x, cornerGroups[i][j].loc.y), 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
			cv::circle(bgrMat, cv::Point(cornerGroups[i][j].loc.x, cornerGroups[i][j].loc.y), 3, colors[i % colors.size()], -1, cv::LINE_AA);
		}
	}
	return bgrMat;
}

Mat Drawer::drawQuads(Mat &image, const vector<Quad> &quads)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	cv::Scalar color;
	for (int i = 0; i < quads.size(); i++)
	{
		color = cv::Scalar(50, 255, 50);
		vector<Point2d> corners = quads[i].corners;

		for (int j = 0; j < 4; j++)
		{
			cv::line(bgrMat, cv::Point(corners[j].x, corners[j].y), cv::Point(corners[(j + 1) % 4].x, corners[(j + 1) % 4].y), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
		}
		for (int j = 0; j < 4; j++)
			cv::line(bgrMat, cv::Point(corners[j].x, corners[j].y), cv::Point(corners[(j + 1) % 4].x, corners[(j + 1) % 4].y), color, 2, cv::LINE_AA);
	}

	return bgrMat;
}

Mat Drawer::drawMarkers(Mat image, const vector<Marker> &markers)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	for (int i = 0; i < markers.size(); i++)
	{
		vector<Point2d> corners = markers[i].corners;
		Point2d center = markers[i].center;

		cv::circle(bgrMat, cv::Point(corners[0].x, corners[0].y), 6, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
		for (int j = 0; j < 4; j++)
			cv::line(bgrMat, cv::Point(corners[j].x, corners[j].y), cv::Point(corners[(j + 1) % 4].x, corners[(j + 1) % 4].y), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);

		cv::circle(bgrMat, cv::Point(corners[0].x, corners[0].y), 5, cv::Scalar(50, 255, 50), -1, cv::LINE_AA);
		for (int j = 0; j < 4; j++)
			cv::line(bgrMat, cv::Point(corners[j].x, corners[j].y), cv::Point(corners[(j + 1) % 4].x, corners[(j + 1) % 4].y), cv::Scalar(50, 255, 50), 2, cv::LINE_AA);

		cv::circle(bgrMat, cv::Point(center.x, center.y), 6, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
		cv::circle(bgrMat, cv::Point(center.x, center.y), 5, cv::Scalar(50, 255, 50), -1, cv::LINE_AA);

		cv::putText(bgrMat, std::to_string(markers[i].id), center, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
		cv::putText(bgrMat, std::to_string(markers[i].id), center, cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(50, 50, 255), 2, cv::LINE_AA);
	}

	return bgrMat;
}

Mat Drawer::drawEllipses(Mat image, const vector<Marker> &markers)
{
	Mat bgrMat;
	if (image.channels() == 1)
	{
		Mat greyMat = image.clone();
		cv::cvtColor(greyMat, bgrMat, cv::COLOR_GRAY2BGR);
	}
	else
		bgrMat = image.clone();

	int dotWidth = 1;
	int whiteDotWidth = 2;

	for (int i = 0; i < markers.size(); i++)
	{
		// skip if the ellipse is not localized
		if (markers[i].C.size().width == 1)
			continue;

		for (int y = 0; y < image.size().height; y++)
		{
			vector<double> xOfPointsOnConic;

			double a = markers[i].C.at<double>(0, 0);
			double b = markers[i].C.at<double>(0, 1) * 2 * y + markers[i].C.at<double>(0, 2) * 2;
			double c = markers[i].C.at<double>(1, 1) * y * y + markers[i].C.at<double>(1, 2) * 2 * y + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				xOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				xOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				xOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < xOfPointsOnConic.size(); j++)
			{
				xOfPointsOnConic[j] = round(xOfPointsOnConic[j]);

				if ((xOfPointsOnConic[j] < 0) || (xOfPointsOnConic[j] >= image.size().width))
					continue;

				colorAPixel(bgrMat, xOfPointsOnConic[j], y, cv::Scalar(255, 255, 255), whiteDotWidth);
			}
		}

		for (int x = 0; x < image.size().width; x++)
		{
			vector<double> yOfPointsOnConic;

			double a = markers[i].C.at<double>(1, 1);
			double b = markers[i].C.at<double>(0, 1) * 2 * x + markers[i].C.at<double>(1, 2) * 2;
			double c = markers[i].C.at<double>(0, 0) * x * x + markers[i].C.at<double>(0, 2) * 2 * x + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				yOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				yOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				yOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < yOfPointsOnConic.size(); j++)
			{
				yOfPointsOnConic[j] = round(yOfPointsOnConic[j]);

				if ((yOfPointsOnConic[j] < 0) || (yOfPointsOnConic[j] >= image.size().height))
					continue;

				colorAPixel(bgrMat, x, yOfPointsOnConic[j], cv::Scalar(255, 255, 255), whiteDotWidth);
			}
		}

		for (int y = 0; y < image.size().height; y++)
		{
			vector<double> xOfPointsOnConic;

			double a = markers[i].C.at<double>(0, 0);
			double b = markers[i].C.at<double>(0, 1) * 2 * y + markers[i].C.at<double>(0, 2) * 2;
			double c = markers[i].C.at<double>(1, 1) * y * y + markers[i].C.at<double>(1, 2) * 2 * y + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				xOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				xOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				xOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < xOfPointsOnConic.size(); j++)
			{
				xOfPointsOnConic[j] = round(xOfPointsOnConic[j]);

				if ((xOfPointsOnConic[j] < 0) || (xOfPointsOnConic[j] >= image.size().width))
					continue;

				colorAPixel(bgrMat, xOfPointsOnConic[j], y, cv::Scalar(50, 255, 50), dotWidth);
			}
		}

		for (int x = 0; x < image.size().width; x++)
		{
			vector<double> yOfPointsOnConic;

			double a = markers[i].C.at<double>(1, 1);
			double b = markers[i].C.at<double>(0, 1) * 2 * x + markers[i].C.at<double>(1, 2) * 2;
			double c = markers[i].C.at<double>(0, 0) * x * x + markers[i].C.at<double>(0, 2) * 2 * x + markers[i].C.at<double>(2, 2);

			double disc = b * b - 4 * a * c;

			if (disc == 0)
				yOfPointsOnConic.push_back(-b / (2 * a));
			else if (disc > 0)
			{
				yOfPointsOnConic.push_back((-b + sqrt(disc)) / (2 * a));
				yOfPointsOnConic.push_back((-b - sqrt(disc)) / (2 * a));
			}

			for (int j = 0; j < yOfPointsOnConic.size(); j++)
			{
				yOfPointsOnConic[j] = round(yOfPointsOnConic[j]);

				if ((yOfPointsOnConic[j] < 0) || (yOfPointsOnConic[j] >= image.size().height))
					continue;

				colorAPixel(bgrMat, x, yOfPointsOnConic[j], cv::Scalar(50, 255, 50), dotWidth);
			}
		}
	}

	return bgrMat;
}

void Drawer::save(const string &path, Mat image)
{
	vector<int> compressionParams = {cv::IMWRITE_PNG_COMPRESSION, 0};
	cv::imwrite(path, image, compressionParams);
}