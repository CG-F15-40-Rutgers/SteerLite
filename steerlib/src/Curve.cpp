//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return;

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	for (int i = 0; i < controlPoints[controlPoints.size() - 1].time; i += window)
	{
		Point point1, point2;

		calculatePoint(point1, i);
		calculatePoint(point2, i + window);

		DrawLib::drawLine(point1, point2, curveColor, curveThickness);
	}

	return;
#endif
}

bool compareCurvePoints(const CurvePoint& a, const CurvePoint& b)
{
	return a.time < b.time;
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	sort(controlPoints.begin(), controlPoints.end(), compareCurvePoints);
	for (int i = 0; i < controlPoints.size(); i++)
	{
		std::cout << controlPoints[i].time << " ";
	}
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;
	
	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (type == hermiteCurve)
	{
		return (controlPoints.size() >= 2);
	}
	else
	{
		return (controlPoints.size() >= 3);
	}
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for (int i = 0; i < controlPoints.size(); i++)
	{
		if ((time >= controlPoints[i].time) && (time <= controlPoints[i + 1].time))
		{
			nextPoint = i;
			return true;
		}
	}

	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	// Calculate time interval, and normal time required for later curve calculations
	normalTime = time - controlPoints[nextPoint].time;
	intervalTime = controlPoints[nextPoint + 1].time - controlPoints[nextPoint].time;

	// Calculate position at t = time on Hermite curve
	Vector a = ((-2 * (controlPoints[nextPoint + 1].position - controlPoints[nextPoint].position)) / pow(intervalTime, 3)) + ((controlPoints[nextPoint].tangent + controlPoints[nextPoint + 1].tangent) / pow(intervalTime, 2));
	Vector b = ((3 * (controlPoints[nextPoint + 1].position - controlPoints[nextPoint].position)) / pow(intervalTime, 2)) - ((2 * controlPoints[nextPoint].tangent + controlPoints[nextPoint + 1].tangent) / intervalTime);
	Vector c = controlPoints[nextPoint].tangent;
	Vector d = controlPoints[nextPoint].position.vector();

	Vector f = a * pow(normalTime, 3) + b * pow(normalTime, 2) + c * normalTime + d;
	
	newPosition = Point(f.x, f.y, f.z);

	// Return result
	return newPosition;
}

Vector Curve::calculateCatmullSlope(const unsigned int i)
{
	if (i == 0)
	{
		return (((controlPoints[i + 2].time - controlPoints[i].time) / (controlPoints[i + 2].time - controlPoints[i + 1].time)) * ((controlPoints[i + 1].position - controlPoints[i].position) / (controlPoints[i + 1].time - controlPoints[i].time))) - (((controlPoints[i + 1].time - controlPoints[i].time) / (controlPoints[i + 2].time - controlPoints[i + 1].time)) * ((controlPoints[i + 2].position - controlPoints[i].position) / (controlPoints[i + 2].time - controlPoints[i].time)));
	}
	else if (i == controlPoints.size() - 1)
	{
		return (((controlPoints[i].time - controlPoints[i - 2].time) / (controlPoints[i].time - controlPoints[i - 1].time)) * ((controlPoints[i].position - controlPoints[i - 1].position) / (controlPoints[i].time - controlPoints[i - 1].time))) - (((controlPoints[i].time - controlPoints[i - 1].time) / (controlPoints[i].time - controlPoints[i - 1].time)) * ((controlPoints[i].position - controlPoints[i - 2].position) / (controlPoints[i].time - controlPoints[i - 2].time)));
	}
	else
	{
		return (((controlPoints[i].time - controlPoints[i - 1].time) / (controlPoints[i + 1].time - controlPoints[i - 1].time)) * ((controlPoints[i + 1].position - controlPoints[i].position) / (controlPoints[i + 1].time - controlPoints[i].time))) + (((controlPoints[i + 1].time - controlPoints[i].time) / (controlPoints[i + 1].time - controlPoints[i - 1].time)) * ((controlPoints[i].position - controlPoints[i - 1].position) / (controlPoints[i].time - controlPoints[i - 1].time)));
	}
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate time interval, and normal time required for later curve calculations
	normalTime = time - controlPoints[nextPoint].time;
	intervalTime = controlPoints[nextPoint + 1].time - controlPoints[nextPoint].time;

	// Calculate position at t = time on Catmull-Rom curve
	Vector s1 = calculateCatmullSlope(nextPoint);
	Vector s2 = calculateCatmullSlope(nextPoint + 1);

	Vector a = ((-2 * (controlPoints[nextPoint + 1].position - controlPoints[nextPoint].position)) / pow(intervalTime, 3)) + ((s1 + s2) / pow(intervalTime, 2));
	Vector b = ((3 * (controlPoints[nextPoint + 1].position - controlPoints[nextPoint].position)) / pow(intervalTime, 2)) - ((2 * s1 + s2) / intervalTime);
	Vector c = s1;
	Vector d = controlPoints[nextPoint].position.vector();

	Vector f = a * pow(normalTime, 3) + b * pow(normalTime, 2) + c * normalTime + d;

	newPosition = Point(f.x, f.y, f.z);

	// Return result
	return newPosition;
}
