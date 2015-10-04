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
	if(!checkRobust())
	{
		return;
	}

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	float startTime = controlPoints.front().time; // start time 
	float endTime = controlPoints.back().time; // end time 
	Point prevPoint = controlPoints.front().position; // start position
	Point currentPoint;

	for (float t = startTime; t <= endTime; t += window) { // loop using the final time and incrementing time through window 
		if (t > endTime - window) { //if it's the final point
			currentPoint = controlPoints.back().position;
		}

		calculatePoint(currentPoint, t);
		DrawLib::drawLine(prevPoint, currentPoint, curveColor, curveThickness);
		prevPoint = currentPoint;
	}
	return;
#endif
}

//Compare two CurvePoints
bool compareCurvePoints(const CurvePoint point1, const CurvePoint point2) {
	return point1.time < point2.time;
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	sort(controlPoints.begin(), controlPoints.end(), compareCurvePoints);

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
	if (controlPoints.size() < 2) 
	{
		return false;
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	int index = 0;

	for (index; index < controlPoints.size(); ++index)
	{
		if (controlPoints[index].time > time)
		{
			nextPoint = index;
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
	
	CurvePoint point1 = controlPoints[nextPoint - 1];
	CurvePoint point2 = controlPoints[nextPoint];

	normalTime = time - point1.time;
	intervalTime = point2.time - point1.time;
	float s = normalTime / intervalTime;

	float h1 = 2 * pow(s, 3) - 3 * pow(s, 2) + 1;
	float h2 = -2 * pow(s, 3) + 3 * pow(s, 2);
	float h3 = pow(normalTime,3)/pow(intervalTime, 2) - 2 * pow(normalTime, 2)/intervalTime + normalTime;
	float h4 = pow(normalTime, 3) / pow(intervalTime, 2) - pow(normalTime, 2) / intervalTime;

	newPosition = h1*point1.position + h2*point2.position + h3*point1.tangent + h4*point2.tangent;

	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	CurvePoint p1 = controlPoints[nextPoint - 1];
	CurvePoint p2 = controlPoints[nextPoint];
	
	Vector tan1;
	Vector tan2;
	
	normalTime = time - p1.time;
	intervalTime = p2.time - p1.time;
	
	float s = normalTime/intervalTime;
	
	if (nextPoint == 1) {
		tan1 = 2*((p2.position-p1.position)) - ((controlPoints[nextPoint+1].position-p1.position)/2);
		tan2 = (controlPoints[nextPoint+1].position-p1.position)/2;
	} else if (nextPoint == (controlPoints.size() - 1)) {
		tan1 = (p2.position-controlPoints[nextPoint-2].position)/2;
		tan2 = tan1 - (2*(p1.position-p2.position));
	} else {
		tan1 = (p2.position-controlPoints[nextPoint-2].position)/2;
		tan2 = (controlPoints[nextPoint+1].position-p1.position)/2;
	}
	
	float h1 = 2*pow(s, 3) - 3*pow(s,2) + 1;
	float h2 = -2*pow(s,3) + 3*pow(s,2);
	float h3 = pow(s,3) - 2*pow(s,2) + s;
	float h4 = pow(s,3) - pow(s,2);
	
	newPosition = h1*p1.position + h2*p2.position + h3*tan1 + h4*tan2;
	
	return newPosition;
}
