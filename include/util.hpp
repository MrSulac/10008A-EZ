#pragma once

#include <vector>

// 2D vector
struct Vec2 {
	double x;
	double y;
};

// Statistics
double median(std::vector<double>& values);

// Vectors
Vec2 add(Vec2 a, Vec2 b);
Vec2 subtract(Vec2 a, Vec2 b);
Vec2 multiply(Vec2 v, double s);
double dot(Vec2 a, Vec2 b);
double cross(Vec2 a, Vec2 b);
double magnitude(Vec2 v);
Vec2 normalize(Vec2 v);
Vec2 rotate(Vec2 v, double angle);

// Angle Conversions
double degToRad(double degrees);
double radToDeg(double radians);
double normalizeAngle(double angle);
double angleDifference(double a, double b);

// Unit Conversions
double mmToInches(double mm);