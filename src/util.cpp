#include "util.hpp"

#include <algorithm>
#include <cmath>

// Statistics

double median(std::vector<double>& values) {
    size_t n = values.size();

    if (n == 0)
        return 0; // Handle empty vector case

    auto middle = values.begin() + n / 2;

    std::nth_element(values.begin(), middle, values.end());

    if (n % 2 == 1)
        return *middle; // If odd, return the middle element

    auto lower = std::max_element(values.begin(), middle); 
    // If even, get max of the lower half and average with middle

    return (*lower + *middle) / 2.0;
}

// Vectors

Vec2 add(Vec2 a, Vec2 b) {
    return {
        a.x + b.x,
        a.y + b.y
    };
}

Vec2 subtract(Vec2 a, Vec2 b) {
    return {
        a.x - b.x,
        a.y - b.y
    };
}

Vec2 multiply(Vec2 v, double s) {
    return {
        v.x * s,
        v.y * s
    };
}

double dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}

double cross(Vec2 a, Vec2 b) {
    return a.x * b.y - a.y * b.x;
}

double magnitude(Vec2 v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

Vec2 normalize(Vec2 v) {
    double mag = magnitude(v);

    // Prevent division by zero
    if (mag == 0)
        return {0, 0};

    return {
        v.x / mag,
        v.y / mag
    };
}

Vec2 rotate(Vec2 v, double angle) {
    double radians = angle * M_PI / 180.0;

    double cosA = std::cos(radians);
    double sinA = std::sin(radians);

    return {
        v.x * cosA - v.y * sinA,
        v.x * sinA + v.y * cosA
    };
}


// ====================
// Angle Conversions
// ====================

double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

double radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

// Angles

// Converts any angle to [0, 360)
double normalizeAngle(double angle) {
    angle = std::fmod(angle, 360.0);

    if (angle < 0)
        angle += 360.0;

    return angle;
}


// Returns the shortest signed difference between two angles.
// Result is in [-180, 180).
//
// Example:
// angleDifference(359, 1) = 2
// angleDifference(1, 359) = -2
double angleDifference(double a, double b) {
    return std::fmod(a - b + 540.0, 360.0) - 180.0;
}

// Units
double mmToInches(double mm) {
    return mm / 25.4;
}
