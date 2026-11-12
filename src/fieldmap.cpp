#include "fieldmap.hpp"
#include "subsystems.hpp"

#include <cmath>

// Field Map Definitions
// Bottom Left Corner is Origin (0, 0)

const std::vector<Segment> perimeterMap = {
    // Bottom
    {{0, 0}, {144, 0},
     0, FIELD_WALL},

    // Right
    {{144, 0}, {144, 144},
     1, FIELD_WALL},

    // Top
    {{144, 144}, {0, 144},
     2, FIELD_WALL},

    // Left
    {{0, 144}, {0, 0},
     3, FIELD_WALL}
};

const std::vector<Segment> gameElementMap = {};

std::vector<Segment> localizationMap;

void initializeLocalizationMap() {

    localizationMap.clear();

    for (const Segment& segment : perimeterMap) {
        localizationMap.push_back(segment);
    }

    for (const Segment& segment : gameElementMap) {
        localizationMap.push_back(segment);
    }
}