#pragma once

#include "bot/config.hpp"
#include <vector>

enum SegmentType {
    FIELD_WALL,
    GAME_ELEMENT,
    OTHER
};

struct Segment {
    Vec2 A;
    Vec2 B;

    int id;
    SegmentType type;
};

extern std::vector<Segment> localizationMap;
void initializeLocalizationMap();