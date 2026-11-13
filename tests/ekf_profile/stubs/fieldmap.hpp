#pragma once
#include "bot/config.hpp"
struct Segment{Vec2 A,B;int id;};
inline std::vector<Segment> localizationMap;
inline void initializeLocalizationMap(){localizationMap={{{0,0},{144,0},0},{{144,0},{144,144},1},{{144,144},{0,144},2},{{0,144},{0,0},3}};}
