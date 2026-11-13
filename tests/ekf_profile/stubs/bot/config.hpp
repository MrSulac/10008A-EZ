#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 2048
namespace pros {
inline std::uint32_t clockMs=0;
inline std::function<void()> tick;
inline std::uint32_t millis() { return clockMs; }
inline std::uint64_t micros() { return std::uint64_t(clockMs)*1000; }
class Mutex { bool locked=false; public: bool take(std::uint32_t=0xffffffff) { if(locked) std::abort(); locked=true; return true; } bool give(){locked=false;return true;} };
class Distance { public: int value=9999; unsigned reads=0; int get(){++reads;return value;} };
class Task { public: Task(void(*)(void*),void*,int,int,const char*){} static void delay_until(std::uint32_t* wake,std::uint32_t delta){*wake+=delta;clockMs=*wake;if(tick)tick();} };
namespace usd { inline bool installed=false; inline int is_installed(){return installed;} }
namespace competition {
inline bool connected=false, disabled=false, autonomous=true;
inline bool is_connected(){return connected;} inline bool is_disabled(){return disabled;} inline bool is_autonomous(){return autonomous;}
}
}
struct Vec2{double x,y;};
struct Sensor { pros::Distance& distanceSensor; Vec2 offset; double angle; };
inline pros::Distance frontDistance,backDistance,leftDistance,rightDistance;
inline Sensor frontSensor{frontDistance,{0,5.75},0},backSensor{backDistance,{0,-5.75},180},leftSensor{leftDistance,{-5.75,0},270},rightSensor{rightDistance,{5.75,0},90};
