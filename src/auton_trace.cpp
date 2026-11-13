#include "auton_trace.hpp"
#include "ekf.hpp"
#include "subsystems.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#ifndef AUTON_TRACE_DIRECTORY
#define AUTON_TRACE_DIRECTORY "/usd"
#endif
namespace auton_trace {
namespace {
struct Frame {
    std::uint32_t ms;
    float x, y, theta, odomX, odomY, odomTheta, left, right;
    bool ekf;
};
enum class Kind { Pose, Begin, End };
struct Entry { Kind kind = Kind::Pose; Frame frame{}; char name[80] = {}; };
std::array<Entry, QUEUE_CAPACITY> queue;
std::size_t head = 0, tail = 0, queued = 0;
pros::Mutex mutex;
Status current;
std::uint32_t began = 0, serial = 0;
FILE* output = nullptr; // Owned only by the background task.
char path[192] = {}, title[80] = {};
std::uint32_t lastFlush = 0, lastRetry = 0;
bool phaseOpen = false, fileFailed = false;
bool push(const Entry& entry) {
    if (queued >= (entry.kind == Kind::Pose ? QUEUE_CAPACITY - 2 : QUEUE_CAPACITY)) { ++current.dropped; return false; }
    queue[tail] = entry; tail = (tail + 1) % QUEUE_CAPACITY; ++queued; return true;
}
void capture() {
    mutex.take();
    if (!current.recording) { mutex.give(); return; }
    const auto estimate = ekf::getEstimate();
    const double ox = chassis.odom_x_get(), oy = chassis.odom_y_get(), ot = chassis.odom_theta_get();
    const bool useEkf = estimate.enabled && estimate.healthy && pros::millis() - estimate.updatedMs <= 200;
    const auto pose = useEkf ? estimate.pose : ekf::Pose{ox, oy, ot};
    if (std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta)) {
        Entry entry;
        entry.frame = {pros::millis() - began, static_cast<float>(pose.x), static_cast<float>(pose.y),
            static_cast<float>(pose.theta), static_cast<float>(ox), static_cast<float>(oy), static_cast<float>(ot),
            static_cast<float>(chassis.drive_sensor_left()), static_cast<float>(chassis.drive_sensor_right()), useEkf};
        if (push(entry)) ++current.samples;
    }
    mutex.give();
}
void error() { mutex.take(); current.writeError = true; mutex.give(); }
void closeFile() {
    if (!output) return;
    const bool hadError = fileFailed || std::ferror(output);
    const bool flushed = std::fflush(output) == 0;
    const bool closed = std::fclose(output) == 0;
    output = nullptr;
    mutex.take();
    if (flushed && closed && !hadError) ++current.filesSaved; else current.writeError = true;
    mutex.give();
}
void openFile() {
    if (output || !phaseOpen) return;
    if (!pros::usd::is_installed()) return;
    if (!path[0]) {
        bool found = false;
        for (unsigned attempt = 0; attempt < 1000; ++attempt) {
            std::snprintf(path, sizeof(path), "%s/match-%s-%lu-%lu.csv", AUTON_TRACE_DIRECTORY,
                title, static_cast<unsigned long>(pros::millis()), static_cast<unsigned long>(++serial));
            FILE* existing = std::fopen(path, "r");
            if (!existing) { found = true; break; }
            std::fclose(existing);
        }
        if (!found) { error(); phaseOpen = false; return; }
    }
    output = std::fopen(path, "a");
    if (!output) { error(); return; }
    fileFailed = false;
    if (std::ftell(output) == 0) {
        std::fprintf(output, "#auton_visualizer_v1\n#name,%s\n", title);
        std::fprintf(output, "t,x,y,theta,odom_x,odom_y,odom_theta,left,right,source\n");
    }
    if (std::ferror(output)) { fileFailed = true; error(); closeFile(); phaseOpen = false; return; }
    mutex.take(); std::snprintf(current.file, sizeof(current.file), "%s", path); mutex.give();
}
void process(const Entry& entry) {
    if (entry.kind == Kind::Begin) {
        closeFile(); phaseOpen = true;
        std::snprintf(title, sizeof(title), "%s", entry.name);
        path[0] = '\0';
        openFile();
    } else if (entry.kind == Kind::End) {
        closeFile(); phaseOpen = false;
    } else {
        if (!output && pros::millis() - lastRetry >= 1000) { lastRetry = pros::millis(); openFile(); }
        if (output) {
            const auto& f = entry.frame;
            if (std::fprintf(output, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n", f.ms / 1000.0,
                f.x, f.y, f.theta, f.odomX, f.odomY, f.odomTheta, f.left, f.right, f.ekf ? "ekf" : "odom") < 0) {
                fileFailed = true; error(); closeFile(); phaseOpen = false;
            }
        }
    }
}
void service() {
    capture();
    // Dequeue under a short lock; all card access occurs after releasing it.
    for (unsigned i = 0; i < 64; ++i) {
        mutex.take();
        if (!queued) { mutex.give(); break; }
        const Entry entry = queue[head]; head = (head + 1) % QUEUE_CAPACITY; --queued;
        mutex.give(); process(entry);
    }
    const auto now = pros::millis();
    if (output && now - lastFlush >= 1000) {
        if (std::fflush(output) != 0) { fileFailed = true; error(); closeFile(); phaseOpen = false; }
        lastFlush = now;
    }
    mutex.take(); current.sdAvailable = pros::usd::is_installed(); mutex.give();
}
void run(void*) {
    auto wake = pros::millis();
    while (true) { service(); if (pros::millis() - wake > 40) wake = pros::millis(); pros::Task::delay_until(&wake, 20); }
}
}
void start() {
    static pros::Task task(run, nullptr, TASK_PRIORITY_DEFAULT - 2, TASK_STACK_DEPTH_DEFAULT, "Match Position Log");
}
bool begin(const char* name) {
    end();
    Entry entry; entry.kind = Kind::Begin;
    std::size_t i = 0;
    for (; name && name[i] && name[i] != '\n' && i + 1 < sizeof(entry.name); ++i) {
        const char c = name[i];
        entry.name[i] = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ? c : '-';
    }
    if (!i) std::strcpy(entry.name, "autonomous");
    mutex.take(); const bool ok = push(entry); current.recording = ok; began = pros::millis(); mutex.give();
    if (ok) capture();
    return ok;
}
void end() {
    capture();
    mutex.take();
    if (current.recording) { Entry entry; entry.kind = Kind::End; push(entry); current.recording = false; }
    mutex.give();
}
Status status() { mutex.take(); const auto copy = current; mutex.give(); return copy; }
}
