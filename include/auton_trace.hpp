#pragma once
#include <cstddef>
#include <cstdint>
// Controller-independent match position recording. SD I/O is background-only.
namespace auton_trace {
constexpr std::size_t QUEUE_CAPACITY = 512; // About ten seconds of queued poses.
struct Status {
    bool recording = false, sdAvailable = false, writeError = false;
    std::uint32_t samples = 0, dropped = 0, filesSaved = 0;
    char file[192] = {};
};
void start();
bool begin(const char* name); // Starts a new phase; closes the prior phase.
void end();
Status status();
}
