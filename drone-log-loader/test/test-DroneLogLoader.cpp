#include <gtest/gtest.h>

#include <DroneLogLoader.hpp>
#include <filesystem>

using std::filesystem::path;
using namespace std::literals::string_literals;

TEST(DroneLogLoader, writeAndRead) {
    path f                = __FILE__;
    auto p                = f.parent_path();
    std::vector<LogEntry> entries(2);
    entries[0].setSize(42);
    entries[0].setMode(43);
    entries[0].setFrametime(44);
    entries[0].setDroneConfig(45);
    entries[0].setRcTuning(46);
    entries[0].setRcThrottle(47);
    entries[0].setRcRoll(48);
    
    entries[1].setSize(49);

    DroneLogLoader dllw = entries;
    dllw.write("dll");

    DroneLogLoader dllr = "dll"s;
    auto entriesr = dllr.getEntries();

    ASSERT_EQ(entriesr[0].getRcThrottle(), entries[0].getRcThrottle());

}