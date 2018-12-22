#include <DroneLogLoader.hpp>
#include <ANSIColors.hpp>
#include <cassert>
#include <fstream>
#include <iterator>

#include <iostream>

void DroneLogLoader::load(const std::filesystem::path &loadfile) {
    std::ifstream file(loadfile, std::ios::binary);
    if (!file) {
        std::cerr << ANSIColors::red << "Error opening file: `" << loadfile
                  << "`" << ANSIColors::reset << std::endl;
        return;
    }
    file.seekg(0, std::ios::end);
    std::streampos filesize = file.tellg();
    file.seekg(0, std::ios::beg);
    assert(filesize % sizeof(DroneLogEntry) == 0);
    entries.clear();
    entries.resize(filesize / sizeof(DroneLogEntry));
    file.read(reinterpret_cast<char *>(entries.data()), filesize);
    assert(bool(file));
}