#include <ANSIColors.hpp>
#include <DroneLogLoader.hpp>
#include <cassert>
#include <fstream>
#include <iterator>

#include <iostream>

using namespace std;

istream& operator>>(istream &is, LogEntry &logentry) {
    char *data = reinterpret_cast<char*>(&logentry);
    for (size_t i = 0; i < sizeof(LogEntry); ++i) 
        is >> (*(data++));
    return is;
}

DroneLogLoader::DroneLogLoader(std::filesystem::path loadFile) {
    cout << "loading file" << endl;
    std::ifstream file(loadFile, std::ios::binary);
    if (!file) {
        std::cerr << ANSIColors::red << "Error opening file: `" << loadFile
                  << "`" << ANSIColors::reset << std::endl;
        return;
    }
    file.unsetf(std::ios::skipws);
    file.seekg(0, std::ios::end);
    std::streampos filesize = file.tellg();
    file.seekg(0, std::ios::beg);
    assert(filesize % sizeof(LogEntry) == 0);
    cout << "creating vector" << endl;
    std::vector<LogEntry> entries;
    cout << "reserving vector" << endl;
    entries.reserve(filesize / sizeof(LogEntry));
    cout << "reading file" << endl;
    // file.read(reinterpret_cast<char *>(entries.data()), filesize);
    entries.insert(entries.begin(), std::istream_iterator<LogEntry>(file),
                   std::istream_iterator<LogEntry>());
    cout << "read file successfully" << endl;
    cout << "moving entries" << endl;
    this->entries = entries;
    cout << "loaded file" << endl;
}