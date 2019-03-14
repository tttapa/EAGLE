#include <ANSIColors.hpp>
#include <DroneLogLoader.hpp>
#include <cassert>
#include <fstream>
#include <iterator>
#include <sstream>

#include <iostream>

using namespace std;

static istream &operator>>(istream &is, LogEntry &logentry) {
    char *data = reinterpret_cast<char *>(&logentry);
    for (size_t i = 0; i < sizeof(LogEntry); ++i)
        is >> (*(data++));
    return is;
}

DroneLogLoader::DroneLogLoader(std::filesystem::path loadFile) {
    if (filesystem::is_directory(loadFile)) {
        stringstream s;
        s << ANSIColors::red << "Error opening file: " << loadFile
          << ": is a directory" << ANSIColors::reset << std::endl;
        throw runtime_error(s.str());
    }
    cout << "loading file " << loadFile << endl;
    std::ifstream file(loadFile, std::ios::binary);
    if (!file) {
        stringstream s;
        s << ANSIColors::red << "Error opening file: " << loadFile
          << ANSIColors::reset << std::endl;
        throw runtime_error(s.str());
    }
    file.unsetf(std::ios::skipws);
    file.seekg(0, std::ios::end);
    auto filesize = file.tellg();
    file.seekg(0, std::ios::beg);
    assert(filesize % sizeof(LogEntry) == 0);
    cout << "creating vector" << endl;
    std::vector<LogEntry> entries;
    cout << "reserving vector, filesize = " << filesize << endl;
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

void DroneLogLoader::write(std::string filename) const {
    auto file = std::fstream(filename, std::ios::out | std::ios::binary);
    file.write((char *) &entries[0], entries.size() * sizeof(entries[0]));
    file.close();
}