#pragma once

#include "Weights.hpp"
#include <filesystem>
#include <ostream>

void printBest(std::ostream &os, size_t generation, const Weights &best);
void appendBestToFile(const std::filesystem::path &filename, size_t generation,
                      const Weights &best);