#pragma once

#include <DroneStateControlOutput.hpp>
#include <Matrix.hpp>
#include <filesystem>
#include <logger.h>
#include <vector>

class DroneLogLoader {
  public:
    DroneLogLoader(std::filesystem::path loadFile);
    DroneLogLoader(std::string loadFile)
        : DroneLogLoader{std::filesystem::path{loadFile}} {}
    DroneLogLoader(std::vector<LogEntry> entries) : entries{entries} {}

    explicit operator bool() const { return !entries.empty(); }

    const LogEntry &operator[](size_t i) const { return entries[i]; }

    size_t size() const { return entries.size(); }
    const auto begin() const { return entries.begin(); }
    const auto end() const { return entries.end(); }

    std::vector<double> getTimeStamps() const {
        std::vector<double> timestamps(entries.size());
        std::transform(entries.begin(), entries.end(), timestamps.begin(),
                       [](const LogEntry &dle) { return dle.frametime; });
        return timestamps;
    }

    std::vector<ColVector<10>> getAttitudeStates() const {
        std::vector<ColVector<10>> states(entries.size());
        std::transform(entries.begin(), entries.end(), states.begin(),
                       [](const LogEntry &dle) {
                           return ColVectorFromCppArray(
                               dle.getAttitudeObserverState());
                       });
        return states;
    }

    std::vector<ColVector<17>> getStates() const {
        std::vector<ColVector<17>> states(entries.size());
        std::transform(
            entries.begin(), entries.end(), states.begin(),
            [](const LogEntry &dle) -> ColVector<17> {
                return vcat(
                    ColVectorFromCppArray(dle.getAttitudeObserverState()),
                    ColVectorFromCppArray(dle.getAltitudeObserverState()),
                    getBlock<2, 6, 0, 1>(ColVectorFromCppArray(
                        dle.getNavigationObserverState())));
            });
        return states;
    }

    std::vector<ColVector<10>> getReference() const {
        std::vector<ColVector<10>> refs(entries.size());
        std::transform(
            entries.begin(), entries.end(), refs.begin(),
            [](const LogEntry &dle) -> ColVector<10> {
                return vcat(
                    ColVectorFromCppArray(dle.getReferenceOrientation()),
                    zeros<3, 1>(),  // angular velocities
                    ColVector<1>{dle.getReferenceHeight()},
                    ColVectorFromCppArray(dle.getReferenceLocation()));
            });
        return refs;
    }

    std::vector<ColVector<4>> getControl() const {
        std::vector<ColVector<4>> ctrls(entries.size());
        std::transform(
            entries.begin(), entries.end(), ctrls.begin(),
            [](const LogEntry &dle) -> ColVector<4> {
                return vcat(
                    ColVectorFromCppArray(dle.getAttitudeControlSignals()),
                    ColVector<1>{dle.getAltitudeControlSignal()});
            });
        return ctrls;
    }

    const std::vector<LogEntry> &getEntries() const { return entries; }

    DroneLogLoader slice(size_t start_index, size_t end_index) {
        return {std::vector<LogEntry>{
            getEntries().begin() + start_index,
            getEntries().begin() + end_index,
        }};
    }

    std::vector<LogEntry>::const_iterator getFirstFlyingEntry() const {
        return std::find_if(
            entries.begin(), entries.end(),
            [](const LogEntry &l) { return l.getRcThrottle() > 0; });
    }

    std::vector<LogEntry>::const_reverse_iterator getFinalFlyingEntry() const {
        return std::find_if(
            entries.rbegin(), entries.rend(),
            [](const LogEntry &l) { return l.getRcThrottle() > 0; });
    }

    size_t getFirstFlyingIndex() const {
        auto it = getFirstFlyingEntry();
        return it == entries.end() ? 0 : it - entries.begin();
    }

    size_t getFinalFlyingIndex() const {
        auto it = getFinalFlyingEntry();
        return it == entries.rend() ? 0 : it - entries.rend();
    }

  private:
    std::vector<LogEntry> entries;
};