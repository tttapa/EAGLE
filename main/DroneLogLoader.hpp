#pragma once

#include <Drone/DroneStateControlOutput.hpp>
#include <Matrix/Matrix.hpp>
#include <filesystem>
#include <vector>

struct DroneLogEntry {
    DroneLogEntry() {
        static_assert(sizeof(float) == 4, "Error: unsupported float size");
        static_assert(sizeof(uint32_t) == 4, "Error: unsupported uint32 size");
        static_assert(sizeof(DroneLogEntry) == 4 * 40,
                      "Error: incorrect DroneLogEntry size");
    }

    uint32_t data;                        // 0
    float frametime;                      // 1
    float tuningParameter;                // 2
    uint32_t size;                        // 3
    float referenceOrientation[4];        // 4-7
    float measurementOrientation[4];      // 8-11
    float measurementAngularVelocity[3];  // 12-14
    float attitudeControlSignals[3];      // 15-17
    float observerOrientation[4];         // 18-20
    float observerAngularVelocity[3];     // 22-24
    float observerMotorSpeeds[3];         // 25-27
    float motorControlSignals[4];         // 28-31
    float referenceHeight;                // 32
    float measurementHeight;              // 33
    float altitudeMarginalControlSignal;  // 34 = u_t
    float observerAltitudeMotorSpeed;     // 35
    float observerHeight;                 // 36
    float observerAltitudeVelocity;       // 37
    float altitudeControlSignal;          // 38 = u_h + u_t
    float yawOffset;                      // 39

    DroneAttitudeState getAttitudeState() const {
        return vcat(ColVectorFromCppArray(observerOrientation),
                    ColVectorFromCppArray(observerAngularVelocity),
                    ColVectorFromCppArray(observerMotorSpeeds));
    }

    DroneState getState() const {
        return vcat(ColVectorFromCppArray(observerOrientation),
                    ColVectorFromCppArray(observerAngularVelocity),
                    ColVectorFromCppArray(observerMotorSpeeds),
                    zeros<2, 1>(),  // TODO: xy velocity
                    ColVectorFromCppArray({observerAltitudeVelocity}),
                    zeros<2, 1>(),  // TODO: xy location
                    ColVectorFromCppArray({observerHeight}),
                    ColVectorFromCppArray({observerAltitudeMotorSpeed}));
    }

    DroneOutput getReference() const {
        return vcat(ColVectorFromCppArray(referenceOrientation),
                    zeros<3, 1>(),  // omega
                    zeros<2, 1>(),  // xy location
                    ColVectorFromCppArray({referenceHeight}));
    }

    DroneControl getControl() const {
        return vcat(ColVectorFromCppArray(attitudeControlSignals),
                    ColVectorFromCppArray({altitudeControlSignal}));
    }

} __attribute__((__packed__));

class DroneLogLoader {
  public:
    DroneLogLoader(const std::filesystem::path &loadfile) { load(loadfile); }

    void load(const std::filesystem::path &loadfile);

    const DroneLogEntry &operator[](size_t i) const { return entries[i]; }

    std::vector<double> getTimeStamps() const {
        std::vector<double> timestamps(entries.size());
        std::transform(entries.begin(), entries.end(), timestamps.begin(),
                       [](const DroneLogEntry &dle) { return dle.frametime; });
        return timestamps;
    }

    std::vector<ColVector<10>> getAttitudeStates() const {
        std::vector<ColVector<10>> states(entries.size());
        std::transform(
            entries.begin(), entries.end(), states.begin(),
            [](const DroneLogEntry &dle) { return dle.getAttitudeState(); });
        return states;
    }

    std::vector<ColVector<17>> getStates() const {
        std::vector<ColVector<17>> states(entries.size());
        std::transform(entries.begin(), entries.end(), states.begin(),
                       [](const DroneLogEntry &dle) { return dle.getState(); });
        return states;
    }

    std::vector<ColVector<10>> getReference() const {
        std::vector<ColVector<10>> refs(entries.size());
        std::transform(
            entries.begin(), entries.end(), refs.begin(),
            [](const DroneLogEntry &dle) { return dle.getReference(); });
        return refs;
    }

    std::vector<ColVector<4>> getControl() const {
        std::vector<ColVector<4>> ctrls(entries.size());
        std::transform(
            entries.begin(), entries.end(), ctrls.begin(),
            [](const DroneLogEntry &dle) { return dle.getControl(); });
        return ctrls;
    }

  private:
    std::vector<DroneLogEntry> entries;
};