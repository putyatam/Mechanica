#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace mechanica {

enum class MediumModel {
    IdealGas,
    WeaklyCompressibleLiquid
};

struct Medium {
    std::string name;
    MediumModel model = MediumModel::IdealGas;

    // Gas
    double specificGasConstant = 287.05; // J/(kg K), dry air

    // Liquid
    double referenceDensity = 1000.0;    // kg/m^3
    double bulkModulus = 2.2e9;          // Pa

    // Shared
    double dynamicViscosity = 1.8e-5;    // Pa*s
    double vaporPressure = 2339.0;       // Pa
};

struct ControlVolume {
    std::size_t medium = 0;
    double volume = 0.001;               // m^3
    double mass = 0.001;                 // kg
    double temperature = 293.15;         // K
    double pressure = 101325.0;           // Pa
};

struct Portal {
    static constexpr std::size_t Environment = static_cast<std::size_t>(-1);

    std::size_t a = 0;
    std::size_t b = Environment;

    // This is geometry, not a "valve block".
    // A future topology compiler will derive effective openings from actual solids.
    double area = 0.0;                   // m^2
    double dischargeCoefficient = 0.72;
    bool open = true;

    double environmentPressure = 101325.0;
    double environmentTemperature = 293.15;
};

struct BoundaryPatch {
    std::size_t volume = 0;
    double area = 0.01;                  // m^2
    double outsidePressure = 101325.0;   // Pa

    [[nodiscard]] double force(const std::vector<ControlVolume>& volumes) const;
};

class ContinuumWorld {
public:
    std::size_t addMedium(Medium medium);
    std::size_t addVolume(ControlVolume volume);
    std::size_t addPortal(Portal portal);

    Medium& medium(std::size_t id);
    ControlVolume& volume(std::size_t id);
    Portal& portal(std::size_t id);

    const Medium& medium(std::size_t id) const;
    const ControlVolume& volume(std::size_t id) const;
    const std::vector<ControlVolume>& volumes() const { return mVolumes; }

    void setVolume(std::size_t id, double volumeM3);
    void step(double dt);

private:
    void updatePressures();
    double densityAt(const Medium& m, double pressure, double temperature) const;

    std::vector<Medium> mMedia;
    std::vector<ControlVolume> mVolumes;
    std::vector<Portal> mPortals;
};

class PressureChamberLab {
public:
    PressureChamberLab();

    void reset();
    void step(double dt);

    // Geometry / mechanics. There is deliberately no Cylinder or Pump object.
    double area = 0.0100;            // m^2
    double chamberLength = 0.30;     // m, moving wall position
    double minLength = 0.025;
    double maxLength = 0.50;
    double wallVelocity = 0.0;       // m/s
    double movingWallMass = 25.0;    // kg
    double guideDamping = 20.0;      // N*s/m

    // Opening to the environment. 0 = sealed.
    double openingArea = 0.0;        // m^2

    [[nodiscard]] double pressure() const;
    [[nodiscard]] double pressureForce() const;
    [[nodiscard]] double gasMass() const;

private:
    ContinuumWorld mWorld;
    std::size_t mAir = 0;
    std::size_t mChamber = 0;
    std::size_t mOpening = 0;
    BoundaryPatch mMovingBoundary;
};

} // namespace mechanica
