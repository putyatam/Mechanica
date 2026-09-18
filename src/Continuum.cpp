#include "Continuum.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mechanica {

namespace {
constexpr double kAmbient = 101325.0;
constexpr double kGravity = 9.80665;
constexpr double kMinimumVolume = 1.0e-8;
constexpr double kMinimumMass = 1.0e-12;
}

double BoundaryPatch::force(const std::vector<ControlVolume>& volumes) const {
    if (volume >= volumes.size()) {
        return 0.0;
    }
    return (volumes[volume].pressure - outsidePressure) * area;
}

std::size_t ContinuumWorld::addMedium(Medium medium) {
    mMedia.push_back(std::move(medium));
    return mMedia.size() - 1;
}

std::size_t ContinuumWorld::addVolume(ControlVolume volume) {
    if (volume.medium >= mMedia.size()) {
        throw std::runtime_error("ControlVolume references an invalid medium");
    }
    mVolumes.push_back(volume);
    updatePressures();
    return mVolumes.size() - 1;
}

std::size_t ContinuumWorld::addPortal(Portal portal) {
    if (portal.a >= mVolumes.size()) {
        throw std::runtime_error("Portal A references an invalid control volume");
    }
    if (portal.b != Portal::Environment && portal.b >= mVolumes.size()) {
        throw std::runtime_error("Portal B references an invalid control volume");
    }
    mPortals.push_back(portal);
    return mPortals.size() - 1;
}

Medium& ContinuumWorld::medium(std::size_t id) { return mMedia.at(id); }
ControlVolume& ContinuumWorld::volume(std::size_t id) { return mVolumes.at(id); }
Portal& ContinuumWorld::portal(std::size_t id) { return mPortals.at(id); }
const Medium& ContinuumWorld::medium(std::size_t id) const { return mMedia.at(id); }
const ControlVolume& ContinuumWorld::volume(std::size_t id) const { return mVolumes.at(id); }

void ContinuumWorld::setVolume(std::size_t id, double volumeM3) {
    mVolumes.at(id).volume = std::max(volumeM3, kMinimumVolume);
}

double ContinuumWorld::densityAt(const Medium& m, double pressure, double temperature) const {
    if (m.model == MediumModel::IdealGas) {
        const double t = std::max(temperature, 1.0);
        return std::max(pressure, 1.0) / (m.specificGasConstant * t);
    }
    return m.referenceDensity * (1.0 + (pressure - kAmbient) / m.bulkModulus);
}

void ContinuumWorld::updatePressures() {
    for (auto& v : mVolumes) {
        const auto& m = mMedia.at(v.medium);
        v.volume = std::max(v.volume, kMinimumVolume);
        v.mass = std::max(v.mass, kMinimumMass);

        if (m.model == MediumModel::IdealGas) {
            v.pressure = v.mass * m.specificGasConstant * v.temperature / v.volume;
        } else {
            const double rho = v.mass / v.volume;
            v.pressure = kAmbient + m.bulkModulus * (rho / m.referenceDensity - 1.0);
            v.pressure = std::max(v.pressure, m.vaporPressure);
        }
    }
}

void ContinuumWorld::step(double dt) {
    if (dt <= 0.0) {
        return;
    }

    updatePressures();

    // Semi-explicit lumped finite-volume flow.
    // This is intentionally a generic opening between volumes, not a hydraulic component.
    // Later the geometry compiler will generate these portals from actual openings/gaps.
    for (auto& p : mPortals) {
        if (!p.open || p.area <= 0.0 || p.a >= mVolumes.size()) {
            continue;
        }

        auto& a = mVolumes[p.a];
        const auto& med = mMedia[a.medium];

        const double pA = a.pressure;
        const double pB = (p.b == Portal::Environment)
            ? p.environmentPressure
            : mVolumes[p.b].pressure;

        const double deltaP = pA - pB;
        if (std::abs(deltaP) < 0.01) {
            continue;
        }

        const bool fromA = deltaP > 0.0;
        const double upstreamPressure = fromA ? pA : pB;
        const double upstreamTemperature = fromA
            ? a.temperature
            : ((p.b == Portal::Environment) ? p.environmentTemperature : mVolumes[p.b].temperature);

        const double rho = std::max(
            densityAt(med, upstreamPressure, upstreamTemperature),
            1.0e-6
        );

        // Orifice relation. A later gas model will replace this with choked/compressible
        // flow when the pressure ratio requires it.
        const double speed = std::sqrt(2.0 * std::abs(deltaP) / rho);
        double massFlowRate = p.dischargeCoefficient * p.area * rho * speed; // kg/s
        double dm = massFlowRate * dt;

        if (fromA) {
            dm = std::min(dm, a.mass * 0.20); // stability limiter, not a physical cap
            a.mass -= dm;
            if (p.b != Portal::Environment) {
                mVolumes[p.b].mass += dm;
            }
        } else {
            if (p.b == Portal::Environment) {
                // The external world is treated as a huge reservoir of the same medium.
                const double maxFill = rho * std::max(a.volume, kMinimumVolume) * 0.20;
                dm = std::min(dm, std::max(maxFill, 1.0e-9));
                a.mass += dm;
            } else {
                auto& b = mVolumes[p.b];
                dm = std::min(dm, b.mass * 0.20);
                b.mass -= dm;
                a.mass += dm;
            }
        }
    }

    updatePressures();
}

PressureChamberLab::PressureChamberLab() {
    Medium air;
    air.name = "Dry air";
    air.model = MediumModel::IdealGas;
    air.specificGasConstant = 287.05;
    air.dynamicViscosity = 1.81e-5;
    mAir = mWorld.addMedium(air);

    ControlVolume chamber;
    chamber.medium = mAir;
    chamber.volume = area * chamberLength;
    chamber.temperature = 293.15;
    chamber.mass = kAmbient * chamber.volume / (air.specificGasConstant * chamber.temperature);
    mChamber = mWorld.addVolume(chamber);

    Portal opening;
    opening.a = mChamber;
    opening.b = Portal::Environment;
    opening.area = openingArea;
    opening.environmentPressure = kAmbient;
    mOpening = mWorld.addPortal(opening);

    mMovingBoundary.volume = mChamber;
    mMovingBoundary.area = area;
    mMovingBoundary.outsidePressure = kAmbient;
}

void PressureChamberLab::reset() {
    chamberLength = 0.30;
    wallVelocity = 0.0;

    auto& chamber = mWorld.volume(mChamber);
    chamber.volume = area * chamberLength;
    chamber.temperature = 293.15;
    chamber.mass = kAmbient * chamber.volume
        / (mWorld.medium(mAir).specificGasConstant * chamber.temperature);

    mWorld.portal(mOpening).area = openingArea;
    mMovingBoundary.area = area;
    mWorld.step(1.0e-6);
}

void PressureChamberLab::step(double dt) {
    area = std::max(area, 1.0e-5);
    movingWallMass = std::max(movingWallMass, 0.01);
    chamberLength = std::clamp(chamberLength, minLength, maxLength);

    mMovingBoundary.area = area;
    mWorld.portal(mOpening).area = std::max(openingArea, 0.0);

    // Moving solid boundary changes the actual gas volume.
    mWorld.setVolume(mChamber, area * chamberLength);
    mWorld.step(dt);

    // Pressure is converted back into mechanical force only through a surface integral.
    // In 3D this becomes sum(p * n * dA) over wetted boundary patches.
    const double fPressure = mMovingBoundary.force(mWorld.volumes());
    const double fWeight = movingWallMass * kGravity;
    const double fDamping = guideDamping * wallVelocity;
    const double acceleration = (fPressure - fWeight - fDamping) / movingWallMass;

    wallVelocity += acceleration * dt;
    chamberLength += wallVelocity * dt;

    if (chamberLength < minLength) {
        chamberLength = minLength;
        if (wallVelocity < 0.0) wallVelocity *= -0.15;
    } else if (chamberLength > maxLength) {
        chamberLength = maxLength;
        if (wallVelocity > 0.0) wallVelocity *= -0.15;
    }

    mWorld.setVolume(mChamber, area * chamberLength);
}

double PressureChamberLab::pressure() const {
    return mWorld.volume(mChamber).pressure;
}

double PressureChamberLab::pressureForce() const {
    return mMovingBoundary.force(mWorld.volumes());
}

double PressureChamberLab::gasMass() const {
    return mWorld.volume(mChamber).mass;
}

} // namespace mechanica
