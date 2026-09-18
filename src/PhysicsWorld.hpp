#pragma once

#include "BlockLibrary.hpp"
#include "BuildWorld.hpp"
#include "MaterialLibrary.hpp"
#include "Renderer.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace mechanica {

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&)=delete;
    PhysicsWorld& operator=(const PhysicsWorld&)=delete;

    void step(float dt);

    void buildAssemblies(
        const BuildWorld& world,
        const BlockLibrary& library,
        const MaterialLibrary& materials
    );
    void clearAssemblies();

    void fillPartRootPoses(std::size_t instanceCount,std::vector<PartRootPose>& out) const;

    [[nodiscard]] std::size_t assemblyBodyCount() const;
    [[nodiscard]] std::size_t structuralLinkCount() const;
    [[nodiscard]] std::size_t yieldedLinkCount() const;
    [[nodiscard]] std::size_t brokenLinkCount() const;
    [[nodiscard]] double maxStressMPa() const;
    [[nodiscard]] double maxElasticStrain() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};

} // namespace mechanica
