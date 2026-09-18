#pragma once

#include "BlockLibrary.hpp"
#include "BuildWorld.hpp"
#include "MaterialLibrary.hpp"
#include "Math3D.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace mechanica {

struct PartRootPose {
    bool valid=false;
    Vec3 buildOrigin{};
    Vec3 worldPosition{};
    float qx=0.0f,qy=0.0f,qz=0.0f,qw=1.0f;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&)=delete;
    Renderer& operator=(const Renderer&)=delete;

    bool initialize();
    void shutdown();

    void render(
        const BuildWorld& world,
        const BlockLibrary& library,
        const MaterialLibrary& materials,
        const Mat4& viewProjection,
        const PlacementPreview* preview,
        const std::vector<PlacementPreview>* extraPreviews,
        const std::unordered_map<std::uint64_t,bool>* attachmentChoices,
        const std::vector<PartRootPose>* simulatedPoses
    );

    std::uint32_t renderBlockPreview(
        const BlockDefinition& definition,
        const MaterialLibrary& materials,
        int width,
        int height,
        float yaw,
        float pitch,
        float zoom
    );

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};

} // namespace mechanica
