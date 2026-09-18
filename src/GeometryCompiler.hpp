#pragma once

#include "BlockLibrary.hpp"
#include "Math3D.hpp"

#include <string>
#include <vector>

namespace mechanica {

struct CompiledGeometryVertex {
    Vec3 position{};
    Vec3 normal{};
};

struct CompiledGeometry {
    std::vector<CompiledGeometryVertex> vertices;
    double volumeM3 = 0.0;
    Aabb bounds{};
    std::string materialId = "steel_s235";
    std::string error;

    [[nodiscard]] bool valid() const {
        return error.empty() && !vertices.empty();
    }
};

CompiledGeometry compileBlockGeometry(const BlockDefinition& definition);

} // namespace mechanica
