#pragma once

#include "Math3D.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace mechanica {

struct MaterialDefinition {
    std::string id;
    std::string nameRu;

    double densityKgM3 = 1000.0;
    double youngModulusPa = 1.0e9;
    double poissonRatio = 0.30;
    double yieldStrengthPa = 1.0e7;
    double tensileStrengthPa = 2.0e7;

    double friction = 0.5;
    double restitution = 0.1;

    double thermalConductivityWmK = 1.0;
    double specificHeatJkgK = 1000.0;
    double electricalResistivityOhmM = 1.0e9;

    Vec3 color{0.7f,0.7f,0.7f};
    std::string note;
};

class MaterialLibrary {
public:
    MaterialLibrary();

    [[nodiscard]] const std::vector<MaterialDefinition>& all() const { return mMaterials; }
    [[nodiscard]] const MaterialDefinition& get(std::string_view id) const;
    [[nodiscard]] int indexOf(std::string_view id) const;

private:
    std::vector<MaterialDefinition> mMaterials;
};

} // namespace mechanica
