#pragma once

#include "Math3D.hpp"
#include "MaterialLibrary.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mechanica {

enum class GeometryKind : int {
    Box = 0,
    Cylinder = 1,
    Sphere = 2,
    Tube = 3,
    Extrude = 4,
    Revolve = 5
};

enum class BooleanOp : int {
    Add = 0,
    Subtract = 1,
    Intersect = 2,
    Hull = 3,
    MinkowskiSum = 4,
    MinkowskiDifference = 5
};

enum class ProfilePlane : int {
    XY = 0,
    XZ = 1,
    YZ = 2
};

struct GeometryComponent {
    std::uint64_t id = 0;
    std::string name = "Компонент";
    GeometryKind kind = GeometryKind::Box;
    BooleanOp booleanOp = BooleanOp::Add;
    int operationGroup = 0;

    Vec3 position{};
    Vec3 rotationDeg{};
    Vec3 scale{1.0f,1.0f,1.0f};

    Vec3 size{0.5f,0.5f,0.5f};

    float radius = 0.25f;
    float innerRadius = 0.15f;
    float height = 0.5f;
    int radialSegments = 64;

    std::vector<Vec2> profile;
    ProfilePlane profilePlane = ProfilePlane::XY;

    std::string materialId = "steel_s235";
};

struct BlockDefinition {
    std::string id;
    std::string nameRu;
    std::string group = "Базовые блоки";
    int csgResolution = 20;
    std::vector<GeometryComponent> components;
};

struct VoxelBox {
    Vec3 center{};
    Vec3 size{};
    std::string materialId;
};

class BlockLibrary {
public:
    explicit BlockLibrary(std::filesystem::path storagePath);

    bool loadOrCreateDefaults();
    bool save() const;

    [[nodiscard]] const std::vector<BlockDefinition>& blocks() const { return mBlocks; }
    [[nodiscard]] const std::vector<std::string>& groups() const { return mGroups; }

    [[nodiscard]] const BlockDefinition* find(std::string_view id) const;
    [[nodiscard]] BlockDefinition* findMutable(std::string_view id);

    void addGroup(std::string name);
    void upsert(BlockDefinition block);
    std::string makeUniqueId(std::string_view base) const;

    [[nodiscard]] std::uint64_t revision() const { return mRevision; }

    static Aabb localBounds(const BlockDefinition& def);
    static double componentVolume(const GeometryComponent& component);
    static double blockMassKg(const BlockDefinition& def, const MaterialLibrary& materials);

    static bool requiresCsg(const BlockDefinition& def);
    static std::vector<VoxelBox> compileVoxelBoxes(const BlockDefinition& def);

private:
    void createDefaults();

    std::filesystem::path mStoragePath;
    std::vector<std::string> mGroups;
    std::vector<BlockDefinition> mBlocks;
    std::uint64_t mRevision = 1;
};

std::vector<Vec2> makeRegularPolygon(int sides, float radius);

} // namespace mechanica
