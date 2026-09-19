#pragma once

#include "BlockLibrary.hpp"
#include "Math3D.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace mechanica {

struct BlockInstance {
    std::uint64_t id = 0;
    std::string definitionId;
    Transform transform{};
    std::optional<BlockDefinition> localOverride;
    std::unordered_set<std::uint64_t> attachments;
};

struct PlacementPreview {
    bool hasCandidate = false;
    bool valid = false;
    BlockInstance candidate{};
    std::uint64_t hoveredId = 0;
    std::vector<std::uint64_t> touchingIds;
};

class BuildWorld {
public:
    [[nodiscard]] const std::vector<BlockInstance>& instances() const { return mInstances; }
    [[nodiscard]] std::vector<BlockInstance>& instances() { return mInstances; }

    [[nodiscard]] const BlockDefinition* definitionFor(const BlockInstance& instance, const BlockLibrary& library) const;

    void beginPlacement(std::string definitionId);
    void cancelPlacement();
    [[nodiscard]] bool placing() const { return !mPlacementDefinitionId.empty(); }
    [[nodiscard]] const std::string& placementDefinitionId() const { return mPlacementDefinitionId; }

    void rotatePlacementY(float degrees=90.0f);
    void setPlacementScale(Vec3 scale) { mPlacementScale=maxVec(scale,{0.02f,0.02f,0.02f}); }
    [[nodiscard]] Vec3 placementScale() const { return mPlacementScale; }

    void setGridEnabled(bool value) { mGridEnabled=value; }
    [[nodiscard]] bool gridEnabled() const { return mGridEnabled; }

    void setGridStep(float value) { mGridStep=std::clamp(value,0.005f,5.0f); }
    [[nodiscard]] float gridStep() const { return mGridStep; }

    [[nodiscard]] PlacementPreview preview(const Ray& ray, const BlockLibrary& library) const;
    [[nodiscard]] PlacementPreview analyzeCandidate(const BlockInstance& candidate, const BlockLibrary& library) const;
    std::uint64_t place(
        const PlacementPreview& preview,
        const std::vector<std::uint64_t>& attachTo,
        const BlockLibrary& library
    );
    std::uint64_t importInstance(const BlockInstance& source, Vec3 positionOffset={});

    [[nodiscard]] std::uint64_t pick(const Ray& ray, const BlockLibrary& library) const;

    void select(std::uint64_t id, bool additive);
    void clearSelection();
    [[nodiscard]] bool isSelected(std::uint64_t id) const;
    [[nodiscard]] const std::unordered_set<std::uint64_t>& selection() const { return mSelection; }

    void deleteSelected();
    void translateSelected(Vec3 delta);
    void rotateSelectedAround(Vec3 pivot, Vec3 deltaRotationDeg);
    void scaleSelectedAround(Vec3 pivot, Vec3 scaleFactor);

    void setSingleSelectedPosition(Vec3 position);
    void setSingleSelectedScale(Vec3 scale);
    void setSingleSelectedRotation(Vec3 rotationDeg);
    [[nodiscard]] Vec3 selectionCenter(const BlockLibrary& library) const;

    BlockInstance* find(std::uint64_t id);
    const BlockInstance* find(std::uint64_t id) const;

    void applyLocalOverride(std::uint64_t id, BlockDefinition definition, const BlockLibrary& library);
    void resetLocalOverride(std::uint64_t id);
    bool setInstanceMaterial(std::uint64_t id, const std::string& materialId, const BlockLibrary& library);

    [[nodiscard]] std::vector<std::uint64_t> touchingIds(std::uint64_t id, const BlockLibrary& library) const;
    [[nodiscard]] bool isAttached(std::uint64_t a, std::uint64_t b) const;
    bool setAttachment(std::uint64_t a, std::uint64_t b, bool attached, const BlockLibrary& library);

    void pruneInvalidAttachments(const BlockLibrary& library);

    [[nodiscard]] std::vector<std::vector<std::size_t>> rigidComponents(const BlockLibrary& library) const;

    [[nodiscard]] Aabb worldBounds(const BlockInstance& instance, const BlockLibrary& library) const;
    [[nodiscard]] bool areTouching(const BlockInstance& a, const BlockInstance& b, const BlockLibrary& library) const;

    [[nodiscard]] std::uint64_t revision() const { return mRevision; }
    [[nodiscard]] std::uint64_t geometryRevision() const { return mGeometryRevision; }

private:
    bool overlaps(const BlockInstance& a, const BlockInstance& b, const BlockLibrary& library) const;
    std::uint64_t nextId();

    std::vector<BlockInstance> mInstances;
    std::unordered_set<std::uint64_t> mSelection;

    std::string mPlacementDefinitionId;
    Vec3 mPlacementScale{1.0f,1.0f,1.0f};
    float mPlacementRotationY=0.0f;

    bool mGridEnabled=true;
    float mGridStep=0.25f;

    std::uint64_t mNextId=1;
    std::uint64_t mRevision=1;
    std::uint64_t mGeometryRevision=1;
};

} // namespace mechanica
