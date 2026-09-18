#include "BuildWorld.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace mechanica {

namespace {

float intervalOverlap(float amin,float amax,float bmin,float bmax) {
    return std::min(amax,bmax)-std::max(amin,bmin);
}

Vec3 aabbHalf(const Aabb& b) { return b.size()*0.5f; }

int dominantFaceAxis(const Vec3& hitPoint,const Aabb& box,float& sign) {
    const Vec3 c=box.center();
    const Vec3 h=aabbHalf(box);
    Vec3 rel=hitPoint-c;
    float scores[3]={
        std::abs(rel.x)/std::max(h.x,1e-6f),
        std::abs(rel.y)/std::max(h.y,1e-6f),
        std::abs(rel.z)/std::max(h.z,1e-6f)
    };
    int axis=0;
    if(scores[1]>scores[axis]) axis=1;
    if(scores[2]>scores[axis]) axis=2;
    const float v=axis==0?rel.x:(axis==1?rel.y:rel.z);
    sign=v>=0.0f?1.0f:-1.0f;
    return axis;
}

float getAxis(const Vec3& v,int axis) { return axis==0?v.x:(axis==1?v.y:v.z); }
void setAxis(Vec3& v,int axis,float value) {
    if(axis==0)v.x=value; else if(axis==1)v.y=value; else v.z=value;
}

} // namespace

const BlockDefinition* BuildWorld::definitionFor(const BlockInstance& instance,const BlockLibrary& library) const {
    if(instance.localOverride) return &*instance.localOverride;
    return library.find(instance.definitionId);
}

void BuildWorld::beginPlacement(std::string definitionId) {
    mPlacementDefinitionId=std::move(definitionId);
    mPlacementScale={1.0f,1.0f,1.0f};
    mPlacementRotationY=0.0f;
    clearSelection();
}

void BuildWorld::cancelPlacement() {
    mPlacementDefinitionId.clear();
}

void BuildWorld::rotatePlacementY(float degrees) {
    mPlacementRotationY=std::fmod(mPlacementRotationY+degrees,360.0f);
}

Aabb BuildWorld::worldBounds(const BlockInstance& instance,const BlockLibrary& library) const {
    const auto* def=definitionFor(instance,library);
    if(!def) {
        Aabb b; b.include(instance.transform.position-Vec3{0.25f,0.25f,0.25f});
        b.include(instance.transform.position+Vec3{0.25f,0.25f,0.25f});
        return b;
    }
    return transformAabb(BlockLibrary::localBounds(*def),transformMatrix(instance.transform));
}

bool BuildWorld::overlaps(const BlockInstance& a,const BlockInstance& b,const BlockLibrary& library) const {
    const Aabb A=worldBounds(a,library), B=worldBounds(b,library);
    constexpr float e=0.0005f;
    return intervalOverlap(A.min.x,A.max.x,B.min.x,B.max.x)>e &&
           intervalOverlap(A.min.y,A.max.y,B.min.y,B.max.y)>e &&
           intervalOverlap(A.min.z,A.max.z,B.min.z,B.max.z)>e;
}

bool BuildWorld::areTouching(const BlockInstance& a,const BlockInstance& b,const BlockLibrary& library) const {
    const Aabb A=worldBounds(a,library), B=worldBounds(b,library);
    const float eps=std::max(0.003f,mGridEnabled?mGridStep*0.03f:0.003f);

    const float ox=intervalOverlap(A.min.x,A.max.x,B.min.x,B.max.x);
    const float oy=intervalOverlap(A.min.y,A.max.y,B.min.y,B.max.y);
    const float oz=intervalOverlap(A.min.z,A.max.z,B.min.z,B.max.z);

    const bool tx=(std::abs(A.max.x-B.min.x)<=eps || std::abs(B.max.x-A.min.x)<=eps) && oy>eps && oz>eps;
    const bool ty=(std::abs(A.max.y-B.min.y)<=eps || std::abs(B.max.y-A.min.y)<=eps) && ox>eps && oz>eps;
    const bool tz=(std::abs(A.max.z-B.min.z)<=eps || std::abs(B.max.z-A.min.z)<=eps) && ox>eps && oy>eps;
    return tx||ty||tz;
}

PlacementPreview BuildWorld::preview(const Ray& ray,const BlockLibrary& library) const {
    PlacementPreview result;
    if(!placing() || !library.find(mPlacementDefinitionId)) return result;

    float nearestT=std::numeric_limits<float>::max();
    std::uint64_t nearestId=0;
    Aabb nearestBox;

    for(const auto& instance:mInstances) {
        float t=0.0f;
        const Aabb b=worldBounds(instance,library);
        if(rayAabb(ray,b,t) && t<nearestT) {
            nearestT=t; nearestId=instance.id; nearestBox=b;
        }
    }
    result.hoveredId=nearestId;

    float groundT=std::numeric_limits<float>::max();
    if(ray.direction.y<-1e-6f) {
        const float t=-ray.origin.y/ray.direction.y;
        if(t>0.0f) groundT=t;
    }

    BlockInstance candidate;
    candidate.definitionId=mPlacementDefinitionId;
    candidate.transform.scale=mPlacementScale;
    candidate.transform.rotationDeg.y=mPlacementRotationY;

    const auto* def=library.find(mPlacementDefinitionId);
    const Aabb candidateLocal=BlockLibrary::localBounds(*def);
    const Aabb candidateAtOrigin=transformAabb(candidateLocal,
        rotationEulerDeg(candidate.transform.rotationDeg)*scaling(candidate.transform.scale));
    const Vec3 candidateHalf=candidateAtOrigin.size()*0.5f;
    const Vec3 candidateLocalCenter=candidateAtOrigin.center();

    if(nearestId && nearestT<groundT) {
        const Vec3 hit=ray.origin+ray.direction*nearestT;
        float sign=1.0f;
        const int axis=dominantFaceAxis(hit,nearestBox,sign);

        Vec3 position=hit;
        if(mGridEnabled) position=snap(position,mGridStep);

        const float face=sign>0.0f?getAxis(nearestBox.max,axis):getAxis(nearestBox.min,axis);
        const float half=getAxis(candidateHalf,axis);
        const float centerOffset=getAxis(candidateLocalCenter,axis);
        setAxis(position,axis,face+sign*half-centerOffset);

        candidate.transform.position=position;
    } else if(groundT<std::numeric_limits<float>::max()) {
        Vec3 p=ray.origin+ray.direction*groundT;
        if(mGridEnabled) p=snap(p,mGridStep);
        p.y=-candidateAtOrigin.min.y;
        candidate.transform.position=p;
    } else {
        return result;
    }

    result=analyzeCandidate(candidate,library);
    result.hoveredId=nearestId;
    return result;
}

PlacementPreview BuildWorld::analyzeCandidate(const BlockInstance& candidate,const BlockLibrary& library) const {
    PlacementPreview result;
    result.candidate=candidate;
    result.hasCandidate=true;
    result.valid=true;

    for(const auto& existing:mInstances) {
        if(overlaps(candidate,existing,library)) result.valid=false;
        if(areTouching(candidate,existing,library)) result.touchingIds.push_back(existing.id);
    }

    return result;
}

std::uint64_t BuildWorld::place(
    const PlacementPreview& preview,
    const std::vector<std::uint64_t>& attachTo,
    const BlockLibrary& library
) {
    if(!preview.hasCandidate || !preview.valid) return 0;

    BlockInstance instance=preview.candidate;
    instance.id=nextId();

    for(std::uint64_t id:attachTo) {
        const auto* other=find(id);
        if(other && areTouching(instance,*other,library)) instance.attachments.insert(id);
    }

    mInstances.push_back(std::move(instance));
    BlockInstance& placed=mInstances.back();

    for(std::uint64_t id:placed.attachments) {
        if(auto* other=find(id)) other->attachments.insert(placed.id);
    }

    ++mRevision;
    ++mGeometryRevision;
    return placed.id;
}

std::uint64_t BuildWorld::importInstance(const BlockInstance& source,Vec3 positionOffset) {
    BlockInstance copy=source;
    copy.id=nextId();
    copy.transform.position+=positionOffset;
    copy.attachments.clear();
    mInstances.push_back(std::move(copy));
    ++mRevision;
    ++mGeometryRevision;
    return mInstances.back().id;
}

std::uint64_t BuildWorld::pick(const Ray& ray,const BlockLibrary& library) const {
    float nearest=std::numeric_limits<float>::max();
    std::uint64_t id=0;
    for(const auto& instance:mInstances) {
        float t=0.0f;
        if(rayAabb(ray,worldBounds(instance,library),t) && t<nearest) {
            nearest=t; id=instance.id;
        }
    }
    return id;
}

void BuildWorld::select(std::uint64_t id,bool additive) {
    if(!additive) mSelection.clear();
    if(id) {
        if(additive && mSelection.contains(id)) mSelection.erase(id);
        else mSelection.insert(id);
    }
}

void BuildWorld::clearSelection() { mSelection.clear(); }
bool BuildWorld::isSelected(std::uint64_t id) const { return mSelection.contains(id); }

BlockInstance* BuildWorld::find(std::uint64_t id) {
    for(auto& i:mInstances) if(i.id==id) return &i;
    return nullptr;
}
const BlockInstance* BuildWorld::find(std::uint64_t id) const {
    for(const auto& i:mInstances) if(i.id==id) return &i;
    return nullptr;
}

void BuildWorld::deleteSelected() {
    if(mSelection.empty()) return;

    for(auto& i:mInstances) {
        for(auto id:mSelection) i.attachments.erase(id);
    }

    std::erase_if(mInstances,[&](const BlockInstance& i){return mSelection.contains(i.id);});
    mSelection.clear();
    ++mRevision;
    ++mGeometryRevision;
}

void BuildWorld::translateSelected(Vec3 delta) {
    if(mSelection.empty()) return;
    for(auto& i:mInstances) if(mSelection.contains(i.id)) i.transform.position+=delta;
    ++mRevision;
}

void BuildWorld::rotateSelectedAround(Vec3 pivot,Vec3 deltaRotationDeg) {
    if(mSelection.empty()) return;

    for(auto& i:mInstances) {
        if(!mSelection.contains(i.id)) continue;

        const Vec3 relative=i.transform.position-pivot;
        i.transform.position=pivot+rotateVectorEulerDeg(relative,deltaRotationDeg);
        i.transform.rotationDeg+=deltaRotationDeg;
    }

    ++mRevision;
}

void BuildWorld::scaleSelectedAround(Vec3 pivot,Vec3 scaleFactor) {
    if(mSelection.empty()) return;

    scaleFactor=maxVec(scaleFactor,{0.001f,0.001f,0.001f});

    for(auto& i:mInstances) {
        if(!mSelection.contains(i.id)) continue;

        const Vec3 relative=i.transform.position-pivot;
        i.transform.position=pivot+(relative*scaleFactor);
        i.transform.scale=i.transform.scale*scaleFactor;
        i.transform.scale=maxVec(i.transform.scale,{0.02f,0.02f,0.02f});
    }

    ++mRevision;
}

void BuildWorld::setSingleSelectedPosition(Vec3 position) {
    if(mSelection.size()!=1) return;
    if(auto* i=find(*mSelection.begin())) {
        i->transform.position=position;
        ++mRevision;
    }
}

void BuildWorld::setSingleSelectedScale(Vec3 scaleValue) {
    if(mSelection.size()!=1) return;
    if(auto* i=find(*mSelection.begin())) {
        i->transform.scale=maxVec(scaleValue,{0.02f,0.02f,0.02f});
        ++mRevision;
    }
}

void BuildWorld::setSingleSelectedRotation(Vec3 rotationDeg) {
    if(mSelection.size()!=1) return;
    if(auto* i=find(*mSelection.begin())) {
        i->transform.rotationDeg=rotationDeg;
        ++mRevision;
    }
}

Vec3 BuildWorld::selectionCenter(const BlockLibrary& library) const {
    Aabb all;
    for(const auto& i:mInstances) if(mSelection.contains(i.id)) all.include(worldBounds(i,library));
    return all.valid()?all.center():Vec3{};
}

void BuildWorld::applyLocalOverride(std::uint64_t id,BlockDefinition definition,const BlockLibrary& library) {
    if(auto* i=find(id)) {
        definition.id="instance_"+std::to_string(id);
        i->localOverride=std::move(definition);
        ++mRevision;
        ++mGeometryRevision;
        pruneInvalidAttachments(library);
    }
}

void BuildWorld::resetLocalOverride(std::uint64_t id) {
    if(auto* i=find(id)) {
        i->localOverride.reset();
        ++mRevision;
        ++mGeometryRevision;
    }
}

bool BuildWorld::setInstanceMaterial(
    std::uint64_t id,
    const std::string& materialId,
    const BlockLibrary& library
) {
    auto* instance=find(id);
    if(!instance)return false;

    const BlockDefinition* source=definitionFor(*instance,library);
    if(!source)return false;

    BlockDefinition edited=*source;
    edited.id="instance_"+std::to_string(id);

    for(auto& component:edited.components)
        component.materialId=materialId;

    instance->localOverride=std::move(edited);
    ++mRevision;
    ++mGeometryRevision;
    return true;
}

std::vector<std::uint64_t> BuildWorld::touchingIds(
    std::uint64_t id,
    const BlockLibrary& library
) const {
    std::vector<std::uint64_t> result;
    const auto* source=find(id);
    if(!source)return result;

    for(const auto& other:mInstances) {
        if(other.id==id)continue;
        if(areTouching(*source,other,library))
            result.push_back(other.id);
    }

    return result;
}

bool BuildWorld::isAttached(std::uint64_t a,std::uint64_t b) const {
    const auto* first=find(a);
    return first && first->attachments.contains(b);
}

bool BuildWorld::setAttachment(
    std::uint64_t a,
    std::uint64_t b,
    bool attached,
    const BlockLibrary& library
) {
    if(a==b)return false;
    auto* first=find(a);
    auto* second=find(b);
    if(!first||!second)return false;

    if(attached) {
        if(!areTouching(*first,*second,library))return false;
        first->attachments.insert(b);
        second->attachments.insert(a);
    } else {
        first->attachments.erase(b);
        second->attachments.erase(a);
    }

    ++mRevision;
    return true;
}

void BuildWorld::pruneInvalidAttachments(const BlockLibrary& library) {
    for(auto& a:mInstances) {
        std::vector<std::uint64_t> remove;
        for(auto id:a.attachments) {
            const auto* b=find(id);
            if(!b || !areTouching(a,*b,library)) remove.push_back(id);
        }
        for(auto id:remove) a.attachments.erase(id);
    }
    // Ensure symmetry after pruning.
    for(auto& a:mInstances) {
        for(auto id:a.attachments) if(auto* b=find(id)) b->attachments.insert(a.id);
    }
}

std::vector<std::vector<std::size_t>> BuildWorld::rigidComponents(const BlockLibrary& library) const {
    (void)library;
    std::vector<std::vector<std::size_t>> groups;
    std::vector<bool> visited(mInstances.size(),false);

    for(std::size_t start=0;start<mInstances.size();++start) {
        if(visited[start]) continue;
        std::vector<std::size_t> group;
        std::queue<std::size_t> q;
        q.push(start); visited[start]=true;

        while(!q.empty()) {
            const auto current=q.front(); q.pop();
            group.push_back(current);
            for(auto attachedId:mInstances[current].attachments) {
                for(std::size_t j=0;j<mInstances.size();++j) {
                    if(!visited[j] && mInstances[j].id==attachedId) {
                        visited[j]=true; q.push(j); break;
                    }
                }
            }
        }
        groups.push_back(std::move(group));
    }
    return groups;
}

std::uint64_t BuildWorld::nextId() { return mNextId++; }

} // namespace mechanica
