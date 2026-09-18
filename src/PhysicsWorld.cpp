#include "PhysicsWorld.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>

#include <algorithm>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <limits>
#include <vector>

namespace mechanica {

using namespace JPH;
using namespace JPH::literals;

namespace Layers {
static constexpr ObjectLayer NON_MOVING=0;
static constexpr ObjectLayer MOVING=1;
static constexpr ObjectLayer NUM_LAYERS=2;
}
namespace BroadPhaseLayers {
static constexpr BroadPhaseLayer NON_MOVING(0);
static constexpr BroadPhaseLayer MOVING(1);
static constexpr uint NUM_LAYERS=2;
}

class ObjectLayerPairFilterImpl final:public ObjectLayerPairFilter {
public:
    bool ShouldCollide(ObjectLayer a,ObjectLayer b) const override {
        if(a==Layers::NON_MOVING)return b==Layers::MOVING;
        return a==Layers::MOVING;
    }
};

class BPLayerInterfaceImpl final:public BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl(){mMap[Layers::NON_MOVING]=BroadPhaseLayers::NON_MOVING;mMap[Layers::MOVING]=BroadPhaseLayers::MOVING;}
    uint GetNumBroadPhaseLayers() const override{return BroadPhaseLayers::NUM_LAYERS;}
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override{JPH_ASSERT(layer<Layers::NUM_LAYERS);return mMap[layer];}
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer layer) const override {
        if(layer==BroadPhaseLayers::NON_MOVING)return "NON_MOVING";
        if(layer==BroadPhaseLayers::MOVING)return "MOVING";
        JPH_ASSERT(false);return "INVALID";
    }
#endif
private:
    BroadPhaseLayer mMap[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final:public ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(ObjectLayer a,BroadPhaseLayer b) const override {
        if(a==Layers::NON_MOVING)return b==BroadPhaseLayers::MOVING;
        return a==Layers::MOVING;
    }
};

struct InstanceBody {
    BodyID body;
    Vec3 buildOrigin{};
    std::size_t instanceIndex=0;
};

struct StructuralLink {
    FixedConstraint* constraint=nullptr;
    std::uint64_t instanceA=0;
    std::uint64_t instanceB=0;

    double contactAreaM2=0.0;
    double youngModulusPa=1.0e9;
    double yieldStrengthPa=1.0e7;
    double tensileStrengthPa=2.0e7;

    double stressPa=0.0;
    double strain=0.0;
    double damage=0.0;

    bool yielded=false;
    bool broken=false;
};

Quat eulerQuat(Vec3 deg) {
    const float rx=deg.x*kPi/180.0f,ry=deg.y*kPi/180.0f,rz=deg.z*kPi/180.0f;
    return Quat::sRotation(JPH::Vec3::sAxisZ(),rz)*
           Quat::sRotation(JPH::Vec3::sAxisY(),ry)*
           Quat::sRotation(JPH::Vec3::sAxisX(),rx);
}

Vec3 worldComponentPosition(const BlockInstance& i,const GeometryComponent& c) {
    return i.transform.position+rotateVectorEulerDeg(c.position*i.transform.scale,i.transform.rotationDeg);
}

ShapeRefC makeConvexHull(const std::vector<JPH::Vec3>& points,float density) {
    if(points.size()<4)return nullptr;
    ConvexHullShapeSettings h;
    h.SetEmbedded();
    h.mDensity=density;
    h.mMaxConvexRadius=0.0f;
    h.mPoints.reserve(static_cast<uint>(points.size()));
    for(const auto& p:points)h.mPoints.push_back(p);
    ShapeSettings::ShapeResult result=h.Create();
    if(result.HasError())return nullptr;
    return result.Get();
}

ShapeRefC fallbackBox(Vec3 half,float density) {
    BoxShapeSettings settings(
        JPH::Vec3(
            std::max(half.x,0.005f),
            std::max(half.y,0.005f),
            std::max(half.z,0.005f)
        ),
        0.0f
    );
    settings.SetEmbedded();
    settings.mDensity=density;

    ShapeSettings::ShapeResult result=settings.Create();
    if(result.HasError())return nullptr;
    return result.Get();
}

void addComponent(
    StaticCompoundShapeSettings& compound,
    Vec3 assemblyOrigin,
    const BlockInstance& instance,
    const GeometryComponent& c,
    const MaterialDefinition& mat
) {
    const Vec3 sc=maxVec(instance.transform.scale,{0.001f,0.001f,0.001f});
    const Vec3 pos=worldComponentPosition(instance,c)-assemblyOrigin;
    const Quat rot=eulerQuat(instance.transform.rotationDeg)*eulerQuat(c.rotationDeg);
    const float density=static_cast<float>(mat.densityKgM3);

    auto addAt=[&](ShapeRefC shape,Vec3 localPos,Quat localRot){
        if(shape)compound.AddShape(JPH::Vec3(localPos.x,localPos.y,localPos.z),localRot,shape);
    };
    auto addDefault=[&](ShapeRefC shape){
        addAt(shape,pos,rot);
    };

    switch(c.kind) {
        case GeometryKind::Box: {
            const Vec3 half=(c.size*sc)*0.5f;
            addDefault(fallbackBox(half,density));
            break;
        }
        case GeometryKind::Cylinder: {
            // Non-uniform X/Z scaling becomes a convex hull so that an ellipse is represented physically.
            const float rx=c.radius*sc.x,rz=c.radius*sc.z,hh=c.height*sc.y*0.5f;
            if(std::abs(rx-rz)<1e-4f) {
                CylinderShapeSettings settings(
                    std::max(hh,0.005f),
                    std::max(rx,0.005f),
                    0.0f
                );
                settings.SetEmbedded();
                settings.mDensity=density;
                ShapeSettings::ShapeResult result=settings.Create();
                if(!result.HasError())addDefault(result.Get());
            } else {
                std::vector<JPH::Vec3> pts;
                const int n=std::clamp(c.radialSegments,12,48);
                for(int ring:{-1,1})for(int k=0;k<n;++k){
                    const float a=2*kPi*k/n;
                    pts.emplace_back(std::cos(a)*rx,ring*hh,std::sin(a)*rz);
                }
                addDefault(makeConvexHull(pts,density));
            }
            break;
        }
        case GeometryKind::Sphere: {
            const float r=c.radius;
            if(std::abs(sc.x-sc.y)<1e-4f&&std::abs(sc.x-sc.z)<1e-4f) {
                SphereShapeSettings settings(std::max(r*sc.x,0.005f));
                settings.SetEmbedded();
                settings.mDensity=density;
                ShapeSettings::ShapeResult result=settings.Create();
                if(!result.HasError())addDefault(result.Get());
            } else {
                std::vector<JPH::Vec3> pts;
                const int slices=20,stacks=10;
                for(int y=0;y<=stacks;++y){
                    const float v=-kPi*0.5f+kPi*y/stacks;
                    for(int x=0;x<slices;++x){
                        const float u=2*kPi*x/slices,cv=std::cos(v);
                        pts.emplace_back(r*sc.x*cv*std::cos(u),r*sc.y*std::sin(v),r*sc.z*cv*std::sin(u));
                    }
                }
                addDefault(makeConvexHull(pts,density));
            }
            break;
        }
        case GeometryKind::Tube: {
            // A hollow tube is represented as a ring of convex boxes, preserving the empty center.
            const int n=std::clamp(c.radialSegments,12,48);
            const float ro=c.radius,ri=std::clamp(c.innerRadius,0.001f,ro-0.001f);
            const float mid=(ro+ri)*0.5f;
            const float radial=(ro-ri);
            const float tangential=2.0f*mid*std::tan(kPi/static_cast<float>(n))*0.98f;
            for(int k=0;k<n;++k){
                const float a=2*kPi*k/n;
                const Vec3 local{std::cos(a)*mid*sc.x,0.0f,std::sin(a)*mid*sc.z};
                const Vec3 rotated=rotateVectorEulerDeg(local,instance.transform.rotationDeg+c.rotationDeg);
                const Vec3 childPos=pos+rotated;
                const Vec3 half{radial*sc.x*0.5f,c.height*sc.y*0.5f,tangential*sc.z*0.5f};
                ShapeRefC sh=fallbackBox(half,density);
                const Quat q=rot*Quat::sRotation(JPH::Vec3::sAxisY(),-a);
                addAt(sh,childPos,q);
            }
            break;
        }
        case GeometryKind::Extrude: {
            std::vector<JPH::Vec3> pts;
            const float z=c.size.z*sc.z*0.5f;
            for(const auto& p:c.profile) {
                pts.emplace_back(p.x*sc.x,p.y*sc.y,-z);
                pts.emplace_back(p.x*sc.x,p.y*sc.y, z);
            }
            ShapeRefC hull=makeConvexHull(pts,density);
            if(!hull) {
                BlockDefinition temporary;
                temporary.id="tmp";
                temporary.nameRu="tmp";
                temporary.components.push_back(c);
                Aabb b=BlockLibrary::localBounds(temporary);
                addDefault(fallbackBox((b.size()*sc)*0.5f,density));
            } else addDefault(hull);
            break;
        }
        case GeometryKind::Revolve: {
            std::vector<JPH::Vec3> pts;
            const int n=std::clamp(c.radialSegments,12,48);
            for(const auto& p:c.profile)for(int k=0;k<n;++k){
                const float a=2*kPi*k/n;
                pts.emplace_back(p.x*sc.x*std::cos(a),p.y*sc.y,p.x*sc.z*std::sin(a));
            }
            ShapeRefC hull=makeConvexHull(pts,density);
            if(!hull) addDefault(fallbackBox({0.25f,0.25f,0.25f},density));
            else addDefault(hull);
            break;
        }
    }
}


struct StructuralMaterialSummary {
    double youngPa=std::numeric_limits<double>::max();
    double yieldPa=std::numeric_limits<double>::max();
    double tensilePa=std::numeric_limits<double>::max();
};

StructuralMaterialSummary summarizeStructure(
    const BlockDefinition& def,
    const MaterialLibrary& materials
) {
    StructuralMaterialSummary result;
    bool any=false;

    for(const auto& c:def.components) {
        if(c.booleanOp==BooleanOp::Subtract)continue;

        const auto& mat=materials.get(c.materialId);
        result.youngPa=std::min(result.youngPa,mat.youngModulusPa);
        result.yieldPa=std::min(result.yieldPa,mat.yieldStrengthPa);
        result.tensilePa=std::min(result.tensilePa,mat.tensileStrengthPa);
        any=true;
    }

    if(!any) {
        result.youngPa=1.0e9;
        result.yieldPa=1.0e7;
        result.tensilePa=2.0e7;
    }

    return result;
}

double contactArea(
    const BlockInstance& a,
    const BlockInstance& b,
    const BuildWorld& world,
    const BlockLibrary& library
) {
    const Aabb A=world.worldBounds(a,library);
    const Aabb B=world.worldBounds(b,library);

    const double ox=std::max(
        0.0,
        static_cast<double>(std::min(A.max.x,B.max.x)-std::max(A.min.x,B.min.x))
    );
    const double oy=std::max(
        0.0,
        static_cast<double>(std::min(A.max.y,B.max.y)-std::max(A.min.y,B.min.y))
    );
    const double oz=std::max(
        0.0,
        static_cast<double>(std::min(A.max.z,B.max.z)-std::max(A.min.z,B.min.z))
    );

    const double eps=0.02;

    double area=0.0;
    if(std::abs(static_cast<double>(A.max.x-B.min.x))<eps ||
       std::abs(static_cast<double>(B.max.x-A.min.x))<eps)
        area=std::max(area,oy*oz);

    if(std::abs(static_cast<double>(A.max.y-B.min.y))<eps ||
       std::abs(static_cast<double>(B.max.y-A.min.y))<eps)
        area=std::max(area,ox*oz);

    if(std::abs(static_cast<double>(A.max.z-B.min.z))<eps ||
       std::abs(static_cast<double>(B.max.z-A.min.z))<eps)
        area=std::max(area,ox*oy);

    // Rotated AABB contacts can under-estimate the patch. Keep a tiny but finite
    // fallback instead of producing infinite stress from numerical zero.
    return std::max(area,1.0e-5);
}

ShapeRefC buildInstanceShape(
    const BlockInstance& inst,
    const BlockDefinition& def,
    const MaterialLibrary& materials,
    double& totalMass,
    double& weightedFriction,
    double& weightedRestitution
) {
    StaticCompoundShapeSettings compound;
    compound.SetEmbedded();

    totalMass=0.0;
    weightedFriction=0.0;
    weightedRestitution=0.0;

    const double scaleVol=std::abs(
        static_cast<double>(inst.transform.scale.x)*
        static_cast<double>(inst.transform.scale.y)*
        static_cast<double>(inst.transform.scale.z)
    );

    if(BlockLibrary::requiresCsg(def)) {
        const auto boxes=BlockLibrary::compileVoxelBoxes(def);
        const Quat rotation=eulerQuat(inst.transform.rotationDeg);

        for(const auto& b:boxes) {
            const auto& mat=materials.get(b.materialId);

            const Vec3 localCenter=b.center*inst.transform.scale;
            const Vec3 relative=
                rotateVectorEulerDeg(localCenter,inst.transform.rotationDeg);

            const Vec3 half=(b.size*inst.transform.scale)*0.5f;
            ShapeRefC shape=fallbackBox(
                {std::abs(half.x),std::abs(half.y),std::abs(half.z)},
                static_cast<float>(mat.densityKgM3)
            );

            if(shape)
                compound.AddShape(
                    JPH::Vec3(relative.x,relative.y,relative.z),
                    rotation,
                    shape
                );

            const double mass=
                static_cast<double>(b.size.x)*
                static_cast<double>(b.size.y)*
                static_cast<double>(b.size.z)*
                scaleVol*
                mat.densityKgM3;

            totalMass+=mass;
            weightedFriction+=mass*mat.friction;
            weightedRestitution+=mass*mat.restitution;
        }
    }
    else {
        for(const auto& c:def.components) {
            const auto& mat=materials.get(c.materialId);

            // addComponent expects an assembly origin in world coordinates.
            // Using the instance position makes all child shapes local to the body.
            addComponent(
                compound,
                inst.transform.position,
                inst,
                c,
                mat
            );

            const double mass=
                BlockLibrary::componentVolume(c)*
                scaleVol*
                mat.densityKgM3;

            totalMass+=mass;
            weightedFriction+=mass*mat.friction;
            weightedRestitution+=mass*mat.restitution;
        }
    }

    ShapeSettings::ShapeResult shapeResult=compound.Create();
    if(shapeResult.HasError())return nullptr;
    return shapeResult.Get();
}

struct PhysicsWorld::Impl {
    TempAllocatorImpl tempAllocator{64*1024*1024};
    JobSystemThreadPool jobSystem;
    BPLayerInterfaceImpl broadPhase;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroad;
    ObjectLayerPairFilterImpl objectVsObject;
    PhysicsSystem physics;
    BodyID floorID;
    std::vector<InstanceBody> assemblies;
    std::vector<StructuralLink> structuralLinks;

    std::size_t yieldedLinks=0;
    std::size_t brokenLinks=0;
    double maxStress=0.0;
    double maxStrain=0.0;

    Impl():jobSystem(cMaxPhysicsJobs,cMaxPhysicsBarriers,
        static_cast<int>(std::max(1u,std::thread::hardware_concurrency()>1?std::thread::hardware_concurrency()-1:1u))) {
        physics.Init(65536,0,65536,32768,broadPhase,objectVsBroad,objectVsObject);
        BodyCreationSettings floor(
            new BoxShape(JPH::Vec3(40.0f,0.5f,40.0f)),
            RVec3(0.0_r,-0.5_r,0.0_r),Quat::sIdentity(),
            EMotionType::Static,Layers::NON_MOVING
        );
        floor.mFriction=0.70f;
        floorID=physics.GetBodyInterface().CreateAndAddBody(floor,EActivation::DontActivate);
    }
};

PhysicsWorld::PhysicsWorld() {
    RegisterDefaultAllocator();
    Factory::sInstance=new Factory();
    RegisterTypes();
    m=std::make_unique<Impl>();
}
PhysicsWorld::~PhysicsWorld() {
    if(m){
        clearAssemblies();
        auto& bi=m->physics.GetBodyInterface();
        if(!m->floorID.IsInvalid()){bi.RemoveBody(m->floorID);bi.DestroyBody(m->floorID);}
        m.reset();
    }
    UnregisterTypes();delete Factory::sInstance;Factory::sInstance=nullptr;
}

void PhysicsWorld::step(float dt) {
    m->physics.Update(dt,1,&m->tempAllocator,&m->jobSystem);

    m->yieldedLinks=0;
    m->brokenLinks=0;
    m->maxStress=0.0;
    m->maxStrain=0.0;

    if(dt<=0.0f)return;

    for(auto& link:m->structuralLinks) {
        if(!link.constraint)continue;

        if(link.broken) {
            ++m->brokenLinks;
            continue;
        }

        const double linearImpulse=
            static_cast<double>(link.constraint->GetTotalLambdaPosition().Length());

        const double angularImpulse=
            static_cast<double>(link.constraint->GetTotalLambdaRotation().Length());

        const double force=linearImpulse/static_cast<double>(dt);
        const double moment=angularImpulse/static_cast<double>(dt);

        const double area=std::max(link.contactAreaM2,1.0e-8);
        const double side=std::sqrt(area);

        // Approximate section modulus as a square contact patch.
        const double sectionModulus=
            std::max(side*side*side/6.0,1.0e-10);

        const double normalStress=force/area;
        const double bendingStress=moment/sectionModulus;

        link.stressPa=normalStress+bendingStress;
        link.strain=
            link.stressPa/
            std::max(link.youngModulusPa,1.0);

        m->maxStress=std::max(m->maxStress,link.stressPa);
        m->maxStrain=std::max(m->maxStrain,link.strain);

        if(link.stressPa>link.yieldStrengthPa) {
            link.yielded=true;
            ++m->yieldedLinks;

            // Plastic damage accumulates only after yield.
            const double overload=
                link.stressPa/
                std::max(link.yieldStrengthPa,1.0)-1.0;

            link.damage+=
                std::max(0.0,overload)*
                static_cast<double>(dt)*
                0.8;
        }

        if(
            link.stressPa>=link.tensileStrengthPa ||
            link.damage>=1.0
        ) {
            link.constraint->SetEnabled(false);
            link.broken=true;
            ++m->brokenLinks;
        }
    }
}

void PhysicsWorld::buildAssemblies(
    const BuildWorld& world,
    const BlockLibrary& library,
    const MaterialLibrary& materials
) {
    clearAssemblies();

    auto& bodyInterface=m->physics.GetBodyInterface();

    std::unordered_map<std::uint64_t,Body*> bodyByInstanceId;
    bodyByInstanceId.reserve(world.instances().size());

    // One physical body per placed block definition. A block may itself contain
    // many geometry components / CSG boxes, but those stay a compound shape.
    for(std::size_t index=0;index<world.instances().size();++index) {
        const auto& inst=world.instances()[index];
        const auto* def=world.definitionFor(inst,library);
        if(!def)continue;

        double totalMass=0.0;
        double weightedFriction=0.0;
        double weightedRestitution=0.0;

        ShapeRefC shape=buildInstanceShape(
            inst,
            *def,
            materials,
            totalMass,
            weightedFriction,
            weightedRestitution
        );

        if(!shape)continue;

        BodyCreationSettings settings(
            shape,
            RVec3(
                inst.transform.position.x,
                inst.transform.position.y,
                inst.transform.position.z
            ),
            Quat::sIdentity(),
            EMotionType::Dynamic,
            Layers::MOVING
        );

        if(totalMass>1.0e-6) {
            settings.mOverrideMassProperties=
                EOverrideMassProperties::CalculateInertia;

            settings.mMassPropertiesOverride.mMass=
                static_cast<float>(totalMass);

            settings.mFriction=
                static_cast<float>(weightedFriction/totalMass);

            settings.mRestitution=
                static_cast<float>(weightedRestitution/totalMass);
        }

        Body* body=bodyInterface.CreateBody(settings);
        if(!body)continue;

        bodyInterface.AddBody(body->GetID(),EActivation::Activate);

        bodyByInstanceId.emplace(inst.id,body);
        m->assemblies.push_back({
            body->GetID(),
            inst.transform.position,
            index
        });
    }

    // Explicit user attachments become measurable fixed constraints.
    // This lets the material model determine yielding and fracture from solver impulses.
    for(const auto& a:world.instances()) {
        auto bodyAIt=bodyByInstanceId.find(a.id);
        if(bodyAIt==bodyByInstanceId.end())continue;

        const auto* defA=world.definitionFor(a,library);
        if(!defA)continue;

        const auto propsA=summarizeStructure(*defA,materials);

        for(std::uint64_t otherId:a.attachments) {
            if(otherId<=a.id)continue; // each undirected edge only once

            const auto* b=world.find(otherId);
            if(!b)continue;

            auto bodyBIt=bodyByInstanceId.find(otherId);
            if(bodyBIt==bodyByInstanceId.end())continue;

            const auto* defB=world.definitionFor(*b,library);
            if(!defB)continue;

            const auto propsB=summarizeStructure(*defB,materials);

            FixedConstraintSettings fixed;
            fixed.mAutoDetectPoint=true;

            FixedConstraint* constraint=
                static_cast<FixedConstraint*>(
                    fixed.Create(
                        *bodyAIt->second,
                        *bodyBIt->second
                    )
                );

            if(!constraint)continue;

            m->physics.AddConstraint(constraint);

            StructuralLink link;
            link.constraint=constraint;
            link.instanceA=a.id;
            link.instanceB=otherId;
            link.contactAreaM2=contactArea(a,*b,world,library);
            link.youngModulusPa=
                std::min(propsA.youngPa,propsB.youngPa);
            link.yieldStrengthPa=
                std::min(propsA.yieldPa,propsB.yieldPa);
            link.tensileStrengthPa=
                std::min(propsA.tensilePa,propsB.tensilePa);

            m->structuralLinks.push_back(link);
        }
    }
}

void PhysicsWorld::clearAssemblies() {
    if(!m)return;

    for(auto& link:m->structuralLinks) {
        if(link.constraint)
            m->physics.RemoveConstraint(link.constraint);
    }
    m->structuralLinks.clear();

    auto& bi=m->physics.GetBodyInterface();
    for(const auto& a:m->assemblies) {
        if(!a.body.IsInvalid()) {
            bi.RemoveBody(a.body);
            bi.DestroyBody(a.body);
        }
    }

    m->assemblies.clear();
    m->yieldedLinks=0;
    m->brokenLinks=0;
    m->maxStress=0.0;
    m->maxStrain=0.0;
}

void PhysicsWorld::fillPartRootPoses(
    std::size_t count,
    std::vector<PartRootPose>& out
) const {
    out.assign(count,PartRootPose{});

    const auto& bi=m->physics.GetBodyInterface();

    for(const auto& a:m->assemblies) {
        if(a.instanceIndex>=out.size())continue;

        const RVec3 p=bi.GetPosition(a.body);
        const Quat q=bi.GetRotation(a.body);

        PartRootPose pose;
        pose.valid=true;
        pose.buildOrigin=a.buildOrigin;
        pose.worldPosition={
            static_cast<float>(p.GetX()),
            static_cast<float>(p.GetY()),
            static_cast<float>(p.GetZ())
        };
        pose.qx=q.GetX();
        pose.qy=q.GetY();
        pose.qz=q.GetZ();
        pose.qw=q.GetW();

        out[a.instanceIndex]=pose;
    }
}

std::size_t PhysicsWorld::assemblyBodyCount() const {
    return m->assemblies.size();
}

std::size_t PhysicsWorld::structuralLinkCount() const {
    return m->structuralLinks.size();
}

std::size_t PhysicsWorld::yieldedLinkCount() const {
    return m->yieldedLinks;
}

std::size_t PhysicsWorld::brokenLinkCount() const {
    return m->brokenLinks;
}

double PhysicsWorld::maxStressMPa() const {
    return m->maxStress/1.0e6;
}

double PhysicsWorld::maxElasticStrain() const {
    return m->maxStrain;
}

} // namespace mechanica
