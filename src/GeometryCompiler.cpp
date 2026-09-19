#include "GeometryCompiler.hpp"

#include <manifold/manifold.h>
#include <manifold/cross_section.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <vector>

namespace mechanica {

namespace {

using manifold::Manifold;
using manifold::CrossSection;
using manifold::Polygons;
using manifold::SimplePolygon;
using manifold::vec2;
using manifold::vec3;

Polygons profilePolygons(const GeometryComponent& component) {
    SimplePolygon polygon;
    polygon.reserve(component.profile.size());

    double signedArea = 0.0;
    for(std::size_t i=0;i<component.profile.size();++i) {
        const auto& a=component.profile[i];
        const auto& b=component.profile[(i+1)%component.profile.size()];
        signedArea += static_cast<double>(a.x)*b.y-static_cast<double>(b.x)*a.y;
    }

    if(signedArea>=0.0) {
        for(const auto& p:component.profile)
            polygon.emplace_back(p.x,p.y);
    } else {
        for(auto it=component.profile.rbegin();it!=component.profile.rend();++it)
            polygon.emplace_back(it->x,it->y);
    }

    return Polygons{std::move(polygon)};
}

Manifold orientProfileSolid(Manifold solid,ProfilePlane plane,bool revolve) {
    if(revolve) {
        switch(plane) {
            case ProfilePlane::XY:
                return solid.Rotate(-90.0,0.0,0.0);
            case ProfilePlane::XZ:
                return solid;
            case ProfilePlane::YZ:
                return solid.Rotate(0.0,90.0,0.0);
        }
    } else {
        switch(plane) {
            case ProfilePlane::XY:
                return solid;
            case ProfilePlane::XZ:
                return solid.Warp([](vec3& v) {
                    v=vec3(v.x,v.z,v.y);
                });
            case ProfilePlane::YZ:
                return solid.Warp([](vec3& v) {
                    v=vec3(v.z,v.x,v.y);
                });
        }
    }
    return solid;
}

Manifold componentManifold(const GeometryComponent& c) {
    Manifold solid;

    switch(c.kind) {
        case GeometryKind::Box:
            solid=Manifold::Cube(
                vec3(
                    std::max(0.0001f,c.size.x),
                    std::max(0.0001f,c.size.y),
                    std::max(0.0001f,c.size.z)
                ),
                true
            );
            break;

        case GeometryKind::Cylinder:
            solid=Manifold::Cylinder(
                std::max(0.0001f,c.height),
                std::max(0.0001f,c.radius),
                std::max(0.0001f,c.radius),
                std::clamp(c.radialSegments,12,192),
                true
            ).Rotate(-90.0,0.0,0.0);
            break;

        case GeometryKind::Sphere:
            solid=Manifold::Sphere(
                std::max(0.0001f,c.radius),
                std::clamp(c.radialSegments,16,192)
            );
            break;

        case GeometryKind::Tube: {
            const double outer=std::max(0.0002f,c.radius);
            const double inner=std::clamp(
                static_cast<double>(c.innerRadius),
                0.0001,
                outer-0.0001
            );
            const double h=std::max(0.0001f,c.height);
            const int segments=std::clamp(c.radialSegments,16,192);

            Manifold outside=Manifold::Cylinder(h,outer,outer,segments,true);
            Manifold inside=Manifold::Cylinder(h*1.002,inner,inner,segments,true);
            solid=(outside-inside).Rotate(-90.0,0.0,0.0);
            break;
        }

        case GeometryKind::Extrude: {
            if(c.profile.size()<3)return Manifold();

            const double depth=std::max(0.0001f,c.size.z);
            CrossSection section(profilePolygons(c));
            if(section.IsEmpty())return Manifold();

            solid=Manifold::Extrude(section.ToPolygons(),depth)
                .Translate(vec3(0.0,0.0,-depth*0.5));

            solid=orientProfileSolid(std::move(solid),c.profilePlane,false);
            break;
        }

        case GeometryKind::Revolve: {
            if(c.profile.size()<3)return Manifold();

            CrossSection section(profilePolygons(c));
            if(section.IsEmpty())return Manifold();

            solid=Manifold::Revolve(
                section.ToPolygons(),
                std::clamp(c.radialSegments,16,192),
                360.0
            );
            solid=orientProfileSolid(std::move(solid),c.profilePlane,true);
            break;
        }
    }

    solid=solid.Scale(vec3(
        std::max(0.0001f,c.scale.x),
        std::max(0.0001f,c.scale.y),
        std::max(0.0001f,c.scale.z)
    ));

    solid=solid.Rotate(
        c.rotationDeg.x,
        c.rotationDeg.y,
        c.rotationDeg.z
    );

    solid=solid.Translate(vec3(
        c.position.x,
        c.position.y,
        c.position.z
    ));

    return solid;
}

Manifold applyOperation(Manifold current,const Manifold& shape,BooleanOp operation) {
    if(current.IsEmpty())return shape;

    switch(operation) {
        case BooleanOp::Add:
            return current+shape;
        case BooleanOp::Subtract:
            return current-shape;
        case BooleanOp::Intersect:
            return current^shape;
        case BooleanOp::Hull:
            return Manifold::Hull(std::vector<Manifold>{current,shape});
        case BooleanOp::MinkowskiSum:
            return current.MinkowskiSum(shape);
        case BooleanOp::MinkowskiDifference:
            return current.MinkowskiDifference(shape);
    }

    return current+shape;
}

} // namespace

CompiledGeometry compileBlockGeometry(const BlockDefinition& definition) {
    CompiledGeometry out;

    if(definition.components.empty()) {
        out.error="У блока нет геометрии.";
        return out;
    }

    try {
        std::map<int,std::vector<const GeometryComponent*>> groups;
        for(const auto& component:definition.components)
            groups[component.operationGroup].push_back(&component);

        Manifold result;
        bool hasResult=false;

        for(const auto& [groupId,components]:groups) {
            (void)groupId;

            Manifold groupResult;
            bool hasGroup=false;

            for(const GeometryComponent* componentPtr:components) {
                const auto& component=*componentPtr;
                Manifold shape=componentManifold(component);
                if(shape.IsEmpty())continue;

                if(!hasGroup) {
                    groupResult=std::move(shape);
                    hasGroup=true;
                } else {
                    groupResult=applyOperation(
                        std::move(groupResult),
                        shape,
                        component.booleanOp
                    );
                }

                if(out.materialId=="steel_s235" && !component.materialId.empty())
                    out.materialId=component.materialId;
            }

            if(!hasGroup || groupResult.IsEmpty())
                continue;

            if(!hasResult) {
                result=std::move(groupResult);
                hasResult=true;
            } else {
                result=result+groupResult;
            }
        }

        if(!hasResult || result.IsEmpty()) {
            out.error="Операции дали пустую геометрию.";
            return out;
        }

        Manifold withNormals=result.CalculateNormals(0,52.5);
        const manifold::MeshGL mesh=withNormals.GetMeshGL(0);

        if(mesh.numProp<3 || mesh.triVerts.size()<3) {
            out.error="Геометрическое ядро вернуло пустую сетку.";
            return out;
        }

        auto position=[&](std::uint32_t index) {
            const std::size_t base=static_cast<std::size_t>(index)*mesh.numProp;
            return Vec3{
                mesh.vertProperties[base+0],
                mesh.vertProperties[base+1],
                mesh.vertProperties[base+2]
            };
        };

        auto normal=[&](std::uint32_t index,Vec3 fallback) {
            if(mesh.numProp<6)return fallback;
            const std::size_t base=static_cast<std::size_t>(index)*mesh.numProp;
            return normalized(Vec3{
                mesh.vertProperties[base+3],
                mesh.vertProperties[base+4],
                mesh.vertProperties[base+5]
            });
        };

        out.vertices.reserve(mesh.triVerts.size());

        for(std::size_t i=0;i+2<mesh.triVerts.size();i+=3) {
            const auto ia=mesh.triVerts[i+0];
            const auto ib=mesh.triVerts[i+1];
            const auto ic=mesh.triVerts[i+2];

            const Vec3 a=position(ia);
            const Vec3 b=position(ib);
            const Vec3 c=position(ic);
            const Vec3 face=normalized(cross(b-a,c-a));

            out.vertices.push_back({a,normal(ia,face)});
            out.vertices.push_back({b,normal(ib,face)});
            out.vertices.push_back({c,normal(ic,face)});

            out.bounds.include(a);
            out.bounds.include(b);
            out.bounds.include(c);
        }

        out.volumeM3=result.Volume();
    } catch(const std::exception& e) {
        out.error=e.what();
    } catch(...) {
        out.error="Неизвестная ошибка геометрического ядра.";
    }

    return out;
}

} // namespace mechanica
