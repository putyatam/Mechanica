#include "BlockLibrary.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace mechanica {

using nlohmann::json;

namespace {

json vec2Json(const Vec2& v) { return json::array({v.x,v.y}); }
json vec3Json(const Vec3& v) { return json::array({v.x,v.y,v.z}); }

Vec2 readVec2(const json& j, Vec2 fallback={}) {
    if(!j.is_array() || j.size()<2) return fallback;
    return {j[0].get<float>(),j[1].get<float>()};
}
Vec3 readVec3(const json& j, Vec3 fallback={}) {
    if(!j.is_array() || j.size()<3) return fallback;
    return {j[0].get<float>(),j[1].get<float>(),j[2].get<float>()};
}

const char* kindName(GeometryKind k) {
    switch(k) {
        case GeometryKind::Box: return "box";
        case GeometryKind::Cylinder: return "cylinder";
        case GeometryKind::Sphere: return "sphere";
        case GeometryKind::Tube: return "tube";
        case GeometryKind::Extrude: return "extrude";
        case GeometryKind::Revolve: return "revolve";
    }
    return "box";
}

GeometryKind parseKind(std::string_view s) {
    if(s=="cylinder") return GeometryKind::Cylinder;
    if(s=="sphere") return GeometryKind::Sphere;
    if(s=="tube") return GeometryKind::Tube;
    if(s=="extrude") return GeometryKind::Extrude;
    if(s=="revolve") return GeometryKind::Revolve;
    return GeometryKind::Box;
}

const char* booleanName(BooleanOp op) {
    switch(op) {
        case BooleanOp::Add:return "add";
        case BooleanOp::Subtract:return "subtract";
        case BooleanOp::Intersect:return "intersect";
        case BooleanOp::Hull:return "hull";
        case BooleanOp::MinkowskiSum:return "minkowski_sum";
        case BooleanOp::MinkowskiDifference:return "minkowski_difference";
    }
    return "add";
}

BooleanOp parseBoolean(std::string_view s) {
    if(s=="subtract")return BooleanOp::Subtract;
    if(s=="intersect")return BooleanOp::Intersect;
    if(s=="hull")return BooleanOp::Hull;
    if(s=="minkowski_sum")return BooleanOp::MinkowskiSum;
    if(s=="minkowski_difference")return BooleanOp::MinkowskiDifference;
    return BooleanOp::Add;
}

const char* planeName(ProfilePlane plane) {
    switch(plane) {
        case ProfilePlane::XY:return "xy";
        case ProfilePlane::XZ:return "xz";
        case ProfilePlane::YZ:return "yz";
    }
    return "xy";
}

ProfilePlane parsePlane(std::string_view s) {
    if(s=="xz")return ProfilePlane::XZ;
    if(s=="yz")return ProfilePlane::YZ;
    return ProfilePlane::XY;
}

double polygonArea(const std::vector<Vec2>& p) {
    if(p.size()<3) return 0.0;
    double a=0.0;
    for(std::size_t i=0;i<p.size();++i) {
        const auto& v=p[i];
        const auto& w=p[(i+1)%p.size()];
        a += static_cast<double>(v.x)*w.y-static_cast<double>(w.x)*v.y;
    }
    return std::abs(a)*0.5;
}

Aabb componentBounds(const GeometryComponent& c) {
    Aabb local;
    switch(c.kind) {
        case GeometryKind::Box: {
            const Vec3 h=c.size*0.5f;
            local.include({-h.x,-h.y,-h.z}); local.include({h.x,h.y,h.z});
            break;
        }
        case GeometryKind::Cylinder:
        case GeometryKind::Tube: {
            local.include({-c.radius,-c.height*0.5f,-c.radius});
            local.include({ c.radius, c.height*0.5f, c.radius});
            break;
        }
        case GeometryKind::Sphere: {
            local.include({-c.radius,-c.radius,-c.radius});
            local.include({ c.radius, c.radius, c.radius});
            break;
        }
        case GeometryKind::Extrude: {
            const float d=std::max(0.001f,c.size.z)*0.5f;
            const auto includeProfilePoint=[&](const Vec2& p,float sign) {
                switch(c.profilePlane) {
                    case ProfilePlane::XY: local.include({p.x,p.y,sign*d}); break;
                    case ProfilePlane::XZ: local.include({p.x,sign*d,p.y}); break;
                    case ProfilePlane::YZ: local.include({sign*d,p.x,p.y}); break;
                }
            };
            if(c.profile.empty()) {
                for(const auto& p:std::vector<Vec2>{{-0.25f,-0.25f},{0.25f,0.25f}}) {
                    includeProfilePoint(p,-1.0f);
                    includeProfilePoint(p,1.0f);
                }
            } else {
                for(const auto& p:c.profile) {
                    includeProfilePoint(p,-1.0f);
                    includeProfilePoint(p,1.0f);
                }
            }
            break;
        }
        case GeometryKind::Revolve: {
            float r=0.0f, amin=0.0f, amax=0.0f;
            bool first=true;
            for(const auto& p:c.profile) {
                r=std::max(r,std::abs(p.x));
                if(first){amin=amax=p.y;first=false;}
                else {amin=std::min(amin,p.y);amax=std::max(amax,p.y);}
            }
            if(first){r=0.25f;amin=-0.25f;amax=0.25f;}
            switch(c.profilePlane) {
                case ProfilePlane::XY:
                    local.include({-r,amin,-r}); local.include({r,amax,r});
                    break;
                case ProfilePlane::XZ:
                    local.include({-r,-r,amin}); local.include({r,r,amax});
                    break;
                case ProfilePlane::YZ:
                    local.include({amin,-r,-r}); local.include({amax,r,r});
                    break;
            }
            break;
        }
    }
    Transform t;
    t.position=c.position;
    t.rotationDeg=c.rotationDeg;
    t.scale=c.scale;
    return transformAabb(local,transformMatrix(t));
}

} // namespace

std::vector<Vec2> makeRegularPolygon(int sides, float radius) {
    sides=std::clamp(sides,3,128);
    radius=std::max(radius,0.001f);
    std::vector<Vec2> result;
    result.reserve(static_cast<std::size_t>(sides));
    for(int i=0;i<sides;++i) {
        const float a=2.0f*kPi*static_cast<float>(i)/static_cast<float>(sides);
        result.push_back({std::cos(a)*radius,std::sin(a)*radius});
    }
