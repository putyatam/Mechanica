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
    return result;
}

BlockLibrary::BlockLibrary(std::filesystem::path storagePath)
    : mStoragePath(std::move(storagePath)) {}

bool BlockLibrary::loadOrCreateDefaults() {
    if(!std::filesystem::exists(mStoragePath)) {
        createDefaults();
        return save();
    }

    try {
        std::ifstream in(mStoragePath);
        if(!in) return false;
        json root; in>>root;

        mGroups.clear();
        mBlocks.clear();

        for(const auto& g:root.value("groups",json::array()))
            if(g.is_string()) mGroups.push_back(g.get<std::string>());

        for(const auto& jb:root.value("blocks",json::array())) {
            BlockDefinition b;
            b.id=jb.value("id","");
            b.nameRu=jb.value("nameRu","Без имени");
            b.group=jb.value("group","Базовые блоки");
            b.csgResolution=std::clamp(jb.value("csgResolution",20),12,32);

            for(const auto& jc:jb.value("components",json::array())) {
                GeometryComponent c;
                c.id=jc.value("id",0ull);
                c.name=jc.value("name","Компонент");
                c.kind=parseKind(jc.value("kind","box"));
                c.booleanOp=parseBoolean(jc.value("booleanOp","add"));
                c.operationGroup=jc.value("operationGroup",0);
                c.position=readVec3(jc.value("position",json::array()));
                c.rotationDeg=readVec3(jc.value("rotationDeg",json::array()));
                c.scale=readVec3(jc.value("scale",json::array({1.0,1.0,1.0})),{1.0f,1.0f,1.0f});
                c.size=readVec3(jc.value("size",json::array({0.5,0.5,0.5})),{0.5f,0.5f,0.5f});
                c.radius=jc.value("radius",0.25f);
                c.innerRadius=jc.value("innerRadius",0.15f);
                c.height=jc.value("height",0.5f);
                c.radialSegments=jc.value("radialSegments",24);
                c.materialId=jc.value("materialId","steel_s235");
                c.profilePlane=parsePlane(jc.value("profilePlane","xy"));
                for(const auto& p:jc.value("profile",json::array())) c.profile.push_back(readVec2(p));
                b.components.push_back(std::move(c));
            }

            if(!b.id.empty() && !b.components.empty()) mBlocks.push_back(std::move(b));
        }

        if(mGroups.empty()) mGroups.push_back("Базовые блоки");
        if(mBlocks.empty()) createDefaults();
        ++mRevision;
        return true;
    } catch(...) {
        createDefaults();
        return save();
    }
}

bool BlockLibrary::save() const {
    try {
        std::filesystem::create_directories(mStoragePath.parent_path());

        json root;
        root["version"]=1;
        root["groups"]=mGroups;
        root["blocks"]=json::array();

        for(const auto& b:mBlocks) {
            json jb;
            jb["id"]=b.id;
            jb["nameRu"]=b.nameRu;
            jb["group"]=b.group;
            jb["csgResolution"]=b.csgResolution;
            jb["components"]=json::array();

            for(const auto& c:b.components) {
                json jc;
                jc["id"]=c.id;
                jc["name"]=c.name;
                jc["kind"]=kindName(c.kind);
                jc["booleanOp"]=booleanName(c.booleanOp);
                jc["operationGroup"]=c.operationGroup;
                jc["position"]=vec3Json(c.position);
                jc["rotationDeg"]=vec3Json(c.rotationDeg);
                jc["scale"]=vec3Json(c.scale);
                jc["size"]=vec3Json(c.size);
                jc["radius"]=c.radius;
                jc["innerRadius"]=c.innerRadius;
                jc["height"]=c.height;
                jc["radialSegments"]=c.radialSegments;
                jc["materialId"]=c.materialId;
                jc["profilePlane"]=planeName(c.profilePlane);
                jc["profile"]=json::array();
                for(const auto& p:c.profile) jc["profile"].push_back(vec2Json(p));
                jb["components"].push_back(std::move(jc));
            }
            root["blocks"].push_back(std::move(jb));
        }

        std::ofstream out(mStoragePath);
        if(!out) return false;
        out<<std::setw(2)<<root<<"\n";
        return true;
    } catch(...) {
        return false;
    }
}

const BlockDefinition* BlockLibrary::find(std::string_view id) const {
    for(const auto& b:mBlocks) if(b.id==id) return &b;
    return nullptr;
}
BlockDefinition* BlockLibrary::findMutable(std::string_view id) {
    for(auto& b:mBlocks) if(b.id==id) return &b;
    return nullptr;
}

void BlockLibrary::addGroup(std::string name) {
    if(name.empty()) return;
    if(std::find(mGroups.begin(),mGroups.end(),name)==mGroups.end()) {
        mGroups.push_back(std::move(name));
        ++mRevision;
        save();
    }
}

void BlockLibrary::upsert(BlockDefinition block) {
    if(block.id.empty()) block.id=makeUniqueId(block.nameRu);
    if(block.group.empty()) block.group="Базовые блоки";
    addGroup(block.group);

    if(auto* existing=findMutable(block.id)) *existing=std::move(block);
    else mBlocks.push_back(std::move(block));

    ++mRevision;
    save();
}

std::string BlockLibrary::makeUniqueId(std::string_view base) const {
    std::string cleaned;
    for(unsigned char ch:std::string(base)) {
        if((ch>='a'&&ch<='z')||(ch>='A'&&ch<='Z')||(ch>='0'&&ch<='9')) cleaned.push_back(static_cast<char>(std::tolower(ch)));
        else if(ch==' '||ch=='-'||ch=='_') cleaned.push_back('_');
    }
    if(cleaned.empty()) cleaned="user_block";

    std::string candidate=cleaned;
    int suffix=2;
    while(find(candidate)) candidate=cleaned+"_"+std::to_string(suffix++);
    return candidate;
}

Aabb BlockLibrary::localBounds(const BlockDefinition& def) {
    Aabb result;
    for(const auto& c:def.components) result.include(componentBounds(c));
    if(!result.valid()) {
        result.include({-0.25f,-0.25f,-0.25f});
        result.include({0.25f,0.25f,0.25f});
    }
    return result;
}

double BlockLibrary::componentVolume(const GeometryComponent& c) {
    const double scaleVolume=
        std::abs(static_cast<double>(c.scale.x)*
                 static_cast<double>(c.scale.y)*
                 static_cast<double>(c.scale.z));
    double baseVolume=0.0;
    switch(c.kind) {
        case GeometryKind::Box:
            baseVolume=std::max(0.0f,c.size.x)*std::max(0.0f,c.size.y)*std::max(0.0f,c.size.z);
            break;
        case GeometryKind::Cylinder:
            baseVolume=kPi*c.radius*c.radius*std::max(0.0f,c.height);
            break;
        case GeometryKind::Sphere:
            baseVolume=4.0/3.0*kPi*c.radius*c.radius*c.radius;
            break;
        case GeometryKind::Tube:
            baseVolume=kPi*std::max(0.0f,c.radius*c.radius-c.innerRadius*c.innerRadius)*std::max(0.0f,c.height);
            break;
        case GeometryKind::Extrude:
            baseVolume=polygonArea(c.profile)*std::max(0.0f,c.size.z);
            break;
        case GeometryKind::Revolve: {
            if(c.profile.size()<2) return 0.0;
            double volume=0.0;
            for(std::size_t i=0;i+1<c.profile.size();++i) {
                const double r1=std::abs(c.profile[i].x);
                const double r2=std::abs(c.profile[i+1].x);
                const double h=std::abs(c.profile[i+1].y-c.profile[i].y);
                volume += kPi*h*(r1*r1+r1*r2+r2*r2)/3.0;
            }
            baseVolume=volume;
            break;
        }
    }
    return baseVolume*scaleVolume;
}

double BlockLibrary::blockMassKg(const BlockDefinition& def, const MaterialLibrary& materials) {
    double mass=0.0;

    if(requiresCsg(def)) {
        for(const auto& b:compileVoxelBoxes(def)) {
            const double volume=
                static_cast<double>(b.size.x)*
                static_cast<double>(b.size.y)*
                static_cast<double>(b.size.z);
            mass+=volume*materials.get(b.materialId).densityKgM3;
        }
        return mass;
    }

    for(const auto& c:def.components)
        mass += componentVolume(c)*materials.get(c.materialId).densityKgM3;

    return mass;
}

bool BlockLibrary::requiresCsg(const BlockDefinition& def) {
    for(const auto& c:def.components)
        if(c.booleanOp!=BooleanOp::Add)return true;
    return false;
}

namespace {

Vec3 toComponentLocal(const GeometryComponent& c,Vec3 p) {
    const Vec3 d=p-c.position;
    const Vec3 r=c.rotationDeg*(kPi/180.0f);
    const Mat4 inv=
        rotationX(-r.x)*
        rotationY(-r.y)*
        rotationZ(-r.z);
    Vec3 local=transformVector(inv,d);
    local.x/=std::max(std::abs(c.scale.x),1.0e-6f);
    local.y/=std::max(std::abs(c.scale.y),1.0e-6f);
    local.z/=std::max(std::abs(c.scale.z),1.0e-6f);
    return local;
}

bool pointInPolygon2D(const std::vector<Vec2>& polygon,Vec2 p) {
    if(polygon.size()<3)return false;
    bool inside=false;
    for(std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
        const auto& a=polygon[i];
        const auto& b=polygon[j];
        const bool crosses=((a.y>p.y)!=(b.y>p.y)) &&
            (p.x < (b.x-a.x)*(p.y-a.y)/(b.y-a.y+1.0e-20f)+a.x);
        if(crosses)inside=!inside;
    }
    return inside;
}

bool pointInside(const GeometryComponent& c,Vec3 worldPoint) {
    const Vec3 p=toComponentLocal(c,worldPoint);

    switch(c.kind) {
        case GeometryKind::Box: {
            const Vec3 h=c.size*0.5f;
            return std::abs(p.x)<=h.x && std::abs(p.y)<=h.y && std::abs(p.z)<=h.z;
        }
        case GeometryKind::Cylinder:
            return p.x*p.x+p.z*p.z<=c.radius*c.radius &&
                   std::abs(p.y)<=c.height*0.5f;
        case GeometryKind::Sphere:
            return lengthSq(p)<=c.radius*c.radius;
        case GeometryKind::Tube: {
            const float r2=p.x*p.x+p.z*p.z;
            return r2<=c.radius*c.radius &&
                   r2>=c.innerRadius*c.innerRadius &&
                   std::abs(p.y)<=c.height*0.5f;
        }
        case GeometryKind::Extrude: {
            const float halfDepth=std::max(0.001f,c.size.z)*0.5f;
            switch(c.profilePlane) {
                case ProfilePlane::XY:
                    return std::abs(p.z)<=halfDepth &&
                           pointInPolygon2D(c.profile,{p.x,p.y});
                case ProfilePlane::XZ:
                    return std::abs(p.y)<=halfDepth &&
                           pointInPolygon2D(c.profile,{p.x,p.z});
                case ProfilePlane::YZ:
                    return std::abs(p.x)<=halfDepth &&
                           pointInPolygon2D(c.profile,{p.y,p.z});
            }
            return false;
        }
        case GeometryKind::Revolve: {
            switch(c.profilePlane) {
                case ProfilePlane::XY: {
                    const float radius=std::sqrt(p.x*p.x+p.z*p.z);
                    return pointInPolygon2D(c.profile,{radius,p.y});
                }
                case ProfilePlane::XZ: {
                    const float radius=std::sqrt(p.x*p.x+p.y*p.y);
                    return pointInPolygon2D(c.profile,{radius,p.z});
                }
                case ProfilePlane::YZ: {
                    const float radius=std::sqrt(p.y*p.y+p.z*p.z);
                    return pointInPolygon2D(c.profile,{radius,p.x});
                }
            }
            return false;
        }
    }
    return false;
}

} // namespace

std::vector<VoxelBox> BlockLibrary::compileVoxelBoxes(const BlockDefinition& def) {
    std::vector<VoxelBox> result;
    if(def.components.empty())return result;

    Aabb bounds=localBounds(def);
    if(!bounds.valid())return result;

    Vec3 size=bounds.size();
    const float longest=std::max({size.x,size.y,size.z,0.001f});
    const int target=std::clamp(def.csgResolution,12,28);

    const int nx=std::clamp(static_cast<int>(std::round(target*size.x/longest)),4,target);
    const int ny=std::clamp(static_cast<int>(std::round(target*size.y/longest)),4,target);
    const int nz=std::clamp(static_cast<int>(std::round(target*size.z/longest)),4,target);

    const Vec3 cell{
        size.x/static_cast<float>(nx),
        size.y/static_cast<float>(ny),
        size.z/static_cast<float>(nz)
    };

    const std::size_t count=
        static_cast<std::size_t>(nx)*
        static_cast<std::size_t>(ny)*
        static_cast<std::size_t>(nz);

    std::vector<int> material(count,-1);
    auto index=[&](int x,int y,int z){
        return static_cast<std::size_t>((z*ny+y)*nx+x);
    };

    std::vector<int> groupOrder;
    groupOrder.reserve(def.components.size());
    for(const auto& c:def.components) {
        if(std::find(groupOrder.begin(),groupOrder.end(),c.operationGroup)==groupOrder.end())
            groupOrder.push_back(c.operationGroup);
    }

    for(int z=0;z<nz;++z)for(int y=0;y<ny;++y)for(int x=0;x<nx;++x) {
        const Vec3 p{
            bounds.min.x+(x+0.5f)*cell.x,
