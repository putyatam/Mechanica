#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace mechanica {

constexpr float kPi = 3.14159265358979323846f;

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    constexpr Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    constexpr Vec3 operator*(const Vec3& r) const { return {x * r.x, y * r.y, z * r.z}; }
    constexpr Vec3 operator/(const Vec3& r) const { return {x / r.x, y / r.y, z / r.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& r) { x += r.x; y += r.y; z += r.z; return *this; }
    Vec3& operator-=(const Vec3& r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
};

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline float lengthSq(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v) { return std::sqrt(lengthSq(v)); }

inline Vec3 normalized(const Vec3& v) {
    const float l = length(v);
    return l > 1.0e-7f ? v / l : Vec3{};
}

inline Vec3 minVec(const Vec3& a, const Vec3& b) {
    return {std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z)};
}

inline Vec3 maxVec(const Vec3& a, const Vec3& b) {
    return {std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z)};
}

inline float snap(float value, float step) {
    if (step <= 1.0e-7f) return value;
    return std::round(value / step) * step;
}

inline Vec3 snap(const Vec3& value, float step) {
    return {snap(value.x, step), snap(value.y, step), snap(value.z, step)};
}

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct Aabb {
    Vec3 min{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    Vec3 max{ -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };

    void include(const Vec3& p) {
        min = minVec(min, p);
        max = maxVec(max, p);
    }

    void include(const Aabb& b) {
        include(b.min);
        include(b.max);
    }

    [[nodiscard]] Vec3 center() const { return (min + max) * 0.5f; }
    [[nodiscard]] Vec3 size() const { return max - min; }
    [[nodiscard]] bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
};

struct Transform {
    Vec3 position{};
    Vec3 rotationDeg{};
    Vec3 scale{1.0f, 1.0f, 1.0f};
};

struct Mat4 {
    float m[16]{};

    static Mat4 identity() {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            float v = 0.0f;
            for (int k = 0; k < 4; ++k) v += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = v;
        }
    }
    return r;
}

inline Mat4 translation(const Vec3& t) {
    Mat4 r = Mat4::identity();
    r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
    return r;
}

inline Mat4 scaling(const Vec3& s) {
    Mat4 r{};
    r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z; r.m[15] = 1.0f;
    return r;
}

inline Mat4 rotationX(float rads) {
    Mat4 r = Mat4::identity();
    const float c = std::cos(rads), s = std::sin(rads);
    r.m[5] = c; r.m[6] = s;
    r.m[9] = -s; r.m[10] = c;
    return r;
}

inline Mat4 rotationY(float rads) {
    Mat4 r = Mat4::identity();
    const float c = std::cos(rads), s = std::sin(rads);
    r.m[0] = c; r.m[2] = -s;
    r.m[8] = s; r.m[10] = c;
    return r;
}

inline Mat4 rotationZ(float rads) {
    Mat4 r = Mat4::identity();
    const float c = std::cos(rads), s = std::sin(rads);
    r.m[0] = c; r.m[1] = s;
    r.m[4] = -s; r.m[5] = c;
    return r;
}

inline Mat4 rotationEulerDeg(const Vec3& d) {
    const Vec3 r = d * (kPi / 180.0f);
    return rotationZ(r.z) * rotationY(r.y) * rotationX(r.x);
}

inline Mat4 transformMatrix(const Transform& t) {
    return translation(t.position) * rotationEulerDeg(t.rotationDeg) * scaling(t.scale);
}

inline Vec3 transformPoint(const Mat4& m, const Vec3& p) {
    return {
        m.m[0]*p.x + m.m[4]*p.y + m.m[8]*p.z + m.m[12],
        m.m[1]*p.x + m.m[5]*p.y + m.m[9]*p.z + m.m[13],
        m.m[2]*p.x + m.m[6]*p.y + m.m[10]*p.z + m.m[14]
    };
}

inline Vec3 transformVector(const Mat4& m, const Vec3& p) {
    return {
        m.m[0]*p.x + m.m[4]*p.y + m.m[8]*p.z,
        m.m[1]*p.x + m.m[5]*p.y + m.m[9]*p.z,
        m.m[2]*p.x + m.m[6]*p.y + m.m[10]*p.z
    };
}

inline Vec3 rotateVectorEulerDeg(const Vec3& p, const Vec3& degrees) {
    return transformVector(rotationEulerDeg(degrees), p);
}

inline Mat4 rotationQuaternion(float x, float y, float z, float w) {
    Mat4 r = Mat4::identity();
    const float xx=x*x, yy=y*y, zz=z*z, xy=x*y, xz=x*z, yz=y*z, wx=w*x, wy=w*y, wz=w*z;
    r.m[0]=1-2*(yy+zz); r.m[1]=2*(xy+wz); r.m[2]=2*(xz-wy);
    r.m[4]=2*(xy-wz); r.m[5]=1-2*(xx+zz); r.m[6]=2*(yz+wx);
    r.m[8]=2*(xz+wy); r.m[9]=2*(yz-wx); r.m[10]=1-2*(xx+yy);
    return r;
}

inline Mat4 perspective(float fovYRadians, float aspect, float nearPlane, float farPlane) {
    const float f = 1.0f / std::tan(fovYRadians * 0.5f);
    Mat4 r{};
    r.m[0] = f / std::max(aspect, 0.001f);
    r.m[5] = f;
    r.m[10] = (farPlane + nearPlane) / (nearPlane - farPlane);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * farPlane * nearPlane) / (nearPlane - farPlane);
    return r;
}

inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalized(center - eye);
    const Vec3 s = normalized(cross(f, up));
    const Vec3 u = cross(s, f);
    Mat4 r = Mat4::identity();
    r.m[0]=s.x; r.m[4]=s.y; r.m[8]=s.z;
    r.m[1]=u.x; r.m[5]=u.y; r.m[9]=u.z;
    r.m[2]=-f.x; r.m[6]=-f.y; r.m[10]=-f.z;
    r.m[12]=-dot(s,eye); r.m[13]=-dot(u,eye); r.m[14]=dot(f,eye);
    return r;
}

inline Aabb transformAabb(const Aabb& local, const Mat4& m) {
    Aabb out;
    for (int i=0;i<8;++i) {
        Vec3 p{
            (i&1)?local.max.x:local.min.x,
            (i&2)?local.max.y:local.min.y,
            (i&4)?local.max.z:local.min.z
        };
        out.include(transformPoint(m,p));
    }
    return out;
}

inline bool rayAabb(const Ray& ray, const Aabb& b, float& outT) {
    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::max();
    const float ro[3] = {ray.origin.x,ray.origin.y,ray.origin.z};
    const float rd[3] = {ray.direction.x,ray.direction.y,ray.direction.z};
    const float mn[3] = {b.min.x,b.min.y,b.min.z};
    const float mx[3] = {b.max.x,b.max.y,b.max.z};
    for(int a=0;a<3;++a) {
        if(std::abs(rd[a])<1e-7f) {
            if(ro[a]<mn[a] || ro[a]>mx[a]) return false;
            continue;
        }
        float t1=(mn[a]-ro[a])/rd[a], t2=(mx[a]-ro[a])/rd[a];
        if(t1>t2) std::swap(t1,t2);
        tmin=std::max(tmin,t1); tmax=std::min(tmax,t2);
        if(tmin>tmax) return false;
    }
    outT=tmin;
    return true;
}

} // namespace mechanica
