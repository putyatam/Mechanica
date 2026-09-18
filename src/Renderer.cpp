#include "Renderer.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_opengl_glext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace mechanica {

namespace {

template<class T>
bool loadProc(T& out,const char* name) {
    out=reinterpret_cast<T>(SDL_GL_GetProcAddress(name));
    if(!out) std::fprintf(stderr,"OpenGL function missing: %s\n",name);
    return out!=nullptr;
}

struct Vertex {
    float px,py,pz;
    float nx,ny,nz;
    float r,g,b,a;
};

struct InstanceGpu {
    float model[16];
    float tint[4];
};

struct CpuMesh { std::vector<Vertex> vertices; };

Vec3 applyPoint(const Mat4& m,const Vec3& p){return transformPoint(m,p);}
Vec3 applyNormal(const Mat4& m,const Vec3& n){return normalized(transformVector(m,n));}

void tri(CpuMesh& mesh,const Mat4& tr,Vec3 a,Vec3 b,Vec3 c,Vec3 color) {
    const Vec3 wa=applyPoint(tr,a), wb=applyPoint(tr,b), wc=applyPoint(tr,c);
    const Vec3 n=normalized(cross(wb-wa,wc-wa));
    for(const Vec3& p:{wa,wb,wc}) mesh.vertices.push_back({p.x,p.y,p.z,n.x,n.y,n.z,color.x,color.y,color.z,1.0f});
}

void quad(CpuMesh& mesh,const Mat4& tr,Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 color) {
    tri(mesh,tr,a,b,c,color); tri(mesh,tr,a,c,d,color);
}

void box(CpuMesh& mesh,const Mat4& tr,Vec3 size,Vec3 color) {
    const Vec3 h=size*0.5f;
    const Vec3 p000{-h.x,-h.y,-h.z}, p001{-h.x,-h.y,h.z}, p010{-h.x,h.y,-h.z}, p011{-h.x,h.y,h.z};
    const Vec3 p100{h.x,-h.y,-h.z}, p101{h.x,-h.y,h.z}, p110{h.x,h.y,-h.z}, p111{h.x,h.y,h.z};
    quad(mesh,tr,p100,p110,p111,p101,color);
    quad(mesh,tr,p001,p011,p010,p000,color);
    quad(mesh,tr,p010,p011,p111,p110,color);
    quad(mesh,tr,p001,p000,p100,p101,color);
    quad(mesh,tr,p101,p111,p011,p001,color);
    quad(mesh,tr,p000,p010,p110,p100,color);
}

void cylinder(CpuMesh& mesh,const Mat4& tr,float radius,float height,int segments,Vec3 color) {
    segments=std::clamp(segments,8,96);
    const float y0=-height*0.5f,y1=height*0.5f;
    for(int i=0;i<segments;++i) {
        const float a0=2*kPi*i/segments, a1=2*kPi*(i+1)/segments;
        const Vec3 b0{std::cos(a0)*radius,y0,std::sin(a0)*radius};
        const Vec3 b1{std::cos(a1)*radius,y0,std::sin(a1)*radius};
        const Vec3 t0{b0.x,y1,b0.z},t1{b1.x,y1,b1.z};
        quad(mesh,tr,b0,t0,t1,b1,color);
        tri(mesh,tr,{0,y1,0},t0,t1,color);
        tri(mesh,tr,{0,y0,0},b1,b0,color);
    }
}

void sphere(CpuMesh& mesh,const Mat4& tr,float radius,int segments,Vec3 color) {
    const int slices=std::clamp(segments,12,64);
    const int stacks=std::max(6,slices/2);
    for(int y=0;y<stacks;++y) {
        const float v0=-kPi*0.5f+kPi*y/stacks;
        const float v1=-kPi*0.5f+kPi*(y+1)/stacks;
        for(int x=0;x<slices;++x) {
            const float u0=2*kPi*x/slices,u1=2*kPi*(x+1)/slices;
            auto p=[&](float u,float v){
                const float cv=std::cos(v);
                return Vec3{radius*cv*std::cos(u),radius*std::sin(v),radius*cv*std::sin(u)};
            };
            quad(mesh,tr,p(u0,v0),p(u0,v1),p(u1,v1),p(u1,v0),color);
        }
    }
}

void tube(CpuMesh& mesh,const Mat4& tr,float ro,float ri,float height,int segments,Vec3 color) {
    segments=std::clamp(segments,8,96);
    ri=std::clamp(ri,0.001f,ro-0.001f);
    const float y0=-height*0.5f,y1=height*0.5f;
    for(int i=0;i<segments;++i) {
        const float a0=2*kPi*i/segments,a1=2*kPi*(i+1)/segments;
        auto p=[](float r,float a,float y){return Vec3{std::cos(a)*r,y,std::sin(a)*r};};
        const Vec3 o00=p(ro,a0,y0),o01=p(ro,a1,y0),o10=p(ro,a0,y1),o11=p(ro,a1,y1);
        const Vec3 i00=p(ri,a0,y0),i01=p(ri,a1,y0),i10=p(ri,a0,y1),i11=p(ri,a1,y1);
        quad(mesh,tr,o00,o10,o11,o01,color);
        quad(mesh,tr,i01,i11,i10,i00,color);
        quad(mesh,tr,o10,i10,i11,o11,color);
        quad(mesh,tr,o01,i01,i00,o00,color);
    }
}

float signedArea(const std::vector<Vec2>& p) {
    float a=0;
    for(std::size_t i=0;i<p.size();++i) {
        const auto& v=p[i]; const auto& w=p[(i+1)%p.size()];
        a+=v.x*w.y-w.x*v.y;
    }
    return a*0.5f;
}

bool pointInTriangle(Vec2 p,Vec2 a,Vec2 b,Vec2 c) {
    auto cross2=[](Vec2 u,Vec2 v){return u.x*v.y-u.y*v.x;};
    auto sub=[](Vec2 u,Vec2 v){return Vec2{u.x-v.x,u.y-v.y};};
    const float c1=cross2(sub(b,a),sub(p,a));
    const float c2=cross2(sub(c,b),sub(p,b));
    const float c3=cross2(sub(a,c),sub(p,c));
    const bool neg=(c1<0)||(c2<0)||(c3<0),pos=(c1>0)||(c2>0)||(c3>0);
    return !(neg&&pos);
}

std::vector<std::array<int,3>> triangulate(std::vector<Vec2> p) {
    std::vector<std::array<int,3>> out;
    if(p.size()<3) return out;
    std::vector<int> idx(p.size());
    for(std::size_t i=0;i<p.size();++i) idx[i]=static_cast<int>(i);
    const bool ccw=signedArea(p)>0;

    int guard=0;
    while(idx.size()>3 && guard++<10000) {
        bool clipped=false;
        for(std::size_t ii=0;ii<idx.size();++ii) {
            const int ia=idx[(ii+idx.size()-1)%idx.size()],ib=idx[ii],ic=idx[(ii+1)%idx.size()];
            const Vec2 a=p[ia],b=p[ib],c=p[ic];
            const float cr=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
            if((ccw&&cr<=1e-7f)||(!ccw&&cr>=-1e-7f)) continue;
            bool contains=false;
            for(int j:idx) if(j!=ia&&j!=ib&&j!=ic&&pointInTriangle(p[j],a,b,c)){contains=true;break;}
            if(contains) continue;
            out.push_back(ccw?std::array<int,3>{ia,ib,ic}:std::array<int,3>{ic,ib,ia});
            idx.erase(idx.begin()+static_cast<std::ptrdiff_t>(ii));
            clipped=true; break;
        }
        if(!clipped) break;
    }
    if(idx.size()==3) out.push_back(ccw?std::array<int,3>{idx[0],idx[1],idx[2]}:std::array<int,3>{idx[2],idx[1],idx[0]});
    return out;
}

void extrude(CpuMesh& mesh,const Mat4& tr,const std::vector<Vec2>& profile,float depth,Vec3 color) {
    if(profile.size()<3) return;
    const float z0=-depth*0.5f,z1=depth*0.5f;
    const auto tris=triangulate(profile);
    for(const auto& t:tris) {
        const Vec2 a=profile[t[0]],b=profile[t[1]],c=profile[t[2]];
        tri(mesh,tr,{a.x,a.y,z1},{b.x,b.y,z1},{c.x,c.y,z1},color);
        tri(mesh,tr,{c.x,c.y,z0},{b.x,b.y,z0},{a.x,a.y,z0},color);
    }
    for(std::size_t i=0;i<profile.size();++i) {
        const Vec2 a=profile[i],b=profile[(i+1)%profile.size()];
        quad(mesh,tr,{a.x,a.y,z0},{a.x,a.y,z1},{b.x,b.y,z1},{b.x,b.y,z0},color);
    }
}

void revolve(CpuMesh& mesh,const Mat4& tr,const std::vector<Vec2>& profile,int segments,Vec3 color) {
    if(profile.size()<2) return;
    segments=std::clamp(segments,8,96);
    auto p=[](Vec2 q,float a){return Vec3{q.x*std::cos(a),q.y,q.x*std::sin(a)};};
    for(int s=0;s<segments;++s) {
        const float a0=2*kPi*s/segments,a1=2*kPi*(s+1)/segments;
        for(std::size_t i=0;i+1<profile.size();++i) {
            quad(mesh,tr,p(profile[i],a0),p(profile[i+1],a0),p(profile[i+1],a1),p(profile[i],a1),color);
        }
    }
}

CpuMesh buildDefinitionMesh(const BlockDefinition& def,const MaterialLibrary& materials) {
    CpuMesh mesh;

    if(BlockLibrary::requiresCsg(def)) {
        const auto boxes=BlockLibrary::compileVoxelBoxes(def);
        for(const auto& b:boxes) {
            const Vec3 color=materials.get(b.materialId).color;
            box(mesh,translation(b.center),b.size,color);
        }
        return mesh;
    }

    for(const auto& c:def.components) {
        Transform ct; ct.position=c.position; ct.rotationDeg=c.rotationDeg;
        const Mat4 tr=transformMatrix(ct);
        const Vec3 color=materials.get(c.materialId).color;
        switch(c.kind) {
            case GeometryKind::Box: box(mesh,tr,c.size,color); break;
            case GeometryKind::Cylinder: cylinder(mesh,tr,c.radius,c.height,c.radialSegments,color); break;
            case GeometryKind::Sphere: sphere(mesh,tr,c.radius,c.radialSegments,color); break;
            case GeometryKind::Tube: tube(mesh,tr,c.radius,c.innerRadius,c.height,c.radialSegments,color); break;
            case GeometryKind::Extrude: extrude(mesh,tr,c.profile,std::max(0.001f,c.size.z),color); break;
            case GeometryKind::Revolve: revolve(mesh,tr,c.profile,c.radialSegments,color); break;
        }
    }
    return mesh;
}

void copyMatrix(float* dst,const Mat4& m){std::memcpy(dst,m.m,sizeof(float)*16);}

} // namespace

struct Renderer::Impl {
    PFNGLGENVERTEXARRAYSPROC GenVertexArrays=nullptr;
    PFNGLBINDVERTEXARRAYPROC BindVertexArray=nullptr;
    PFNGLDELETEVERTEXARRAYSPROC DeleteVertexArrays=nullptr;
    PFNGLGENBUFFERSPROC GenBuffers=nullptr;
    PFNGLBINDBUFFERPROC BindBuffer=nullptr;
    PFNGLBUFFERDATAPROC BufferData=nullptr;
    PFNGLDELETEBUFFERSPROC DeleteBuffers=nullptr;
    PFNGLENABLEVERTEXATTRIBARRAYPROC EnableVertexAttribArray=nullptr;
    PFNGLVERTEXATTRIBPOINTERPROC VertexAttribPointer=nullptr;
    PFNGLVERTEXATTRIBDIVISORPROC VertexAttribDivisor=nullptr;
    PFNGLCREATESHADERPROC CreateShader=nullptr;
    PFNGLSHADERSOURCEPROC ShaderSource=nullptr;
    PFNGLCOMPILESHADERPROC CompileShader=nullptr;
    PFNGLGETSHADERIVPROC GetShaderiv=nullptr;
    PFNGLGETSHADERINFOLOGPROC GetShaderInfoLog=nullptr;
    PFNGLCREATEPROGRAMPROC CreateProgram=nullptr;
    PFNGLATTACHSHADERPROC AttachShader=nullptr;
    PFNGLLINKPROGRAMPROC LinkProgram=nullptr;
    PFNGLGETPROGRAMIVPROC GetProgramiv=nullptr;
    PFNGLGETPROGRAMINFOLOGPROC GetProgramInfoLog=nullptr;
    PFNGLDELETESHADERPROC DeleteShader=nullptr;
    PFNGLDELETEPROGRAMPROC DeleteProgram=nullptr;
    PFNGLUSEPROGRAMPROC UseProgram=nullptr;
    PFNGLGETUNIFORMLOCATIONPROC GetUniformLocation=nullptr;
    PFNGLUNIFORMMATRIX4FVPROC UniformMatrix4fv=nullptr;
    PFNGLDRAWARRAYSINSTANCEDPROC DrawArraysInstanced=nullptr;
    PFNGLGENFRAMEBUFFERSPROC GenFramebuffers=nullptr;
    PFNGLBINDFRAMEBUFFERPROC BindFramebuffer=nullptr;
    PFNGLFRAMEBUFFERTEXTURE2DPROC FramebufferTexture2D=nullptr;
    PFNGLCHECKFRAMEBUFFERSTATUSPROC CheckFramebufferStatus=nullptr;
    PFNGLDELETEFRAMEBUFFERSPROC DeleteFramebuffers=nullptr;
    PFNGLGENRENDERBUFFERSPROC GenRenderbuffers=nullptr;
    PFNGLBINDRENDERBUFFERPROC BindRenderbuffer=nullptr;
    PFNGLRENDERBUFFERSTORAGEPROC RenderbufferStorage=nullptr;
    PFNGLFRAMEBUFFERRENDERBUFFERPROC FramebufferRenderbuffer=nullptr;
    PFNGLDELETERENDERBUFFERSPROC DeleteRenderbuffers=nullptr;

    struct GpuMesh {
        GLuint vao=0,vbo=0,instanceBuffer=0;
        GLsizei vertexCount=0;
    };

    GLuint program=0;
    GLint vpLocation=-1;

    GLuint previewFbo=0;
    GLuint previewColor=0;
    GLuint previewDepth=0;
    int previewW=0;
    int previewH=0;

    std::unordered_map<std::string,GpuMesh> meshes;
    std::uint64_t cachedLibraryRevision=0,cachedWorldRevision=0;

    bool load() {
        bool ok=true;
        ok&=loadProc(GenVertexArrays,"glGenVertexArrays"); ok&=loadProc(BindVertexArray,"glBindVertexArray");
        ok&=loadProc(DeleteVertexArrays,"glDeleteVertexArrays"); ok&=loadProc(GenBuffers,"glGenBuffers");
        ok&=loadProc(BindBuffer,"glBindBuffer"); ok&=loadProc(BufferData,"glBufferData");
        ok&=loadProc(DeleteBuffers,"glDeleteBuffers"); ok&=loadProc(EnableVertexAttribArray,"glEnableVertexAttribArray");
        ok&=loadProc(VertexAttribPointer,"glVertexAttribPointer"); ok&=loadProc(VertexAttribDivisor,"glVertexAttribDivisor");
        ok&=loadProc(CreateShader,"glCreateShader"); ok&=loadProc(ShaderSource,"glShaderSource");
        ok&=loadProc(CompileShader,"glCompileShader"); ok&=loadProc(GetShaderiv,"glGetShaderiv");
        ok&=loadProc(GetShaderInfoLog,"glGetShaderInfoLog"); ok&=loadProc(CreateProgram,"glCreateProgram");
        ok&=loadProc(AttachShader,"glAttachShader"); ok&=loadProc(LinkProgram,"glLinkProgram");
        ok&=loadProc(GetProgramiv,"glGetProgramiv"); ok&=loadProc(GetProgramInfoLog,"glGetProgramInfoLog");
        ok&=loadProc(DeleteShader,"glDeleteShader"); ok&=loadProc(DeleteProgram,"glDeleteProgram");
        ok&=loadProc(UseProgram,"glUseProgram"); ok&=loadProc(GetUniformLocation,"glGetUniformLocation");
        ok&=loadProc(UniformMatrix4fv,"glUniformMatrix4fv"); ok&=loadProc(DrawArraysInstanced,"glDrawArraysInstanced");
        ok&=loadProc(GenFramebuffers,"glGenFramebuffers"); ok&=loadProc(BindFramebuffer,"glBindFramebuffer");
        ok&=loadProc(FramebufferTexture2D,"glFramebufferTexture2D"); ok&=loadProc(CheckFramebufferStatus,"glCheckFramebufferStatus");
        ok&=loadProc(DeleteFramebuffers,"glDeleteFramebuffers"); ok&=loadProc(GenRenderbuffers,"glGenRenderbuffers");
        ok&=loadProc(BindRenderbuffer,"glBindRenderbuffer"); ok&=loadProc(RenderbufferStorage,"glRenderbufferStorage");
        ok&=loadProc(FramebufferRenderbuffer,"glFramebufferRenderbuffer"); ok&=loadProc(DeleteRenderbuffers,"glDeleteRenderbuffers");
        return ok;
    }

    GLuint shader(GLenum type,const char* src) {
        GLuint s=CreateShader(type); ShaderSource(s,1,&src,nullptr); CompileShader(s);
        GLint good=GL_FALSE; GetShaderiv(s,GL_COMPILE_STATUS,&good);
        if(!good){char log[2048]{};GLsizei n=0;GetShaderInfoLog(s,2048,&n,log);std::fprintf(stderr,"%s\n",log);DeleteShader(s);return 0;}
        return s;
    }

    bool makeProgram() {
        const char* vs=R"GLSL(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec4 aColor;
layout(location=3) in vec4 iM0;
layout(location=4) in vec4 iM1;
layout(location=5) in vec4 iM2;
layout(location=6) in vec4 iM3;
layout(location=7) in vec4 iTint;
uniform mat4 uVP;
out vec3 vNormal;
out vec4 vColor;
void main(){
    mat4 M=mat4(iM0,iM1,iM2,iM3);
    vec4 w=M*vec4(aPos,1.0);
    vNormal=normalize(mat3(M)*aNormal);
    vec3 c=mix(aColor.rgb,iTint.rgb,iTint.a);
    vColor=vec4(c,aColor.a);
    gl_Position=uVP*w;
})GLSL";
        const char* fs=R"GLSL(#version 330 core
in vec3 vNormal; in vec4 vColor; out vec4 FragColor;
void main(){
    vec3 L=normalize(vec3(0.45,1.0,0.35));
    float d=max(dot(normalize(vNormal),L),0.0);
    float light=0.34+0.66*d;
    FragColor=vec4(vColor.rgb*light,vColor.a);
})GLSL";
        GLuint v=shader(GL_VERTEX_SHADER,vs),f=shader(GL_FRAGMENT_SHADER,fs);
        if(!v||!f)return false;
        program=CreateProgram();AttachShader(program,v);AttachShader(program,f);LinkProgram(program);DeleteShader(v);DeleteShader(f);
        GLint good=GL_FALSE;GetProgramiv(program,GL_LINK_STATUS,&good);
        if(!good){char log[2048]{};GLsizei n=0;GetProgramInfoLog(program,2048,&n,log);std::fprintf(stderr,"%s\n",log);return false;}
        vpLocation=GetUniformLocation(program,"uVP");
        return true;
    }

    void destroyMesh(GpuMesh& g) {
        if(g.instanceBuffer)DeleteBuffers(1,&g.instanceBuffer);
        if(g.vbo)DeleteBuffers(1,&g.vbo);
        if(g.vao)DeleteVertexArrays(1,&g.vao);
        g={};
    }

    void clearMeshes() {
        for(auto& [k,g]:meshes) destroyMesh(g);
        meshes.clear();
    }

    GpuMesh upload(const CpuMesh& cpu) {
        GpuMesh g; if(cpu.vertices.empty()) return g;
        GenVertexArrays(1,&g.vao);BindVertexArray(g.vao);
        GenBuffers(1,&g.vbo);BindBuffer(GL_ARRAY_BUFFER,g.vbo);
        BufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(cpu.vertices.size()*sizeof(Vertex)),cpu.vertices.data(),GL_STATIC_DRAW);
        g.vertexCount=static_cast<GLsizei>(cpu.vertices.size());
        const GLsizei stride=sizeof(Vertex);
        EnableVertexAttribArray(0);VertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,stride,reinterpret_cast<void*>(offsetof(Vertex,px)));
        EnableVertexAttribArray(1);VertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,stride,reinterpret_cast<void*>(offsetof(Vertex,nx)));
        EnableVertexAttribArray(2);VertexAttribPointer(2,4,GL_FLOAT,GL_FALSE,stride,reinterpret_cast<void*>(offsetof(Vertex,r)));

        GenBuffers(1,&g.instanceBuffer);BindBuffer(GL_ARRAY_BUFFER,g.instanceBuffer);
        const GLsizei is=sizeof(InstanceGpu);
        for(GLuint c=0;c<4;++c){
            EnableVertexAttribArray(3+c);
            VertexAttribPointer(3+c,4,GL_FLOAT,GL_FALSE,is,reinterpret_cast<void*>(offsetof(InstanceGpu,model)+sizeof(float)*4*c));
            VertexAttribDivisor(3+c,1);
        }
        EnableVertexAttribArray(7);VertexAttribPointer(7,4,GL_FLOAT,GL_FALSE,is,reinterpret_cast<void*>(offsetof(InstanceGpu,tint)));VertexAttribDivisor(7,1);
        BindVertexArray(0);
        return g;
    }

    std::string keyFor(const BlockInstance& i) const {
        return i.localOverride ? ("instance:"+std::to_string(i.id)) : ("def:"+i.definitionId);
    }

    void rebuild(const BuildWorld& world,const BlockLibrary& lib,const MaterialLibrary& mats) {
        clearMeshes();
        for(const auto& i:world.instances()) {
            const std::string key=keyFor(i);
            if(meshes.contains(key))continue;
            if(const auto* d=world.definitionFor(i,lib)) meshes.emplace(key,upload(buildDefinitionMesh(*d,mats)));
        }

        CpuMesh grid;
        constexpr int count=32; constexpr float step=0.5f; constexpr float length=count*step*2.0f;
        for(int i=-count;i<=count;++i){
            const bool major=i%5==0; const Vec3 c=major?Vec3{0.28f,0.30f,0.34f}:Vec3{0.17f,0.18f,0.21f};
            const float thick=major?0.014f:0.007f;
            box(grid,translation({0,-0.012f,i*step}),{length,0.018f,thick},c);
            box(grid,translation({i*step,-0.012f,0}),{thick,0.018f,length},c);
        }
        meshes.emplace("__grid",upload(grid));
        cachedLibraryRevision=lib.revision();
        cachedWorldRevision=world.geometryRevision();
    }

    void draw(GpuMesh& mesh,const std::vector<InstanceGpu>& instances,const Mat4& vp) {
        if(!mesh.vao||instances.empty())return;
        BindVertexArray(mesh.vao);BindBuffer(GL_ARRAY_BUFFER,mesh.instanceBuffer);
        BufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(instances.size()*sizeof(InstanceGpu)),instances.data(),GL_STREAM_DRAW);
        UseProgram(program);UniformMatrix4fv(vpLocation,1,GL_FALSE,vp.m);
        DrawArraysInstanced(GL_TRIANGLES,0,mesh.vertexCount,static_cast<GLsizei>(instances.size()));
        BindVertexArray(0);
    }

    void ensurePreviewTarget(int width,int height) {
        width=std::max(width,64);
        height=std::max(height,64);
        if(previewFbo && width==previewW && height==previewH)return;

        if(previewDepth){DeleteRenderbuffers(1,&previewDepth);previewDepth=0;}
        if(previewColor){glDeleteTextures(1,&previewColor);previewColor=0;}
        if(previewFbo){DeleteFramebuffers(1,&previewFbo);previewFbo=0;}

        previewW=width;previewH=height;

        GenFramebuffers(1,&previewFbo);
        BindFramebuffer(GL_FRAMEBUFFER,previewFbo);

        glGenTextures(1,&previewColor);
        glBindTexture(GL_TEXTURE_2D,previewColor);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,width,height,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
        FramebufferTexture2D(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_TEXTURE_2D,previewColor,0);

        GenRenderbuffers(1,&previewDepth);
        BindRenderbuffer(GL_RENDERBUFFER,previewDepth);
        RenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH24_STENCIL8,width,height);
        FramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_STENCIL_ATTACHMENT,GL_RENDERBUFFER,previewDepth);

        BindFramebuffer(GL_FRAMEBUFFER,0);
    }
};

Renderer::Renderer():m(std::make_unique<Impl>()){}
Renderer::~Renderer(){shutdown();}

bool Renderer::initialize() {
    if(!m->load()||!m->makeProgram())return false;
    return true;
}

void Renderer::shutdown() {
    if(!m)return;
    m->clearMeshes();
    if(m->previewDepth){m->DeleteRenderbuffers(1,&m->previewDepth);m->previewDepth=0;}
    if(m->previewColor){glDeleteTextures(1,&m->previewColor);m->previewColor=0;}
    if(m->previewFbo){m->DeleteFramebuffers(1,&m->previewFbo);m->previewFbo=0;}
    if(m->program){m->DeleteProgram(m->program);m->program=0;}
}

void Renderer::render(
    const BuildWorld& world,
    const BlockLibrary& library,
    const MaterialLibrary& materials,
    const Mat4& viewProjection,
    const PlacementPreview* preview,
    const std::vector<PlacementPreview>* extraPreviews,
    const std::unordered_map<std::uint64_t,bool>* attachmentChoices,
    const std::vector<PartRootPose>* simulatedPoses
) {
    if(m->cachedLibraryRevision!=library.revision() || m->cachedWorldRevision!=world.geometryRevision())
        m->rebuild(world,library,materials);

    glEnable(GL_DEPTH_TEST);glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    if(auto it=m->meshes.find("__grid");it!=m->meshes.end()) {
        InstanceGpu g{};copyMatrix(g.model,Mat4::identity());g.tint[3]=0;
        m->draw(it->second,{g},viewProjection);
    }

    std::unordered_map<std::string,std::vector<InstanceGpu>> batches;
    for(std::size_t idx=0;idx<world.instances().size();++idx) {
        const auto& instance=world.instances()[idx];
        Mat4 model;
        if(simulatedPoses && idx<simulatedPoses->size() && (*simulatedPoses)[idx].valid) {
            const auto& p=(*simulatedPoses)[idx];
            Transform local=instance.transform;
            local.position=instance.transform.position-p.buildOrigin;
            model=translation(p.worldPosition)*rotationQuaternion(p.qx,p.qy,p.qz,p.qw)*transformMatrix(local);
        } else model=transformMatrix(instance.transform);

        InstanceGpu gpu{};copyMatrix(gpu.model,model);

        if(world.isSelected(instance.id)){
            gpu.tint[0]=0.25f;gpu.tint[1]=0.70f;gpu.tint[2]=1.0f;gpu.tint[3]=0.42f;
        }

        if(preview) {
            const bool touching=std::find(preview->touchingIds.begin(),preview->touchingIds.end(),instance.id)!=preview->touchingIds.end();
            if(touching) {
                bool attached=true;
                if(attachmentChoices) {
                    auto it=attachmentChoices->find(instance.id);
                    if(it!=attachmentChoices->end())attached=it->second;
                }
                if(attached){gpu.tint[0]=0.20f;gpu.tint[1]=1.0f;gpu.tint[2]=0.40f;gpu.tint[3]=0.55f;}
                else {gpu.tint[0]=1.0f;gpu.tint[1]=0.30f;gpu.tint[2]=0.24f;gpu.tint[3]=0.55f;}
            }
        }

        batches[m->keyFor(instance)].push_back(gpu);
    }

    for(auto& [key,instances]:batches) {
        auto it=m->meshes.find(key);
        if(it!=m->meshes.end())m->draw(it->second,instances,viewProjection);
    }

    auto drawPreview=[&](const PlacementPreview& item,float alpha){
        if(!item.hasCandidate)return;
        const auto* def=library.find(item.candidate.definitionId);
        if(!def)return;

        CpuMesh cpu=buildDefinitionMesh(*def,materials);
        auto temp=m->upload(cpu);
        InstanceGpu gpu{};copyMatrix(gpu.model,transformMatrix(item.candidate.transform));

        if(item.valid){gpu.tint[0]=0.18f;gpu.tint[1]=1.0f;gpu.tint[2]=0.42f;}
        else {gpu.tint[0]=1.0f;gpu.tint[1]=0.18f;gpu.tint[2]=0.18f;}
        gpu.tint[3]=alpha;

        m->draw(temp,{gpu},viewProjection);
        m->destroyMesh(temp);
    };

    if(preview)drawPreview(*preview,0.55f);

    if(extraPreviews) {
        for(const auto& item:*extraPreviews)drawPreview(item,0.34f);
    }
}


std::uint32_t Renderer::renderBlockPreview(
    const BlockDefinition& definition,
    const MaterialLibrary& materials,
    int width,
    int height,
    float yaw,
    float pitch,
    float zoom
) {
    if(!m)return 0;

    m->ensurePreviewTarget(width,height);

    GLint oldViewport[4]{};
    glGetIntegerv(GL_VIEWPORT,oldViewport);

    m->BindFramebuffer(GL_FRAMEBUFFER,m->previewFbo);
    glViewport(0,0,m->previewW,m->previewH);
    glClearColor(0.065f,0.075f,0.095f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    CpuMesh cpu=buildDefinitionMesh(definition,materials);
    auto gpuMesh=m->upload(cpu);

    const Aabb bounds=BlockLibrary::localBounds(definition);
    const Vec3 center=bounds.center();
    const float radius=std::max(0.25f,length(bounds.size())*0.5f);
    const float distance=radius*std::clamp(zoom,1.8f,8.0f);

    const Vec3 eye{
        center.x+distance*std::cos(pitch)*std::sin(yaw),
        center.y+distance*std::sin(pitch),
        center.z+distance*std::cos(pitch)*std::cos(yaw)
    };

    const Mat4 vp=
        perspective(48.0f*kPi/180.0f,static_cast<float>(m->previewW)/std::max(m->previewH,1),0.01f,std::max(100.0f,distance*20.0f))*
        lookAt(eye,center,{0,1,0});

    InstanceGpu inst{};
    copyMatrix(inst.model,Mat4::identity());
    inst.tint[3]=0.0f;
    m->draw(gpuMesh,{inst},vp);
    m->destroyMesh(gpuMesh);

    m->BindFramebuffer(GL_FRAMEBUFFER,0);
    glViewport(oldViewport[0],oldViewport[1],oldViewport[2],oldViewport[3]);

    return static_cast<std::uint32_t>(m->previewColor);
}

} // namespace mechanica
