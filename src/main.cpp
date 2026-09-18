#include "BlockLibrary.hpp"
#include "BuildWorld.hpp"
#include "Continuum.hpp"
#include "MaterialLibrary.hpp"
#include "Math3D.hpp"
#include "PhysicsWorld.hpp"
#include "Renderer.hpp"

#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_opengl3.h>
#include <misc/cpp/imgui_stdlib.h>
#include <ImGuizmo.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using Clock=std::chrono::steady_clock;
using namespace mechanica;

struct Camera {
    Vec3 target{0.0f,0.8f,0.0f};
    float yaw=-0.70f,pitch=0.58f,distance=10.5f;

    Vec3 position() const {
        const float cp=std::cos(pitch);
        return {target.x+distance*cp*std::sin(yaw),target.y+distance*std::sin(pitch),target.z+distance*cp*std::cos(yaw)};
    }
    Mat4 view() const {
        return lookAt(position(),target,{0,1,0});
    }
    Mat4 projection(float aspect) const {
        return perspective(55.0f*kPi/180.0f,aspect,0.05f,300.0f);
    }
    Mat4 viewProjection(float aspect) const {
        return projection(aspect)*view();
    }
    Ray mouseRay(float x,float y,float width,float height) const {
        const Vec3 eye=position();
        const Vec3 f=normalized(target-eye);
        const Vec3 r=normalized(cross(f,{0,1,0}));
        const Vec3 u=cross(r,f);
        const float aspect=width/std::max(height,1.0f);
        const float t=std::tan(55.0f*kPi/180.0f*0.5f);
        const float nx=2.0f*x/std::max(width,1.0f)-1.0f;
        const float ny=1.0f-2.0f*y/std::max(height,1.0f);
        return {eye,normalized(f+r*(nx*aspect*t)+u*(ny*t))};
    }
    void orbit(float dx,float dy){yaw-=dx*0.006f;pitch=std::clamp(pitch+dy*0.006f,0.12f,1.42f);}
    void zoom(float wheel){distance=std::clamp(distance*std::pow(0.88f,wheel),2.5f,45.0f);}
    void pan(float fwd,float right,float dt) {
        Vec3 f=target-position();f.y=0;f=normalized(f);
        Vec3 r=normalized(cross(f,{0,1,0}));
        const float speed=std::max(2.0f,distance*0.45f);
        target+=f*(fwd*speed*dt)+r*(right*speed*dt);
    }
};

struct BlockEditorState {
    bool active=false;
    bool hasBackup=false;
    BuildWorld savedWorld;
    Camera savedCamera;
    std::string nameRu="Новый блок";
    std::string group="Базовые блоки";
    bool geometryInspector=false;

    bool open=false;
    std::uint64_t editingInstanceId=0;
    BlockDefinition draft;
    int selectedComponent=0;
    int regularSides=8;
    float regularRadius=0.30f;
    int copyBlockIndex=0;
    float previewYaw=-0.65f;
    float previewPitch=0.45f;
    float previewZoom=3.2f;
    int profilePoint=-1;

    void resetInspector() {
        open=false;
        editingInstanceId=0;
        selectedComponent=0;
        profilePoint=-1;
        geometryInspector=false;
    }
};

enum class ToolPanel {
    None,
    Symmetry,
    Move,
    Rotate,
    Scale
};

struct ToolState {
    ToolPanel panel=ToolPanel::None;
    bool symmetryEnabled=false;
    bool symmetryX=false;
    bool symmetryY=false;
    bool symmetryZ=false;
    float moveStep=0.05f;
    float rotateStep=5.0f;
    float scaleStep=0.05f;
};

std::vector<PlacementPreview> makeSymmetryPreviews(
    const PlacementPreview& base,
    const ToolState& tools,
    const BuildWorld& world,
    const BlockLibrary& library
) {
    std::vector<PlacementPreview> result;
    if(!base.hasCandidate)return result;

    result.push_back(base);

    if(!tools.symmetryEnabled)return result;
    const int masks=(tools.symmetryX?1:0)|(tools.symmetryY?2:0)|(tools.symmetryZ?4:0);
    if(masks==0)return result;

    for(int subset=1;subset<8;++subset) {
        if((subset&~masks)!=0)continue;

        BlockInstance mirrored=base.candidate;
        if(subset&1)mirrored.transform.position.x=-mirrored.transform.position.x;
        if(subset&2)mirrored.transform.position.y=-mirrored.transform.position.y;
        if(subset&4)mirrored.transform.position.z=-mirrored.transform.position.z;

        bool duplicate=false;
        for(const auto& existing:result) {
            if(lengthSq(existing.candidate.transform.position-mirrored.transform.position)<1.0e-8f) {
                duplicate=true;break;
            }
        }
        if(!duplicate)result.push_back(world.analyzeCandidate(mirrored,library));
    }

    return result;
}

const char* kindRu(GeometryKind k) {
    switch(k) {
        case GeometryKind::Box:return "Параллелепипед";
        case GeometryKind::Cylinder:return "Цилиндр";
        case GeometryKind::Sphere:return "Сфера";
        case GeometryKind::Tube:return "Труба";
        case GeometryKind::Extrude:return "Выдавленный профиль";
        case GeometryKind::Revolve:return "Тело вращения";
    }
    return "Форма";
}

GeometryComponent defaultComponent(GeometryKind k,std::uint64_t id) {
    GeometryComponent c;c.id=id;c.kind=k;c.name=kindRu(k);
    if(k==GeometryKind::Extrude){c.profile={{-0.3f,-0.2f},{0.3f,-0.2f},{0.3f,0.2f},{-0.3f,0.2f}};c.size.z=0.3f;}
    if(k==GeometryKind::Revolve){c.profile={{0.0f,-0.3f},{0.20f,-0.3f},{0.30f,0.0f},{0.20f,0.3f},{0.0f,0.3f}};}
    if(k==GeometryKind::Tube){c.radius=0.30f;c.innerRadius=0.20f;c.height=0.8f;}
    return c;
}

void materialCombo(std::string& id,const MaterialLibrary& materials) {
    int index=materials.indexOf(id);
    const auto& all=materials.all();
    if(ImGui::BeginCombo("Материал",all[static_cast<std::size_t>(index)].nameRu.c_str())) {
        for(int i=0;i<static_cast<int>(all.size());++i) {
            const bool selected=i==index;
            if(ImGui::Selectable(all[static_cast<std::size_t>(i)].nameRu.c_str(),selected)) {
                id=all[static_cast<std::size_t>(i)].id;index=i;
            }
            if(selected)ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    const auto& m=materials.get(id);
    ImGui::TextDisabled("ρ %.0f кг/м³ | E %.1f ГПа | σт %.0f МПа",
        m.densityKgM3,m.youngModulusPa/1e9,m.yieldStrengthPa/1e6);
}

void profileEditor(std::vector<Vec2>& profile,bool revolve,BlockEditorState& state) {
    if(!revolve) {
        ImGui::SetNextItemWidth(78);
        ImGui::InputInt("Сторон##poly",&state.regularSides);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(92);
        ImGui::InputFloat("Радиус##poly",&state.regularRadius,0.01f,0.1f,"%.3f");

        if(ImGui::Button("Правильный многоугольник",{-1,0}))
            profile=makeRegularPolygon(state.regularSides,state.regularRadius);

        if(ImGui::Button("Прямоугольник",{-1,0}))
            profile={{-0.3f,-0.2f},{0.3f,-0.2f},{0.3f,0.2f},{-0.3f,0.2f}};
    }

    ImGui::TextDisabled(
        revolve
            ?"X = радиус, Y = высота. Точки перетаскиваются мышью."
            :"Произвольный контур: перетаскивай вершины мышью."
    );

    // Interactive 2D profile canvas.
    const ImVec2 canvasSize{
        std::max(260.0f,ImGui::GetContentRegionAvail().x),
        245.0f
    };
    const ImVec2 canvasMin=ImGui::GetCursorScreenPos();
    const ImVec2 canvasMax{canvasMin.x+canvasSize.x,canvasMin.y+canvasSize.y};

    ImGui::InvisibleButton(
        "##profile_canvas",
        canvasSize,
        ImGuiButtonFlags_MouseButtonLeft
    );

    ImDrawList* dl=ImGui::GetWindowDrawList();
    dl->AddRectFilled(canvasMin,canvasMax,IM_COL32(27,30,36,255));
    dl->AddRect(canvasMin,canvasMax,IM_COL32(74,80,92,255));

    float extent=0.5f;
    for(const auto& point:profile)
        extent=std::max(extent,std::max(std::abs(point.x),std::abs(point.y))*1.25f);

    const float pixelsPerUnit=
        std::min(canvasSize.x,canvasSize.y)*
        0.43f/
        extent;

    const ImVec2 center{
        canvasMin.x+canvasSize.x*0.5f,
        canvasMin.y+canvasSize.y*0.5f
    };

    auto toScreen=[&](Vec2 p){
        return ImVec2{
            center.x+p.x*pixelsPerUnit,
            center.y-p.y*pixelsPerUnit
        };
    };
    auto toProfile=[&](ImVec2 p){
        Vec2 value{
            (p.x-center.x)/pixelsPerUnit,
            -(p.y-center.y)/pixelsPerUnit
        };
        if(revolve)value.x=std::max(0.0f,value.x);
        return value;
    };

    // Grid and axes.
    const float gridStep=
        extent>2.0f?0.5f:
        extent>0.8f?0.2f:
        0.1f;

    for(float v=-extent;v<=extent+gridStep*0.5f;v+=gridStep) {
        const float x=center.x+v*pixelsPerUnit;
        const float y=center.y-v*pixelsPerUnit;
        dl->AddLine({x,canvasMin.y},{x,canvasMax.y},IM_COL32(47,51,60,255));
        dl->AddLine({canvasMin.x,y},{canvasMax.x,y},IM_COL32(47,51,60,255));
    }

    dl->AddLine({canvasMin.x,center.y},{canvasMax.x,center.y},IM_COL32(105,110,123,255),1.5f);
    dl->AddLine({center.x,canvasMin.y},{center.x,canvasMax.y},IM_COL32(105,110,123,255),1.5f);

    if(profile.size()>=2) {
        for(std::size_t i=0;i+1<profile.size();++i)
            dl->AddLine(toScreen(profile[i]),toScreen(profile[i+1]),IM_COL32(115,190,245,255),2.0f);

        if(!revolve&&profile.size()>=3)
            dl->AddLine(toScreen(profile.back()),toScreen(profile.front()),IM_COL32(115,190,245,255),2.0f);
    }

    for(int i=0;i<static_cast<int>(profile.size());++i) {
        const ImVec2 point=toScreen(profile[static_cast<std::size_t>(i)]);
        const bool selected=state.profilePoint==i;
        dl->AddCircleFilled(
            point,
            selected?7.0f:5.5f,
            selected?IM_COL32(255,194,72,255):IM_COL32(225,231,240,255)
        );
        dl->AddCircle(point,8.0f,IM_COL32(25,28,34,255),0,1.5f);
    }

    if(ImGui::IsItemHovered()&&ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const ImVec2 mouse=ImGui::GetIO().MousePos;
        state.profilePoint=-1;
        float best=12.0f*12.0f;

        for(int i=0;i<static_cast<int>(profile.size());++i) {
            const ImVec2 point=toScreen(profile[static_cast<std::size_t>(i)]);
            const float dx=point.x-mouse.x;
            const float dy=point.y-mouse.y;
            const float d=dx*dx+dy*dy;
            if(d<best){best=d;state.profilePoint=i;}
        }

        if(state.profilePoint<0&&ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            profile.push_back(toProfile(mouse));
            state.profilePoint=static_cast<int>(profile.size())-1;
        }
    }

    if(
        state.profilePoint>=0 &&
        state.profilePoint<static_cast<int>(profile.size()) &&
        ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
        ImGui::IsItemActive()
    ) {
        profile[static_cast<std::size_t>(state.profilePoint)]=
            toProfile(ImGui::GetIO().MousePos);
    }

    if(!ImGui::IsMouseDown(ImGuiMouseButton_Left)&&!ImGui::IsItemHovered())
        state.profilePoint=-1;

    ImGui::TextDisabled("Двойной щелчок по пустому месту — добавить вершину.");

    // Exact numeric edit remains available below the graphical canvas.
    for(int i=0;i<static_cast<int>(profile.size());++i) {
        ImGui::PushID(i);
        float point[2]={
            profile[static_cast<std::size_t>(i)].x,
            profile[static_cast<std::size_t>(i)].y
        };

        ImGui::SetNextItemWidth(205);
        if(ImGui::DragFloat2("##point",point,0.005f,-20.0f,20.0f,"%.3f")) {
            if(revolve)point[0]=std::max(0.0f,point[0]);
            profile[static_cast<std::size_t>(i)]={point[0],point[1]};
        }

        ImGui::SameLine();
        if(
            ImGui::SmallButton("×") &&
            profile.size()>(revolve?2u:3u)
        ) {
            profile.erase(profile.begin()+i);
            if(state.profilePoint==i)state.profilePoint=-1;
            --i;
        }
        ImGui::PopID();
    }

    if(ImGui::Button("+ Точка",{-1,0})) {
        profile.push_back(
            profile.empty()
                ?Vec2{}
                :Vec2{profile.back().x+0.1f,profile.back().y}
        );
    }
}

bool drawBlockEditor(
    BlockEditorState& state,
    BlockLibrary& library,
    MaterialLibrary& materials,
    BuildWorld& world,
    Renderer& renderer
) {
    if(!state.open)return false;
    bool libraryChanged=false;

    ImGuiIO& io=ImGui::GetIO();
    ImGui::SetNextWindowPos({io.DisplaySize.x*0.5f,io.DisplaySize.y*0.5f},ImGuiCond_Always,{0.5f,0.5f});
    ImGui::SetNextWindowSize({std::min(1320.0f,io.DisplaySize.x-28.0f),std::min(820.0f,io.DisplaySize.y-28.0f)},ImGuiCond_Always);

    bool keepOpen=true;
    if(ImGui::Begin("Редактор блока",&keepOpen,ImGuiWindowFlags_NoCollapse)) {
        ImGui::SetNextItemWidth(260);ImGui::InputText("Название",&state.draft.nameRu);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(220);ImGui::InputText("Подгруппа",&state.draft.group);
        ImGui::SameLine();
        ImGui::TextDisabled("масса: %.3f кг",BlockLibrary::blockMassKg(state.draft,materials));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(92);
        ImGui::SliderInt("точность CSG",&state.draft.csgResolution,12,64);
        ImGui::Separator();

        const ImGuiTableFlags tableFlags=
            ImGuiTableFlags_BordersInnerV|
            ImGuiTableFlags_Resizable|
            ImGuiTableFlags_SizingStretchProp;

        if(ImGui::BeginTable("block_editor_layout",3,tableFlags,ImVec2(0,-62))) {
            ImGui::TableSetupColumn("Структура",ImGuiTableColumnFlags_WidthFixed,245.0f);
            ImGui::TableSetupColumn("3D-просмотр",ImGuiTableColumnFlags_WidthStretch,1.8f);
            ImGui::TableSetupColumn("Параметры",ImGuiTableColumnFlags_WidthFixed,355.0f);
            ImGui::TableNextRow();

            // Left: block component structure.
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("components_panel",{0,0},false);
            ImGui::TextUnformatted("Компоненты блока");
            ImGui::TextDisabled("Собирай форму из простых тел\nили профилей.");
            ImGui::Separator();

            for(int i=0;i<static_cast<int>(state.draft.components.size());++i) {
                auto& c=state.draft.components[static_cast<std::size_t>(i)];
                const std::string label=c.name+"##component_"+std::to_string(c.id);
                if(ImGui::Selectable(label.c_str(),state.selectedComponent==i)) {
                    state.selectedComponent=i;
                    state.profilePoint=-1;
                }
            }

            if(state.selectedComponent>=0&&state.selectedComponent<static_cast<int>(state.draft.components.size())) {
                const int selected=state.selectedComponent;

                if(ImGui::Button("↑ Выше")&&selected>0) {
                    std::swap(
                        state.draft.components[static_cast<std::size_t>(selected)],
                        state.draft.components[static_cast<std::size_t>(selected-1)]
                    );
                    --state.selectedComponent;
                }

                ImGui::SameLine();

                if(
                    ImGui::Button("↓ Ниже") &&
                    selected+1<static_cast<int>(state.draft.components.size())
                ) {
                    std::swap(
                        state.draft.components[static_cast<std::size_t>(selected)],
                        state.draft.components[static_cast<std::size_t>(selected+1)]
                    );
                    ++state.selectedComponent;
                }
            }

            ImGui::SeparatorText("Добавить форму");
            if(ImGui::Button("Куб / параллелепипед",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Box,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}
            if(ImGui::Button("Цилиндр",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Cylinder,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}
            if(ImGui::Button("Сфера",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Sphere,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}
            if(ImGui::Button("Полая труба",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Tube,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}
            if(ImGui::Button("Контур → выдавливание",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Extrude,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}
            if(ImGui::Button("Профиль → вращение",{-1,0})){state.draft.components.push_back(defaultComponent(GeometryKind::Revolve,state.draft.components.size()+1));state.selectedComponent=static_cast<int>(state.draft.components.size()-1);}

            ImGui::SeparatorText("Вставить готовый блок");
            const auto& blocks=library.blocks();
            if(!blocks.empty()) {
                state.copyBlockIndex=std::clamp(state.copyBlockIndex,0,static_cast<int>(blocks.size()-1));
                if(ImGui::BeginCombo("##copy_existing",blocks[static_cast<std::size_t>(state.copyBlockIndex)].nameRu.c_str())) {
                    for(int i=0;i<static_cast<int>(blocks.size());++i) {
                        if(ImGui::Selectable(blocks[static_cast<std::size_t>(i)].nameRu.c_str(),i==state.copyBlockIndex))
                            state.copyBlockIndex=i;
                    }
                    ImGui::EndCombo();
                }
                if(ImGui::Button("Добавить компоненты",{-1,0})) {
                    const auto& src=blocks[static_cast<std::size_t>(state.copyBlockIndex)];
                    std::uint64_t next=state.draft.components.size()+1;
                    for(auto c:src.components){c.id=next++;state.draft.components.push_back(std::move(c));}
                }
            }
            ImGui::EndChild();

            // Center: true rendered 3D preview.
            ImGui::TableSetColumnIndex(1);
            ImGui::BeginChild("preview_panel",{0,0},false,ImGuiWindowFlags_NoScrollbar);
            const ImVec2 avail=ImGui::GetContentRegionAvail();
            const ImVec2 imageSize{std::max(220.0f,avail.x),std::max(220.0f,avail.y-34.0f)};

            const std::uint32_t previewTexture=renderer.renderBlockPreview(
                state.draft,
                materials,
                static_cast<int>(imageSize.x),
                static_cast<int>(imageSize.y),
                state.previewYaw,
                state.previewPitch,
                state.previewZoom
            );

            ImGui::Image(
                ImTextureRef(static_cast<ImTextureID>(previewTexture)),
                imageSize,
                ImVec2(0,1),
                ImVec2(1,0)
            );

            if(ImGui::IsItemHovered()) {
                if(ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                    state.previewYaw-=io.MouseDelta.x*0.008f;
                    state.previewPitch=std::clamp(state.previewPitch+io.MouseDelta.y*0.008f,-1.35f,1.35f);
                }
                if(std::abs(io.MouseWheel)>0.001f) {
                    state.previewZoom=std::clamp(state.previewZoom*std::pow(0.88f,io.MouseWheel),1.8f,8.0f);
                }
            }

            ImGui::TextDisabled("ПКМ — вращать просмотр    колесо — приблизить/отдалить");
            ImGui::EndChild();

            // Right: selected component parameters.
            ImGui::TableSetColumnIndex(2);
            ImGui::BeginChild("component_properties",{0,0},false);
            if(state.selectedComponent>=0 && state.selectedComponent<static_cast<int>(state.draft.components.size())) {
                auto& c=state.draft.components[static_cast<std::size_t>(state.selectedComponent)];
                ImGui::Text("%s",kindRu(c.kind));
                ImGui::InputText("Имя",&c.name);

                const char* booleanItems[]={"Добавить","Вычесть","Пересечь"};
                int booleanIndex=static_cast<int>(c.booleanOp);
                if(ImGui::Combo("Булева операция",&booleanIndex,booleanItems,3))
                    c.booleanOp=static_cast<BooleanOp>(booleanIndex);

                if(c.booleanOp!=BooleanOp::Add)
                    ImGui::TextDisabled("Результат вычисляется через объёмную CSG-сетку.");

                ImGui::DragFloat3("Положение",&c.position.x,0.01f,-20,20,"%.3f");
                ImGui::DragFloat3("Поворот",&c.rotationDeg.x,1.0f,-360,360,"%.1f°");

                switch(c.kind) {
                    case GeometryKind::Box:
                        ImGui::DragFloat3("Размер",&c.size.x,0.01f,0.01f,20.0f,"%.3f м");
                        break;
                    case GeometryKind::Cylinder:
                        ImGui::DragFloat("Радиус",&c.radius,0.01f,0.005f,10.0f,"%.3f м");
                        ImGui::DragFloat("Высота",&c.height,0.01f,0.005f,20.0f,"%.3f м");
                        ImGui::SliderInt("Сегментов",&c.radialSegments,8,64);
                        break;
                    case GeometryKind::Sphere:
                        ImGui::DragFloat("Радиус",&c.radius,0.01f,0.005f,10.0f,"%.3f м");
                        ImGui::SliderInt("Качество",&c.radialSegments,12,48);
                        break;
                    case GeometryKind::Tube:
                        ImGui::DragFloat("Внешний радиус",&c.radius,0.01f,0.01f,10.0f,"%.3f м");
                        ImGui::DragFloat("Внутренний радиус",&c.innerRadius,0.01f,0.001f,std::max(0.002f,c.radius-0.001f),"%.3f м");
                        ImGui::DragFloat("Высота",&c.height,0.01f,0.005f,20.0f,"%.3f м");
                        ImGui::SliderInt("Сегментов",&c.radialSegments,12,64);
                        break;
                    case GeometryKind::Extrude:
                        ImGui::DragFloat("Глубина",&c.size.z,0.01f,0.005f,20.0f,"%.3f м");
                        profileEditor(c.profile,false,state);
                        break;
                    case GeometryKind::Revolve:
                        ImGui::SliderInt("Сегментов",&c.radialSegments,12,64);
                        profileEditor(c.profile,true,state);
                        break;
                }

                ImGui::Separator();
                materialCombo(c.materialId,materials);

                ImGui::Separator();
                if(ImGui::Button("Удалить компонент",{-1,0}) && state.draft.components.size()>1) {
                    state.draft.components.erase(state.draft.components.begin()+state.selectedComponent);
                    state.selectedComponent=std::min(state.selectedComponent,static_cast<int>(state.draft.components.size())-1);
                }
            } else {
                ImGui::TextWrapped(
                    "Выбери компонент слева. Центральное окно всегда показывает итоговый блок, "
                    "поэтому сложную форму можно собирать визуально, а не только по числам."
                );
            }
            ImGui::EndChild();

            ImGui::EndTable();
        }

        ImGui::Separator();
        if(state.editingInstanceId) {
            if(ImGui::Button("Применить к экземпляру")) {
                world.applyLocalOverride(state.editingInstanceId,state.draft,library);
                state.open=false;
            }
            ImGui::SameLine();
            if(ImGui::Button("Сохранить как новый блок")) {
                BlockDefinition copy=state.draft;
                copy.id=library.makeUniqueId(copy.nameRu);
                library.upsert(std::move(copy));
                libraryChanged=true;
            }
        } else {
            if(ImGui::Button("Сохранить в библиотеку")) {
                if(state.draft.components.empty())state.draft.components.push_back(defaultComponent(GeometryKind::Box,1));
                if(state.draft.id.empty())state.draft.id=library.makeUniqueId(state.draft.nameRu);
                library.upsert(state.draft);
                libraryChanged=true;
                state.open=false;
            }
        }
        ImGui::SameLine();
        if(ImGui::Button("Закрыть"))state.open=false;
    }
    ImGui::End();

    if(!keepOpen)state.open=false;
    return libraryChanged;
}

void drawDebugPressure(PressureChamberLab& lab) {
    ImGui::Begin("Физическая лаборатория");
    ImGui::TextWrapped("Отладочный слой: давление возникает из состояния среды и геометрии объёма, а не из готового блока «гидроцилиндр».");
    float opening=static_cast<float>(lab.openingArea*1e6);
    if(ImGui::SliderFloat("Отверстие, мм²",&opening,0,300,"%.1f"))lab.openingArea=opening*1e-6;
    ImGui::Text("Давление %.2f кПа | сила %.1f Н",lab.pressure()/1000.0,lab.pressureForce());
    if(ImGui::Button("Сброс"))lab.reset();
    ImGui::End();
}

bool basicHotbarButton(const BlockDefinition& b,bool active) {
    if(active)ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.30f,0.48f,0.67f,1.0f));
    const bool r=ImGui::Button(b.nameRu.c_str(),{92,46});
    if(active)ImGui::PopStyleColor();
    return r;
}

Vec3 sceneCenter(const BuildWorld& world,const BlockLibrary& library) {
    Aabb bounds;
    for(const auto& instance:world.instances())
        bounds.include(world.worldBounds(instance,library));
    return bounds.valid()?bounds.center():Vec3{};
}

BlockDefinition makeBlockFromScene(
    const BuildWorld& world,
    const BlockLibrary& library,
    const std::string& name,
    const std::string& group
) {
    BlockDefinition result;
    result.nameRu=name.empty()?"Новый блок":name;
    result.group=group.empty()?"Базовые блоки":group;

    const Vec3 center=sceneCenter(world,library);
    std::uint64_t nextId=1;
    int nextGroup=0;

    for(const auto& instance:world.instances()) {
        const auto* source=world.definitionFor(instance,library);
        if(!source)continue;

        std::unordered_map<int,int> groups;
        for(auto component:source->components) {
            auto [it,inserted]=groups.emplace(component.operationGroup,nextGroup);
            if(inserted)++nextGroup;
            component.operationGroup=it->second;
            component.id=nextId++;

            const Vec3 local=component.position*instance.transform.scale;
            component.position=
                instance.transform.position+
                rotateVectorEulerDeg(local,instance.transform.rotationDeg)-
                center;
            component.rotationDeg+=instance.transform.rotationDeg;
            component.scale=component.scale*instance.transform.scale;
            result.components.push_back(std::move(component));
        }
    }
    return result;
}

void enterWorkshop(
    BlockEditorState& editor,
    BuildWorld& world,
    Camera& camera,
    const BlockLibrary& library,
    bool fromSelection
) {
    editor.savedWorld=world;
    editor.savedCamera=camera;
    editor.hasBackup=true;
    editor.active=true;
    editor.nameRu=fromSelection?"Блок из выделения":"Новый блок";
    editor.group="Базовые блоки";
    editor.resetInspector();

    BuildWorld workshop;
    if(fromSelection&&!world.selection().empty()) {
        const Vec3 center=world.selectionCenter(library);
        std::unordered_map<std::uint64_t,std::uint64_t> ids;

        for(const auto& instance:world.instances()) {
            if(!world.isSelected(instance.id))continue;
            ids[instance.id]=workshop.importInstance(instance,center*-1.0f);
        }

        for(const auto& instance:world.instances()) {
            if(!world.isSelected(instance.id))continue;
            for(std::uint64_t other:instance.attachments) {
                if(other<=instance.id || !ids.contains(other))continue;
                workshop.setAttachment(ids[instance.id],ids[other],true,library);
            }
        }
    }

    world=std::move(workshop);
    camera=Camera{};
    camera.target={0.0f,0.5f,0.0f};
    camera.distance=8.0f;
}

void leaveWorkshop(BlockEditorState& editor,BuildWorld& world,Camera& camera) {
    if(editor.hasBackup) {
        world=std::move(editor.savedWorld);
        camera=editor.savedCamera;
    }
    editor.active=false;
    editor.hasBackup=false;
    editor.resetInspector();
}

void drawGeometryInspector(
    BlockEditorState& editor,
    BuildWorld& world,
    BlockLibrary& library,
    MaterialLibrary& materials,
    float width
) {
    if(!editor.geometryInspector||world.selection().size()!=1)return;
    const auto id=*world.selection().begin();
    auto* instance=world.find(id);
    if(!instance)return;
    const auto* source=world.definitionFor(*instance,library);
    if(!source||source->components.empty())return;

    BlockDefinition edited=*source;
    editor.selectedComponent=std::clamp(
        editor.selectedComponent,0,static_cast<int>(edited.components.size())-1);

    ImGui::SetNextWindowPos({width-450.0f,375.0f},ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({435.0f,505.0f},ImGuiCond_FirstUseEver);
    bool open=editor.geometryInspector;
    bool changed=false;

    if(ImGui::Begin("Геометрия блока",&open)) {
        for(int i=0;i<static_cast<int>(edited.components.size());++i) {
            auto& c=edited.components[static_cast<std::size_t>(i)];
            if(ImGui::Selectable((c.name+"##geom"+std::to_string(c.id)).c_str(),
                                 editor.selectedComponent==i))
                editor.selectedComponent=i;
        }

        auto add=[&](GeometryKind kind,const char* label){
            if(ImGui::Button(label)) {
                edited.components.push_back(defaultComponent(kind,edited.components.size()+1));
                editor.selectedComponent=static_cast<int>(edited.components.size())-1;
                changed=true;
            }
        };
        add(GeometryKind::Box,"+ Куб"); ImGui::SameLine();
        add(GeometryKind::Cylinder,"+ Цилиндр"); ImGui::SameLine();
        add(GeometryKind::Sphere,"+ Сфера");
        add(GeometryKind::Tube,"+ Труба"); ImGui::SameLine();
        add(GeometryKind::Extrude,"+ Контур"); ImGui::SameLine();
        add(GeometryKind::Revolve,"+ Вращение");

        auto& c=edited.components[static_cast<std::size_t>(editor.selectedComponent)];
        ImGui::Separator();
        changed|=ImGui::InputText("Имя",&c.name);

        const char* ops[]={
            "Объединить","Вычесть","Пересечь",
            "Выпуклая оболочка","Сумма Минковского","Разность Минковского"
        };
        int op=static_cast<int>(c.booleanOp);
        if(ImGui::Combo("Операция",&op,ops,6)) {
            c.booleanOp=static_cast<BooleanOp>(op);
            changed=true;
        }

        changed|=ImGui::DragFloat3("Положение",&c.position.x,0.01f,-50,50,"%.3f");
        changed|=ImGui::DragFloat3("Поворот",&c.rotationDeg.x,1.0f,-360,360,"%.1f°");
        changed|=ImGui::DragFloat3("Масштаб формы",&c.scale.x,0.01f,0.01f,20,"%.3f");

        if(c.kind==GeometryKind::Box)
            changed|=ImGui::DragFloat3("Размер",&c.size.x,0.01f,0.001f,50,"%.3f м");
        else if(c.kind==GeometryKind::Cylinder||c.kind==GeometryKind::Tube) {
            changed|=ImGui::DragFloat("Радиус",&c.radius,0.01f,0.001f,20,"%.3f м");
            if(c.kind==GeometryKind::Tube)
                changed|=ImGui::DragFloat("Внутренний радиус",&c.innerRadius,0.01f,0.001f,c.radius,"%.3f м");
            changed|=ImGui::DragFloat("Высота",&c.height,0.01f,0.001f,50,"%.3f м");
            changed|=ImGui::SliderInt("Сегментов",&c.radialSegments,16,192);
        } else if(c.kind==GeometryKind::Sphere) {
            changed|=ImGui::DragFloat("Радиус",&c.radius,0.01f,0.001f,20,"%.3f м");
            changed|=ImGui::SliderInt("Сегментов",&c.radialSegments,16,192);
        } else {
            const char* planes[]={"XY","XZ","YZ"};
            int plane=static_cast<int>(c.profilePlane);
            if(ImGui::Combo("Плоскость контура",&plane,planes,3)) {
                c.profilePlane=static_cast<ProfilePlane>(plane);
                changed=true;
            }
            if(c.kind==GeometryKind::Extrude)
                changed|=ImGui::DragFloat("Глубина",&c.size.z,0.01f,0.001f,50,"%.3f м");
            else
                changed|=ImGui::SliderInt("Сегментов",&c.radialSegments,16,192);

            const auto oldProfile=c.profile;
            profileEditor(c.profile,c.kind==GeometryKind::Revolve,editor);
            if(oldProfile!=c.profile)changed=true;
        }

        int materialIndex=materials.indexOf(c.materialId);
        const auto& all=materials.all();
        if(ImGui::BeginCombo("Материал компонента",all[static_cast<std::size_t>(materialIndex)].nameRu.c_str())) {
            for(int i=0;i<static_cast<int>(all.size());++i) {
                if(ImGui::Selectable(all[static_cast<std::size_t>(i)].nameRu.c_str(),i==materialIndex)) {
                    c.materialId=all[static_cast<std::size_t>(i)].id;
                    changed=true;
                }
            }
            ImGui::EndCombo();
        }

        if(edited.components.size()>1&&ImGui::Button("Удалить компонент",{-1,0})) {
            edited.components.erase(edited.components.begin()+editor.selectedComponent);
            editor.selectedComponent=std::min(
                editor.selectedComponent,static_cast<int>(edited.components.size())-1);
            changed=true;
        }
    }
    ImGui::End();
    editor.geometryInspector=open;

    if(changed)world.applyLocalOverride(id,std::move(edited),library);
}

} // namespace

int main(int,char**) {
    if(!SDL_Init(SDL_INIT_VIDEO)){std::fprintf(stderr,"SDL_Init: %s\n",SDL_GetError());return 1;}

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,24);

    SDL_Window* window=SDL_CreateWindow("Mechanica 0.5",1500,900,SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
    if(!window){SDL_Quit();return 1;}
    SDL_GLContext gl=SDL_GL_CreateContext(window);
    if(!gl){SDL_DestroyWindow(window);SDL_Quit();return 1;}
    SDL_GL_MakeCurrent(window,gl);SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();ImGui::CreateContext();
    ImGuiIO& io=ImGui::GetIO();io.ConfigFlags|=ImGuiConfigFlags_NavEnableKeyboard;

    ImFont* ru=io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf",18.0f,nullptr,io.Fonts->GetGlyphRangesCyrillic());
    if(!ru)ru=io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf",18.0f,nullptr,io.Fonts->GetGlyphRangesCyrillic());

    ImGui::StyleColorsDark();
    auto& style=ImGui::GetStyle();style.WindowRounding=8;style.FrameRounding=5;style.WindowPadding={12,10};

    ImGui_ImplSDL3_InitForOpenGL(window,gl);ImGui_ImplOpenGL3_Init("#version 330");

    Renderer renderer;if(!renderer.initialize()){return 1;}
    MaterialLibrary materials;
    BlockLibrary library(std::filesystem::current_path()/"user_data"/"block_library.json");
    library.loadOrCreateDefaults();

    BuildWorld world;
    PhysicsWorld physics;
    PressureChamberLab pressureLab;
    Camera camera;
    BlockEditorState blockEditor;
    ToolState tools;

    bool running=true,simulating=false,showLibrary=true,showDebug=false,rightMouse=false;
    bool requestClick=false,requestToggleSim=false;
    bool requestSaveWorkshop=false,requestCancelWorkshop=false;

    bool placedConnectionEdit=false;
    std::uint64_t placedConnectionSource=0;
    std::unordered_map<std::uint64_t,bool> placedConnectionChoices;

    bool attachmentEdit=false;
    bool altWasDown=false;
    float attachmentMouseX=0.0f;
    float attachmentMouseY=0.0f;
    PlacementPreview frozenPlacement;

    double accumulator=0;constexpr double fixedDt=1.0/120.0;
    auto previous=Clock::now();std::vector<PartRootPose> poses;
    std::unordered_map<std::uint64_t,bool> attachChoices;
    std::vector<std::uint64_t> lastTouching;

    while(running) {
        SDL_Event e;
        while(SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);
            if(e.type==SDL_EVENT_QUIT)running=false;
            if(e.type==SDL_EVENT_WINDOW_CLOSE_REQUESTED && e.window.windowID==SDL_GetWindowID(window))running=false;

            if(e.type==SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if(e.button.button==SDL_BUTTON_LEFT)requestClick=true;
                if(e.button.button==SDL_BUTTON_RIGHT)rightMouse=true;
            }
            if(e.type==SDL_EVENT_MOUSE_BUTTON_UP&&e.button.button==SDL_BUTTON_RIGHT)rightMouse=false;
            if(e.type==SDL_EVENT_MOUSE_MOTION&&rightMouse&&!attachmentEdit&&!blockEditor.open&&!io.WantCaptureMouse)camera.orbit(e.motion.xrel,e.motion.yrel);
            if(e.type==SDL_EVENT_MOUSE_WHEEL&&!io.WantCaptureMouse)camera.zoom(e.wheel.y);

            if(e.type==SDL_EVENT_KEY_DOWN&&!e.key.repeat) {
                switch(e.key.scancode) {
                    case SDL_SCANCODE_ESCAPE: world.cancelPlacement();break;
                    case SDL_SCANCODE_B: showLibrary=!showLibrary;break;
                    case SDL_SCANCODE_G: if(!simulating)world.setGridEnabled(!world.gridEnabled());break;
                    case SDL_SCANCODE_R: if(!simulating&&world.placing())world.rotatePlacementY();break;
                    case SDL_SCANCODE_DELETE:
                    case SDL_SCANCODE_X: if(!simulating&&!world.placing())world.deleteSelected();break;
                    case SDL_SCANCODE_SPACE: requestToggleSim=true;break;
                    case SDL_SCANCODE_F2:showDebug=!showDebug;break;
                    default:break;
                }
            }
        }

        const auto now=Clock::now();double frameDt=std::min(0.10,std::chrono::duration<double>(now-previous).count());previous=now;
        const bool* keys=SDL_GetKeyboardState(nullptr);

        if(!io.WantCaptureKeyboard && !blockEditor.open) {
            if(world.selection().empty() || world.placing()) {
                float f=0,r=0;if(keys[SDL_SCANCODE_W])f+=1;if(keys[SDL_SCANCODE_S])f-=1;if(keys[SDL_SCANCODE_D])r+=1;if(keys[SDL_SCANCODE_A])r-=1;
                camera.pan(f,r,static_cast<float>(frameDt));
            } else if(!simulating) {
                const float step=world.gridEnabled()?world.gridStep():0.05f;
                const float speed=(keys[SDL_SCANCODE_LSHIFT]||keys[SDL_SCANCODE_RSHIFT])?0.2f:1.0f;
                Vec3 d{};
                if(keys[SDL_SCANCODE_LEFT])d.x-=step*speed;
                if(keys[SDL_SCANCODE_RIGHT])d.x+=step*speed;
                if(keys[SDL_SCANCODE_UP])d.z-=step*speed;
                if(keys[SDL_SCANCODE_DOWN])d.z+=step*speed;
                if(keys[SDL_SCANCODE_PAGEUP])d.y+=step*speed;
                if(keys[SDL_SCANCODE_PAGEDOWN])d.y-=step*speed;
                if(lengthSq(d)>0)world.translateSelected(d);
            }
        }

        if(simulating) {
            accumulator+=frameDt;
            while(accumulator>=fixedDt){physics.step(static_cast<float>(fixedDt));pressureLab.step(fixedDt);accumulator-=fixedDt;}
            physics.fillPartRootPoses(world.instances().size(),poses);
        } else {accumulator=0;poses.clear();}

        int ww=0,wh=0,pw=0,ph=0;SDL_GetWindowSize(window,&ww,&wh);SDL_GetWindowSizeInPixels(window,&pw,&ph);
        float mx=0,my=0;SDL_GetMouseState(&mx,&my);
        const Ray ray=camera.mouseRay(mx,my,static_cast<float>(ww),static_cast<float>(wh));

        PlacementPreview preview;
        if(!simulating&&world.placing())preview=world.preview(ray,library);

        const bool altDown=keys[SDL_SCANCODE_LALT]||keys[SDL_SCANCODE_RALT];

        if(!simulating&&world.placing()&&altDown&&!altWasDown&&!attachmentEdit&&preview.hasCandidate&&preview.valid) {
            attachmentEdit=true;
            frozenPlacement=preview;
            attachmentMouseX=mx;
            attachmentMouseY=my;
            attachChoices.clear();
            for(auto id:frozenPlacement.touchingIds)attachChoices[id]=true;
            lastTouching=frozenPlacement.touchingIds;
        }

        if(attachmentEdit&&!altDown&&altWasDown) {
            attachmentEdit=false;
            SDL_WarpMouseInWindow(window,attachmentMouseX,attachmentMouseY);
        }

        if(!attachmentEdit&&preview.touchingIds!=lastTouching) {
            attachChoices.clear();
            for(auto id:preview.touchingIds)attachChoices[id]=true;
            lastTouching=preview.touchingIds;
        }

        const PlacementPreview* activePreview=nullptr;
        if(!simulating&&world.placing())
            activePreview=attachmentEdit?&frozenPlacement:&preview;

        PlacementPreview placedConnectionPreview;
        const PlacementPreview* renderPreview=activePreview;
        const std::unordered_map<std::uint64_t,bool>* renderAttachmentChoices=
            activePreview?&attachChoices:nullptr;

        if(!simulating&&!world.placing()&&placedConnectionEdit&&world.find(placedConnectionSource)) {
            placedConnectionPreview.touchingIds=world.touchingIds(placedConnectionSource,library);
            if(const auto* source=world.find(placedConnectionSource)) {
                for(std::uint64_t attached:source->attachments)
                    if(std::find(placedConnectionPreview.touchingIds.begin(),
                                 placedConnectionPreview.touchingIds.end(),attached)
                       ==placedConnectionPreview.touchingIds.end())
                        placedConnectionPreview.touchingIds.push_back(attached);
            }
            placedConnectionChoices.clear();
            for(auto id:placedConnectionPreview.touchingIds)
                placedConnectionChoices[id]=world.isAttached(placedConnectionSource,id);
            renderPreview=&placedConnectionPreview;
            renderAttachmentChoices=&placedConnectionChoices;
        }

        std::vector<PlacementPreview> symmetryPreviews;
        std::vector<PlacementPreview> extraSymmetryPreviews;
        if(activePreview&&!attachmentEdit) {
            symmetryPreviews=makeSymmetryPreviews(*activePreview,tools,world,library);
            if(symmetryPreviews.size()>1)
                extraSymmetryPreviews.assign(symmetryPreviews.begin()+1,symmetryPreviews.end());
        }

        const float aspect=static_cast<float>(pw)/std::max(ph,1);
        const Mat4 view=camera.view();
        const Mat4 projection=camera.projection(aspect);
        const Mat4 vp=projection*view;

        glViewport(0,0,pw,ph);glClearColor(0.065f,0.075f,0.095f,1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        renderer.render(
            world,
            library,
            materials,
            vp,
            renderPreview,
            extraSymmetryPreviews.empty()?nullptr:&extraSymmetryPreviews,
            renderAttachmentChoices,
            simulating?&poses:nullptr
        );

        ImGui_ImplOpenGL3_NewFrame();ImGui_ImplSDL3_NewFrame();ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        // Верхняя игровая строка и инструменты.
        ImVec2 toolSettingsPos{14.0f,72.0f};
        ImGui::SetNextWindowPos({14,14},ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin(
            "##top",
            nullptr,
            ImGuiWindowFlags_NoTitleBar|
            ImGuiWindowFlags_AlwaysAutoResize|
            ImGuiWindowFlags_NoMove|
            ImGuiWindowFlags_NoSavedSettings
        );

        ImGui::TextUnformatted(blockEditor.active?"MECHANICA — РЕДАКТОР БЛОКА":"MECHANICA");
        ImGui::SameLine();

        if(blockEditor.active) {
            ImGui::SetNextItemWidth(180);
            ImGui::InputText("##workshop_name",&blockEditor.nameRu);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(150);
            ImGui::InputText("##workshop_group",&blockEditor.group);
            ImGui::SameLine();
            if(ImGui::Button("Сохранить"))requestSaveWorkshop=true;
            ImGui::SameLine();
            if(ImGui::Button("Отмена"))requestCancelWorkshop=true;
            ImGui::SameLine(); ImGui::TextDisabled("|"); ImGui::SameLine();
        }

        if(ImGui::Button(simulating?"Вернуться к сборке":"▶ Запустить"))
            requestToggleSim=true;

        ImGui::SameLine();
        if(!simulating && ImGui::Button(showLibrary?"Скрыть библиотеку":"Библиотека [B]"))
            showLibrary=!showLibrary;

        if(!simulating&&!blockEditor.open) {
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            auto transformTool=[&](const char* label,ToolPanel panel){
                const bool active=tools.panel==panel;
                if(active)ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.29f,0.49f,0.70f,1));
                const bool clicked=ImGui::Button(label);
                if(active)ImGui::PopStyleColor();
                if(clicked)tools.panel=active?ToolPanel::None:panel;
                if(tools.panel==panel)
                    toolSettingsPos={ImGui::GetItemRectMin().x,ImGui::GetItemRectMax().y+5};
            };

            if(tools.symmetryEnabled)
                ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.29f,0.49f,0.70f,1));
            const bool symmetryClicked=ImGui::Button("Симметрия");
            if(tools.symmetryEnabled)ImGui::PopStyleColor();
            if(symmetryClicked) {
                tools.symmetryEnabled=!tools.symmetryEnabled;
                tools.panel=tools.symmetryEnabled?ToolPanel::Symmetry:
                    (tools.panel==ToolPanel::Symmetry?ToolPanel::None:tools.panel);
            }
            if(tools.panel==ToolPanel::Symmetry)
                toolSettingsPos={ImGui::GetItemRectMin().x,ImGui::GetItemRectMax().y+5};

            ImGui::SameLine(); transformTool("Поворот",ToolPanel::Rotate);
            ImGui::SameLine(); transformTool("Перемещение",ToolPanel::Move);
            ImGui::SameLine(); transformTool("Масштаб",ToolPanel::Scale);

            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            bool grid=world.gridEnabled();
            if(ImGui::Checkbox("Сетка [G]",&grid))
                world.setGridEnabled(grid);

            ImGui::SameLine();
            float gs=world.gridStep();
            ImGui::SetNextItemWidth(72);
            if(ImGui::DragFloat("шаг сетки",&gs,0.01f,0.005f,2.0f,"%.2f"))
                world.setGridStep(gs);
        }

        ImGui::SameLine();
        if(simulating) {
            ImGui::TextDisabled(
                "%zu блоков | %zu тел | %zu связей | %.1f МПа | ε %.4g | текучесть %zu | разрушено %zu | %.0f FPS",
                world.instances().size(),
                physics.assemblyBodyCount(),
                physics.structuralLinkCount(),
                physics.maxStressMPa(),
                physics.maxElasticStrain(),
                physics.yieldedLinkCount(),
                physics.brokenLinkCount(),
                io.Framerate
            );
        } else {
            ImGui::TextDisabled(
                "%zu блоков | %.0f FPS",
                world.instances().size(),
                io.Framerate
            );
        }

        ImGui::End();

        if(!simulating&&!blockEditor.open&&tools.panel!=ToolPanel::None) {
            ImGui::SetNextWindowPos(toolSettingsPos,ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.93f);

            ImGui::Begin(
                "##tool_settings",
                nullptr,
                ImGuiWindowFlags_NoTitleBar|
                ImGuiWindowFlags_AlwaysAutoResize|
                ImGuiWindowFlags_NoMove|
                ImGuiWindowFlags_NoSavedSettings
            );

            if(tools.panel==ToolPanel::Symmetry) {
                ImGui::TextUnformatted("Оси симметрии");

                auto axisButton=[&](const char* label,bool& value){
                    if(value)ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.30f,0.62f,0.78f,1.0f));
                    if(ImGui::Button(label,{42,34}))value=!value;
                    if(value)ImGui::PopStyleColor();
                };

                axisButton("X",tools.symmetryX);
                ImGui::SameLine();
                axisButton("Y",tools.symmetryY);
                ImGui::SameLine();
                axisButton("Z",tools.symmetryZ);

                ImGui::TextDisabled("Зеркальные копии строятся\nотносительно мировых плоскостей.");
            }
            else if(tools.panel==ToolPanel::Move) {
                ImGui::TextUnformatted("Шаг перемещения");
                ImGui::SetNextItemWidth(150);
                ImGui::DragFloat("##move_step",&tools.moveStep,0.005f,0.001f,10.0f,"%.3f м");
            }
            else if(tools.panel==ToolPanel::Rotate) {
                ImGui::TextUnformatted("Шаг вращения");
                ImGui::SetNextItemWidth(150);
                ImGui::DragFloat("##rotate_step",&tools.rotateStep,0.5f,0.1f,90.0f,"%.1f°");
            }
            else if(tools.panel==ToolPanel::Scale) {
                ImGui::TextUnformatted("Шаг масштаба");
                ImGui::SetNextItemWidth(150);
                ImGui::DragFloat("##scale_step",&tools.scaleStep,0.01f,0.001f,2.0f,"%.3f");
            }

            ImGui::End();
        }

        // Настоящий 3D-гизмо для выбранного блока или группы.
        if(
            !simulating &&
            !blockEditor.open &&
            !world.placing() &&
            !world.selection().empty() &&
            (tools.panel==ToolPanel::Move || tools.panel==ToolPanel::Rotate || tools.panel==ToolPanel::Scale)
        ) {
            const Vec3 pivot=world.selectionCenter(library);

            Mat4 gizmoMatrix=translation(pivot);
            Mat4 delta=Mat4::identity();

            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
            ImGuizmo::SetRect(0.0f,0.0f,static_cast<float>(ww),static_cast<float>(wh));

            ImGuizmo::OPERATION operation=ImGuizmo::TRANSLATE;
            float snapValues[3]={tools.moveStep,tools.moveStep,tools.moveStep};

            if(tools.panel==ToolPanel::Rotate) {
                operation=ImGuizmo::ROTATE;
                snapValues[0]=tools.rotateStep;
            }
            else if(tools.panel==ToolPanel::Scale) {
                operation=ImGuizmo::SCALE;
                snapValues[0]=tools.scaleStep;
            }

            const bool manipulated=ImGuizmo::Manipulate(
                view.m,
                projection.m,
                operation,
                ImGuizmo::WORLD,
                gizmoMatrix.m,
                delta.m,
                snapValues
            );

            if(manipulated&&ImGuizmo::IsUsing()) {
                float dt[3]{},dr[3]{},ds[3]{1,1,1};
                ImGuizmo::DecomposeMatrixToComponents(delta.m,dt,dr,ds);

                if(tools.panel==ToolPanel::Move) {
                    world.translateSelected({dt[0],dt[1],dt[2]});
                }
                else if(tools.panel==ToolPanel::Rotate) {
                    world.rotateSelectedAround(pivot,{dr[0],dr[1],dr[2]});
                }
                else if(tools.panel==ToolPanel::Scale) {
                    world.scaleSelectedAround(
                        pivot,
                        {
                            std::max(ds[0],0.001f),
                            std::max(ds[1],0.001f),
                            std::max(ds[2],0.001f)
                        }
                    );
                }
            }
        }

        if(!simulating&&showLibrary&&!blockEditor.open) {
            ImGui::SetNextWindowPos({14,82},ImGuiCond_FirstUseEver);ImGui::SetNextWindowSize({270,520},ImGuiCond_FirstUseEver);
            ImGui::Begin("Библиотека",&showLibrary);
            if(ImGui::Button("+ Создать свой блок",{-1,0}))blockEditor.newBlock();
            ImGui::Separator();
            for(const auto& group:library.groups()) {
                if(ImGui::CollapsingHeader(group.c_str(),ImGuiTreeNodeFlags_DefaultOpen)) {
                    for(const auto& b:library.blocks())if(b.group==group) {
                        const bool active=world.placing()&&world.placementDefinitionId()==b.id;
                        if(active)ImGui::PushStyleColor(ImGuiCol_Button,ImVec4(0.28f,0.47f,0.68f,1));
                        if(ImGui::Button((b.nameRu+"##lib_"+b.id).c_str(),{-1,36}))world.beginPlacement(b.id);
                        if(active)ImGui::PopStyleColor();
                    }
                }
            }
            ImGui::End();
        }

        // Хотбар содержит только начальные базовые формы.
        if(!simulating&&!blockEditor.open) {
            std::vector<const BlockDefinition*> basics;
            for(const auto& b:library.blocks())if(b.group=="Базовые блоки"&&basics.size()<6)basics.push_back(&b);
            const float width=std::max(100.0f,static_cast<float>(basics.size())*98.0f+16.0f);
            ImGui::SetNextWindowPos({ww*0.5f,wh-16.0f},ImGuiCond_Always,{0.5f,1.0f});
            ImGui::SetNextWindowSize({width,68},ImGuiCond_Always);ImGui::SetNextWindowBgAlpha(0.84f);
            ImGui::Begin("##hotbar",nullptr,ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoSavedSettings);
            for(std::size_t i=0;i<basics.size();++i){if(i)ImGui::SameLine();if(basicHotbarButton(*basics[i],world.placing()&&world.placementDefinitionId()==basics[i]->id))world.beginPlacement(basics[i]->id);}
            ImGui::End();
        }

        // Контекст размещения. Связи редактируются прямо в мире через Alt.
        if(!simulating&&world.placing()&&!blockEditor.open) {
            ImGui::SetNextWindowPos({static_cast<float>(ww)-14.0f,82},ImGuiCond_Always,{1,0});
            ImGui::SetNextWindowBgAlpha(0.84f);
            ImGui::Begin("Размещение",nullptr,ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoSavedSettings);

            if(const auto* d=library.find(world.placementDefinitionId()))
                ImGui::Text("%s",d->nameRu.c_str());

            Vec3 scale=world.placementScale();
            float scaleValues[3]={scale.x,scale.y,scale.z};
            if(ImGui::DragFloat3("Масштаб",scaleValues,0.02f,0.02f,20.0f,"%.2f"))
                world.setPlacementScale({scaleValues[0],scaleValues[1],scaleValues[2]});

            if(ImGui::Button("Повернуть [R]"))
                world.rotatePlacementY();
            ImGui::SameLine();
            if(ImGui::Button("Выбор [Esc]"))
                world.cancelPlacement();

            ImGui::Separator();

            const PlacementPreview* shownPlacement=attachmentEdit?&frozenPlacement:&preview;
            const std::size_t contacts=shownPlacement?shownPlacement->touchingIds.size():0;

            if(attachmentEdit) {
                ImGui::TextColored(
                    ImVec4(1.0f,0.78f,0.25f,1.0f),
                    "ALT: блок зафиксирован"
                );
                ImGui::TextWrapped(
                    "Щёлкай ЛКМ по подсвеченным соседям: "
                    "зелёный — связь будет создана, красный — связи не будет."
                );
                ImGui::TextDisabled("Отпусти Alt — курсор вернётся к точке установки.");
            } else if(contacts>0) {
                ImGui::Text("Будущих соединений: %zu",contacts);
                ImGui::TextDisabled(
                    "Соседи подсвечены зелёным.\n"
                    "Удерживай Alt, чтобы исключить или вернуть отдельные связи."
                );
            } else {
                ImGui::TextDisabled("Сейчас блок ни с чем не соприкасается.");
            }

            if(tools.symmetryEnabled&&(tools.symmetryX||tools.symmetryY||tools.symmetryZ)) {
                ImGui::Separator();
                ImGui::TextDisabled(
                    "Симметрия: %s%s%s",
                    tools.symmetryX?"X ":"",
                    tools.symmetryY?"Y ":"",
                    tools.symmetryZ?"Z":""
                );
            }

            ImGui::End();
        }

        // Контекст выделения.
        if(!simulating&&!world.placing()&&!world.selection().empty()&&!blockEditor.open) {
            ImGui::SetNextWindowPos({static_cast<float>(ww)-14.0f,82},ImGuiCond_Always,{1,0});ImGui::SetNextWindowBgAlpha(0.86f);
            ImGui::Begin("Выделение",nullptr,ImGuiWindowFlags_AlwaysAutoResize|ImGuiWindowFlags_NoSavedSettings);
            ImGui::Text("Выбрано: %zu",world.selection().size());
            ImGui::TextDisabled("Ctrl + ЛКМ — добавить к выбору\nСтрелки — X/Z, PgUp/PgDn — Y");

            if(world.selection().size()==1) {
                auto id=*world.selection().begin();auto* inst=world.find(id);
                if(inst) {
                    const auto* def=world.definitionFor(*inst,library);
                    ImGui::Separator();
                    ImGui::Text("%s  (#%llu)",def?def->nameRu.c_str():"Блок",static_cast<unsigned long long>(id));
                    float pos[3]={inst->transform.position.x,inst->transform.position.y,inst->transform.position.z};
                    if(ImGui::DragFloat3("Позиция",pos,0.01f,-1000,1000,"%.3f"))world.setSingleSelectedPosition({pos[0],pos[1],pos[2]});
                    float rot[3]={inst->transform.rotationDeg.x,inst->transform.rotationDeg.y,inst->transform.rotationDeg.z};
                    if(ImGui::DragFloat3("Поворот",rot,1.0f,-360.0f,360.0f,"%.1f°"))world.setSingleSelectedRotation({rot[0],rot[1],rot[2]});
                    float sc[3]={inst->transform.scale.x,inst->transform.scale.y,inst->transform.scale.z};
                    if(ImGui::DragFloat3("Масштаб",sc,0.01f,0.02f,20.0f,"%.3f"))world.setSingleSelectedScale({sc[0],sc[1],sc[2]});

                    if(def) {
                        const double scaleVol=std::abs(inst->transform.scale.x*inst->transform.scale.y*inst->transform.scale.z);
                        ImGui::Text("Масса: %.3f кг",BlockLibrary::blockMassKg(*def,materials)*scaleVol);
                        ImGui::Text("Компонентов: %zu | соединений: %zu",def->components.size(),inst->attachments.size());
                        if(ImGui::Button("Редактировать компоненты",{-1,0}))blockEditor.editInstance(id,*def);
                        if(inst->localOverride&&ImGui::Button("Вернуть шаблон из библиотеки",{-1,0}))world.resetLocalOverride(id);
                    }
                }
            }

            if(ImGui::Button("Удалить выбранное [Del]",{-1,0}))world.deleteSelected();
            ImGui::End();
        }

        if(showDebug)drawDebugPressure(pressureLab);
        drawBlockEditor(blockEditor,library,materials,world,renderer);

        // Игровой клик применяется после UI.
        if(requestClick&&!simulating&&!blockEditor.open&&!io.WantCaptureMouse) {
            if(world.placing()) {
                if(attachmentEdit) {
                    const std::uint64_t clicked=world.pick(ray,library);
                    const bool allowed=
                        std::find(
                            frozenPlacement.touchingIds.begin(),
                            frozenPlacement.touchingIds.end(),
                            clicked
                        )!=frozenPlacement.touchingIds.end();

                    if(allowed)
                        attachChoices[clicked]=!attachChoices[clicked];
                }
                else {
                    std::vector<PlacementPreview> placements=
                        symmetryPreviews.empty()
                            ? std::vector<PlacementPreview>{preview}
                            : symmetryPreviews;

                    for(std::size_t i=0;i<placements.size();++i) {
                        const auto& candidate=placements[i];
                        if(!candidate.hasCandidate||!candidate.valid)continue;

                        std::vector<std::uint64_t> attach;

                        if(i==0) {
                            for(auto [id,on]:attachChoices)
                                if(on)attach.push_back(id);
                        } else {
                            // Зеркальная копия по умолчанию прикрепляется ко всем
                            // объектам, которых она действительно касается.
                            attach=candidate.touchingIds;
                        }

                        world.place(candidate,attach,library);
                    }
                }
            }
            else if(!ImGuizmo::IsUsing()&&!ImGuizmo::IsOver()) {
                const auto id=world.pick(ray,library);
                const bool additive=keys[SDL_SCANCODE_LCTRL]||keys[SDL_SCANCODE_RCTRL];
                world.select(id,additive);
            }
        }
        requestClick=false;

        if(requestToggleSim&&!blockEditor.open) {
            if(simulating){physics.clearAssemblies();simulating=false;poses.clear();}
            else if(!world.instances().empty()){world.pruneInvalidAttachments(library);physics.buildAssemblies(world,library,materials);simulating=physics.assemblyBodyCount()>0;}
            requestToggleSim=false;
        }

        altWasDown=altDown;

        ImGui::Render();ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());SDL_GL_SwapWindow(window);
    }

    renderer.shutdown();ImGui_ImplOpenGL3_Shutdown();ImGui_ImplSDL3_Shutdown();ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl);SDL_DestroyWindow(window);SDL_Quit();
    return 0;
}
