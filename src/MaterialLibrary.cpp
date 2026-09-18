#include "MaterialLibrary.hpp"

#include <stdexcept>

namespace mechanica {

MaterialLibrary::MaterialLibrary() {
    auto add = [&](const char* id, const char* name, double rho, double E_GPa, double nu,
                   double yieldMPa, double tensileMPa, double friction, double restitution,
                   double k, double cp, double resistivity, Vec3 color, const char* note) {
        MaterialDefinition m;
        m.id=id; m.nameRu=name;
        m.densityKgM3=rho;
        m.youngModulusPa=E_GPa*1.0e9;
        m.poissonRatio=nu;
        m.yieldStrengthPa=yieldMPa*1.0e6;
        m.tensileStrengthPa=tensileMPa*1.0e6;
        m.friction=friction;
        m.restitution=restitution;
        m.thermalConductivityWmK=k;
        m.specificHeatJkgK=cp;
        m.electricalResistivityOhmM=resistivity;
        m.color=color;
        m.note=note;
        mMaterials.push_back(std::move(m));
    };

    add("steel_s235","Сталь конструкционная S235",7850,210,0.30,235,360,0.60,0.08,50,470,1.6e-7,{0.48f,0.50f,0.53f},"Типичные справочные свойства; прочность зависит от толщины и поставки.");
    add("steel_1045","Сталь AISI 1045",7850,205,0.29,530,625,0.60,0.08,49.8,486,1.7e-7,{0.42f,0.44f,0.47f},"Нормализованное состояние, ориентировочные значения.");
    add("ss304","Нержавеющая сталь AISI 304",8000,193,0.29,215,505,0.52,0.06,16.2,500,7.2e-7,{0.62f,0.64f,0.66f},"Отожжённое состояние.");
    add("cast_iron_gray","Серый чугун",7200,110,0.26,170,250,0.65,0.04,46,460,8.0e-7,{0.30f,0.31f,0.33f},"Для хрупких материалов предел текучести условный.");
    add("al6061_t6","Алюминий 6061-T6",2700,68.9,0.33,276,310,0.45,0.12,167,896,3.99e-8,{0.72f,0.74f,0.76f},"AA typical / T6.");
    add("al7075_t6","Алюминий 7075-T6",2810,71.7,0.33,503,572,0.45,0.10,130,960,5.15e-8,{0.68f,0.70f,0.73f},"Типичные свойства T6.");
    add("ti6al4v","Титан Ti-6Al-4V",4430,113.8,0.31,790,860,0.50,0.08,6.7,526,1.68e-6,{0.50f,0.53f,0.56f},"ELI annealed; свойства меняются с обработкой.");
    add("copper_c110","Медь C110",8960,117,0.34,69,220,0.75,0.04,391,385,1.72e-8,{0.72f,0.34f,0.16f},"Отожжённая/мягкая медь, ориентировочно.");
    add("brass_c360","Латунь C360",8500,97,0.31,124,338,0.50,0.06,115,380,6.8e-8,{0.72f,0.58f,0.22f},"Свинцовистая автоматная латунь.");
    add("bronze_c932","Бронза C932",8930,103,0.34,125,310,0.45,0.05,60,380,1.0e-7,{0.58f,0.38f,0.20f},"Подшипниковая бронза, ориентировочно.");
    add("magnesium_az31b","Магний AZ31B",1770,45,0.35,200,260,0.45,0.08,96,1040,9.2e-8,{0.66f,0.67f,0.64f},"Прокат, ориентировочные значения.");
    add("glass_soda_lime","Стекло натрий-кальциевое",2500,70,0.23,35,45,0.40,0.03,1.0,840,1.0e10,{0.55f,0.74f,0.78f},"Хрупкое; прочность особенно зависит от дефектов поверхности.");
    add("concrete","Бетон обычный",2400,30,0.20,20,3,0.70,0.02,1.7,880,1.0e9,{0.48f,0.47f,0.44f},"Анизотропия и разница растяжение/сжатие будут учтены отдельной моделью разрушения.");
    add("oak","Дуб",700,11,0.35,45,90,0.52,0.08,0.17,2400,1.0e12,{0.48f,0.29f,0.13f},"Усреднённо вдоль волокон; древесина анизотропна.");
    add("birch_plywood","Берёзовая фанера",680,10,0.30,30,60,0.50,0.08,0.13,1700,1.0e11,{0.68f,0.50f,0.28f},"Усреднённые эффективные свойства.");
    add("abs","ABS-пластик",1040,2.1,0.35,40,45,0.40,0.18,0.18,1300,1.0e13,{0.22f,0.24f,0.27f},"Типичный литьевой ABS.");
    add("nylon6","Полиамид 6 (Nylon 6)",1130,2.7,0.39,45,70,0.35,0.18,0.25,1670,1.0e12,{0.76f,0.73f,0.62f},"Свойства заметно зависят от влажности.");
    add("polycarbonate","Поликарбонат",1200,2.3,0.37,63,65,0.35,0.22,0.20,1200,1.0e14,{0.52f,0.64f,0.72f},"Типичные свойства прозрачного ПК.");
    add("pom","POM / ацеталь",1410,3.0,0.35,65,70,0.25,0.15,0.31,1460,1.0e14,{0.75f,0.75f,0.72f},"Инженерный ацеталь, ориентировочно.");
    add("natural_rubber","Натуральная резина",930,0.010,0.49,5,20,0.90,0.60,0.13,2000,1.0e13,{0.13f,0.13f,0.14f},"Сильно нелинейный материал; E здесь только малодеформационный ориентир.");
}

const MaterialDefinition& MaterialLibrary::get(std::string_view id) const {
    for (const auto& m : mMaterials) if (m.id == id) return m;
    return mMaterials.front();
}

int MaterialLibrary::indexOf(std::string_view id) const {
    for (std::size_t i=0;i<mMaterials.size();++i) if (mMaterials[i].id == id) return static_cast<int>(i);
    return 0;
}

} // namespace mechanica
