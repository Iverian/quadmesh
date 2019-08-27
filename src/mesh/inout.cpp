#include <gen/mesh_builder.hpp>
#include <qmsh/inout.hpp>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/writer.h>

#include <fstream>

namespace qmsh {

Mesh build_mesh(const gm::Shell& shell, const qmsh::Config& conf)
{
    return MeshBuilder(shell, conf).get();
}

void json_export(const std::vector<Mesh>& obj, const std::string& filename)
{
    rapidjson::Document d;
    d.SetArray();

    auto& alloc = d.GetAllocator();
    for (auto& m : obj) {
        rapidjson::Value v;
        d.PushBack(m.serialize(v, alloc), alloc);
    }

    std::ofstream of(filename);
    rapidjson::OStreamWrapper osw(of);
    rapidjson::Writer writer(osw);
    writer.SetMaxDecimalPlaces(5);
    d.Accept(writer);
}

} // namespace qmsh
