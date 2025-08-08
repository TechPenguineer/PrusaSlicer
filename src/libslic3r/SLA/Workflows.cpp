#include "Workflows.hpp"

#include <fstream>

#include "boost/nowide/fstream.hpp"
#include "boost/filesystem.hpp"
#include "boost/system/error_code.hpp"
#include "boost/log/trivial.hpp"
#include "nlohmann/json.hpp"

#include "libslic3r/Utils.hpp"

namespace Slic3r {
namespace sla {

WorkflowManager::WorkflowManager()
{
    // Make sure that the file exists in datadir, if not, copy it from resources dir.
    std::string filename_datadir = data_dir() + "/workflows.json";
    std::string filename_resdir = resources_dir() + "/workflows.json";

    boost::system::error_code ec;
    if (! boost::filesystem::exists(filename_datadir, ec)) {
        BOOST_LOG_TRIVIAL(error) << "workflows.json file not found in datadir, about to copy from resources...";
        if (boost::filesystem::exists(filename_resdir, ec))
            boost::filesystem::copy_file(filename_resdir, filename_datadir, ec);
    }
    if (! boost::filesystem::exists(filename_datadir, ec)) {
        BOOST_LOG_TRIVIAL(error) << "ERROR: Unable to copy workflows.json from resources. SLA Workflows will not be available.";
        return;
    }

    boost::nowide::ifstream f(filename_datadir);
    if (f) {
        try {
            m_data = std::make_unique<nlohmann::json>(nlohmann::json::parse(f, nullptr, false));
        } catch (const nlohmann::json::exception&) {
            // Do nothing on purpose.
        }
    }
}



WorkflowManager::~WorkflowManager()
{
}



std::vector<Workflow> WorkflowManager::workflows_for_material_sorted(const std::string &material_uuid) const
{
    std::vector<Workflow> out;
    out.push_back({"", "(no workflow)", true});

    if (! m_data || material_uuid.empty() || m_data->is_discarded() || !m_data->contains("materials") || !m_data->contains("workflows"))
        return out;

    const nlohmann::json& materials = (*m_data)["materials"];
    if (!materials.contains(material_uuid))
        return out;

    const nlohmann::json& material_data = materials[material_uuid];
    std::string default_workflow_uuid;
    if (material_data.contains("default_workflow"))
        default_workflow_uuid = material_data["default_workflow"].get<std::string>();

    if (!material_data.contains("slx_workflows"))
        return out;

    const nlohmann::json& all_workflows = (*m_data)["workflows"];
    for (const auto& workflow_uuid_json : material_data["slx_workflows"]) {
        std::string workflow_uuid = workflow_uuid_json.get<std::string>();
        if (all_workflows.contains(workflow_uuid)) {
            const nlohmann::json& workflow_data = all_workflows[workflow_uuid];
            if (workflow_data.contains("name"))
                out.push_back({workflow_uuid, workflow_data["name"].get<std::string>(), workflow_uuid == default_workflow_uuid});
        }
    }

    std::sort(out.begin() + 1, out.end(), [](const auto& a , const auto& b) { return a.name < b.name; });
    return out;
}

} // namespace sla
} // namespace Slic3r