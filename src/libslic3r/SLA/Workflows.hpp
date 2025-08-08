#ifndef LIBSLIC3R_SLA_WORKFLOWS_HPP
#define LIBSLIC3R_SLA_WORKFLOWS_HPP

#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "nlohmann/json_fwd.hpp"

namespace Slic3r {
namespace sla {

struct Workflow
{
    std::string uuid;
    std::string name;
    bool is_default = false;
};

class WorkflowManager
{
    std::unique_ptr<nlohmann::json> m_data;

public:
    explicit WorkflowManager();
    ~WorkflowManager();

    std::vector<Workflow> workflows_for_material_sorted(const std::string &material_uuid) const;
};

} // namespace sla
} // namespace Slic3r

#endif // LIBSLIC3R_SLA_WORKFLOWS_HPP
