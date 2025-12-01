#ifndef SUPPORT_TREE_SLICER_HPP
#define SUPPORT_TREE_SLICER_HPP

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/SLA/SupportTreeTypes.hpp"

namespace Slic3r {
namespace sla {

ExPolygons slice_support_tree_at_height(const sla::SupportTreeOutput& output, float height);

std::vector<ExPolygons> slice_support_tree(
    const sla::SupportTreeOutput& output,
    const std::vector<float>& heights);

}
}
#endif