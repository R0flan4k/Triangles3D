#pragma once

#include "Triangles.h"
#include "octree.h"

using Stereometry::triangle_t;
using Stereometry::point_t;
using Octree::octree_node_t;

namespace TrglesIntersections {

struct triangle_unit_t {
    triangle_t trgle;
    const octree_node_t &octree_node;
    bool is_intersect; // true if we figured out that
                       // this triangle in intersect with
                       // some other triangle.

    triangle_unit_t(const point_t &p1, const point_t &p2,
                    const point_t &p3, octree_node_t &octree)
    : trgle(p1, p2, p3), octree_node(octree.insert_trgle(trgle)),
      is_intersect(false) {}
};

}