#pragma once

#include "Triangles.h"
#include "octree.h"

#include <iostream>

using Stereometry::triangle_t;
using Stereometry::point_t;
using Octree::octree_node_t;

namespace TrglesIntersections {

struct triangle_unit_t {
    triangle_t trgle;
    const octree_node_t *octree_node;
    bool is_intersect; // true if we figured out that
                       // this triangle in intersect with
                       // some other triangle.
    triangle_unit_t() : trgle(), octree_node(NULL), is_intersect(false) {}
    triangle_unit_t(const point_t &p1, const point_t &p2,
                    const point_t &p3, octree_node_t &octree)
    : trgle(p1, p2, p3), octree_node(octree.insert_trgle(trgle)),
      is_intersect(false) {}
};

template <typename VectT, typename RandomIt>
int get_triangles_input(VectT &trgles, size_t trgls_num,
                        octree_node_t &octree,
                        RandomIt first, RandomIt last)
{
    assert(trgles.size() == trgls_num);
    for (size_t i = 0; first != last; trgls_num--, i++)
    {
        float coords[9];
        auto cur_start = first;
        std::advance(first, 9);
        std::copy(cur_start, first, coords);
        point_t p[3] = {point_t{coords[0], coords[1], coords[2]},
                        point_t{coords[3], coords[4], coords[5]},
                        point_t{coords[6], coords[7], coords[8]}};

        assert(p[0].valid() && p[1].valid() && p[2].valid());
        trgles[i] = triangle_unit_t{p[0], p[1], p[2], octree};
    }
    if (trgls_num != 0) return 1;
    return 0;
}

static bool node_check_intersection(triangle_unit_t &trgle,
                                    const octree_node_t &node)
{
    if (trgle.is_intersect) return true;
    size_t vect_size = node.data().size();
    for (size_t i = 0; i < vect_size; i++)
    {
        if (node.data()[i] == &(trgle.trgle)) continue;
        assert(node.data()[i]);
        if (trgle.trgle.is_intersect(*(node.data()[i])))
            return trgle.is_intersect = true;
    }
    return false;
}

template <typename VectT>
size_t octree_calculate_intersections(VectT &trgles, size_t trgls_num)
{
    trgles[0].trgle.dump();
    size_t inters_cnt = 0;
    for (size_t i = 0; i < trgls_num; i++)
    {
        for (const octree_node_t * cur_node = trgles[i].octree_node;
             cur_node != NULL; cur_node = cur_node->parent())
        {
            cur_node->dump();
            if (node_check_intersection(trgles[i], *cur_node))
            {
                inters_cnt++;
                break;
            }
        }
    }
    return inters_cnt;
}

}