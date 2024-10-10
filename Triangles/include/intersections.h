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

    triangle_unit_t(const point_t &p1, const point_t &p2,
                    const point_t &p3)
    : trgle(p1, p2, p3), is_intersect(false) {}
};

template <typename VectT, typename RandomIt>
int get_triangles_input(VectT &trgles, size_t trgls_num,
                        octree_node_t &octree,
                        RandomIt first, RandomIt last)
{
    for (; first != last; trgls_num--)
    {
        point_t p1, p2, p3;
        p1.x = *(first++); p1.y = *(first++); p1.z = *(first++);
        p2.x = *(first++); p2.y = *(first++); p2.z = *(first++);
        p3.x = *(first++); p3.y = *(first++); p3.z = *(first++);
        
        assert(p1.valid() && p2.valid() && p3.valid());
        trgles.emplace_back(p1, p2, p3);
        trgles[trgles.size() - 1].octree_node = octree.insert_trgle(trgles[trgles.size() - 1].trgle);
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
        // std::cout << "Vhod v is intersect\n";
        if (trgle.trgle.is_intersect(*(node.data()[i])))
            return trgle.is_intersect = true;
    }
    return false;
}

template <typename VectT>
size_t octree_calculate_intersections(VectT &trgles, size_t trgls_num)
{
    size_t inters_cnt = 0;
    for (size_t i = 0; i < trgls_num; i++)
    {
        for (const octree_node_t * cur_node = trgles[i].octree_node;
             cur_node != NULL; cur_node = cur_node->parent())
        {
            // std::cout << "Vhod v node calc\n";
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