#include "Triangles.h"
#include "octree.h"
#include "intersections.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <utility>
#include <limits>

using Stereometry::triangle_t;
using Stereometry::point_t;
using Octree::octree_node_t;
using TrglesIntersections::triangle_unit_t;

template <typename VectT>
static int get_triangles_input(VectT &trgles, size_t trgls_num,
                                octree_node_t &octree)
{
    for (size_t i = 0; i < trgls_num; i++)
    {
        point_t p1, p2, p3;
        std::cin >> p1.x >> p1.y >> p1.z
                 >> p2.x >> p2.y >> p2.z
                 >> p3.x >> p3.y >> p3.z;
        if (!std::cin.good())
        {
            std::cerr << "Input error: triangle " << i / 9 << "." << std::endl;
            return 1;
        }
        trgles.push_back(triangle_unit_t{p1, p2, p3, octree});
    }
    return 0;
}

static bool node_check_intersection(triangle_unit_t &trgle,
                                    const octree_node_t &node)
{
    if (trgle.is_intersect) return true;
    size_t vect_size = node.data().size();
    for (size_t i = 0; i < vect_size; i++)
    {
        if (node.data_elem(i) == &(trgle.trgle)) continue;
        if (trgle.trgle.is_intersect(*(node.data_elem(i))))
            return trgle.is_intersect = true;
    }
    return false;
}

template <typename VectT>
static size_t octree_calculate_intersections(VectT &trgles, size_t trgls_num)
{
    size_t inters_cnt = 0;
    for (size_t i = 0; i < trgls_num; i++)
    {
        for (const octree_node_t * cur_node = &(trgles[i].octree_node);
             cur_node != NULL; cur_node = cur_node->parent())
        {
            if (node_check_intersection(trgles[i], *cur_node))
            {
                inters_cnt++;
                break;
            }
        }
    }
    return inters_cnt;
}

int main()
{
    size_t n;
    std::cin >> n;
    if (!std::cin.good())
    {
        std::cerr << "Input error: number of triangles." << std::endl;
        return 1;
    }

    octree_node_t octree{point_t{0, 0, 0},                         // Central point.
                         std::numeric_limits<float>::max() * 0.5f, // Half size.
                         NULL};                                    // Parent node (NULL cause it is base node).
    std::vector<triangle_unit_t> trgles;
    if (get_triangles_input(trgles, n, octree)) return 1;

    size_t intersect_count = octree_calculate_intersections(trgles, n);
    std::cout << intersect_count << std::endl;
    return 0;
}

#if 0
    for (size_t i = 0; i < n; i++)
    {   
        assert(trgles.size() > i);
        if (trgles[i].second)
        {
            intersect_count++;
            continue;
        }
        for (size_t j = i + 1; j < n; j++)
        {
            if (trgles[j].second) continue;
            if (trgles[i].first.is_intersect(trgles[j].first))
            {
                intersect_count++;
                trgles[j].second = true;
                break;
            }
        }
    }
#endif