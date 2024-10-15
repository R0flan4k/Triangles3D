#include "Triangles.h"
#include "octree.h"
#include "intersections.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <utility>
#include <limits>
#include <iterator>

using Stereometry::triangle_t;
using Stereometry::point_t;
using Octree::octree_node_t;
using TrglesIntersections::triangle_unit_t;

int main()
{
    size_t n;
    std::cin >> n;
    if (!std::cin.good())
    {
        std::cerr << "Input error: number of triangles." << std::endl;
        return 1;
    }

    // We mustn't reallocate triangle units container because
    // octree nodes contains pointers to triangles inside node.
    std::vector<triangle_unit_t> trgles{n}; 
    octree_node_t octree{point_t{0, 0, 0},                         // Central point.
                         std::numeric_limits<float>::max() * 0.5f, // Half size.
                         NULL};                                    // Parent node (NULL cause it is base node).
    if (TrglesIntersections::get_triangles_input(trgles, n, octree, 
                                                 std::istream_iterator<float>(std::cin),
                                                 std::istream_iterator<float>())) 
        return 1;
    for (size_t i = 0; i < trgles.size(); i++)
    {
        assert(trgles[i].octree_node);
        trgles[i].octree_node->dump();
    }
    size_t intersect_count = TrglesIntersections::octree_calculate_intersections(trgles, n);
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