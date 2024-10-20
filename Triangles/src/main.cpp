#include "Triangles.h"
#include "octree.h"
#include "intersections.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <utility>
#include <limits>
#include <iterator>
#include <algorithm>

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

    std::vector<float> input(std::istream_iterator<float>(std::cin),
                             std::istream_iterator<float>());
    float abs_bound = 1 + *(std::max_element(input.cbegin(), input.cend(), 
    [](float a, float b)
    {
        return std::abs(a) < std::abs(b);
    }));

    TrglesIntersections::octree_trgles_intersect_cntr_t ts{n, abs_bound, 
                                                           input.cbegin(), input.cend()};
    size_t intersect_count = ts.calculate_intersections();
    std::cout << intersect_count << std::endl;
    return 0;
}