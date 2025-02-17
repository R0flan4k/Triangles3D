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

using Octree::octree_node_t;
using Stereometry::triangle_t;
using Stereometry::vector_t;
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
    if (n == 0)
        return 0;

    std::vector<float> input(std::istream_iterator<float>(std::cin),
                             std::istream_iterator<float>());
    if (input.size() != n * 9)
    {
        std::cerr << "Input error: wrong number of points." << std::endl;
        return 1;
    }
    
    float abs_bound = *(std::max_element(input.cbegin(), input.cend(), 
    [](float a, float b)
    {
        return std::abs(a) < std::abs(b);
    }));
    abs_bound = std::abs(abs_bound) + 1;

    TrglesIntersections::octree_trgles_intersect_cntr_t<float> ts{
        n, abs_bound, input.cbegin(), input.cend()};
    ts.calculate_intersections();
    ts.intersections_dump();
    return 0;
}
