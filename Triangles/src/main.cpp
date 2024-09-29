#include "Triangles.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <utility>

int main()
{
    size_t n;
    std::cin >> n;
    if (!std::cin.good())
    {
        std::cerr << "Input error: number of triangles." << std::endl;
        return 1;
    }

    std::vector<std::pair<Stereometry::triangle_t, bool>> trgles;
    for (size_t i = 0; i < n; i++)
    {
        Stereometry::point_t p1, p2, p3;
        std::cin >> p1.x >> p1.y >> p1.z
                 >> p2.x >> p2.y >> p2.z
                 >> p3.x >> p3.y >> p3.z;
        if (!std::cin.good())
        {
            std::cerr << "Input error: triangle " << i / 9 << "." << std::endl;
            return 1;
        }
        trgles.push_back({Stereometry::triangle_t{p1, p2, p3}, false});
    }

    size_t intersect_count = 0;
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

    std::cout << intersect_count << std::endl;
    return 0;
}