#pragma once

#include "Triangles.h"
#include "octree.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

using Stereometry::triangle_t;
using Stereometry::point_t;
using Octree::octree_node_t;

namespace TrglesIntersections {

struct triangle_unit_t {
    triangle_t trgle;
    octree_node_t<triangle_unit_t>* ocnode;
    bool is_intersect;

    const point_t& p1() const {return trgle.p1();}
    const point_t& p2() const {return trgle.p2();}
    const point_t& p3() const {return trgle.p3();}

    triangle_unit_t(const point_t &p1, const point_t &p2,
                    const point_t &p3, bool is_inter,
                    octree_node_t<triangle_unit_t> &octree)
    : trgle(p1, p2, p3), ocnode(octree.insert_trgle(this)),
      is_intersect(is_inter) {}
};

struct octree_trgles_intersect_cntr_t {
    using NodeT = octree_node_t<triangle_unit_t>;
    NodeT octree;
    std::vector<triangle_unit_t> data;

public:
    template <typename RandomIt>
    octree_trgles_intersect_cntr_t(size_t n, float octree_half_size,
                                   RandomIt first, const RandomIt last)
    : octree(point_t{0, 0, 0}, octree_half_size, NULL)
    {
        data.reserve(n*9);

        for (; first != last;)
        {
            float coords[9];
            auto point_end = std::next(first, 9);
            std::copy(first, point_end, coords);
            first = point_end;
            point_t p[3] = {point_t{coords[0], coords[1], coords[2]},
                            point_t{coords[3], coords[4], coords[5]},
                            point_t{coords[6], coords[7], coords[8]}};
            assert(p[0].valid() && p[1].valid() && p[2].valid());
            data.emplace_back(p[0], p[1], p[2], false, octree);
        }
    }

    size_t calculate_intersections()
    {
        size_t inters_cnt = 0;
        for (auto start = data.begin(), end = data.end();
             start != end; ++start)
        {
            if (start->is_intersect) 
            {
                ++inters_cnt;
                continue;
            }

            for (NodeT *ocnode= start->ocnode;
                 ocnode != NULL;  ocnode = ocnode->parent())
            {
                if (node_check_intersection(*start, *ocnode))
                {
                    ++inters_cnt;
                    break;
                }
            }
        }
        return inters_cnt;
    }

private:
    bool node_check_intersection(triangle_unit_t &trgle,
                                 NodeT &ocnode)
    {
        for (auto start = ocnode.data().begin(), end = ocnode.data().end();
             start != end; ++start)
        {
            assert(*start);
            if (*start == &trgle)
                continue;

            if (trgle.trgle.is_intersect((*start)->trgle))
                return trgle.is_intersect = (*start)->is_intersect = true;
                
        }
        return false;
    }
};

}