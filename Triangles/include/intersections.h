#pragma once

#include "Triangles.h"
#include "octree.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

using Octree::octree_node_t;
using Stereometry::triangle_t;
using Stereometry::vector_t;

namespace TrglesIntersections {

struct triangle_unit_t {
    triangle_t trgle;
    octree_node_t<triangle_unit_t>* ocnode;
    bool is_intersect;

    const vector_t &p1() const { return trgle.p1(); }
    const vector_t &p2() const { return trgle.p2(); }
    const vector_t &p3() const { return trgle.p3(); }
    bool is_special_point() const {return trgle.is_special_point();}
    bool is_special_interval() const {return trgle.is_special_interval();}

    triangle_unit_t(const vector_t &p1, const vector_t &p2, const vector_t &p3,
                    bool is_inter, octree_node_t<triangle_unit_t> &octree)
        : trgle(p1, p2, p3), ocnode(octree.insert_trgle(this)),
          is_intersect(is_inter)
    {}
};

class octree_trgles_intersect_cntr_t {
    using NodeT = octree_node_t<triangle_unit_t>;
    NodeT octree_;
    std::vector<triangle_unit_t> data_;

public:
    template <typename RandomIt>
    octree_trgles_intersect_cntr_t(size_t n, float octree_half_size,
                                   RandomIt first, const RandomIt last)
        : octree_(vector_t{0, 0, 0}, octree_half_size, NULL)
    {
        data_.reserve(n);
        for (; first != last;)
        {
            float coords[9];
            auto point_end = std::next(first, 9);
            std::copy(first, point_end, coords);
            first = point_end;
            vector_t p[3] = {vector_t{coords[0], coords[1], coords[2]},
                             vector_t{coords[3], coords[4], coords[5]},
                             vector_t{coords[6], coords[7], coords[8]}};
            assert(p[0].valid() && p[1].valid() && p[2].valid());
            data_.emplace_back(p[0], p[1], p[2], false, octree_);
        }
    }

    void sz_dump() const
    {
        std::cout << "Data size: " << data_.size() << std::endl;
        std::cout << "Data capa: " << data_.capacity() << std::endl;
    }

    void octree_dump() const
    {
        octree_.dump();
    }

    size_t calculate_intersections()
    {
        size_t inters_cnt = 0;
        for (auto start = data_.begin(), end = data_.end();
             start != end; ++start)
        {
            for (NodeT *ocnode = start->ocnode;
                 ocnode != NULL;  ocnode = ocnode->parent())
                inters_cnt += node_check_intersection(*start, *ocnode);
        }
        return inters_cnt;
    }

    void intersections_dump() const
    {
        for (size_t i = 0, data_sz = data_.size();
             i < data_sz; ++i)
        {
            if (data_[i].is_intersect) std::cout << i << std::endl;
        }
    }

private:
    size_t node_check_intersection(triangle_unit_t &trgle,
                                   NodeT &ocnode)
    {
        size_t inters_cnt = 0;
        for (auto start = ocnode.data().begin(), end = ocnode.data().end();
             start != end; ++start)
        {
            assert(*start);
            if (*start == &trgle)
                continue;

            if ((*start)->is_intersect && trgle.is_intersect)
                continue;

            if (trgle.trgle.is_intersect((*start)->trgle))
            {
                if (!(*start)->is_intersect)
                {
                    ocnode.set_triangle_intersection(start);
                    ++inters_cnt;
                }
                if (!trgle.is_intersect)
                {
                    trgle.is_intersect = true;
                    ++inters_cnt;
                }
            }
        }
        return inters_cnt;
    }
};

}