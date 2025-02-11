#pragma once

#include "Triangles.h"
#include "double_comparing.h"

#include <cassert>
#include <iostream>
#include <list>
#include <memory>

namespace Octree {

using DblCmp::are_geq;
using Stereometry::triangle_t;
using Stereometry::vector_t;

template <typename DataT> class octree_node_t final {
    using ChildT = octree_node_t<DataT>;

    std::array<std::unique_ptr<ChildT>, 8> children_;
    octree_node_t * parent_;       
    std::list<DataT*> data_;
    vector_t center_;
    float half_size_;
    const size_t depth_;

public:
    const size_t max_depth;

public:
    octree_node_t* parent() const {return parent_;}
    const std::list<DataT*>& data() const {return data_;}

    octree_node_t(vector_t center, float half_size, octree_node_t *parent,
                  size_t depth = 0, size_t max_dep = 5)
        : children_(), parent_(parent), center_(center), half_size_(half_size),
          depth_(depth), max_depth(max_dep)
    {}

    octree_node_t(vector_t center, size_t max_dep = 5)
        : children_(), parent_(nullptr), center_(center), half_size_(NAN),
          depth_(0), max_depth(max_dep)
    {}

    int get_position(const vector_t &p) const
    {
        int res = 0;
        if (are_geq(p.x, center_.x))
            res |= 4;
        if (are_geq(p.y, center_.y))
            res |= 2;
        if (are_geq(p.z, center_.z))
            res |= 1;
        return res;
    }

    octree_node_t* insert_trgle(DataT *trgle)
    {
        assert(trgle);

        if (depth_ >= max_depth)
        {
            data_.push_back(trgle);
            return this;
        }

        int p1_child = get_position(trgle->p1()),
            p2_child = get_position(trgle->p2()), 
            p3_child = get_position(trgle->p3());

        // Check if the triangle placed in child node.
        if (p1_child == p2_child && p2_child == p3_child)
        {
            if (children_[p1_child] == nullptr)
            {
                vector_t child_center = {
                    center_.x + half_size_ * (p1_child & 4 ? 0.5f : -0.5f),
                    center_.y + half_size_ * (p1_child & 2 ? 0.5f : -0.5f),
                    center_.z + half_size_ * (p1_child & 1 ? 0.5f : -0.5f)};
                children_[p1_child] = std::make_unique<ChildT>(
                    child_center, half_size_ * 0.5f, this, depth_ + 1);
            }
            return children_[p1_child]->insert_trgle(trgle);
        }

        data_.push_back(trgle);
        return this;
    }

    using ListIt = typename std::list<DataT*>::const_iterator;
    void set_triangle_intersection(ListIt it)
    {
        (*it)->is_intersect = true;
    }

    void dump() const
    {
        std::cout << "===========================================" << std::endl;
        std::cout << "              Octree node                  " << std::endl;
        std::cout << "Center: \t" << center_.x << ", " << center_.y << ", "
                  << center_.z << std::endl;
        std::cout << "Half size: \t" << half_size_ << std::endl;
        std::cout << "Triangles:" << std::endl;
        size_t i = 0;
        for (auto start = data_.cbegin(), end = data_.cend(); start != end; ++start, ++i)
        {
            std::cout << "\t[" << i << "]:" << std::endl;
            std::cout << "\t\t" << (*start)->p1().x << ", " << (*start)->p1().y
                      << ", " << (*start)->p1().z << std::endl;
            std::cout << "\t\t" << (*start)->p2().x << ", " << (*start)->p2().y
                      << ", " << (*start)->p2().z << std::endl;
            std::cout << "\t\t" << (*start)->p3().x << ", " << (*start)->p3().y
                      << ", " << (*start)->p3().z << std::endl;
            std::cout << "\tIs intersect:\t" << (*start)->is_intersect << std::endl;
        }
        for (size_t i = 0; i < 8; ++i)
        {
            if (children_[i] != nullptr)
            {
                std::cout << "\nChild[" << i << "]:\n";
                children_[i]->dump();
            }
        }
        std::cout << "===========================================" << std::endl;
    }
};
}