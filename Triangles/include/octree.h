#pragma once

#include "Triangles.h"
#include "double_comparing.h"

#include <list>
#include <cassert>
#include <iostream>

namespace Octree {

using DblCmp::are_geq;
using Stereometry::triangle_t;
using Stereometry::vector_t;

const size_t max_depth = 5;

template <typename ChildT> struct ocnode_children_buff {
protected:
    std::array<ChildT *, 8> children;

    ocnode_children_buff() noexcept
    {
        for (size_t i = 0; i < 8; ++i)
            children[i] = nullptr;
    }

    virtual ~ocnode_children_buff()
    {
        for (size_t i = 0; i < 8; i++)
            delete children[i];
    }

    ocnode_children_buff(const ocnode_children_buff &c) noexcept
    {
        for (size_t i = 0; i < 8; ++i)
            children[i] = c.children[i];
    }

    ocnode_children_buff(ocnode_children_buff &&c) noexcept
    {
        for (size_t i = 0; i < 8; ++i)
            std::swap(children[i], c.children[i]);
    }

    ocnode_children_buff &operator=(const ocnode_children_buff &c) noexcept
    {
        for (size_t i = 0; i < 8; ++i)
            children[i] = c.children[i];
        return *this;
    }

    ocnode_children_buff &operator=(ocnode_children_buff &&c) noexcept
    {
        if (&c == this)
            return *this;
        for (size_t i = 0; i < 8; ++i)
            std::swap(children[i], c.children[i]);
        return *this;
    }
};

template <typename DataT>
class octree_node_t final : private ocnode_children_buff<octree_node_t<DataT>> {
    using ocnode_children_buff<octree_node_t<DataT>>::children;
    octree_node_t * parent_;       
    std::list<DataT*> data_;
    vector_t center_;
    float half_size_;
    const size_t depth_;

public:
    octree_node_t* parent() const {return parent_;}
    const std::list<DataT*>& data() const {return data_;}

    octree_node_t(vector_t center, float half_size, octree_node_t *parent,
                  size_t depth = 0)
        : parent_(parent), center_(center), half_size_(half_size), depth_(depth)
    {}

    octree_node_t(vector_t center)
        : parent_(nullptr), center_(center), half_size_(NAN), depth_(0)
    {}

    int get_position(const vector_t &p) const
    {
        assert(p.valid());
        int res = 0;
        if (are_geq(p.x, center_.x)) res |= 4;
        if (are_geq(p.y, center_.y)) res |= 2;
        if (are_geq(p.z, center_.z)) res |= 1;
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
            if (children[p1_child] == nullptr)
            {
                vector_t child_center = {
                    center_.x + half_size_ * (p1_child & 4 ? 0.5f : -0.5f),
                    center_.y + half_size_ * (p1_child & 2 ? 0.5f : -0.5f),
                    center_.z + half_size_ * (p1_child & 1 ? 0.5f : -0.5f)};
                children[p1_child] = new octree_node_t(
                    child_center, half_size_ * 0.5f, this, depth_ + 1);
            }
            return children[p1_child]->insert_trgle(trgle);
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
        std::cout << "Center: \t" << center_.x << ", " << center_.y << ", " << center_.z << std::endl;
        std::cout << "Half size: \t" << half_size_ << std::endl;
        std::cout << "Triangles:" << std::endl;
        size_t i = 0;
        for (auto start = data_.cbegin(), end = data_.cend(); start != end; ++start, ++i)
        {
            std::cout << "\t[" << i << "]:" << std::endl;
            std::cout << "\t\t" << (*start)->p1().x << ", " << (*start)->p1().y << ", " << (*start)->p1().z << std::endl;
            std::cout << "\t\t" << (*start)->p2().x << ", " << (*start)->p2().y << ", " << (*start)->p2().z << std::endl;
            std::cout << "\t\t" << (*start)->p3().x << ", " << (*start)->p3().y << ", " << (*start)->p3().z << std::endl;
            std::cout << "\tIs intersect:\t" << (*start)->is_intersect << std::endl;
        }
        for (size_t i = 0; i < 8; ++i)
        {
            if (children[i])
            {
                std::cout << "\nChild[" << i << "]:\n";
                children[i]->dump();
            }
        }
        std::cout << "===========================================" << std::endl;
    }
};
}