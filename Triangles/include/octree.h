#pragma once

#include "Triangles.h"

#include <vector>
#include <cassert>

namespace Octree {

using Stereometry::triangle_t;
using Stereometry::point_t;
                                         
class octree_node_t {      
    octree_node_t * parent_;       
    octree_node_t * children_[8];
    std::vector<const triangle_t*> data_;
    point_t center_;
    float half_size_;

public:
    const octree_node_t& parent() const {return *parent_;}
    const std::vector<const triangle_t*>& data() const {return data_;}
    const triangle_t* data_elem(size_t id) const
    {
        if (id >= data_.size()) return NULL;
        return data_[id];
    }

    octree_node_t(point_t center, float half_size, octree_node_t * parent)
    : center_(center), half_size_(half_size)
    {
        for (size_t i = 0; i < 8; i++)
            children_[i] = NULL;
    }

    int get_position(const point_t &p) const
    {
        assert(p.valid());
        int res = 0;
        if (p.x > center_.x) res |= 4;
        if (p.y > center_.y) res |= 2;
        if (p.z > center_.z) res |= 1;
    }

    const octree_node_t& insert_trgle(const triangle_t &trgle)
    {
        int p1_child = get_position(trgle.p1()),
            p2_child = get_position(trgle.p2()), 
            p3_child = get_position(trgle.p3());
        
        if (p1_child == p2_child == p3_child)
        {
            if (children_[p1_child] == NULL)
            {
                point_t child_center = 
                    {center_.x + half_size_ * (p1_child & 4 ? 0.5f : -0.5f),
                     center_.y + half_size_ * (p1_child & 2 ? 0.5f : -0.5f),
                     center_.z + half_size_ * (p1_child & 1 ? 0.5f : -0.5f)};
                children_[p1_child] = new octree_node_t(child_center, half_size_ * 0.5f,
                                                        this);
            }
            return children_[p1_child]->insert_trgle(trgle);
        }
        
        data_.push_back(&trgle);
        return *this;
    }
};

}