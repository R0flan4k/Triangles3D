#pragma once

#include "Triangles.h"
#include "double_comparing.h"

#include <vector>
#include <cassert>
#include <iostream>

namespace Octree {

using Stereometry::triangle_t;
using Stereometry::point_t;
using DblCmp::are_geq;

class octree_node_t {      
    octree_node_t * parent_;       
    octree_node_t * children_[8];
    std::vector<const triangle_t*> data_; // IDs of triangles in this node.
    point_t center_;
    float half_size_;

public:
    const octree_node_t* parent() const {return parent_;}
    const std::vector<const triangle_t*>& data() const {return data_;}

    octree_node_t(point_t center, float half_size, octree_node_t * parent)
    : center_(center), half_size_(half_size)
    {
        for (size_t i = 0; i < 8; i++)
            children_[i] = NULL;
    }

    ~octree_node_t()
    {
        for (size_t i = 0; i < 8; i++)
        {
            if (children_[i])
                delete children_[i];
        }
    }

    int get_position(const point_t &p) const
    {
        assert(p.valid());
        int res = 0;
        if (are_geq(p.x, center_.x)) res |= 4;
        if (are_geq(p.y, center_.y)) res |= 2;
        if (are_geq(p.z, center_.z)) res |= 1;
        return res;
    }

    const octree_node_t* insert_trgle(const triangle_t &trgle)
    {
        int p1_child = get_position(trgle.p1()),
            p2_child = get_position(trgle.p2()), 
            p3_child = get_position(trgle.p3());
        
        // Check if the triangle placed in child node.
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
        return this;
    }

    void dump() const
    {
        std::cout << "===========================================" << std::endl;
        std::cout << "              Octree node                  " << std::endl;
        std::cout << "Center: \t" << center_.x << ", " << center_.y << ", " << center_.z << std::endl;
        std::cout << "Half size: \t" << half_size_ << std::endl;
        std::cout << "Triangles:" << std::endl;
        size_t vect_size = data_.size();
        for (size_t i = 0; i < vect_size; i++)
        {
            std::cout << "\t[" << i << "]:" << std::endl;
            std::cout << "\t\t" << data_[i]->p1().x << ", " << data_[i]->p1().y << ", " << data_[i]->p1().z << std::endl;
            std::cout << "\t\t" << data_[i]->p2().x << ", " << data_[i]->p2().y << ", " << data_[i]->p2().z << std::endl;
            std::cout << "\t\t" << data_[i]->p3().x << ", " << data_[i]->p3().y << ", " << data_[i]->p3().z << std::endl;
        }
        std::cout << "===========================================" << std::endl;
    }
};

}