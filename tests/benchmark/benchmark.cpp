#include "intersections.h"
#include "octree.h"
#include "Triangles.h"

#include <benchmark/benchmark.h>
#include <vector>
#include <random>
#include <limits>

using Octree::octree_node_t;
using Stereometry::vector_t;
using TrglesIntersections::triangle_unit_t;

template <typename VectT>
void generate_input(size_t triangles_num, VectT &input,
                    float min_bound, float max_bound)
{
    // Random float number generator.
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution dis(min_bound, max_bound);

    for (size_t i = 0; i < triangles_num * 9; i++)
    {
        input.push_back(dis(rng));
    }
}

static void BM_random_octree_trgles(benchmark::State &state)
{
    // Setup.
    size_t n = state.range(0);
    std::vector<float> input;
    generate_input(n/2, input, -20.f, 0.f);
    generate_input(n - n/2, input, 0.f, 20.f);

    TrglesIntersections::octree_trgles_intersect_cntr_t ts{n, 20.f,
                                                           input.cbegin(), input.cend()};
    
    // This code gets timed.
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(ts.calculate_intersections());
    }
}

static void BM_random_trivial_trgles(benchmark::State &state)
{
    // Setup.
    size_t n = state.range(0);
    std::vector<float> input;
    generate_input(n, input, -20.f, 20.f);
    std::vector<triangle_t> trgles;

    for (size_t i = 0; i < n * 9; i += 9)
    {
        vector_t p1 = {input[i + 0], input[i + 1], input[i + 2]},
                 p2 = {input[i + 3], input[i + 4], input[i + 5]},
                 p3 = {input[i + 6], input[i + 7], input[i + 8]};

        trgles.emplace_back(p1, p2, p3);
    }

    // This code gets timed.
    for (auto _ : state)
    {   
        for (size_t i = 0; i < n; i++)
            for (size_t j = i + 1; j < n; j++)
                if (trgles[i].is_intersect(trgles[j])) 
                    benchmark::DoNotOptimize(52);
    }
}

static void BM_octree_trgles1(benchmark::State &state)
{
    // Setup.
    std::vector<float> input{1, 1, 0, 3, 1, 0, 1, 3, 0,
                             0, 0, 0, 1, 0, 0, 0, 1, 0,
                             1, 0.5, 0, 1, 0.5, 1, 0, 0, 0.5,
                             1, 0, 0, 0, 1, 0, 0, 0, 1,
                             0, 0, 0, 0, 3, 3, 0, 0, 3,
                             1, 1, 0, 1, 2, 3, 5, 4, 8,
                             9, 9, 9, 9, 9, 9, 9, 9, 9,
                             8, 8, 8, 8, 8, 8, -10, 8, 8};

    TrglesIntersections::octree_trgles_intersect_cntr_t ts{8, 11,
                                                           input.cbegin(), input.cend()};
    // This code gets timed.
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(ts.calculate_intersections());
    }
}

static void BM_trivial_trgles1(benchmark::State &state)
{
    // Setup.
    size_t n = 8;
    std::vector<triangle_t> trgles{triangle_t{{1, 1, 0}, {3, 1, 0}, {1, 3, 0}},
                                   triangle_t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
                                   triangle_t{{1, 0.5, 0}, {1, 0.5, 1}, {0, 0, 0.5}},
                                   triangle_t{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                                   triangle_t{{0, 0, 0}, {0, 3, 3}, {0, 0, 3}},
                                   triangle_t{{1, 1, 0}, {1, 2, 3}, {5, 4, 8}},
                                   triangle_t{{9, 9, 9}, {9, 9, 9}, {9, 9, 9}},
                                   triangle_t{{8, 8, 8}, {8, 8, 8}, {-10, 8, 8}}};

    // This code gets timed.
    for (auto _ : state)
    {   
        for (size_t i = 0; i < n; i++)
            for (size_t j = i + 1; j < n; j++)
                if (trgles[i].is_intersect(trgles[j])) 
                    benchmark::DoNotOptimize(52);
    }
}

BENCHMARK(BM_random_trivial_trgles)->Arg(100);
BENCHMARK(BM_random_octree_trgles)->Arg(100);
BENCHMARK(BM_trivial_trgles1)->Arg(100);
BENCHMARK(BM_octree_trgles1)->Arg(100);

BENCHMARK_MAIN();