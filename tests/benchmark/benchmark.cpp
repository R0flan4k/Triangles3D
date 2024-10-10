#include "intersections.h"
#include "octree.h"
#include "Triangles.h"

#include <benchmark/benchmark.h>
#include <vector>
#include <random>
#include <limits>

using Stereometry::point_t;
using Octree::octree_node_t;
using TrglesIntersections::triangle_unit_t;

template <typename VectT>
void generate_input(size_t triangles_num, VectT &input)
{
    // Random float number generator.
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution dis(-100.f, 100.f);

    for (size_t i = 0; i < triangles_num * 9; i++)
    {
        input.push_back(dis(rng));
    }
}

static void BM_octree_trgles(benchmark::State &state)
{
    // Setup.
    std::vector<float> input;
    generate_input(state.range(0), input);
    std::cout << "Input generated\n";
    octree_node_t octree{point_t{0, 0, 0},
                         std::numeric_limits<float>::max() * 0.5f,
                         NULL};
    std::vector<triangle_unit_t> trgles;
    TrglesIntersections::get_triangles_input(trgles, state.range(0), octree,
                                             input.begin(), input.end());
    std::cout << "Input processed\n";
    
    // This code gets timed.
    for (auto _ : state)
    {
        benchmark::DoNotOptimize(TrglesIntersections::octree_calculate_intersections(trgles, state.range(0)));
    }
}

static void BM_trivial_trgles(benchmark::State &state)
{
    // Setup.
    std::vector<float> input;
    generate_input(state.range(0), input);
    std::cout << "Input generated\n";
    std::vector<triangle_t> trgles;
    for (size_t i = 0; i < state.range(0) * 9; i += 9)
    {
        point_t p1 = {input[i + 0], input[i + 1], input[i + 2]},
                p2 = {input[i + 3], input[i + 4], input[i + 5]},
                p3 = {input[i + 6], input[i + 7], input[i + 8]};
        std::cout << "[" << i / 9 << " trgle]:\n";
        std::cout << "\t"; p1.dump();
        std::cout << "\t"; p2.dump();
        std::cout << "\t"; p3.dump();
        trgles.push_back(triangle_t{p1, p2, p3});
    }
    std::cout << "Input processed\n";

    // This code gets timed.
    for (auto _ : state)
    {
        for (size_t i = 0; i < state.range(0); i++)
            for (size_t j = i + 1; j < state.range(0); j++)
            {
                std::cout << "[" << i <<"] and [" << j << "] trgles.\n";
                if (trgles[i].is_intersect(trgles[j])) break;
            }
    }
}

BENCHMARK(BM_trivial_trgles)->Arg(100);
BENCHMARK(BM_octree_trgles)->Arg(100);

BENCHMARK_MAIN();