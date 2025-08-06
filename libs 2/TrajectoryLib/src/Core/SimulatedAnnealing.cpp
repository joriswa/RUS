#include "TrajectoryLib/Core/SimulatedAnnealing.h"
#include <algorithm>
#include <optional>

namespace TrajectoryLib {
namespace Core {

SimulatedAnnealingResult optimizeSimulatedAnnealing(
    const CostFunction& cost_function,
    const SimulatedAnnealingParams& params,
    const std::optional<double>& initial_value)
{
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> search_dis(params.search_min, params.search_max);
    std::uniform_real_distribution<> acceptance_dis(0.0, 1.0);

    // Initialize current solution and cost
    double current_value;
    double current_cost = std::numeric_limits<double>::infinity();
    bool has_valid_solution = false;

    // Best solution tracking
    double best_value;
    double best_cost = std::numeric_limits<double>::infinity();

    // Initialize current value
    if (initial_value.has_value()) {
        current_value = std::clamp(initial_value.value(), params.search_min, params.search_max);
    } else {
        current_value = search_dis(gen);
    }

    // Temperature and iteration tracking
    double temperature = params.T_max;
    int no_improvement_counter = 0;
    int iterations_used = 0;

    // Simulated Annealing main loop
    for (int iter = 0; iter < params.max_iterations && temperature > params.T_min; ++iter) {
        iterations_used = iter + 1;

        // Generate perturbation radius based on temperature
        double search_range = params.search_max - params.search_min;
        double perturbation_radius = search_range * (temperature / params.T_max);

        // Generate neighbor value by perturbing current_value
        std::normal_distribution<> perturb_dis(0.0, perturbation_radius);
        double neighbor_value = current_value + perturb_dis(gen);
        neighbor_value = std::clamp(neighbor_value, params.search_min, params.search_max);

        // Evaluate neighbor solution
        auto [neighbor_cost, is_valid] = cost_function(neighbor_value);

        // If neighbor is not valid, continue
        if (!is_valid) {
            temperature *= params.alpha;
            continue;
        }

        // Simulated Annealing acceptance criterion
        bool accept = false;

        if (!has_valid_solution) {
            // Accept first valid solution
            accept = true;
            has_valid_solution = true;
            current_value = neighbor_value;
            current_cost = neighbor_cost;
        } else if (neighbor_cost < current_cost) {
            // Accept better solution
            accept = true;
        } else {
            // Accept worse solution with probability exp(-ΔE/T)
            double delta = neighbor_cost - current_cost;
            double acceptance_probability = std::exp(-delta / temperature);

            if (acceptance_dis(gen) < acceptance_probability) {
                accept = true;
            }
        }

        if (accept) {
            current_value = neighbor_value;
            current_cost = neighbor_cost;

            // Update best solution if this is the best so far
            if (neighbor_cost < best_cost) {
                best_value = neighbor_value;
                best_cost = neighbor_cost;
                no_improvement_counter = 0;
            } else {
                no_improvement_counter++;
            }
        }

        // Cool down temperature
        temperature *= params.alpha;

        // Early termination if no improvement for too long
        if (no_improvement_counter >= params.max_no_improvement) {
            break;
        }
    }

    // Prepare result
    SimulatedAnnealingResult result;
    result.success = has_valid_solution && (best_cost < std::numeric_limits<double>::infinity());
    
    if (result.success) {
        result.best_value = best_value;
        result.best_cost = best_cost;
    }
    
    result.iterations_used = iterations_used;

    return result;
}

} // namespace Core
} // namespace TrajectoryLib
