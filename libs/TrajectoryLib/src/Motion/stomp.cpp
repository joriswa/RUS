/**
 * @file stomp.cpp
 * @brief This contains the stomp core algorithm
 *
 * @author Jorge Nicho
 * @date March 7, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "TrajectoryLib/Motion/stomp.h"
#include "TrajectoryLib/Core/utils.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <limits.h>
#include <math.h>
#include <numeric>

static const double DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT = 1.0; /**< Default noisy cost importance weight */
static const double MIN_COST_DIFFERENCE = 1e-8; /**< Minimum cost difference allowed during probability calculation */
static const double MIN_CONTROL_COST_WEIGHT = 1e-8; /**< Minimum control cost weight allowed */

/**
 * @brief Compute a linear interpolated trajectory given a start and end state
 * @param first             The start position
 * @param last              The final position
 * @param num_timesteps     The number of timesteps
 * @param trajectory_joints The returned linear interpolated trajectory
 */
static void computeLinearInterpolation(const std::vector<double>& first,
                                       const std::vector<double>& last,
                                       int num_timesteps,
                                       Eigen::MatrixXd& trajectory_joints)
{
  trajectory_joints.setZero(first.size(), num_timesteps);
  for (int unsigned i = 0; i < first.size(); i++)
  {
    double dtheta = (last[i] - first[i]) / (num_timesteps - 1);
    for (unsigned int j = 0; j < num_timesteps; j++)
    {
      trajectory_joints(i, j) = first[i] + j * dtheta;
    }
  }
}

/**
 * @brief Compute a cubic interpolated trajectory given a start and end state
 * @param first             The start position
 * @param last              The final position
 * @param num_points        The number of points in the trajectory
 * @param dt                The timestep in seconds
 * @param trajectory_joints The returned cubic interpolated trajectory
 */
static void computeCubicInterpolation(const std::vector<double>& first,
                                      const std::vector<double>& last,
                                      int num_points,
                                      double dt,
                                      Eigen::MatrixXd& trajectory_joints)
{
  std::vector<double> coeffs(4, 0);
  double total_time = (num_points - 1) * dt;
  for (int unsigned i = 0; i < first.size(); i++)
  {
    coeffs[0] = first[i];
    coeffs[2] = (3 / (pow(total_time, 2))) * (last[i] - first[i]);
    coeffs[3] = (-2 / (pow(total_time, 3))) * (last[i] - first[i]);

    double t;
    for (unsigned j = 0; j < num_points; j++)
    {
      t = j * dt;
      trajectory_joints(i, j) = coeffs[0] + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3);
    }
  }
}

/**
 * @brief Compute a minimum cost trajectory given a start and end state
 * @param first                        The start position
 * @param last                         The final position
 * @param control_cost_matrix_R_padded The control cost matrix with padding
 * @param inv_control_cost_matrix_R    The inverse constrol cost matrix
 * @param trajectory_joints            The returned minimum cost trajectory
 * @return True if successful, otherwise false
 */
bool computeMinCostTrajectory(const std::vector<double>& first,
                              const std::vector<double>& last,
                              const Eigen::MatrixXd& control_cost_matrix_R_padded,
                              const Eigen::MatrixXd& inv_control_cost_matrix_R,
                              Eigen::MatrixXd& trajectory_joints)
{
  using namespace stomp;

  if (control_cost_matrix_R_padded.rows() != control_cost_matrix_R_padded.cols())
  {
    return false;
  }

  int timesteps = control_cost_matrix_R_padded.rows() - 2 * (FINITE_DIFF_RULE_LENGTH - 1);
  int start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
  int end_index_padded = start_index_padded + timesteps - 1;
  std::vector<Eigen::VectorXd> linear_control_cost(first.size(), Eigen::VectorXd::Zero(timesteps));
  trajectory_joints.setZero(first.size(), timesteps);

  for (unsigned int d = 0; d < first.size(); d++)
  {
    linear_control_cost[d].transpose() =
        first[d] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        control_cost_matrix_R_padded.block(0, start_index_padded, FINITE_DIFF_RULE_LENGTH - 1, timesteps);

    linear_control_cost[d].transpose() +=
        last[d] * Eigen::VectorXd::Ones(FINITE_DIFF_RULE_LENGTH - 1).transpose() *
        control_cost_matrix_R_padded.block(
            end_index_padded + 1, start_index_padded, FINITE_DIFF_RULE_LENGTH - 1, timesteps);

    linear_control_cost[d] *= 2;

    trajectory_joints.row(d) = -0.5 * inv_control_cost_matrix_R * linear_control_cost[d];
    trajectory_joints(d, 0) = first[d];
    trajectory_joints(d, timesteps - 1) = last[d];
  }

  return true;
}

/**
 * @brief Compute the parameters control costs
 * @param parameters            The parameters used to compute the control cost
 * @param dt                    The timestep in seconds
 * @param control_cost_weight   The control cost weight
 * @param control_cost_matrix_R The control cost matrix
 * @param control_costs returns The parameters control costs
 */
void computeParametersControlCosts(const Eigen::MatrixXd& parameters,
                                   double dt,
                                   double control_cost_weight,
                                   const Eigen::MatrixXd& control_cost_matrix_R,
                                   Eigen::MatrixXd& control_costs)
{
  std::size_t num_timesteps = parameters.cols();
  double cost = 0;
  for (auto d = 0u; d < parameters.rows(); d++)
  {
    cost = double(parameters.row(d) * (control_cost_matrix_R * parameters.row(d).transpose()));
    control_costs.row(d).setConstant(0.5 * (1 / dt) * cost);
  }

  double max_coeff = control_costs.maxCoeff();
  control_costs /= (max_coeff > 1e-8) ? max_coeff : 1;
  control_costs *= control_cost_weight;
}

namespace stomp
{
Stomp::Stomp(const StompConfiguration& config, TaskPtr task) : config_(config), task_(task) { resetVariables(); }

bool Stomp::clear() { return resetVariables(); }

void Stomp::setConfig(const StompConfiguration& config)
{
  config_ = config;
  resetVariables();
}

bool Stomp::solve(const std::vector<double>& first,
                  const std::vector<double>& last,
                  Eigen::MatrixXd& parameters_optimized)
{
  // initialize trajectory
  if (!computeInitialTrajectory(first, last))
  {
  }

  return solve(parameters_optimized_, parameters_optimized);
}

bool Stomp::solve(const Eigen::VectorXd& first, const Eigen::VectorXd& last, Eigen::MatrixXd& parameters_optimized)
{
  // converting to std vectors
  std::vector<double> start(first.size());
  std::vector<double> end(last.size());

  Eigen::VectorXd::Map(&start[0], first.size()) = first;
  Eigen::VectorXd::Map(&end[0], last.size()) = last;

  return solve(start, end, parameters_optimized);
}

bool Stomp::solve(const Eigen::MatrixXd& initial_parameters, Eigen::MatrixXd& parameters_optimized)
{
  if (parameters_optimized_.isZero())
  {
    parameters_optimized_ = initial_parameters;
  }

  // check initial trajectory size
  if (initial_parameters.rows() != config_.num_dimensions || initial_parameters.cols() != config_.num_timesteps)
  {
    return false;
  }
  else
  {
    if (initial_parameters.cols() != config_.num_timesteps)
    {
      return false;
    }
  }

  current_iteration_ = 1;
  unsigned int valid_iterations = 0;
  current_lowest_cost_ = std::numeric_limits<double>::max();

  // computing initialial trajectory cost
  if (!computeOptimizedCost())
  {
    return false;
  }

  parameters_valid_prev_ = parameters_valid_;
  while (current_iteration_ <= config_.num_iterations && runSingleIteration())
  {

    if (parameters_valid_)
    {

      valid_iterations++;
    }
    else
    {
      valid_iterations = 0;
    }

    if (valid_iterations > config_.num_iterations_after_valid)
    {
      break;
    }

    current_iteration_++;
  }

  parameters_optimized = parameters_optimized_;

  // notifying task
  task_->done(parameters_valid_, current_iteration_, current_lowest_cost_, parameters_optimized);

  return parameters_valid_;
}

bool Stomp::resetVariables()
{
  proceed_ = true;
  parameters_total_cost_ = 0;
  parameters_valid_ = false;
  num_active_rollouts_ = 0;
  current_iteration_ = 0;

  // verifying configuration
  if (config_.max_rollouts <= config_.num_rollouts)
  {
    config_.max_rollouts = config_.num_rollouts + 1;  // one more to accommodate optimized trajectory
  }

  // noisy rollouts allocation
  int d = config_.num_dimensions;
  num_active_rollouts_ = 0;
  noisy_rollouts_.resize(config_.max_rollouts);
  reused_rollouts_.resize(config_.max_rollouts);

  // initializing rollout
  Rollout rollout;
  rollout.noise.resize(d, config_.num_timesteps);
  rollout.noise.setZero();

  rollout.parameters_noise.resize(d, config_.num_timesteps);
  rollout.parameters_noise.setZero();

  rollout.probabilities.resize(d, config_.num_timesteps);
  rollout.probabilities.setZero();

  rollout.full_probabilities.clear();
  rollout.full_probabilities.resize(d);

  rollout.full_costs.clear();
  rollout.full_costs.resize(d);

  rollout.control_costs.resize(d, config_.num_timesteps);
  rollout.control_costs.setZero();

  rollout.total_costs.resize(d, config_.num_timesteps);
  rollout.total_costs.setZero();

  rollout.state_costs.resize(config_.num_timesteps);
  rollout.state_costs.setZero();

  rollout.importance_weight = DEFAULT_NOISY_COST_IMPORTANCE_WEIGHT;

  for (unsigned int r = 0; r < config_.max_rollouts; r++)
  {
    noisy_rollouts_[r] = rollout;
    reused_rollouts_[r] = rollout;
  }

  // parameter updates
  parameters_updates_.resize(d, config_.num_timesteps);
  parameters_updates_.setZero();

  parameters_control_costs_.resize(d, config_.num_timesteps);
  parameters_control_costs_.setZero();

  parameters_state_costs_.resize(config_.num_timesteps);
  parameters_state_costs_.setZero();

  parameters_optimized_.resize(config_.num_dimensions, config_.num_timesteps);
  parameters_optimized_.setZero();

  // generate finite difference matrix
  start_index_padded_ = FINITE_DIFF_RULE_LENGTH - 1;
  num_timesteps_padded_ = config_.num_timesteps + 2 * (FINITE_DIFF_RULE_LENGTH - 1);
  generateFiniteDifferenceMatrix(
      num_timesteps_padded_, DerivativeOrders::STOMP_ACCELERATION, config_.delta_t, finite_diff_matrix_A_padded_);

  /* control cost matrix (R = A_transpose * A):
   * Note: Original code multiplies the A product by the time interval.  However this is not
   * what was described in the literature
   */
  control_cost_matrix_R_padded_ =
      config_.delta_t * finite_diff_matrix_A_padded_.transpose() * finite_diff_matrix_A_padded_;
  control_cost_matrix_R_ = control_cost_matrix_R_padded_.block(
      start_index_padded_, start_index_padded_, config_.num_timesteps, config_.num_timesteps);
  inv_control_cost_matrix_R_ = control_cost_matrix_R_.fullPivLu().inverse();

  /*
   * Applying scale factor to ensure that max(R^-1)==1
   */
  double maxVal = std::abs(inv_control_cost_matrix_R_.maxCoeff());
  control_cost_matrix_R_padded_ *= maxVal;
  control_cost_matrix_R_ *= maxVal;
  inv_control_cost_matrix_R_ /= maxVal;  // used in computing the minimum control cost initial trajectory

  return true;
}

bool Stomp::computeInitialTrajectory(const std::vector<double>& first, const std::vector<double>& last)
{
  bool valid = true;

  switch (config_.initialization_method)
  {
    case TrajectoryInitializations::CUBIC_POLYNOMIAL_INTERPOLATION:

      computeCubicInterpolation(first, last, config_.num_timesteps, config_.delta_t, parameters_optimized_);
      break;
    case TrajectoryInitializations::LINEAR_INTERPOLATION:

      computeLinearInterpolation(first, last, config_.num_timesteps, parameters_optimized_);
      break;
    case TrajectoryInitializations::MININUM_CONTROL_COST:

      valid = computeMinCostTrajectory(
          first, last, control_cost_matrix_R_padded_, inv_control_cost_matrix_R_, parameters_optimized_);
      break;
  }

  return valid;
}

bool Stomp::cancel()
{
  proceed_ = false;
  return !proceed_;
}

bool Stomp::runSingleIteration()
{
  if (!proceed_)
  {
    return false;
  }

  bool proceed = generateNoisyRollouts() && computeNoisyRolloutsCosts() && filterNoisyRollouts() &&
                 computeProbabilities() && updateParameters() && computeOptimizedCost();

  // notifying end of iteration
  task_->postIteration(0, config_.num_timesteps, current_iteration_, current_lowest_cost_, parameters_optimized_);

  return proceed;
}

bool Stomp::generateNoisyRollouts()
{
  // calculating number of rollouts to reuse from previous iteration
  std::vector<std::pair<double, int> > rollout_cost_sorter;  // Used to sort noisy trajectories in ascending order wrt
                                                             // their total cost
  double h = config_.exponentiated_cost_sensitivity;
  int rollouts_stored = num_active_rollouts_ - 1;  // don't take the optimized rollout into account
  rollouts_stored = rollouts_stored < 0 ? 0 : rollouts_stored;
  int rollouts_generate = config_.num_rollouts;
  int rollouts_total = rollouts_generate + rollouts_stored + 1;
  int rollouts_reuse = rollouts_total < config_.max_rollouts ?
                           rollouts_stored :
                           config_.max_rollouts - (rollouts_generate + 1);  // +1 for optimized params

  // selecting least costly rollouts from previous iteration
  if (rollouts_reuse > 0)
  {
    // find min and max cost for exponential cost scaling
    double min_cost = std::numeric_limits<double>::max();
    double max_cost = std::numeric_limits<double>::min();
    for (int r = 1; r < rollouts_stored; ++r)
    {
      double c = noisy_rollouts_[r].total_cost;
      if (c < min_cost)
        min_cost = c;
      if (c > max_cost)
        max_cost = c;
    }

    double cost_denom = max_cost - min_cost;
    if (cost_denom < 1e-8)
      cost_denom = 1e-8;

    // compute weighted cost on all rollouts
    double cost_prob;
    double weighted_prob;
    for (auto r = 0u; r < rollouts_stored; ++r)
    {
      // Apply noise generated on the previous iteration onto the current trajectory
      noisy_rollouts_[r].noise = noisy_rollouts_[r].parameters_noise - parameters_optimized_;

      cost_prob = exp(-h * (noisy_rollouts_[r].total_cost - min_cost) / cost_denom);
      weighted_prob = cost_prob * noisy_rollouts_[r].importance_weight;
      rollout_cost_sorter.push_back(std::make_pair(-weighted_prob, r));
    }

    std::sort(rollout_cost_sorter.begin(), rollout_cost_sorter.end());

    // use the best ones: (copy them into reused_rollouts)
    for (auto r = 0u; r < rollouts_stored; ++r)
    {
      int reuse_index = rollout_cost_sorter[r].second;
      reused_rollouts_[r] = noisy_rollouts_[reuse_index];
    }

    // copy them back from reused_rollouts_ into rollouts_
    for (auto r = 0u; r < rollouts_reuse; ++r)
    {
      noisy_rollouts_[rollouts_generate + r] = reused_rollouts_[r];
    }
  }

  // adding optimized trajectory as the last rollout
  noisy_rollouts_[rollouts_generate + rollouts_reuse].parameters_noise = parameters_optimized_;
  noisy_rollouts_[rollouts_generate + rollouts_reuse].noise.setZero();
  noisy_rollouts_[rollouts_generate + rollouts_reuse].state_costs = parameters_state_costs_;
  noisy_rollouts_[rollouts_generate + rollouts_reuse].control_costs = parameters_control_costs_;

  // generate new noisy rollouts
  for (auto r = 0u; r < rollouts_generate; r++)
  {
    if (!proceed_)
    {
      return false;
    }

    if (!task_->generateNoisyParameters(parameters_optimized_,
                                        0,
                                        config_.num_timesteps,
                                        current_iteration_,
                                        r,
                                        noisy_rollouts_[r].parameters_noise,
                                        noisy_rollouts_[r].noise))
    {
      return false;
    }
  }

  // update total active rollouts
  num_active_rollouts_ = rollouts_reuse + rollouts_generate + 1;

  return true;
}

bool Stomp::filterNoisyRollouts()
{
  // apply post noise generation filters
  bool filtered = false;
  for (auto r = 0u; r < config_.num_rollouts; r++)
  {
    if (!proceed_)
    {
      return false;
    }

    if (!task_->filterNoisyParameters(
            0, config_.num_timesteps, current_iteration_, r, noisy_rollouts_[r].parameters_noise, filtered))
    {
      return false;
    }

    if (filtered)
    {
      noisy_rollouts_[r].noise = noisy_rollouts_[r].parameters_noise - parameters_optimized_;
    }
  }

  return true;
}

bool Stomp::computeNoisyRolloutsCosts()
{
  // computing state and control costs
  bool valid = computeRolloutsStateCosts() && computeRolloutsControlCosts();

  if (valid)
  {
    // compute total costs
    double total_state_cost;
    double total_control_cost;

    for (auto r = 0u; r < num_active_rollouts_; r++)
    {
      Rollout& rollout = noisy_rollouts_[r];
      total_state_cost = rollout.state_costs.sum();

      // Compute control + state cost for each joint
      total_control_cost = 0;
      double ccost = 0;
      for (auto d = 0u; d < config_.num_dimensions; d++)
      {
        ccost = rollout.control_costs.row(d).sum();
        total_control_cost += ccost;
        rollout.full_costs[d] = ccost + total_state_cost;
      }
      rollout.total_cost = total_state_cost + total_control_cost;

      // Compute total cost for each time step
      for (auto d = 0u; d < config_.num_dimensions; d++)
      {
        rollout.total_costs.row(d) = rollout.state_costs.transpose() + rollout.control_costs.row(d);
      }
    }
  }

  return valid;
}

bool Stomp::computeRolloutsStateCosts()
{
  bool all_valid = true;
  bool proceed = true;
  for (auto r = 0u; r < config_.num_rollouts; r++)
  {
    if (!proceed_)
    {
      proceed = false;
      break;
    }

    Rollout& rollout = noisy_rollouts_[r];
    if (!task_->computeNoisyCosts(
            rollout.parameters_noise, 0, config_.num_timesteps, current_iteration_, r, rollout.state_costs, all_valid))
    {
      proceed = false;
      break;
    }
  }

  return proceed;
}
bool Stomp::computeRolloutsControlCosts()
{
  Eigen::ArrayXXd Ax;  // accelerations
  for (auto r = 0u; r < num_active_rollouts_; r++)
  {
    Rollout& rollout = noisy_rollouts_[r];

    if (config_.control_cost_weight < MIN_CONTROL_COST_WEIGHT)
    {
      for (auto d = 0u; d < config_.num_dimensions; d++)
      {
        rollout.control_costs.row(d).setConstant(0);
      }
    }
    else
    {
      computeParametersControlCosts(rollout.parameters_noise,
                                    config_.delta_t,
                                    config_.control_cost_weight,
                                    control_cost_matrix_R_,
                                    rollout.control_costs);
    }
  }
  return true;
}

bool Stomp::computeProbabilities()
{
  double cost;
  double min_cost;
  double max_cost;
  double denom;
  double numerator;
  double probl_sum = 0.0;  // total probability sum of all rollouts for each joint
  const double h = config_.exponentiated_cost_sensitivity;
  double exponent = 0;

  for (auto d = 0u; d < config_.num_dimensions; ++d)
  {
    for (auto t = 0u; t < config_.num_timesteps; t++)
    {
      // find min and max cost over all rollouts at timestep 't':
      min_cost = noisy_rollouts_[0].total_costs(d, t);
      max_cost = min_cost;
      for (auto r = 0u; r < num_active_rollouts_; ++r)
      {
        cost = noisy_rollouts_[r].total_costs(d, t);
        if (cost < min_cost)
          min_cost = cost;
        if (cost > max_cost)
          max_cost = cost;
      }

      denom = max_cost - min_cost;

      // prevent division by zero:
      if (denom < MIN_COST_DIFFERENCE)
      {
        denom = MIN_COST_DIFFERENCE;
      }

      probl_sum = 0.0;
      for (auto r = 0u; r < num_active_rollouts_; ++r)
      {
        // this is the exponential term in the probability calculation described in the literature
        exponent = -h * (noisy_rollouts_[r].total_costs(d, t) - min_cost) / denom;
        noisy_rollouts_[r].probabilities(d, t) = noisy_rollouts_[r].importance_weight * exp(exponent);

        probl_sum += noisy_rollouts_[r].probabilities(d, t);
      }

      // scaling each probability value by the sum of all probabilities corresponding to all rollouts at time "t"
      for (auto r = 0u; r < num_active_rollouts_; ++r)
      {
        noisy_rollouts_[r].probabilities(d, t) /= probl_sum;
      }
    }

    // computing full probabilities
    min_cost = noisy_rollouts_[0].full_costs[d];
    max_cost = min_cost;
    double c = 0.0;
    for (int r = 1; r < num_active_rollouts_; ++r)
    {
      c = noisy_rollouts_[r].full_costs[d];
      if (c < min_cost)
        min_cost = c;
      if (c > max_cost)
        max_cost = c;
    }

    denom = max_cost - min_cost;
    denom = denom < MIN_COST_DIFFERENCE ? MIN_COST_DIFFERENCE : denom;

    probl_sum = 0.0;
    for (int r = 0; r < num_active_rollouts_; ++r)
    {
      noisy_rollouts_[r].full_probabilities[d] =
          noisy_rollouts_[r].importance_weight * exp(-h * (noisy_rollouts_[r].full_costs[d] - min_cost) / denom);
      probl_sum += noisy_rollouts_[r].full_probabilities[d];
    }
    for (int r = 0; r < num_active_rollouts_; ++r)
    {
      noisy_rollouts_[r].full_probabilities[d] /= probl_sum;
    }
  }

  return true;
}

bool Stomp::updateParameters()
{
  // computing updates from probabilities using convex combination
  parameters_updates_.setZero();
  for (auto d = 0u; d < config_.num_dimensions; d++)
  {
    for (auto r = 0u; r < num_active_rollouts_; r++)
    {
      auto& rollout = noisy_rollouts_[r];
      parameters_updates_.row(d) += (rollout.noise.row(d).array() * rollout.probabilities.row(d).array()).matrix();
    }
  }

  // filtering updates
  if (!task_->filterParameterUpdates(
          0, config_.num_timesteps, current_iteration_, parameters_optimized_, parameters_updates_))
  {
    return false;
  }

  // updating parameters
  parameters_optimized_ += parameters_updates_;

  return true;
}

bool Stomp::computeOptimizedCost()
{
  // control costs
  parameters_total_cost_ = 0;
  if (config_.control_cost_weight > MIN_CONTROL_COST_WEIGHT)
  {
    computeParametersControlCosts(parameters_optimized_,
                                  config_.delta_t,
                                  config_.control_cost_weight,
                                  control_cost_matrix_R_,
                                  parameters_control_costs_);

    // adding all costs
    parameters_total_cost_ = parameters_control_costs_.rowwise().sum().sum();
  }

  // state costs
  if (task_->computeCosts(parameters_optimized_,
                          0,
                          config_.num_timesteps,
                          current_iteration_,
                          parameters_state_costs_,
                          parameters_valid_))
  {
    parameters_total_cost_ += parameters_state_costs_.sum();
  }
  else
  {
    return false;
  }

  // stop optimizing when valid solution is found
  if (current_lowest_cost_ > parameters_total_cost_)
  {
    current_lowest_cost_ = parameters_total_cost_;
    parameters_valid_prev_ = parameters_valid_;
  }
  else
  {
    if (parameters_valid_prev_)
    {
      // reverting updates as no improvement was made
      parameters_optimized_ -= parameters_updates_;
      parameters_valid_ = parameters_valid_prev_;
    }
  }

  return true;
}

} /* namespace stomp */
