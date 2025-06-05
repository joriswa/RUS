#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>

class STOMP1D
{
public:
    STOMP1D(int num_timesteps,
            int num_iterations,
            int num_rollouts,
            double learning_rate,
            double noise_stddev)
        : num_timesteps_(num_timesteps)
        , num_iterations_(num_iterations)
        , num_rollouts_(num_rollouts)
        , learning_rate_(learning_rate)
        , noise_stddev_(noise_stddev)
    {
        trajectory_ = Eigen::VectorXd::Zero(num_timesteps_);
        R_ = computeControlCostMatrix();
    }

    Eigen::VectorXd optimize(const Eigen::Vector2d &start_goal)
    {
        initializeTrajectory(start_goal);

        for (int iter = 0; iter < num_iterations_; ++iter) {
            Eigen::MatrixXd noisy_trajectories = generateNoisyTrajectories();
            Eigen::VectorXd costs = computeCosts(noisy_trajectories);
            Eigen::VectorXd probabilities = computeProbabilities(costs);
            updateTrajectory(noisy_trajectories, probabilities);
        }

        return trajectory_;
    }

private:
    void initializeTrajectory(const Eigen::Vector2d &start_goal)
    {
        double start = start_goal(0);
        double goal = start_goal(1);
        for (int i = 0; i < num_timesteps_; ++i) {
            trajectory_(i) = start + (goal - start) * i / (num_timesteps_ - 1);
        }
    }

    Eigen::MatrixXd generateNoisyTrajectories()
    {
        Eigen::MatrixXd noisy_trajectories(num_rollouts_, num_timesteps_);
        std::normal_distribution<double> dist(0.0, noise_stddev_);
        std::default_random_engine generator;

        for (int i = 0; i < num_rollouts_; ++i) {
            for (int j = 0; j < num_timesteps_; ++j) {
                noisy_trajectories(i, j) = trajectory_(j) + dist(generator);
            }
        }

        return noisy_trajectories;
    }

    Eigen::VectorXd computeCosts(const Eigen::MatrixXd &noisy_trajectories)
    {
        Eigen::VectorXd costs(num_rollouts_);
        for (int i = 0; i < num_rollouts_; ++i) {
            costs(i) = (noisy_trajectories.row(i).transpose() * R_ * noisy_trajectories.row(i))
                           .value();
        }
        return costs;
    }

    Eigen::VectorXd computeProbabilities(const Eigen::VectorXd &costs)
    {
        double h = -10.0; // Arbitrary value for exponential scaling
        Eigen::VectorXd exp_costs = (h * (costs.array() - costs.mean())).exp();
        return exp_costs / exp_costs.sum();
    }

    void updateTrajectory(const Eigen::MatrixXd &noisy_trajectories,
                          const Eigen::VectorXd &probabilities)
    {
        Eigen::VectorXd update = (noisy_trajectories.transpose() * probabilities).transpose()
                                 - trajectory_;
        trajectory_ += learning_rate_ * update;
    }

    Eigen::MatrixXd computeControlCostMatrix()
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_timesteps_ - 2, num_timesteps_);
        for (int i = 0; i < num_timesteps_ - 2; ++i) {
            A(i, i) = 1;
            A(i, i + 1) = -2;
            A(i, i + 2) = 1;
        }
        return A.transpose() * A;
    }

    int num_timesteps_;
    int num_iterations_;
    int num_rollouts_;
    double learning_rate_;
    double noise_stddev_;
    Eigen::VectorXd trajectory_;
    Eigen::MatrixXd R_;
};

int main()
{
    int num_timesteps = 100;
    int num_iterations = 100;
    int num_rollouts = 20;
    double learning_rate = 0.1;
    double noise_stddev = 0.1;

    STOMP1D stomp(num_timesteps, num_iterations, num_rollouts, learning_rate, noise_stddev);

    Eigen::Vector2d start_goal(0.0, 1.0);
    Eigen::VectorXd optimized_trajectory = stomp.optimize(start_goal);

    std::cout << "Optimized trajectory:" << std::endl;
    std::cout << optimized_trajectory.transpose() << std::endl;

    return 0;
}
