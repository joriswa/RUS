#include <Hauser10/ParabolicRamp.h>
#include <Hauser10/DynamicPath.h>
#include <iostream>

int main() {
    std::cout << "RUS Hauser10 Trajectory Planning Example" << std::endl;
    
    // Create a simple 1D trajectory
    ParabolicRamp::ParabolicRamp1D ramp;
    
    // Set start and end conditions
    ramp.x0 = 0.0;   // start position
    ramp.x1 = 10.0;  // end position
    ramp.dx0 = 0.0;  // start velocity
    ramp.dx1 = 0.0;  // end velocity
    
    // Set constraints
    double vmax = 5.0;  // max velocity
    double amax = 2.0;  // max acceleration
    
    // Solve for minimum time trajectory
    if (ramp.SolveMinTime(amax, vmax)) {
        std::cout << "Successfully computed trajectory:" << std::endl;
        std::cout << "  Duration: " << ramp.EndTime() << " seconds" << std::endl;
        std::cout << "  Switch time: " << ramp.tswitch1 << " seconds" << std::endl;
        
        // Evaluate trajectory at different times
        double t_mid = ramp.EndTime() / 2.0;
        double pos_mid = ramp.Evaluate(t_mid);
        double vel_mid = ramp.Derivative(t_mid);
        
        std::cout << "  At t=" << t_mid << "s: position=" << pos_mid 
                  << ", velocity=" << vel_mid << std::endl;
    } else {
        std::cout << "Failed to solve trajectory!" << std::endl;
        return 1;
    }
    
    std::cout << "Hauser10 example completed successfully!" << std::endl;
    return 0;
}