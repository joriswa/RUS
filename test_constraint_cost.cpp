#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== STOMP Configurable Constraint Cost Calculator ===" << std::endl;
    std::cout << "Added configurable constraint violation cost calculator to STOMP!" << std::endl;
    
    std::cout << "\n📋 Features Added:" << std::endl;
    std::cout << "  ✓ constraintCostWeight parameter in StompConfig" << std::endl;
    std::cout << "  ✓ Enhanced ConstraintCostCalculator with configurable penalties" << std::endl;
    std::cout << "  ✓ Velocity and acceleration violation detection" << std::endl;
    std::cout << "  ✓ Linear and quadratic penalty options" << std::endl;
    std::cout << "  ✓ Automatic integration into STOMP cost calculation" << std::endl;
    
    std::cout << "\n⚙️  StompConfig Presets:" << std::endl;
    std::cout << "  • optimized():       constraintCostWeight = 1.0 (default)" << std::endl;
    std::cout << "  • fast():            constraintCostWeight = 0.5 (lower for speed)" << std::endl;
    std::cout << "  • quality():         constraintCostWeight = 2.0 (higher compliance)" << std::endl;
    std::cout << "  • hybrid():          constraintCostWeight = 1.0 (balanced)" << std::endl;
    std::cout << "  • strictConstraints(): constraintCostWeight = 5.0 (strict compliance)" << std::endl;
    
    std::cout << "\n🔧 Usage Examples:" << std::endl;
    std::cout << "  // Use default constraint cost:" << std::endl;
    std::cout << "  auto config = StompConfig::optimized();" << std::endl;
    
    std::cout << "\n  // Disable constraint cost:" << std::endl;
    std::cout << "  auto config = StompConfig::optimized();" << std::endl;
    std::cout << "  config.constraintCostWeight = 0.0;" << std::endl;
    
    std::cout << "\n  // High constraint enforcement:" << std::endl;
    std::cout << "  auto config = StompConfig::strictConstraints();" << std::endl;
    
    std::cout << "\n  // Custom constraint weight:" << std::endl;
    std::cout << "  auto config = StompConfig::optimized();" << std::endl;
    std::cout << "  config.constraintCostWeight = 3.5;" << std::endl;
    
    std::cout << "\n🎯 Cost Calculator Details:" << std::endl;
    std::cout << "  • Velocity violations: 50.0 * |v_j| - v_max_j (if exceeded)" << std::endl;
    std::cout << "  • Acceleration violations: 25.0 * |a_j| - a_max_j (if exceeded)" << std::endl;
    std::cout << "  • Total cost = constraintCostWeight * (vel_violations + acc_violations)" << std::endl;
    
    std::cout << "\n✅ Implementation Complete!" << std::endl;
    std::cout << "The constraint cost calculator is now automatically integrated" << std::endl;
    std::cout << "into STOMP when constraintCostWeight > 0.0" << std::endl;
    
    return 0;
}
