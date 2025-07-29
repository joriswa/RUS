int main() { 
    StompConfig config; 
    config.maxComputeTimeMs = 3000.0; 
    config.enableEarlyStopping = true; 
    std::cout << "Hybrid max iterations: " << config.maxIterations << std::endl; std::cout << "Hybrid time limit: " << config.maxComputeTimeMs << std::endl; std::cout << "Hybrid early stopping: " << config.enableEarlyStopping << std::endl; return 0; }
