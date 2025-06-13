cmake_minimum_required(VERSION 3.16)

# Define the main evaluator target
add_executable(UltrasoundRepositioningEvaluator
    main.cpp
    repositioning_evaluator.cpp
    repositioning_evaluator.h
)

# Define the SinglePoseEvaluator target
add_executable(SinglePoseEvaluator
    single_pose_examples.cpp
    single_pose_evaluator.cpp
    single_pose_evaluator.h
)

# Define example configs executable
add_executable(ExampleConfigs
    example_configs.cpp
    repositioning_evaluator.cpp
    repositioning_evaluator.h
)

# Define test configs executable
add_executable(TestAlgorithmConfigs
    test_algorithm_configs.cpp
    repositioning_evaluator.cpp
    repositioning_evaluator.h
    single_pose_evaluator.cpp
    single_pose_evaluator.h
)

# Define simple test executable
add_executable(TestSinglePose
    test_single_pose.cpp
    single_pose_evaluator.cpp
    single_pose_evaluator.h
)

# Define simple demo executable
add_executable(SimpleSinglePoseDemo
    simple_single_pose_demo.cpp
    single_pose_evaluator.cpp
    single_pose_evaluator.h
)

# Define clearance functionality test executable
add_executable(TestClearanceFunctionality
    test_clearance_functionality.cpp
    single_pose_evaluator.cpp
    single_pose_evaluator.h
)

# Set target properties for all executables
set_target_properties(UltrasoundRepositioningEvaluator SinglePoseEvaluator ExampleConfigs TestAlgorithmConfigs TestSinglePose SimpleSinglePoseDemo TestClearanceFunctionality PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Include directories for all targets
foreach(target UltrasoundRepositioningEvaluator SinglePoseEvaluator ExampleConfigs TestAlgorithmConfigs TestSinglePose SimpleSinglePoseDemo TestClearanceFunctionality)
    target_include_directories(${target} PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include
        ${CMAKE_SOURCE_DIR}/libs/USLib/include
        ${CMAKE_SOURCE_DIR}/libs/GeometryLib/include
        ${CMAKE_SOURCE_DIR}/libs/Hauser10/include
    )
endforeach()

# Link libraries for all targets
foreach(target UltrasoundRepositioningEvaluator SinglePoseEvaluator ExampleConfigs TestAlgorithmConfigs TestSinglePose SimpleSinglePoseDemo TestClearanceFunctionality)
    target_link_libraries(${target}
        TrajectoryLib
        USLib
        GeometryLib
        Hauser10
        ${CMAKE_THREAD_LIBS_INIT}
    )
endforeach()

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system)

# Link external dependencies for all targets
foreach(target UltrasoundRepositioningEvaluator SinglePoseEvaluator ExampleConfigs TestAlgorithmConfigs TestSinglePose SimpleSinglePoseDemo TestClearanceFunctionality)
    target_link_libraries(${target}
        Eigen3::Eigen
        Boost::filesystem
        Boost::thread
        Boost::system
    )
endforeach()

# Set optimization flags for release for all targets
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    foreach(target UltrasoundRepositioningEvaluator SinglePoseEvaluator ExampleConfigs TestAlgorithmConfigs TestSinglePose SimpleSinglePoseDemo TestClearanceFunctionality)
        target_compile_options(${target} PRIVATE -O3 -DNDEBUG)
    endforeach()
endif()
