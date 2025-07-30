# RUS - Robot Ultrasound System Libraries

This repository contains libraries for robot trajectory planning and ultrasound positioning.

## Libraries

### Core Libraries (Always Built)
- **GeometryLib**: Basic geometry utilities and obstacle detection
- **Hauser10**: Advanced trajectory planning algorithms based on Hauser et al.

### Advanced Libraries (Qt3D dependent)  
- **TrajectoryLib**: Main trajectory planning library with visualization
- **USLib**: Ultrasound-specific trajectory planning extensions

### Applications
- **PathPlanner**: Interactive path planning GUI application
- **ParameterTuning**: Parameter optimization tools

## Build Pipeline

The repository includes a GitHub Actions workflow that automatically builds the libraries as artifacts.

### Automatic Builds

The pipeline runs automatically on:
- Push to `main` or `develop` branches
- Pull requests to `main` or `develop`
- Manual trigger via workflow dispatch

### Build Strategy

The build pipeline uses a two-stage approach:

1. **Basic Build**: Always builds the core libraries (GeometryLib, Hauser10) that have minimal dependencies
2. **Advanced Build**: Attempts to build the full libraries including Qt3D-dependent components

This ensures that at least the core functionality is always available, even if some dependencies are missing.

### Artifacts

Built libraries are uploaded as GitHub Actions artifacts with:
- Static libraries (`.a` files)
- Header files for development
- CMake configuration files for easy integration
- Build manifest with details about what was built

## Local Building

### Dependencies

#### Ubuntu/Debian
```bash
sudo apt-get install \
  cmake \
  build-essential \
  qtbase5-dev \
  qtbase5-dev-tools \
  qt5-qmake \
  libqt5opengl5-dev \
  qt3d5-dev \
  libeigen3-dev \
  libboost-all-dev \
  liborocos-kdl-dev \
  libyaml-cpp-dev \
  libjsoncpp-dev
```

### Building

#### Basic Libraries Only
```bash
mkdir build && cd build
cmake .. -DBUILD_ADVANCED_LIBS=OFF -DBUILD_APPS=OFF
make -j$(nproc)
make install
```

#### Full Build (if dependencies available)
```bash
mkdir build && cd build
cmake .. -DBUILD_ADVANCED_LIBS=ON -DBUILD_APPS=ON
make -j$(nproc)
make install
```

#### Build Options

- `BUILD_ADVANCED_LIBS`: Build TrajectoryLib and USLib (default: ON)
- `BUILD_APPS`: Build GUI applications (default: ON)
- `CMAKE_INSTALL_PREFIX`: Installation directory (default: `/usr/local`)

## Using the Libraries

### CMake Integration

After installation, you can use the libraries in your CMake project:

```cmake
find_package(RUSLibraries REQUIRED)

# For basic libraries
target_link_libraries(your_target GeometryLib Hauser10)

# For advanced libraries (if available)
target_link_libraries(your_target TrajectoryLib USLib)
```

### Individual Library Usage

Each library also provides its own CMake configuration:

```cmake
find_package(GeometryLib REQUIRED)
find_package(Hauser10 REQUIRED)
find_package(TrajectoryLib REQUIRED)  # If built
find_package(USLib REQUIRED)         # If built

target_link_libraries(your_target GeometryLib::GeometryLib)
```

## Development

The build system is designed to be robust and handle missing dependencies gracefully. The core trajectory planning algorithms in GeometryLib and Hauser10 can be built and used independently of the Qt-based visualization components.

## License

[Add license information here]