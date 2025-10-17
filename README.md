# Chip's Challenge: Intelligent Agent Simulation

## Project Overview

**Chip's Challenge** is an advanced **artificial intelligence** and **autonomous agent** simulation project that demonstrates sophisticated **pathfinding algorithms**, **decision-making systems**, and **multi-objective optimization** in a dynamic game environment. This project showcases cutting-edge **machine learning** principles, **algorithmic intelligence**, and **computational problem-solving** techniques through the implementation of an intelligent robotic agent capable of navigating complex maze environments while collecting resources and solving puzzles.

## Key Features & Technologies

### Core AI Technologies
- **A* Pathfinding Algorithm**: Advanced heuristic-based search algorithm for optimal pathfinding
- **Intelligent Decision Making**: Multi-layered decision tree for strategic resource collection
- **State Space Search**: Comprehensive exploration of possible game states
- **Heuristic Optimization**: Manhattan distance calculations for efficient navigation
- **Priority Queue Management**: Sophisticated data structure implementation for pathfinding
- **Dynamic Environment Interaction**: Real-time adaptation to changing game conditions

### Advanced Software Architecture
- **Object-Oriented Design**: Modular, extensible codebase with clean separation of concerns
- **Design Patterns**: Implementation of Strategy, Observer, and Factory patterns
- **Event-Driven Programming**: Asynchronous simulation updates and real-time rendering
- **Configuration Management**: Flexible, externalized configuration system
- **Resource Management**: Efficient memory and computational resource utilization
- **Error Handling**: Robust exception handling and graceful degradation

### Simulation & Visualization
- **Real-Time Visualization**: Interactive GUI with sprite-based rendering
- **Multi-Threaded Processing**: Concurrent simulation execution with timeout management
- **Performance Metrics**: Comprehensive analytics and success rate tracking
- **Configurable Parameters**: Customizable simulation parameters and map configurations
- **Cross-Platform Compatibility**: Java-based implementation for universal deployment

## Game Mechanics & Challenge

The simulation presents a **complex puzzle-solving environment** where an intelligent agent must:

1. **Navigate Dynamic Mazes**: Traverse complex environments with obstacles, walls, and water hazards
2. **Resource Collection Strategy**: Systematically collect colored keys and chips in optimal order
3. **Constraint Satisfaction**: Solve multi-constraint problems with interdependent requirements
4. **Goal-Oriented Behavior**: Achieve victory by reaching the portal after completing all objectives
5. **Adaptive Planning**: Modify strategy based on environmental changes and resource availability

### Environmental Elements
- **Spatial Navigation**: 2D grid-based movement with collision detection
- **Resource Management**: Inventory system for key collection and usage
- **Constraint Solving**: Door-key matching and chip collection requirements
- **State Transitions**: Dynamic environment updates based on agent actions
- **Performance Optimization**: Efficient algorithms for large-scale problem solving

## Technical Architecture

### Core Components

#### Agent System (Robot.java)
- **Intelligent Pathfinding**: A* algorithm implementation with Manhattan distance heuristics
- **Strategic Planning**: Multi-phase approach (keys → chips → goal)
- **State Management**: Comprehensive tracking of collected resources and current objectives
- **Decision Engine**: Sophisticated logic for target selection and action prioritization

#### Environment Engine (Environment.java)
- **Spatial Modeling**: 2D grid-based world representation with neighbor relationships
- **Collision Detection**: Advanced boundary and obstacle checking algorithms
- **State Synchronization**: Real-time environment updates and agent interaction
- **Resource Tracking**: Dynamic inventory management and collection validation

#### Simulation Framework
- **Visualization Engine**: Real-time GUI with sprite rendering and status panels
- **Configuration System**: Externalized parameters for customizable simulations
- **Testing Infrastructure**: Comprehensive JUnit test suite with performance benchmarking
- **Performance Analytics**: Success rate tracking and optimization metrics

### Algorithmic Innovations

#### A* Pathfinding Implementation
The project implements a sophisticated A* pathfinding algorithm with the following key features:

```java
// Sophisticated heuristic-based search with priority queue optimization
private boolean calculatePath(Position goal) {
    while (!frontier.isEmpty()) {
        Node current = frontier.poll();
        if (current.getPosition().equals(goal)) return true;
        exploreNeighbors(current, positions, goal);
    }
    return false;
}
```

#### Multi-Objective Optimization
- **Phase 1**: Key collection with distance-based prioritization
- **Phase 2**: Chip collection with greedy nearest-neighbor approach
- **Phase 3**: Goal navigation with obstacle avoidance

## Installation & Setup

### Prerequisites
- **Java Development Kit (JDK) 8+**: Required for compilation and execution
- **Integrated Development Environment**: Eclipse, IntelliJ IDEA, or VS Code recommended
- **JUnit 5**: For running comprehensive test suites
- **Git**: For version control and project management

### System Requirements
- **Operating System**: Windows, macOS, or Linux
- **Memory**: Minimum 512MB RAM (2GB recommended)
- **Storage**: 50MB available disk space
- **Display**: 1024x768 resolution minimum for optimal visualization

## How to Run the Program

### Method 1: Visual Simulation (Recommended)

1. **Compile the Project**:
   ```bash
   cd /Users/kushpatel/Documents/repos/ChipsChallenge-1/CSC411_ProblemSet06
   javac -d bin -cp src src/edu/ncsu/csc411/ps06/simulation/VisualizeSimulation.java
   ```

2. **Run the Visual Simulation**:
   ```bash
   java -cp bin edu.ncsu.csc411.ps06.simulation.VisualizeSimulation
   ```

3. **Configuration Options**:
   - Modify `mapFile` variable in `VisualizeSimulation.java` to change maps (10 total maps/challenges)
   - Adjust `configFile` to use different sized visualization windows (`configSmall.txt`, `configNormal.txt`, `configLarge.txt`)
   - Customize simulation parameters in configuration files

### Method 2: Headless Simulation (Fast Testing)

1. **Run Headless Simulation**:
   ```bash
   java -cp bin edu.ncsu.csc411.ps06.simulation.RunSimulation
   ```

2. **Custom Map Testing**:
   ```bash
   java -cp bin edu.ncsu.csc411.ps06.simulation.RunSimulation maps/public/map04.txt 1000
   ```

### Method 3: Comprehensive Testing Suite

1. **Run All Test Cases**:
   ```bash
   # Using JUnit (if configured in IDE)
   java -cp bin:lib/junit-platform-console-standalone.jar org.junit.platform.console.ConsoleLauncher --class-path bin --select-class edu.ncsu.csc411.ps06.public_test_cases.PS06_TestCase
   ```

2. **Individual Test Execution**:
   ```bash
   java -cp bin edu.ncsu.csc411.ps06.public_test_cases.PS06_TestCase
   ```

### Method 4: IDE Integration

1. **Eclipse/IntelliJ Setup**:
   - Import project as existing Java project
   - Configure build path to include `src` directory
   - Set `bin` as output directory
   - Run `VisualizeSimulation.java` as Java Application

2. **VS Code Configuration**:
   - Install Java Extension Pack
   - Open project folder
   - Use "Run Java" command on main classes

## Performance & Optimization

### Success Metrics
- **Success Rate**: 70% minimum across all test cases
- **Performance Threshold**: 2-second timeout per simulation
- **Efficiency**: Optimal pathfinding with minimal computational overhead
- **Reliability**: Consistent performance across multiple trials

### Algorithmic Complexity
- **Time Complexity**: O(b^d) where b is branching factor, d is solution depth
- **Space Complexity**: O(b^d) for frontier and explored sets
- **Heuristic Accuracy**: Manhattan distance provides admissible heuristic
- **Optimization**: Priority queue ensures efficient node selection

## Advanced Features

### Customization Options
- **Map Editor**: Create custom environments using text-based map files
- **Parameter Tuning**: Adjust simulation speed, tile size, and debug settings
- **Algorithm Modification**: Extend pathfinding with additional heuristics
- **Performance Profiling**: Built-in timing and success rate analytics

### Extensibility
- **Multi-Agent Support**: Framework supports multiple concurrent agents
- **Custom Heuristics**: Implement domain-specific optimization functions
- **Advanced AI**: Integration points for machine learning algorithms
- **Plugin Architecture**: Modular design for easy feature additions

## Research Applications

This project demonstrates fundamental concepts in:
- **Artificial Intelligence**: Search algorithms, heuristics, and problem-solving
- **Computer Science**: Data structures, algorithms, and software engineering
- **Game Theory**: Strategic decision-making and optimization
- **Cognitive Science**: Agent behavior modeling and simulation

## Future Enhancements

- **Machine Learning Integration**: Neural network-based decision making
- **Multi-Agent Coordination**: Collaborative problem-solving algorithms
- **Advanced Visualization**: 3D rendering and interactive debugging tools
- **Performance Optimization**: Parallel processing and GPU acceleration
- **Cloud Deployment**: Scalable simulation hosting and analytics

## Project Structure

```
CSC411_ProblemSet06/
├── src/                          # Source code directory
│   └── edu/ncsu/csc411/ps06/
│       ├── agent/                # AI agent implementation
│       │   └── Robot.java        # Main intelligent agent class
│       ├── environment/          # Game environment and world modeling
│       │   ├── Action.java       # Agent action definitions
│       │   ├── Environment.java  # World state management
│       │   ├── Position.java     # Spatial coordinate system
│       │   ├── Tile.java         # Individual tile representation
│       │   └── TileStatus.java   # Tile type enumerations
│       ├── simulation/           # Simulation and visualization
│       │   ├── RunSimulation.java    # Headless simulation runner
│       │   └── VisualizeSimulation.java # GUI simulation interface
│       └── utils/                # Utility classes and helpers
│           ├── ConfigurationLoader.java # Configuration management
│           └── MapManager.java   # Map file processing
├── maps/                         # Game level definitions
│   └── public/                   # Public test case maps
│       ├── map01.txt            # Test case 1
│       ├── map02.txt            # Test case 2
│       └── ...                  # Additional test cases
├── config/                       # Configuration files
│   ├── configSmall.txt          # Small display configuration
│   ├── configNormal.txt         # Normal display configuration
│   └── configLarge.txt          # Large display configuration
├── sprite/                       # Visual assets and graphics
│   ├── chip_forward.png         # Agent sprite
│   ├── key_*.png               # Key sprites (blue, green, red, yellow)
│   ├── door_*.png              # Door sprites
│   └── ...                     # Additional visual assets
├── test/                        # Test suite and validation
│   └── edu/ncsu/csc411/ps06/
│       └── public_test_cases/
│           └── PS06_TestCase.java # JUnit test cases
├── bin/                         # Compiled bytecode
└── README.md                    # This documentation file
```

## Map File Format

The project uses a simple text-based format for defining game levels:

```
CH BL BL WL BL BL BL
BL BL BL WL BL BL PL
ST BL BL DP BL BL BL
BL BL BL WL BL BL BL
```

### Map Legend
- **ST**: Agent Starting Position
- **BL**: Blank Tile (walkable)
- **WL**: Wall Tile (obstacle)
- **WA**: Water Tile (obstacle)
- **CH**: Chip (collectible)
- **PL**: Portal (goal)
- **DP**: Door to Portal
- **DG/DY/DB/DR**: Door (Green/Yellow/Blue/Red)
- **KG/KY/KB/KR**: Key (Green/Yellow/Blue/Red)

## Configuration Parameters

### Visual Simulation Settings
- **ITERATIONS**: Maximum simulation steps (default: 1000)
- **TILESIZE**: Pixel size of each tile (default: 64)
- **DELAY**: Milliseconds between simulation updates (default: 200)
- **DEBUG**: Enable debug output (default: true)

### Performance Tuning
- **TIMEOUT**: Test case timeout in milliseconds (default: 2000)
- **NUM_TRIALS**: Number of test trials per map (default: 10)
- **PASS_THRESHOLD**: Success rate threshold for passing (default: 0.7)

## Troubleshooting

### Common Issues

1. **Compilation Errors**:
   - Ensure JDK 8+ is installed and configured
   - Check that all source files are in the correct package structure
   - Verify classpath includes both `src` and `bin` directories

2. **Runtime Errors**:
   - Confirm all sprite files are present in the `sprite/` directory
   - Check that map files exist and are properly formatted
   - Verify configuration files are accessible

3. **Performance Issues**:
   - Reduce `TILESIZE` for better performance on slower systems
   - Increase `DELAY` to slow down simulation for better visualization
   - Use `configSmall.txt` for smaller display windows

4. **Test Failures**:
   - Check that agent implementation meets success rate requirements
   - Verify timeout settings are appropriate for system performance
   - Ensure all required dependencies are available


## License

This project is developed for educational purposes as part of CSC411 Problem Set 06. All rights reserved by the original authors and North Carolina State University.

---

**Developed as part of CSC411 Problem Set 06 - Advanced AI and Agent Simulation**

*This project showcases the intersection of artificial intelligence, algorithmic design, and software engineering principles in creating intelligent autonomous systems capable of solving complex, multi-constraint optimization problems.*
