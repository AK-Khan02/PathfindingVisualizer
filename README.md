# Pathfinding Algorithm Visualizer

A sophisticated web-based tool for visualizing various pathfinding algorithms in real-time. Built with vanilla JavaScript and modern web technologies, this visualizer helps users understand how different pathfinding algorithms work through interactive visualization.

## Features

### 15 Different Pathfinding Algorithms
1. **Dijkstra's Algorithm** - Guarantees shortest path using breadth-first approach
2. **A* Search** - Optimized pathfinding using heuristic functions
3. **Greedy Best-First Search** - Fast but non-optimal heuristic-based search
4. **Breadth-First Search (BFS)** - Systematic level-by-level exploration
5. **Depth-First Search (DFS)** - Deep branch exploration with backtracking
6. **Bidirectional Search** - Simultaneous search from start and goal
7. **Jump Point Search (JPS)** - Optimized for uniform-cost grids
8. **IDA* Search** - Memory-efficient iterative deepening A*
9. **Bellman-Ford Algorithm** - Handles negative weights (simplified for grid)
10. **Floyd-Warshall Algorithm** - All-pairs shortest path algorithm
11. **Random Walk** - Non-deterministic exploration with backtracking
12. **Hill Climbing** - Simple gradient descent pathfinding
13. **Beam Search** - Width-limited best-first search
14. **Iterative Deepening DFS** - Depth-limited search with increasing depth
15. **Swarm Algorithm** - Hybrid approach combining Dijkstra's and A*

### Interactive Features
- Click and drag to create walls/obstacles
- Real-time visualization of algorithm execution
- Clear board functionality
- Algorithm information panel with detailed explanations
- Responsive black and white design

## Technical Implementation

### Project Structure
```
pathfinding-visualizer/
├── index.html          # Main HTML structure
├── style.css          # Styling and layout
├── main.js           # Core algorithm implementations
└── README.md         # Documentation
```

### Core Components

#### Grid System
- 15x15 grid for optimal visibility
- Interactive cell system for wall creation
- Start (green) and end (red) nodes
- Visited nodes (blue) and final path (yellow)

#### Visualization Engine
- Asynchronous execution for smooth animation
- Color-coded states for different node types
- Real-time update of explored nodes
- Path reconstruction visualization

#### Algorithm Information Panel
- Dynamic updates based on selected algorithm
- Detailed explanation of algorithm behavior
- Description of obstacle handling
- Visualization process explanation

### Technologies Used

#### HTML5
- Semantic structure for the application
- Provides the core document structure
- Implements accessibility-friendly markup
- Creates the basic layout with `<div>` containers for grid and controls

#### CSS3
- Responsive grid layout using CSS Grid and Flexbox
- Black and white color scheme implementation
- Handles cell state visualizations (start, end, wall, visited, path)
- Responsive design with media queries
- Manages layout flexibilty and grid cell interactions
- Provides smooth transitions and hover effects

#### JavaScript (ES6+)
- **Core Algorithm Implementations**
  - Dynamic pathfinding algorithm classes
  - Asynchronous algorithm execution
  - Efficient data structure management (Sets, Maps)
  - State tracking for grid and node exploration

- **Event Handling**
  - Mouse interaction for wall creation
  - Algorithm selection and visualization triggers
  - Dynamic UI updates

- **Visualization Engine**
  - Async/await for smooth animations
  - Node state management
  - Path reconstruction algorithms
  - Performance-optimized exploration methods

- **Advanced Features**
  - Closure-based state management
  - Functional programming techniques
  - Event delegation
  - Modular algorithm design

#### Vite
- Modern build tool for web development
- Provides fast development server
- Offers quick hot module replacement (HMR)
- Supports ES modules out of the box
- Efficient bundling and optimization
- Initially considered for this project, but ultimately not used in favor of vanilla JavaScript for educational and performance reasons

#### Web APIs Used
- **DOM Manipulation**
  - `document.createElement()`
  - `addEventListener()`
  - `classList` for dynamic styling

- **Async Programming**
  - `Promise`-based visualization
  - `setTimeout()` for controlled animations
  - Asynchronous algorithm exploration

#### Performance Optimization Techniques
- Minimal DOM manipulation
- Efficient data structure selection
- Throttled animations
- Minimal external dependencies
- Leveraging native browser APIs

#### Design Principles
- Mobile-first responsive design
- Accessibility considerations
- Clean, minimalist UI
- Educational visualization approach

*Note: No external libraries or frameworks were used, showcasing pure vanilla JavaScript capabilities.*

## Getting Started

### Prerequisites
- Modern web browser (Chrome, Firefox, Safari, Edge)
- Basic understanding of pathfinding concepts

### Installation
1. Clone the repository:
```bash
git clone [repository-url]
```

2. Navigate to the project directory:
```bash
cd pathfinding-visualizer
```

3. Open `index.html` in your web browser:
```bash
# On Windows
start index.html

# On macOS
open index.html

# On Linux
xdg-open index.html
```

### Usage
1. Select an algorithm from the dropdown menu
2. Click and drag on the grid to create walls
3. Click "Visualize" to start the pathfinding process
4. Use "Clear Board" to reset the grid

## Algorithm Complexity

| Algorithm | Time Complexity | Space Complexity | Optimal Path? |
|-----------|----------------|------------------|---------------|
| Dijkstra  | O(V²)         | O(V)            | Yes          |
| A*        | O(E)          | O(V)            | Yes          |
| BFS       | O(V + E)      | O(V)            | Yes          |
| DFS       | O(V + E)      | O(V)            | No           |
| Bidirectional| O(V + E)    | O(V)            | Yes          |

*V = number of vertices (nodes), E = number of edges*

## Performance Considerations
- Animations are throttled for visualization purposes
- Grid size optimized for performance and visibility
- Efficient data structures (Sets and Maps) for tracking visited nodes
- Asynchronous execution prevents UI blocking

## Browser Support
- Chrome (latest)
- Firefox (latest)
- Safari (latest)
- Edge (latest)

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
- Inspired by various pathfinding visualization tools
- Built with modern web development best practices
- Designed for educational purposes
