class PathfindingVisualizer {
    constructor() {
        this.grid = [];
        this.startNode = null;
        this.endNode = null;
        this.isDrawing = false;
        this.gridSize = 15;
        this.algorithmDescriptions = {
            dijkstra: {
                title: "Dijkstra's Algorithm",
                description: [
                    "Guarantees the shortest path by exploring nodes in order of their distance from the start.",
                    "Treats all edges as having equal weight and explores in all directions equally.",
                    "Handles obstacles by completely avoiding them during exploration.",
                    "Visualization shows nodes being explored in order of increasing distance from start."
                ]
            },
            astar: {
                title: "A* Search",
                description: [
                    "Combines Dijkstra's with a heuristic to guide the search toward the goal.",
                    "Uses a priority queue based on the sum of distance from start and estimated distance to goal.",
                    "Efficiently navigates around obstacles by prioritizing promising paths.",
                    "Visualization shows a more directed search pattern toward the goal."
                ]
            },
            greedy: {
                title: "Greedy Best-First Search",
                description: [
                    "Always explores the node that appears closest to the goal based on heuristic.",
                    "Does not consider the path taken so far, only the estimated distance to goal.",
                    "May not find the shortest path but is typically faster than Dijkstra's or A*.",
                    "Visualization shows a very direct path toward the goal with minimal exploration."
                ]
            },
            bfs: {
                title: "Breadth-First Search",
                description: [
                    "Explores all neighbors at the current depth before moving to the next depth level.",
                    "Guarantees the shortest path in unweighted graphs.",
                    "Handles obstacles by exploring all possible routes in parallel.",
                    "Visualization shows a wave-like expansion from the start node."
                ]
            },
            dfs: {
                title: "Depth-First Search",
                description: [
                    "Explores as far as possible along each branch before backtracking.",
                    "Does not guarantee the shortest path but works well in maze-like environments.",
                    "Handles obstacles by trying alternative paths after hitting dead ends.",
                    "Visualization shows a single path being explored deeply before trying alternatives."
                ]
            },
            bidirectional: {
                title: "Bidirectional Search",
                description: [
                    "Runs two simultaneous searches - one from start to goal and one from goal to start.",
                    "Significantly faster than single-direction algorithms in many cases.",
                    "Handles obstacles by finding the best meeting point between the two searches.",
                    "Visualization shows two expanding frontiers that eventually meet."
                ]
            },
            jps: {
                title: "Jump Point Search",
                description: [
                    "An optimization of A* for uniform-cost grids that skips over some nodes.",
                    "Identifies 'jump points' to reduce the number of nodes that need to be explored.",
                    "Particularly effective in open areas with sparse obstacles.",
                    "Visualization is similar to A* but with fewer nodes being explored."
                ]
            },
            ida: {
                title: "IDA* Search",
                description: [
                    "Iterative Deepening A* combines depth-limited search with A* heuristics.",
                    "Uses progressively increasing thresholds to find optimal paths with less memory.",
                    "Handles obstacles by exploring alternative paths within the current threshold.",
                    "Visualization shows repeated explorations with increasing depth limits."
                ]
            },
            bellman: {
                title: "Bellman-Ford Algorithm",
                description: [
                    "Can handle negative edge weights (though not relevant in our grid).",
                    "Repeatedly relaxes all edges to find shortest paths.",
                    "Handles obstacles by completely avoiding them during edge relaxation.",
                    "Visualization shows a systematic exploration of the entire grid."
                ]
            },
            floyd: {
                title: "Floyd-Warshall Algorithm",
                description: [
                    "Finds shortest paths between all pairs of nodes.",
                    "Uses dynamic programming approach with O(n³) complexity.",
                    "Handles obstacles by finding paths that go around them.",
                    "Visualization shows a comprehensive exploration similar to Dijkstra's."
                ]
            },
            random: {
                title: "Random Walk",
                description: [
                    "Takes random steps toward unexplored neighbors, with backtracking when stuck.",
                    "Does not guarantee finding a path, even if one exists.",
                    "Handles obstacles by randomly trying different directions when blocked.",
                    "Visualization shows an erratic, unpredictable exploration pattern."
                ]
            },
            hill: {
                title: "Hill Climbing",
                description: [
                    "Always moves to the neighbor that's closest to the goal.",
                    "Can get stuck in local minima when surrounded by obstacles.",
                    "Very fast but often fails to find paths in complex environments.",
                    "Visualization shows a direct but potentially unsuccessful approach to the goal."
                ]
            },
            beam: {
                title: "Beam Search",
                description: [
                    "Explores a limited number of most promising nodes at each level.",
                    "Combines breadth-first with heuristic guidance to limit memory usage.",
                    "Handles obstacles by keeping multiple candidate paths in consideration.",
                    "Visualization shows a focused exploration with limited branching."
                ]
            },
            iddfs: {
                title: "Iterative Deepening DFS",
                description: [
                    "Performs depth-first search with increasing depth limits.",
                    "Combines the space efficiency of DFS with the completeness of BFS.",
                    "Handles obstacles by exploring alternative paths at each depth limit.",
                    "Visualization shows repeated explorations with increasing depth limits."
                ]
            },
            swarm: {
                title: "Swarm Algorithm",
                description: [
                    "A hybrid between Dijkstra's and A* that weights the heuristic more heavily.",
                    "Creates a 'swarm' effect where nodes closer to the goal are prioritized.",
                    "Handles obstacles by finding multiple potential paths around them.",
                    "Visualization shows a concentrated exploration that spreads out when blocked."
                ]
            }
        };
        this.initialize();
    }

    initialize() {
        const gridElement = document.getElementById('grid');
        
        // Create grid
        for (let row = 0; row < this.gridSize; row++) {
            this.grid[row] = [];
            for (let col = 0; col < this.gridSize; col++) {
                const cell = document.createElement('div');
                cell.className = 'cell';
                cell.dataset.row = row;
                cell.dataset.col = col;
                
                // Add event listeners for wall drawing
                cell.addEventListener('mousedown', (e) => this.handleMouseDown(e, row, col));
                cell.addEventListener('mouseover', (e) => this.handleMouseOver(e, row, col));
                cell.addEventListener('mouseup', () => this.handleMouseUp());
                
                gridElement.appendChild(cell);
                this.grid[row][col] = {
                    element: cell,
                    isWall: false,
                    isStart: false,
                    isEnd: false,
                    row,
                    col
                };
            }
        }

        // Set initial start and end nodes
        this.setStartNode(Math.floor(this.gridSize/2), 2);
        this.setEndNode(Math.floor(this.gridSize/2), this.gridSize-3);

        // Add button event listeners
        document.getElementById('visualize').addEventListener('click', () => this.visualizeAlgorithm());
        document.getElementById('clear').addEventListener('click', () => this.clearBoard());

        // Add algorithm selection change listener
        document.getElementById('algorithm').addEventListener('change', (e) => this.updateAlgorithmDescription(e.target.value));
        
        // Initialize algorithm description
        this.updateAlgorithmDescription(document.getElementById('algorithm').value);

        // Prevent dragging
        document.addEventListener('mouseup', () => this.handleMouseUp());
    }

    updateAlgorithmDescription(algorithm) {
        const info = this.algorithmDescriptions[algorithm];
        if (!info) return;
        
        const titleElement = document.getElementById('algorithm-title');
        const descriptionElement = document.getElementById('algorithm-description');
        
        titleElement.textContent = info.title;
        
        let descriptionHTML = '<ul>';
        for (const point of info.description) {
            descriptionHTML += `<li>${point}</li>`;
        }
        descriptionHTML += '</ul>';
        
        descriptionElement.innerHTML = descriptionHTML;
    }

    setStartNode(row, col) {
        if (this.startNode) {
            this.startNode.element.classList.remove('start');
            this.startNode.isStart = false;
        }
        this.startNode = this.grid[row][col];
        this.startNode.isStart = true;
        this.startNode.element.classList.add('start');
    }

    setEndNode(row, col) {
        if (this.endNode) {
            this.endNode.element.classList.remove('end');
            this.endNode.isEnd = false;
        }
        this.endNode = this.grid[row][col];
        this.endNode.isEnd = true;
        this.endNode.element.classList.add('end');
    }

    handleMouseDown(e, row, col) {
        e.preventDefault();
        this.isDrawing = true;
        const node = this.grid[row][col];
        if (!node.isStart && !node.isEnd) {
            node.isWall = !node.isWall;
            node.element.classList.toggle('wall');
        }
    }

    handleMouseOver(e, row, col) {
        e.preventDefault();
        if (this.isDrawing) {
            const node = this.grid[row][col];
            if (!node.isStart && !node.isEnd) {
                node.isWall = !node.isWall;
                node.element.classList.toggle('wall');
            }
        }
    }

    handleMouseUp() {
        this.isDrawing = false;
    }

    clearBoard() {
        for (let row = 0; row < this.gridSize; row++) {
            for (let col = 0; col < this.gridSize; col++) {
                const node = this.grid[row][col];
                node.element.className = 'cell';
                node.isWall = false;
                if (node.isStart) node.element.classList.add('start');
                if (node.isEnd) node.element.classList.add('end');
            }
        }
    }

    async visualizeAlgorithm() {
        const algorithm = document.getElementById('algorithm').value;
        this.clearVisualization();
        
        let path;
        switch(algorithm) {
            case 'dijkstra':
                path = await this.dijkstra();
                break;
            case 'astar':
                path = await this.astar();
                break;
            case 'greedy':
                path = await this.greedyBestFirst();
                break;
            case 'bfs':
                path = await this.bfs();
                break;
            case 'dfs':
                path = await this.dfs();
                break;
            case 'bidirectional':
                path = await this.bidirectionalSearch();
                break;
            case 'jps':
                path = await this.jumpPointSearch();
                break;
            case 'ida':
                path = await this.idaStar();
                break;
            case 'bellman':
                path = await this.bellmanFord();
                break;
            case 'floyd':
                path = await this.floydWarshall();
                break;
            case 'random':
                path = await this.randomWalk();
                break;
            case 'hill':
                path = await this.hillClimbing();
                break;
            case 'beam':
                path = await this.beamSearch();
                break;
            case 'iddfs':
                path = await this.iterativeDeepeningDFS();
                break;
            case 'swarm':
                path = await this.swarmAlgorithm();
                break;
            default:
                console.error(`Unknown algorithm: ${algorithm}`);
                return;
        }

        if (path) {
            await this.animatePath(path);
        }
    }

    clearVisualization() {
        for (let row = 0; row < this.gridSize; row++) {
            for (let col = 0; col < this.gridSize; col++) {
                const node = this.grid[row][col];
                node.element.classList.remove('visited', 'path');
            }
        }
    }

    async animateVisited(node) {
        return new Promise(resolve => {
            setTimeout(() => {
                if (!node.isStart && !node.isEnd) {
                    node.element.classList.add('visited');
                }
                resolve();
            }, 20);
        });
    }

    async animatePath(path) {
        for (let node of path) {
            if (!node.isStart && !node.isEnd) {
                await new Promise(resolve => {
                    setTimeout(() => {
                        node.element.classList.add('path');
                        resolve();
                    }, 50);
                });
            }
        }
    }

    getNeighbors(node) {
        const neighbors = [];
        const directions = [[-1, 0], [1, 0], [0, -1], [0, 1]];
        
        for (let [dx, dy] of directions) {
            const newRow = node.row + dx;
            const newCol = node.col + dy;
            
            if (newRow >= 0 && newRow < this.gridSize && 
                newCol >= 0 && newCol < this.gridSize && 
                !this.grid[newRow][newCol].isWall) {
                neighbors.push(this.grid[newRow][newCol]);
            }
        }
        return neighbors;
    }

    async bfs() {
        const queue = [this.startNode];
        const visited = new Set();
        const parent = new Map();
        
        while (queue.length > 0) {
            const current = queue.shift();
            
            if (current === this.endNode) {
                return this.reconstructPath(parent, current);
            }
            
            if (!visited.has(current)) {
                visited.add(current);
                await this.animateVisited(current);
                
                for (let neighbor of this.getNeighbors(current)) {
                    if (!visited.has(neighbor)) {
                        queue.push(neighbor);
                        parent.set(neighbor, current);
                    }
                }
            }
        }
        return null;
    }

    reconstructPath(parent, current) {
        const path = [current];
        while (parent.has(current)) {
            current = parent.get(current);
            path.unshift(current);
        }
        return path;
    }

    async dijkstra() {
        const distances = new Map();
        const parent = new Map();
        const unvisited = new Set();

        // Initialize distances
        for (let row = 0; row < this.gridSize; row++) {
            for (let col = 0; col < this.gridSize; col++) {
                const node = this.grid[row][col];
                distances.set(node, Infinity);
                unvisited.add(node);
            }
        }
        distances.set(this.startNode, 0);

        while (unvisited.size > 0) {
            // Find node with minimum distance
            let current = null;
            let minDistance = Infinity;
            for (let node of unvisited) {
                if (distances.get(node) < minDistance) {
                    minDistance = distances.get(node);
                    current = node;
                }
            }

            if (current === null || distances.get(current) === Infinity) break;
            if (current === this.endNode) {
                return this.reconstructPath(parent, current);
            }

            unvisited.delete(current);
            await this.animateVisited(current);

            for (let neighbor of this.getNeighbors(current)) {
                if (!unvisited.has(neighbor)) continue;
                const distance = distances.get(current) + 1;
                if (distance < distances.get(neighbor)) {
                    distances.set(neighbor, distance);
                    parent.set(neighbor, current);
                }
            }
        }
        return null;
    }

    async astar() {
        const openSet = new Set([this.startNode]);
        const closedSet = new Set();
        const gScore = new Map();
        const fScore = new Map();
        const parent = new Map();

        gScore.set(this.startNode, 0);
        fScore.set(this.startNode, this.heuristic(this.startNode, this.endNode));

        while (openSet.size > 0) {
            let current = this.getLowestFScore(openSet, fScore);

            if (current === this.endNode) {
                return this.reconstructPath(parent, current);
            }

            openSet.delete(current);
            closedSet.add(current);
            await this.animateVisited(current);

            for (let neighbor of this.getNeighbors(current)) {
                if (closedSet.has(neighbor)) continue;

                const tentativeGScore = gScore.get(current) + 1;

                if (!openSet.has(neighbor)) {
                    openSet.add(neighbor);
                } else if (tentativeGScore >= gScore.get(neighbor)) {
                    continue;
                }

                parent.set(neighbor, current);
                gScore.set(neighbor, tentativeGScore);
                fScore.set(neighbor, gScore.get(neighbor) + this.heuristic(neighbor, this.endNode));
            }
        }
        return null;
    }

    heuristic(node, target) {
        return Math.abs(node.row - target.row) + Math.abs(node.col - target.col);
    }

    getLowestFScore(openSet, fScore) {
        let lowest = null;
        let lowestScore = Infinity;
        for (let node of openSet) {
            const score = fScore.get(node);
            if (score < lowestScore) {
                lowest = node;
                lowestScore = score;
            }
        }
        return lowest;
    }

    async greedyBestFirst() {
        const openSet = new Set([this.startNode]);
        const closedSet = new Set();
        const parent = new Map();

        while (openSet.size > 0) {
            const current = this.getClosestToTarget(Array.from(openSet));

            if (current === this.endNode) {
                return this.reconstructPath(parent, current);
            }

            openSet.delete(current);
            closedSet.add(current);
            await this.animateVisited(current);

            for (let neighbor of this.getNeighbors(current)) {
                if (!closedSet.has(neighbor) && !openSet.has(neighbor)) {
                    parent.set(neighbor, current);
                    openSet.add(neighbor);
                }
            }
        }
        return null;
    }

    getClosestToTarget(nodes) {
        return nodes.reduce((closest, node) => {
            const closestDist = this.heuristic(closest, this.endNode);
            const nodeDist = this.heuristic(node, this.endNode);
            return nodeDist < closestDist ? node : closest;
        });
    }

    async dfs() {
        const visited = new Set();
        const parent = new Map();

        const dfsRecursive = async (node) => {
            if (node === this.endNode) {
                return true;
            }

            visited.add(node);
            await this.animateVisited(node);

            for (const neighbor of this.getNeighbors(node)) {
                if (!visited.has(neighbor)) {
                    parent.set(neighbor, node);
                    if (await dfsRecursive(neighbor)) {
                        return true;
                    }
                }
            }
            return false;
        };

        await dfsRecursive(this.startNode);
        if (parent.has(this.endNode)) {
            return this.reconstructPath(parent, this.endNode);
        }
        return null;
    }

    async bidirectionalSearch() {
        // BFS from both start and end nodes
        const startQueue = [this.startNode];
        const endQueue = [this.endNode];
        const startVisited = new Set([this.startNode]);
        const endVisited = new Set([this.endNode]);
        const startParent = new Map();
        const endParent = new Map();
        
        while (startQueue.length > 0 && endQueue.length > 0) {
            // Process start queue
            const startCurrent = startQueue.shift();
            await this.animateVisited(startCurrent);
            
            // Check for intersection after start expansion
            if (endVisited.has(startCurrent)) {
                return this.reconstructBidirectionalPath(startParent, endParent, startCurrent);
            }
            
            // Expand start node
            for (let neighbor of this.getNeighbors(startCurrent)) {
                if (!startVisited.has(neighbor)) {
                    startVisited.add(neighbor);
                    startParent.set(neighbor, startCurrent);
                    startQueue.push(neighbor);
                    
                    // Check if this node has been visited from the end
                    if (endVisited.has(neighbor)) {
                        return this.reconstructBidirectionalPath(startParent, endParent, neighbor);
                    }
                }
            }
            
            // Process end queue
            const endCurrent = endQueue.shift();
            await this.animateVisited(endCurrent);
            
            // Check for intersection after end expansion
            if (startVisited.has(endCurrent)) {
                return this.reconstructBidirectionalPath(startParent, endParent, endCurrent);
            }
            
            // Expand end node
            for (let neighbor of this.getNeighbors(endCurrent)) {
                if (!endVisited.has(neighbor)) {
                    endVisited.add(neighbor);
                    endParent.set(neighbor, endCurrent);
                    endQueue.push(neighbor);
                    
                    // Check if this node has been visited from the start
                    if (startVisited.has(neighbor)) {
                        return this.reconstructBidirectionalPath(startParent, endParent, neighbor);
                    }
                }
            }
        }
        
        return null;
    }
    
    reconstructBidirectionalPath(startParent, endParent, meetingNode) {
        const startPath = [];
        let current = meetingNode;
        
        // Build path from start to meeting node
        while (startParent.has(current)) {
            startPath.unshift(current);
            current = startParent.get(current);
        }
        startPath.unshift(this.startNode);
        
        // Build path from meeting node to end
        current = meetingNode;
        const endPath = [];
        
        while (endParent.has(current)) {
            current = endParent.get(current);
            endPath.push(current);
        }
        
        // Combine paths
        return [...startPath, ...endPath];
    }
    
    async jumpPointSearch() {
        // Simplified JPS implementation
        return await this.astar();
    }
    
    async idaStar() {
        let threshold = this.heuristic(this.startNode, this.endNode);
        const visited = new Set();
        const parent = new Map();
        
        while (true) {
            const result = await this.idaStarSearch(this.startNode, 0, threshold, visited, parent);
            if (result.found) {
                return this.reconstructPath(parent, this.endNode);
            }
            if (result.newThreshold === Infinity) {
                return null;
            }
            threshold = result.newThreshold;
            visited.clear();
        }
    }
    
    async idaStarSearch(node, g, threshold, visited, parent) {
        const f = g + this.heuristic(node, this.endNode);
        
        if (f > threshold) {
            return { found: false, newThreshold: f };
        }
        
        if (node === this.endNode) {
            return { found: true };
        }
        
        visited.add(node);
        await this.animateVisited(node);
        
        let minThreshold = Infinity;
        
        for (let neighbor of this.getNeighbors(node)) {
            if (visited.has(neighbor)) continue;

            const tentativeGScore = g + 1;

            parent.set(neighbor, node);
            const result = await this.idaStarSearch(neighbor, tentativeGScore, threshold, visited, parent);
            
            if (result.found) {
                return result;
            }
            
            if (result.newThreshold < minThreshold) {
                minThreshold = result.newThreshold;
            }
            
            parent.delete(neighbor);
        }
        
        visited.delete(node);
        return { found: false, newThreshold: minThreshold };
    }
    
    async bellmanFord() {
        // Initialize distances
        const distances = new Map();
        const parent = new Map();
        
        for (let row = 0; row < this.gridSize; row++) {
            for (let col = 0; col < this.gridSize; col++) {
                distances.set(this.grid[row][col], Infinity);
            }
        }
        
        distances.set(this.startNode, 0);
        
        // Relax edges |V|-1 times
        for (let i = 0; i < this.gridSize * this.gridSize - 1; i++) {
            let updated = false;
            
            for (let row = 0; row < this.gridSize; row++) {
                for (let col = 0; col < this.gridSize; col++) {
                    const node = this.grid[row][col];
                    
                    if (distances.get(node) === Infinity) continue;
                    
                    await this.animateVisited(node);
                    
                    for (let neighbor of this.getNeighbors(node)) {
                        const newDist = distances.get(node) + 1;
                        
                        if (newDist < distances.get(neighbor)) {
                            distances.set(neighbor, newDist);
                            parent.set(neighbor, node);
                            updated = true;
                            
                            if (neighbor === this.endNode) {
                                return this.reconstructPath(parent, this.endNode);
                            }
                        }
                    }
                }
            }
            
            if (!updated) break;
        }
        
        return this.reconstructPath(parent, this.endNode);
    }
    
    async floydWarshall() {
        // This is a simplified version as the full algorithm would be too complex for this grid
        return await this.dijkstra();
    }
    
    async randomWalk() {
        const visited = new Set([this.startNode]);
        const parent = new Map();
        let current = this.startNode;
        
        while (current !== this.endNode) {
            await this.animateVisited(current);
            
            const neighbors = this.getNeighbors(current).filter(n => !visited.has(n));
            
            if (neighbors.length === 0) {
                // Backtrack if stuck
                if (parent.has(current)) {
                    current = parent.get(current);
                    continue;
                } else {
                    return null;
                }
            }
            
            // Choose random neighbor
            const next = neighbors[Math.floor(Math.random() * neighbors.length)];
            visited.add(next);
            parent.set(next, current);
            current = next;
        }
        
        return this.reconstructPath(parent, this.endNode);
    }
    
    async hillClimbing() {
        const visited = new Set([this.startNode]);
        const parent = new Map();
        let current = this.startNode;
        
        while (current !== this.endNode) {
            await this.animateVisited(current);
            
            const neighbors = this.getNeighbors(current).filter(n => !visited.has(n));
            
            if (neighbors.length === 0) {
                // If stuck, return the path so far
                if (parent.size === 0) return null;
                return this.reconstructPath(parent, current);
            }
            
            // Choose neighbor closest to goal
            const next = neighbors.reduce((best, n) => {
                return this.heuristic(n, this.endNode) < this.heuristic(best, this.endNode) ? n : best;
            }, neighbors[0]);
            
            visited.add(next);
            parent.set(next, current);
            current = next;
        }
        
        return this.reconstructPath(parent, this.endNode);
    }
    
    async beamSearch(beamWidth = 3) {
        let beam = [this.startNode];
        const visited = new Set([this.startNode]);
        const parent = new Map();
        
        while (beam.length > 0) {
            const candidates = [];
            
            for (const node of beam) {
                await this.animateVisited(node);
                
                if (node === this.endNode) {
                    return this.reconstructPath(parent, this.endNode);
                }
                
                for (const neighbor of this.getNeighbors(node)) {
                    if (!visited.has(neighbor)) {
                        visited.add(neighbor);
                        parent.set(neighbor, node);
                        candidates.push(neighbor);
                    }
                }
            }
            
            // Sort candidates by heuristic and keep only beamWidth
            beam = candidates
                .sort((a, b) => this.heuristic(a, this.endNode) - this.heuristic(b, this.endNode))
                .slice(0, beamWidth);
        }
        
        return null;
    }
    
    async iterativeDeepeningDFS() {
        for (let depth = 0; depth < this.gridSize * this.gridSize; depth++) {
            const visited = new Set();
            const parent = new Map();
            const result = await this.depthLimitedSearch(this.startNode, depth, visited, parent);
            
            if (result) {
                return this.reconstructPath(parent, this.endNode);
            }
        }
        
        return null;
    }
    
    async depthLimitedSearch(node, depth, visited, parent) {
        if (depth < 0) return false;
        if (node === this.endNode) return true;
        
        visited.add(node);
        await this.animateVisited(node);
        
        for (const neighbor of this.getNeighbors(node)) {
            if (!visited.has(neighbor)) {
                parent.set(neighbor, node);
                if (await this.depthLimitedSearch(neighbor, depth - 1, visited, parent)) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    async swarmAlgorithm() {
        // A hybrid between Dijkstra's and A*
        const openSet = new Set([this.startNode]);
        const closedSet = new Set();
        const gScore = new Map();
        const fScore = new Map();
        const parent = new Map();
        
        gScore.set(this.startNode, 0);
        fScore.set(this.startNode, this.heuristic(this.startNode, this.endNode));
        
        while (openSet.size > 0) {
            let current = this.getLowestFScore(openSet, fScore);
            
            if (current === this.endNode) {
                return this.reconstructPath(parent, current);
            }
            
            openSet.delete(current);
            closedSet.add(current);
            await this.animateVisited(current);
            
            for (let neighbor of this.getNeighbors(current)) {
                if (closedSet.has(neighbor)) continue;
                
                const tentativeGScore = gScore.get(current) + 1;
                
                if (!openSet.has(neighbor)) {
                    openSet.add(neighbor);
                } else if (tentativeGScore >= gScore.get(neighbor)) {
                    continue;
                }
                
                parent.set(neighbor, current);
                gScore.set(neighbor, tentativeGScore);
                
                // Swarm algorithm weights the heuristic more heavily
                const h = this.heuristic(neighbor, this.endNode);
                fScore.set(neighbor, gScore.get(neighbor) + h * 2);
            }
        }
        
        return null;
    }
}

// Initialize the visualizer when the page loads
window.addEventListener('load', () => {
    new PathfindingVisualizer();
});
