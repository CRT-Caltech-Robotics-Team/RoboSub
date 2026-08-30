#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <SFML/Graphics.hpp>


// ------------------------------------------------------------
// POSITION
// ------------------------------------------------------------
//
// Represents a coordinate in the grid.
//
// Unlike Node*, a Position contains the actual x and y values.
// This is what we return from A* so we don't return pointers
// to Nodes that are destroyed when AStar() finishes.
//
struct Position
{
    int x;
    int y;
};


// ------------------------------------------------------------
// NODE
// ------------------------------------------------------------
//
// Represents one location in the grid while A* is running.
//
// gCost  = actual cost from START to this node
// hCost  = estimated cost from this node to GOAL
// parent = previous node in the path
//
struct Node
{
    int x;
    int y;

    double gCost;
    double hCost;

    Node* parent;

    // Total estimated cost:
    //
    //     f = g + h
    //
    double fCost() const
    {
        return gCost + hCost;
    }
};


// ------------------------------------------------------------
// PRIORITY QUEUE COMPARATOR
// ------------------------------------------------------------
//
// std::priority_queue normally puts the largest value
// at the top.
//
// A* needs the node with the SMALLEST fCost.
//
// This comparator reverses the normal ordering so that
// the lowest fCost has the highest priority.
//
struct CompareNode
{
    bool operator()(Node* a, Node* b)
    {
        return a->fCost() > b->fCost();
    }
};


// ------------------------------------------------------------
// HEURISTIC
// ------------------------------------------------------------
//
// Estimates the distance between two nodes.
//
// We're using Manhattan distance because the submarine
// can currently move only:
//
//     UP
//     DOWN
//     LEFT
//     RIGHT
//
double heuristic(Node* a, Node* b)
{
    return std::abs(a->x - b->x) +
           std::abs(a->y - b->y);
}


// ------------------------------------------------------------
// IS VALID
// ------------------------------------------------------------
//
// Checks whether a coordinate is inside the grid.
//
bool isValid(int x, int y, int width, int height)
{
    return x >= 0 && x < width &&
           y >= 0 && y < height;
}


// ------------------------------------------------------------
// GET NEIGHBORS
// ------------------------------------------------------------
//
// Finds the nodes immediately adjacent to the current node.
//
// Current movement:
//
//        UP
//         ↑
//         |
// LEFT ← NODE → RIGHT
//         |
//         ↓
//       DOWN
//
std::vector<Node*> getNeighbors(
    Node* current,
    std::vector<std::vector<Node>>& grid)
{
    std::vector<Node*> neighbors;

    // Movement directions.
    int directions[4][2] =
    {
        {1, 0},   // Right
        {-1, 0},  // Left
        {0, 1},   // Down
        {0, -1}   // Up
    };

    int width = grid[0].size();
    int height = grid.size();

    // Check each direction.
    for (auto& direction : directions)
    {
        int newX = current->x + direction[0];
        int newY = current->y + direction[1];

        // Only add the neighbor if it is inside
        // the grid.
        if (isValid(newX, newY, width, height))
        {
            neighbors.push_back(&grid[newY][newX]);
        }
    }

    return neighbors;
}


// ------------------------------------------------------------
// A* ALGORITHM
// ------------------------------------------------------------
//
// Finds a path from:
//
//     START = (startX, startY)
//
// to:
//
//     GOAL = (goalX, goalY)
//
// The map contains:
//
//     0 = free space
//     1 = obstacle
//
// Returns:
//
//     vector<Position>
//
// The important part is that we return Position values rather
// than Node pointers. The Nodes only exist while AStar()
// is running.
//
std::vector<Position> AStar(
    std::vector<std::vector<int>>& map,
    int startX,
    int startY,
    int goalX,
    int goalY)
{
    int height = map.size();
    int width = map[0].size();


    // --------------------------------------------------------
    // CREATE GRID
    // --------------------------------------------------------
    //
    // Create one Node for every location in the map.
    //
    std::vector<std::vector<Node>> grid(
        height,
        std::vector<Node>(width)
    );


    // Initialize every node.
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            grid[y][x].x = x;
            grid[y][x].y = y;

            // We don't initially know the cost to reach
            // this node.
            grid[y][x].gCost =
                std::numeric_limits<double>::infinity();

            grid[y][x].hCost = 0;

            // No previous node yet.
            grid[y][x].parent = nullptr;
        }
    }


    // Get pointers to the START and GOAL nodes.
    //
    // These pointers are safe to use while AStar() is running
    // because grid still exists.
    Node* start = &grid[startY][startX];
    Node* goal = &grid[goalY][goalX];


    // --------------------------------------------------------
    // INITIALIZE START
    // --------------------------------------------------------

    // Cost from START to START is zero.
    start->gCost = 0;

    // Estimate distance from START to GOAL.
    start->hCost = heuristic(start, goal);


    // --------------------------------------------------------
    // OPEN LIST
    // --------------------------------------------------------
    //
    // The priority queue contains nodes that we have
    // discovered but have not finished processing.
    //
    // The node with the lowest fCost() comes out first.
    //
    std::priority_queue<
        Node*,
        std::vector<Node*>,
        CompareNode
    > openList;


    // --------------------------------------------------------
    // CLOSED LIST
    // --------------------------------------------------------
    //
    // Contains nodes whose best path has already been
    // processed.
    //
    std::vector<Node*> closedList;


    // Start the search with START.
    openList.push(start);


    // --------------------------------------------------------
    // MAIN A* LOOP
    // --------------------------------------------------------

    while (!openList.empty())
    {
        // Get the node with the lowest fCost.
        Node* current = openList.top();

        // Remove it from OPEN.
        openList.pop();


        // ----------------------------------------------------
        // CHECK IF ALREADY PROCESSED
        // ----------------------------------------------------
        //
        // The same Node pointer can appear in the priority
        // queue more than once.
        //
        // If we've already processed this Node, ignore it.
        //
        if (std::find(
                closedList.begin(),
                closedList.end(),
                current)
            != closedList.end())
        {
            continue;
        }


        // ----------------------------------------------------
        // MOVE CURRENT TO CLOSED
        // ----------------------------------------------------
        //
        // We've now selected this node for processing.
        //
        closedList.push_back(current);


        // ----------------------------------------------------
        // CHECK FOR GOAL
        // ----------------------------------------------------

        if (current == goal)
        {
            // This time we store POSITION VALUES rather than
            // Node pointers.
            //
            // This prevents us from returning pointers to
            // Nodes that will be destroyed when AStar()
            // finishes.
            std::vector<Position> path;


            // Follow the parent pointers backward:
            //
            // GOAL
            //   ↓
            // Node
            //   ↓
            // Node
            //   ↓
            // START
            //
            while (current != nullptr)
            {
                path.push_back({
                    current->x,
                    current->y
                });

                current = current->parent;
            }


            // We built the path backwards, so reverse it.
            //
            // GOAL → ... → START
            //
            // becomes:
            //
            // START → ... → GOAL
            //
            std::reverse(
                path.begin(),
                path.end()
            );


            return path;
        }


        // ----------------------------------------------------
        // CHECK NEIGHBORS
        // ----------------------------------------------------

        for (Node* neighbor :
             getNeighbors(current, grid))
        {
            // Ignore obstacles.
            //
            // 1 = obstacle
            // 0 = free
            //
            if (map[neighbor->y][neighbor->x] == 1)
            {
                continue;
            }


            // Ignore nodes that have already been processed.
            //
            if (std::find(
                    closedList.begin(),
                    closedList.end(),
                    neighbor)
                != closedList.end())
            {
                continue;
            }


            // ------------------------------------------------
            // CALCULATE NEW G COST
            // ------------------------------------------------
            //
            // Moving one grid cell costs 1.
            //
            double newGCost =
                current->gCost + 1;


            // ------------------------------------------------
            // CHECK FOR BETTER PATH
            // ------------------------------------------------
            //
            // If this is a cheaper route to the neighbor,
            // update the neighbor.
            //
            if (newGCost < neighbor->gCost)
            {
                // Update actual cost from START.
                neighbor->gCost = newGCost;


                // Estimate remaining cost to GOAL.
                neighbor->hCost =
                    heuristic(neighbor, goal);


                // Remember how we reached this node.
                neighbor->parent = current;


                // Add the node to the priority queue.
                //
                // If this node was already there, we simply
                // add it again with its improved cost.
                openList.push(neighbor);
            }
        }
    }


    // --------------------------------------------------------
    // NO PATH FOUND
    // --------------------------------------------------------

    return {};
}


// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------

int main()
{
    // --------------------------------------------------------
    // MAP
    // --------------------------------------------------------
    //
    // 0 = free space
    // 1 = obstacle
    //
    std::vector<std::vector<int>> map =
    {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0}
    };


    // --------------------------------------------------------
    // START AND GOAL
    // --------------------------------------------------------

    int startX = 0;
    int startY = 0;

    int goalX = 9;
    int goalY = 9;


    // --------------------------------------------------------
    // RUN A*
    // --------------------------------------------------------

    std::vector<Position> path =
        AStar(
            map,
            startX,
            startY,
            goalX,
            goalY
        );


    // --------------------------------------------------------
    // PRINT RESULT
    // --------------------------------------------------------

    if (path.empty())
    {
        std::cout << "No path found!\n";
        return 0;
    }


    std::cout << "Path:\n";

    // Print each position in the path.
    for (const Position& position : path)
    {
        std::cout
            << "("
            << position.x
            << ", "
            << position.y
            << ")\n";
    }


    return 0;
}