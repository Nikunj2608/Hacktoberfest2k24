#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    int gCost, hCost;
    Node* parent;
    Node(int x, int y) : x(x), y(y), gCost(0), hCost(0), parent(nullptr) {}
    
    int fCost() const { return gCost + hCost; }
    bool operator<(const Node& other) const { return fCost() > other.fCost(); }
    bool operator==(const Node& other) const { return x == other.x && y == other.y; }
};

class JumpPointSearch {
private:
    std::vector<std::vector<int>> grid;
    std::vector<std::vector<bool>> visited;
    int rows, cols;
    Node* start;
    Node* goal;

    bool isWalkable(int x, int y) const {
        return x >= 0 && y >= 0 && x < rows && y < cols && grid[x][y] == 0;
    }

    int calculateHeuristic(Node* a, Node* b) {
        return abs(a->x - b->x) + abs(a->y - b->y); // Manhattan distance
    }

    Node* jump(int x, int y, int dx, int dy) {
        if (!isWalkable(x, y)) return nullptr;
        if (x == goal->x && y == goal->y) return new Node(x, y);

        if ((dx != 0 && (isWalkable(x - dx, y + dy) && !isWalkable(x - dx, y))) ||
            (dy != 0 && (isWalkable(x + dx, y - dy) && !isWalkable(x, y - dy)))) {
            return new Node(x, y);
        }

        if (dx != 0 && dy != 0) {
            if (jump(x + dx, y, dx, 0) || jump(x, y + dy, 0, dy)) {
                return new Node(x, y);
            }
        }

        return jump(x + dx, y + dy, dx, dy);
    }

    std::vector<Node*> getSuccessors(Node* current) {
        std::vector<Node*> successors;
        int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

        for (int i = 0; i < 8; ++i) {
            Node* jumpNode = jump(current->x + dx[i], current->y + dy[i], dx[i], dy[i]);
            if (jumpNode != nullptr && !visited[jumpNode->x][jumpNode->y]) {
                jumpNode->parent = current;
                successors.push_back(jumpNode);
            }
        }

        return successors;
    }

    std::vector<Node> reconstructPath(Node* node) {
        std::vector<Node> path;
        while (node != nullptr) {
            path.push_back(*node);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    JumpPointSearch(const std::vector<std::vector<int>>& grid, Node* start, Node* goal)
        : grid(grid), start(start), goal(goal) {
        rows = grid.size();
        cols = grid[0].size();
        visited.resize(rows, std::vector<bool>(cols, false));
    }

    std::vector<Node> findPath() {
        std::priority_queue<Node> openSet;
        start->hCost = calculateHeuristic(start, goal);
        openSet.push(*start);

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            if (current == *goal) {
                return reconstructPath(&current);
            }

            visited[current.x][current.y] = true;

            std::vector<Node*> successors = getSuccessors(&current);
            for (Node* successor : successors) {
                successor->gCost = current.gCost + 1;
                successor->hCost = calculateHeuristic(successor, goal);
                openSet.push(*successor);
            }
        }

        return {}; // No path found
    }
};

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };

    Node start(0, 0);
    Node goal(4, 4);
    JumpPointSearch jps(grid, &start, &goal);
    
    std::vector<Node> path = jps.findPath();
    
    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& node : path) {
            std::cout << "(" << node.x << ", " << node.y << ")\n";
        }
    } else {
        std::cout << "No path found.\n";
    }

    return 0;
}
