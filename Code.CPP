#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace std;

const double INF = numeric_limits<double>::infinity();

struct Edge {
    char from;
    char to;
    double weight;
};

struct Graph {
    string nodes;
    vector<Edge> edges;
    map<char, map<char, double>> neighbours;

    Graph(string nodeList, vector<Edge> edgeList)
        : nodes(nodeList), edges(edgeList) {

        for (char node : nodes) {
            neighbours[node] = {};
        }

        for (const Edge& edge : edges) {
            if (!neighbours.count(edge.from) ||
                !neighbours.count(edge.to)) {
                throw invalid_argument("Unknown edge endpoint");
            }

            if (!isfinite(edge.weight) || edge.weight < 0) {
                throw invalid_argument(
                    "Weights must be finite and non-negative"
                );
            }

            neighbours[edge.from][edge.to] = edge.weight;
            neighbours[edge.to][edge.from] = edge.weight;
        }
    }
};

Graph createGraph() {
    return Graph("ABCDEFGHIJKLMNOPQRST", {
        {'A', 'B', 4}, {'B', 'C', 6},
        {'C', 'D', 5}, {'D', 'E', 7},
        {'F', 'G', 3}, {'G', 'H', 5},
        {'H', 'I', 4}, {'I', 'J', 6},
        {'K', 'L', 6}, {'L', 'M', 3},
        {'M', 'N', 5}, {'N', 'O', 4},
        {'P', 'Q', 5}, {'Q', 'R', 4},
        {'R', 'S', 6}, {'S', 'T', 3},
        {'A', 'F', 7}, {'B', 'G', 4},
        {'C', 'H', 8}, {'D', 'I', 6},
        {'E', 'J', 5}, {'F', 'K', 5},
        {'G', 'L', 7}, {'H', 'M', 3},
        {'I', 'N', 8}, {'J', 'O', 4},
        {'K', 'P', 6}, {'L', 'Q', 5},
        {'M', 'R', 7}, {'N', 'S', 4},
        {'O', 'T', 6}
    });
}

struct Route {
    double time;
    vector<char> path;
};

Route dijkstra(const Graph& graph, char start, char destination) {
    if (!graph.neighbours.count(start) ||
        !graph.neighbours.count(destination)) {
        throw invalid_argument("Unknown start or destination");
    }

    map<char, double> distance;
    map<char, char> previous;
    map<char, bool> settled;

    for (char node : graph.nodes) {
        distance[node] = INF;
        settled[node] = false;
    }

    distance[start] = 0;

    // Equal distances are processed in alphabetical order.
    using Item = pair<double, char>;
    priority_queue<Item, vector<Item>, greater<Item>> pending;

    pending.push({0, start});

    while (!pending.empty()) {
        double currentDistance = pending.top().first;
        char current = pending.top().second;
        pending.pop();

        if (settled[current] ||
            currentDistance != distance[current]) {
            continue;
        }

        settled[current] = true;

        // Stop only after the destination is settled.
        if (current == destination) {
            break;
        }

        for (const auto& neighbour : graph.neighbours.at(current)) {
            char next = neighbour.first;
            double weight = neighbour.second;
            double candidate = currentDistance + weight;

            // Equal alternatives keep the existing predecessor.
            if (!settled[next] && candidate < distance[next]) {
                distance[next] = candidate;
                previous[next] = current;
                pending.push({candidate, next});
            }
        }
    }

    if (!isfinite(distance[destination])) {
        return {INF, {}};
    }

    vector<char> path;
    char current = destination;

    while (true) {
        path.push_back(current);

        if (current == start) {
            break;
        }

        current = previous.at(current);
    }

    reverse(path.begin(), path.end());

    return {distance[destination], path};
}

void displayRoute(const Route& route) {
    if (!isfinite(route.time)) {
        cout << "unreachable; path []\n";
        return;
    }

    // Matches the format in the test results.
    cout << route.time << "; path ";

    for (size_t i = 0; i < route.path.size(); ++i) {
        if (i > 0) {
            cout << " - ";
        }

        cout << route.path[i];
    }

    cout << '\n';
}

struct MinimumSpanningTree {
    vector<Edge> edges;
    double total = 0;
    bool connected = false;
};

MinimumSpanningTree kruskal(const Graph& graph) {
    map<char, char> parent;

    for (char node : graph.nodes) {
        parent[node] = node;
    }

    function<char(char)> findRoot = [&](char node) -> char {
        if (parent[node] != node) {
            parent[node] = findRoot(parent[node]);
        }

        return parent[node];
    };

    vector<Edge> sortedEdges = graph.edges;

    // Normalise endpoints so ties follow alphabetical order.
    for (Edge& edge : sortedEdges) {
        if (edge.from > edge.to) {
            swap(edge.from, edge.to);
        }
    }

    sort(sortedEdges.begin(), sortedEdges.end(),
        [](const Edge& first, const Edge& second) {
            return tie(first.weight, first.from, first.to) <
                   tie(second.weight, second.from, second.to);
        }
    );

    MinimumSpanningTree result;

    for (const Edge& edge : sortedEdges) {
        char firstRoot = findRoot(edge.from);
        char secondRoot = findRoot(edge.to);

        // Different roots mean adding this edge creates no cycle.
        if (firstRoot != secondRoot) {
            parent[firstRoot] = secondRoot;
            result.edges.push_back(edge);
            result.total += edge.weight;

            if (result.edges.size() == graph.nodes.size() - 1) {
                break;
            }
        }
    }

    result.connected =
        !graph.nodes.empty() &&
        result.edges.size() == graph.nodes.size() - 1;

    return result;
}

void displayMST(const MinimumSpanningTree& tree) {
    cout << "\nMinimum spanning tree\n";

    for (const Edge& edge : tree.edges) {
        cout << edge.from << " - " << edge.to
             << ": " << edge.weight << " minutes\n";
    }

    // Matches the MST summary in the test results.
    cout << "{\"edges\": " << tree.edges.size()
         << ", \"total\": " << tree.total
         << ", \"connected\": "
         << (tree.connected ? "true" : "false")
         << "}\n";
}

int main() {
    try {
        Graph graph = createGraph();

        string from;
        string to;

        cout << "Enter node from (A-T): ";
        if (!(cin >> from)) {
            return 1;
        }

        cout << "Enter node to (A-T): ";
        if (!(cin >> to)) {
            return 1;
        }

        // Calculate the MST for the entire network.
        MinimumSpanningTree tree = kruskal(graph);

        try {
            if (from.size() != 1 || to.size() != 1) {
                throw invalid_argument(
                    "Unknown start or destination"
                );
            }

            Route route = dijkstra(graph, from[0], to[0]);
            displayRoute(route);
        }
        catch (const invalid_argument& error) {
            cout << error.what() << '\n';
            displayMST(tree);
            return 1;
        }

        displayMST(tree);
    }
    catch (const exception& error) {
        cout << error.what() << '\n';
        return 1;
    }

    return 0;
}
