#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <map>
#include <unordered_set>

using namespace std;

struct Node {
    double lon, lat;
    vector<pair<Node*, double>> edges;
};

struct Graph {
    vector<unique_ptr<Node>> nodes;
    unordered_map<string, Node*> node_map;

    Node* findClosestNode(double lat, double lon) {
        double minDistance = numeric_limits<double>::max();
        Node* closestNode = nullptr;
        for (const auto& node : nodes) {
            double distance = sqrt(pow(node->lat - lat, 2) + pow(node->lon - lon, 2));
            if (distance < minDistance) {
                minDistance = distance;
                closestNode = node.get();
            }
        }
        return closestNode;
    }

    Node* addNode(double lon, double lat) {
        auto vertex = make_unique<Node>(Node{lon, lat});
        Node* vertex_ptr = vertex.get();
        nodes.push_back(move(vertex));
        ostringstream key_stream;
        key_stream << fixed << setprecision(10) << lon << "," << lat;
        string identifier = key_stream.str();
        node_map[identifier] = vertex_ptr;
        return vertex_ptr;
    }

    Node* getNode(double lon, double lat) {
        ostringstream key_stream;
        key_stream << fixed << setprecision(10) << lon << "," << lat;
        string identifier = key_stream.str();
        if (node_map.find(identifier) != node_map.end()) {
            return node_map[identifier];
        }
        return nullptr;
    }

    void parseFile(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            return;
        }
        string line;
        while (getline(file, line)) {
            processLine(line);
        }
    }

    void processLine(const string& line) {
        istringstream lineStream(line);
        string parentData;
        if (!getline(lineStream, parentData, ':')) {
            return;
        }
        auto [lon1, lat1] = extractCoordinates(parentData);
        Node* parentNode = getNode(lon1, lat1);
        if (!parentNode) {
            parentNode = addNode(lon1, lat1);
        }
        string edgesData;
        while (getline(lineStream, edgesData, ';')) {
            processEdgeData(edgesData, parentNode);
        }
    }

    pair<double, double> extractCoordinates(const string& data) {
        string cleanedData = data;
        replace(cleanedData.begin(), cleanedData.end(), ',', ' ');
        double lon = 0.0, lat = 0.0;
        istringstream coordStream(cleanedData);
        if (!(coordStream >> lon >> lat)) {
            return {0.0, 0.0};
        }
        return {lon, lat};
    }

    void processEdgeData(const string& edgeData, Node* parentNode) {
        double lon2, lat2, weight;
        string cleanedEdgeData = edgeData;
        replace(cleanedEdgeData.begin(), cleanedEdgeData.end(), ',', ' ');
        istringstream edgeStream(cleanedEdgeData);
        if (!(edgeStream >> lon2 >> lat2 >> weight)) {
            return;
        }
        Node* childNode = getNode(lon2, lat2);
        if (!childNode) {
            childNode = addNode(lon2, lat2);
        }
        parentNode->edges.emplace_back(childNode, weight);
        childNode->edges.emplace_back(parentNode, weight);
    }
};

vector<Node*> bfs(Node* source, Node* destination) {
    if (!source || !destination) return {};
    queue<Node*> q;
    unordered_map<Node*, Node*> predecessors;
    unordered_map<Node*, bool> visited;
    q.push(source);
    visited[source] = true;
    while (!q.empty()) {
        Node* current_vertex = q.front();
        q.pop();
        if (current_vertex == destination) {
            vector<Node*> route;
            for (Node* at = destination; at != nullptr; at = predecessors[at]) {
                route.push_back(at);
            }
            reverse(route.begin(), route.end());
            return route;
        }
        for (const auto& connection : current_vertex->edges) {
            Node* adjacent = connection.first;
            if (!visited[adjacent]) {
                q.push(adjacent);
                visited[adjacent] = true;
                predecessors[adjacent] = current_vertex;
            }
        }
    }
    return {};
}

vector<Node*> dfs(Node* source, Node* destination) {
    if (!source || !destination) return {};
    unordered_map<Node*, bool> visited;
    unordered_map<Node*, Node*> predecessors;
    stack<Node*> s;
    s.push(source);
    visited[source] = true;
    while (!s.empty()) {
        Node* current_vertex = s.top();
        s.pop();
        if (current_vertex == destination) {
            vector<Node*> route;
            for (Node* at = destination; at != nullptr; at = predecessors[at]) {
                route.push_back(at);
            }
            reverse(route.begin(), route.end());
            return route;
        }
        for (const auto& connection : current_vertex->edges) {
            Node* adjacent = connection.first;
            if (!visited[adjacent]) {
                s.push(adjacent);
                visited[adjacent] = true;
                predecessors[adjacent] = current_vertex;
            }
        }
    }
    return {};
}

vector<Node*> dijkstra(Graph& graph, Node* source, Node* destination) {
    unordered_map<Node*, Node*> predecessors;
    if (!source || !destination) return {};
    unordered_map<Node*, double> distances;
    for (const auto& vertex : graph.nodes)
        distances[vertex.get()] = numeric_limits<double>::infinity();
    distances[source] = 0.0;
    priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, greater<>> priority_q;
    priority_q.emplace(0.0, source);
    while (!priority_q.empty()) {
        auto [current_distance, current_vertex] = priority_q.top();
        priority_q.pop();
        if (current_vertex == destination) {
            vector<Node*> route;
            for (Node* at = destination; at != nullptr; at = predecessors[at]) {
                route.push_back(at);
            }
            reverse(route.begin(), route.end());
            return route;
        }
        for (const auto& connection : current_vertex->edges) {
            Node* adjacent = connection.first;
            double edge_weight = connection.second;
            double new_distance = current_distance + edge_weight;
            if (new_distance < distances[adjacent]) {
                distances[adjacent] = new_distance;
                predecessors[adjacent] = current_vertex;
                priority_q.emplace(new_distance, adjacent);
            }
        }
    }
    return {};
}

static double heuristic(Node* a, Node* b) {
    return sqrt(pow(a->lat - b->lat, 2) + pow(a->lon - b->lon, 2));
}

vector<Node *> aStar(Graph& graph, Node* closest_start, Node* closest_end) {
    if (!closest_start || !closest_end) {
        return {};
    }
    map<Node*, double> g_score;
    map<Node*, double> f_score;
    priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, greater<>> open_set;
    g_score[closest_start] = 0;
    f_score[closest_start] = heuristic(closest_start, closest_end);
    open_set.push({f_score[closest_start], closest_start});
    map<Node*, Node*> came_from;
    while (!open_set.empty()) {
        auto current = open_set.top().second;
        open_set.pop();
        if (current == closest_end) {
            vector<Node *> path;
            for (Node* at = closest_end; at != nullptr; at = came_from[at]) {
                path.emplace_back(at);
            }
            reverse(path.begin(), path.end());
            return path;
        }
        for (const auto& neighbor : current->edges) {
            Node* neighbor_node = neighbor.first;
            double tentative_g_score = g_score[current] + neighbor.second;
            if (g_score.find(neighbor_node) == g_score.end() || tentative_g_score < g_score[neighbor_node]) {
                came_from[neighbor_node] = current;
                g_score[neighbor_node] = tentative_g_score;
                f_score[neighbor_node] = g_score[neighbor_node] + heuristic(neighbor_node, closest_end);
                open_set.push({f_score[neighbor_node], neighbor_node});
            }
        }
    }
    return {};
}

void runTests() {
    // Test for BFS
    {
        Graph graph;
        Node* a = graph.addNode(0, 0);
        Node* b = graph.addNode(1, 0);
        Node* c = graph.addNode(1, 1);
        Node* d = graph.addNode(0, 1);
        a->edges.emplace_back(b, 1);
        a->edges.emplace_back(d, 1);
        b->edges.emplace_back(c, 1);
        d->edges.emplace_back(c, 1);
        vector<Node*> path = bfs(a, c);
        assert(path.size() == 3 && path[0] == a && path[1] == b && path[2] == c);
    }

    // Test for DFS
    {
        Graph graph;
        Node* a = graph.addNode(0, 0);
        Node* b = graph.addNode(1, 0);
        Node* c = graph.addNode(1, 1);
        Node* d = graph.addNode(0, 1);
        a->edges.emplace_back(b, 1);
        a->edges.emplace_back(d, 1);
        b->edges.emplace_back(c, 1);
        vector<Node*> path = dfs(a, c);
        assert(path.size() == 3 && path[0] == a && path[1] == b && path[2] == c);
    }

    // Test for Dijkstra's algorithm
    {
        Graph graph;
        Node* a = graph.addNode(0, 0);
        Node* b = graph.addNode(2, 0);
        Node* c = graph.addNode(2, 2);
        a->edges.emplace_back(b, 4); // a - b (weight 4)
        a->edges.emplace_back(c, 2); // a - c (weight 2)
        b->edges.emplace_back(c, 1); // b - c (weight 1)
        vector<Node*> path = dijkstra(graph, a, c);
        assert(path.size() == 2 && path[0] == a && path[1] == c);
    }

    // Test for A*
    {
        Graph graph;
        Node* a = graph.addNode(0, 0);
        Node* b = graph.addNode(2, 0);
        Node* c = graph.addNode(2, 2);
        a->edges.emplace_back(b, 4); // a - b (weight 4)
        a->edges.emplace_back(c, 2); // a - c (weight 2)
        vector<Node*> path = aStar(graph, a, c);
        assert(path.size() == 2 && path[0] == a && path[1] == c);
    }

    cout << "All tests passed!" << endl;
}


int main() {
    runTests();


   Graph graph;
   graph.parseFile("C:/Users/b0605/Downloads/spb_graph (2).txt");

   constexpr double start_lat=59.848294; constexpr double start_lon=30.329455;
   constexpr double goal_lat=59.957238; constexpr double goal_lon=30.308108;

   Node *start_node=graph.findClosestNode(start_lat,start_lon);
   Node *goal_node=graph.findClosestNode(goal_lat ,goal_lon);

   // BFS
    cout << "BFS:" << endl;
    auto start_time = chrono::high_resolution_clock::now();
    const vector<Node *> bfs_path = bfs(start_node, goal_node);
    auto end_time = chrono::high_resolution_clock::now();
    double total_length = 0;
    for (size_t i = 0; i < bfs_path.size(); ++i) {
        //cout << "(" << bfs_path[i]->lat << ", " << bfs_path[i]->lon << ")";

        if (i < bfs_path.size() - 1) {
            //cout << " - ";
            for (const auto& edge : bfs_path[i]->edges) {
                if (edge.first == bfs_path[i + 1]) {
                    total_length += edge.second;
                    break;
                }
            }
        }
    }
    cout << endl;
    cout << "Total length: " << total_length << endl;
    cout << "Size: " << bfs_path.size() << endl;
    auto bfs_duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    cout << "Time of BFS: " << bfs_duration.count() << " ms" << endl;

    // Dijkstra
    cout << endl;
    cout << "Dijkstra:" << endl;
    start_time = chrono::high_resolution_clock::now();
    const vector<Node *> dijkstra_path = dijkstra(graph, start_node, goal_node);
    end_time = chrono::high_resolution_clock::now();
    total_length = 0;
    for (size_t i = 0; i < dijkstra_path.size(); ++i) {
        //cout << "(" << dijkstra_path[i]->lat << ", " << dijkstra_path[i]->lon << ")";

        if (i < dijkstra_path.size() - 1) {
            //cout << " - ";
            for (const auto& edge : dijkstra_path[i]->edges) {
                if (edge.first == dijkstra_path[i + 1]) {
                    total_length += edge.second;
                    break;
                }
            }
        }
    }
    cout << endl;
    cout << "Total length: " << total_length << endl;
    cout << "Size: " << dijkstra_path.size() << endl;
    auto dijkstra_duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    cout << "Time of Dijkstra: " << dijkstra_duration.count() << " ms" << std::endl;

    // DFS
    cout << endl;
    cout << "DFS:" << endl;
    start_time = chrono::high_resolution_clock::now();
    const vector<Node *> dfs_path = dfs(start_node, goal_node);
    end_time = chrono::high_resolution_clock::now();
    total_length = 0;
    for (size_t i = 0; i < dfs_path.size(); ++i) {
        //cout << "(" << dfs_path[i]->lat << ", " << dfs_path[i]->lon << ")";

        if (i < dfs_path.size() - 1) {
            //cout << " - ";
            for (const auto& edge : dfs_path[i]->edges) {
                if (edge.first == dfs_path[i + 1]) {
                    total_length += edge.second;
                    break;
                }
            }
        }
    }
    cout << endl;
    cout << "Total length: " << total_length << endl;
    cout << "Size: " << dfs_path.size() << endl;
    auto dfs_duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    cout << "Time of DFS: " << dfs_duration.count() << " ms" << endl;

    // A*
    cout << endl;
    cout << "A*:" << endl;
    start_time = chrono::high_resolution_clock::now();
    const vector<Node *> a_star_path = aStar(graph, start_node, goal_node);
    end_time = chrono::high_resolution_clock::now();
    total_length = 0;
    for (size_t i = 0; i < a_star_path.size(); ++i) {
        //cout << "(" << a_star_path[i]->lat << ", " << a_star_path[i]->lon << ")";

        if (i < a_star_path.size() - 1) {
            //cout << " - ";
            for (const auto& edge : a_star_path[i]->edges) {
                if (edge.first == a_star_path[i + 1]) {
                    total_length += edge.second;
                    break;
                }
            }
        }
    }
    cout << endl;
    cout << "Total length: " << total_length << endl;
    cout << "Size: " << a_star_path.size() << endl;
    auto a_star_duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    cout << "Time of A*: " << a_star_duration.count() << " ms" << endl;
}