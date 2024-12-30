#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <string>
#include <set>
#include <algorithm>
#include <chrono>
using namespace std;


struct Node {
    double lon, lat;
    vector<pair<Node*, double>> edges;
};


struct Graph {
    vector<Node*> nodes;
    unordered_map<string, Node*> node_map;

    Node* find_closest_node(double lat, double lon) {
        double min_distance = numeric_limits<double>::max();
        Node* node_founded = nullptr;

        // Перебор всех узлов графа
        for (auto node : nodes) {
            double distance = sqrt(pow(node->lat - lat, 2) + pow(node->lon - lon, 2));
            if (distance < min_distance) {
                node_founded = node;
                min_distance = distance;
            }
        }

        return node_founded;
    }

    vector<string> split(string s, char del) {
        stringstream ss(s);
        string word;
        vector<string> tokens;

        while (getline(ss, word, del)) {
            tokens.push_back(word);
        }

        return tokens;
    }

    Node* get_or_create_node(double lat, double lon) {
        string key = to_string(lat) + "," + to_string(lon);
        if (node_map.find(key) == node_map.end()) {
            Node* new_node = new Node{lon, lat, {}};
            node_map[key] = new_node;
            nodes.push_back(new_node);
        }
        return node_map[key];
    }

    void processing_line(string line) {
        vector<string> parts = split(line, ':');
        vector<string> coords = split(parts[0], ',');

        double lat = stod(coords[0]);
        double lon = stod(coords[1]);

        Node* parent_node = get_or_create_node(lat, lon);

        vector<string> edges = split(parts[1], ';');
        for (string edge : edges) {
            vector<string> edge_parts = split(edge, ',');

            double subsidiary_lat = stod(edge_parts[0]);
            double subsidiary_lon = stod(edge_parts[1]);
            double weight = stod(edge_parts[2]);

            Node* subsidiary_node = get_or_create_node(subsidiary_lat, subsidiary_lon);

            parent_node->edges.push_back(make_pair(subsidiary_node, weight));
            subsidiary_node->edges.push_back(make_pair(parent_node, weight));
        }
    }

    void read_graph(string filename) {
        ifstream file(filename);

        string line;
        while (getline(file, line)) {
            processing_line(line);
        }
    }

    vector<Node*> dfs(double start_lat, double start_lon, double target_lat, double target_lon) {
        Node* start_node = find_closest_node(start_lat, start_lon);
        Node* target_node = find_closest_node(target_lat, target_lon);

        vector<Node*> best_path, path;
        set<Node*> visited;

        path.push_back(start_node);
        visited.insert(start_node);

        while (!path.empty()) {
            Node* current = path.back();

            if (current == target_node) {
                if (best_path.empty() || path.size() < best_path.size()) {
                    best_path = path;
                    if (path.size() == 1) {
                        return best_path;
                    }
                }
                path.pop_back();
                continue;
            }

            bool flag = true;
            for (auto& edge : current->edges) {
                Node* neighbor = edge.first;
                if (visited.find(neighbor) == visited.end()) {
                    path.push_back(neighbor);
                    visited.insert(neighbor);
                    flag = false;
                    break;
                }
            }

            if (flag) {
                path.pop_back();
            }
        }

        return best_path;
    }

    vector<Node*> bfs(double start_lat, double start_lon, double target_lat, double target_lon) {
        Node* start_node = find_closest_node(start_lat, start_lon);
        Node* target_node = find_closest_node(target_lat, target_lon);

        vector<Node*> stack;
        set<Node*> visited;
        unordered_map<Node*, Node*> parent;

        stack.push_back(start_node);
        visited.insert(start_node);

        while (!stack.empty()) {
            Node* current = stack[0];
            stack.erase(stack.begin());

            if (current == target_node) {
                vector<Node*> path;
                while (current) {
                    path.push_back(current);
                    current = parent[current];
                }

                reverse(path.begin(), path.end());
                return path;
            }

            for (auto& edge : current->edges) {
                Node* neighbor = edge.first;
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    parent[neighbor] = current;
                    stack.push_back(neighbor);
                }
            }
        }

        return {};
    }

    vector<Node*> dijkstra(double start_lat, double start_lon, double target_lat, double target_lon) {
        Node* start_node = find_closest_node(start_lat, start_lon);
        Node* target_node = find_closest_node(target_lat, target_lon);

        unordered_map<Node*, double> distances;
        unordered_map<Node*, Node*> parents;
        set<pair<double, Node*>> stack;

        for (Node* node : nodes) {
            distances[node] = numeric_limits<double>::max();
        }
        distances[start_node] = 0.0;
        stack.insert({0.0, start_node});

        while (!stack.empty()) {
            Node* current = stack.begin()->second;
            stack.erase(stack.begin());

            if (current == target_node) {
                break;
            }

            for (auto& edge : current->edges) {
                Node* neighbor = edge.first;
                double weight = edge.second;

                double new_distance = distances[current] + weight;
                if (new_distance < distances[neighbor]) {
                    stack.erase({distances[neighbor], neighbor});
                    distances[neighbor] = new_distance;
                    parents[neighbor] = current;
                    stack.insert({new_distance, neighbor});
                }
            }
        }

        vector<Node*> path;
        Node* current = target_node;
        while (current) {
            path.push_back(current);
            current = parents[current];
        }

        reverse(path.begin(), path.end());
        return path;
    }

    double metric(Node* a, Node* b) {
        double dx = a->lon - b->lon;
        double dy = a->lat - b->lat;
        return sqrt(dx * dx + dy * dy);
    }

    vector<Node*> Astar(double start_lat, double start_lon, double target_lat, double target_lon) {
        Node* start_node = find_closest_node(start_lat, start_lon);
        Node* target_node = find_closest_node(target_lat, target_lon);

        unordered_map<Node*, double> f;
        unordered_map<Node*, double> h;
        unordered_map<Node*, Node*> parents;
        set<pair<double, Node*>> stack;

        for (Node* node : nodes) {
            f[node] = numeric_limits<double>::infinity();
            h[node] = numeric_limits<double>::infinity();
        }
        f[start_node] = 0.0;
        h[start_node] = metric(start_node, target_node);
        stack.insert({h[start_node], start_node});

        while (!stack.empty()) {
            Node* current = stack.begin()->second;
            stack.erase(stack.begin());

            if (current == target_node) {
                vector<Node*> path;
                while (current) {
                    path.push_back(current);
                    current = parents[current];
                }
                reverse(path.begin(), path.end());
                return path;
            }

            for (pair<Node*, double> edge : current->edges) {
                Node* neighbor = edge.first;
                double weight = edge.second;

                double tentative_f = f[current] + weight;
                if (tentative_f < f[neighbor]) {
                    parents[neighbor] = current;
                    f[neighbor] = tentative_f;
                    h[neighbor] = f[neighbor] + metric(neighbor, target_node);

                    stack.insert({h[neighbor], neighbor});
                }
            }
        }

        return {};
    }
};


// int main() {
//     Graph graph;
//     string filename = "spb_graph.txt";
//     vector<Node*> path;

//     graph.read_graph(filename);

//     auto start = chrono::high_resolution_clock::now();
//     path = graph.dfs(30.3585261, 59.8864419, 30.3027079, 59.9570161);
//     cout << "Path size: "<< path.size() << endl;
//     auto end = chrono::high_resolution_clock::now();
//     chrono::duration<double> delta = end - start;
//     cout << "DFS: " << delta.count() << " s"<< endl;;

//     if (!path.empty()) {
//         cout << "Shorted path:" << endl;
//         for (const auto& node : path) {
//             cout << "(" << node->lat << ", " << node->lon << ") => ";
//         }
//         cout << "\n";
//     } else {
//         cout << "Path didn't found" << endl;
//     }

//     start = chrono::high_resolution_clock::now();
//     path = graph.bfs(30.3585261, 59.8864419, 30.3027079, 59.9570161);
//     cout << "Path size: "<< path.size() << endl;
//     end = chrono::high_resolution_clock::now();
//     delta = end - start;
//     cout << "BFS: " << delta.count() << " s" << endl;

//     if (!path.empty()) {
//         cout << "Shorted path:" << endl;
//         for (const auto& node : path) {
//             cout << "(" << node->lat << ", " << node->lon << ") => ";
//         }
//         cout << "\n";
//     } else {
//         cout << "Path didn't found" << endl;
//     }

//     start = chrono::high_resolution_clock::now();
//     path = graph.dijkstra(30.3585261, 59.8864419, 30.3027079, 59.9570161);
//     cout << "Path size: "<< path.size() << endl;
//     end = chrono::high_resolution_clock::now();
//     delta = end - start;
//     cout << "Dijkstra: " << delta.count() << " s" << endl;

//     if (!path.empty()) {
//         cout << "Shorted path:" << endl;
//         for (const auto& node : path) {
//             cout << "(" << node->lat << ", " << node->lon << ") => ";
//         }
//         cout << "\n";
//     } else {
//         cout << "Path didn't found" << endl;
//     }

//     start = chrono::high_resolution_clock::now();
//     path = graph.Astar(30.3585261, 59.8864419, 30.3027079, 59.9570161);
//     cout << "Path size: "<< path.size() << endl;
//     end = chrono::high_resolution_clock::now();
//     delta = end - start;
//     cout << "A*: " << delta.count() << " s"<< endl;;

//     if (!path.empty()) {
//         cout << "Shorted path:" << endl;
//         for (const auto& node : path) {
//             cout << "(" << node->lat << ", " << node->lon << ") => ";
//         }
//         cout << "\n";
//     } else {
//         cout << "Path didn't found" << endl;
//     }

//     return 0;
// }


void test_read_graph() {
    Graph graph;
    graph.read_graph("graph.txt");

    assert(graph.nodes.size() == 8);

    Node* node1 = graph.find_closest_node(59.8864419, 30.3585261);
    assert(node1->edges.size() == 2);

    Node* node2 = graph.find_closest_node(59.8844419, 30.3555261);
    assert(node2->edges.size() == 2);
}

void test_dfs() {
    Graph graph;
    graph.read_graph("graph.txt");

    auto path = graph.dfs(59.8864419, 30.3585261, 59.9570161, 30.3027079);
    assert(!path.empty());
    assert(path.front()->lat == 59.8864419);
    assert(path.back()->lat == 59.9570161);
}

void test_bfs() {
    Graph graph;
    graph.read_graph("graph.txt");

    auto path = graph.bfs(59.8864419, 30.3585261, 59.9570161, 30.3027079);
    assert(!path.empty());
    assert(path.front()->lat == 59.8864419);
    assert(path.back()->lat == 59.9570161);
    assert(path.size() <= 5);
}

void test_dijkstra() {
    Graph graph;
    graph.read_graph("spb_graph.txt");

    auto path = graph.dijkstra(59.8864419, 30.3585261, 59.9570161, 30.3027079);
    assert(!path.empty());
    assert(path.front()->lat == 59.8864419);
    assert(path.back()->lat == 59.9570161);
    assert(path.size() <= 5);
    cout << path.size() << endl;
}

void test_astar() {
    Graph graph;
    graph.read_graph("graph.txt");

    auto path = graph.Astar(59.8864419, 30.3585261, 59.9570161, 30.3027079);
    assert(!path.empty());
    assert(path.front()->lat == 59.8864419);
    assert(path.back()->lat == 59.9570161);


    assert(path.size() <= 5);
    
    
}


void run_tests() {
    test_read_graph();
    test_dfs();
    test_bfs();
    test_dijkstra();
    test_astar();

    cout << "All tests passed" << endl;
}

int main() {
    run_tests();
    return 0;
}