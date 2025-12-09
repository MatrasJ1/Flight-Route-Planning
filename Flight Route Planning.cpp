// Flight Route Planning.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <limits>
#include <algorithm>
using namespace std;

const double INF = numeric_limits<double>::infinity(); // Represents no connection

vector<string> airports;
vector<vector<double>> matrix;

// Add airport, return its index
int addAirport(const string& code) {
    for (int i = 0; i < airports.size(); i++) {
        if (airports[i] == code) {
            return i;
        }
    }
    airports.push_back(code);
    int n = airports.size();

    // Resize existing rows to accommodate new column
    for (int i = 0; i < n - 1; i++) {
        matrix[i].resize(n, INF);
    }
    // Add new row for the new airport
    vector<double> newRow(n, INF);
    newRow[n - 1] = 0; // distance to itself
    matrix.push_back(newRow);

    return n - 1;
}

// Add a flight
void addFlight(const string& start, const string& dest, int distance) {
    int i = addAirport(start);
    int j = addAirport(dest);
    matrix[i][j] = distance;
}

// Print adjacency matrix
void printMatrix() {
    int n = airports.size();
    cout << setw(6) << "";
    for (int j = 0; j < n; j++) {
        cout << setw(6) << airports[j];
    }
    cout << endl;

    for (int i = 0; i < n; i++) {
        cout << setw(6) << airports[i];
        for (int j = 0; j < n; j++) {
            if (matrix[i][j] == INF) {
                cout << setw(6) << "-";
            }
            else {
                cout << setw(6) << matrix[i][j];
            }
        }
        cout << endl;
    }
}

// Dijkstra shortest path
void shortestPath(const string& src, const string& dst) {
    int n = airports.size();
    int start = -1, end = -1;
    //Assign index for starting airport and ending airport
    for (int i = 0; i < n; i++) {
        if (airports[i] == src) {
            start = i;
        }
        if (airports[i] == dst){ 
            end = i;
        }
    }
    if (start == -1 || end == -1) {
        cout << "Airport not found.\n";
        return;
    }

    vector<double> dist(n, INF), prev(n, -1);
    vector<bool> used(n, false);
    dist[start] = 0;

    // Find airport with smallest distance from previous
    for (int k = 0; k < n; k++) {
        int u = -1;
        for (int i = 0; i < n; i++) {
            if (!used[i] && (u == -1 || dist[i] < dist[u])) {
                u = i;
            }
        }
        //if smallest distance is infinity, all airports are unreachable
        if (dist[u] == INF) {
            break;
        }
        used[u] = true;

        for (int v = 0; v < n; v++) {
            if (matrix[u][v] < INF) {
                //checks if new path from u is shorter than known distance to  v
                if (dist[v] > dist[u] + matrix[u][v]) {
                    dist[v] = dist[u] + matrix[u][v];
                    prev[v] = u; //record u as previous airport
                }
            }
        }
    }

    if (dist[end] == INF) {
        cout << "No path found from " << src << " to " << dst << ".\n";
        return;
    }

    //traverse path taken
    vector<int> path;
    for (int at = end; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    cout << "Shortest path from " << src << " to " << dst << " (" << dist[end] << " miles):\n";
    for (int i = 0; i < path.size(); i++) {
        cout << airports[path[i]];
        if (i + 1 < path.size()) {
            cout << " -> ";
        }
    }
    cout << endl;
}

// Find hubs
void findHubs() {
    int n = airports.size();
    vector<int> outDegree(n, 0), inDegree(n, 0);

    //calculate in-degree and out-degree of each airport
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            if (matrix[i][j] < INF && i != j) {
                outDegree[i]++;
                inDegree[j]++;
            }

    //calculate max in-degree and max out-degree
    int maxOut = *max_element(outDegree.begin(), outDegree.end());
    int maxIn = *max_element(inDegree.begin(), inDegree.end());

    cout << "Hubs by outgoing flights:\n";
    for (int i = 0; i < n; i++) {
        if (outDegree[i] == maxOut) {
            cout << airports[i] << " (" << outDegree[i] << " flights)\n";
        }
    }
    cout << "Hubs by incoming flights:\n";
    for (int i = 0; i < n; i++) {
        if (inDegree[i] == maxIn) {
            cout << airports[i] << " (" << inDegree[i] << " flights)\n";
        }
    }
}

// Check unreachable airports
void unreachableFrom(const string& src) {
    int n = airports.size();
    int start = -1;
    //find index of starting airport
    for (int i = 0; i < n; i++) {
        if (airports[i] == src) {
            start = i;
        }
    }
    if (start == -1) {
        cout << "Airport not found.\n";
        return;
    }

    vector<bool> visited(n, false);
    //stack for depth-first-search algorithm
    vector<int> stack = { start };
    while (!stack.empty()) {
        int u = stack.back(); 
        stack.pop_back();
        //if already visited, continue
        if (visited[u]) {
            continue;
        }
        visited[u] = true;
        for (int v = 0; v < n; v++) {
            if (matrix[u][v] < INF && !visited[v]) {
                stack.push_back(v);
            }
        }
    }

    cout << "Airports unreachable from " << src << ":\n";
    bool any = false;
    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            cout << airports[i] << endl;
            any = true;
        }
    }
    if (!any) {
        cout << "None.\n";
    }
}

// Menu
void menu() {
    int choice = -1;
    string src, dst;
    int distance;
    while(choice != 0){
        cout << "\n--- Flight Network Menu ---\n";
        cout << "1. Add flight\n";
        cout << "2. Print adjacency matrix\n";
        cout << "3. Shortest path\n";
        cout << "4. Find hubs\n";
        cout << "5. Check unreachable airports\n";
        cout << "0. Exit\n";
        cout << "Enter choice: "; 
        cin >> choice;

        switch (choice) {
        case 1:
            cout << "Origin: "; 
            cin >> src;
            cout << "Destination: "; 
            cin >> dst;
            cout << "Distance: "; 
            cin >> distance;
            addFlight(src, dst, distance);
            break;
        case 2:
            printMatrix();
            break;
        case 3:
            cout << "From: ";
            cin >> src;
            for (int i = 0; i < airports.size(); i++) {
                if (src == airports.at(i)) {
                    continue;
                }
                shortestPath(src, airports.at(i));

            }
            break;
        case 4:
            findHubs();
            break;
        case 5:
            cout << "From: ";
            cin >> src;
            unreachableFrom(src);
            break;
        }
    }
}

// Load dataset
void loadDataset() {
    addFlight("MCI", "DEN", 532);
    addFlight("DEN", "PHX", 602);
    addFlight("DEN", "LAX", 862);
    addFlight("DEN", "SLC", 391);
    addFlight("DEN", "LAS", 628);
    addFlight("DEN", "MSP", 680);
    addFlight("DEN", "MCI", 532);

    addFlight("MCI", "ORD", 403);
    addFlight("ORD", "LGA", 733);
    addFlight("ORD", "DFW", 802);
    addFlight("ORD", "BOS", 867);
    addFlight("ORD", "LAS", 1514);
    addFlight("ORD", "MSP", 334);
    addFlight("ORD", "MCI", 403);

    addFlight("MCI", "ATL", 692);
    addFlight("ATL", "LGA", 762);
    addFlight("ATL", "MCO", 404);
    addFlight("ATL", "FLL", 581);
    addFlight("ATL", "DFW", 731);
    addFlight("ATL", "DCA", 547);

    addFlight("MCI", "PHX", 1042);
    addFlight("PHX", "DEN", 602);
    addFlight("PHX", "LAX", 370);
    addFlight("PHX", "SEA", 1107);
    addFlight("PHX", "SAN", 304);
    addFlight("PHX", "SLC", 507);

    addFlight("MCI", "DFW", 461);
    addFlight("DFW", "ORD", 802);
    addFlight("DFW", "ATL", 731);
    addFlight("DFW", "LAX", 1235);
    addFlight("DFW", "LGA", 1389);
    addFlight("DFW", "IAH", 224);

    addFlight("LGA", "MCI", 1107);

    addFlight("LAX", "LAS", 236);

    addFlight("IAH", "ATL", 689);

    addFlight("MCO", "ORD", 1005);

    addFlight("DFW", "DEN", 641);

}

int main() {
    loadDataset();
    menu();
    return 0;
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
