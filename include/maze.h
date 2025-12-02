//This to store and define the maze
#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include <math.h>



const int SIZE = 9;

struct Node {
    bool north;
    bool east;
    bool south;
    bool west;
    bool visited;
};

class Maze {
private:
    Node nodes[SIZE][SIZE];

public:
    Maze() {
        // Initialize all nodes to no paths and not visited
        for (int y = 0; y < SIZE; y++)
            for (int x = 0; x < SIZE; x++)
                nodes[y][x] = {0,0,0,0,false};
    }

    // Mark a node as visited
    void markVisited(int y, int x) {
        nodes[y][x].visited = true;
    }

    // Set available paths of a node
    void setPaths(int y, int x, bool n, bool e, bool s, bool w) {
        nodes[y][x].north = n;
        nodes[y][x].east  = e;
        nodes[y][x].south = s;
        nodes[y][x].west  = w;
    }

    // Check if a path exists
    bool canGoNorth(int y, int x) { return nodes[y][x].north; }
    bool canGoEast(int y, int x)  { return nodes[y][x].east; }
    bool canGoSouth(int y, int x) { return nodes[y][x].south; }
    bool canGoWest(int y, int x)  { return nodes[y][x].west; }

    // Check if node visited
    bool isVisited(int y, int x) { return nodes[y][x].visited; }

    void printMazeFull() {
    Serial.println("Maze Structure:");
    for (int y = 0; y < SIZE; y++) {
        for (int x = 0; x < SIZE; x++) {
            Node &n = nodes[y][x];
            Serial.print("[");
            Serial.print(n.north ? "N" : " "); 
            Serial.print(n.east  ? "E" : " ");
            Serial.print(n.south ? "S" : " "); 
            Serial.print(n.west  ? "W" : " ");
            Serial.print(n.visited ? "V" : " ");
            Serial.print("] ");
        }
        Serial.println();
    }
    Serial.println();
}
    Node getNode(int y, int x) {
    return nodes[y][x];  // returns a copy of the node
}
};
