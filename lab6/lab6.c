#include "eyebot.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/*
    BELOW CODE FOR PRIORITY QUEUE SOURCED FROM:
    https://rosettacode.org/wiki/Priority_queue#C
*/
typedef struct {
    int priority;
    int data;
} node_t;

typedef struct {
    node_t *nodes;
    int len;
    int size;
} heap_t;

void push (heap_t *h, int priority, int data) {
    if (h->len + 1 >= h->size) {
        h->size = h->size ? h->size * 2 : 4;
        h->nodes = (node_t *)realloc(h->nodes, h->size * sizeof (node_t));
    }
    int i = h->len + 1;
    int j = i / 2;
    while (i > 1 && h->nodes[j].priority > priority) {
        h->nodes[i] = h->nodes[j];
        i = j;
        j = j / 2;
    }
    h->nodes[i].priority = priority;
    h->nodes[i].data = data;
    h->len++;
}

bool empty(heap_t *h) {
    return !h->len;
}

int pop (heap_t *h) {
    int i, j, k;
    if (!h->len) {
        return NULL;
    }
    int data = h->nodes[1].data;
    h->nodes[1] = h->nodes[h->len];
    h->len--;
    i = 1;
    while (1) {
        k = i;
        j = 2 * i;
        if (j <= h->len && h->nodes[j].priority < h->nodes[k].priority) {
            k = j;
        }
        if (j + 1 <= h->len && h->nodes[j + 1].priority < h->nodes[k].priority) {
            k = j + 1;
        }
        if (k == i) {
            break;
        }
        h->nodes[i] = h->nodes[k];
        i = k;
    }
    h->nodes[i] = h->nodes[h->len + 1];
    return data;
}
/*
    ABOVE CODE FOR PRIORITY QUEUE SOURCED FROM:
    https://rosettacode.org/wiki/Priority_queue#C
*/

//Read the specified file (containing nodes and connectivities), and return an adjacency matrix
//Store the number of nodes in retNumNodes
double **readNodes(char *filename, int *retNumNodes, int ***retCoordinates) {

    //Attempt to open file
    FILE *fin = fopen(filename, "r");
    if(fin == NULL) {
        fprintf(stderr, "Couldn't open file \'%s\'\n", filename);
        exit(EXIT_FAILURE);
    }

    //Allocate starting memory for connected nodes list, and coordinates list
    int numNodes = 0;
    int **connectedNodes = (int **) malloc(sizeof(int *));
    int **coordinates = (int **) malloc(sizeof(int *));

    //Read through file line by line and count number of nodes
    char line[BUFSIZ];
    while(fgets(line, BUFSIZ, fin) != NULL) {
        //Realloc memory to store next values for connected nodes and coordinates
        int numConnectedNodes = 0;
        connectedNodes = (int **) realloc(connectedNodes, sizeof(int *) * (numNodes+1));
        coordinates = (int **) realloc(coordinates, sizeof(int *) * (numNodes+1));

        //Allocate memory for current node's coordinates
        coordinates[numNodes] = (int *) malloc(sizeof(int) * 3);

        //Read coordinates
        int x, y;
        char *token = strtok(line, " ");
        x = atoi(token);
        token = strtok(NULL, " ");
        y = atoi(token);
        coordinates[numNodes][0] = x;
        coordinates[numNodes][1] = y;

        //Read connected nodes
        connectedNodes[numNodes] = (int *) malloc(sizeof(int));
        connectedNodes[numNodes][numConnectedNodes++] = numNodes;
        while((token = strtok(NULL, " ")) != NULL) {
            connectedNodes[numNodes] = (int *) realloc(connectedNodes[numNodes], sizeof(int) * (numConnectedNodes+1));
            connectedNodes[numNodes][numConnectedNodes++] = (atoi(token)-1);
        }

        //Store number of connected nodes
        coordinates[numNodes][2] = numConnectedNodes;

        //Increment number of nodes
        numNodes++;
    }

    //Test print
    for(int i=0; i<numNodes; i++) {
        // printf("%d %d ", coordinates[i][0], coordinates[i][1]);
        for(int j=0; j<coordinates[i][2]; j++) {
            // printf("%d ", connectedNodes[i][j]);
        }
        // printf("\n");
    }
    // printf("\n");

    //Form adjacency matrix
    double **adjMatrix = (double **) malloc(sizeof(double *) * numNodes);
    for(int i=0; i<numNodes; i++) {
        adjMatrix[i] = (double *) malloc(sizeof(double) * numNodes);
    }

    //Init all values in adjacency matrix to 0
    for(int i=0; i<numNodes; i++) {
        for(int j=0; j<numNodes; j++) {
            adjMatrix[i][j] = -1;
        }
    }

    //Compute adjacency matrix
    for(int i=0; i<numNodes; i++) {
        //Loop through indices of connected nodes
        for(int j=0; j<coordinates[i][2]; j++) {
            //Calculate euclidian distance between curNode and curConnectedNode
            adjMatrix[i][connectedNodes[i][j]] = sqrt(pow(abs(coordinates[i][0]-coordinates[connectedNodes[i][j]][0]),2) + pow(abs(coordinates[i][1]-coordinates[connectedNodes[i][j]][1]),2));
        }
    }

    //Free all data not being returned to calling context
    for(int i=0; i<numNodes; i++) {
        free(connectedNodes[i]);
    }
    free(connectedNodes);

    //Return the number of nodes, and the array of coordinates (by modifying pointers)
    *retNumNodes = numNodes;
    *retCoordinates = coordinates;

    //Return adjacency matrix
    return adjMatrix;
}

//Calculate heuristic from curNode to goal using euclidian distance
double heuristic(int curNode, int goal, int **coordinates) {
    //Coordinates of curNode
    int cx = coordinates[curNode][0];
    int cy = coordinates[curNode][1];

    //Coordinates of goal
    int gx = coordinates[goal][0];
    int gy = coordinates[goal][1];

    //Euclidian distance
    return sqrt( pow( abs(cx-gx), 2) + pow( abs(cy-gy), 2) );
}

//Perform A* pathfinding from start node (0) to goal node (numNodes-1)
//Return path (array of integers), path length distance to goal (through pointer dereferencing)
int *AStar(int **coordinates, double **adj, int numNodes, int *retPathLength, double *distToGoal) {

    //Start and goal nodes
    int start = 0;
    int goal = numNodes-1;

    //List of visited nodes
    bool visited[numNodes];

    //Map of parent nodes
    int parents[numNodes];

    //List of distances and function values
    double distances[numNodes];
    double function[numNodes];

    //Initialize all distances, parents and visited nodes
    for(int i=0; i<numNodes; i++) {
        distances[i] = INFINITY;
        function[i] = INFINITY;
        parents[i] = -1;
        visited[i] = false;
    }
    parents[0] = -2;
    distances[0] = 0;
    function[0] = 0;

    //Priority queue for nodes
    heap_t *h = (heap_t *) calloc(1, sizeof(heap_t));
    push(h, function[start], start);

    //While priority queue isn't empty
    while(!empty(h) && !visited[goal]) {

        //Pop current node off priority queue and mark as visited
        int curNode = pop(h);

        //If node is visited, skip it
        if(visited[curNode]) continue;

        //Mark node as visited
        visited[curNode] = true;

        //Check each adjacent node of current node
        for(int i=0; i<numNodes; i++) {
            if(adj[curNode][i] != -1) {
                if(!visited[i]) {
                    //Neighbour is connected, and unvisited. Check distance against existing distance
                    if(function[i] > (distances[curNode] + adj[curNode][i] + heuristic(i, goal, coordinates))) {
                        //Update distance and function for current neighbour
                        distances[i] = distances[curNode] + adj[curNode][i];
                        function[i] = distances[i] + heuristic(i, goal, coordinates);

                        //Add to priority queue
                        push(h, function[i], i);

                        //Mark parent
                        parents[i] = curNode;
                    }
                    ;
                }
                ;
            }
        }
    }

    //Check if there is a path to the goal
    if(!visited[goal]) {
        *distToGoal = -1;
        *retPathLength = -1;
        return NULL;
    }

    //Reconstruct path
    int *revPath = (int *) malloc(sizeof(int));
    int pathLength = 0;
    revPath[pathLength++] = goal;
    int parent = parents[goal];
    while(parent != -2) {
        revPath = (int *) realloc(revPath, sizeof(int) * (pathLength + 1));
        revPath[pathLength++] = parent;
        parent = parents[parent];
    }

    //Reverse path
    int *path = (int *) malloc(sizeof(int) * pathLength);
    for(int i=0; i<pathLength; i++) {
        path[i] = revPath[pathLength-1-i];
    }

    //Free reversed path
    free(revPath);

    //Return path and path length
    *distToGoal = distances[goal];
    *retPathLength = pathLength;
    return path;
}

//Converts degrees to radians
double toRadian(double deg) {
    return ((M_PI/180.0) * deg);
}

//Converts radians to degrees
double radToDeg(double rad) {
    return ((180.0/M_PI) * rad);
}

//Drive from source to goal using path found by A* pathfinding algorithm
void waypoints(int *path, int **coordinates, int pathLength) {
    //Waypoints array
    int x[100];
    int y[100];
    int i=0;
    //Read in waypoints from path and coordinates
    for(i=0; i<pathLength; i++) {
        x[i] = coordinates[path[i]][0];
        y[i] = -coordinates[path[i]][1];
    }

    //Move between points
    int fin=0;
    for(int j=0; j<i; j++) {
        //Move to current waypoint
        int curx, cury, curphi;
        VWGetPosition(&curx, &cury, &curphi);

        //Calculate angle and magnitude
        int angle = radToDeg(atan2(y[j]-cury, x[j]-curx)) - curphi;
        int mag = sqrt(pow(x[j]-curx,2) + pow(y[j]-cury,2));
        // if(j==i) {
        //     angle += 180;
        // }
        //Turn
        VWTurn(angle, 30);
        printf("%d %d\n", curx, -cury);
        printf("%d %d\n", x[j], -y[j]);
        printf("%d\n\n", angle);
        while(!VWDone()) {
        }
        //Drive
        VWStraight(mag, 50);
        while(!VWDone()) {
        }
    }
}

//Free all allocated memory for path, coordinated and adj arrays
void freeMem(int *path, int **coordinates, double **adj, int numNodes) {
    free(path);
    for(int i=0; i<numNodes; i++) {
        free(coordinates[i]);
        free(adj[i]);
    }
    free(coordinates);
    free(adj);
}

int main(int argc, char **argv) {
    //Read nodes file into adjacency matrix. Record coordinates of each node, and the number of nodes
    int numNodes = 0;
    int **coordinates;
    double **adj = readNodes(argv[1], &numNodes, &coordinates);

    //Print adjacency matrix
    for(int i=0; i<numNodes; i++) {
        for(int j=0; j<numNodes; j++) {
            printf("%.1f\t", adj[i][j]);
        }
        printf("\n");
    }
    printf("\n");

    //Perform A* pathfinding from source (first node) to goal (last node)
    double dist;
    int pathLength;
    int *path;
    path = AStar(coordinates, adj, numNodes, &pathLength, &dist);

    //Print path (if it exists)
    if(pathLength != -1) {
        printf("Goal: %d\n", numNodes-1);
        printf("%d", path[0]);
        for(int i=1; i<pathLength; i++) {
            printf("->%d", path[i]);
        }
        printf("\n");
        printf("Distance: %.1f\n", dist);
    }
    else {
        //Path doesn't exist (to goal)
        printf("Goal: %d\n", numNodes-1);
        printf("No path to goal exists\n");
        exit(EXIT_SUCCESS);
    }

    //Drive path from source to goal
    waypoints(path, coordinates, pathLength);
    exit(EXIT_SUCCESS);

    //Free all allocated memory
    freeMem(path, coordinates, adj, numNodes);
}
