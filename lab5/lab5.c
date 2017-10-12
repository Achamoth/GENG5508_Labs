#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

int VSIZE;
int HSIZE;

//Open specified file, read in occupancy grid, and return grid as 2D int array
int **readOccupancyGrid(char *filename) {
    //Result
    int **grid;

    //Open file
    FILE *fin = fopen(filename, "r");
    if(fin == NULL) {
        fprintf(stderr, "Couldn't open file \'%s\'\n", filename);
        exit(EXIT_FAILURE);
    }

    //Read first two lines of file
    char line[BUFSIZ];
    fgets(line, BUFSIZ, fin);
    fgets(line, BUFSIZ, fin);
    int rows, cols;
    sscanf(line, "%d %d", &rows, &cols);
    VSIZE = rows;
    HSIZE = cols;

    //Allocate memory for int array
    grid = (int **) malloc(sizeof(int *) * rows);
    for(int i=0; i<rows; i++) {
        grid[i] = (int *) malloc(sizeof(int) * cols);
    }

    //Read in grid data
    int i=0;
    while(fgets(line, BUFSIZ, fin) != NULL) {
        int k=0;
        for(int j=0; j<cols; j++) {
            grid[i][j] = line[k]-'0';
            k += 2;
        }
        i++;
    }

    //Return pointer to array
    return grid;
}

//Visit specified square, and recursively visit all adjacent obstacle squares
void visit(bool visited[VSIZE][HSIZE], int **grid, int i, int j, int colors[VSIZE][HSIZE], int color) {

    //Check if square is occupied
    if(grid[i][j] == 0) return ;

    //Visit square, and use recursion to visit all squares adjacent that are obstacles
    visited[i][j] = true;
    colors[i][j] = color;

    if(i-1 >= 0) {
        //Visit square above current square
        if(!visited[i-1][j]) visit(visited, grid, i-1, j, colors, color);
    }

    if(i+1 < VSIZE) {
        //Visit square below current square
        if(!visited[i+1][j]) visit(visited, grid, i+1, j, colors, color);
    }

    if(j-1 >= 0) {
        //Visit square to left of current square
        if(!visited[i][j-1]) visit(visited, grid, i, j-1, colors, color);
    }

    if(j+1 < HSIZE) {
        //Visit square to right of current square
        if(!visited[i][j+1]) visit(visited, grid, i, j+1, colors, color);
    }
}

//Returns true if the specified point is a voronoi point
bool isVoronoiPoint(int i, int j, int k, int **grid, int colors[VSIZE][HSIZE]) {
    //Top neighbour is i-1
    if(i-1 >= 0) {
        //Check if it's been overwritten with k in a different color
        if(grid[i-1][j] == k && colors[i-1][j] != colors[i][j] && colors[i-1][j] != 0) {
            return true;
        }
    }

    //Right neighbour is j+1
    if(j+1 < HSIZE) {
        //Check if its been overwritten with k in a different color
        if(grid[i][j+1] == k && colors[i][j+1] != colors[i][j] && colors[i][j+1] != 0) {
            return true;
        }
    }

    //Else, it's not a voronoi point
    return false;
}

//Returns 0 if no neighbours are colored, 1 if all colored neighbours are the same color, 2 if there is more than one color
//Need to consider edges as well; they're considered colored
int getNumColoredNeighbours(int i, int j, int colors[VSIZE][HSIZE]) {
    int *neighbourCols = malloc(sizeof(int) * 1);
    int k = 0;

    int VORONOI = 0;
    int PINK = 1;
    int BLUE = 2;
    int ORANGE = 3;
    int YELLOW = 4;
    int RED = 5;
    int GREEN = 6;
    int SILVER = 7;
    int GOLD = 8;

    //Pixel above
    if(i-1 >= 0) {
        if(colors[i-1][j] != -1 && colors[i-1][j] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i-1][j];
        }
    }
    //Pixel below
    if(i+1 < VSIZE) {
        if(colors[i+1][j] != -1 && colors[i+1][j] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i+1][j];
        }
    }
    //Pixel to the left
    if(j-1 >= 0) {
        if(colors[i][j-1] != -1 && colors[i][j-1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i][j-1];
        }
    }
    //Pixel to the right
    if(j+1 < HSIZE) {
        if(colors[i][j+1] != -1 && colors[i][j+1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i][j+1];
        }
    }
    //Pixel above and to the left
    if(i-1 >= 0 && j-1 >= 0) {
        if(colors[i-1][j-1] != -1 && colors[i-1][j-1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i-1][j-1];
        }
    }
    //Pixel above and to the right
    if(i-1 >= 0 && j+1 < HSIZE) {
        if(colors[i-1][j+1] != -1 && colors[i-1][j+1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i-1][j+1];
        }
    }
    //Pixel below and to the left
    if(j-1 >= 0 && i+1 < VSIZE) {
        if(colors[i+1][j-1] != -1 && colors[i+1][j-1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i+1][j-1];
        }
    }
    //Pixel below and to the right
    if(j+1 >= 0 && i+1 < VSIZE) {
        if(colors[i+1][j+1] != -1 && colors[i+1][j+1] != VORONOI) {
            neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
            neighbourCols[k++] = colors[i+1][j+1];
        }
    }
    //Check if the pixel is on the top row
    if(i == 0) {
        neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
        neighbourCols[k++] = PINK;
    }
    //Check if pixel is on the bottom row
    if(i == VSIZE-1) {
        neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
        neighbourCols[k++] = YELLOW;
    }
    //Check if pixel is on the left column
    if(j == 0) {
        neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
        neighbourCols[k++] = ORANGE;
    }
    //Check if pixel is on the right column
    if(j == HSIZE-1) {
        neighbourCols = realloc(neighbourCols, sizeof(int) * (k+1));
        neighbourCols[k++] = BLUE;
    }

    //If k==0, there are no colored neighbours
    if(k == 0) {
        return 0;
    }
    //If k==1, there is only one colored neighbour
    if(k==1) {
        return 1;
    }
    //Else, if k>1, and they're not all the same, then there is more than one colored neighbour
    int lastCol = neighbourCols[0];
    for(int m=1; m<k; m++) {
        int curCol = neighbourCols[m];
        if(lastCol != curCol) {
            return 2;
        }
        lastCol = curCol;
    }
    return 1;
}

int getColorOfNeighbour(int i, int j, int colors[VSIZE][HSIZE]) {
    int color = -1;

    int VORONOI = 0;
    int PINK = 1;
    int BLUE = 2;
    int ORANGE = 3;
    int YELLOW = 4;
    int RED = 5;
    int GREEN = 6;
    int SILVER = 7;
    int GOLD = 8;

    //Pixel above
    if(i-1 >= 0) {
        if(colors[i-1][j] != -1 && colors[i-1][j] != VORONOI) {
            return colors[i-1][j];
        }
    }
    //Pixel below
    if(i+1 < VSIZE) {
        if(colors[i+1][j] != -1 && colors[i+1][j] != VORONOI) {
            return colors[i+1][j];
        }
    }
    //Pixel to the right
    if(j-1 >= 0) {
        if(colors[i][j-1] != -1 && colors[i][j-1] != VORONOI) {
            return colors[i][j-1];
        }
    }
    //Pixel to the left
    if(j+1 < HSIZE) {
        if(colors[i][j+1] != -1 && colors[i][j+1] != VORONOI) {
            return colors[i][j+1];
        }
    }
    //Pixel above and to the left
    if(i-1 >= 0 && j-1 >= 0) {
        if(colors[i-1][j-1] != -1 && colors[i-1][j-1] != VORONOI) {
            return colors[i-1][j-1];
        }
    }
    //Pixel above and to the right
    if(i-1 >= 0 && j+1 < HSIZE) {
        if(colors[i-1][j+1] != -1 && colors[i-1][j+1] != VORONOI) {
            return colors[i-1][j+1];
        }
    }
    //Pixel below and to the left
    if(j-1 >= 0 && i+1 < VSIZE) {
        if(colors[i+1][j-1] != -1 && colors[i+1][j-1] != VORONOI) {
            return colors[i+1][j-1];
        }
    }
    //Pixel below and to the right
    if(j+1 >= 0 && i+1 < VSIZE) {
        if(colors[i+1][j+1] != -1 && colors[i+1][j+1] != VORONOI) {
            return colors[i+1][j+1];
        }
    }

    //Top row
    if(i==0) {
        return PINK;
    }
    //Bottom row
    if(i==VSIZE-1) {
        return YELLOW;
    }
    //Left column
    if(j==0) {
        return ORANGE;
    }
    //Right column
    if(j==HSIZE-1) {
        return BLUE;
    }
}

//Perform brushfire algorithm on grid
bool **brushfire(int **grid) {
    //Allocate memory for voronoi records
    bool **voronoi = (bool **) malloc(sizeof(bool *) * VSIZE);
    for(int i=0; i<VSIZE; i++) voronoi[i] = (bool *) malloc(sizeof(bool) * HSIZE);

    //Allocate memory for color records
    int colors[VSIZE][HSIZE];
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            colors[i][j] = -1;
        }
    }

    //Color enums
    int VORONOI = 0;
    int PINK = 1;
    int BLUE = 2;
    int ORANGE = 3;
    int YELLOW = 4;
    int RED = 5;
    int GREEN = 6;
    int SILVER = 7;
    int GOLD = 8;

    //Visited array
    bool visited[VSIZE][HSIZE];
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            visited[i][j] = false;
        }
    }

    int curColor = 4;
    //Loop through, and assign color to each obstacle
    for(int i=0; i<VSIZE; i++) {
        for (int j=0; j<HSIZE; j++) {
            //Check if square is occupied
            if(grid[i][j] == 1) {
                //If it is, check if it's already been visited
                if(!visited[i][j]) {
                    //If it hasn't, visit and color it and the entire obstacle it belongs to
                    visit(visited, grid, i, j, colors, curColor);
                    curColor++;
                }
            }
        }
    }

    //Test print color of each square
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            // printf("%d\t", colors[i][j]);
        }
        // printf("\n");
    }

    //TODO: Here onwards may not be correct
    //Begin brushfire algorithm
    int k=2;
    bool change = true;
    //Loop until there is no change
    while(change) {
        //Intially record change as false
        change = false;
        //Loop over all squares
        for(int i=0; i<VSIZE; i++) {
            for(int j=0; j<HSIZE; j++) {
                //Check if the pixel is free
                if(grid[i][j] == 0) {
                    //Check if it's neighbours are labeled (or a border)
                    int numColoredNeighbours = getNumColoredNeighbours(i, j, colors);
                    if(numColoredNeighbours == 1) {
                        //Get the color
                        change = true;
                        grid[i][j] = k;
                        colors[i][j] = getColorOfNeighbour(i, j, colors);
                    }
                    else if(numColoredNeighbours > 1) {
                        //Make this square a voronoi point
                        change = true;
                        grid[i][j] = k;
                        colors[i][j] = VORONOI;
                    }
                }
            }
        }

        //Loop over all squares again to identify voronoi points
        for(int i=0; i<VSIZE; i++) {
            for(int j=0; j<HSIZE; j++) {
                //Check each pixel to see if it's a voronoi point
                bool isVoronoi = isVoronoiPoint(i, j, k, grid, colors);
                if(isVoronoi) {
                    colors[i][j] = VORONOI;
                }
            }
        }
        k++;
    }

    //Determine all voronoi points
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            if(colors[i][j] == VORONOI) {
                voronoi[i][j] = true;
            }
        }
    }

    //Return voronoi points
    return voronoi;
}

int main(int argc, char **argv) {
    //Read in occupancy grid
    int **grid = readOccupancyGrid(argv[1]);

    //Test print
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            // printf("%d ", grid[i][j]);
        }
        // printf("\n");
    }

    //Perform brushfire algorithm on grid
    bool **voronoi = brushfire(grid);

    //Print voronoi points
    for(int i=0; i<VSIZE; i++) {
        for(int j=0; j<HSIZE; j++) {
            printf("%d ", voronoi[i][j] ? 1 : 0);
        }
        printf("\n");
    }

    //Free grid
    for(int i=0; i<VSIZE; i++) free(grid[i]);
    free(grid);

    //Free voronoi
    for(int i=0; i<VSIZE; i++) free(voronoi[i]);
    free(voronoi);
}
