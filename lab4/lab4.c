#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "image.c"

int **occupied;
int **clear;
int **paths;
int numOccupied;
int numClear;
int numPaths = 0;

//Recursively subdivide image, and store center coordinates and size of clear and occupied spaces
void subdivide(BYTE img[128][128], int x, int y, int size) {
  //Continuously check whether or not the block is all occupied/free
  bool clearOrOccupied = true;
  //Check first square
  int first = (int) img[x][y];
  //Loop over all squares
  for(int i=x; i<x+size; i++) {
    for(int j=y; j<y+size; j++) {
      //If all squares aren't identical, perform another recursive subdivision step
      if(img[i][j] != first) {
        //Recursive subdivision. Mark bool as false, and break out of loop
        clearOrOccupied = false;
        break;
      }
    }
  }
  if(!clearOrOccupied) {
    //Recursively subdivide
    subdivide(img,x,y,size/2);
    subdivide(img,x+(size/2),y,size/2);
    subdivide(img,x,y+(size/2),size/2);
    subdivide(img,x+(size/2),y+(size/2),size/2);
  }
  else {
    //Square is completely clear or completely occupied
    if(first == 0) {
      //Clear
      clear = (int **) realloc(clear, sizeof(int *) * (numClear+1));
      clear[numClear] = (int *) malloc(sizeof(int) * 3);
      clear[numClear][0] = x;
      clear[numClear][1] = y;
      clear[numClear++][2] = size;
    }
    else if(first == 1) {
      //Occupied
      occupied = (int **) realloc(occupied, sizeof(int *) * (numOccupied+1));
      occupied[numOccupied] = (int *) malloc(sizeof(int) * 3);
      occupied[numOccupied][0] = x;
      occupied[numOccupied][1] = y;
      occupied[numOccupied++][2] = size;
    }
  }
}

void freeClearOccupied() {
  for(int i=0; i<numClear; i++) {
    free(clear[i]);
  }
  free(clear);
  for(int i=0; i<numOccupied; i++) {
    free(occupied[i]);
  }
  free(occupied);
}

void freePaths() {
  for(int i=0; i<numPaths; i++) {
    free(paths[i]);
  }
  free(paths);
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the floats i_x and i_y.
//Source: https://cboard.cprogramming.com/c-programming/154196-check-if-two-line-segments-intersect.html
char get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
    float p2_x, float p2_y, float p3_x, float p3_y/*, float *i_x, float *i_y*/)
{
    float s1_x, s1_y, s2_x, s2_y, sn, tn, sd, td;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    sn = -s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y);
    sd = -s2_x * s1_y + s1_x * s2_y;
    tn =  s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x);
    td = -s2_x * s1_y + s1_x * s2_y;

    if (sn >= 0 && sn <= sd && tn >= 0 && tn <= td)
    {
        // Collision detected
        /*t = tn / td;
        if (i_x != NULL)
            *i_x = p0_x + (tn * s1_x);
        if (i_y != NULL)
            *i_y = p0_y + (tn * s1_y);*/
        return 1;
    }

    return 0; // No collision
}

//Given two clear blocks (the indices into the global 'clear' array for each block), determine whether or not a straight line between them intersects an occupied block
bool intersection(int b1, int b2) {
  //How on earth do we do this?
  //Coordinates and size of first clear block (left hand corner)
  int ax = clear[b1][0] + clear[b1][2]/2;
  int ay = clear[b1][1] + clear[b1][2]/2;

  //Coordinates and size of second clear block (left hand corner)
  int bx = clear[b2][0] + clear[b2][2]/2;
  int by = clear[b2][1] + clear[b2][2]/2;

  //Check size of clear blocks (small ones are useless)
  if(clear[b1][2] < 16 || clear[b2][2] < 16) {
    //Clear block(s) too small; return true (as though there is intersection), as the block is too small for vehicle to drive there
    return true;
  }

  //Loop over all occupied blocks
  for(int i=0; i<numOccupied; i++) {
  //Coordinates and size
  int ox = occupied[i][0]; //x coordinate (left upper hand corner)
  int oy = occupied[i][1]; //y coordinate (left upper hand corner)
  int osize = occupied[i][2]; //size
    // top left
    int x1 = ox;
    int y1 = oy;
    // top right
    int x2 = ox + osize;
    int y2 = oy;
    // bottom right
    int x3 = ox + osize;
    int y3 = oy + osize;
    //bottom left
    int x4 = ox;
    int y4 = oy + osize;

    //top side (top left -> top right)
    if(get_line_intersection(ax, ay, bx, by, x1, y1, x2, y2)) return true;
    //right side (top right -> bottom right)
    if(get_line_intersection(ax, ay, bx, by, x2, y2, x3, y3)) return true;
    //bottom side (bottom right -> bottom left)
    if(get_line_intersection(ax, ay, bx, by, x3, y3, x4, y4)) return true;
    //left side (bottom left -> top right)
    if(get_line_intersection(ax, ay, bx, by, x4, y4, x1, y1)) return true;
  }
  return false;
}

void constructPaths() {
    //Loop over all pairs of clear blocks
  for(int i=0; i<numClear; i++) {
    for(int j=i+1; j<numClear; j++) {
    	//Determine whether or not this pair of clear blocks is unobstructed
      if(!intersection(i, j)) {
        //No intersection with occupied block. Record path
        paths = (int **) realloc(paths, sizeof(int *) * (numPaths+1));
        paths[numPaths] = (int *) malloc(sizeof(int) *4);
        int sizeOfClear1 = clear[i][2];
        int sizeOfClear2 = clear[j][2];
        paths[numPaths][0] = clear[i][0] + sizeOfClear1/2; //x-coordinate of center of first clear block
        paths[numPaths][1] = clear[i][1] + sizeOfClear1/2; //y-coordinate of center of first clear block
        paths[numPaths][2] = clear[j][0] + sizeOfClear2/2; //x-coordinate of center of second clear block
        paths[numPaths++][3] = clear[j][1] + sizeOfClear2/2; //y-coordinate of center of second clear block
      }
    }
  }
}

int main(int argc, char * argv[]) {
  if(argc != 2) {
    printf("Usage: %s <file.pbm>", argv[0]);
  }
  //Byte array of image
  BYTE *img;

  //Read pbm file into byte array
  read_pbm(argv[1], &img);

  //Display byte array on LCD screen
  LCDArea(0,0,128,128,0xFFFFFF,1);
  LCDImageStart(0,0,128,128);
  LCDImageBinary(img);

  //Construct 2D array
  BYTE image[128][128];
  for(int i=0; i<128; i++) {
    for(int j=0; j<128; j++) {
      image[j][i] = img[i*128+j];
    }
  }

  //Allocate memory for occupied and clear buffers
  occupied = (int **) malloc(sizeof(int *));
  clear = (int **) malloc(sizeof (int *));
  numOccupied = 0;
  numClear = 0;

  //Recursively subdivide
  subdivide(image, 0, 0, 128);

  //Print number of clear and occupied squares found
  printf("Clear: %d, Occupied: %d\n", numClear, numOccupied);

  //Display boxes
  for(int i=0; i<numOccupied; i++) {
    //Occupied (print red lines)
    LCDArea(occupied[i][0], occupied[i][1], occupied[i][0] + occupied[i][2], occupied[i][1] + occupied[i][2], 0xff0000,0);
  }
  for(int i=0; i<numClear; i++) {
    //Clear (print green lines)
    LCDArea(clear[i][0], clear[i][1], clear[i][0] + clear[i][2], clear[i][1] + clear[i][2], 0x00ff00,0);
  }

  //Construct paths array
  paths = (int **) malloc(sizeof(int *));
    numPaths = 0;
  constructPaths();

  //Display paths using LCD
  for(int i=0; i<numPaths; i++) {
    //Paint path on LCD display
    LCDLine(paths[i][0], paths[i][1], paths[i][2], paths[i][3], 0xffff00);
    //Print distance of path to stdout
    printf("Path %d: %f\n",i+1, sqrt( pow( abs( paths[i][0]-paths[i][2] ) , 2) + pow( abs( paths[i][1]-paths[i][3] ) , 2) ) );
  }

  //Continuously display image file
  while(true);

  //Free memory allocated for clear and occupied arrays
  freeClearOccupied();
  freePaths();

  return 0;
}
