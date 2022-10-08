#include <iostream>
#include <vector>
#include <cassert>
#include <fstream>
#include <string>
#include <cstring>
#include <list>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wdigraph.h"
#include "dijkstra.h"

#define MAX_SIZE 1024

struct Point {
    long long lat, lon;
};


/*
Description: Computed the Manhattan distance between two points
Arguments: 
  onst Point& pt1: The first point
  const Point& pt2: The second point
Returns:
  The manhattan distance between the two given argument points
*/
long long manhattan(const Point& pt1, const Point& pt2) {
  long long dLat = pt1.lat - pt2.lat, dLon = pt1.lon - pt2.lon;
  return abs(dLat) + abs(dLon);
}

/*
Description: finds the id of the point that is closest to the given point "pt"
Arguments: 
  const Point& pt: The point close to a vertex in 'points'
  const unordered_map<int, Point>& points: An unordered map of vertices and their
        corresponding latitude and longitude.
Returns:
  The ID of the closest vertex in points to the given point
*/
int findClosest(const Point& pt, const unordered_map<int, Point>& points) {
  pair<int, Point> best = *points.begin();

  for (const auto& check : points) {
    if (manhattan(pt, check.second) < manhattan(pt, best.second)) {
      best = check;
    }
  }
  return best.first;
}

/*
Description: Reads the graph from the file that has the same format as the 
  "Edmonton graph" file
Arguments: 
  const string& filename: The name of the file to read from
  WDigraph& g: The weighted direction graph to be created
  unordered_map<int, Point>& points: An unordered map of numbered points (lat,lon)
Returns:
  Doesn't return anything as it is a void function, but it created a weighted
  directional graph described by the input.
*/
void readGraph(const string& filename, WDigraph& g, unordered_map<int, Point>& points) {
  ifstream fin(filename);
  string line;

  while (getline(fin, line)) {
    // split the string around the commas, there will be 4 substrings either way
    string p[4];
    int at = 0;
    for (auto c : line) {
      if (c == ',') {
        // start new string
        ++at;
      }
      else {
        // append character to the string we are building
        p[at] += c;
      }
    }

    if (at != 3) {
      // empty line
      break;
    }

    if (p[0] == "V") {
      // new Point
      int id = stoi(p[1]);
      assert(id == stoll(p[1])); // sanity check: asserts if some id is not 32-bit
      points[id].lat = static_cast<long long>(stod(p[2])*100000);
      points[id].lon = static_cast<long long>(stod(p[3])*100000);
      g.addVertex(id);
    }
    else {
      // new directed edge
      int u = stoi(p[1]), v = stoi(p[2]);
      g.addEdge(u, v, manhattan(points[u], points[v]));
    }
  }
}


/*
Description: Opens a pipe and handles errors upon opening
Arguments: 
  const char * pname: Pipe name
  int mode: The desired more (read or write)
Returns:
  File descriptor if the pipe is opened succesfully
*/
int create_and_open_fifo(const char * pname, int mode) {
  // creating a fifo special file in the current working directory
  // with read-write permissions for communication with the plotter
  // both proecsses must open the fifo before they can perform
  // read and write operations on it
  if (mkfifo(pname, 0666) == -1) {
    cout << "Unable to make a fifo. Ensure that this pipe does not exist already!" << endl;
    exit(-1);
  }

  // opening the fifo for read-only or write-only access
  // a file descriptor that refers to the open file description is
  // returned
  int fd = open(pname, mode);

  if (fd == -1) {
    cout << "Error: failed on opening named pipe." << endl;
    exit(-1);
  }

  return fd;
}


int main() {
  WDigraph graph;
  unordered_map<int, Point> points;

  int byteswritten = 0;
  const char *inpipe = "inpipe";
  const char *outpipe = "outpipe";

  // Open the two pipes
  int in = create_and_open_fifo(inpipe, O_RDONLY);
  cout << "inpipe opened..." << endl;
  int out = create_and_open_fifo(outpipe, O_WRONLY);
  cout << "outpipe opened..." << endl;  

  // build the graph
  readGraph("server/edmonton-roads-2.0.1.txt", graph, points);

  //read requests until Q is read
  while (true){  

  Point sPoint, ePoint;
  vector<string> BELL (4,""); //start lat and lon + end lat and lon = 4
  char buffer[MAX_SIZE];
  int position = 0;

  int bytesread = read(in, buffer, MAX_SIZE);
  if (buffer[0] == 'Q'){ //Exit condition from client
    break;
  }

  //going over all the bytes read in from the pipe and assigning lat and lon
  //to corresponding point (start first then end)
  for (int i = 0;i < bytesread; i++){
    if (buffer[i] == ' ' || buffer[i] == '\n'){
      BELL[position].pop_back();
      position++;
    }
    else if(buffer[i] == '.'){
      continue;
    }
    else{
      BELL[position].push_back(buffer[i]);
    }
  }
  sPoint.lat = stoll(BELL[0]), sPoint.lon = stoll(BELL[1]);
  ePoint.lat = stoll(BELL[2]), ePoint.lon = stoll(BELL[3]);


  // get the points closest to the two points we read
  int start = findClosest(sPoint, points), end = findClosest(ePoint, points);
  if (start == end){

  }
  // run dijkstra's algorithm, this is the unoptimized version that
  // does not stop when the end is reached but it is still fast enough
  unordered_map<int, PIL> tree;
  dijkstra(graph, start, tree);

  // Code below is modified to communicate with the client
  char e[2] = {'E','\n'};
  // no path
  if (tree.find(end) == tree.end() || start == end) {
    byteswritten = write(out, e, 2);
  }
  else {
    // read off the path by stepping back through the search tree
    list<int> path;
    while (end != start) {
      path.push_front(end);
      end = tree[end].first;
    }
    path.push_front(start);

    for (int v : path) {
      long long div = 100000;
      string strlat, strlon;

      double lat = (double)points[v].lat/div, lon = (double)points[v].lon/div;
      strlat = to_string(lat), strlon = to_string(lon);
      strlat.pop_back();
      strlon.pop_back();

      string output = strlat+' '+strlon;

      char outarr[output.size()+2];
      strcpy(outarr,output.c_str());
      outarr[19] = '\n';
      outarr[20] = '\0';

      byteswritten = write(out, outarr, strlen(outarr));
      byteswritten += 1; // no functionality. Just makes compilers happy to see byteswritten change apon loop iterations
    }
    byteswritten = write(out, e, 2);
  }
  }
  //Code below executes when client is closed. Closes and unlinks both pipes.
  close(in);
  close(out);

  unlink(inpipe);
  unlink(outpipe);

  return 0;
}
