//=======================================================================
#ifndef MAPLOADER_CPP
#define MAPLOADER_CPP
#include <lazycbs/map_loader.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <climits>
#include <float.h>

using namespace boost;
using namespace std;
namespace lazycbs{
MapLoader::MapLoader(int rows, int cols) {
  map_filename = "NEW_EMPTY";
  int i, j;
  this->rows = rows;
  this->cols = cols;
  this->my_map = new bool[rows*cols];
  std::fill_n(this->my_map, rows * cols, false);
  actions_offset = new int[5];
  actions_offset[0] = 0; actions_offset[1] = -cols; actions_offset[2] = 1; actions_offset[3] = cols; actions_offset[4] = -1; // [WAIT, NORTH, EAST, SOUTH, WEST]
  // Add a blocked border so neighbor expansion never needs boundary checks.
  i = 0;
  for (j = 0; j < cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  i = rows - 1;
  for (j = 0; j < cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j = 0;
  for (i = 0; i < rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j = cols - 1;
  for (i = 0; i < rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;

}

MapLoader::MapLoader(string fname){
  map_filename = string(fname);
  string line;
  ifstream myfile (fname.c_str());
  bool* loaded_map;
  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    const int parsed_rows = atoi((*beg).c_str()); // read number of rows
    beg++;
    const int parsed_cols = atoi((*beg).c_str()); // read number of cols
    loaded_map = new bool[parsed_rows * parsed_cols];
    std::fill_n(loaded_map, parsed_rows * parsed_cols, false);

    // Read the grid row-by-row. "S" and "G" are marked as traversable cells.
    for (int i = 0; i < parsed_rows; i++) {
      getline (myfile, line);
      tokenizer< char_separator<char> > col_tok(line, sep);
      tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
      for (int j = 0; j < parsed_cols; j++, ++c_beg) {
        const int loc = parsed_cols * i + j;
        if ((*c_beg).compare("S") == 0) {
          loaded_map[loc] = false;
          start_loc = loc;
        } else if ((*c_beg).compare("G") == 0) {
          loaded_map[loc] = false;
          goal_loc = loc;
        } else {
          loaded_map[loc] = atoi((*c_beg).c_str()) == 1;
        }
      }
    }
    myfile.close();
    this->rows = parsed_rows;
    this->cols = parsed_cols;
    this->my_map = loaded_map;
    // Initialize action offsets once the final column count is known.
    actions_offset = new int[5];
    actions_offset[0] = 0; actions_offset[1] = -parsed_cols; actions_offset[2] = 1; actions_offset[3] = parsed_cols; actions_offset[4] = -1;
  }
  else
    cerr << "Map file not found." << std::endl;
}

MapLoader::MapLoader(int rows, int cols, std::vector<std::pair<int, int> > obstacles){
  this->rows = rows+2;
  this->cols = cols+2;
  bool* loaded_map = new bool[this->rows*this->cols];
  for (int i=0; i<this->rows*this->cols; i++)
      loaded_map[i] = false;
  for(int i=0; i<this->rows; i++){
    loaded_map[this->cols*i+0] = true;
    loaded_map[this->cols*i + this->cols-1] = true;
  }
  for(int i=0; i<this->cols; i++){
    loaded_map[this->cols*0 + i] = true;
    loaded_map[this->cols*(this->rows-1) + i] = true;
  }
  for (const auto& obstacle : obstacles) {
    const int x = obstacle.first + 1;
    const int y = obstacle.second + 1;
    loaded_map[x*this->cols + y] = true;
  }

    this->my_map = loaded_map;
    // Initialize action offsets once the padded dimensions are known.
    actions_offset = new int[5];
    actions_offset[0] = 0; actions_offset[1] = -this->cols; actions_offset[2] = 1; actions_offset[3] = this->cols; actions_offset[4] = -1;

}

char* MapLoader::mapToChar() {
  char* mapChar = new char[rows*cols];
  for (int i=0; i<rows*cols; i++) {
    if ( i == start_loc )
      mapChar[i] = 'S';
    else if ( i == goal_loc )
      mapChar[i] = 'G';
    else if (this->my_map[i] == true)
      mapChar[i] = '*';
    else
      mapChar[i] = ' ';
  }
  return mapChar;
}

void MapLoader::printMap () {
  char* mapChar = mapToChar();
  printMap (mapChar);
  delete[] mapChar;
}


void MapLoader::printMap (char* mapChar) {
  cout << "MAP:";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    cout << mapChar[i];
  }
  cout << endl;
}

void MapLoader::printHeuristic (const double* mapH, const int agent_id) {
  cout << endl << "AGENT "<<agent_id<<":";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    if (mapH[i] == DBL_MAX)
      cout << "*,";
    else
      cout << mapH[i] << ",";
  }
  cout << endl;
}

bool* MapLoader::get_map() const {
  bool* retVal = new bool [ this->rows * this->cols ];
  memcpy (retVal, this->my_map, sizeof(bool)* this->rows * this->cols );
  return retVal;
}

MapLoader::~MapLoader() {
  delete[] this->my_map;
  delete[] this->actions_offset;
}

void MapLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open (fname);
  myfile << rows << "," << cols << endl;
  for (int i=0; i<rows; i++) {
    for (int j=0; j<cols; j++) {
      if ( my_map[linearize_coordinate(i,j)] == true)
	myfile << "1";
      else
	myfile << "0";
      if (j<cols-1)
	myfile << ",";
      else
	myfile << endl;
    }
  }
  myfile.close();
}

void MapLoader::printPath(vector<int> path) {
  for (size_t i=0; i<path.size(); i++) {
    cout << "[" << row_coordinate(path[i]) << "," << col_coordinate(path[i]) << "] ; ";
  }
  cout << endl;
}

MapLoader::valid_actions_t MapLoader::get_action (int id1, int id2) const {
  int diff = id2-id1;
  valid_actions_t retVal = WAIT;
  if (diff == -cols)
    retVal = NORTH;
  if (diff == cols)
    retVal = SOUTH;
  if (diff == 1)
    retVal = EAST;
  if (diff == -1)
    retVal = WEST;
  return retVal;
}
}
#endif
