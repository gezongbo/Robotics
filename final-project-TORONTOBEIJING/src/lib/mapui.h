#ifndef MapUI_H
#define MapUI_H
#define DOT_SIZE 5
#define LINE_WIDTH 3

// #define NCURSES
#ifdef NCURSES
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ncurses.h>
#include "src/lib/ui.h"
#endif

#include <iostream>
#include "trojanmap.h"
#include <time.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

class MapUI {
 private:
  TrojanMap map;

 public:
  // Create the menu.
  void PrintMenu();

  // Create the Dynamic menu.
  #ifdef NCURSES
  void DynamicPrintMenu();
  #endif
  // Visualization
  // Plot the map
  void PlotMap();

  // Given a location id, plot the point on the map.
  void PlotPoint(std::string id);

  // Given a lat and lon, plot the point on the map.
  void PlotPoint(double lat, double lon);

  // Given a vector of location ids draws the path (connects the points)
  void PlotPath(std::vector<std::string> &location_ids);

  // Given a vector of location ids draws the points on the map (no path).
  void PlotPoints(std::vector<std::string> &location_ids);

  // Given a vector of location ids draws the points on the map with path.
  void PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square);

  // Given a vector of location ids draws the points with their order on the map (no path).
  void PlotPointsOrder(std::vector<std::string> &location_ids);

  // Given a vector of location ids and origin, draws the points with their label.
  void PlotPointsLabel(std::vector<std::string> &location_ids, std::string origin);

  // Create the videos of the progress to get the path
  void CreateAnimation(std::vector<std::vector<std::string>>, std::string);

  // Transform the location to the position on the map
  std::pair<double, double> GetPlotLocation(double lat, double lon);
};

#endif
