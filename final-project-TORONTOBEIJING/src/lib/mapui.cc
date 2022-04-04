#include "mapui.h"
/**
 * PrintMenu: Create the menu
 * 
 */
void MapUI::PrintMenu() {
  std::string menu =
      "TrojanMap\n"
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the location                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Cycle Detection                                          \n"
      "* 6. Topological Sort                                         \n"
      "* 7. Find Nearby                                              \n"
      "* 8. Exit                                                     \n"
      "**************************************************************\n"
      "Please select 1 - 8: ";
  std::cout << menu;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  std::cout << "\n";
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the location                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
      std::string tmp = map.FindClosestName(input);
      std::cout << "Did you mean " << tmp << " instead of " << input << "? [y/n]";
      getline(std::cin, input);
      if (input == "y") {
        results = map.GetPosition(tmp);
        std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
        PlotPoint(results.first, results.second);
      }
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.CalculateShortestPath_Dijkstra(input1, input2);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Dijkstra*****************************\n";
    std::cout << menu;
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << "\"" << x << "\",";
      std::cout << "\nThe distance of the path is:" << map.CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;

    start = std::chrono::high_resolution_clock::now();
    results = map.CalculateShortestPath_Bellman_Ford(input1, input2);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Bellman_Ford*************************\n";
    std::cout << menu;
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << "\"" << x << "\",";
      std::cout << "\nThe distance of the path is:" << map.CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;

    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : map.data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    for (auto x: locations) std::cout << "\"" << x << "\",";
    std::cout << "\nCalculating ..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.TravellingTrojan_Brute_force(locations);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second, "output0.avi");
    menu = "*************************Results******************************\n";
    std::cout << menu;
    menu = "TravellingTrojan_Brute_force\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << "\"" << x << "\",";
      std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output0.avi.          \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;

    std::cout << "Calculating ..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    results = map.TravellingTrojan_Backtracking(locations);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second, "output0_backtracking.avi");
    menu = "*************************Results******************************\n";
    std::cout << menu;
    menu = "TravellingTrojan_Backtracking\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << "\"" << x << "\",";
      std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output0_backtracking.avi.\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    
    std::cout << "Calculating ..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    results = map.TravellingTrojan_2opt(locations);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second, "output0_2opt.avi");
    menu = "*************************Results******************************\n";
    std::cout << menu;
    menu = "TravellingTrojan_2opt\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << "\"" << x << "\",";
      std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output0_2opt.avi.     \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;

    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Cycle Detection                                          \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    std::cout << "Please input the left bound longitude(between -118.320 and -118.250):";
    getline(std::cin, input);
    std::vector<double> square;
    square.push_back(atof(input.c_str()));

    std::cout << "Please input the right bound longitude(between -118.320 and -118.250):";
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    std::cout << "Please input the upper bound latitude(between 34.000 and 34.040):";
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    std::cout << "Please input the lower bound latitude(between 34.000 and 34.040):";
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));
    auto subgraph = map.GetSubgraph(square);
    PlotPointsandEdges(subgraph, square);
    
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.CycleDetection(subgraph, square);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results == true)
      std::cout << "there exists a cycle in the subgraph " << std::endl;
    else
      std::cout << "there exist no cycle in the subgraph " << std::endl;
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    std::cout << "Please input the locations filename:";;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    std::cout << "Please input the dependencies filename:";;
    std::string dependencies_filename;
    getline(std::cin, dependencies_filename);
    
    // Read location names from CSV file
    std::vector<std::string> location_names;
    if (locations_filename == "") 
      location_names = {"Ralphs", "KFC", "Chick-fil-A"};
    else
      location_names = map.ReadLocationsFromCSVFile(locations_filename);
    
    // Read dependencies from CSV file
    std::vector<std::vector<std::string>> dependencies;
    if (dependencies_filename == "")
      dependencies = {{"Ralphs","Chick-fil-A"}, {"Ralphs","KFC"}, {"Chick-fil-A","KFC"}};
    else
      dependencies = map.ReadDependenciesFromCSVFile(dependencies_filename);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = map.DeliveringTrojan(location_names, dependencies);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "*************************Results******************************\n";
    if (result.size() > 0) {
      std::cout << "Topological Sorting Results:" << std::endl;
      for (auto x : result) std::cout << x << std::endl;
      std::vector<std::string> node_ids;
      for (auto x: result) node_ids.push_back(map.GetID(x));
      PlotPointsOrder(node_ids);
    } else {
      std::cout << "There is no topological sort for the given graph.\n";
    }
    std::cout << "**************************************************************\n";
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    PrintMenu();
    break;
  }
    case '7':
  {
    menu =
        "**************************************************************\n"
        "* 7. Find Nearby                                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    
    menu = "Please input the attribute:";
    std::cout << menu;
    std::string attribute;
    getline(std::cin, attribute);


    menu = "Please input the locations:";
    std::cout << menu;
    std::string origin;
    getline(std::cin, origin);

    menu = "Please input radius r:";
    std::cout << menu;
    getline(std::cin, input);
    int r = std::stoi(input);

    menu = "Please input number k:";
    std::cout << menu;
    getline(std::cin, input);
    int k = std::stoi(input);
    
    auto start = std::chrono::high_resolution_clock::now();
    auto result = map.FindNearby(attribute, origin, r, k);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Find Nearby Results:" << std::endl;
    int cnt = 1;
    for (auto x : result) { 
      std::cout << cnt << " " << map.data[x].name << std::endl;
      cnt++;
    }
    PlotPointsLabel(result, map.GetID(origin));
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count()/1000 << " ms" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '8':
    break;
  default:
  {
    PrintMenu();
    break;
  }
  }
}



/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void MapUI::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  auto result = GetPlotLocation(map.data[id].lat, map.data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void MapUI::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void MapUI::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  auto start = GetPlotLocation(map.data[location_ids[0]].lat, map.data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < int(location_ids.size()); i++) {
    auto start = GetPlotLocation(map.data[location_ids[i - 1]].lat, map.data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(map.data[location_ids[i]].lat, map.data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void MapUI::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(map.data[x].lat, map.data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void MapUI::PlotMap() {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  for (auto& kv : map.data) {
    auto x = kv.first;
    if (map.data.count(x) == 0)
      std::cout << x << std::endl;
    auto result = GetPlotLocation(map.data[x].lat, map.data[x].lon);
    for(auto& y : map.data[x].neighbors) {
      auto start = GetPlotLocation(map.data[x].lat, map.data[x].lon);
      auto end = GetPlotLocation(map.data[y].lat, map.data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              3);
    }
    cv::circle(img, cv::Point(result.first, result.second), 5,
            cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void MapUI::PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  auto upperleft = GetPlotLocation(square[2], square[0]);
  auto lowerright = GetPlotLocation(square[3], square[1]);
  cv::Point pt1(int(upperleft.first), int(upperleft.second));
  cv::Point pt2(int(lowerright.first), int(lowerright.second));
  cv::rectangle(img, pt2, pt1, cv::Scalar(0, 0, 255));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(map.data[x].lat, map.data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for(auto y : map.data[x].neighbors) {
      if (map.inSquare(y, square) == false) continue;
      auto start = GetPlotLocation(map.data[x].lat, map.data[x].lon);
      auto end = GetPlotLocation(map.data[y].lat, map.data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPointsOrder: Given a vector of location ids, draws the points on the map and show the order.
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void MapUI::PlotPointsOrder(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(map.data[x].lat, map.data[x].lon);
    cv::putText(img, map.data[x].name, cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
  }
  // Plot dots and lines
  auto start = GetPlotLocation(map.data[location_ids[0]].lat, map.data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < int(location_ids.size()); i++) {
    auto start = GetPlotLocation(map.data[location_ids[i - 1]].lat, map.data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(map.data[location_ids[i]].lat, map.data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void MapUI::PlotPointsLabel(std::vector<std::string> &location_ids, std::string origin) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  int cnt = 1;
  auto result = GetPlotLocation(map.data[origin].lat, map.data[origin].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 255, 0), cv::FILLED);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(map.data[x].lat, map.data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::putText(img, std::to_string(cnt), cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
    cnt++;
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void MapUI::CreateAnimation(std::vector<std::vector<std::string>> path_progress, std::string filename){
  cv::VideoWriter video("src/lib/" + filename, cv::VideoWriter::fourcc('M','J','P','G'), 2, cv::Size(1280,900));
  cv::Mat img;
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/map.png");
    img = cv::imread(image_path, cv::IMREAD_COLOR);
    // cv::resize(img, img, cv::Size(img.cols, img.rows));
    auto start = GetPlotLocation(map.data[location_ids[0]].lat, map.data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < int(location_ids.size()); i++) {
      auto start = GetPlotLocation(map.data[location_ids[i - 1]].lat, map.data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(map.data[location_ids[i]].lat, map.data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::startWindowThread();
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
  for (int i = 0 ; i < 5; i++)
    video.write(img);
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> MapUI::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(33.9990000, -118.3210000);
  std::pair<double, double> upperRight(34.0410000, -118.2490000);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1280,
                                   (1 - (lat - bottomLeft.first) / h) * 900);
  return result;
}

#ifdef NCURSES
/**
 * DynamicPrintMenu: Create the dynamic menu
 * 
 */
void MapUI::DynamicPrintMenu() {
  UI ui;
  initscr();      // Start curses mode
  start_color();  // Start color
                  // Initialize some color pairs (foreground, background)
  init_pair(1, COLOR_RED, COLOR_BLACK);
  init_pair(2, COLOR_GREEN, COLOR_BLACK);
  init_pair(3, COLOR_YELLOW, COLOR_BLACK);
  init_pair(4, COLOR_BLUE, COLOR_BLACK);

  std::string menu =
      "Torjan Map\n"
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Exit                                                     \n"
      "**************************************************************\n"
      "Please select 1 - 4: ";
  std::string s = menu;
  ui.ScrollLongText(menu);
  char number = getch();
  clear();
  refresh();
  int y=0;
  char input[100];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    y = ui.ScrollLongText(menu);
    menu = "Please input a partial location:";
    y = ui.ScrollLongText(menu,10,y);
    scanw("%s",input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    clear();
    menu = "*************************Results******************************";
    y = ui.ScrollLongText(menu);
    if (results.size() != 0) {
      for (auto x : results) y = ui.ScrollLongText(x,10,y);
    } else {
      ui.ScrollLongText("No matched locations./n",10,y);
    }
    menu = "**************************************************************\n";
    y=ui.ScrollLongText(menu,10,y);
    y=ui.ScrollLongText("Time taken by function: " + std::to_string(duration.count()/1000) + " ms",10,y);
    y=ui.ScrollLongText("Press any keys to continue.",10,y);
    getchar();
    clear();
    DynamicPrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    y = ui.ScrollLongText(menu);
    menu = "Please input a location:";
    y = ui.ScrollLongText(menu,10,y);
    scanw("%s",input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    clear();
    menu = "*************************Results******************************";
    y = ui.ScrollLongText(menu);
    if (results.first != -1) {
      y = ui.ScrollLongText("Latitude: " + std::to_string(results.first), 10, y);
      y = ui.ScrollLongText("Longitude: " + std::to_string(results.second), 10, y);
      PlotPoint(results.first, results.second);
    } else {
      y = ui.ScrollLongText("No matched location.",10,y);
    }
    menu = "**************************************************************\n";
    y=ui.ScrollLongText(menu,10,y);
    y=ui.ScrollLongText("Time taken by function: " + std::to_string(duration.count()/1000) + " ms",10,y);
    y=ui.ScrollLongText("Press any keys to continue.",10,y);
    getchar();
    clear();
    DynamicPrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                    \n"
        "**************************************************************\n";
    y = ui.ScrollLongText(menu);
    menu = "Please input the start location:";
    y = ui.ScrollLongText(menu, 10, y);
    char input1[100];
    scanw("%s",input1);
    menu = "Please input the destination:";
    y = ui.ScrollLongText(menu, 10, y);
    char input2[100];
    scanw("%s",input2);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = map.CalculateShortestPath_Dijkstra(input1, input2);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    clear();
    menu = "*************************Results******************************";
    y = ui.ScrollLongText(menu);
    if (results.size() != 0) {
      // for (auto x : results) y = ui.ScrollLongText(x, 10, y);
      y = ui.ScrollLongText("The distance of the path is:" + std::to_string(map.CalculatePathLength(results)) + " miles", 10, y);
      PlotPath(results);
    } else {
      y = ui.ScrollLongText("No route from the start point to the destination.", 10, y);
    }
    menu = "**************************************************************\n";
    y=ui.ScrollLongText(menu,10,y);
    y=ui.ScrollLongText("Time taken by function: " + std::to_string(duration.count()/1000) + " ms",10,y);
    y=ui.ScrollLongText("Press any keys to continue.",10,y);
    getchar();
    clear();
    DynamicPrintMenu();
    break;
  }
  case '4':
  {
    
    endwin();  // End curses mode
    break;
    // return EXIT_SUCCESS;
  }
  default:
  {
    DynamicPrintMenu();
    break;
  }
  }
}
#endif