#include <ros/ros.h>

#include <iostream>
#include "lib/grid_map.h"
#include "lib/world_item.h"
#include "lib/laser_scanner.h"
#include "lib/laser_scan.h"

#include <json/json.h>
#include <fstream>

using namespace std;

bool parseEnvironmentConfig(const std::string& configFilePath, GridMap& grid_map, std::vector<UnicyclePlatform>& robots, std::vector<LaserScanner>& scanners) {
    // Read the JSON file
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        std::cerr << "Failed to open the configuration file: " << configFilePath << std::endl;
        return false;
    }

    Json::Value root;
    configFile >> root;

    // Parse the map configuration
    const Json::Value& mapConfig = root["map"];
    std::string mapFilePath = mapConfig["file"].asString();
    float resolution = mapConfig["resolution"].asFloat();
    

    // Load the grid map from the image file
    grid_map.loadFromImage(mapFilePath.c_str(), resolution);

    // Parse the robot configurations
    const Json::Value& robotConfigs = root["robots"];
    for (const auto& robotConfig : robotConfigs) {
        float tx = robotConfig["position"]["x"].asFloat();
        float ty = robotConfig["position"]["y"].asFloat();
        float alpha = robotConfig["position"]["theta"].asFloat();
        Isometry2f robotPose = fromCoefficients(tx, ty, alpha);

        float maxLinearVelocity = robotConfig["max_linear_velocity"].asFloat();
        float maxAngularVelocity = robotConfig["max_angular_velocity"].asFloat();

        UnicyclePlatform robot(grid_map, robotPose);
        robot.maxLinearVelocity = maxLinearVelocity;
        robot.maxAngularVelocity = maxAngularVelocity;

        robots.push_back(robot);
    }

    // Parse the laser scanner configurations
    const Json::Value& scannerConfigs = root["laser_scanners"];
    for (const auto& scannerConfig : scannerConfigs) {
        float tx = scannerConfig["position"]["x"].asFloat();
        float ty = scannerConfig["position"]["y"].asFloat();
        float alpha = scannerConfig["position"]["theta"].asFloat();
        Isometry2f scannerPose = fromCoefficients(tx, ty, alpha);

        int beamCount = scannerConfig["beam_count"].asInt();
        float maxRange = scannerConfig["max_range"].asFloat();
        float minRange = scannerConfig["min_range"].asFloat();
        float angleMin = scannerConfig["angle_min"].asFloat();
        float angleMax = scannerConfig["angle_max"].asFloat();

        LaserScan scan(minRange, maxRange, angleMin, angleMax, beamCount);
        LaserScanner scanner(scan, grid_map, scannerPose);

        scanners.push_back(scanner);
    }

    return true;
}



Isometry2f fromCoefficients(float tx, float ty, float alpha) {
  Isometry2f iso;
  iso.setIdentity();
  iso.translation()<< tx, ty;
  iso.linear()=Eigen::Rotation2Df(alpha).matrix();
  return iso;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "multi_rs");
  ros::NodeHandle nh;

  GridMap grid_map;
  std::vector<UnicyclePlatform> robots;
  std::vector<LaserScanner> scanners;  

  if (!parseEnvironmentConfig("environment.json", grid_map, robots, scanners)) {
      std::cerr << "Failed to parse the environment configuration." << std::endl;
      return -1;
  }

  World world_object(grid_map);
  WorldItem object_0(world_object, fromCoefficients(5, 0, 0.5));
  Vector2f grid_middle(grid_map.cols/2, grid_map.rows/2);
  Vector2f world_middle = grid_map.grid2world(grid_middle);
  UnicyclePlatform robot(world_object, fromCoefficients(world_middle.x(), world_middle.y(), -0.5));
  robot.radius=1;

  LaserScan scan;
  LaserScanner scanner(scan, robot, fromCoefficients(3, 0, -0));
  scanner.radius = 0.5;

  float dt=0.1;
  Canvas canvas;
  while (true) {
    world_object.tick(dt);
    world_object.draw(canvas);
    int ret = showCanvas(canvas, dt*100);
    if (ret>0)
      std::cerr << "Key pressed: " << ret << std::endl;

    switch(ret) {
    case 81: //left;
      robot.rv+=0.1;
      break;
    case 82: //up;
      robot.tv+=0.1;
      break;
    case 83: //right;
      robot.rv-=0.1;
      break;
    case 84: //down;
      robot.tv-=0.1;
      break;
    case 32: // space
      robot.rv=0;
      robot.tv=0;
      break;
    default:;
    }
    
  }
}
