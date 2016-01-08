#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

#include <sstream>

class AnalysisElement;
class LatticeCache;

class Analysis
{
protected:
  ros::NodeHandle m_node;
  
  ros::Subscriber m_energy_listner;
  ros::Subscriber m_monitor_listner;
  ros::Subscriber m_neighbours_listner;
    
  std::shared_ptr<LatticeCache> m_lattice;
  
public:
  Analysis();
  ~Analysis();
  
  void Initialize();
  
  void NeighboursUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void MonitorUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void EnergyUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

#endif