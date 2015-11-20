#include "Analysis.h"
#include "AnalysisElement.h"

#include "nav_msgs/OccupancyGrid.h"

///////////////////////////////
Analysis::Analysis() 
: m_node()
, m_lattice()
{
  // neighbours
  m_neighbours_listner = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/neighbours", 1, &Analysis::NeighboursUpdate, this);
  
  // monitor
  m_monitor_listner = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/monitor", 1, &Analysis::MonitorUpdate, this);
  
  // energy
  m_energy_listner= m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/energy", 1, &Analysis::EnergyUpdate, this);
   
}
 
///////////////////////////////
Analysis::~Analysis()
{
  
}

///////////////////////////////
void Analysis::EnergyUpdate(const nav_msgs::OccupancyGrid::ConstPtr msg)
{  
  m_lattice.resize(msg->data.size());
  int l_time = msg->info.map_load_time;
  for(int i = 0; i < msg->data.size(); ++i)
  {
  }
}

///////////////////////////////
void Analysis::MonitorUpdate(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
  m_lattice.resize(msg->data.size());
  int l_time = msg->info.map_load_time;
  for(int i = 0; i < msg->data.size(); ++i)
  {
  }
}

///////////////////////////////
void Analysis::NeighboursUpdate(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
  m_lattice.resize(msg->data.size());
  int l_time = msg->info.map_load_time;
  for(int i = 0; i < msg->data.size(); ++i)
  {
  }
}