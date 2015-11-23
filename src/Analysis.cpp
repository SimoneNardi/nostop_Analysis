#include "Analysis.h"
#include "AnalysisElement.h"

#include "nav_msgs/OccupancyGrid.h"

///////////////////////////////
Analysis::Analysis() 
: m_node()
, m_lattice()
{
}
 
///////////////////////////////
Analysis::~Analysis()
{
}

///////////////////////////////
void Analysis::Initialize()
{
  // neighbours
  m_neighbours_listner = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/neighbours", 1, &Analysis::NeighboursUpdate, this);
  
  // monitor
  m_monitor_listner = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/monitor", 1, &Analysis::MonitorUpdate, this);
  
  // energy
  m_energy_listner= m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/energy", 1, &Analysis::EnergyUpdate, this);
}

///////////////////////////////
void Analysis::EnergyUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{  
  m_lattice.resize(msg->data.size());
  ros::Time l_time = msg->info.map_load_time;
  for(size_t i = 0; i < msg->data.size(); ++i)
  {
    int8_t l_value = msg->data[i];
    m_lattice[i].setEnergy(l_value);
  }
}

///////////////////////////////
void Analysis::MonitorUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  m_lattice.resize(msg->data.size());
  ros::Time l_time = msg->info.map_load_time;
  for(size_t i = 0; i < msg->data.size(); ++i)
  {
    int8_t l_value = msg->data[i];
    m_lattice[i].setMonitor(l_value);
  }
}

///////////////////////////////
void Analysis::NeighboursUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  m_lattice.resize(msg->data.size());
  ros::Time l_time = msg->info.map_load_time;
  for(size_t i = 0; i < msg->data.size(); ++i)
  {
    int8_t l_value = msg->data[i];
    m_lattice[i].setNeighbours(l_value);
  }
}