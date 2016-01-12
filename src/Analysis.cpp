#include "Analysis.h"
#include "AnalysisElement.h"
#include "LatticeCache.h"

#include "nav_msgs/OccupancyGrid.h"

///////////////////////////////
Analysis::Analysis() 
: m_node()
, m_lattice(nullptr)
{
  m_lattice = std::make_shared<LatticeCache>();
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
  ros::Time l_time = msg->info.map_load_time;
  m_lattice->updateEnergy(msg->header.seq, msg->data);
}

///////////////////////////////
void Analysis::MonitorUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ros::Time l_time = msg->info.map_load_time;
  m_lattice->updateMonitor(msg->header.seq, msg->data);
}

///////////////////////////////
void Analysis::NeighboursUpdate(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ros::Time l_time = msg->info.map_load_time;
  m_lattice->updateNeighbours(msg->header.seq, msg->data);
}