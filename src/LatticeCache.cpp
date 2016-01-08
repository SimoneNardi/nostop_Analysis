#include "LatticeCache.h"
#include "AnalysisElement.h"
//#include <boost/graph/graph_concepts.hpp>

//////////////////////////////
Lattice::Lattice(int time)
: m_updated(0)
, m_time(time)
, m_elems()
{}

//////////////////////////////
void Lattice::setTime(int time)
{
  resetData();
  m_time = time;
}

//////////////////////////////
void Lattice::resetData()
{
  m_updated = 0;
  m_elems.clear();
}

//////////////////////////////
void Lattice::setEnergy(std::vector<int8_t> const& data)
{  
  if (m_updated == 0)
    m_elems.resize(data.size());
  
  for(size_t i = 0; i < data.size(); ++i)
  {
    int8_t l_value = data[i];
    m_elems[i].setEnergy(l_value);
  }
  
  m_updated++;
  
  if (m_updated == 3)
    computePotential();
  
  ROS_INFO("%d, Energy map updated.", m_time);
}

//////////////////////////////
void Lattice::setMonitor(std::vector<int8_t> const& data)
{  
  if (m_updated == 0)
    m_elems.resize(data.size());
  
  for(size_t i = 0; i < data.size(); ++i)
  {
    int8_t l_value = data[i];
    m_elems[i].setEnergy(l_value);
  }
  
  m_updated++;
  
  if (m_updated == 3)
    computePotential();
  
  ROS_INFO("%d, Monitor map updated.", m_time);
}

//////////////////////////////
void Lattice::setNeighbours(std::vector<int8_t> const& data)
{  
  if (m_updated == 0)
    m_elems.resize(data.size());
  
  for(size_t i = 0; i < data.size(); ++i)
  {
    int8_t l_value = data[i];
    m_elems[i].setEnergy(l_value);
  }
  
  m_updated++;
  
  if (m_updated == 3)
    computePotential();
  
  ROS_INFO("%d, Neighbours map updated.", m_time);
}

//////////////////////////////
int Lattice::getTime()
{
  return m_time;
}

//////////////////////////////
double Lattice::getEnergy(int index)
{
  return m_elems[index].getEnergy();
}

//////////////////////////////
double Lattice::getMonitor(int index)
{
  return m_elems[index].getMonitor();
}

//////////////////////////////
double Lattice::getNeighbours(int index)
{
  return m_elems[index].getNeighbours();
}

//////////////////////////////
void Lattice::computePotential()
{
  m_potential = 0;
  for(size_t i = 0; i < m_elems.size(); ++i)
  {
    double l_neighbours = m_elems[i].getNeighbours();
    double l_monitor = m_elems[i].getMonitor();
    
    m_potential += l_monitor * log(l_neighbours);
  }
  
  g_outFile << m_time << " " << m_potential << std::endl;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

std::ofstream g_outFile;

/////////////////////////////////////////////////
LatticeCache::LatticeCache(int size_of_cache)
{
  m_cache.resize(size_of_cache);
  
  g_outFile.open("potential.txt");//, std::ios_base::app);
}

/////////////////////////////////////////////////
std::shared_ptr<Lattice> LatticeCache::getLattice(int time)
{
  int l_mintime = -1;
  int l_minindex = -1;
  for(size_t i = m_cache.size()-1; i >= 0; --i)
  {
    std::shared_ptr<Lattice> l_lattice = m_cache[i];
    
    int l_time = l_lattice->getTime();
    if (l_time == time)
    {
      return l_lattice;
    }
    
    if(l_mintime > l_time)
    {
      l_mintime = l_time;
      l_minindex = i;
    }
  }
  
  m_cache.erase( m_cache.begin() + l_minindex );
  std::shared_ptr<Lattice> l_lattice = std::make_shared<Lattice>(time);
  m_cache.push_back( l_lattice );
  
  return l_lattice;
}

/////////////////////////////////////////////////
void LatticeCache::updateEnergy(int time, std::vector<int8_t> const& data)
{
  std::shared_ptr<Lattice> l_lattice = this->getLattice(time);
  l_lattice->setEnergy(data);
}
 
/////////////////////////////////////////////////
void LatticeCache::updateMonitor(int time, std::vector<int8_t> const& data )
{
   std::shared_ptr<Lattice> l_lattice = this->getLattice(time);
   l_lattice->setMonitor(data);
}

/////////////////////////////////////////////////
void LatticeCache::updateNeighbours(int time, std::vector<int8_t> const& data)
{
   std::shared_ptr<Lattice> l_lattice = this->getLattice(time);
   l_lattice->setNeighbours(data);
}
