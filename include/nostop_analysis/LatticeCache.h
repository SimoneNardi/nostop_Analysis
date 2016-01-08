#ifndef LATTICE_CACHE_H
#define LATTICE_CACHE_H

#include <ros/ros.h>

#include <fstream>

#include "Threads.h"

class AnalysisElement;

extern std::ofstream g_outFile;

////////////////////////
class Lattice
{
protected:
    Mutex1 m_mutex;
    
    int m_time;
    int m_updated;
    std::vector<AnalysisElement> m_elems;
    
    double m_potential;
public:
  Lattice(int time = 0);
  
  void setTime(int time_);
  void setEnergy(std::vector<int8_t> const& data);
  void setMonitor(std::vector<int8_t> const& data);
  void setNeighbours(std::vector<int8_t> const& data);
    
  int getTime();
  double getEnergy(int index);
  double getMonitor(int index);
  double getNeighbours(int index);

protected:  
  void computePotential();
  void resetData();
};

////////////////////////
class LatticeCache
{
protected:
  Mutex1 m_mutex;
  std::vector< std::shared_ptr<Lattice> > m_cache;
public:
  LatticeCache(int size_of_cache = 5);
  
  void updateEnergy(int time, std::vector<int8_t> const& data);
  void updateMonitor(int time, std::vector<int8_t> const& data );
  void updateNeighbours(int time, std::vector<int8_t> const& data);
    
  std::shared_ptr<Lattice> getLattice(int time);
};

#endif