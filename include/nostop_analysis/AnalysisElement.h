#ifndef ANALYSIS_ELEMENT_H
#define ANALYSIS_ELEMENT_H

#include <ros/ros.h>

class AnalysisElement;
{
  int m_time;
  double m_monitor;
  double m_neighbours;
  double m_energy;
    
  public:
    AnalysisElement(int time = 0, double monitor = 0., double neighbours = 0., double energy = 0.);
    ~AnalysisElement();
    
    void setEnergy(double val);
    void setMonitor(double val);
    void setNeighbours(double val);
};

#endif