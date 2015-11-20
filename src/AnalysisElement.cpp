#include "AnalysisElement.h"

///////////////////////////////////////
AnalysisElement::AnalysisElement(int time, double monitor, double neighbours, double energy)
: m_monitor(monitor)
, m_energy(energy)
, m_neighbours(neighbours)
{}

///////////////////////////////////////
AnalysisElement::~AnalysisElement()
{}
      
///////////////////////////////////////
void AnalysisElement::setEnergy(double val)
{
  m_energy = val;
}

///////////////////////////////////////
void AnalysisElement::setMonitor(double val)
{
  m_monitor = val;
}

///////////////////////////////////////
void AnalysisElement::setNeighbours(double val)
{
  m_neighbours = val;
}