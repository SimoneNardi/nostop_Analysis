#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <ros/ros.h>

class Analysis
{
protected:
  ros::Subscriber m_listner;
  
public:
  Analysis();
  ~Analysis();
};

#endif