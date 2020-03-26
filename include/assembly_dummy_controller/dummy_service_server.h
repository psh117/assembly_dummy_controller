#pragma once

#include <ros/ros.h>

template<typename T>
bool dummyServiceFunction(typename T::Request &req, typename T::Response &res)
{
  return true;
}
