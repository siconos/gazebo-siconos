/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <cmath>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "gazebo/math/SignalStatsPrivate.hh"
#include "gazebo/math/SignalStats.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
SignalStatistic::SignalStatistic()
  : dataPtr(new SignalStatisticPrivate)
{
  this->dataPtr->data = 0.0;
  this->dataPtr->count = 0;
}

//////////////////////////////////////////////////
SignalStatistic::~SignalStatistic()
{
}

//////////////////////////////////////////////////
SignalStatistic::SignalStatistic(const SignalStatistic &_ss)
  : dataPtr(_ss.dataPtr->Clone())
{
}

//////////////////////////////////////////////////
size_t SignalStatistic::Count() const
{
  return this->dataPtr->count;
}

//////////////////////////////////////////////////
void SignalStatistic::Reset()
{
  this->dataPtr->data = 0;
  this->dataPtr->count = 0;
}

//////////////////////////////////////////////////
double SignalMean::Value() const
{
  if (this->dataPtr->count == 0)
  {
    return 0;
  }
  return this->dataPtr->data / this->dataPtr->count;
}

//////////////////////////////////////////////////
std::string SignalMean::ShortName() const
{
  return "mean";
}

//////////////////////////////////////////////////
void SignalMean::InsertData(const double _data)
{
  this->dataPtr->data += _data;
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalRootMeanSquare::Value() const
{
  if (this->dataPtr->count == 0)
  {
    return 0;
  }
  return sqrt(this->dataPtr->data / this->dataPtr->count);
}

//////////////////////////////////////////////////
std::string SignalRootMeanSquare::ShortName() const
{
  return "rms";
}

//////////////////////////////////////////////////
void SignalRootMeanSquare::InsertData(const double _data)
{
  this->dataPtr->data += _data * _data;
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalMaxAbsoluteValue::Value() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
std::string SignalMaxAbsoluteValue::ShortName() const
{
  return "maxAbs";
}

//////////////////////////////////////////////////
void SignalMaxAbsoluteValue::InsertData(const double _data)
{
  double absData = std::abs(_data);
  if (absData > this->dataPtr->data)
  {
    this->dataPtr->data = absData;
  }
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
SignalStats::SignalStats()
  : dataPtr(new SignalStatsPrivate)
{
}

//////////////////////////////////////////////////
SignalStats::SignalStats(const ignition::math::SignalStats &_s)
  : dataPtr(new SignalStatsPrivate)
{
  for (std::map<std::string, double>::const_iterator iter = _s.Map().begin();
       iter != _s.Map().end(); ++iter)
  {
    this->InsertStatistic(iter->first);
    this->InsertData(iter->second);
  }
}

//////////////////////////////////////////////////
SignalStats::~SignalStats()
{
}

//////////////////////////////////////////////////
SignalStats::SignalStats(const SignalStats &_ss)
  : dataPtr(_ss.dataPtr->Clone())
{
}

//////////////////////////////////////////////////
size_t SignalStats::Count() const
{
  if (this->dataPtr->stats.empty())
    return 0;

  return this->dataPtr->stats.front()->Count();
}

//////////////////////////////////////////////////
std::map<std::string, double> SignalStats::Map() const
{
  std::map<std::string, double> map;
  for (auto const &statistic : this->dataPtr->stats)
  {
    map[statistic->ShortName()] = statistic->Value();
  }
  return map;
}

//////////////////////////////////////////////////
void SignalStats::InsertData(const double _data)
{
  for (auto &statistic : this->dataPtr->stats)
  {
    statistic->InsertData(_data);
  }
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistic(const std::string &_name)
{
  // Check if the statistic is already inserted
  {
    std::map<std::string, double> map = this->Map();
    if (map.find(_name) != map.end())
    {
      std::cerr << "Unable to InsertStatistic ["
                << _name
                << "] since it has already been inserted."
                << std::endl;
      return false;
    }
  }

  SignalStatisticPtr stat;
  if (_name == "maxAbs")
  {
    stat.reset(new SignalMaxAbsoluteValue());
    this->dataPtr->stats.push_back(stat);
  }
  else if (_name == "mean")
  {
    stat.reset(new SignalMean());
    this->dataPtr->stats.push_back(stat);
  }
  else if (_name == "rms")
  {
    stat.reset(new SignalRootMeanSquare());
    this->dataPtr->stats.push_back(stat);
  }
  else
  {
    // Unrecognized name string
    std::cerr << "Unable to InsertStatistic ["
              << _name
              << "] since it is an unrecognized name."
              << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistics(const std::string &_names)
{
  if (_names.empty())
  {
    std::cerr << "Unable to InsertStatistics "
              << "since no names were supplied."
              << std::endl;
    return false;
  }

  bool result = true;
  std::vector<std::string> names;
  boost::split(names, _names, boost::is_any_of(","));
  for (auto &statistic : names)
  {
    result = result && this->InsertStatistic(statistic);
  }
  return result;
}

//////////////////////////////////////////////////
void SignalStats::Reset()
{
  for (auto &statistic : this->dataPtr->stats)
  {
    statistic->Reset();
  }
}

//////////////////////////////////////////////////
ignition::math::SignalStats SignalStats::Ign() const
{
  ignition::math::SignalStats result;

  for (auto const &statistic : this->dataPtr->stats)
  {
    result.InsertStatistic(statistic->ShortName());
    result.InsertData(statistic->Value());
  }

  return result;
}

//////////////////////////////////////////////////
SignalStats &SignalStats::operator=(const ignition::math::SignalStats &_s)
{
  std::map<std::string, double> data = _s.Map();

  for (std::map<std::string, double>::const_iterator iter =  data.begin();
       iter != data.end(); ++iter)
  {
    this->InsertStatistic(iter->first);
    this->InsertData(iter->second);
  }

  return *this;
}
