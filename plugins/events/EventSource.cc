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

#include "EventSource.hh"

using namespace gazebo;

// static data initialization
event::EventT<void (std::string, bool)> SimEventConnector::spawnModel;

////////////////////////////////////////////////////////////////////////////////
EventSource::~EventSource()
{
}

////////////////////////////////////////////////////////////////////////////////
EventSource::EventSource(transport::PublisherPtr _pub,
                         const std::string& _type,
                         physics::WorldPtr _world)
  :type(_type), world(_world), active(true), pub(_pub)
{
  GZ_ASSERT(_world, "EventSource world pointer is NULL");
  GZ_ASSERT(_pub, "EventSource pub pointer is NULL");
}

////////////////////////////////////////////////////////////////////////////////
void EventSource::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
void EventSource::Load(const sdf::ElementPtr _sdf)
{
  this->name = _sdf->GetElement("name")->Get<std::string>();

  // active by default, but this can be set in the world file
  if (_sdf->HasElement("active"))
  {
    sdf::ElementPtr activeE = _sdf->GetElement("active");
    this->active = activeE->Get<std::string>() == "true";
  }
}


////////////////////////////////////////////////////////////////////////////////
void EventSource::Emit(const std::string &_data) const
{
  if (this->IsActive())
  {
    // add event name, type and data as strings (data is JSON)
    gazebo::msgs::SimEvent msg;
    msg.set_type(this->type);
    msg.set_name(this->name);
    msg.set_data(_data);
    // add world stats to give context to the event (mostly time & sim state)
    gazebo::msgs::WorldStatistics *worldStatsMsg;
    worldStatsMsg = msg.mutable_world_statistics();
    worldStatsMsg->set_iterations(this->world->Iterations());
    worldStatsMsg->set_paused(this->world->IsPaused());
    msgs::Set(worldStatsMsg->mutable_sim_time(), this->world->SimTime());
    msgs::Set(worldStatsMsg->mutable_real_time(), this->world->RealTime());
    msgs::Set(worldStatsMsg->mutable_pause_time(), this->world->PauseTime());
    // send it on the publisher we got in the ctor
    this->pub->Publish(msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
bool EventSource::IsActive() const
{
  // inactive events do not fire.
  return this->active;
}
