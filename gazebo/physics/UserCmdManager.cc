/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "gazebo/transport/transport.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldState.hh"

#include "gazebo/physics/UserCmdManagerPrivate.hh"
#include "gazebo/physics/UserCmdManager.hh"

using namespace gazebo;
using namespace physics;


/////////////////////////////////////////////////
UserCmd::UserCmd(const unsigned int _id,
                 physics::WorldPtr _world,
                 const std::string &_description,
                 const msgs::UserCmd::Type &_type)
  : dataPtr(new UserCmdPrivate())
{
  this->dataPtr->id = _id;
  this->dataPtr->world = _world;
  this->dataPtr->description = _description;
  this->dataPtr->type = _type;

  // Record current world state
  this->dataPtr->startState = WorldState(this->dataPtr->world);
}

/////////////////////////////////////////////////
UserCmd::~UserCmd()
{
  this->dataPtr->world.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void UserCmd::Undo()
{
  // Record / override the state for redo
  this->dataPtr->endState = WorldState(this->dataPtr->world);

  // Reset physics states for the whole world
  this->dataPtr->world->ResetPhysicsStates();

  // Set state to the moment the command was executed
  this->dataPtr->world->SetState(this->dataPtr->startState);
}

/////////////////////////////////////////////////
void UserCmd::Redo()
{
  // Reset physics states for the whole world
  this->dataPtr->world->ResetPhysicsStates();

  // Set state to the moment undo was triggered
  this->dataPtr->world->SetState(this->dataPtr->endState);
}

/////////////////////////////////////////////////
unsigned int UserCmd::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
std::string UserCmd::Description() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
msgs::UserCmd::Type UserCmd::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
UserCmdManager::UserCmdManager(const WorldPtr _world)
  : dataPtr(new UserCmdManagerPrivate())
{
  this->dataPtr->world = _world;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->userCmdSub = this->dataPtr->node->Subscribe("~/user_cmd",
      &UserCmdManager::OnUserCmdMsg, this, true);

  this->dataPtr->undoRedoSub = this->dataPtr->node->Subscribe("~/undo_redo",
      &UserCmdManager::OnUndoRedoMsg, this);

  this->dataPtr->userCmdStatsPub =
    this->dataPtr->node->Advertise<msgs::UserCmdStats>("~/user_cmd_stats");

  this->dataPtr->worldControlPub =
      this->dataPtr->node->Advertise<msgs::WorldControl>("~/world_control");

  this->dataPtr->modelModifyPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->lightModifyPub =
      this->dataPtr->node->Advertise<msgs::Light>("~/light/modify");

  this->dataPtr->idCounter = 0;
}

/////////////////////////////////////////////////
UserCmdManager::~UserCmdManager()
{
  this->dataPtr->world.reset();

  // Clean transport
  {
    this->dataPtr->lightModifyPub.reset();
    this->dataPtr->modelModifyPub.reset();
    this->dataPtr->userCmdStatsPub.reset();
    this->dataPtr->worldControlPub.reset();

    this->dataPtr->userCmdSub.reset();
    this->dataPtr->undoRedoSub.reset();

    this->dataPtr->node.reset();
  }

  this->dataPtr->undoCmds.clear();
  this->dataPtr->redoCmds.clear();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void UserCmdManager::OnUserCmdMsg(ConstUserCmdPtr &_msg)
{
  // Generate unique id
  unsigned int id = this->dataPtr->idCounter++;

  // Create command
  UserCmdPtr cmd(new UserCmd(id, this->dataPtr->world, _msg->description(),
      _msg->type()));

  // Forward message after we've saved the current state
  switch (_msg->type())
  {
    case msgs::UserCmd::MOVING:
    {
      for (int i = 0; i < _msg->model_size(); ++i)
        this->dataPtr->modelModifyPub->Publish(_msg->model(i));

      for (int i = 0; i < _msg->light_size(); ++i)
        this->dataPtr->lightModifyPub->Publish(_msg->light(i));

      break;
    }
    case msgs::UserCmd::SCALING:
    {
      for (int i = 0; i < _msg->model_size(); ++i)
        this->dataPtr->modelModifyPub->Publish(_msg->model(i));

      break;
    }
    case msgs::UserCmd::WORLD_CONTROL:
    {
      if (_msg->has_world_control())
      {
        this->dataPtr->worldControlPub->Publish(_msg->world_control());
      }
      else
      {
        gzwarn << "World control command [" << _msg->description() <<
            "] without a world control message. Command won't be executed."
            << std::endl;
      }

      break;
    }
    case msgs::UserCmd::WRENCH:
    {
      // Set publisher
      std::string topicName = "~/";
      topicName += _msg->entity_name() + "/wrench";
      boost::replace_all(topicName, "::", "/");

      auto wrenchPub = this->dataPtr->node->Advertise<msgs::Wrench>(topicName);
      wrenchPub->Publish(_msg->wrench());
      wrenchPub->Fini();

      break;
    }
    default:
    {
      gzwarn << "Unsupported command type [" << _msg->type() << "]" <<
          std::endl;
      break;
    }
  }

  // Add it to undo list
  this->dataPtr->undoCmds.push_back(cmd);

  // Clear redo list
  this->dataPtr->redoCmds.clear();

  // Publish stats
  this->PublishCurrentStats();
}

/////////////////////////////////////////////////
void UserCmdManager::OnUndoRedoMsg(ConstUndoRedoPtr &_msg)
{
  // Undo
  if (_msg->undo())
  {
    if (this->dataPtr->undoCmds.empty())
    {
      gzwarn << "No commands to be undone" << std::endl;
      return;
    }

    // Get the last done command
    UserCmdPtr cmd = this->dataPtr->undoCmds.back();

    // If there's an id, get that command instead
    if (_msg->has_id())
    {
      bool found = false;
      for (auto cmdIt : this->dataPtr->undoCmds)
      {
        if (cmdIt->Id() == _msg->id())
        {
          cmd = cmdIt;
          found = true;
          break;
        }
      }
      if (!found)
      {
        gzerr << "Requested command [" << _msg->id() <<
            "] is not in the undo queue and won't be undone." << std::endl;
        return;
      }
    }

    // Undo all commands up to the desired one
    for (auto cmdIt : boost::adaptors::reverse(this->dataPtr->undoCmds))
    {
      // Undo it
      cmdIt->Undo();

      // Transfer to the redo list
      this->dataPtr->undoCmds.pop_back();
      this->dataPtr->redoCmds.push_back(cmdIt);

      if (cmdIt == cmd)
        break;
    }
  }
  // Redo
  else
  {
    if (this->dataPtr->redoCmds.empty())
    {
      gzwarn << "No commands to be undone" << std::endl;
      return;
    }

    // Get last undone command
    UserCmdPtr cmd = this->dataPtr->redoCmds.back();

    // If there's an id, get that command instead
    if (_msg->has_id())
    {
      bool found = false;
      for (auto cmdIt : this->dataPtr->redoCmds)
      {
        if (cmdIt->Id() == _msg->id())
        {
          cmd = cmdIt;
          found = true;
          break;
        }
      }
      if (!found)
      {
        gzerr << "Requested command [" << _msg->id() <<
            "] is not in the redo queue and won't be redone." << std::endl;
        return;
      }
    }

    // Redo all commands up to the desired one
    for (auto cmdIt : boost::adaptors::reverse(this->dataPtr->redoCmds))
    {
      // Redo it
      cmdIt->Redo();

      // Transfer to the undo list
      this->dataPtr->redoCmds.pop_back();
      this->dataPtr->undoCmds.push_back(cmdIt);

      if (cmdIt == cmd)
        break;
    }
  }

  this->PublishCurrentStats();
}

/////////////////////////////////////////////////
void UserCmdManager::PublishCurrentStats()
{
  msgs::UserCmdStats statsMsg;

  statsMsg.set_undo_cmd_count(this->dataPtr->undoCmds.size());
  statsMsg.set_redo_cmd_count(this->dataPtr->redoCmds.size());

  for (auto cmd : this->dataPtr->undoCmds)
  {
    msgs::UserCmd *msg = statsMsg.add_undo_cmd();
    msg->set_id(cmd->Id());
    msg->set_description(cmd->Description());
    msg->set_type(cmd->Type());
  }

  for (auto cmd : this->dataPtr->redoCmds)
  {
    msgs::UserCmd *msg = statsMsg.add_redo_cmd();
    msg->set_id(cmd->Id());
    msg->set_description(cmd->Description());
    msg->set_type(cmd->Type());
  }

  this->dataPtr->userCmdStatsPub->Publish(statsMsg);
}
