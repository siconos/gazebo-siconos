/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Gripper Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Gripper Interface
- PLAYER_GRIPPER_CMD_STATE
*/

/* TODO
- PLAYER_GRIPPER_REQ_GET_GEOM
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "GripperInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
GripperInterface::GripperInterface(player_devaddr_t addr,
                                   GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{

  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_gripper_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
GripperInterface::~GripperInterface()
{
  // Release this interface
  gz_gripper_free(this->iface);
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int GripperInterface::ProcessMessage(QueuePointer &respQueue,
                                     player_msghdr_t *hdr, void *data)
{

//   This code works with Player CVS
#ifdef PLAYER_GRIPPER_CMD_OPEN
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                            PLAYER_GRIPPER_CMD_OPEN, this->device_addr))
  {
    gz_gripper_lock(this->iface, 1);
    this->iface->data->cmd = GAZEBO_GRIPPER_CMD_OPEN;
    gz_gripper_unlock(this->iface);

    return 0;
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRIPPER_CMD_CLOSE, this->device_addr))
  {
    gz_gripper_lock(this->iface, 1);
    this->iface->data->cmd = GAZEBO_GRIPPER_CMD_CLOSE;
    gz_gripper_unlock(this->iface);

    return 0;
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRIPPER_CMD_STOP, this->device_addr))
  {
    gz_gripper_lock(this->iface, 1);
    this->iface->data->cmd = GAZEBO_GRIPPER_CMD_STOP;
    gz_gripper_unlock(this->iface);

    return 0;
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRIPPER_CMD_STORE, this->device_addr))
  {
    gz_gripper_lock(this->iface, 1);
    this->iface->data->cmd = GAZEBO_GRIPPER_CMD_STORE;
    gz_gripper_unlock(this->iface);

    return 0;
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRIPPER_CMD_RETRIEVE, this->device_addr))
  {
    gz_gripper_lock(this->iface, 1);
    this->iface->data->cmd = GAZEBO_GRIPPER_CMD_RETRIEVE;
    gz_gripper_unlock(this->iface);

    return 0;
  }
  // is it a geometry request?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_GRIPPER_REQ_GET_GEOM,
                                 this->device_addr))
  {
    // TODO: implement me

    player_gripper_geom_t pgeom;

    pgeom.pose.px = 0;
    pgeom.pose.py = 0;
    pgeom.pose.pz = 0;
    pgeom.pose.proll = 0;
    pgeom.pose.ppitch = 0;
    pgeom.pose.pyaw = 0;

    pgeom.outer_size.sw = 0;
    pgeom.outer_size.sl = 0;
    pgeom.outer_size.sh = 0;

    pgeom.inner_size.sw = 0;
    pgeom.inner_size.sl = 0;
    pgeom.inner_size.sh = 0;

    pgeom.num_beams = 2;

    this->driver->Publish(this->device_addr, respQueue,
                          PLAYER_MSGTYPE_RESP_ACK,
                          PLAYER_GRIPPER_REQ_GET_GEOM,
                          (void*)&pgeom, sizeof(pgeom), NULL);

    return 0;
  }
#endif

  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void GripperInterface::Update()
{
  player_gripper_data_t data;
  struct timeval ts;

  gz_gripper_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    // break beams are now implemented
    data.beams = 0;

    data.beams |= this->iface->data->grip_limit_reach ? 0x01 : 0x00;
    data.beams |= this->iface->data->lift_limit_reach ? 0x02 : 0x00;
    data.beams |= this->iface->data->outer_beam_obstruct ? 0x04 : 0x00;
    data.beams |= this->iface->data->inner_beam_obstruct ? 0x08 : 0x00;
    data.beams |= this->iface->data->left_paddle_open ? 0x10 : 0x00;
    data.beams |= this->iface->data->right_paddle_open ? 0x20 : 0x00;

    // This works with player cvs.
#ifdef PLAYER_GRIPPER_STATE_OPEN
    // set the proper state
    if (this->iface->data->state == GAZEBO_GRIPPER_STATE_OPEN)
      data.state = PLAYER_GRIPPER_STATE_OPEN;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_CLOSED)
      data.state = PLAYER_GRIPPER_STATE_CLOSED;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_MOVING)
      data.state = PLAYER_GRIPPER_STATE_MOVING;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_ERROR)
      data.state = PLAYER_GRIPPER_STATE_ERROR;
#endif

    this->driver->Publish( this->device_addr, NULL,
                           PLAYER_MSGTYPE_DATA,
                           PLAYER_GRIPPER_DATA_STATE,
                           (void*)&data, sizeof(data), &this->datatime );
  }

  gz_gripper_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void GripperInterface::Subscribe()
{
  // Open the interface
  if (gz_gripper_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void GripperInterface::Unsubscribe()
{
  gz_gripper_close(this->iface);
}
