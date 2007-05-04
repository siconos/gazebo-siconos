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
/* Desc: Camera Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Camera Interface
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "CameraInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
CameraInterface::CameraInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_camera_alloc();

  // Save frames?
  this->save = cf->ReadInt(section, "save", 0);
  this->frameno = 0;

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
CameraInterface::~CameraInterface()
{
  // Release this interface
  gz_camera_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int CameraInterface::ProcessMessage(MessageQueue *respQueue,
                   player_msghdr_t *hdr, void *data)
{
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void CameraInterface::Update()
{
  size_t size;
  char filename[256];
 
  struct timeval ts;

  gz_camera_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    // Set the image properties
    this->data.width = this->iface->data->width;
    this->data.height = this->iface->data->height;
    this->data.bpp = 24;
    this->data.fdiv = 1;
    this->data.format = PLAYER_CAMERA_FORMAT_RGB888;
    this->data.compression = PLAYER_CAMERA_COMPRESS_RAW;
    this->data.image_count = this->iface->data->image_size;

    // Set the image pixels
    assert((size_t) this->iface->data->image_size < sizeof(this->data.image));

    memcpy(this->data.image, this->iface->data->image, 
        this->iface->data->image_size);

    size = sizeof(this->data) - sizeof(this->data.image) + 
      this->iface->data->image_size;

    // Send data to server
    this->driver->Publish(this->device_addr, NULL, 
                  PLAYER_MSGTYPE_DATA,
                  PLAYER_CAMERA_DATA_STATE,
                  (void*)&this->data, size, &this->datatime);

    // Save frames
    if (this->save)
    {
      //printf("click %d\n", this->frameno);
      snprintf(filename, sizeof(filename), "click-%04d.ppm",this->frameno++);
      this->SaveFrame(filename);
    }

  }

  gz_camera_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void CameraInterface::Subscribe()
{
  // Open the interface
  if (gz_camera_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void CameraInterface::Unsubscribe()
{
  gz_camera_close(this->iface);
}

////////////////////////////////////////////////////////////////////////////////
// Save an image frame
void CameraInterface::SaveFrame(const char *filename)
{
  int i, width, height;
  FILE *file;

  file = fopen(filename, "w+");
  if (!file)
    return;

  width = this->data.width;
  height = this->data.height;

  if (this->data.format == PLAYER_CAMERA_FORMAT_RGB888)
  {
    // Write ppm  
    fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
    for (i = 0; i < height; i++)
      fwrite(this->data.image + i * width * 3, 1, width * 3, file);
  }
  else
  {
    PLAYER_WARN("unsupported format for saving");
  }

  fclose(file);

  return;
}
