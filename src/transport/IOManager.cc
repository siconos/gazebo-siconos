/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <iostream>
#include "transport/IOManager.hh"

using namespace gazebo;
using namespace transport;

IOManager::IOManager()
  : work(this->io_service), count(0)
{
  this->thread = boost::thread( boost::bind(&boost::asio::io_service::run, 
                                            &this->io_service) );
}

IOManager::~IOManager()
{
  this->Stop();
}

void IOManager::Stop()
{
  this->io_service.stop();
  this->thread.join();
}

boost::asio::io_service &IOManager::GetIO()
{
  return this->io_service;
}

void IOManager::IncCount()
{
  this->count++;
}

void IOManager::DecCount()
{
  this->count--;
}

unsigned int IOManager::GetCount() const
{
  return this->count;
}
