/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "SiconosWorld.hh"

SiconosWorld::SiconosWorld()
    : SiconosBodies()
{
    g.zero();
}

void SiconosWorld::init()
{
}

void SiconosWorld::compute()
{
}

void SiconosWorld::SetGravity(double _x, double _y, double _z)
{
    g.setValue(0, _x);
    g.setValue(1, _y);
    g.setValue(2, _z);
}
