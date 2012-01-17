/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Desc: Color class
 * Author: Nate Koenig
 * Date: 08 May 2009
 */

#include <algorithm>
#include <math.h>

#include "common/Console.hh"
#include "common/Color.hh"

using namespace gazebo;
using namespace common;

const Color Color::White = Color(1, 1, 1, 1);
const Color Color::Black = Color(0, 0, 0, 1);
const Color Color::Red = Color(1, 0, 0, 1);
const Color Color::Green = Color(0, 1, 0, 1);
const Color Color::Blue = Color(0, 0, 1, 1);

//////////////////////////////////////////////////
// Constructor
  Color::Color()
: r(0), g(0), b(0), a(0)
{
  this->Clamp();
}

//////////////////////////////////////////////////
// Constructor
  Color::Color(float _r, float _g, float _b, const float _a)
: r(_r), g(_g), b(_b), a(_a)
{
  this->Clamp();
}

//////////////////////////////////////////////////
// Copy Constructor
  Color::Color(const Color &_pt)
: r(_pt.r), g(_pt.g), b(_pt.b), a(_pt.a)
{
  this->Clamp();
}

//////////////////////////////////////////////////
// Destructor
Color::~Color()
{
}

//////////////////////////////////////////////////
// Reset the vector
void Color::Reset()
{
  this->r = this->g = this->b = this->a = 0;
}

//////////////////////////////////////////////////
// Set the contents of the vector
void Color::Set(float _r, float _g, float _b, float _a)
{
  this->r = _r;
  this->g = _g;
  this->b = _b;
  this->a = _a;

  this->Clamp();
}

//////////////////////////////////////////////////
/// Set a color based on HSV values
void Color::SetFromHSV(float _h, float _s, float _v)
{
  int i;
  float f, p , q, t;

  _h = (int)(_h) % 360;

  if (_s == 0)
  {
    //acromatic (grey)
    this->r = this->g = this->b = _v;
    return;
  }

  _h /= 60; // sector 0 - 5
  i = (int)floor(_h);

  f = _h - i;

  p = _v * (1-_s);
  q = _v * (1 - _s * f);
  t = _v * (1 - _s * (1-f));

  switch(i)
  {
    case 0:
      this->r = _v;
      this->g = t;
      this->b = p;
      break;
    case 1:
      this->r = q;
      this->g = _v;
      this->b = p;
      break;
    case 2:
      this->r = p;
      this->g = _v;
      this->b = t;
      break;
    case 3:
      this->r = p;
      this->g = q;
      this->b = _v;
      break;
    case 4:
      this->r = t;
      this->g = p;
      this->b = _v;
      break;
    case 5:
      this->r = _v;
      this->g = p;
      this->b = q;
      break;
  }

  this->Clamp();
}

//////////////////////////////////////////////////
/// Get the color in HSV colorspace
math::Vector3 Color::GetAsHSV() const
{
  math::Vector3 hsv;
  float x, v, f, i;

  x = std::min(this->r, std::min(this->g, this->b));
  v = std::max(this->r, std::max(this->g, this->b));

  if(v == x)
  {
    gzerr << "rgb to hsv undefined\n";
    return hsv;
  }

  if (r == x)
    f = this->g - this->b;
  else if (this->g == x)
    f = this->b - this->r;
  else
    f = this->r - this->g;

  if (this->r == x)
    i = 3;
  else if (this->g == x)
    i = 5;
  else
    i = 1;

  hsv.x = i - f /(v - x);
  hsv.y = (v - x)/v;
  hsv.z = v;

  return hsv;
}


//////////////////////////////////////////////////
/// Get the color in YUV colorspace
math::Vector3 Color::GetAsYUV() const
{
  math::Vector3 yuv;

  yuv.x = 0.299*this->r + 0.587*this->g + 0.114*this->b;
  yuv.y = -0.1679*this->r - 0.332*this->g + 0.5*this->b + 0.5;
  yuv.z = 0.5*this->r - 0.4189*this->g - 0.08105*this->b + 0.5;

  yuv.x = yuv.x < 0 ? 0: yuv.x;
  yuv.x = yuv.x > 255 ? 255.0: yuv.x;

  yuv.y = yuv.y < 0 ? 0: yuv.y;
  yuv.y = yuv.y > 255 ? 255.0: yuv.y;

  yuv.z = yuv.z < 0 ? 0: yuv.z;
  yuv.z = yuv.z > 255 ? 255.0: yuv.z;



  /*if (yuv.x > 255)
    yuv.x = 255;
    if (yuv.x < 0)
    yuv.x = 0;

    if (yuv.y > 255)
    yuv.y = 255;
    if (yuv.y < 0)
    yuv.y = 0;

    if (yuv.z > 255)
    yuv.z = 255;
    if (yuv.z < 0)
    yuv.z = 0;
    */

  return yuv;
}

//////////////////////////////////////////////////
// Set from yuv
void Color::SetFromYUV(float _y, float _u, float _v)
{
  this->r = _y + 1.140*_v;
  this->g = _y - 0.395*_u - 0.581*_v;
  this->b = _y + 2.032*_u;
  this->Clamp();
}

//////////////////////////////////////////////////
// Array index operator
float Color::operator[](unsigned int index)
{
  switch(index)
  {
    case 0: return this->R();
    case 1: return this->G();
    case 2: return this->B();
    case 3: return this->A();
    default: gzerr << "Invalid color index[" << index << "]\n";
  }

  return 0;
}

//////////////////////////////////////////////////
// Get the red color
float Color::R() const
{
  return this->r;
}

//////////////////////////////////////////////////
// Get the green color
float Color::G() const
{
  return this->g;
}

//////////////////////////////////////////////////
// Get the blue color
float Color::B() const
{
  return this->b;
}

//////////////////////////////////////////////////
// Get the alpha color
float Color::A() const
{
  return this->a;
}

//////////////////////////////////////////////////
/// Set the red color
void Color::R(float _v)
{
  this->r = _v;
}

//////////////////////////////////////////////////
/// Set the green color
void Color::G(float _v)
{
  this->g = _v;
}

//////////////////////////////////////////////////
/// Set the blue color
void Color::B(float _v)
{
  this->b = _v;
}

//////////////////////////////////////////////////
/// Set the alpha color
void Color::A(float _v)
{
  this->a = _v;
}

//////////////////////////////////////////////////
// Equals operator
const Color &Color::operator =(const Color &pt)
{
  this->r = pt.r;
  this->g = pt.g;
  this->b = pt.b;
  this->a = pt.a;

  return *this;
}

//////////////////////////////////////////////////
// Addition operator
Color Color::operator+(const Color &pt) const
{
  return Color(this->r + pt.r, this->g + pt.g, this->b + pt.b, this->a + pt.a);
}

Color Color::operator+(const float &v) const
{
  return Color(this->r + v, this->g + v, this->b + v, this->a + v);
}

const Color &Color::operator+=(const Color &pt)
{
  this->r += pt.r;
  this->g += pt.g;
  this->b += pt.b;
  this->a += pt.a;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
// Subtraction operators
Color Color::operator-(const Color &pt) const
{
  return Color(this->r - pt.r, this->g - pt.g, this->b - pt.b, this->a - pt.a);
}

Color Color::operator-(const float &v) const
{
  return Color(this->r - v, this->g - v, this->b - v, this->a - v);
}

const Color &Color::operator-=(const Color &pt)
{
  this->r -= pt.r;
  this->g -= pt.g;
  this->b -= pt.b;
  this->a -= pt.a;

  this->Clamp();

  return *this;
}


//////////////////////////////////////////////////
// Division operators
const Color Color::operator/(const float &i) const
{
  return Color(this->r / i, this->g / i, this->b / i, this->a / i);
}

const Color Color::operator/(const Color &pt) const
{
  return Color(this->r / pt.r, this->g / pt.g, this->b / pt.b, this->a / pt.a);
}

const Color &Color::operator/=(const Color &pt)
{
  this->r /= pt.r;
  this->g /= pt.g;
  this->b /= pt.b;
  this->a /= pt.a;

  this->Clamp();

  return *this;
}


//////////////////////////////////////////////////
// Mulitplication operators
const Color Color::operator*(const float &i) const
{
  return Color(this->r * i, this->g * i, this->b * i, this->a * i);
}

const Color Color::operator*(const Color &pt) const
{
  return Color(this->r * pt.r, this->g * pt.g, this->b * pt.b, this->a * pt.a);
}

const Color &Color::operator*=(const Color &pt)
{
  this->r *= pt.r;
  this->g *= pt.g;
  this->b *= pt.b;
  this->a *= pt.a;

  this->Clamp();

  return *this;
}


//////////////////////////////////////////////////
// Equality operator
bool Color::operator ==(const Color &pt) const
{
  return this->r == pt.r && this->g == pt.g && this->b == pt.b &&
         this->a == pt.a;
}

//////////////////////////////////////////////////
// Inequality operator
bool Color::operator!=(const Color &pt) const
{
  return !(*this == pt);
}

//////////////////////////////////////////////////
/// Clamp the color values
void Color::Clamp()
{
  this->r = this->r < 0 ? 0: this->r;
  this->r = this->r > 1 ? this->r/255.0: this->r;

  this->g = this->g < 0 ? 0: this->g;
  this->g = this->g > 1 ? this->g/255.0: this->g;

  this->b = this->b < 0 ? 0: this->b;
  this->b = this->b > 1 ? this->b/255.0: this->b;
}



