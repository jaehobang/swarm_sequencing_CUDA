/*
 *  Copyright 2017 Sasanka Nagavalli
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <math_constants.h>
#include <math_functions.h>

__forceinline__ __host__ __device__
float wrapToPi(float v)
{
  float w = copysignf(CUDART_PI_F, v);
  return fmodf(v+w, 2.0f*CUDART_PI_F) - w;
}

__forceinline__ __host__ __device__
float doClamp(float v, float lo, float hi)
{
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

#endif // UTILITY_H
