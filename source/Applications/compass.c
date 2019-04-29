/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 漏 2011-2014  Bill Nesbitt
*/

#include "aq.h"
#include "compass.h"

float compassNormalize(float heading) {
    while (heading < 0.0f)
	heading += 360.0f;
    while (heading >= 360.0f)
	heading -= 360.0f;

    return heading;//比较并正常化，就是将一个角度控制在0-360度的范围之内，如为-50度，转化为310度，380度，转化为20度
}

// calculate the shortest distance in yaw to get from b => a//计算出一个yaw角从a转到b的最小角度? 还是b到a?
float compassDifference(float a, float b) {
    float diff = b - a;

    while (diff > 180.0f)
	diff -= 360.0f;
    while (diff <= -180.0f)
	diff += 360.0f;

    return diff;
}
