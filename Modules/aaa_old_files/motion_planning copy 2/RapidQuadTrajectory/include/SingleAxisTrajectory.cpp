/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SingleAxisTrajectory.h"

#include <math.h>
#include <algorithm>//For min/max
#include <limits>//for max double

using namespace RapidQuadrocopterTrajectoryGenerator;

SingleAxisTrajectory::SingleAxisTrajectory(void)
  :_a(0),_b(0),_g(0),_cost(std::numeric_limits<double>::max())
{
  Reset();
}

/*!
 * Resets the goal state constraints and the cost.
 */
void SingleAxisTrajectory::Reset(void)
{
  _posGoalDefined = _velGoalDefined = _accGoalDefined = false;
  _cost = std::numeric_limits<double>::max();
  _accPeakTimes.initialised = false;
}

/*!
 * Calculate the coefficients that define a trajectory. This solves, in closed form
 * the optimal trajectory for some given set of goal constraints.
 * Before calling this,
 *   - define the initial state
 *   - define the relevant parts of the trajectory goal state.
 *
 * @param Tf the trajectory duration [s]
 * @see SetInitialState()
 * @see SetGoalPosition()
 * @see SetGoalVelocity()
 * @see SetGoalAcceleration()
 */
void SingleAxisTrajectory::GenerateTrajectory(const double Tf)
{
  //define starting position:
  double delta_a = _af - _a0;
  double delta_v = _vf - _v0 - _a0*Tf;
  double delta_p = _pf - _p0 - _v0*Tf - 0.5*_a0*Tf*Tf;

  //powers of the end time:
  const double T2 = Tf*Tf;
  const double T3 = T2*Tf;
  const double T4 = T3*Tf;
  const double T5 = T4*Tf;

  //solve the trajectories, depending on what's constrained:
  if(_posGoalDefined && _velGoalDefined && _accGoalDefined)
  {
    _a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5;
    _b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5;
    _g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5;
  }
  else if(_posGoalDefined && _velGoalDefined)
  {
    _a = (-120*Tf*delta_v + 320*   delta_p)/T5;
    _b = (  72*T2*delta_v - 200*Tf*delta_p)/T5;
    _g = ( -12*T3*delta_v +  40*T2*delta_p)/T5;
  }
  else if(_posGoalDefined && _accGoalDefined)
  {
    _a = (-15*T2*delta_a + 90*   delta_p)/(2*T5);
    _b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5);
    _g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5);
  }
  else if(_velGoalDefined && _accGoalDefined)
  {
    _a = 0;
    _b = ( 6*Tf*delta_a - 12*   delta_v)/T3;
    _g = (-2*T2*delta_a +  6*Tf*delta_v)/T3;
  }
  else if(_posGoalDefined)
  {
    _a =  20*delta_p/T5;
    _b = -20*delta_p/T4;
    _g =  10*delta_p/T3;
  }
  else if(_velGoalDefined)
  {
    _a = 0;
    _b =-3*delta_v/T3;
    _g = 3*delta_v/T2;
  }
  else if(_accGoalDefined)
  {
    _a = 0;
    _b = 0;
    _g = delta_a/Tf;
  }
  else
  {//Nothing to do!
    _a = _b = _g = 0;
  }

  //Calculate the cost:
  _cost =  _g*_g + _b*_g*Tf + _b*_b*T2/3.0 + _a*_g*T2/3.0 + _a*_b*T3/4.0 + _a*_a*T4/20.0;
}

/*!
 * Calculate the extrema of the acceleration trajectory. This is made relatively
 * easy by the fact that the acceleration is a cubic polynomial, so we only need
 * to solve for the roots of a quadratic.
 * @param aMinOut [out] The minimum acceleration in the segment
 * @param aMaxOut [out] The maximum acceleration in the segment
 * @param t1 [in] The start time of the segment
 * @param t2 [in] The end time of the segment
 */
void SingleAxisTrajectory::GetMinMaxAcc(double &aMinOut, double &aMaxOut, double t1, double t2)
{
  if(!_accPeakTimes.initialised)
  {
    //calculate the roots of the polynomial
    if(_a)
    {//solve a quadratic for t
      double det = _b*_b - 2*_g*_a;
      if(det<0)
      {//no real roots
        _accPeakTimes.t[0] = 0;
        _accPeakTimes.t[1] = 0;
      }
      else
      {
        _accPeakTimes.t[0] = (-_b + sqrt(det))/_a;
        _accPeakTimes.t[1] = (-_b - sqrt(det))/_a;
      }
    }
    else
    {//solve linear equation: _g + _b*t == 0:
			if(_b) _accPeakTimes.t[0] = -_g/_b;
			else _accPeakTimes.t[0] = 0;
      _accPeakTimes.t[1] = 0;
    }

    _accPeakTimes.initialised = 1;
  }

	//Evaluate the acceleration at the boundaries of the period:
  aMinOut = std::min(GetAcceleration(t1), GetAcceleration(t2));
  aMaxOut = std::max(GetAcceleration(t1), GetAcceleration(t2));

	//Evaluate at the maximum/minimum times:
  for(int i=0; i<2; i++)
  {
    if(_accPeakTimes.t[i] <= t1) continue;
    if(_accPeakTimes.t[i] >= t2) continue;

    aMinOut = std::min(aMinOut, GetAcceleration(_accPeakTimes.t[i]));
    aMaxOut = std::max(aMaxOut, GetAcceleration(_accPeakTimes.t[i]));
  }
}

/*!
 * Calculate the maximum jerk squared along the trajectory. This is made easy
 * because the jerk is quadratic, so we need to evaluate at most three points.
 * @param t1 [in] The start time of the segment
 * @param t2 [in] The end time of the segment
 * @return the maximum jerk
 */
double SingleAxisTrajectory::GetMaxJerkSquared(double t1, double t2)
{
  double jMaxSqr = std::max(pow(GetJerk(t1),2),pow(GetJerk(t2),2));

  if(_a)
  {
    double tMax = -1;
    tMax = -_b/_a;
    if(tMax>t1 && tMax<t2)
    {
			jMaxSqr = std::max(pow(GetJerk(tMax),2),jMaxSqr);
    }
  }

  return jMaxSqr;
}
