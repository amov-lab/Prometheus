/**
 * Rapid trajectory generation for quadrocopters
 *
 *  Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
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

#pragma once

namespace RapidQuadrocopterTrajectoryGenerator
{

//! An axis along one spatial direction
/*!
 * For more information, please refer to Section II of the publication.
 */
class SingleAxisTrajectory
{
public:
  //! Constructor, calls Reset() function.
  SingleAxisTrajectory(void);

  //! Define the trajectory's initial state (position, velocity, acceleration), at time zero
  void SetInitialState(const double pos0, const double vel0, const double acc0){_p0=pos0; _v0=vel0; _a0=acc0; Reset();};

  //! Define the trajectory's final position (if you don't call this, it is left free)
  void SetGoalPosition(const double posf)    {_posGoalDefined = true; _pf = posf;};

  //! Define the trajectory's final velocity (if you don't call this, it is left free)
  void SetGoalVelocity(const double velf)    {_velGoalDefined = true; _vf = velf;};

  //! Define the trajectory's final acceleration (if you don't call this, it is left free)
  void SetGoalAcceleration(const double accf){_accGoalDefined = true; _af = accf;};

  //! Generate the trajectory, from the defined initial state to the defined components of the final state.
  void GenerateTrajectory(const double timeToGo);

  //! Resets the cost, coefficients, and goal state constraints. Does *not* reset the initial state
  void Reset(void);

  //! Returns the jerk at time t
  double GetJerk(double t)         const{return _g  + _b*t  + (1/2.0)*_a*t*t;};

  //! Returns the acceleration at time t
  double GetAcceleration(double t) const{return _a0 + _g*t  + (1/2.0)*_b*t*t  + (1/6.0)*_a*t*t*t;};

  //! Returns the velocity at time t
  double GetVelocity(double t)     const{return _v0 + _a0*t + (1/2.0)*_g*t*t  + (1/6.0)*_b*t*t*t + (1/24.0)*_a*t*t*t*t;};

  //! Returns the position at time t
  double GetPosition(double t)     const{return _p0 + _v0*t + (1/2.0)*_a0*t*t + (1/6.0)*_g*t*t*t + (1/24.0)*_b*t*t*t*t + (1/120.0)*_a*t*t*t*t*t;};

  //! Calculate the extrema of the acceleration trajectory over a section
  void GetMinMaxAcc(double &aMinOut, double &aMaxOut, double t1, double t2);

  //! Calculate the extrema of the jerk squared over a section
  double GetMaxJerkSquared(double t1, double t2);

  //! Get the parameters defining the trajectory
  double GetParamAlpha(void) const {return _a;};
  //! Get the parameters defining the trajectory
  double GetParamBeta (void) const {return _b;};
  //! Get the parameters defining the trajectory
  double GetParamGamma(void) const {return _g;};
  //! Get the parameters defining the trajectory
  double GetInitialAcceleration(void) const {return _a0;};
  //! Get the parameters defining the trajectory
  double GetInitialVelocity(void)     const {return _v0;};
  //! Get the parameters defining the trajectory
  double GetInitialPosition(void)     const {return _p0;};

  //! Get the trajectory cost value
  double GetCost(void) const{return _cost;};

private:
  double _p0, _v0, _a0;//!< The initial state (position, velocity, acceleration)
  double _pf, _vf, _af;//!< The goal state (position, velocity, acceleration)
  bool _posGoalDefined, _velGoalDefined, _accGoalDefined;//!< The components of the goal state defined to be fixed (position, velocity, acceleration)
  double _a, _b, _g;//!< The three coefficients that define the trajectory

  double _cost;//!< The trajectory cost, J

	struct
	{
		double t[2];
		bool initialised;
	} _accPeakTimes;//!< The times at which the acceleration has minimum/maximum
};

};//namespace
