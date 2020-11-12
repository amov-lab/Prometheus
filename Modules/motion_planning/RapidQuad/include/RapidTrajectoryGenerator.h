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

#pragma once
#include "SingleAxisTrajectory.h"
#include "Vec3.h"

namespace RapidQuadrocopterTrajectoryGenerator
{

//! A quadrocopter state interception trajectory.
/*!
 * A quadrocopter state interception trajectory. The trajectory starts at a
 * state defined by the vehicle's position, velocity, and acceleration. The
 * acceleration can be calculated directly from the quadrocopter's attitude
 * and thrust value. The trajectory duration is fixed, and given by the user.
 *
 * The trajectory goal state can include any combination of components from
 * the quadrocopter's position, velocity, and acceleration. The acceleration
 * allows to encode the direction of the quadrocopter's thrust at the end time.
 *
 * The trajectories are generated without consideration for any constraints,
 * and are optimal with respect to the integral of the jerk squared (which is
 * equivalent to an upper bound on a product of the inputs).
 *
 * The trajectories can then be tested with respect to input constraints
 * (thrust/body rates) with an efficient, recursive algorithm. Whether linear
 * combinations of states along the trajectory remain within some bounds can
 * also be tested efficiently.
 *
 * For more information, please see the publication `A computationally efficient motion primitive for quadrocopter trajectory generation', avaible here: http://www.mwm.im/research/publications/
 *
 * NOTE: in the publication, axes are 1-indexed, while here they are
 * zero-indexed.
 */
class RapidTrajectoryGenerator
{
public:

  enum InputFeasibilityResult
  {
    InputFeasible = 0,//!<The trajectory is input feasible
    InputIndeterminable = 1,//!<Cannot determine whether the trajectory is feasible with respect to the inputs
    InputInfeasibleThrustHigh = 2,//!<Trajectory is infeasible, failed max. thrust test first
    InputInfeasibleThrustLow = 3,//!<Trajectory is infeasible, failed min. thrust test first
    InputInfeasibleRates = 4,//!<Trajectory is infeasible, failed max. rates test first
  };

  enum StateFeasibilityResult
  {
    StateFeasible = 0,//!<The trajectory is feasible w.r.t. the test
    StateInfeasible = 1,//!<Trajectory is infeasible
  };

  //! Constructor, user must define initial state, and the direction of gravity.
  RapidTrajectoryGenerator(const Vec3 x0, const Vec3 v0, const Vec3 a0, const Vec3 gravity);

  //set the final state for all axes:
  //! Fix the full position at the end time (see also the per-axis functions).
  void SetGoalPosition(const Vec3 in);
  //! Fix the full velocity at the end time (see also the per-axis functions).
  void SetGoalVelocity(const Vec3 in);
  //! Fix the full acceleration at the end time (see also the per-axis functions).
  void SetGoalAcceleration(const Vec3 in);

  //set final state per axis:
  //! Fix the position at the end time in one axis. If not set, it is left free.
  void SetGoalPositionInAxis(const unsigned axNum, const double in){_axis[axNum].SetGoalPosition(in);};
  //! Fix the velocity at the end time in one axis. If not set, it is left free.
  void SetGoalVelocityInAxis(const unsigned axNum, const double in){_axis[axNum].SetGoalVelocity(in);};
  //! Fix the acceleration at the end time in one axis. If not set, it is left free.
  void SetGoalAccelerationInAxis(const unsigned axNum, const double in){_axis[axNum].SetGoalAcceleration(in);};

  //! Reset the trajectory, clearing any end state constraints.
  void Reset(void);

  /*! Calculate the optimal trajectory of duration `timeToGo`.
   *
   * Calculate the full trajectory, for all the parameters defined so far.
   * @param timeToGo The trajectory duration, in [s].
   */
  void Generate(const double timeToGo);

  /*! Test the trajectory for input feasibility.
   *
   * Test whether the inputs required along the trajectory are within the allowable limits.
   * Note that the test either
   *   - proves feasibility,
   *   - proves infeasibility,
   *   - fails to prove anything ("indeterminate")
   *
   * The user must also specify a minimumTimeSection, which then determines the
   * precision of tests (and thus limit the number of recursion steps).
   *
   * Refer to the paper for a full discussion on these tests.
   *
   * Note that if the result is not feasible, the result is that of the first
   * section which tested infeasible/indeterminate.
   *
   * @param fminAllowed Minimum thrust value inputs allowed [m/s**2].
   * @param fmaxAllowed Maximum thrust value inputs allowed [m/s**2].
   * @param wmaxAllowed Maximum body rates input allowed [rad/s].
   * @param minTimeSection Minimum time section to test during the recursion [s].
   * @return an instance of InputFeasibilityResult.
   */
  InputFeasibilityResult CheckInputFeasibility(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double minTimeSection);

  /*! Test the trajectory for position feasibility.
   *
   * Test whether the position flown along the trajectory is feasible with
   * respect to a user defined boundary (modelled as a plane). The plane is
   * defined by a point lying on the plane, and the normal of the plane.
   *
   * No test is done for degenerate normals (of the form (0,0,0)).
   *
   * @param boundaryPoint The coordinates of any point on the plane defining the boundary.
   * @param boundaryNormal A vector pointing in the allowable direction, away from the boundary.
   * @return An instance of StateFeasibilityResult.
   */
  StateFeasibilityResult CheckPositionFeasibility(Vec3 boundaryPoint, Vec3 boundaryNormal);

  //! Return the jerk along the trajectory at time t
  Vec3 GetJerk(double t) const         {return Vec3(_axis[0].GetJerk(t), _axis[1].GetJerk(t), _axis[2].GetJerk(t));};
  //! Return the acceleration along the trajectory at time t
  Vec3 GetAcceleration(double t) const {return Vec3(_axis[0].GetAcceleration(t), _axis[1].GetAcceleration(t), _axis[2].GetAcceleration(t));};
  //! Return the velocity along the trajectory at time t
  Vec3 GetVelocity(double t) const     {return Vec3(_axis[0].GetVelocity(t), _axis[1].GetVelocity(t), _axis[2].GetVelocity(t));};
  //! Return the position along the trajectory at time t
  Vec3 GetPosition(double t) const     {return Vec3(_axis[0].GetPosition(t), _axis[1].GetPosition(t), _axis[2].GetPosition(t));};

  //! Return the quadrocopter's normal vector along the trajectory at time t
  Vec3   GetNormalVector(double t) const {return (GetAcceleration(t) - _grav).GetUnitVector();};
  //! Return the quadrocopter's thrust input along the trajectory at time t
  double GetThrust(double t) const     {return (GetAcceleration(t) - _grav).GetNorm2();};
  /*! Return the quadrocopter's body rates along the trajectory at time t
   *
   * Returns the required body rates along the trajectory. These are expressed
   * in the world frame (in which the trajectory was planned). To convert them
   * to (p,q,r), this needs to be rotated by the quadrocopter's attitude.
   *
   * The rates are calculated by taking the rates required to rotate the
   * quadrocopter's normal at time `t` to that at time `t+dt`. Therefore, if
   * the trajectory is used as implicit MPC control law, the value dt should
   * correspond to the controller period.
   *
   * @param t Time along the trajectory to be evaluated [s].
   * @param timeStep The timestep size for the finite differencing [s].
   * @return The body rates, expressed in the inertial frame [rad/s]
   */
  Vec3   GetOmega(double t, double timeStep) const;

  //! Return the total cost of the trajectory.
  double GetCost(void) const { return _axis[0].GetCost() + _axis[1].GetCost() + _axis[2].GetCost();};

  //! Return the parameter defining the trajectory.
	double GetAxisParamAlpha(int i)          const{return _axis[i].GetParamAlpha();};
  //! Return the parameter defining the trajectory.
	double GetAxisParamBeta(int i)           const{return _axis[i].GetParamBeta();};
  //! Return the parameter defining the trajectory.
	double GetAxisParamGamma(int i)          const{return _axis[i].GetParamGamma();};

private:
	//! Test a section of the trajectory for input feasibility (recursion).
  InputFeasibilityResult CheckInputFeasibilitySection(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double t1, double t2, double minTimeSection);

  SingleAxisTrajectory _axis[3];//!<The axes along the single trajectories
  Vec3 _grav;//!<gravity in the frame of the trajectory
  double _tf; //!<trajectory end time [s]
};

};//namespace
