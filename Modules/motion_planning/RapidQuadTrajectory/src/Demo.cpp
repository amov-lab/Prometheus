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


#include <iostream>
using namespace std;

#include "RapidTrajectoryGenerator.h"
using namespace RapidQuadrocopterTrajectoryGenerator;

//Two simple helper function to make testing easier
const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::InputFeasible:             return "Feasible";
  case RapidTrajectoryGenerator::InputIndeterminable:       return "Indeterminable";
  case RapidTrajectoryGenerator::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
  case RapidTrajectoryGenerator::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
  case RapidTrajectoryGenerator::InputInfeasibleRates:      return "InfeasibleRates";
  }
  return "Unknown!";
};

const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::StateFeasible:   return "Feasible";
  case RapidTrajectoryGenerator::StateInfeasible: return "Infeasible";
  }
  return "Unknown!";
};

int main(void)
{
  //Define the trajectory starting state:
  Vec3 pos0 = Vec3(0, 0, 2); //position
  Vec3 vel0 = Vec3(0, 0, 0); //velocity
  Vec3 acc0 = Vec3(0, 0, 0); //acceleration

  //define the goal state:
  Vec3 posf = Vec3(1, 0, 1); //position
  Vec3 velf = Vec3(0, 0, 1); //velocity
  Vec3 accf = Vec3(0, 0, 0); //acceleration

  //define the duration:
  double Tf = 1.3;

  double fmin = 5;//[m/s**2]
  double fmax = 25;//[m/s**2]
  double wmax = 20;//[rad/s]
  double minTimeSec = 0.02;//[s]

  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]

  //Define the state constraints. We'll only check that we don't fly into the floor:
  Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
  Vec3 floorNormal = Vec3(0,0,1);//we want to be in this direction of the boundary

  RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
  traj.SetGoalPosition(posf);
  traj.SetGoalVelocity(velf);
  traj.SetGoalAcceleration(accf);

  // Note: if you'd like to leave some states free, you can encode it like below.
  // Here we would be leaving the velocity in `x` (axis 0) free:
  //
  // traj.SetGoalVelocityInAxis(1,velf[1]);
  // traj.SetGoalVelocityInAxis(2,velf[2]);

  traj.Generate(Tf);

  for(int i = 0; i < 3; i++)
  {
    cout << "Axis #" << i << "\n";
    cout << "\talpha = " << traj.GetAxisParamAlpha(i);
    cout << "\tbeta = "  << traj.GetAxisParamBeta(i);
    cout << "\tgamma = " << traj.GetAxisParamGamma(i);
    cout << "\n";
  }
  cout << "Total cost = " << traj.GetCost() << "\n";
  cout << "Input feasible? " << GetInputFeasibilityResultName(traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec)) << "\n";
  cout << "Position feasible? " << GetStateFeasibilityResultName(traj.CheckPositionFeasibility(floorPos, floorNormal)) << "\n";
  return 0;
}

