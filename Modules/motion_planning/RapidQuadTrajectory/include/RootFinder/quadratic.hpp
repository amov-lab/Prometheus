/*  DYNAMO:- Event driven molecular dynamics simulator 
    http://www.marcusbannerman.co.uk/dynamo
    Copyright (C) 2010  Marcus N Campbell Bannerman <m.bannerman@gmail.com>

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    version 3 as published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cmath>
#include <stdexcept>

namespace magnet {
  namespace math {
    //Solve a quadratic of the form x^2 + B x + C == 0
    inline bool quadraticSolve(const double& B, const double& C,
			       double& root1, double& root2)
    {
      double discriminant = (B * B) - (4 * C);
      
      //Cannot do imaginary numbers, yet
      if (discriminant < 0) return false;
      
      //This avoids a cancellation of errors. See
      //http://en.wikipedia.org/wiki/Quadratic_equation#Floating_point_implementation
      double t = -0.5 * ( B + ((B < 0) ? -1 : 1) * std::sqrt(discriminant));
      
      root1 = t;
			if(t==0) root2 = 0;
      else root2 = C / t;

      return true;
    }


    inline bool quadSolve(const double& C, const double& B, const double& A, 
			  double& root1, double& root2)
    {
      // Contingency: if A = 0, not a quadratic = linear
      if(A == 0)
	{
	  //If B is zero then we have a NaN
	  if(B == 0) return false;
      
	  root1 = -1.0 * C / B;
	  root2 = root1;
	}

      double discriminant = (B * B) - (4 * A * C);
      
      //Cannot do imaginary numbers, yet
      if (discriminant < 0) return false;
      
      //This avoids a cancellation of errors. See
      //http://en.wikipedia.org/wiki/Quadratic_equation#Floating_point_implementation
      double t = -0.5 * ( B + ((B < 0) ? -1 : 1) * std::sqrt(discriminant));
      
      root1 = t / A;
      root2 = C / t;

      return true;
    }

    typedef enum {
      ROOT_SMALLEST_EITHER   =   1,
      ROOT_SMALLEST_POSITIVE =   2,
      ROOT_SMALLEST_NEGATIVE =   4,
      ROOT_LARGEST_EITHER    =   8,
      ROOT_LARGEST_POSITIVE  =  16,
      ROOT_LARGEST_NEGATIVE  =  32
    } rootTypeEnum;

    template<rootTypeEnum rootType>
    inline bool quadSolve(const double& C, const double& B, const double& A, double& ans)
    {
      double root1(0), root2(0);

      if (!quadSolve(C,B,A,root1,root2)) return false;

      switch (rootType)
	{
	case ROOT_SMALLEST_EITHER:
	  ans = (fabs(root1) < fabs(root2)) ? root1 : root2;
	  break;
	case ROOT_LARGEST_EITHER:
	  ans = (fabs(root1) < fabs(root2)) ? root2 : root1;
	case ROOT_LARGEST_NEGATIVE:
	  if (root1 < 0 && root2 < 0)
	    ans = ((root1 < root2) ? root1 : root2);
	  else if (root1 < 0)
	    ans = root1;
	  else if (root2 < 0)
	    ans = root2;
	  else
	    return false;
	  break;
	case ROOT_SMALLEST_NEGATIVE:
	  if (root1 < 0 && root2 < 0)
	    ans = ((root1 < root2) ? root2 : root1);
	  else if (root1 < 0)
	    ans = root1;
	  else if (root2 < 0)
	    ans = root2;
	  else
	    return false;
	  break;
	case ROOT_LARGEST_POSITIVE:
	  if (root1 > 0 && root2 > 0)
	    ans = ((root1 > root2) ? root1 : root2);
	  else if (root1 > 0)
	    ans = root1;
	  else if (root2 > 0)
	    ans = root2;
	  else
	    return false;
	  break;
	case ROOT_SMALLEST_POSITIVE:
	  if (root1 > 0 && root2 > 0)
	    ans = ((root1 > root2) ? root2 : root1);
	  else if (root1 > 0)
	    ans = root1;
	  else if (root2 > 0)
	    ans = root2;
	  else
	    return false;
	  break;
	default:
	  throw std::runtime_error("Unknown root selected");
	  break;
	}
      return true;
    }
  }
}
