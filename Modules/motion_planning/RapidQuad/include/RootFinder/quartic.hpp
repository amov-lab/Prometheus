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

#include "quartic_yacfraid.hpp"
#include "quartic_descartes.hpp"
#include "quartic_neumark.hpp"
#include "quartic_ferrari.hpp"

namespace magnet {
  namespace math {
    namespace detail {
      inline void quarticNewtonRootPolish(const double& a, const double& b, const double& c, const double& d,
					  double& root, size_t iterations)
      {
	for (size_t it = 0; it < iterations; ++it)
	  {
	    double error = (((root + a)*root + b) * root + c) * root + d;
	    double derivative = ((4 * root + 3 * a) * root + 2 * b) * root + c;
	    
	    if ((error == 0) || derivative == 0) return;
	    
	    root -= error / derivative;
	  }
      } 
    }

    //Solves quartics of the form x^4 + a x^3 + b x^2 + c x + d ==0
    inline size_t quarticSolve(const double& a, const double& b, const double& c, const double& d,
			       double& root1, double& root2, double& root3, double& root4)
    {
      static const double maxSqrt = std::sqrt(std::numeric_limits<double>::max());

      if (std::abs(a) > maxSqrt)
	yacfraidQuarticSolve(a,b,c,d,root1,root2,root3,root4);

      if (d == 0)
	{//Solve a cubic with a trivial root of 0
	  root1 = 0;
	  return 1 + cubicSolve(a, b, c, root2, root3, root4);
	}
  
      if ((a == 0) && (c== 0))
	{//We have a biquadratic
	  double quadRoot1,quadRoot2;
	  if (quadSolve(d,b,1, quadRoot1, quadRoot2))
	    {
	      if (quadRoot1 < quadRoot2) std::swap(quadRoot1,quadRoot2);
	  
	      if (quadRoot1 < 0)
		return 0;
	  
	      root1 = std::sqrt(quadRoot1);
	      root2 = -std::sqrt(quadRoot1);
	  
	      if (quadRoot2 < 0)
		return 2;

	      root3 = std::sqrt(quadRoot2);
	      root4 = -std::sqrt(quadRoot2);
	      return 4;
	    }
	  else
	    return 0;
	}
  
      //Now we have to resort to some dodgy formulae!
      size_t k = 0, nr;
      if (a < 0.0) k += 2;
      if (b < 0.0) k += 1;
      if (c < 0.0) k += 8;
      if (d < 0.0) k += 4;
      switch (k)
	{
	case 3 :
	case 9 : 
	  nr = ferrariQuarticSolve(a,b,c,d,root1,root2,root3,root4); 
	  break;
	case 5 :
	  nr = descartesQuarticSolve(a,b,c,d,root1,root2,root3,root4); 
	  break;
	case 15 :
	  //This algorithm is stable if we flip the sign of the roots
	  nr = descartesQuarticSolve(-a,b,-c,d,root1,root2,root3,root4); 
	  root1 *=-1; root2 *=-1; root3 *=-1; root4 *=-1; 
	  break;
	default:
	  nr = neumarkQuarticSolve(a,b,c,d,root1,root2,root3,root4); 
	  //nr = ferrariQuarticSolve(a,b,c,d,root1,root2,root3,root4); 
	  //nr = yacfraidQuarticSolve(a,b,c,d,root1,root2,root3,root4); 
	  break;
	}

      if (nr)   detail::quarticNewtonRootPolish(a, b, c, d, root1, 15);
      if (nr>1) detail::quarticNewtonRootPolish(a, b, c, d, root2, 15);
      if (nr>2) detail::quarticNewtonRootPolish(a, b, c, d, root3, 15);
      if (nr>3) detail::quarticNewtonRootPolish(a, b, c, d, root4, 15);
      
      return nr;  
    }
  }
}
