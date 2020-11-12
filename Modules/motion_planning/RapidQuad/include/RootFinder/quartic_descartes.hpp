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

#include "cubic.hpp"

namespace magnet {
  namespace math {
    inline size_t 
    descartesQuarticSolve(const double& a, const double& b, const double& c, const double& d, 
			  double& root1, double& root2, double& root3, double& root4)
    {
      double rts[4];
      double worst3[3];
      double qrts[4][3];        /* quartic roots for each cubic root */

      if (d == 0.0)
	{
	  root1 = 0.0;
	  return cubicSolve(a,b,c,root2,root3,root4) + 1;
	}

      int j, n4[4];
      double v1[4],v2[4],v3[4];
      double k,y;
      double p,q,r;
      double e0,e1,e2;
      double g,h;
      double asq;
      double ainv4;
      double e1invk;

      asq = a*a;
      e2 = b - asq * (3.0/8.0);
      e1 = c + a*(asq*0.125 - b*0.5);
      e0 = d + asq*(b*0.0625 - asq*(3.0/256.0)) - a*c*0.25;

      p = 2.0*e2;
      q = e2*e2 - 4.0*e0;
      r = -e1*e1;

      size_t n3 = cubicSolve(p,q,r,v3[0],v3[1],v3[2]);
      for (size_t j3 = 0; j3 < n3; ++j3)
	{
	  y = v3[j3];
	  if (y <= 0.0)
	    n4[j3] = 0;
	  else
	    {
	      k = std::sqrt(y);
	      ainv4 = a*0.25;
	      e1invk = e1/k;
	      g = (y + e2 + e1invk)*0.5;
	      h = (y + e2 - e1invk)*0.5 ;
	      bool n1 = quadSolve( g, -k, 1.0, v1[0], v1[1]);
	      bool n2 = quadSolve( h, k, 1.0, v2[0], v2[1]);
	      qrts[0][j3] = v1[0] - ainv4;
	      qrts[1][j3] = v1[1] - ainv4;
	      qrts[n1*2][j3] = v2[0] - ainv4;
	      qrts[n1*2+1][j3] = v2[1] - ainv4;
	      n4[j3]= n1*2 + n2*2;
	    } /* y>=0 */
	  for (j = 0; j < n4[j3]; ++j)
	    rts[j] = qrts[j][j3];
	  worst3[j3] = quarticError(a, b, c, d, rts, n4[j3]);
	} /* j3 loop */
      size_t j3 = 0;
      if (n3 != 1)
	{
	  if ((n4[1] > n4[j3]) ||
	      ((worst3[1] < worst3[j3] ) && (n4[1] == n4[j3]))) j3 = 1;
	  if ((n4[2] > n4[j3]) ||
	      ((worst3[2] < worst3[j3] ) && (n4[2] == n4[j3]))) j3 = 2;
	}

      root1 = qrts[0][j3];
      root2 = qrts[1][j3];
      root3 = qrts[2][j3];
      root4 = qrts[3][j3];

      return (n4[j3]);
    }
  }
}
