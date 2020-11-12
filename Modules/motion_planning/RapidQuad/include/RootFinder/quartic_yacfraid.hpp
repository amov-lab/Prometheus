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
#include "quartic_error.hpp"

namespace magnet {
  namespace math {
    inline size_t 
    yacfraidQuarticSolve(const double& a, const double& b, const double& c, const double& d,
			 double& root1, double& root2, double& root3, double& root4)
    {
      int j;
      double y;
      double v3[4];
      double det0,det1,det2,det3;
      double det0rt,det1rt,det2rt,det3rt;
      double e,f,g,h,k;
      double fsq,gsq,hsq,invk;
      double P,Q,R,U;

      double rts[4];
      double worst3[3];
      double qrts[4][3];        /* quartic roots for each cubic root */

      if (d == 0.0)
	{
	  root1 = 0.0;
	  return cubicSolve(a,b,c,root2,root3,root4) + 1;
	}

      double asq = a * a;
      double acu = a * asq;
      double b4 = b * 4.0;
      size_t n3 = 0;
      int n4[4];

      P = asq * b - b4 * b + 2.0 * a * c + 16.0 * d ;
      Q = asq * c - b4 * c + 8.0 * a * d;
      R = asq * d - c * c;
      U = acu - b4 * a + 8.0 * c;
      n4[0] = 0;


      asq = a*a;
      acu = a*asq;
      b4 = b*4.0;
      n3 = 0;

      P = asq*b - b4*b + 2.0*a*c + 16.0*d ;
      Q = asq*c - b4*c + 8.0*a*d;
      R = asq*d - c*c ;
      U = acu - b4*a + 8.0*c;
      n4[0] = 0;
      if (U == 0.0)
	{
	  if (P == 0.0)
	    {
	      det0 = 3.0*asq - 8.0*b;
	      if (det0 < 0.0)
		goto done;
	      det0rt = sqrt(det0);
	      qrts[0][0] = (-a + det0rt)*0.25;
	      qrts[1][0] = qrts[0][0];
	      qrts[2][0] = (-a - det0rt)*0.25;
	      qrts[3][0] = qrts[2][0];
	      n4[0] = 4;
	      goto done;
	    } /* P=0 */
	  else
	    {
	      det1 = asq*asq - 8.0*asq*b + 16.0*b*b - 64.0*d;
	      if (det1 < 0.0)
		goto done;;
	      n4[0] = 0;
	      det1rt =  sqrt(det1);
	      det2 = 3.0*asq - 8.0*b + 2.0*det1rt;
	      if (det2 >= 0.0)
		{
		  det2rt = sqrt(det2);
		  qrts[0][0] = (-a + det2rt)*0.25;
		  qrts[1][0] = (-a - det2rt)*0.25;
		  n4[0] = 2;
		}
	      det3 = 3.0*asq - 8.0*b - 2.0*det1rt;
	      if (det3 >= 0.0)
		{
		  det3rt = sqrt(det3);
		  qrts[n4[0]++][0] = (-a + det3rt)*0.25;
		  qrts[n4[0]++][0] = (-a - det3rt)*0.25;
		}
	      goto done;
	    } /* P<>0 */
	}

      n3 = cubicSolve(P/U,Q/U,R/U,v3[0], v3[1], v3[2]);
      for (size_t j3 = 0; j3 < n3; ++j3)
	{
	  y = v3[j3];
	  j = 0;
	  k = a + 4.0*y;
	  if (k == 0.0)
	    goto donej3;
	  invk = 1.0/k;
	  e = (acu - 4.0*c - 2.0*a*b + (6.0*asq - 16.0*b)*y)*invk;
	  fsq = (acu + 8.0*c - 4.0*a*b)*invk;
	  if (fsq < 0.0)
	    goto donej3;
	  f = sqrt(fsq);
	  gsq = 2.0*(e + f*k);
	  hsq = 2.0*(e - f*k);
	  if (gsq >= 0.0)
	    {
	      g = sqrt(gsq);
	      qrts[j++][j3] = (-a - f - g)*0.25;
	      qrts[j++][j3] = (-a - f + g)*0.25;
	    }
	  if (hsq >= 0.0)
	    {
	      h = sqrt(hsq);
	      qrts[j++][j3] = (-a + f - h)*0.25;
	      qrts[j++][j3] = (-a + f + h)*0.25;
	    }
	donej3:
	  n4[j3] = j;
	  for (j = 0; j < n4[j3]; ++j)
	    rts[j] = qrts[j][j3];
     
	  worst3[j3] = quarticError(a, b, c, d, rts, n4[j3]);
	} /* j3 loop */
    done:
      size_t j3 = 0;
      if (n3 > 1)
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
