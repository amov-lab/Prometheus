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

/* Slightly modified the original files from https://www.marcusbannerman.co.uk/index.php/downloads.html?func=finishdown&id=20
 Mark W. Mueller (mwm@mwm.im)

Changes: 
  - some readibility tweaks (also removes an eclipse warning about braces).
*/

#pragma once

#include "cubic.hpp"

namespace magnet {
  namespace math {
    inline size_t 
    ferrariQuarticSolve(const double& a, const double& b, const double& c, const double& d,
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

      int j;
      int n4[4];
      double asqinv4;
      double ainv2;
      double d4;
      double yinv2;
      double v1[4],v2[4],v3[4];
      double p,q,r;
      double y;
      double e,f,esq,fsq,ef;
      double g,gg,h,hh;

      ainv2 = a*0.5;
      asqinv4 = ainv2*ainv2;
      d4 = d*4.0 ;

      p = b;
      q = a*c-d4;
      r = (asqinv4 - b)*d4 + c*c;
      size_t n3 = cubicSolve(p,q,r,v3[0],v3[1],v3[2]);
      for (size_t j3 = 0; j3 < n3; ++j3)
	{
	  y = v3[j3];
	  yinv2 = y*0.5;
	  esq = asqinv4 - b - y;
	  fsq = yinv2*yinv2 - d;
	  if ((esq < 0.0) && (fsq < 0.0))
	    n4[j3] = 0;
	  else
	    {
	      ef = -(0.25*a*y + 0.5*c);
	      if ( ((a > 0.0)&&(y > 0.0)&&(c > 0.0))
		   || ((a > 0.0)&&(y < 0.0)&&(c < 0.0))
		   || ((a < 0.0)&&(y > 0.0)&&(c < 0.0))
		   || ((a < 0.0)&&(y < 0.0)&&(c > 0.0))
		   ||  (a == 0.0)||(y == 0.0)||(c == 0.0))
		/* use ef - */
		{
		  if ((b < 0.0)&&(y < 0.0))
		    {
		      e = sqrt(esq);
		      f = ef/e;
		    }
		  else if (d < 0.0)
		    {
		      f = sqrt(fsq);
		      e = ef/f;
		    }
		  else
		    {
		      if (esq > 0.0)
			e = sqrt(esq);
		      else
			e = 0.0;
		      if (fsq > 0.0)
			f = sqrt(fsq);
		      else
			f = 0.0;
		      if (ef < 0.0)
			f = -f;
		    }
		}
	      else
		/* use esq and fsq - */
		{
		  if (esq > 0.0)
		    e = sqrt(esq);
		  else
		    e = 0.0;
		  if (fsq > 0.0)
		    f = sqrt(fsq);
		  else
		    f = 0.0;
		  if (ef < 0.0)
		    f = -f;
		}
	      /* note that e >= 0.0 */
	      g = ainv2 - e;
	      gg = ainv2 + e;
	      if ( ((b > 0.0)&&(y > 0.0))
		   || ((b < 0.0)&&(y < 0.0)) )
		{
		  if (((a > 0.0) && (e > 0.0))
		      || ((a < 0.0) && (e < 0.0) ))
		    g = (b + y)/gg;
		  else
		    if (((a > 0.0) && (e < 0.0))
			|| ((a < 0.0) && (e > 0.0) ))
		      gg = (b + y)/g;
		}
	      hh = -yinv2 + f;
	      h = -yinv2 - f;
	      if ( ((f > 0.0)&&(y < 0.0))
		   || ((f < 0.0)&&(y > 0.0)) )
		h = d/hh;
	      else
		if ( ((f < 0.0)&&(y < 0.0))
		     || ((f > 0.0)&&(y > 0.0)) )
		  hh = d/h;

	      bool n1 = quadSolve(hh,gg,1.0,v1[0],v1[1]);
	      bool n2 = quadSolve(h,g,1.0,v2[0],v2[1]);
	      n4[j3] = n1*2+n2*2;
	      qrts[0][j3] = v1[0];
	      qrts[1][j3] = v1[1];
	      qrts[n1*2+0][j3] = v2[0];
	      qrts[n1*2+1][j3] = v2[1];
	    }
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
