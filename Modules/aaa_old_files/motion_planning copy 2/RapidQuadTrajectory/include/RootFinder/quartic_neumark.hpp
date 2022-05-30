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
    //solve the quartic equation -
    //x**4 + a*x**3 + b*x**2 + c*x + d = 0
    inline size_t 
    neumarkQuarticSolve(const double& a, const double& b, const double& c, const double& d, 
			double& root1, double& root2, double& root3, double& root4)
    {
      double rts[4];
      double worst3[3];
      double y,g,gg,h,hh,gdis,gdisrt,hdis,hdisrt,g1,g2,h1,h2;
      double bmy,gerr,herr,y4,bmysq;
      double v1[4],v2[4],v3[4];
      double hmax,gmax;
      double qrts[4][3];        /* quartic roots for each cubic root */

      if (d == 0.0)
	{
	  root1 = 0.0;
	  return cubicSolve(a,b,c,root2,root3,root4) + 1;
	}

      double asq = a * a;

      double d4 = d * 4.0;
      double p =  -b * 2.0;
      double q = b * b + a * c - d4;
      double r = (c - a*b)*c + asq*d;
      size_t cubicRoots = cubicSolve(p,q,r,v3[0],v3[1],v3[2]);

      size_t nQuarticRoots[3]; 

      for (size_t j3 = 0; j3 < cubicRoots; ++j3)
	{
	  y = v3[j3];
    
	  bmy = b - y;
	  y4 = y * 4.0;
	  bmysq = bmy*bmy;
	  gdis = asq - y4;
	  hdis = bmysq - d4;

	  if ((gdis < 0.0) || (hdis < 0.0))
	    nQuarticRoots[j3] = 0;
	  else
	    {
	      g1 = a * 0.5;
	      h1 = bmy* 0.5;
	      gerr = asq + y4;
	      herr = hdis;
	      if (d > 0.0)
		herr = bmysq + d4;
	      if ((y < 0.0) || (herr*gdis > gerr*hdis))
		{
		  gdisrt = sqrt(gdis);
		  g2 = gdisrt*0.5;
		  if (gdisrt != 0.0)
		    h2 = (a*h1 - c)/gdisrt;
		  else
		    h2 = 0.0;
		}
	      else
		{
		  hdisrt = std::sqrt(hdis);
		  h2 = hdisrt*0.5;
		  if (hdisrt != 0.0)
		    g2 = (a*h1 - c)/hdisrt;
		  else
		    g2 = 0.0;
		}
	      /*
		note that in the following, the tests ensure non-zero
		denominators -
	      */
	      h = h1 - h2;
	      hh = h1 + h2;
	      hmax = hh;
	      if (hmax < 0.0)
		hmax =  -hmax;
	      if (hmax < h)
		hmax = h;
	      if (hmax <  -h)
		hmax =  -h;
	      if ((h1 > 0.0)&&(h2 > 0.0))
		h = d/hh;
	      if ((h1 < 0.0)&&(h2 < 0.0))
		h = d/hh;
	      if ((h1 > 0.0)&&(h2 < 0.0))
		hh = d/h;
	      if ((h1 < 0.0)&&(h2 > 0.0))
		hh = d/h;
	      if (h > hmax)
		h = hmax;
	      if (h <  -hmax)
		h =  -hmax;
	      if (hh > hmax)
		hh = hmax;
	      if (hh < -hmax)
		hh =  -hmax;

	      g = g1 - g2;
	      gg = g1 + g2;
	      gmax = gg;
	      if (gmax < 0.0)
		gmax = -gmax;
	      if (gmax < g)
		gmax = g;
	      if (gmax < -g)
		gmax = -g;
	      if ((g1 > 0.0)&&(g2 > 0.0))
		g = y/gg;
	      if ((g1 < 0.0)&&(g2 < 0.0))
		g = y/gg;
	      if ((g1 > 0.0)&&(g2 < 0.0))
		gg = y/g;
	      if ((g1 < 0.0)&&(g2 > 0.0))
		gg = y/g;
	      if (g > gmax)
		g = gmax;
	      if (g <  -gmax)
		g = -gmax;
	      if (gg > gmax)
		gg = gmax;
	      if (gg <  -gmax)
		gg = -gmax;

	      size_t n1 = quadSolve(hh,gg, 1.0,v1[0],v1[1]);
	      size_t n2 = quadSolve(h,g,1.0,v2[0],v2[1]);
	      nQuarticRoots[j3] = 2*n1 + 2*n2;
	      qrts[0][j3] = v1[0];
	      qrts[1][j3] = v1[1];
	      qrts[2*n1+0][j3] = v2[0];
	      qrts[2*n1+1][j3] = v2[1];
	    }

	  for (size_t j = 0; j < nQuarticRoots[j3]; ++j)
	    rts[j] = qrts[j][j3];
	  worst3[j3] = quarticError(a, b, c, d, rts,nQuarticRoots[j3]);

	} /* j3 loop */
      size_t  j3 = 0;
      if (cubicRoots > 1)
	{
	  if ((nQuarticRoots[1] > nQuarticRoots[j3]) ||
	      ((worst3[1] < worst3[j3] ) && (nQuarticRoots[1] == nQuarticRoots[j3]))) j3 = 1;
	  if ((nQuarticRoots[2] > nQuarticRoots[j3]) ||
	      ((worst3[2] < worst3[j3] ) && (nQuarticRoots[2] == nQuarticRoots[j3]))) j3 = 2;
	}

      root1 = qrts[0][j3];
      root2 = qrts[1][j3];
      root3 = qrts[2][j3];
      root4 = qrts[3][j3];

      return nQuarticRoots[j3];
    }
  }
}
