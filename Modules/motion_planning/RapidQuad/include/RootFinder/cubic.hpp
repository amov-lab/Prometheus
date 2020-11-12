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
  - undef max
  - some readibility tweaks.
*/

#pragma once

#include <cmath>
#include <limits>
#include "quadratic.hpp"

//WinDef.h defines a max() macro, which screws up compilation here...
#ifdef max
#undef max
#endif

namespace magnet
{
  namespace math
  {
    namespace detail
    {
      inline void
      cubicNewtonRootPolish(const double& p, const double& q, const double& r, double& root, size_t iterations)
      {
        for (size_t it = 0; it < iterations; ++it)
        {
          double error = ((root + p) * root + q) * root + r;
          double derivative = (3.0 * root + 2 * p) * root + q;

          if ((error == 0) || derivative == 0)
            return;

          root -= error / derivative;
        }
      }
    }

    //Please read  http://linus.it.uts.edu.au/~don/pubs/solving.html
    //For solving cubics like x^3 + p * x^2 + q * x + r == 0
    //This function always returns root1 >= root2 >= root3!
    inline size_t
    cubicSolve(const double& p, const double& q, const double& r, double& root1, double& root2, double& root3)
    {
      static const double maxSqrt = std::sqrt(std::numeric_limits<double>::max());
//      static const double maxCubeRoot = std::pow(std::numeric_limits<double>::max(), 1.0 / 3.0); //Unused

      if (r == 0)
      {
        //no constant term, so divide by x and the result is a
        //quadratic, but we must include the trivial x = 0 root
        if (quadraticSolve(p, q, root1, root2))
        {
          root3 = 0;
          if (root1 < root2)
            std::swap(root1, root2);
          if (root2 < 0)
          {
            std::swap(root2, root3);
            if (root1 < 0)
              std::swap(root1, root2);
          }
          return 3;
        }
        else
        {
          root1 = 0;
          return 1;
        }
      }

      if ((p == 0) && (q == 0))
      {
        //Special case
        //Equation is x^3 == -r
        root1 = root2 = root3 = std::pow(-r, 1.0 / 3.0);
        return 3;
      }

      if ((p > maxSqrt) || (p < -maxSqrt))
      {
        //Equation limits to x^3 + p * x^2 == 0
        root1 = -p;
        return 1;
      }

      if (q > maxSqrt)
      {
        //Special case, if q is large and the root is -r/q,
        //The x^3 term is negligble, and all other terms cancel.
        root1 = -r / q;
        return 1;
      }

      if (q < -maxSqrt)
      {
        //Special case, equation is x^3 + q x == 0
        root1 = -std::sqrt(-q);
        return 1;
      }

      if ((r > maxSqrt) || (r < -maxSqrt))
      {
        //Another special case
        //Equation is x^3 == -r
        root1 = -std::pow(r, 1.0 / 3.0);
        return 1;
      }

      double v = r + (2.0 * p * p / 9.0 - q) * (p / 3.0);

      if ((v > maxSqrt) || (v < -maxSqrt))
      {
        root1 = -p;
        return 1;
      }

      double uo3 = q / 3.0 - p * p / 9.0;
      double u2o3 = uo3 + uo3;

      if ((u2o3 > maxSqrt) || (u2o3 < -maxSqrt))
      {
        if (p == 0)
        {
          if (q > 0)
          {
            root1 = -r / q;
            return 1;
          }

          if (q < 0)
          {
            root1 = -std::sqrt(-q);
            return 1;
          }

          root1 = 0;
          return 1;
        }

        root1 = -q / p;
        return 1;
      }

      double uo3sq4 = u2o3 * u2o3;
      if (uo3sq4 > maxSqrt)
      {
        if (p == 0)
        {
          if (q > 0)
          {
            root1 = -r / q;
            return 1;
          }

          if (q < 0)
          {
            root1 = -std::sqrt(-q);
            return 1;
          }

          root1 = 0;
          return 1;
        }

        root1 = -q / p;
        return 1;
      }

      double j = (uo3sq4 * uo3) + v * v;

      if (j > 0)
      {	  //Only one root (but this test can be wrong due to a
          //catastrophic cancellation in j
          //(i.e. (uo3sq4 * uo3) == v * v)
        double w = std::sqrt(j);
        //double mcube;
        if (v < 0)
          root1 = std::pow(0.5 * (w - v), 1.0 / 3.0) - (uo3) * std::pow(2.0 / (w - v), 1.0 / 3.0) - p / 3.0;
        else
          root1 = uo3 * std::pow(2.0 / (w + v), 1.0 / 3.0) - std::pow(0.5 * (w + v), 1.0 / 3.0) - p / 3.0;

        //We now polish the root up before we use it in other calculations
        detail::cubicNewtonRootPolish(p, q, r, root1, 15);

        //We double check that there are no more roots by using a
        //quadratic formula on the factored problem, this helps when
        //the j test is wrong due to numerical error.

        //We have a choice of either -r/root1, or q -
        //(p+root1)*root1 for the constant term of the quadratic.
        //
        //The division one usually results in more accurate roots
        //when it finds them but fails to detect real roots more
        //often than the multiply.
        if (quadraticSolve(p + root1, -r / root1, root2, root3))
          return 3;
        //
        //However, the multiply detects roots where there are none,
        //the division does not. So we must either accept missing
        //roots or extra roots, here we choose missing roots
        //
        //if (quadSolve(q-(p+root1)*root1, p + root1, 1.0, root2, root3))
        //  return 3;

        return 1;
      }

      if (uo3 >= 0)
      {	  //Multiple root detected
        root1 = root2 = root3 = std::pow(v, 1.0 / 3.0) - p / 3.0;
        return 3;
      }

      double muo3 = -uo3;
      double s;
      if (muo3 > 0)
      {
        s = std::sqrt(muo3);
        if (p > 0)
          s = -s;
      }
      else
        s = 0;

      double scube = s * muo3;
      if (scube == 0)
      {
        root1 = -p / 3.0;
        return 1;
      }

      double t = -v / (scube + scube);
      double k = std::acos(t) / 3.0;
      double cosk = std::cos(k);
      root1 = (s + s) * cosk - p / 3.0;

      double sinsqk = 1.0 - cosk * cosk;
      if (sinsqk < 0)
        return 1;

      double rt3sink = std::sqrt(3.) * std::sqrt(sinsqk);
      root2 = s * (-cosk + rt3sink) - p / 3.0;
      root3 = s * (-cosk - rt3sink) - p / 3.0;

      detail::cubicNewtonRootPolish(p, q, r, root1, 15);
      detail::cubicNewtonRootPolish(p, q, r, root2, 15);
      detail::cubicNewtonRootPolish(p, q, r, root3, 15);

      return 3;
    }
  }
}
