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
  - Removed boost code
  - some readibility tweaks.
*/

#pragma once

#include <cmath>

namespace magnet {
namespace math {
inline double quarticError(const double& a, const double& b, const double& c,
		const double& d, const double roots[4], const size_t rootCount) {
//      boost::array<double, 4> errors;
	double errors[4];

	for (size_t root = 0; root < rootCount; ++root) {
		const double value = (((roots[root] + a) * roots[root] + b)
				* roots[root] + c) * roots[root] + d;

		if (value == 0) {
			errors[root] = 0;
			continue;
		}

		const double deriv = ((4 * roots[root] + 3 * a) * roots[root] + 2 * b)
				* roots[root] + c;

		if (deriv != 0)
			errors[root] = std::abs(value / deriv);
		else {
			const double secDeriv = (12 * roots[root] + 6 * a) * roots[root]
					+ 2 * b;
			if (secDeriv != 0)
				errors[root] = std::sqrt(std::abs(value / secDeriv));
			else {
				const double thirdDeriv = 24 * roots[root] + 6 * a;
				if (thirdDeriv != 0)
					errors[root] = std::pow(std::abs(value / thirdDeriv),
							1.0 / 3.0);
				else
					errors[root] = std::sqrt(std::sqrt(std::abs(value) / 24));
			}
		}
	}

	double err = 0;
	for (unsigned i = 0; i < rootCount; i++) {
		if(errors[i]>err) err = errors[i];
	}
	return err;
//	return *std::max_element(errors.begin(), errors.begin() + rootCount);
}
}
}
