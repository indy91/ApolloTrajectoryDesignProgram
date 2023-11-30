/***************************************************************************
This file is part of the Apollo Trajectory Design Program.

The Apollo Trajectory Design Program is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The Apollo Trajectory Design Program is distributed in the hope that it will
be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public Licensealong with
the Apollo Trajectory Design Program. If not, see <https://www.gnu.org/licenses/>.

**************************************************************************/

#pragma once

//PI
const double PI = 3.14159265358979323846;
const double PI05 = 1.57079632679489661923;
const double PI2 = 6.28318530717958647693;
//Degrees to radians
const double RAD = PI / 180.0;
//Radians to degrees
const double DEG = 180.0 / PI;
//One hour in seconds
const double HRS = 3600.0;

namespace OrbMech
{
	//Equatorial radius of Earth, meters
	const double R_Earth = 6378165.0;
	//Mean lunar radius of Moon, Earth radii
	const double R_Moon = 1738090.0 / R_Earth;

	//Nautical miles to meters
	const double NM2M = 1852.0;
	//Feet per meter
	const double FT2M = 0.3048;

	//Earth radii to nautical miles
	const double ER2NM = R_Earth / NM2M;
	//Earth radii per hour per feet per second
	const double FPS2ERPH = FT2M * HRS / R_Earth;

	//Standard Earth gravity (m/s^2)
	const double G0 = 9.80665;
	const double G0ER = G0 * HRS * HRS / R_Earth;

	//Standard gravitational parameter of the Earth (Er^3/hr^2)
	const double mu_Earth = 19.90941651408238;
	//Rotational rate of the Earth (rad/hr)
	const double w_Earth = 0.262516145280049;

	//Standard gravitational parameter of the Moon (Er^3/hr^2)
	const double mu_Moon = 0.244883757;
	//Rotational rate of the Moon (rad/hr)
	const double w_Moon = 9.582118128e-03;

	//Standard gravitational parameter of the Sun (Er^3/hr^2)
	const double mu_Sun = 6628865.7;

	const double J2_Earth = 1082.6269e-6;
	const double J3_Earth = -2.51e-6;
	const double J4_Earth = -1.60e-6;
	const double J5_Earth = -0.15e-6;
	const double J2_Moon = 207.108e-6;
	const double J3_Moon = -2.1e-5;

	//Drag coefficent
	const double CD = 2.0;
	//Pounds (mass) to kg
	const double LBS2KG = 0.45359237;
	//Pounds (force) to Newton
	const double LBF2N = G0 * LBS2KG;
	//Converts RHO*AREA/MASS to 0.5*RHO*AREA/MASS in Er
	const double CFRAM = -653175.58;

}