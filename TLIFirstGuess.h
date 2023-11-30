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

#include "OrbMech.h"

struct TLIFirstGuessInputs
{
	double BHA; //Hour angle at midnight
	double Hour; //Hour at which to start the search
	double GMTBASE; //MJD at midnight before liftoff

	double C1; //Declination of the input state vector, radians
	double FLO1; //Geographic longitude of the input orbit state vector
	double AZ1; //Inertial azimuth of the input orbit state vector
	double R1; //Radius of the input orbit state vector, nautical miles
	double ORBNUM; //Number of the inertial orbit revolution
	int IPOA; //1 = Pacific window (AZ2V <= 90°), 2 = Atlantic window (AZ2V > 90°)
	int IPERT; //0 = no perturbations, 1 = Earth oblateness, 2 = solar gravitation, 3 = both
	double XPC; //Longitude of pericynthion of the simulated trajectory, measured in a moon-centered MOP coordinate system
				//from the extension of the earth-moon axis on the back side of the moon, degrees
	double YPC; //Declination of pericynthion of the simulated trajectory, in a moon-centered MOP coordinate system
	double RPC; //Radius of pericynthion of the simulation trajectory, nautical miles
	int MAXIT = 50;
	double TOL = 0.001;
	double TTW = 0.7; //Valid for 0.63 to 0.8
	bool debug = true;

	OrbMech::SunMoonEphemerisTable* MDGSUN;
};

struct TLIFirstGuessOutputs
{
	double T1; //Time of launch from base time
	double CTIO; //Time in Earth orbit
	double C3; //Energy of TLI
	double S; //Angle sigma
	double TOPCY; //Time of PC from base time
};

class TLIFirstGuess
{
public:
	TLIFirstGuess();

	void Calculate(const TLIFirstGuessInputs& in);

	void GetOutput(TLIFirstGuessOutputs& out);
	int GetError() { return CO1; }
private:
	void CIST(double RAGBT);
	void UPDATE(double TOIDIS);

	void PERCYN(double T);
	double ENERGY(int IPERT, double C4) const;
	double FLYTYM(int IPERT) const;
	void GEOLAT(double FI, double C, double RA, int I, double& A, double& B, double& AZ, double& RN) const;
	void GEOARG(double FI, double A, double RN, double& AZ, double& B, double& C, double& RA) const;
	void SUBB(int IPERT, double &BETA, double &AFNTMH) const;
	double SUBCL(int IPERT) const;
	void TLIMP(double W);
	double HELP(double X) const;
	double ANGLE(VECTOR3 VEC1, VECTOR3 VEC2, VECTOR3 VEC3) const;

	TLIFirstGuessInputs inp;
	TLIFirstGuessOutputs outp;

	VECTOR3 MH; //Hypersurface
	double WV;
	double FIV;
	double RNV;
	double A1;

	double FIVTL;
	double R4;
	double APS;
	double FTOCO;

	VECTOR3 HM; //Angular momentum vector of the Moon
	double EMR;
	double EMRDOT;
	double SMOPL;
	double A6;
	double FIM;
	double RNM;
	double WM;
	double AM;
	double DS;

	//Error indicator
	int CO1;
};