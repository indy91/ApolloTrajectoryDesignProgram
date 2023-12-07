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

#include <string>
#include "VectorMath.h"
#include "MathConstants.h"
#include "DataTables.h"
 
namespace OrbMech
{
	enum Bodies
	{
		Body_Earth,
		Body_Moon
	};

	struct SphericalCoordinates
	{
		double lat;
		double lng;
		double rad;
		double vel;
		double azi;
		double fpa;
	};

	struct SunMoonEphemerisTable
	{
		int EPOCH;
		//MJD of first entry
		double MJD;
		//71 sets of data, 12 hours apart, covering 35 days, starting 5 days before midnight of launch day
		//0-2: Sun position vector (Er), 3-5: Moon position vector (Er), 6-8: Moon velocity vector (Er/hr), 9-17: Moon libration vector
		double data[71][18];
	};

	struct EphemerisData
	{
		double GMT = 0.0;
		VECTOR3 R = _V(0, 0, 0);
		VECTOR3 V = _V(0, 0, 0);
		int RBI = -1; //0 = Earth, 1 = Moon
	};

	struct SIVBVentingTable
	{
		SIVBVentingTable()
		{
			MCGVEN = 0.2;
			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 10; j++)
				{
					MDTVTV[i][j] = 0.0;
				}
			}

			//Vent thrust
			MDTVTV[0][0] = 175.0 / LBF2N;
			MDTVTV[0][1] = 170.0 / LBF2N;
			MDTVTV[0][2] = 80.0 / LBF2N;
			MDTVTV[0][3] = 65.0 / LBF2N;
			MDTVTV[0][4] = 45.0 / LBF2N;
			MDTVTV[0][5] = 40.0 / LBF2N;
			MDTVTV[0][6] = 40.0 / LBF2N;
			MDTVTV[0][7] = 0.0;
			MDTVTV[0][8] = 0.0;
			//Vent times
			MDTVTV[1][0] = 0.0;
			MDTVTV[1][1] = 1300.0 / 3600.0;
			MDTVTV[1][2] = 3300.0 / 3600.0;
			MDTVTV[1][3] = 6500.0 / 3600.0;
			MDTVTV[1][4] = 11000.0 / 3600.0;
			MDTVTV[1][5] = 15000.0 / 3600.0;
			MDTVTV[1][6] = 20000.0 / 3600.0;
			MDTVTV[1][7] = 20001.0 / 3600.0; //Switches off vent model at this time
			MDTVTV[1][8] = 9999999999.9;
			MCTVSP = 850.0 / 3600.0; //Find better value
			MCTVEN = 1.0;
		}

		//Reference GET for MDTVTV (Venting thrust table) (hrs.)
		double MCGVEN;
		//Venting thrust table
		double MDTVTV[2][10];
		//Venting specific impulse
		double MCTVSP;
		//Venting scale factor
		double MCTVEN;
	};

	struct DensityTables
	{
		DensityTables()
		{
			r0 = 6356766.0; //45°32'33'' latitude
			C2 = 401.8743008658905; //1.4*8.31432/(28.9644/1000)
			CSMAX = 269.4412597;

			H_b[0] = 79000.0;
			H_b[1] = 61000.0;
			H_b[2] = 52000.0;
			H_b[3] = 47000.0;
			H_b[4] = 32000.0;
			H_b[5] = 20000.0;
			H_b[6] = 11000.0;
			H_b[7] = 0.0;

			ln_rho_b[0] = -10.819278;
			ln_rho_b[1] = -8.289699;
			ln_rho_b[2] = -7.182942;
			ln_rho_b[3] = -6.551831;
			ln_rho_b[4] = -4.325646;
			ln_rho_b[5] = -2.430021;
			ln_rho_b[6] = -1.010821;
			ln_rho_b[7] = 0.202941;

			L_M_B[0] = 0.0;
			L_M_B[1] = -4.0 / 1000.0;
			L_M_B[2] = -2.0 / 1000.0;
			L_M_B[3] = 0.0;
			L_M_B[4] = 2.8 / 1000.0;
			L_M_B[5] = 1.0 / 1000.0;
			L_M_B[6] = 0.0;
			L_M_B[7] = -6.5 / 1000.0;

			T_M_B[0] = 180.65;
			T_M_B[1] = 252.65;
			T_M_B[2] = 270.65;
			T_M_B[3] = 270.65;
			T_M_B[4] = 228.65;
			T_M_B[5] = 216.65;
			T_M_B[6] = 216.65;
			T_M_B[7] = 288.15;

			A[0] = 0.000000e+00;
			A[1] = -7.540799e+00;
			A[2] = -1.608160e+01;
			A[3] = 0.000000e+00;
			A[4] = 1.320114e+01;
			A[5] = 3.516319e+01;
			A[6] = 0.000000e+00;
			A[7] = -4.255876e+00;

			B[0] = 1.891126e-04;
			B[1] = -1.583218e-05;
			B[2] = -7.389618e-06;
			B[3] = 1.262265e-04;
			B[4] = 1.224579e-05;
			B[5] = 4.615740e-06;
			B[6] = 1.576884e-04;
			B[7] = -2.255770e-05;

			r_b[0] = r0 + 600.0 * 1000.0;
			r_b[1] = r0 + 500.0 * 1000.0;
			r_b[2] = r0 + 400.0 * 1000.0;
			r_b[3] = r0 + 300.0 * 1000.0;
			r_b[4] = r0 + 230.0 * 1000.0;
			r_b[5] = r0 + 190.0 * 1000.0;
			r_b[6] = r0 + 170.0 * 1000.0;
			r_b[7] = r0 + 160.0 * 1000.0;
			r_b[8] = r0 + 150.0 * 1000.0;
			r_b[9] = r0 + 120.0 * 1000.0;
			r_b[10] = r0 + 110.0 * 1000.0;
			r_b[11] = r0 + 100.0 * 1000.0;
			r_b[12] = r0 + 90.0 * 1000.0;

			L_M[0] = 1.1 / 1000.0;
			L_M[1] = 1.7 / 1000.0;
			L_M[2] = 2.6 / 1000.0;
			L_M[3] = 3.3 / 1000.0;
			L_M[4] = 4.0 / 1000.0;
			L_M[5] = 5.0 / 1000.0;
			L_M[6] = 7.0 / 1000.0;
			L_M[7] = 10.0 / 1000.0;
			L_M[8] = 15.0 / 1000.0;
			L_M[9] = 20.0 / 1000.0;
			L_M[10] = 10.0 / 1000.0;
			L_M[11] = 5.0 / 1000.0;
			L_M[12] = 3.0 / 1000.0;

			T_M_b[0] = 2590.65;
			T_M_b[1] = 2420.65;
			T_M_b[2] = 2160.65;
			T_M_b[3] = 1830.65;
			T_M_b[4] = 1550.65;
			T_M_b[5] = 1350.65;
			T_M_b[6] = 1210.65;
			T_M_b[7] = 1110.65;
			T_M_b[8] = 960.65;
			T_M_b[9] = 360.65;
			T_M_b[10] = 260.65;
			T_M_b[11] = 210.65;
			T_M_b[12] = 180.65;

			A7[0] = 1.511805e+00;
			A7[1] = 1.262093e+00;
			A7[2] = 1.140239e+00;
			A7[3] = 1.090911e+00;
			A7[4] = 1.062535e+00;
			A7[5] = 1.043037e+00;
			A7[6] = 1.027220e+00;
			A7[7] = 1.017338e+00;
			A7[8] = 1.009940e+00;
			A7[9] = 1.002792e+00;
			A7[10] = 1.004047e+00;
			A7[11] = 1.006568e+00;
			A7[12] = 1.009429e+00;

			B7[0] = 3.920299e+01;
			B7[1] = 2.179890e+01;
			B7[2] = 1.326098e+01;
			B7[3] = 1.029864e+01;
			B7[4] = 8.452201e+00;
			B7[5] = 6.719039e+00;
			B7[6] = 4.755544e+00;
			B7[7] = 3.306984e+00;
			B7[8] = 2.195356e+00;
			B7[9] = 1.650043e+00;
			B7[10] = 3.314444e+00;
			B7[11] = 6.666131e+00;
			B7[12] = 1.117639e+01;

			ln_rho_b_T_M_b[0] = -20.539228;
			ln_rho_b_T_M_b[1] = -19.383705;
			ln_rho_b_T_M_b[2] = -18.081362;
			ln_rho_b_T_M_b[3] = -16.539251;
			ln_rho_b_T_M_b[4] = -15.232175;
			ln_rho_b_T_M_b[5] = -14.348024;
			ln_rho_b_T_M_b[6] = -13.843007;
			ln_rho_b_T_M_b[7] = -13.563008;
			ln_rho_b_T_M_b[8] = -13.248066;
			ln_rho_b_T_M_b[9] = -11.642415;
			ln_rho_b_T_M_b[10] = -10.572165;
			ln_rho_b_T_M_b[11] = -9.163673;
			ln_rho_b_T_M_b[12] = -7.465218;
		}

		//Radius of Earth at latitude 45° 33' 33''
		double r0;
		//K*R/M0
		double C2;
		//Speed of sound at 90km
		double CSMAX;

		//Tables for 90km and below
		//Base altitude
		double H_b[8];
		//Molecular scale temperature
		double T_M_B[8];
		double ln_rho_b[8];
		double L_M_B[8];
		double A[8];
		double B[8];

		//Tables for 90km and above
		//Base radius
		double r_b[13];
		//Gradient of the molecular-scale temperature in degrees Kelvin per meter
		double L_M[13];
		//Molecular scale temperature in Kelvon
		double T_M_b[13];
		//Coefficents
		double A7[13], B7[13], ln_rho_b_T_M_b[13];
	};

	struct TLIBRNOutput
	{
		EphemerisData sv2;
		double W2;
		VECTOR3 T;
		double alpha;
		double beta;
		double DV;
	};

	//Greenwich hour angle
	double HANGLE(int E, int Y, int D);
	int DayOfYear(int Y, int M, int D);
	double TJUDAT(int Y, int M, int D);

	int InitializeEphemerides(int Epoch, double MJD, SunMoonEphemerisTable &table);
	bool JPLEPH(const SunMoonEphemerisTable &table, double GMTBASE, int IND, double HOUR, VECTOR3* R_EM, VECTOR3* V_EM, VECTOR3* R_ES, MATRIX3* PNL = NULL);

	void BPlaneTargetingPC(const SunMoonEphemerisTable &MDGSUN, double GMTBASE, EphemerisData sv_PC, double RCA_m, double phi_star, double& D_BT, double& D_BR, EphemerisData& sv_PC_apo, VECTOR3 &R_EM, VECTOR3& V_EM);
	void BPlaneTargetingPG(EphemerisData sv_PG, VECTOR3 R_EM, VECTOR3 V_M, double RCA_E, double& D_BT);

	void GLFDEN(double ALT, double& DENS, double& SPOS);

	double stumpS(double z);
	double stumpC(double z);
	double kepler_U(double dt, double ro, double vro, double a, double mu, double x0);
	void f_and_g(double x, double t, double ro, double a, double& f, double& g, double mu);
	void fDot_and_gDot(double x, double r, double ro, double a, double& fdot, double& gdot, double mu);
	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3& R1, VECTOR3& V1, double mu, double x = 0.0);

	double ElevationAngle(VECTOR3 R_S, VECTOR3 R);
	double LunarSunElevationAngleAtMJD(const SunMoonEphemerisTable& MDGSUN, double GMTBASE, double HOUR, VECTOR3 R_S_SG);

	//Conversions
	void adbar_from_rv(double rmag, double vmag, double rtasc, double decl, double fpav, double az, VECTOR3& R, VECTOR3& V);
	MATRIX3 EMP_Matrix(VECTOR3 R, VECTOR3 V);
	VECTOR3 r_from_latlong(double lat, double lng);
	void latlong_from_r(VECTOR3 R, double& lat, double& lng);
	void SStoHHMMSS(double time, int& hours, int& minutes, double& seconds);

	double sign(double a);
	double cot(double a);
	void normalizeAngle(double& a, bool positive = true);

	std::string FixedWidthString(std::string str, unsigned len);

	//EOI
	EOIProcessorDataSet GetEOIProcessorData(int Pad, int altitude);
	void EOIProcessor(const EOIProcessorDataSet &data, double A_Z, OrbMech::SphericalCoordinates& coord, double& T_L);
	//TLI
	void TLINominalMissionPolynomial(double C3, double sigma, double delta, double FW, double R_I, double& DV, double& alpha, double& beta, double& eta_alpha, double& R_P);
	void TLIBRN(EphemerisData sv1, double C3, double sigma, double delta, double F, double F_I, double W_I, double WDOT, double T_MRS, TLIBRNOutput &out);
	//Burn simulation
	void BURN(VECTOR3 R, VECTOR3 V, double dv, double dgamma, double dpsi, double isp, double& dv_R, double& mfm0, VECTOR3& RF, VECTOR3& VF);

	//Least squares
	void LeastSquares(double* x, double* y, int n, int m, double *beta);
}