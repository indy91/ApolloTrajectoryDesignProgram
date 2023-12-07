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

#include "pch.h"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "MathConstants.h"
#include "OrbMech.h"

namespace OrbMech
{
	double HANGLE(int E, int Y, int D)
	{
		//E - Epoch year
		//Y - Year
		//D - Day

		int XN;
		static const double A = 0.0929;
		static const double B = 8640184.542;
		static const double W1 = 1.720217954160054e-2;
		double C, T, DE, BHA, DI, DELTA;

		XN = (E - 1901) / 4;
		C = -86400.0 * (double)(E - 1900) - 74.164;
		T = 2 * C / (-B - sqrt(B * B - 4 * A * C));
		DE = 36525.0 * T - 365.0 * (double)(E - 1900) + 0.5 - (double)XN;
		if (Y == E)
		{
			DI = D;
		}
		else
		{
			int X = Y % 4;
			if (X == 0)
			{
				DI = D - 366.0;
			}
			else
			{
				DI = D - 365.0;
			}
		}
		DELTA = DI - DE;
		BHA = PI2 / 3.6 + W1 * DELTA;
		return BHA;
	}

	int DayOfYear(int Y, int M, int D)
	{
		int RefDay;
		int m[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
		double Y_J = (double)Y + 0.001;
		double J = Y_J - 1960.0;
		int K = (int)(J / 4);
		J = floor(J);
		if (((int)J) == 4 * K)
		{
			m[1] = 29;
		}
		double T = (double)D + 0.5;
		J = floor(T);
		double T_apo = (double)M - 0.5;
		K = (int)T_apo;
		RefDay = (int)J;

		if (K != 0)
		{
			for (int i = 0; i < K; i++)
			{
				RefDay += m[i];
			}
		}
		return RefDay;
	}

	double TJUDAT(int Y, int M, int D)
	{
		int Y_apo = Y - 1900;
		int TMM[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

		int Z = Y_apo / 4;
		if (Y_apo % 4 == 0)
		{
			Z = Z - 1;
			for (int i = 2; i < 12; i++)
			{
				TMM[i] += 1;
			}
		}
		return 2415020.5 + (double)(365 * Y_apo + Z + TMM[M - 1] + D - 1);
	}

	int InitializeEphemerides(int Epoch, double MJD, SunMoonEphemerisTable& table)
	{
		table.EPOCH = Epoch;

		//Table starts 5 days before input MJD
		table.MJD = MJD - 5.0;

		//Open ephemerides files

		std::string line;

		char Buff[128];
		snprintf(Buff, 128, "./Ephemerides/MoonEphemeris%d.txt", Epoch);

		std::ifstream moonfile(Buff);
		if (!moonfile.is_open()) return 1;

		//Go to second line
		std::getline(moonfile, line);
		std::getline(moonfile, line);

		double MJD0;
		if (sscanf_s(line.c_str(), "%lf", &MJD0) != 1) return 1;

		//Calculate first line of ephemerides to read
		int num = 2 * (int)(table.MJD - MJD0);

		int line_no = 0;
		while (line_no != num && std::getline(moonfile, line)) {
			++line_no;
		}

		for (int i = 0; i < 71; i++)
		{
			std::getline(moonfile, line);
			if (sscanf_s(line.c_str(), "%lf %lf %lf %lf %lf %lf", &table.data[i][3], &table.data[i][4], &table.data[i][5], &table.data[i][6], &table.data[i][7], &table.data[i][8]) != 6)
			{
				return 2;
			}
		}

		moonfile.close();

		//Read Sun ephemeris
		snprintf(Buff, 128, "./Ephemerides/SolarEphemeris%d.txt", Epoch);

		std::ifstream sunfile(Buff);
		if (!sunfile.is_open()) return 1;

		//Go to second line
		std::getline(sunfile, line);
		std::getline(sunfile, line);

		if (sscanf_s(line.c_str(), "%lf", &MJD0) != 1) return 1;

		//Calculate first line of ephemerides to read
		num = 2 * (int)(table.MJD - MJD0);

		line_no = 0;
		while (line_no != num && std::getline(sunfile, line)) {
			++line_no;
		}

		for (int i = 0; i < 71; i++)
		{
			std::getline(sunfile, line);
			if (sscanf_s(line.c_str(), "%lf %lf %lf", &table.data[i][0], &table.data[i][1], &table.data[i][2]) < 3)
			{
				return 2;
			}
		}

		sunfile.close();

		//Read Moon libration matrix

		snprintf(Buff, 128, "./Ephemerides/MoonLibration%d.txt", Epoch);

		std::ifstream moonlibfile(Buff);
		if (!moonlibfile.is_open()) return 1;

		//Go to second line
		std::getline(moonlibfile, line);
		std::getline(moonlibfile, line);

		if (sscanf_s(line.c_str(), "%lf", &MJD0) != 1) return 1;

		//Calculate first line of ephemerides to read
		num = 2 * (int)(table.MJD - MJD0);

		line_no = 0;
		while (line_no != num && std::getline(moonlibfile, line)) {
			++line_no;
		}

		for (int i = 0; i < 71; i++)
		{
			std::getline(moonlibfile, line);
			if (sscanf_s(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &table.data[i][9], &table.data[i][10], &table.data[i][11],
				&table.data[i][12], &table.data[i][13], &table.data[i][14], &table.data[i][15], &table.data[i][16], &table.data[i][17]) < 9)
			{
				return 2;
			}
		}

		moonlibfile.close();

		return 0;
	}

	bool JPLEPH(const SunMoonEphemerisTable &table, double GMTBASE, int IND, double HOUR, VECTOR3* R_EM, VECTOR3* V_EM, VECTOR3* R_ES, MATRIX3* PNL)
	{
		//INPUTS:
		//IND: 1 = Sun and Moon ephemerides. 2 = all data. 3 = all Moon data. 4 = Moon ephemerides. 5 = libration matrix only
		double T, C[6];
		int i, j, k;

		//T is 0 to 1, from last to next 12 hour interval
		T = (HOUR - floor(HOUR / 12.0) * 12.0) / 12.0;
		C[0] = -((T + 1.0) * T * (T - 1.0) * (T - 2.0) * (T - 3.0)) / 120.0;
		C[1] = ((T + 2.0) * T * (T - 1.0) * (T - 2.0) * (T - 3.0)) / 24.0;
		C[2] = -((T + 2.0) * (T + 1.0) * (T - 1.0) * (T - 2.0) * (T - 3.0)) / 12.0;
		C[3] = ((T + 2.0) * (T + 1.0) * T * (T - 2.0) * (T - 3.0)) / 12.0;
		C[4] = -((T + 2.0) * (T + 1.0) * T * (T - 1.0) * (T - 3.0)) / 24.0;
		C[5] = ((T + 2.0) * (T + 1.0) * T * (T - 1.0) * (T - 2.0)) / 120.0;

		//Calculate MJD from GMT
		double MJD = GMTBASE + HOUR / 24.0;
		//Calculate position of time in array
		i = (int)((MJD - table.MJD) * 2.0);
		//Calculate starting point in the array
		j = i - 2;
		//Is time contained in Sun/Moon data array?
		if (j < 0 || j > 65) return true; //Error return

		double x[18] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		bool des[4] = { false, false, false, false };

		//Which data to get?
		switch (abs(IND))
		{
		case 1:
			des[0] = des[1] = des[2] = true;
			break;
		case 2:
			des[0] = des[1] = des[2] = des[3] = true;
			break;
		case 3:
			des[1] = des[2] = des[3] = true;
			break;
		case 4:
			des[1] = des[2] = true;
			break;
		case 5:
			des[3] = true;
			break;
		default:
			return true; //Error return
		}

		for (k = 0; k < 18; k++)
		{
			if (k < 3)
			{
				//Sun position
				if (des[0] == false) continue;
			}
			else if (k < 6)
			{
				//Moon position
				if (des[1] == false) continue;
			}
			else if (k < 9)
			{
				//Moon velocity
				if (des[2] == false) continue;
			}
			else
			{
				//Moon libration
				if (des[3] == false) break;
			}
			x[k] = C[0] * table.data[j][k] + C[1] * table.data[j + 1][k] + C[2] * table.data[j + 2][k] + C[3] * table.data[j + 3][k] + C[4] * table.data[j + 4][k] + C[5] * table.data[j + 5][k];
		}
		if (des[0] && R_ES)
		{
			*R_ES = _V(x[0], x[1], x[2]);
		}
		if (des[1] && R_EM)
		{
			*R_EM = _V(x[3], x[4], x[5]);
		}
		if (des[2] && V_EM)
		{
			*V_EM = _V(x[6], x[7], x[8]);
		}
		if (des[3] && PNL)
		{
			*PNL = _M(x[9], x[10], x[11], x[12], x[13], x[14], x[15], x[16], x[17]);
		}

		return false;
	}

	void BPlaneTargetingPG(EphemerisData sv_PG, VECTOR3 R_EM, VECTOR3 V_M, double RCA_E, double& D_BT)
	{
		//R_EM, V_EM - vectors from Earth to Moon at pericynthion (not perigee)

		VECTOR3 S_E, R_E, N_P, B_E_u, B_E, R_u, T_E, W_E;
		double r, v, a, e, f, b_E, e_star, b_E_star, B_R_E_star, I_STE_star, B_T_E_star, B_T_E, B_R_E;

		W_E = _V(0, 0, 1);
		//R_M = -R_EM; //Moon-Earth line (from Earth-moon line)
		S_E = -unit(R_EM);// unit(R_M); //Moon-Earth line
		//N_M = -unit(crossp(R_M, V_M)); //Normal vector to the moon travel plane
		T_E = unit(crossp(S_E, W_E)); //unit(crossp(S_E, N_M)); //Unit vector in the moon-earth plane, perpendicular to S_E
		R_E = crossp(S_E, T_E);// -N_M;// -crossp(S_E, T_E);

		r = length(sv_PG.R);
		v = length(sv_PG.V);

		a = -mu_Earth / 2.0 / (v * v / 2.0 - mu_Earth / r);

		N_P = unit(crossp(sv_PG.R, sv_PG.V)); //Normal vector of flight plane
		B_E_u = unit(crossp(S_E, N_P)); //Vector in the flight plane, perpendicular to S_E and in the opposite sense of V_P
		R_u = unit(sv_PG.R); //Unit perigee vector
		f = acos(dotp(R_u, B_E_u)); //True anomaly
		e = 1.0 - r / a; //Eccentricity
		b_E = r * (1.0 + e) / (1.0 + e * cos(f)); //Semi-minor axis
		B_E = B_E_u * b_E;
		//Actual Earth impact parameters
		B_T_E = dotp(B_E, T_E);
		B_R_E = dotp(B_E, R_E);
		//Desired Earth impact parameters
		B_R_E_star = B_R_E; //same as actual
		e_star = 1.0 - RCA_E / a;
		b_E_star = RCA_E * (1.0 + e_star) / (1.0 + e_star * cos(f)); //Magnitude of the desired elliptic impact parameters
		I_STE_star = asin(fmin(1.0, fmax(-1.0, B_R_E_star / b_E_star))); //TBD
		B_T_E_star = b_E_star * cos(I_STE_star);

		D_BT = B_T_E_star - B_T_E;
	}

	void BPlaneTargetingPC(const SunMoonEphemerisTable &MDGSUN, double GMTBASE, EphemerisData sv_PC, double RCA_m, double phi_star, double& D_BT, double& D_BR, EphemerisData &sv_PC_apo, VECTOR3& R_EM, VECTOR3& V_EM)
	{
		MATRIX3 M_EMP;
		VECTOR3 W_M, R, V, P, Q, E, S_M, T_M, R_M, B_M_u, B_M, P_u, Q_u;
		double r, v, a, e, b, b_m_star, theta_star, alpha_star, beta_star, I_st_star, BT_tar, BR_tar, BT_M, BR_M, sin_theta, cos_theta, V_star;

		//Calculate unit angular velocity vector and EMP matrix
		JPLEPH(MDGSUN, GMTBASE, 4, sv_PC.GMT, &R_EM, &V_EM, NULL, NULL);
		M_EMP = EMP_Matrix(R_EM, V_EM);

		R = mul(M_EMP, sv_PC.R);
		V = mul(M_EMP, sv_PC.V);
		W_M = _V(0, 0, 1);

		r = length(R);
		v = length(V);
		P = unit(R);
		Q = unit(V);

		a = mu_Moon / 2.0 / (v * v / 2.0 - mu_Moon / r);
		E = (R * (v * v - mu_Moon / r) - V * dotp(R, V)) / mu_Moon;
		e = length(E);
		b = a * sqrt(e * e - 1.0);

		sin_theta = b / sqrt(a * a + b * b);
		cos_theta = a / sqrt(a * a + b * b);
		S_M = P * cos_theta + Q * sin_theta;

		b_m_star = sqrt(RCA_m * (RCA_m + 2.0 * a));
		theta_star = atan(b_m_star / a);
		alpha_star = PI05 - asin(dotp(S_M, W_M));
		beta_star = PI05 - phi_star;
		I_st_star = PI05 + acos((cos(beta_star) - cos(alpha_star) * cos(theta_star)) / (sin(alpha_star) * sin(theta_star)));

		BT_tar = b_m_star * cos(I_st_star);
		BR_tar = -b_m_star * sin(I_st_star);

		T_M = unit(crossp(S_M, W_M));
		R_M = crossp(S_M, T_M);
		B_M_u = P * sin_theta - Q * cos_theta;
		B_M = B_M_u * b;
		BT_M = dotp(B_M, T_M);
		BR_M = dotp(B_M, R_M);
		D_BT = BT_tar - BT_M;
		D_BR = BR_tar - BR_M;

		//Build departure state vector
		B_M_u = T_M * cos(I_st_star) - R_M * sin(I_st_star);
		P_u = S_M * cos(theta_star) + B_M_u * sin(theta_star);
		Q_u = (S_M - P_u * cos(theta_star)) / sin(theta_star);
		V_star = sqrt(OrbMech::mu_Moon / a + 2.0 * OrbMech::mu_Moon / RCA_m);

		sv_PC_apo = sv_PC;

		sv_PC_apo.R = tmul(M_EMP, P_u * RCA_m);
		sv_PC_apo.V = tmul(M_EMP, Q_u * V_star);
	}

	double sign(double a)
	{
		if (a >= 0.0)
		{
			return 1.0;
		}
		return -1.0;
	}

	double cot(double a)
	{
		return cos(a) / sin(a);
	}

	double stumpS(double z)
	{
		double s;

		s = 0;
		if (z > 0) {
			s = (sqrt(z) - sin(sqrt(z))) / pow(sqrt(z), 3);
		}
		else if (z < 0) {
			s = (sinh(sqrt(-z)) - sqrt(-z)) / pow(sqrt(-z), 3);
		}
		else {
			s = 1.0 / 6.0;
		}
		return s;
	}

	double stumpC(double z)
	{
		double c;
		c = 0;
		if (z > 0) {
			c = (1.0 - cos(sqrt(z))) / z;
		}
		else if (z < 0) {
			c = (cosh(sqrt(-z)) - 1.0) / (-z);
		}
		else {
			c = 0.5;
		}
		return c;
	}

	double kepler_U(double dt, double ro, double vro, double a, double mu, double x0) //This function uses Newton's method to solve the universal Kepler equation for the universal anomaly.
	{
		double error2, ratio, C, S, F, dFdx, x, Z, ddFddx, delta_n;
		int n, nMax;

		error2 = 1e-8;
		nMax = 1000;
		n = 0;
		ratio = 1;
		C = 0;
		S = 0;
		F = 0;
		dFdx = 0;
		x = x0;
		while ((abs(ratio) > error2) && (n <= nMax)) {
			n = n + 1;
			Z = a * x * x;
			C = stumpC(Z);
			S = stumpS(Z);
			F = ro * vro / sqrt(mu) * x * x * C + (1.0 - a * ro) * pow(x, 3.0) * S + ro * x - sqrt(mu) * dt;
			dFdx = ro * vro / sqrt(mu) * x * (1.0 - a * x * x * S) + (1.0 - a * ro) * x * x * C + ro;
			ddFddx = (1.0 - ro * a) * (1.0 - S * Z) * x + ro * vro / sqrt(mu) * (1.0 - C * Z);
			//ratio = F / dFdx;
			delta_n = 2.0 * sqrt(abs(4.0 * pow(dFdx, 2) - 5.0 * F * ddFddx));
			ratio = 5.0 * F / (dFdx + sign(dFdx) * delta_n);
			x = x - ratio;
		}
		return x;
	}

	void f_and_g(double x, double t, double ro, double a, double& f, double& g, double mu)	//calculates the Lagrange f and g coefficients
	{
		double z;

		z = a * x * x;
		f = 1 - x * x / ro * stumpC(z);
		g = t - 1 / sqrt(mu) * pow(x, 3) * stumpS(z);
	}

	void fDot_and_gDot(double x, double r, double ro, double a, double& fdot, double& gdot, double mu)	//calculates the time derivatives of the Lagrange f and g coefficients
	{
		double z;
		z = a * x * x;
		fdot = sqrt(mu) / r / ro * (z * stumpS(z) - 1.0) * x;
		gdot = 1.0 - x * x / r * stumpC(z);
	}

	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3& R1, VECTOR3& V1, double mu, double x)	//computes the state vector (R,V) from the initial state vector (R0,V0) and the elapsed time
	{
		double r0, v0, vr0, alpha, f, g, fdot, gdot, r, xx, paratol, x0;

		//If absolute value of alpha is smaller than paratol, then the parabolic initial guess is used
		paratol = 0.00000001;

		r0 = length(R0);
		v0 = length(V0);
		vr0 = dotp(R0, V0) / r0;
		alpha = 2.0 / r0 - v0 * v0 / mu;

		//Initial guess supplied by the calling function
		if (x != 0.0)
		{
			x0 = x;
		}
		//Calculate initial guess
		else
		{
			//Initial guess for elliptical and hyperbolic orbits (and nonsensical orbits)
			if (abs(alpha) > paratol || v0 == 0.0)
			{
				x0 = sqrt(mu) * abs(alpha) * t;
			}
			//Initial guess for (near) parabolic orbits
			else
			{
				VECTOR3 H = crossp(R0, V0);
				double hmag = length(H);
				double p = hmag * hmag / mu;
				double s = 0.5 * (PI05 - atan(3.0 * sqrt(mu / (p * p * p)) * t));
				double w = atan(pow(tan(s), 1.0 / 3.0));
				x0 = sqrt(p) * (2.0 * cot(2.0 * w));
			}
		}

		xx = kepler_U(t, r0, vr0, alpha, mu, x0);
		f_and_g(xx, t, r0, alpha, f, g, mu);
		R1 = R0 * f + V0 * g;
		r = length(R1);
		fDot_and_gDot(xx, r, r0, alpha, fdot, gdot, mu);
		V1 = R0 * fdot + V0 * gdot;
	}

	void GLFDEN(double ALT, double& DENS, double& SPOS)
	{
		static const DensityTables MHGDEN = DensityTables();

		ALT *= OrbMech::R_Earth;

		if (ALT >= 700.0 * 1000.0)
		{
			DENS = 0.0;
			SPOS = MHGDEN.CSMAX;
			return;
		}

		double H, lnRho, T_M, D_apo, r;
		int i;

		if (ALT < 0)
		{
			H = 0;
			//Set index to work at sea level
			i = 7;
			goto RTCC_GLFDEN_1;
		}

		if (ALT > 90.0 * 1000.0)
		{
			r = MHGDEN.r0 + ALT;
			SPOS = MHGDEN.CSMAX;
		}
		else if (ALT == 90.0 * 1000.0)
		{
			//Set index to work at 90km
			i = 12;
			SPOS = MHGDEN.CSMAX;
			goto RTCC_GLFDEN_3;
		}
		else
		{
			H = MHGDEN.r0 * ALT / (MHGDEN.r0 + ALT);
			goto RTCC_GLFDEN_4;
		}
		//2
		i = 0;
		while (r < MHGDEN.r_b[i])
		{
			i++;
		}
		if (r == MHGDEN.r_b[i])
		{
		RTCC_GLFDEN_3:
			D_apo = 0.0;
			T_M = MHGDEN.T_M_b[i];
		}
		else
		{
			double F, G;
			T_M = MHGDEN.T_M_b[i] + MHGDEN.L_M[i] * (r - MHGDEN.r_b[i]);
			F = T_M / MHGDEN.T_M_b[i];
			G = MHGDEN.r_b[i] / r;
			D_apo = MHGDEN.B7[i] * (G - 1.0 + MHGDEN.A7[i] * log(G * F));
		}
		double ln_rho_T_M, rho_T_M;
		ln_rho_T_M = MHGDEN.ln_rho_b_T_M_b[i] - D_apo;
		rho_T_M = exp(ln_rho_T_M);
		DENS = rho_T_M / T_M;
		return;
	RTCC_GLFDEN_4:
		i = 0;
		while (H < MHGDEN.H_b[i])
		{
			i++;
		}
		if (H == MHGDEN.H_b[i])
		{
		RTCC_GLFDEN_1:
			D_apo = 0.0;
			T_M = MHGDEN.T_M_B[i];
			lnRho = MHGDEN.ln_rho_b[i];
		}
		else
		{
			double D;

			D = MHGDEN.B[i] * (H - MHGDEN.H_b[i]);
			if (MHGDEN.L_M_B[i] == 0.0)
			{
				D_apo = D;
				T_M = MHGDEN.T_M_B[i];
				lnRho = MHGDEN.ln_rho_b[i] - D_apo;
			}
			else
			{
				T_M = MHGDEN.T_M_B[i] * (1.0 + D);
				D_apo = MHGDEN.A[i] * log(1.0 + D);
				lnRho = MHGDEN.ln_rho_b[i] - D_apo;
			}
		}
		SPOS = sqrt(MHGDEN.C2 * T_M);
		DENS = exp(lnRho);
	}

	void adbar_from_rv(double rmag, double vmag, double rtasc, double decl, double fpav, double az, VECTOR3& R, VECTOR3& V)
	{
		R = _V(cos(decl) * cos(rtasc), cos(decl) * sin(rtasc), sin(decl)) * rmag;
		V.x = vmag * (cos(rtasc) * (-cos(az) * sin(fpav) * sin(decl) + cos(fpav) * cos(decl)) - sin(az) * sin(fpav) * sin(rtasc));
		V.y = vmag * (sin(rtasc) * (-cos(az) * sin(fpav) * sin(decl) + cos(fpav) * cos(decl)) + sin(az) * sin(fpav) * cos(rtasc));
		V.z = vmag * (cos(az) * cos(decl) * sin(fpav) + cos(fpav) * sin(decl));
	}

	VECTOR3 r_from_latlong(double lat, double lng)
	{
		return unit(_V(cos(lng) * cos(lat), sin(lng) * cos(lat), sin(lat)));
	}

	void latlong_from_r(VECTOR3 R, double& lat, double& lng)
	{
		VECTOR3 u;

		u = unit(R);
		lat = atan2(u.z, sqrt(u.x * u.x + u.y * u.y));
		lng = atan2(u.y, u.x);
	}

	void SStoHHMMSS(double time, int& hours, int& minutes, double& seconds)
	{
		double mins;

		time = round(time);
		hours = (int)trunc(time / 3600.0);
		mins = fmod(time / 60.0, 60.0);
		minutes = (int)trunc(mins);
		seconds = (mins - minutes) * 60.0;
	}

	MATRIX3 EMP_Matrix(VECTOR3 R, VECTOR3 V)
	{
		//Selenocentric to EMP coordinates
		VECTOR3 X_EMP, Y_EMP, Z_EMP;

		X_EMP = -unit(R);
		Z_EMP = unit(crossp(R, V));
		Y_EMP = crossp(Z_EMP, X_EMP);
		return _M(X_EMP.x, X_EMP.y, X_EMP.z, Y_EMP.x, Y_EMP.y, Y_EMP.z, Z_EMP.x, Z_EMP.y, Z_EMP.z);
	}

	double ElevationAngle(VECTOR3 R_S, VECTOR3 R)
	{
		VECTOR3 Rho, N;

		N = unit(R_S);
		Rho = unit(R - R_S);
		return asin(dotp(Rho, N));
	}

	double LunarSunElevationAngleAtMJD(const SunMoonEphemerisTable &MDGSUN, double GMTBASE, double HOUR, VECTOR3 R_S_SG)
	{
		MATRIX3 PNL;
		VECTOR3 R_EM, V_EM, R_ES, R_S, R;

		//Get ephemerides at MJD
		bool err = JPLEPH(MDGSUN, GMTBASE, 2, HOUR, &R_EM, &V_EM, &R_ES, &PNL);

		if (err) return 0.0;

		//Convert selenographic to selenocentric
		R_S = mul(PNL, R_S_SG);

		//Get vector from Moon to Sun
		R = R_ES - R_EM;

		return ElevationAngle(R_S, R);
	}

	void normalizeAngle(double& a, bool positive)
	{
		a = fmod(a, PI2);
		if (a < 0)
			a += PI2;
		if (!positive)
		{
			if (a > PI)
			{
				a -= PI2;
			}
		}
	}

	std::string FixedWidthString(std::string str, unsigned len)
	{
		if (str.length() > len) return str;

		std::string str2;
		for (unsigned i = 0; i < len - str.length(); i++)
		{
			str2.append(" ");
		}
		str2.append(str);
		return str2;
	}

	EOIProcessorDataSet GetEOIProcessorData(int Pad, int altitude)
	{
		//Pad: TBD
		//Altitude: 1 = 90 NM, 2 = 100 NM

		EOIProcessorDataSet EOIData;

		//Time from launch to EOI, seconds
		EOIData.MDLIEV[0] = 0.76673814e3;
		EOIData.MDLIEV[1] = -0.18916781e1;
		EOIData.MDLIEV[2] = 0.10202785e-1;
		EOIData.MDLIEV[3] = 0.16441395e-5;
		//Declination of EOI, deg
		EOIData.MDLIEV[4] = 0.60994221e2;
		EOIData.MDLIEV[5] = -0.36710496e0;
		EOIData.MDLIEV[6] = -0.68781953e-3;
		EOIData.MDLIEV[7] = 0.44783353e-5;
		//Longitude of EOI, deg
		EOIData.MDLIEV[8] = -0.7232328e2;
		EOIData.MDLIEV[9] = 0.55465072e0;
		EOIData.MDLIEV[10] = -0.48610649e-2;
		EOIData.MDLIEV[11] = 0.90988410e-5;
		//Inertial azimuth of EOI, deg
		EOIData.MDLIEV[12] = 0.12344699e2;
		EOIData.MDLIEV[13] = 0.13104156e1;
		EOIData.MDLIEV[14] = -0.46697509e-2;
		EOIData.MDLIEV[15] = 0.14584433e-4;
		//Flight path angle of EOI in deg
		EOIData.MDLEIC[1] = 0.0;

		
		if (altitude == 1)
		{
			//Radius of EOI in NM
			EOIData.MDLEIC[0] = 3533.934;
			//Inertial velocity of EOI, fps
			EOIData.MDLEIC[2] = 25603.875656;
		}
		else
		{
			//Radius of EOI in NM
			EOIData.MDLEIC[0] = 3.5439336e3;
			//Inertial velocity of EOI, fps
			EOIData.MDLEIC[2] = 25567.72868;
		}

		return EOIData;
	}

	void EOIProcessor(const EOIProcessorDataSet& data, double A_Z, OrbMech::SphericalCoordinates& coord, double& T_L)
	{
		A_Z *= DEG;
		T_L = data.MDLIEV[0] + data.MDLIEV[1] * A_Z + data.MDLIEV[2] * pow(A_Z, 2) + data.MDLIEV[3] * pow(A_Z, 3);
		coord.lat = data.MDLIEV[4] + data.MDLIEV[5] * A_Z + data.MDLIEV[6] * pow(A_Z, 2) + data.MDLIEV[7] * pow(A_Z, 3);
		coord.lng = data.MDLIEV[8] + data.MDLIEV[9] * A_Z + data.MDLIEV[10] * pow(A_Z, 2) + data.MDLIEV[11] * pow(A_Z, 3);
		coord.azi = data.MDLIEV[12] + data.MDLIEV[13] * A_Z + data.MDLIEV[14] * pow(A_Z, 2) + data.MDLIEV[15] * pow(A_Z, 3);
		coord.rad = data.MDLEIC[0];
		coord.fpa = data.MDLEIC[1];
		coord.vel = data.MDLEIC[2];

		coord.lat /= DEG;
		coord.lng /= DEG;
		coord.azi /= DEG;
		coord.rad /= OrbMech::ER2NM;
		coord.fpa /= DEG;
		coord.vel *= OrbMech::FPS2ERPH;
		T_L /= HRS;
	}

	void TLINominalMissionPolynomial(double C3, double sigma, double delta, double FW, double R_I, double& DV, double& alpha, double& beta, double& eta_alpha, double& R_P)
	{
		//INPUT:
		//C3 - twice vis-viva energy desired at cutoff, (e.r./hr)^2
		//sigma - radians
		//delta - radians
		//FW - lbf/lbm
		//R_I - e.r.

		static const double TLI_POLY[5][26] = { { 0.61967804e-1, 0.86219648e-2, -0.24371820e2, 0.41004848e4, -0.99229657e6, 0.14267564e9, -0.54688962e0, -0.78766288e0, 0.10261969e2, 0.52445599e1, -0.15527983e5,
	0.51931839e7, 0.18005069e0, 0.97069489e-1, 0.61442230e1, -0.87765197e3, -0.16502383e0,  0.63224468e0, 0.81844028e3,  -0.33505204e0, -0.92426341e-1, -0.18131458e4, -0.39193696e4},
	{ 0.51541772e0, -0.15528032e0, 0.27185659e2, 0.18763984e3, -0.92712145e6, 0.21114994e9, -0.56424215e0, 0.95105384e1, -0.15294910e2, 0.33896643e2, -0.26903240e5, 0.12131396e8, 0.25371175e0,
		0.22036833e0,  -0.22601576e2, 0.14378586e4, 0.31264540e0, -0.64046690e1,-0.39254760e4, -0.57987931e0, -0.22905910e0, 0.12621438e4, 0.70516077e4, -0.76940409e-4, 0.64393915e-4, 0.48483478e-4 },
	{ 0.48329414e0, 0.18759385e-2, 0.14031932e1, -0.13933485e3, 0.40515931e5, -0.48676865e7, -0.10155877e1, 0.83266987e-1, -0.28021958e1, 0.21207686, 0.98814614e3, -0.17699125e6,
		0.30964851, 0.13152495, 0.92808415, -0.32524984e2, 0.44675108e-2, -0.59053312e-3, -0.10061669e3, -0.60405621, 0.96317404e-2, 0.18026336e3, 0.81684373e2 },
	{ 0.46986962e-2, -0.44724715e-2, -0.17477015e1, 0.16880382e2, 0.14554490e5, -0.27167564e7, 0.25758493e-1, -0.77608981e-1, 0.57075666e0, 0.36716041e1, -0.96377142e3, -0.56658473e5,
		0.61201761e-2, 0.72885987e-2, 0.55059398e0, 0.44179572e1, -0.10348462e-1, 0.49107017e-1,0.44830843e3, 0.98814646e0, -0.73420686e-2, -0.27739134e2, 0.29172916e3},
	{ 0.18679213e1, 0.98320266e0, 0.25028715e2, -0.17104963e4, 0.37348295e6, -0.51521225e8, -0.21640574e0, 0.41744541e0,-0.65859807e1, -0.57578939e1, 0.99505664e4, -0.20041678e7,
		 -0.76178399e-2, -0.14737246e-2, -0.43988981e1, 0.20554193e3, 0.74293949e-1, -0.29415058e0, -0.16188456e4, -0.30007513e-1, 0.28463657e-1, 0.79392771e3, 0.12182700e4 } };

		double DV_M, DI, X1, X2, X3, X4, X5;

		DV_M = sqrt(C3 + 2.0 * mu_Earth / R_I) - sqrt(mu_Earth / R_I);
		DI = asin(sin(abs(delta)) / sin(sigma + 0.314));
		X1 = DV_M - 1.75;
		X2 = DI * DI - 0.0027;
		X3 = sigma - 0.148;
		X4 = 1.0 / FW;
		X5 = R_I;

		double Y[5];

		for (int i = 0; i < 5; i++)
		{
			Y[i] = TLI_POLY[i][0] + TLI_POLY[i][1] * X1 + TLI_POLY[i][2] * X2 + TLI_POLY[i][3] * pow(X2, 2) + TLI_POLY[i][4] * pow(X2, 3) + TLI_POLY[i][5] * pow(X2, 4) +
				TLI_POLY[i][6] * X3 + TLI_POLY[i][7] * pow(X3, 2) + TLI_POLY[i][8] * X1 * X2 + TLI_POLY[i][9] * X2 * X3 + TLI_POLY[i][10] * pow(X2, 2) * X3 +
				TLI_POLY[i][11] * pow(X2, 3) * X3 + TLI_POLY[i][12] * X4 + TLI_POLY[i][13] * X1 * X4 + TLI_POLY[i][14] * X2 * X4 + TLI_POLY[i][15] * X2 * X2 * X4 +
				TLI_POLY[i][16] * X3 * X4 + TLI_POLY[i][17] * X3 * X3 * X4 + TLI_POLY[i][18] * X2 * X2 * X3 * X4 + TLI_POLY[i][19] * X5 + TLI_POLY[i][20] * X1 * X3 +
				TLI_POLY[i][21] * X1 * X2 * X2 + TLI_POLY[i][22] * X1 * X2 * X2 * X3;
		}

		alpha = Y[0];
		beta = Y[1] + (X4 * X4 * (TLI_POLY[1][23] + TLI_POLY[1][24] * X1 + TLI_POLY[1][25] * X4 * X4)) / (pow(X3 + 0.148, 2) + 4.0 * pow(X2 + 0.0027, 2));
		eta_alpha = Y[2];
		R_P = Y[3];
		DV = Y[4];
	}

	void TLIBRN(EphemerisData sv1, double C3, double sigma, double delta, double F, double F_I, double W_I, double WDOT, double T_MRS, TLIBRNOutput& out)
	{
		double DV_I, DT_B1, V_I, R_I, phi_dot_I, dphi_B1, DT_B2, dphi_B2, delta0, ddelta, W;
		double alpha0, beta0, eta_alpha0, R_P0, DV0;
		double alpha, beta, etaalpha, eta, R_P, DV, T_B;

		R_I = length(sv1.R);
		V_I = length(sv1.V);

		DV_I = sqrt(mu_Earth / R_I) - V_I;
		DT_B1 = DV_I / (F / W_I);

		phi_dot_I = V_I / R_I;
		dphi_B1 = phi_dot_I * DT_B1;

		DT_B2 = (1.0 - F_I / F) * T_MRS;
		dphi_B2 = phi_dot_I * DT_B2;
		W = W_I - WDOT * DT_B1;

		if (delta > 2.0 * RAD)
		{
			delta0 = 2.0 * RAD;
			ddelta = delta - 2.0 * RAD;
		}
		else if (delta < -2.0 * RAD)
		{
			delta0 = -2.0 * RAD;
			ddelta = -2.0 * RAD - delta;
		}
		else
		{
			delta0 = delta;
			ddelta = 0.0;
		}

		TLINominalMissionPolynomial(C3, sigma, delta, F / W, R_I, DV0, alpha0, beta0, eta_alpha0, R_P0);

		//Account for possible out-of-plane
		alpha = alpha0 + 4.66 * ddelta;
		beta = beta0 - 2.15 * ddelta;
		etaalpha = eta_alpha0 + 0.923 * ddelta;
		R_P = R_P0 - 0.442 * ddelta;
		DV = DV0 + 6.33 * ddelta;

		//Account for mixture ratio shift
		alpha += dphi_B1 + dphi_B2;
		beta += 3.0 / 4.0 * dphi_B1 + dphi_B2;
		eta = etaalpha - alpha;
		DV += DV_I;

		//Calculate output state vector
		T_B = W_I / WDOT * (1.0 - exp(-(DV / G0ER * WDOT / F))) + DT_B2;

		VECTOR3 R_I_u, N_I_u, S, N_c;
		double C1, p, e, R_c, V_c, gamma_c;

		R_I_u = unit(sv1.R);
		N_I_u = unit(crossp(sv1.R, sv1.V));
		out.T = R_I_u * cos(delta) * cos(alpha) + crossp(N_I_u, R_I_u) * cos(delta) * sin(alpha) + N_I_u * sin(delta);
		S = R_I_u * cos(beta) + crossp(N_I_u, R_I_u) * sin(beta);
		N_c = unit(crossp(out.T, S));
		C1 = R_P * sqrt(2.0 * mu_Earth / R_P + C3);
		p = C1 * C1 / mu_Earth;
		e = sqrt(1.0 + C3 / mu_Earth * p);
		R_c = p / (1.0 + e * cos(eta));
		V_c = mu_Earth / C1 * sqrt(1.0 + 2.0 * e * cos(eta) + e * e);
		gamma_c = atan2(e * sin(eta), 1.0 + e * cos(eta));
		out.sv2.R = (out.T * cos(sigma + eta) + crossp(N_c, out.T) * sin(sigma + eta)) * R_c;
		out.sv2.V = (-out.T * sin(sigma + eta - gamma_c) + crossp(N_c, out.T) * cos(sigma + eta - gamma_c)) * V_c;
		out.sv2.GMT = sv1.GMT + T_B;
		out.sv2.RBI = sv1.RBI;
		out.W2 = W - WDOT * (T_B - DT_B2);
		out.alpha = alpha;
		out.beta = beta;
		out.DV = DV;
	}

	void BURN(VECTOR3 R, VECTOR3 V, double dv, double dgamma, double dpsi, double isp, double& dv_R, double& mfm0, VECTOR3& RF, VECTOR3& VF)
	{
		double r, v;
		RF = R;

		r = length(R);
		v = length(V);

		VECTOR3 Rdot1, Rdot2;
		double d, h;

		d = dotp(R, V);
		h = length(crossp(R, V));
		Rdot1 = V * cos(dgamma) + (R * v * v - V * d) / h * sin(dgamma);
		Rdot2 = R * 2.0 * dotp(R, Rdot1) / r / r * pow(sin(dpsi / 2.0), 2) + Rdot1 * cos(dpsi) - crossp(R, Rdot1) / r * sin(dpsi);
		VF = Rdot2 * (1.0 + dv / v);
		dv_R = sqrt(dv * dv + 4.0 * v * (v + dv) * (pow(sin(dgamma / 2.0), 2) + (h * h * cos(dgamma) - h * d * sin(dgamma)) / (r * r * v * v) * pow(sin(dpsi / 2.0), 2)));
		mfm0 = exp(-dv_R / (isp * OrbMech::G0ER));
	}

	void LUPInvert(double** A, int* P, int N, double** IA) {

		for (int j = 0; j < N; j++) {
			for (int i = 0; i < N; i++) {
				if (P[i] == j)
					IA[i][j] = 1.0;
				else
					IA[i][j] = 0.0;

				for (int k = 0; k < i; k++)
					IA[i][j] -= A[i][k] * IA[k][j];
			}

			for (int i = N - 1; i >= 0; i--) {
				for (int k = i + 1; k < N; k++)
					IA[i][j] -= A[i][k] * IA[k][j];

				IA[i][j] = IA[i][j] / A[i][i];
			}
		}
	}

	int LUPDecompose(double** A, int N, double Tol, int* P)
	{

		int i, j, k, imax;
		double maxA, * ptr, absA;

		for (i = 0; i <= N; i++)
			P[i] = i; //Unit permutation matrix, P[N] initialized with N

		for (i = 0; i < N; i++) {
			maxA = 0.0;
			imax = i;

			for (k = i; k < N; k++)
				if ((absA = fabs(A[k][i])) > maxA) {
					maxA = absA;
					imax = k;
				}

			if (maxA < Tol)
			{
				return 0; //failure, matrix is degenerate
			}

			if (imax != i) {
				//pivoting P
				j = P[i];
				P[i] = P[imax];
				P[imax] = j;

				//pivoting rows of A
				ptr = A[i];
				A[i] = A[imax];
				A[imax] = ptr;

				//counting pivots starting from N (for determinant)
				P[N]++;
			}

			for (j = i + 1; j < N; j++) {
				A[j][i] /= A[i][i];

				for (k = i + 1; k < N; k++)
					A[j][k] -= A[j][i] * A[i][k];
			}
		}

		return 1;  //decomposition done
	}

	void LeastSquares(double* x, double* y, int n, int m, double* beta)
	{
		int i, j, k;

		double** X = new double* [n];
		for (i = 0; i < n; i++)
		{
			X[i] = new double[m];
		}

		double** XTX = new double* [m];
		double** XTXI = new double* [m];
		for (i = 0; i < m; i++)
		{
			XTX[i] = new double[m];
			XTXI[i] = new double[m];
		}

		double** XTXIXT = new double* [m];
		for (i = 0; i < m; i++)
		{
			XTXIXT[i] = new double[n];
		}
		int* P = new int[m + 1];

		//Get X matrix (n x m)
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < m; j++)
			{
				X[i][j] = pow(x[i], j);
			}
		}

		//Get XTX matrix (m x m)
		for (i = 0; i < m; i++)
		{
			for (j = 0; j < m; j++)
			{
				XTX[i][j] = 0.0;

				for (k = 0; k < n; k++)
				{
					XTX[i][j] += X[k][i] * X[k][j];
				}
			}
		}

		//Invert XTX (m x m)
		LUPDecompose(XTX, m, 0.0, P);
		LUPInvert(XTX, P, m, XTXI);

		//Multiply with XT (m x n)
		for (i = 0; i < m; i++)
		{
			for (j = 0; j < n; j++)
			{
				XTXIXT[i][j] = 0.0;

				for (k = 0; k < m; k++)
				{
					XTXIXT[i][j] += XTXI[k][i] * X[j][k];
				}
			}
		}

		//Get vector
		for (i = 0; i < m; i++)
		{
			beta[i] = 0.0;
			for (j = 0; j < n; j++)
			{
				beta[i] += XTXIXT[i][j] * y[j];
			}
		}

		delete[] X;
		delete[] XTX;
		delete[] XTXI;
		delete[] XTXIXT;
		delete[] P;
	}
}