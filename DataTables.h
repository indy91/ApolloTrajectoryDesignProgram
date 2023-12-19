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

struct SaveFileData
{
	SaveFileData::SaveFileData()
	{
		for (int i = 0; i < 30; i++)
		{
			dValues[i] = 0.0;
			iValues[i] = 0;
		}
		LVTargetingObjectivesFile[0] = '\0';
		PresetTapeFile[0] = '\0';
	}

	double dValues[30];
	int iValues[30];
	char LVTargetingObjectivesFile[64];
	char PresetTapeFile[64];
};

struct PerformanceData
{
	double CSMWeight; //lbs
	double SPSPropellant; //lbs
	double LMWeight; //lbs
	double SIVBWeight; //lbs, at insertion
	double T_MRS; //Time of mixture ratio change from ignition, hours
	double TH1; //S-IVB thrust level pre MRS, lbf
	double TH2; //S-IVB thrust level post MRS, lbf
	double MF1; //S-IVB mass flow rate pre MRS, lbs/hr
	double MF2; //S-IVB mass flow rate post MRS, lbs/hr
};

struct SFPDataSet
{
	int Year;
	int Day;
	int Opportunity;
	double LaunchAzimuth;
	int Window; //1 = Pacific window, 2 = Atlantic window
	double lat_PC1;
	double lng_PC1;
	double h_PC1;
	double GET_TLI;
	double lat_PC2;
	double lng_PC2;
	double h_PC2;
	double dpsi_LOI;
	double gamma_LOI;
	double T_LO;
	double DT_LLS;
	double ApproachAzimuth;
	double lat_LLS;
	double lng_LLS;
	double R_LLS;
	double dpsi_TEI;
	double dv_TEI;
	double T_TE;
	double I_FR;
	double GMT_PC1;
	double GMT_PC2;
	double GMT_ND;
	double lat_ND;
	double lng_ND;
	double h_ND;

	//Additional data to save
	double lat_spl; //Splashdown latitude
	double lng_spl; //Splashdown longitude
};

struct LVTargetingObjectives
{
	int Year = 0;
	int Month;
	int Day;
	int Opportunity;

	double LTS; //GMT launch time (from midnight)
	double TC; //Delta time from launch to translunar injection
	double C3; //Injection specific energy
	double AZ; //Launch azimuth
	double LONM; //Selenographic longitude of pericynthion
	double SLAW; //LM adapter weight
	double RCAM; //Radius of pericynthion
	double LATM; //Selenographc latitude of pericynthion

	double TF; //Delta time from TLI to LOI
};

struct LVTargetingObjectivesSet
{
	LVTargetingObjectives set[2]; //2nd and 1st opportunities
};

struct EOIProcessorDataSet
{
	double MDLIEV[16], MDLEIC[3];
};

struct MSFCPresetTape
{
	MSFCPresetTape()
	{
		Day = Month = Year = 0;
		TLO = RAO = 0.0;
		for (int i = 0; i < 53; i++)
		{
			AZ[i] = TD_AZ[i] = 0.0;
		}
		for (int i = 0; i < 15; i++)
		{
			RAS1[i] = DEC1[i] = C31[i] = COS1[i] = EN1[i] = 0.0;
			RAS2[i] = DEC2[i] = C32[i] = COS2[i] = EN2[i] = 0.0;
			TP[i] = 0.0;
		}
		NumAzi = 0;
		ALPHATS1 = BETA1 = RN1 = TST1 = F1 = T3R1 = TAU3R1 = DVBR1 = 0.0;
		ALPHATS2 = BETA2 = RN2 = TST2 = F2 = T3R2 = TAU3R2 = DVBR2 = 0.0;
		AZO = AZS = 0.0;
		for (int i = 0; i < 7; i++)
		{
			f[i] = g[i] = 0.0;
		}
		for (int i = 0; i < 5; i++)
		{
			h1[i] = h2[i] = h3[i] = 0.0;
		}
		TDS1 = TDS2 = TDS3 = 0.0;
		TD1 = TD2 = TD3 = 0.0;
		TSD1 = TSD2 = TSD3 = 0.0;
	}

	int Day;
	int Month;
	int Year;
	double TLO; //sec, reference launch time (midnight)
	double RAO; //theta_EO, deg, reference launch site right ascension (positive east of vernal equinox)
	double AZ[53]; //deg, table of values from launch azimuth from analytic launch azimuth model
	double TD_AZ[53]; //sec, table of values for TD corresponding values of AZ
	double RAS1[15];
	double DEC1[15];
	double C31[15];
	double COS1[15];
	double EN1[15];
	double RAS2[15];
	double DEC2[15];
	double C32[15];
	double COS2[15];
	double EN2[15];
	int NumAzi;
	double TP[15]; //Table of values of t_d, independent variable for all out-of-orbit targeting tables, both TLI opportunities, sec
	double ALPHATS1;
	double BETA1;
	double RN1;
	double TST1;
	double F1;
	double T3R1;
	double TAU3R1;
	double DVBR1;
	double ALPHATS2;
	double BETA2;
	double RN2;
	double TST2;
	double F2;
	double T3R2;
	double TAU3R2;
	double DVBR2;
	double AZO;
	double AZS;
	double f[7];
	double g[7];
	double h1[5]; //deg
	double h2[5]; //deg
	double h3[5]; //deg
	double TDS1;
	double TDS2;
	double TDS3;
	double TD1;
	double TD2;
	double TD3;
	double TSD1;
	double TSD2;
	double TSD3;
};