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
#include <vector>
#include "DataTables.h"

struct QRTPTLIGeneralizedIteratorArray
{
	//INPUTS:
	//Launch azimuth
	double AZ;
	//GMT launch time
	double LTS;
	//Delta time from launch to translunar injection
	double TC;
	//Injection specific energy
	double C3;
	//Perigee to target vector angle
	double sigma;
	//TLI plane change
	double delta;
	double RCA_M;
	double phi_star;
	double RCA_E;
	double DT_TLI_LOI;
	//TLI ignition vector used in hypersurface search
	OrbMech::EphemerisData sv_TLI_Ignition_Hyper;

	//OUTPUTS:
	double alpha;
	double beta;
	double D_BT_M;
	double D_BR_M;
	VECTOR3 R_EM;
	double D_BT_E;
	double TLI_DV;

	OrbMech::EphemerisData sv_EOI;
	//OrbMech::EphemerisData sv_TB6;
	OrbMech::EphemerisData sv_TLI_Ignition;
	double Weight_TLI_Ignition;
	OrbMech::EphemerisData sv_TLI_Cutoff;
	double Weight_TLI_Cutoff;
	VECTOR3 T_u;
	OrbMech::EphemerisData sv_PC;
	OrbMech::EphemerisData sv_PC2;
	OrbMech::EphemerisData sv_PG;

	//CONSTANTS:
	int Epoch, Year, Month, Day, RefDay;
	double BHA;
	double GMTBASE;
	//Time from liftoff to insertion
	double DT_L;
	OrbMech::SphericalCoordinates coord_EOI;
	double Area_SIVB, Weight_Insertion;
	bool FirstOpportunity;
	bool HyperSurfaceSearch;
	double T_MRS; //hrs
	double F_SIVB; //Thrust leve post MRS, lbf
	double F_I_SIVB; //Thrust level pre MRS, lbf
	double WDOT_SIVB;
};

struct QRTPTLMCGeneralizedIteratorArray
{
	//INPUTS
	double dv_TLMC;
	double dgamma_TLMC;
	double dpsi_TLMC;

	//OUTPUTS
	double dv_T_TLMC;
	OrbMech::EphemerisData sv_TLMC_apo, sv_nd, sv_PG;
	double lat_nd, lng_nd, r_nd;

	//CONSTANTS
	OrbMech::EphemerisData sv_TLMC;
	double GMT_nd;
};

struct TLIStateTape
{

};

class QuickResponseTargetingProgram
{
public:
	QuickResponseTargetingProgram();

	void LoadLVTargetingObjectives(std::string filename);
	void GetPreprocessorPlotData(int type, std::vector<double> &xdata1, std::vector<double> &ydata1, std::vector<double>& xdata2, std::vector<double>& ydata2) const;
	void RunTargetingOption(std::string project, PerformanceData perf, int day, bool freereturn, int splittime, int AltitudeOption, MSFCPresetTape &presets);
	void RunTLMCOption();

	bool TLITrajectoryProcessor(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode);
	bool TLMCTrajectoryProcessor(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode);

	std::vector<LVTargetingObjectivesSet> objectives;
protected:

	bool SetupBasics(int Year, int Month, int Day);
	bool CoplanarSearch();
	bool HypersurfaceSearch();
	bool PlaneChangeSearch();

	void GETSET(int entry, const LVTargetingObjectivesSet &obj);
	void HYPSRF(int entry);
	//Computes, stores, prints and punches TLI presettings
	void OPPEND(int LAZ, int opp);
	void PLOTUtility(FILE* file, std::string filename, std::string title, std::string xname, int xtype, std::string yname, int ytype, int opp) const;
	void PLOT();
	void AnalyticalLaunchAzimuthModel(MSFCPresetTape &preset);
	double VariableLaunchAzimith(const MSFCPresetTape& tape, double T_L) const;

	void WritePresetTape(const MSFCPresetTape &tape, std::string filename);
	void WritePresettings(int AltitudeOption, const MSFCPresetTape& tape, std::string filename);

	OrbMech::SunMoonEphemerisTable MDGSUN;
	OrbMech::SIVBVentingTable VentTable;
	bool IsFreeReturn;
	int SplitTimeOption; //1 = Optimum 1st opportunity, 2 = optimum 2nd opportunity, 3 = split time

	EnckeFreeFlightIntegrator encke;
	QRTPTLIGeneralizedIteratorArray iter_arr;

	const double DT_GRR = 17.0 / HRS;
	const double DT_TST_TB6 = 10.0*60.0;
	const double DT_TB6_TIG = 580.5 / HRS;
	const double ER2HR2ToKM2SEC2 = pow(OrbMech::R_Earth / 3600000.0, 2);

	double lambda_L;

	//Translunar ellipse normal vector from free-return search
	VECTOR3 N_1;
	//Perigee vector at injection from free-return search
	VECTOR3 P_E;

	//Saved data for OPPEND
	struct SaveData
	{
		//TLI ignition state vector
		OrbMech::EphemerisData sv_TLI_Ignition;
		//TLI cutoff state vector
		OrbMech::EphemerisData sv_TLI_Cutoff;
	} HBLOCK;

	struct OutputData
	{
		double LaunchAzimuth = 0.0; //deg
		double LaunchTime; //sec
		double RAO; //deg
		double DEC; //deg
		double RAS; //deg
		double COS_SIGMA;
		double C3_KM; //(km/sec)**2
		double C3_FT; //(ft/sec)**2
		double EN; //Eccentricity

		double RN; //ft, reignition radius
		double DT_EOI_TB6; //sec, time from parking orbit insertion to ullage
		double DT_MRS_TLI; //sec, time from mixture ratio shift to cutoff
		double TAU; //sec, estimated time to deplete vehicle mass from assumed mixture ratio shift
		VECTOR3 T_u; //Unit target vector
		double F; //deg, true anomaly
		double ALPHA; //deg
		double BETA; //deg

		double GMT_PC; //sec
		double delta_TLI; //deg, plane change during TLI
		double DV_TLI; //ft/sec, DV of TLI
	};

	struct OutputDataArray
	{
		OutputData data[2][15];
		unsigned int num; //Number of cases available
	} OUTPUT;
};