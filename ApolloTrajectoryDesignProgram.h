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

#include "DataTables.h"
#include "EnckeIntegrator.h"
#include "TLIFirstGuess.h"
#include "OrbMech.h"
#include <string>
#include <vector>

struct FirstGuessLogicDisplay
{
	double Launchtime = 0.0; //GMT
	double TLItime = 0.0; //GET
	double PCtime = 0.0; //GET
	double LandingTime = 0.0;
	double LandingSunElevation = 0.0;
};

struct ConvergedMissionDisplay
{
	double Launchtime = 0.0; //GMT

	double TLIIgnitionTime = 0.0; //GET
	double TLICutoffTime = 0.0; //GET
	double dv_TLI = 0.0; //ft/s

	double GET_PC = 0.0; //GET
	double GMT_PC = 0.0; //GMT
	double lat_PC = 0.0; //deg

	double dv_LOI1 = 0.0; //ft/s
	double dpsi_LOI1 = 0.0; //deg
	double m_LOI1 = 0.0; //lbs

	double T_LOI2 = 0.0; //GET
	double dv_LOI2 = 0.0; //ft/s
	double m_LOI2 = 0.0; //lbs

	double LandingTime = 0.0; //GET
	double LandingSunElevation = 0.0; //deg
	double azi_approach = 0.0; //deg

	double LOPCTime = 0.0; //GET
	double dv_LOPC = 0.0; //ft/s
	double m_LOPC = 0.0; //lbs

	double AscentTime = 0.0; //GET

	double TEITime = 0.0; //GET
	double dv_TEI = 0.0; // ft/s
	double dpsi_TEI = 0.0; //deg
	double m_TEI = 0.0; //lbs

	double m_SPS_remain = 0.0; //lbs
	double dv_remain = 0.0; //ft/s

	double EITime = 0.0; //GET
	double EIInclination = 0.0; //deg

	double lat_spl = 0.0; //deg
	double lng_spl = 0.0; //deg

	double dv_Total = 0.0; //ft/s

	double dv_PC2 = 0.0; //ft/s
	double EIInclination_PC2 = 0.0; //deg
};

struct OptimizedFullMissionInputs
{
	int Year;
	int Month;
	int Day;
	double Azi; //Launch azimuth
	int AlitudeOption; //1 = 90 NM, 2 = 100 NM parking orbit
	int Opportunity; //1 or 2
	int Window; //1 = Pacific window, 2 = Atlantic window

	double DT_TLI_LOI; //Time from TLI cutoff to LOI
	bool FreeReturn;

	//Geometry at landing site
	double lat_LLS; //Latitude of the landing site, degrees
	double lng_LLS; //Longitude of the landing site, degrees
	double H_LLS; //Elevation of the landing site, nautical miles
	double DW; //Angle of perilune from the landing site, negative if the site is post-perilune, degrees
	double psi_DS; //Approach azimuth, degrees

	//Geometry of first LPO
	double H_ALPO; //Apoapsis altitude, NM
	double H_PLPO; //Periapsis altitude, NM
	int R1; //Orbits in first LPO

	//Geometry of second LPO
	double h_a; //Apoapsis altitude, NM
	double h_p; //Periapsis altitude, NM
	int R2; //Orbits in second LPO

	//LOPC
	int REVS3; //Orbits from landing to LOPC
	int REVS4; //Orbits from LOPC to lunar liftoff
	int REVS5; //Orbits from lunar liftoff to TEI

	//Return
	double MaxInclination; //Maximum Earth-referenced inclination of the powered return trajectory, degrees
	double lambda_IP; //Desired splashdown longitude, degrees
	double DT_TEC; //Estimate time from TEI to EI, hours
};

struct ATDPGeneralizedIteratorArray
{
	//Input
	//Spherical coordinates at EOI
	OrbMech::SphericalCoordinates coord_EOI;
	OrbMech::EphemerisData sv_TEI;

	//Independent variables
	double T_L;
	double DT_EPO;
	double C3;
	double sigma; //Optional?
	double dpsi_LOI;
	double DT_LPO;
	double dv_TEI;
	double dgamma_TEI;
	double dpsi_TEI;
	double DT_TEI;

	//Dependent variables
	double D_BT_M;
	double D_BR_M;
	double D_BT_E;
	double dv_R_TEI;
	double incl_pl;

	//Targets
	double RCA_M; //Radius of pericynthion
	double phi_star; //Latitude of pericynthion
	double RCA_E;	//Radius of Earth return perigee
	double lat_LLS;
	double lng_LLS;
	double psi_DS;
	double lng_spl_des; //Desired splashdown longitude

	//Lunar orbit geometry
	double R_ALPO;
	double R_PLPO;
	double h_a;
	double h_p;
	double R_LLS;
	int REVS1; //LOI-1 to LOI-2
	int REVS2; //LOI-2 to landing
	double DW; //Angle from perilune to landing site
	int REVS3; //Orbits from landing to LOPC
	int REVS4; //Orbits from LOPC to lunar liftoff
	int REVS5; //Orbits from lunar liftoff to TEI

	//Reentry constraints
	double MaxInclination;

	//Option flags
	bool FirstOpportunity;

	//Calculation mode
	bool FreeReturn;
	bool LunarOrbitPhase;

	//Constants
	double GMTBASE;
	double F_SIVB;
	double F_I_SIVB;
	double WDOT_SIVB;
	double T_MRS;
	double Weight_Insertion;
	double Area_SIVB;
	double DT_L;
	double m_CSM;
	double m_LM;
	double isp_SPS;
	double isp_DPS;
	double dv_loi_cal;
	double dv_tei_cal;

	//Output
	OrbMech::EphemerisData sv_EOI, sv_TLI_Ignition, sv_TLI_Cutoff;
	OrbMech::EphemerisData sv_PC; //Actual pericynthion state vector
	OrbMech::EphemerisData sv_PC2; //Adjusted pericynthion state vector for better Earth return convergence
	OrbMech::EphemerisData sv_LOI2; //State vector after LOI-2
	OrbMech::EphemerisData sv_LLS; //State vector at landing site
	OrbMech::EphemerisData sv_LOPC; //State vector after LOPC
	OrbMech::EphemerisData sv_LLS2; //Second pass over the landing site
	OrbMech::EphemerisData sv_EI; //Entry interface state vector
	OrbMech::EphemerisData sv_PG; //Earth return perigee state vector
	double Weight_TLI_Ignition, Weight_TLI_Cutoff;
	double dv_TLI;
	VECTOR3 T_u; //Hypersurface target vector
	VECTOR3 R_EM; //Unit vector along the Earth-to-Moon line at periselenum arrival
	double azi_approach;
	double dv_LOI1;
	double dv_LOI2;
	double dpsi_LOPC;
	VECTOR3 DV_LOPC;
	double dv_Total;
	double lat_pl; //Actual EMP latitude of pericynthion
	double lng_pl; //Actual EMP longitude of pericynthion
	double h_pl; //Actual height of pericynthion

	double m_loi1; //Mass after LOI-1
	double m_loi2; //Mass after LOI-2
	double m_lopc; //Mass after LOPC
	double m_tei; //Mass after TEI

	double lat_ip_pr, lng_ip_pr; //Splashdown coordinates after TEI
	double incl_pr; //Powered return inclination
};

class ApolloTrajectoryDesignProgram
{
public:
	ApolloTrajectoryDesignProgram();

	//Step 1: Landing site sun elevation evaluation
	double LunarSunElevationAngle(int Year, int Month, int Day, double Hour, double Lat_SG, double Lng_SG);
	//Step 2: First guess logic
	FirstGuessLogicDisplay CalculateFirstGuessLogic(int Year, int Month, int Day, double Azi, int AlitudeOption, int Window, int Opportunity, int Orbits, double Lat_SG, double Lng_SG);
	//Step 3: Optimize full mission
	int OptimizedFullMission(const PerformanceData &perf, const OptimizedFullMissionInputs &in, ConvergedMissionDisplay &out);

	void CalculateLaunchData(const PerformanceData& perf, std::string project, std::string preset_filename, MSFCPresetTape & presettings, double *data);

	//Trajectory computers
	bool IntegratedTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode);
	bool DPSDVMonitorTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode);
	bool TransearthInjectionTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode);

	void ExportLVDataSet(std::string project) const;
	void ExportSFPDataSets(std::string project) const;
	void ExportRTCCInitFile(int Year, int Month, int Day) const;

	void SaveData();
	void DeleteStoredData();

	std::vector<LVTargetingObjectives>* GetLVTargetingDataTable() { return &LVTargetingDataTable; }

	std::vector<std::string> DebugMessages;
protected:
	bool SetupBasics(int Year, int Month, int Day);
	int CIST(double Hour, OrbMech::SphericalCoordinates coord_EOI, int Window, int Opportunity, TLIFirstGuessOutputs &out);
	bool ConvergeInitialGuessTrajectory(bool FreeReturn); //Constraints are radius, latitude and time at pericynthion
	bool ConvergeOptimizedFullMission(bool FreeReturn, double DT_TLI_LOI); //Constraints are radius, latitude and time at pericynthion
	bool ConvergeDPSAbort();
	bool ConvergeTransearthInjection(bool Optimize, bool ConvergeLongitude, double DT_TEI, double dv, double dpsi, double DT_TEC);
	OrbMech::EphemerisData GetNearestTEIPoint(OrbMech::EphemerisData sv);

	//Backwards propagate state vector with specified approach azimuth at the landing site to LOI
	void PRCOMP(OrbMech::EphemerisData sv_node, double psi_LLS, OrbMech::EphemerisData& sv_LPO1, OrbMech::EphemerisData& sv_LPO2, OrbMech::EphemerisData& sv_LLS) const;
	double LSTime(VECTOR3 U_H, double T_PCYN, VECTOR3 U_PC, double R_ALPO, double R_PLPO, double a_LLS, double e_LLS, int R1, int R2, double& dt3, double &DA) const;
	double DELTAT(double a, double e, double eta, double deta) const;
	OrbMech::EphemerisData BACKUP(OrbMech::EphemerisData sv_LLS, double dt3, double a_LLS, double DA) const;
	bool CTBODY(VECTOR3 R0, VECTOR3 V0, double GMT0, double GMTF, int K, double mu, VECTOR3& RF, VECTOR3& VF) const;
	bool CTBODY(VECTOR3 R0, VECTOR3 V0, double GMT0, double GMTF, int K, double mu, double& alpha, double& F1, double& F2, double& F3, double& F4, VECTOR3& RF, VECTOR3& VF) const;
	void ELEMT(VECTOR3 R, VECTOR3 V, double mu, VECTOR3& H, double& a, double& e, double& i, double& n, double& P, double& eta) const;
	void FCOMP(double a, double& F1, double& F2, double& F3, double& F4) const;
	bool LIBRAT(VECTOR3& R, double GMT, int K) const;
	bool LIBRAT(VECTOR3& R, VECTOR3& V, double GMT, int K) const;
	void LOPC(VECTOR3 R0, VECTOR3 V0, double GMT0, VECTOR3 L, int m, int n, double P, VECTOR3& R3, VECTOR3& V3, double& GMT3, double& mfm0, double& dpsi, VECTOR3& DV, double &t_L) const;
	double MCOMP(double m0, double dv, double isp) const;
	void RNTSIM(VECTOR3 R, VECTOR3 V, double GMT, double lng_L, double& lat, double& lng, double& dlng) const;
	void RVIO(bool vecinp, VECTOR3& R, VECTOR3& V, double& r, double& v, double& theta, double& phi, double& gamma, double& psi) const;

	double AzimuthDifference(VECTOR3 R_u, VECTOR3 U1, VECTOR3 U2) const;
	double VariableLaunchAzimith(const MSFCPresetTape& tape, double T_L) const;

	LVTargetingObjectives WriteLVTargetingObjectives(double LaunchAzimuth, int Window) const;
	SFPDataSet WriteSFPTable(double LaunchAzimuth, int Window) const;
	void ExportSFPDataSet(std::ofstream &file, std::string project, const SFPDataSet& set) const;
	bool LoadMSFCPresetTape(std::string filename, MSFCPresetTape &tape) const;
	void ConvertPresetTapeToRTCCPunchCards(PerformanceData perf, const std::vector<MSFCPresetTape>& tapes, std::string filename) const;

	//INTERNAL
	int Epoch;		//Year that defines the coordinate system
	int Year;		//Launch year
	int Month;		//Launch month
	int Day;		//Day of launch month
	int RefDay;		//Year in year of launch
	double GMTBASE; //MJD at midnight of launch
	double BHA;		//Greenwich hour angle at midnight of launch

	//CONSTANTS
	OrbMech::SunMoonEphemerisTable MDGSUN;
	OrbMech::SIVBVentingTable VentTable;

	const double gamma_reentry;
	const double Reentry_range;
	const double Reentry_dt;

	EnckeFreeFlightIntegrator encke;
	ATDPGeneralizedIteratorArray iter_arr;

	LVTargetingObjectives LVTargetingData;
	std::vector<LVTargetingObjectives> LVTargetingDataTable;

	SFPDataSet SFPData;
	std::vector<SFPDataSet> SFPDataSets;
};