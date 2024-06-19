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
#include <fstream>
#include "EnckeIntegrator.h"
#include "GeneralizedIterator.h"
#include "ApolloTrajectoryDesignProgram.h"

ApolloTrajectoryDesignProgram::ApolloTrajectoryDesignProgram() :
	gamma_reentry(-6.52*RAD),
	Reentry_range(1285.0),
	Reentry_dt(500.0/3600.0)
{

}

void ApolloTrajectoryDesignProgram::LunarSunElevationAngle(int Year, int Month, int Day, double Hour, double Lat_SG, double Lng_SG, double &Elev, bool &Rising)
{
	//Get sun elevation angle at desired time and whether the sun is rising or setting

	VECTOR3 R_S_SG;
	double Elev2;

	SetupBasics(Year, Month, Day);

	R_S_SG = OrbMech::r_from_latlong(Lat_SG*RAD, Lng_SG*RAD) * OrbMech::R_Moon;

	Elev = OrbMech::LunarSunElevationAngleAtMJD(MDGSUN, GMTBASE, Hour, R_S_SG);

	Elev2 = OrbMech::LunarSunElevationAngleAtMJD(MDGSUN, GMTBASE, Hour + 0.01, R_S_SG);

	if (Elev2 > Elev)
	{
		Rising = true;
	}
	else
	{
		Rising = false;
	}
}

FirstGuessLogicDisplay ApolloTrajectoryDesignProgram::CalculateFirstGuessLogic(int Year, int Month, int Day, double Azi, int AlitudeOption, int Window, int Opportunity, int TLI1stOppOrbit, int Orbits, double Lat_SG, double Lng_SG)
{
	DebugMessages.clear();

	FirstGuessLogicDisplay outdisp;
	VECTOR3 R_S_SG;

	SetupBasics(Year, Month, Day);

	EOIProcessorDataSet EOIData;
	TLIFirstGuessOutputs out;
	OrbMech::SphericalCoordinates coord_EOI;
	double T_L;
	int err;

	//Get insertion state
	EOIData = OrbMech::GetEOIProcessorData(0, AlitudeOption);
	OrbMech::EOIProcessor(EOIData, Azi*RAD, coord_EOI, T_L);

	double Hour = 12.0; //Search from noon

	err = CIST(Hour, coord_EOI, Window, Opportunity, TLI1stOppOrbit, out);

	if (err) return outdisp;

	double InsertionGMT, LaunchGMT, TLIGMT, TLIGET, PCGMT, PCGET, LandingGMT, LandingGET;

	InsertionGMT = out.T1 + Hour;
	LaunchGMT = InsertionGMT - T_L;
	TLIGMT = InsertionGMT + out.CTIO;
	TLIGET = TLIGMT - LaunchGMT;
	PCGMT = Hour + out.TOPCY;
	PCGET = PCGMT - LaunchGMT;
	LandingGMT = PCGMT + outdisp.PCtime + 2.0 * (double)Orbits + 1.0;
	LandingGET = LandingGMT - LaunchGMT;

	outdisp.Launchtime = LaunchGMT;
	outdisp.TLItime = TLIGET;
	outdisp.PCtime = PCGET;
	outdisp.LandingTime = LandingGET;

	R_S_SG = OrbMech::r_from_latlong(Lat_SG*RAD, Lng_SG*RAD) * OrbMech::R_Moon;

	outdisp.LandingSunElevation = OrbMech::LunarSunElevationAngleAtMJD(MDGSUN, GMTBASE, LandingGMT, R_S_SG) * DEG;

	return outdisp;
}

int ApolloTrajectoryDesignProgram::OptimizedFullMission(const PerformanceData& perf, const OptimizedFullMissionInputs& in, ConvergedMissionDisplay& out)
{
	DebugMessages.clear();

	EOIProcessorDataSet EOIData;
	TLIFirstGuessOutputs fgout;
	OrbMech::SphericalCoordinates coord_EOI;
	double LaunchAzimuth, DT_L, Hour;
	int err;

	SetupBasics(in.Year, in.Month, in.Day);

	//Get insertion state
	LaunchAzimuth = in.Azi * RAD;
	EOIData = OrbMech::GetEOIProcessorData(0, in.AltitudeOption);
	OrbMech::EOIProcessor(EOIData, LaunchAzimuth, coord_EOI, DT_L);

	Hour = in.EstimatedTime; //Time from which the initial guess logic will search +/- 12 hours

	err = CIST(Hour, coord_EOI, in.Window, in.Opportunity, in.TLI1stOppOrbit, fgout);

	if (err) return 1;

	double InsertionGMT, LaunchGMT, TLIGMT, PCGMT;

	InsertionGMT = fgout.T1 + Hour;
	LaunchGMT = InsertionGMT - DT_L;
	TLIGMT = InsertionGMT + fgout.CTIO;
	PCGMT = Hour + fgout.TOPCY;

	//Set up array constants
	iter_arr.coord_EOI = coord_EOI;
	iter_arr.GMTBASE = GMTBASE;
	iter_arr.F_I_SIVB = perf.TH1;
	iter_arr.F_SIVB = perf.TH2;
	iter_arr.WDOT_SIVB = perf.MF1;
	iter_arr.T_MRS = perf.T_MRS;
	iter_arr.Weight_Insertion = perf.SIVBWeight + perf.CSMWeight + perf.LMWeight;
	iter_arr.Area_SIVB = 365.0;
	iter_arr.m_CSM = perf.CSMWeight;
	iter_arr.m_LM = perf.LMWeight;
	iter_arr.MaxInclination = in.MaxInclination * RAD;
	iter_arr.lng_spl_des = in.lambda_IP * RAD;
	iter_arr.dv_loi_cal = 10.0 * OrbMech::FPS2ERPH;
	iter_arr.dv_tei_cal = 0.0 * OrbMech::FPS2ERPH;

	iter_arr.isp_SPS = 3080.0 / OrbMech::G0 / 3600.0;
	iter_arr.isp_DPS = 3107.0 / OrbMech::G0 / 3600.0;

	//Set up mode specific values
	iter_arr.FirstOpportunity = (in.Opportunity == 1);

	//Set up initial guess
	iter_arr.T_L = LaunchGMT;
	iter_arr.DT_L = DT_L;
	iter_arr.DT_EPO = fgout.CTIO;
	iter_arr.C3 = fgout.C3;
	iter_arr.sigma = fgout.S;

	//Calculate lunar reference radius
	iter_arr.R_LLS = OrbMech::R_Moon + in.H_LLS / OrbMech::ER2NM;

	//Set up targets
	iter_arr.RCA_M = iter_arr.R_LLS + in.H_PLPO / OrbMech::ER2NM;
	iter_arr.psi_DS = in.psi_DS * RAD;

	if (in.FreeReturn)
	{
		iter_arr.sv_PC.GMT = PCGMT;
	}
	else
	{
		iter_arr.sv_PC.GMT = TLIGMT + in.DT_TLI_LOI;
	}
	iter_arr.phi_star = 0.0; //Initial guess
	iter_arr.RCA_E = 1.0 + 24.0 / OrbMech::ER2NM; //24 NM vacuum perigee

	//Set up lunar landing site geometry
	iter_arr.lat_LLS = in.lat_LLS * RAD;
	iter_arr.lng_LLS = in.lng_LLS * RAD;
	iter_arr.DW = in.DW * RAD;
	iter_arr.h_a = in.h_a / OrbMech::ER2NM;
	iter_arr.h_p = in.h_p / OrbMech::ER2NM;
	iter_arr.R_ALPO = iter_arr.R_LLS + in.H_ALPO / OrbMech::ER2NM;
	iter_arr.R_PLPO = iter_arr.R_LLS + in.H_PLPO / OrbMech::ER2NM;
	iter_arr.REVS1 = in.R1;
	iter_arr.REVS2 = in.R2;
	iter_arr.REVS3 = in.REVS3;
	iter_arr.REVS4 = in.REVS4;
	iter_arr.REVS5 = in.REVS5;

	//Converge initial guess trajectory on pericynthion state
	bool converr = ConvergeInitialGuessTrajectory(false);

	if (converr) return 2;

	if (in.FreeReturn)
	{
		//Converge initial free return trajectory
		converr = ConvergeInitialGuessTrajectory(true);
		if (converr) return 25;
	}

	//Converge mission through LOPC
	converr = ConvergeOptimizedFullMission(in.FreeReturn, in.DT_TLI_LOI);

	if (converr) return 3;

	//DPS monitor
	EnckeIntegratorInputTable enckein;

	enckein.AnchorVector = iter_arr.sv_PC;
	enckein.GMTBASE = iter_arr.GMTBASE;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = 2.0;
	enckein.IsForwardIntegration = 1.0;
	enckein.CutoffIndicator = 1;
	enckein.MDGSUN = &MDGSUN;

	encke.Propagate(enckein);

	iter_arr.sv_TEI = enckein.sv_cutoff;

	converr = ConvergeDPSAbort();

	if (converr) return 4;

	//Store PC+2 results
	out.dv_PC2 = iter_arr.dv_R_TEI / OrbMech::FPS2ERPH;
	out.EIInclination_PC2 = iter_arr.incl_pr * DEG;

	//TEI first guess
	OrbMech::EphemerisData sv_TEI1, sv_TEI2;
	double P;

	P = PI2 / (length(iter_arr.sv_LLS2.V)/length(iter_arr.sv_LLS2.R));

	//Propagate through REVS5-orbits
	enckein.AnchorVector = iter_arr.sv_LLS2;
	enckein.GMTBASE = iter_arr.GMTBASE;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = P*(double)in.REVS5;
	enckein.IsForwardIntegration = 1.0;
	enckein.CutoffIndicator = 1;

	encke.Propagate(enckein);
	sv_TEI1 = enckein.sv_cutoff;

	//Get closest point to 180° EMP longitude as first TEI estimate
	sv_TEI2 = GetNearestTEIPoint(sv_TEI1);

	VECTOR3 R_TEI_EMP, V_TEI_EMP;
	double r_emp, v_emp, theta_emp, phi_emp, gamma_emp, psi_emp, dpsi_TEI;

	R_TEI_EMP = sv_TEI2.R;
	V_TEI_EMP = sv_TEI2.V;
	LIBRAT(R_TEI_EMP, V_TEI_EMP, sv_TEI2.GMT, 4);
	RVIO(true, R_TEI_EMP, V_TEI_EMP, r_emp, v_emp, theta_emp, phi_emp, gamma_emp, psi_emp);
	dpsi_TEI = 270.0 * RAD - psi_emp;
	if (dpsi_TEI > PI)
	{
		dpsi_TEI -= PI2;
	}

	iter_arr.sv_TEI = sv_TEI2;

	//Converge a TEI on the reentry conditions only
	converr = ConvergeTransearthInjection(false, false, 0.0, 2800.0 * OrbMech::FPS2ERPH, dpsi_TEI, in.DT_TEC);
	if (converr) return 5;

	//Converge on approximate splashdown longitude
	double dlng, ddt_TEC, DT_TEC;

	dlng = iter_arr.lng_spl_des - iter_arr.lng_ip_pr;
	while (dlng > PI)
	{
		dlng -= PI2;
	}
	while (dlng < -PI)
	{
		dlng += PI2;
	}
	ddt_TEC = dlng / OrbMech::w_Earth;
	DT_TEC = in.DT_TEC - ddt_TEC;

	converr = ConvergeTransearthInjection(false, false, iter_arr.DT_TEI, iter_arr.dv_TEI, iter_arr.dpsi_TEI, DT_TEC);
	if (converr) return 6;

	//Converge a TEI on the desired splashdown longitude and optimize DV
	converr = ConvergeTransearthInjection(true, true, iter_arr.DT_TEI, iter_arr.dv_TEI, iter_arr.dpsi_TEI, DT_TEC);
	if (converr) return 7;

	out.Launchtime = iter_arr.T_L;
	out.T_EOI = iter_arr.sv_EOI.GMT - iter_arr.T_L;
	out.TLIIgnitionTime = iter_arr.sv_TLI_Ignition.GMT - iter_arr.T_L;
	out.TLICutoffTime = iter_arr.sv_TLI_Cutoff.GMT - iter_arr.T_L;
	out.dv_TLI = iter_arr.dv_TLI / OrbMech::FPS2ERPH;
	out.GMT_PC = iter_arr.sv_PC.GMT;
	out.GET_PC = iter_arr.sv_PC.GMT - iter_arr.T_L;
	out.T_LOI2 = iter_arr.sv_LOI2.GMT - iter_arr.T_L;
	out.LandingTime = iter_arr.sv_LLS.GMT - iter_arr.T_L;

	VECTOR3 R_S_SG;

	R_S_SG = OrbMech::r_from_latlong(iter_arr.lat_LLS, iter_arr.lng_LLS) * OrbMech::R_Moon;
	out.LandingSunElevation = OrbMech::LunarSunElevationAngleAtMJD(MDGSUN, GMTBASE, iter_arr.sv_LLS.GMT, R_S_SG) * DEG;

	out.lat_PC = iter_arr.lat_pl * DEG;
	out.dv_LOI1 = iter_arr.dv_LOI1 / OrbMech::FPS2ERPH;
	out.dpsi_LOI1 = iter_arr.dpsi_LOI * DEG;
	out.m_LOI1 = iter_arr.m_loi1;
	out.dv_LOI2 = iter_arr.dv_LOI2 / OrbMech::FPS2ERPH;
	out.m_LOI2 = iter_arr.m_loi2;
	out.LOPCTime = iter_arr.sv_LOPC.GMT - iter_arr.T_L;
	out.dv_LOPC = length(iter_arr.DV_LOPC) / OrbMech::FPS2ERPH;
	out.m_LOPC = iter_arr.m_lopc;
	out.AscentTime = iter_arr.sv_LLS2.GMT - iter_arr.T_L;
	out.dv_Total = (iter_arr.dv_Total + iter_arr.dv_R_TEI) / OrbMech::FPS2ERPH; //TBD
	out.azi_approach = iter_arr.psi_DS * DEG;
	out.TEITime = iter_arr.sv_TEI.GMT - iter_arr.T_L;
	out.dv_TEI = iter_arr.dv_R_TEI / OrbMech::FPS2ERPH;
	out.dpsi_TEI = iter_arr.dpsi_TEI * DEG;
	out.m_TEI = iter_arr.m_tei;
	out.EITime = iter_arr.sv_EI.GMT - iter_arr.T_L;
	out.EIInclination = iter_arr.incl_pr * DEG;
	out.lat_spl = iter_arr.lat_ip_pr * DEG;
	out.lng_spl = iter_arr.lng_ip_pr * DEG;
	if (out.lng_spl > 180.0)
	{
		out.lng_spl -= 360.0;
	}

	out.m_SPS_remain = perf.SPSPropellant - (perf.CSMWeight - out.m_TEI);

	out.dv_remain = iter_arr.isp_SPS * OrbMech::G0ER * log(out.m_TEI/(out.m_TEI - out.m_SPS_remain));
	out.dv_remain /= OrbMech::FPS2ERPH;

	//Write LV targeting objectives
	LVTargetingData = WriteLVTargetingObjectives(LaunchAzimuth, in.Window);
	//Write SFP
	SFPData = WriteSFPTable(LaunchAzimuth, in.Window);

	return 0;
}

void ApolloTrajectoryDesignProgram::CalculateLaunchData(const PerformanceData& perf, std::string project, std::string preset_filename, MSFCPresetTape& presettings, double* data)
{
	char Buff[128];

	double ActualLaunchTime, A_Z, JD, MJD;
	
	if (LoadMSFCPresetTape(preset_filename, presettings) == false) return;

	sprintf_s(Buff, "./Projects/%s-RTCC-%04d-%02d-%02d.txt", project.c_str(), presettings.Year, presettings.Month, presettings.Day);
	std::vector <MSFCPresetTape> tapes;
	tapes.push_back(presettings);
	ConvertPresetTapeToRTCCPunchCards(perf, tapes, Buff);

	//Calculate data for actual launch
	ActualLaunchTime = ceil(presettings.TLO / 60.0) * 60.0;
	A_Z = VariableLaunchAzimith(presettings, ActualLaunchTime)*DEG;

	JD = OrbMech::TJUDAT(presettings.Year, presettings.Month, presettings.Day);
	MJD = JD - 2400000.5;

	data[0] = ActualLaunchTime;
	data[1] = A_Z;
	data[3] = MJD + ActualLaunchTime / 24.0 / 3600.0;
	data[2] = data[3] - 4.0 / 24.0;
}

bool ApolloTrajectoryDesignProgram::ConvergeInitialGuessTrajectory(bool FreeReturn)
{
	iter_arr.FreeReturn = FreeReturn;
	iter_arr.LunarOrbitPhase = false;

	void* constPtr;
	constPtr = &iter_arr;

	bool IntegratedTrajectoryComputerPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &IntegratedTrajectoryComputerPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[0] = true;
	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;
	block.IndVarGuess[0] = iter_arr.T_L;
	block.IndVarGuess[1] = iter_arr.DT_EPO;
	block.IndVarGuess[2] = iter_arr.C3;
	block.IndVarGuess[3] = iter_arr.sigma;
	block.IndVarStep[0] = 0.001;
	block.IndVarStep[1] = 0.001;
	block.IndVarStep[2] = 0.00001;
	block.IndVarWeight[0] = 512.0;
	block.IndVarWeight[1] = 512.0;
	block.IndVarWeight[2] = 512.0;

	block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;
	if (FreeReturn)
	{
		block.DepVarSwitch[7] = true;
	}
	else
	{
		block.DepVarSwitch[4] = true;
	}

	block.DepVarLowerLimit[0] = -500.0 / OrbMech::R_Earth;
	block.DepVarLowerLimit[1] = -500.0 / OrbMech::R_Earth;
	//block.DepVarLowerLimit[2] = -1000.0 / OrbMech::R_Earth;
	block.DepVarLowerLimit[4] = iter_arr.sv_PC.GMT - 0.0001;
	block.DepVarLowerLimit[7] = 64.0965 / OrbMech::ER2NM;

	block.DepVarUpperLimit[0] = 500.0 / OrbMech::R_Earth;
	block.DepVarUpperLimit[1] = 500.0 / OrbMech::R_Earth;
	//block.DepVarUpperLimit[2] = 1000.0 / OrbMech::R_Earth;
	block.DepVarUpperLimit[4] = iter_arr.sv_PC.GMT + 0.0001;
	block.DepVarUpperLimit[7] = 67.5665 / OrbMech::ER2NM;

	block.DepVarClass[0] = 1;
	block.DepVarClass[1] = 1;
	block.DepVarClass[4] = 1;
	block.DepVarClass[7] = 1;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool ApolloTrajectoryDesignProgram::ConvergeOptimizedFullMission(bool FreeReturn, double DT_TLI_LOI)
{
	iter_arr.FreeReturn = FreeReturn;
	iter_arr.LunarOrbitPhase = true;

	void* constPtr;
	constPtr = &iter_arr;

	bool IntegratedTrajectoryComputerPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &IntegratedTrajectoryComputerPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[0] = true;
	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;

	block.IndVarGuess[0] = iter_arr.T_L;
	block.IndVarGuess[1] = iter_arr.DT_EPO;
	block.IndVarGuess[2] = iter_arr.C3;
	block.IndVarGuess[3] = iter_arr.sigma;

	block.IndVarStep[0] = 0.0001;
	block.IndVarStep[1] = 0.0001;
	block.IndVarStep[2] = 0.000001;
	//block.IndVarStep[4] = 0.00001;
	//block.IndVarStep[5] = 0.001;
	block.IndVarWeight[0] = 1.0;// 512.0;
	block.IndVarWeight[1] = 1.0; // 512.0;
	block.IndVarWeight[2] = 0.125;// 1.0; // 512.0;

	if (FreeReturn)
	{
		block.DepVarSwitch[7] = true;
		block.DepVarSwitch[8] = true;
	}
	else
	{
		block.DepVarSwitch[5] = true;
	}
	block.DepVarSwitch[6] = true;
	block.DepVarSwitch[10] = true;
	block.DepVarSwitch[11] = true;

	block.DepVarLowerLimit[2] = -1000.0 / OrbMech::R_Earth;
	block.DepVarLowerLimit[5] = DT_TLI_LOI - 0.0001;
	block.DepVarLowerLimit[6] = 90.0*RAD;
	block.DepVarLowerLimit[7] = 64.0965 / OrbMech::ER2NM;
	block.DepVarLowerLimit[8] = 0.0;
	block.DepVarLowerLimit[10] = -0.1 / OrbMech::ER2NM;
	block.DepVarLowerLimit[11] = -0.01 * RAD;

	block.DepVarUpperLimit[2] = 1000.0 / OrbMech::R_Earth;
	block.DepVarUpperLimit[5] = DT_TLI_LOI + 0.0001;
	block.DepVarUpperLimit[6] = 182.0 * RAD;
	block.DepVarUpperLimit[7] = 67.5665 / OrbMech::ER2NM;
	block.DepVarUpperLimit[8] = 90.0 * RAD;
	block.DepVarUpperLimit[10] = 0.1 / OrbMech::ER2NM;
	block.DepVarUpperLimit[11] = 0.01 * RAD;

	block.DepVarClass[5] = 1;
	block.DepVarClass[6] = 2;
	block.DepVarClass[7] = 1;
	block.DepVarClass[8] = 2;
	block.DepVarClass[10] = 1;
	block.DepVarClass[11] = 1;

	block.DepVarWeight[6] = 64.0;
	block.DepVarWeight[8] = 8.0;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool ApolloTrajectoryDesignProgram::ConvergeDPSAbort()
{
	void* constPtr;
	constPtr = &iter_arr;

	bool DPSDVMonitorTrajectoryComputerPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &DPSDVMonitorTrajectoryComputerPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[0] = true;
	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;

	block.IndVarGuess[0] = 0.0;
	block.IndVarGuess[1] = 0.0;
	block.IndVarGuess[2] = 0.0;

	block.IndVarStep[0] = pow(2, -21);
	block.IndVarStep[1] = pow(2, -21);
	block.IndVarStep[2] = pow(2, -21);

	block.IndVarWeight[0] = 8.0;
	block.IndVarWeight[1] = 8.0;
	block.IndVarWeight[2] = 1.0;

	block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;
	block.DepVarSwitch[2] = true;

	block.DepVarLowerLimit[0] = 64.096513 / OrbMech::ER2NM;
	block.DepVarLowerLimit[1] = 0.0;
	block.DepVarLowerLimit[2] = -0.001;

	block.DepVarUpperLimit[0] = 67.566513 / OrbMech::ER2NM;
	block.DepVarUpperLimit[1] = 90.0 * RAD;
	block.DepVarUpperLimit[2] = -0.001;

	block.DepVarClass[0] = 1;
	block.DepVarClass[1] = 2;
	block.DepVarClass[2] = 3;

	block.DepVarWeight[1] = 8.0;
	block.DepVarWeight[2] = 1.0;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool ApolloTrajectoryDesignProgram::ConvergeTransearthInjection(bool Optimize, bool ConvergeLongitude, double DT_TEI, double dv, double dpsi, double DT_TEC)
{
	void* constPtr;
	constPtr = &iter_arr;

	bool TransearthInjectionTrajectoryComputerPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &TransearthInjectionTrajectoryComputerPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[0] = true;
	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;
	block.IndVarSwitch[3] = true;

	block.IndVarGuess[0] = DT_TEI;
	block.IndVarGuess[1] = dv;
	block.IndVarGuess[2] = 0.0;
	block.IndVarGuess[3] = dpsi;

	block.IndVarStep[0] = pow(2, -19);
	block.IndVarStep[1] = pow(2, -19);
	block.IndVarStep[2] = pow(2, -19);
	block.IndVarStep[3] = pow(2, -19);

	block.IndVarWeight[0] = 1.e-6;
	block.IndVarWeight[1] = 1.0;
	block.IndVarWeight[2] = 1.0;
	block.IndVarWeight[3] = 1.0;

	if (Optimize) block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;
	block.DepVarSwitch[2] = true;
	if (ConvergeLongitude) block.DepVarSwitch[3] = true;
	block.DepVarSwitch[4] = true;

	block.DepVarLowerLimit[0] = iter_arr.m_lopc + 6000.0;
	block.DepVarLowerLimit[1] = 64.096513 / OrbMech::ER2NM;
	block.DepVarLowerLimit[2] = 0.0;
	block.DepVarLowerLimit[3] = -0.2*RAD;
	if (ConvergeLongitude)
	{
		block.DepVarLowerLimit[4] = DT_TEC - 8.0;
	}
	else
	{
		block.DepVarLowerLimit[4] = DT_TEC - 0.1;
	}

	block.DepVarUpperLimit[0] = iter_arr.m_lopc + 6000.0;
	block.DepVarUpperLimit[1] = 67.566513 / OrbMech::ER2NM;
	block.DepVarUpperLimit[2] = iter_arr.MaxInclination;
	block.DepVarUpperLimit[3] = 0.2*RAD;
	if (ConvergeLongitude)
	{
		block.DepVarUpperLimit[4] = DT_TEC + 8.0;
	}
	else
	{
		block.DepVarUpperLimit[4] = DT_TEC + 0.1;
	}

	block.DepVarClass[0] = 3;
	block.DepVarClass[1] = 1;
	block.DepVarClass[2] = 2;
	block.DepVarClass[3] = 1;
	if (ConvergeLongitude)
	{
		block.DepVarClass[4] = 2;
	}
	else
	{
		block.DepVarClass[4] = 1;
	}

	block.DepVarWeight[0] = 1.0;
	block.DepVarWeight[2] = 8.0;
	block.DepVarWeight[4] = 0.125;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool ApolloTrajectoryDesignProgram::SetupBasics(int Year, int Month, int Day)
{
	//Set up date related numbers
	this->Year = Year;
	this->Month = Month;
	this->Day = Day;

	//Error checking
	if (Year < 1951 || Year > 1999)
	{
		DebugMessages.push_back("Input year invalid");
		return true;
	}
	if (Month < 1 || Month > 12)
	{
		DebugMessages.push_back("Input month invalid");
		return true;
	}
	if (Day < 1 || Day > 31)
	{
		DebugMessages.push_back("Input day invalid");
		return true;
	}

	//Epoch is the closest turn of year
	Epoch = Year;
	if (Month >= 7)
	{
		Epoch++;
	}

	//Calculate day of the year
	RefDay = OrbMech::DayOfYear(Year, Month, Day);
	//Calculate MJD at midnight of launch
	double J_D = OrbMech::TJUDAT(Year, Month, Day);
	GMTBASE = J_D - 2400000.5;
	//Calculate hour angle
	BHA = OrbMech::HANGLE(Epoch, Year, RefDay);

	//Initialize ephemerides
	if (OrbMech::InitializeEphemerides(Epoch, GMTBASE, MDGSUN))
	{
		DebugMessages.push_back("Ephemerides initialization failed");
		return true;
	}
	return false;
}

int ApolloTrajectoryDesignProgram::CIST(double Hour, OrbMech::SphericalCoordinates coord_EOI, int Window, int Opportunity, int TLI1stOppOrbit, TLIFirstGuessOutputs& out)
{
	TLIFirstGuessInputs in;
	int err;

	in.BHA = BHA;
	in.Hour = Hour;
	in.GMTBASE = GMTBASE;
	in.C1 = coord_EOI.lat;
	in.FLO1 = coord_EOI.lng;
	in.AZ1 = coord_EOI.azi;
	in.R1 = coord_EOI.rad * OrbMech::ER2NM; //NM
	in.YPC = 0.0;
	in.XPC = 0.0;
	in.RPC = 60.0 + OrbMech::R_Moon * OrbMech::ER2NM; //NM

	in.ORBNUM = (double)(Opportunity - 1 + TLI1stOppOrbit);
	in.IPOA = Window;

	in.IPERT = 3;
	in.MDGSUN = &MDGSUN;

	TLIFirstGuess tli;

	tli.Calculate(in);
	err = tli.GetError();

	if (err == 0)
	{
		tli.GetOutput(out);
	}

	return err;
}

OrbMech::EphemerisData ApolloTrajectoryDesignProgram::GetNearestTEIPoint(OrbMech::EphemerisData sv)
{
	EnckeIntegratorInputTable enckein;
	OrbMech::EphemerisData sv_TEI2;
	VECTOR3 R_TEI_EMP;
	double lat_TEI, lng_TEI, dlng, dt;
	int n, nmax;

	sv_TEI2 = sv;
	n = 0;
	nmax = 10;

	enckein.GMTBASE = iter_arr.GMTBASE;
	enckein.MinIntegTime = 0.0;
	enckein.CutoffIndicator = 1;
	enckein.MDGSUN = &MDGSUN;

	do
	{
		R_TEI_EMP = sv_TEI2.R;
		LIBRAT(R_TEI_EMP, sv_TEI2.GMT, 4);
		OrbMech::latlong_from_r(R_TEI_EMP, lat_TEI, lng_TEI);
		if (lng_TEI < 0)
		{
			lng_TEI += PI2;
		}
		dlng = lng_TEI - PI;
		if (dlng > PI)
		{
			dlng -= PI2;
		}
		else if (dlng < -PI)
		{
			dlng += PI2;
		}
		dt = dlng / (180.0 * RAD);

		enckein.AnchorVector = sv_TEI2;
		enckein.MaxIntegTime = abs(dt);
		enckein.IsForwardIntegration = dt >= 0.0 ? 1.0 : -1.0;

		encke.Propagate(enckein);

		sv_TEI2 = enckein.sv_cutoff;
		n++;

	} while (abs(dt) > 0.1 / 3600.0 && n < nmax);

	return sv_TEI2;
}

bool IntegratedTrajectoryComputerPointer(void* data, std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	return ((ApolloTrajectoryDesignProgram*)data)->IntegratedTrajectoryComputer(var, varPtr, arr, mode);
}

bool ApolloTrajectoryDesignProgram::IntegratedTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	//Independent Variables:
	//0: Launch time, hours
	//1: Coast time in earth parking orbit, hours
	//2: TLI specific energy, (Er/hr)^2
	//3: perigee half angle
	//Dependent variables:
	//0: D_BT
	//1: D_BR
	//3: TLI cutoff weight
	//4: Time of pericynthion
	//5: Time from TLI (cutoff) to pericynthion
	//6: Inclination at perilune in rad
	//7: Height of return in Er
	//8: Inclination at free return in rad
	//10: Delta height of node in Er
	//11: Theta in rad
	//17: Mass after DOI
	//18: Mass after LOPC

	ATDPGeneralizedIteratorArray* vars;
	vars = static_cast<ATDPGeneralizedIteratorArray*>(varPtr);

	OrbMech::TLIBRNOutput tliout;
	EnckeIntegratorInputTable enckein;
	VECTOR3 R_temp, V_temp, HH_pl, V_EM;
	double T_INS, lng_iner;

	vars->T_L = var[0];
	vars->DT_EPO = var[1];
	vars->C3 = var[2];
	vars->sigma = var[3];
	vars->dpsi_LOI = var[4];
	vars->DT_LPO = var[5];

	//Convert spherical coordinates to inertial
	T_INS = vars->T_L + vars->DT_L;
	lng_iner = BHA + vars->coord_EOI.lng + OrbMech::w_Earth * T_INS;
	OrbMech::adbar_from_rv(vars->coord_EOI.rad, vars->coord_EOI.vel, lng_iner, vars->coord_EOI.lat, vars->coord_EOI.fpa + PI05, vars->coord_EOI.azi, vars->sv_EOI.R, vars->sv_EOI.V);
	vars->sv_EOI.GMT = T_INS;
	vars->sv_EOI.RBI = OrbMech::Bodies::Body_Earth;

	//Integrate to TLI ignition
	enckein.AnchorVector = vars->sv_EOI;
	enckein.GMTBASE = vars->GMTBASE;
	enckein.GMTLO = vars->T_L;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = vars->DT_EPO;
	enckein.IsForwardIntegration = 1.0;
	enckein.DensityMultiplier = 1.0;
	enckein.VentPerturbationFactor = 1.0;
	enckein.CutoffIndicator = 1;
	enckein.Area = vars->Area_SIVB;
	enckein.Weight = vars->Weight_Insertion;
	enckein.VentTable = &VentTable;
	enckein.MDGSUN = &MDGSUN;

	encke.Propagate(enckein);

	if (enckein.TerminationCode != 1)
	{
		return true;
	}

	vars->sv_TLI_Ignition = enckein.sv_cutoff;
	vars->Weight_TLI_Ignition = enckein.Weight_cutoff;

	//Simulate TLI
	double T_MRS;
	if (vars->FirstOpportunity)
	{
		T_MRS = vars->T_MRS;
	}
	else
	{
		T_MRS = 1.0 / HRS;
	}

	OrbMech::TLIBRN(vars->sv_TLI_Ignition, vars->C3, vars->sigma, 0.0, vars->F_SIVB, vars->F_I_SIVB, vars->Weight_TLI_Ignition, vars->WDOT_SIVB, T_MRS, tliout);
	vars->sv_TLI_Cutoff = tliout.sv2;
	vars->Weight_TLI_Cutoff = tliout.W2;
	vars->T_u = tliout.T;
	vars->dv_TLI = tliout.DV;

	arr[3] = vars->Weight_TLI_Cutoff;

	//TEST CODE: APOLLO 8 BEFORE MCC-1
	//vars->sv_TLI_Cutoff.R = _V(1.016668551045908e+01, -8.117500555974903e+00, 5.347561518963154e-02);
	//vars->sv_TLI_Cutoff.V = _V(1.462791790688202, -0.628852259790073, -0.243426699310296);
	//vars->sv_TLI_Cutoff.GMT = 21.54969777847873;

	//Propagate to pericynthion
	enckein.AnchorVector = vars->sv_TLI_Cutoff;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = 10.0 * 24.0;
	enckein.IsForwardIntegration = 1.0;
	enckein.EarthRelStopParam = 0.0;
	enckein.MoonRelStopParam = 0.0;
	enckein.StopParamRefFrame = 1;
	enckein.DensityMultiplier = 1.0;
	enckein.VentPerturbationFactor = -1.0;
	enckein.CutoffIndicator = 4;
	enckein.Area = vars->Area_SIVB;
	enckein.Weight = vars->Weight_TLI_Cutoff;

	encke.Propagate(enckein);

	if (enckein.TerminationCode != 4)
	{
		return true;
	}

	vars->sv_PC = enckein.sv_cutoff;

	if (vars->LunarOrbitPhase == false)
	{
		//Calculate error in b-plane
		OrbMech::BPlaneTargetingPC(MDGSUN, GMTBASE, vars->sv_PC, vars->RCA_M, vars->phi_star, vars->D_BT_M, vars->D_BR_M, vars->sv_PC2, vars->R_EM, V_EM);

		arr[0] = vars->D_BT_M;
		arr[1] = vars->D_BR_M;
	}
	else
	{
		if (vars->FreeReturn)
		{
			vars->sv_PC2 = vars->sv_PC;
			JPLEPH(MDGSUN, GMTBASE, 4, vars->sv_PC.GMT, &vars->R_EM, &V_EM, NULL, NULL);
		}
	}

	arr[4] = vars->sv_PC.GMT;
	arr[5] = vars->sv_PC.GMT - vars->sv_TLI_Cutoff.GMT;

	//Calculate EMP inclination
	R_temp = vars->sv_PC.R;
	V_temp = vars->sv_PC.V;
	LIBRAT(R_temp, V_temp, vars->sv_PC.GMT, 4);
	HH_pl = unit(crossp(R_temp, V_temp));

	vars->lat_pl = atan(R_temp.z / sqrt(R_temp.x * R_temp.x + R_temp.y * R_temp.y));
	vars->lng_pl = atan2(R_temp.y, R_temp.x);
	vars->h_pl = length(R_temp) - vars->R_LLS;
	vars->incl_pl = acos(HH_pl.z);
	arr[6] = vars->incl_pl;

	if (vars->LunarOrbitPhase)
	{
		//Fixed orbit
		OrbMech::EphemerisData sv_LOI1;
		VECTOR3 U_DS, R_N, U_H, U_PJ, Gamma, Vectemp;
		double theta, r_N, v_H, V1, gamma_H, m_tlc, a_LLS, v_LPO2;

		PRCOMP(vars->sv_PC, vars->psi_DS, sv_LOI1, vars->sv_LOI2, vars->sv_LLS);
		U_DS = unit(crossp(sv_LOI1.R, sv_LOI1.V));
		R_N = unit(vars->sv_PC.R);
		r_N = length(vars->sv_PC.R);
		v_H = length(vars->sv_PC.V);
		U_H = unit(crossp(vars->sv_PC.R, vars->sv_PC.V));
		gamma_H = 0.0;

		U_PJ = unit(U_DS - R_N * dotp(U_DS, R_N));
		Gamma = unit(crossp(U_H, R_N));

		Vectemp = crossp(U_DS, U_PJ);
		theta = length(Vectemp) * OrbMech::sign(dotp(Vectemp, Gamma));

		arr[10] = vars->R_PLPO - r_N;
		arr[11] = theta;

		V1 = sqrt(OrbMech::mu_Moon * (2.0 / r_N - 2.0 / (r_N + vars->R_ALPO))); //Velocity after LOI-1
		vars->dv_LOI1 = sqrt(pow(v_H, 2) + pow(V1, 2) - 2.0 * v_H * V1 * (cos(gamma_H) * dotp(U_DS, U_H))) + vars->dv_loi_cal;
		vars->dpsi_LOI = AzimuthDifference(R_N, U_H, U_DS);

		m_tlc = vars->m_CSM + vars->m_LM;

		//Save dv
		vars->dv_Total = vars->dv_LOI1;
		//Calculate mass after LOI-1
		vars->m_loi1 = MCOMP(m_tlc, vars->dv_LOI1 + vars->dv_loi_cal, vars->isp_SPS);

		//Simulate LOI-2
		a_LLS = (vars->h_a + vars->h_p) / 2.0 + vars->R_LLS;
		v_LPO2 = sqrt(OrbMech::mu_Moon / a_LLS);
		vars->dv_LOI2 = V1 - v_LPO2;
		vars->m_loi2 = MCOMP(vars->m_loi1, vars->dv_LOI2, vars->isp_SPS);

		//Save dv
		vars->dv_Total += vars->dv_LOI2;

		//Simulate LOPC
		double P, mfm0;

		P = PI2 / (length(vars->sv_LLS.V) / length(vars->sv_LLS.R) + OrbMech::w_Moon);
		LOPC(vars->sv_LLS.R, vars->sv_LLS.V, vars->sv_LLS.GMT, OrbMech::r_from_latlong(vars->lat_LLS,vars->lng_LLS), vars->REVS3, vars->REVS4, P, vars->sv_LOPC.R, vars->sv_LOPC.V, vars->sv_LOPC.GMT, mfm0, vars->dpsi_LOPC, vars->DV_LOPC, vars->sv_LLS2.GMT);
		vars->sv_LOPC.RBI = OrbMech::Body_Moon;
		vars->m_lopc = mfm0 * (vars->m_loi2 - vars->m_LM);
		vars->dv_Total += length(vars->DV_LOPC);

		arr[18] = vars->m_lopc;

		//Propagate to LLS2
		CTBODY(vars->sv_LOPC.R, vars->sv_LOPC.V, vars->sv_LOPC.GMT, vars->sv_LLS2.GMT, 0, OrbMech::mu_Moon, vars->sv_LLS2.R, vars->sv_LLS2.V);
		vars->sv_LLS2.RBI = OrbMech::Body_Moon;
	}

	if (vars->FreeReturn)
	{
		//Propagate to perigee
		enckein.AnchorVector = vars->sv_PC2;
		enckein.MinIntegTime = 0.0;
		enckein.MaxIntegTime = 10.0 * 24.0;
		enckein.IsForwardIntegration = 1.0;
		enckein.EarthRelStopParam = sin(gamma_reentry);
		enckein.MoonRelStopParam = 0.0;
		enckein.StopParamRefFrame = 0;
		enckein.DensityMultiplier = 0.0;
		enckein.VentPerturbationFactor = -1.0;
		enckein.CutoffIndicator = 4;

		encke.Propagate(enckein);

		if (enckein.CutoffIndicator != 4)
		{
			return true;
		}

		vars->sv_EI = enckein.sv_cutoff;

		arr[7] = length(vars->sv_EI.R) - 1.0;

		VECTOR3 H_reentry = crossp(vars->sv_EI.R, vars->sv_EI.V);
		arr[8] = acos(H_reentry.z / length(H_reentry));

		//Found apogee
		//if (length(vars->sv_PG.R) > 30.0)
		//{
		//	return false;
		//}

		//OrbMech::BPlaneTargetingPG(vars->sv_PG, vars->R_EM, V_EM, vars->RCA_E, vars->D_BT_E);

		//arr[2] = vars->D_BT_E;
	}

	return false;
}

bool DPSDVMonitorTrajectoryComputerPointer(void* data, std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	return ((ApolloTrajectoryDesignProgram*)data)->DPSDVMonitorTrajectoryComputer(var, varPtr, arr, mode);
}

bool ApolloTrajectoryDesignProgram::DPSDVMonitorTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	//Independent Variables:
	//0: Delta velocity at TEI, Er/hr
	//1: Delta flight path angle at TEI, rad
	//2: Delta azimuth at TEI, rad
	//Dependent variables:
	//0: Height of entry, Er
	//1: Inclination of return, Er
	//2: Characteristic velocity at TEI, Er/hr

	ATDPGeneralizedIteratorArray* vars;
	vars = static_cast<ATDPGeneralizedIteratorArray*>(varPtr);

	EnckeIntegratorInputTable enckein;
	OrbMech::EphemerisData sv_TEI_apo;
	VECTOR3 H;
	double mfm0, height;

	vars->dv_TEI = var[0];
	vars->dgamma_TEI = var[1];
	vars->dpsi_TEI = var[2];

	enckein.GMTBASE = vars->GMTBASE;
	enckein.MDGSUN = &MDGSUN;

	//Simulate PC+2 DPS burn
	sv_TEI_apo = vars->sv_TEI;
	OrbMech::BURN(vars->sv_TEI.R, vars->sv_TEI.V, vars->dv_TEI, vars->dgamma_TEI, vars->dpsi_TEI, vars->isp_DPS, vars->dv_R_TEI, mfm0, sv_TEI_apo.R, sv_TEI_apo.V);

	//Propagate to flight path angle of reentry
	enckein.AnchorVector = sv_TEI_apo;
	enckein.MinIntegTime = 24.0;
	enckein.EarthRelStopParam = sin(gamma_reentry);
	enckein.StopParamRefFrame = 0;
	enckein.CutoffIndicator = 4;
	enckein.IsForwardIntegration = 1.0;

	encke.Propagate(enckein);

	if (enckein.CutoffIndicator != 4)
	{
		return true;
	}

	vars->sv_EI = enckein.sv_cutoff;

	height = length(vars->sv_EI.R) - 1.0;
	H = unit(crossp(vars->sv_EI.R, vars->sv_EI.V));
	vars->incl_pr = acos(H.z);

	arr[0] = height;
	arr[1] = vars->incl_pr;
	arr[2] = vars->dv_R_TEI;

	return false;
}

bool TransearthInjectionTrajectoryComputerPointer(void* data, std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	return ((ApolloTrajectoryDesignProgram*)data)->TransearthInjectionTrajectoryComputer(var, varPtr, arr, mode);
}

bool ApolloTrajectoryDesignProgram::TransearthInjectionTrajectoryComputer(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	//Independent Variables:
	//0: Time in LPO, hrs
	//1: Delta velocity at TEI, Er/hr
	//2: Delta flight path angle at TEI, rad
	//3: Delta azimuth at TEI, rad
	//Dependent variables:
	//0: Mass after TEI, lbs
	//1: Height of entry, Er
	//2: Inclination of return, Er
	//3: DLNG Earth landing, rad
	//4: Transearth flight time, hr

	ATDPGeneralizedIteratorArray* vars;
	vars = static_cast<ATDPGeneralizedIteratorArray*>(varPtr);

	EnckeIntegratorInputTable enckein;
	OrbMech::EphemerisData sv_TEI, sv_TEI_apo;
	VECTOR3 H;
	double mfm0, height, dlng;

	vars->DT_TEI = var[0];
	vars->dv_TEI = var[1];
	vars->dgamma_TEI = var[2];
	vars->dpsi_TEI = var[3];

	enckein.GMTBASE = vars->GMTBASE;
	enckein.MDGSUN = &MDGSUN;

	if (vars->DT_TEI != 0.0)
	{
		enckein.AnchorVector = vars->sv_TEI;
		enckein.MinIntegTime = 0.0;
		enckein.MaxIntegTime = abs(vars->DT_TEI);
		enckein.CutoffIndicator = 1;
		enckein.IsForwardIntegration = vars->DT_TEI >= 0.0 ? 1.0 : -1.0;

		encke.Propagate(enckein);

		sv_TEI = enckein.sv_cutoff;
	}
	else
	{
		sv_TEI = vars->sv_TEI;
	}

	//Simulate TEI burn
	sv_TEI_apo = sv_TEI;
	OrbMech::BURN(sv_TEI.R, sv_TEI.V, vars->dv_TEI, vars->dgamma_TEI, vars->dpsi_TEI, vars->isp_DPS, vars->dv_R_TEI, mfm0, sv_TEI_apo.R, sv_TEI_apo.V);
	vars->m_tei = vars->m_lopc * mfm0;

	//Propagate to flight path angle of reentry
	enckein.AnchorVector = sv_TEI_apo;
	enckein.MinIntegTime = 24.0;
	enckein.MaxIntegTime = 24.0*10.0;
	enckein.EarthRelStopParam = sin(gamma_reentry);
	enckein.StopParamRefFrame = 0;
	enckein.CutoffIndicator = 4;
	enckein.IsForwardIntegration = 1.0;

	encke.Propagate(enckein);

	if (enckein.CutoffIndicator != 4)
	{
		return true;
	}

	vars->sv_EI = enckein.sv_cutoff;

	height = length(vars->sv_EI.R) - 1.0;
	H = unit(crossp(vars->sv_EI.R, vars->sv_EI.V));
	vars->incl_pr = acos(H.z);

	RNTSIM(vars->sv_EI.R, vars->sv_EI.V, vars->sv_EI.GMT, vars->lng_spl_des, vars->lat_ip_pr, vars->lng_ip_pr, dlng);

	arr[0] = vars->m_tei;
	arr[1] = height;
	arr[2] = vars->incl_pr;
	arr[3] = dlng;
	arr[4] = vars->sv_EI.GMT - vars->sv_TEI.GMT;

	return false;
}

void ApolloTrajectoryDesignProgram::PRCOMP(OrbMech::EphemerisData sv_node, double psi_LLS, OrbMech::EphemerisData& sv_LPO1, OrbMech::EphemerisData& sv_LPO2, OrbMech::EphemerisData& sv_LLS) const
{
	VECTOR3 U_H, U_N, R_LLS_sg, V_LLS_sg;
	double a_LLS, dt3, e_LLS, P_LLS, R, V, cos_gamma, gamma, lat, lng, DA;

	U_H = unit(crossp(sv_node.R, sv_node.V)); //Unit angular momentum vector before LOI
	U_N = unit(sv_node.R); //Unit position vector

	//Build selenographic state vector at LLS
	a_LLS = (iter_arr.h_a + iter_arr.h_p) / 2.0 + iter_arr.R_LLS; //Semi major axis of LPO2
	e_LLS = (iter_arr.h_a + iter_arr.R_LLS) / a_LLS - 1.0; //Eccentricty of LPO2
	P_LLS = a_LLS * (1.0 - pow(e_LLS, 2));
	R = P_LLS / (1.0 + e_LLS * cos(iter_arr.DW));
	V = sqrt(OrbMech::mu_Moon * (2.0 / R - 1.0 / a_LLS));
	cos_gamma = sqrt(OrbMech::mu_Moon * P_LLS) / (R * V);
	if (cos_gamma > 1.0)
	{
		cos_gamma = 1.0;
	}
	gamma = -OrbMech::sign(iter_arr.DW) * acos(cos_gamma);
	lat = iter_arr.lat_LLS;
	lng = iter_arr.lng_LLS;

	RVIO(false, R_LLS_sg, V_LLS_sg, R, V, lng, lat, gamma, psi_LLS);
	sv_LLS.RBI = OrbMech::Body_Moon;

	sv_LLS.GMT = LSTime(U_H, sv_node.GMT, U_N, iter_arr.R_ALPO, iter_arr.R_PLPO, a_LLS, e_LLS, iter_arr.REVS1, iter_arr.REVS2, dt3, DA);

	for (int i = 0; i < 2; i++)
	{
		//Convert to inertial
		sv_LLS.R = R_LLS_sg;
		sv_LLS.V = V_LLS_sg;
		LIBRAT(sv_LLS.R, sv_LLS.V, sv_LLS.GMT, 6);

		//Backup to LOI
		sv_LPO2 = BACKUP(sv_LLS, dt3, a_LLS, DA);
	}

	sv_LPO1 = sv_LPO2;
	//TBD
}

double ApolloTrajectoryDesignProgram::LSTime(VECTOR3 U_H, double T_PCYN, VECTOR3 U_PC, double R_ALPO, double R_PLPO, double a_LLS, double e_LLS, int R1, int R2, double& dt3, double& DA) const
{
	VECTOR3 U_LLS;
	double a1, e1, dt1, dt2, T_LLS, dalpha;

	a1 = (R_ALPO + R_PLPO) / 2.0;
	e1 = R_ALPO / a1 - 1.0;
	dt2 = 0.0; //TBD
	dt1 = PI2 * sqrt(pow(a1, 3) / OrbMech::mu_Moon) * (double)R1 + PI2 * sqrt(pow(a_LLS, 3) / OrbMech::mu_Moon) * (double)R2;
	T_LLS = T_PCYN + dt1;
	for (int i=0;i<2;i++)
	{
		//Convert selenographic to selenocentric
		U_LLS = OrbMech::r_from_latlong(iter_arr.lat_LLS, iter_arr.lng_LLS);
		LIBRAT(U_LLS, T_LLS, 6);

		dalpha = acos(dotp(U_LLS, U_PC));
		if (dotp(crossp(U_LLS, U_PC), U_H) > 0.0)
		{
			dalpha = PI2 - dalpha;
		}
		if (dalpha > 0.0)
		{
			DA = dalpha;
		}
		else
		{
			DA = PI2 + dalpha;
		}
		dt3 = DELTAT(a_LLS, e_LLS, iter_arr.DW > 0.0 ? PI2 - iter_arr.DW : -iter_arr.DW, -DA);
		T_LLS = T_PCYN + dt1 + dt2 - dt3;
	}
	return T_LLS;
}

double ApolloTrajectoryDesignProgram::DELTAT(double a, double e, double eta, double deta) const
{
	double K_e, E, eta_apo, dE, E_apo, DE, DM, dt;

	OrbMech::normalizeAngle(eta);

	K_e = sqrt((1.0 - e) / (1.0 + e));
	E = atan2(sqrt(1.0 - e * e) * sin(eta), cos(eta) + e);
	OrbMech::normalizeAngle(E);
	eta_apo = eta + deta;
	if (eta_apo >= 0 && eta_apo < PI2)
	{
		dE = 0.0;
	}
	else
	{
		if (eta_apo < 0.0)
		{
			dE = 0.0;
			while (eta_apo <= -PI2)
			{
				eta_apo += PI2;
				dE += PI2;
			}
			dE = -(dE + PI2);
		}
		else
		{
			dE = 0.0;
			while (eta_apo >= PI2)
			{
				eta_apo -= PI2;
				dE += PI2;
			}
		}
	}
	E_apo = atan2(sqrt(1.0 - e * e) * sin(eta_apo), cos(eta_apo) + e);
	OrbMech::normalizeAngle(E_apo);
	DE = E_apo - E + dE;
	DM = DE + e * (sin(E) - sin(E_apo));
	dt = sqrt(pow(a, 3) / OrbMech::mu_Moon) * DM;
	return dt;
}

OrbMech::EphemerisData ApolloTrajectoryDesignProgram::BACKUP(OrbMech::EphemerisData sv_LLS, double dt3, double a_LLS, double DA) const
{
	OrbMech::EphemerisData sv_LOI;
	VECTOR3 R, V, U_LLS, R_u, U, H_LLS, R2, V2;
	double T, alpha1, deta, a, e, i, n, P, eta, dt;

	U_LLS = unit(sv_LLS.R);

	//Estimate time of LOI2
	T = sv_LLS.GMT + dt3 - iter_arr.REVS2 * PI2 * sqrt(pow(a_LLS, 3) / OrbMech::mu_Moon);
	//Integrate backwards to T
	CTBODY(sv_LLS.R, sv_LLS.V, sv_LLS.GMT, T, 0, OrbMech::mu_Moon, R, V);
	ELEMT(R, V, OrbMech::mu_Moon, H_LLS, a, e, i, n, P, eta);
	R_u = unit(R);
	//Angle between LOI2 position and the LLS
	alpha1 = acos(dotp(R_u, U_LLS));
	//Unit angular momentum vector of LPO2
	U = unit(crossp(R, V));
	if (dotp(crossp(R_u, U_LLS), U) <= 0.0)
	{
		alpha1 = PI2 - alpha1;
	}
	deta = alpha1 - DA;
	dt = DELTAT(a, e, eta, deta);
	//Integrate to T + DT
	CTBODY(R, V, T, T + dt, 0, OrbMech::mu_Moon, R2, V2);

	//Compute first LPO
	a = (iter_arr.R_ALPO + iter_arr.R_PLPO) / 2.0;
	e = iter_arr.R_ALPO / a - 1.0;
	P = a * (1.0 - pow(e, 2));
	//r = P / (1.0 + e * cos(eta2));

	sv_LOI.R = R2;
	sv_LOI.V = V2;
	sv_LOI.GMT = T + dt;
	sv_LOI.RBI = OrbMech::Body_Moon;

	return sv_LOI;

	/*EnckeIntegratorInputTable enckein;
	double T;

	//Estimate time of LOI-2/DOI
	T = sv_LLS.GMT + dt3 - PI2 * sqrt(pow(a_LLS, 3) / OrbMech::mu_Moon) * (double)iter_arr.REVS2;
	//Integrate the LLS LPO back from T_LLS to T
	enckein.AnchorVector = sv_LLS;
	enckein.GMTBASE = iter_arr.GMTBASE;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = sv_LLS.GMT - T;
	enckein.IsForwardIntegration = -1.0;
	enckein.CutoffIndicator = 1;
	enckein.MDGSUN = &MDGSUN;

	encke.Propagate(enckein);*/
}

bool ApolloTrajectoryDesignProgram::CTBODY(VECTOR3 R0, VECTOR3 V0, double GMT0, double GMTF, int K, double mu, VECTOR3& RF, VECTOR3& VF) const
{
	double F1, F2, F3, F4, alpha;
	return CTBODY(R0, V0, GMT0, GMTF, K, mu, alpha, F1, F2, F3, F4, RF, VF);
}

bool ApolloTrajectoryDesignProgram::CTBODY(VECTOR3 R0, VECTOR3 V0, double GMT0, double GMTF, int K, double mu, double& alpha, double& F1, double& F2, double& F3, double& F4, VECTOR3& RF, VECTOR3& VF) const
{
	double dt;
	int i = 0;

	dt = GMTF - GMT0;

	if (abs(dt) < 1.0e-12)
	{
		RF = R0;
		VF = V0;
		return false;
	}

	double r0, v0, ainv, D0, beta, t, r;

	r0 = length(R0);
	v0 = length(V0);
	ainv = 2.0 / r0 - v0 * v0 / mu;
	D0 = dotp(R0, V0);

	//Initial guess
	beta = 1.0 / 5.0 * dt * sqrt(mu) / r0;

	do
	{
		alpha = -beta * beta * ainv;
		FCOMP(alpha, F1, F2, F3, F4);
		t = (beta * beta * F1 + D0 / sqrt(mu) * beta * F2 + r0 * F3) * beta / sqrt(mu);
		r = D0 / sqrt(mu) * beta * F3 + beta * beta * F2 + r0 * F4;
		beta = beta + (dt - t) * sqrt(mu) / r;
		i++;
		if (i == 20)
		{
			return true;
		}
	} while (abs((t - dt) / dt) >= 10e-12);

	double f, g, fdot, gdot;

	f = 1.0 - beta * beta * F2 / r0;
	g = t - pow(beta, 3) * F1 / sqrt(mu);
	fdot = -sqrt(mu) * beta * F3 / (r0 * r);
	gdot = 1.0 - beta * beta * F2 / r;
	RF = R0 * f + V0 * g;
	VF = R0 * fdot + V0 * gdot;

	if (K == 1 || ainv < 0.0)
	{
		return false;
	}

	VECTOR3 G0, Gdot0;

	G0 = RF;
	Gdot0 = VF;
	LIBRAT(G0, Gdot0, GMTF, 5);
	double n1, n2, n;

	n1 = Gdot0.z * G0.x - G0.z * Gdot0.x;
	n2 = Gdot0.z * G0.y - G0.z * Gdot0.y;
	n = sqrt(n1 * n1 + n2 * n2);
	if (n <= 1e-12)
	{
		return false;
	}

	VECTOR3 H;
	double cos_Omega, sin_Omega, cos_i, sin_i, dOmega, d, rr, vv;

	cos_Omega = n1 / n;
	sin_Omega = n2 / n;
	H = unit(crossp(G0, Gdot0));
	cos_i = H.z;
	sin_i = sqrt(H.x * H.x + H.y * H.y);
	dOmega = -1.5 * OrbMech::J2_Moon * pow(OrbMech::R_Moon,2) * sqrt(OrbMech::mu_Moon) * cos_i * pow(ainv, 3) * sqrt(ainv) * dt;
	d = dotp(G0, Gdot0);
	rr = dotp(G0, G0);
	vv = dotp(Gdot0, Gdot0);

	VECTOR3 N, M, G, Gdot;
	N = _V(cos_Omega * cos(dOmega) - sin_Omega * sin(dOmega), sin_Omega * cos(dOmega) + cos_Omega * sin(dOmega), 0.0);
	M = _V(-cos_i * (sin_Omega * cos(dOmega) + cos_Omega * sin(dOmega)), cos_i * (cos_Omega * cos(dOmega) - sin_Omega * sin(dOmega)), sin_i);
	G = N * (Gdot0.z * rr - G0.z * d) / n + M * G0.z * length(crossp(G0, Gdot0)) / n;
	Gdot = N * (Gdot0.z * d - G0.z * vv) / n + M * Gdot0.z * length(crossp(G0, Gdot0)) / n;
	LIBRAT(G, Gdot, GMTF, 6);
	RF = G;
	VF = Gdot;

	return false;
}

void ApolloTrajectoryDesignProgram::ELEMT(VECTOR3 R, VECTOR3 V, double mu, VECTOR3& H, double& a, double& e, double& i, double& n, double& P, double& eta) const
{
	double ainv, r, v, h;

	r = length(R);
	v = length(V);
	ainv = 2.0 / r - v * v / mu;
	a = 1.0 / ainv;
	e = sqrt(pow(1.0 - r * ainv, 2) + pow(dotp(R, V), 2) * ainv / mu);
	H = crossp(R, V);
	h = length(H);
	i = acos(H.z / h);
	n = pow(mu, 0.5) / pow(abs(a), 1.5);
	if (e != 0.0)
	{
		eta = atan2(h * dotp(R, V), pow(h, 2) - mu * r);
		if (eta < 0)
		{
			eta += PI2;
		}
	}
	else
	{
		eta = 0.0;
	}
	if (e < 1.0)
	{
		P = PI2 / n;
	}
}

void ApolloTrajectoryDesignProgram::FCOMP(double a, double& F1, double& F2, double& F3, double& F4) const
{
	F1 = OrbMech::stumpS(-a);
	F2 = OrbMech::stumpC(-a);

	F3 = a * F1 + 1.0;
	F4 = a * F2 + 1.0;
}

bool ApolloTrajectoryDesignProgram::LIBRAT(VECTOR3& R, double GMT, int K) const
{
	VECTOR3 V = _V(0, 0, 0);
	return LIBRAT(R, V, GMT, K);
}

bool ApolloTrajectoryDesignProgram::LIBRAT(VECTOR3& R, VECTOR3& V, double GMT, int K) const
{
	//Options
	//1: EMP to selenographic
	//2: Selenographic to EMP
	//3: EMP to selenocentric
	//4: Selenocentric to EMP
	//5: Selenocentric to selenographic
	//6: Selenographic to selenocentric

	//Return is error

	if (K == 1 || K == 2)
	{

	}
	else if (K == 3 || K == 4)
	{
		MATRIX3 A;
		VECTOR3 R_ME, V_ME, i, j, k;
		OrbMech::JPLEPH(MDGSUN, iter_arr.GMTBASE, 4, GMT, &R_ME, &V_ME, NULL, NULL);

		i = -unit(R_ME);
		k = unit(crossp(R_ME, V_ME));
		j = unit(crossp(k, i));

		A = _M(i.x, j.x, k.x, i.y, j.y, k.y, i.z, j.z, k.z);

		if (K == 3)
		{
			R = mul(A, R);
			V = mul(A, V);
		}
		else
		{
			R = tmul(A, R);
			V = tmul(A, V);
		}
	}
	else if (K == 5 || K == 6)
	{
		MATRIX3 PNL;
		OrbMech::JPLEPH(MDGSUN, iter_arr.GMTBASE, 5, GMT, NULL, NULL, NULL, &PNL);

		if (K == 5)
		{
			R = tmul(PNL, R);
			V = tmul(PNL, V);
		}
		else
		{
			R = mul(PNL, R);
			V = mul(PNL, V);
		}
	}

	return true;
}

void ApolloTrajectoryDesignProgram::LOPC(VECTOR3 R0, VECTOR3 V0, double GMT0, VECTOR3 L, int m, int n, double P, VECTOR3& R3, VECTOR3& V3, double& GMT3, double& mfm0, double& dpsi, VECTOR3& DV, double& t_L) const
{
	VECTOR3 R1, V1, R2, V2;
	double dt1, GMT1, dt2, dv_R;

	dt1 = P * (-0.25 + (double)m);
	GMT1 = GMT0 + dt1;
	CTBODY(R0, V0, GMT0, GMT1, 2, OrbMech::mu_Moon, R1, V1);
	dt2 = P * (double)(m + n);
	t_L = GMT0 + dt2;
	CTBODY(R0, V0, GMT0, t_L, 2, OrbMech::mu_Moon, R2, V2);
	LIBRAT(R2, V2, t_L, 5);
	dpsi = -asin(dotp(unit(crossp(R2, V2)), L));
	OrbMech::BURN(R1, V1, 0.0, 0.0, dpsi, iter_arr.isp_SPS, dv_R, mfm0, R3, V3);
	DV = V3 - V1;
	GMT3 = GMT1;
}

double ApolloTrajectoryDesignProgram::MCOMP(double m0, double dv, double isp) const
{
	return m0*exp(-dv / (isp * OrbMech::G0ER));
}

void ApolloTrajectoryDesignProgram::RNTSIM(VECTOR3 R, VECTOR3 V, double GMT, double lng_L, double& lat, double& lng, double& dlng) const
{
	VECTOR3 P, S;
	double r, v, theta, alpha_L, alpha_G;

	r = length(R);
	v = length(V);

	P = V / v / cos(gamma_reentry) - R / r * tan(gamma_reentry);
	theta = Reentry_range / 3443.933585;
	S = R / r * cos(theta) + P * sin(theta);

	lat = atan2(S.z, sqrt(S.x * S.x + S.y * S.y));
	alpha_L = atan2(S.y, S.x);
	alpha_G = BHA + GMT*OrbMech::w_Earth;

	dlng = alpha_L - alpha_G - lng_L;
	OrbMech::normalizeAngle(dlng);
	lng = alpha_L - alpha_G;
	OrbMech::normalizeAngle(lng);

	if (dlng > PI)
	{
		dlng -= PI2;
	}
	if (dlng < -PI)
	{
		dlng += PI2;
	}
}

void ApolloTrajectoryDesignProgram::RVIO(bool vecinp, VECTOR3& R, VECTOR3& V, double& r, double& v, double& theta, double& phi, double& gamma, double& psi) const
{
	if (vecinp)
	{
		r = length(R);
		v = length(V);
		phi = asin(R.z / r);
		theta = atan2(R.y, R.x);
		if (theta < 0)
		{
			theta += PI2;
		}
		gamma = asin(dotp(R, V) / r / v);
		psi = atan2(R.x * V.y - R.y * V.x, V.z * r - R.z * dotp(R, V) / r);
		//if (psi < 0)
		//{
		//	psi += PI2;
		//}
	}
	else
	{
		R = _V(cos(phi) * cos(theta), cos(phi) * sin(theta), sin(phi)) * r;
		V = mul(_M(cos(phi) * cos(theta), -sin(theta), -sin(phi) * cos(theta), cos(phi) * sin(theta), cos(theta), -sin(phi) * sin(theta), sin(phi), 0, cos(phi)), _V(sin(gamma), cos(gamma) * sin(psi), cos(gamma) * cos(psi)) * v);
	}
}

double ApolloTrajectoryDesignProgram::AzimuthDifference(VECTOR3 R_u, VECTOR3 U1, VECTOR3 U2) const
{
	VECTOR3 V_proj;
	double psi1, psi2;

	V_proj = unit(crossp(U1, R_u));
	psi1 = atan2(R_u.x * V_proj.y - R_u.y * V_proj.x, V_proj.z - R_u.z * dotp(R_u, V_proj));

	V_proj = unit(crossp(U2, R_u));
	psi2 = atan2(R_u.x * V_proj.y - R_u.y * V_proj.x, V_proj.z - R_u.z * dotp(R_u, V_proj));

	return psi2 - psi1;
}

LVTargetingObjectives ApolloTrajectoryDesignProgram::WriteLVTargetingObjectives(double LaunchAzimuth, int Window) const
{
	LVTargetingObjectives set;

	set.Year = Year;
	set.Month = Month;
	set.Day = Day;
	set.Opportunity = iter_arr.FirstOpportunity ? 1 : 2;
	set.LTS = iter_arr.T_L;
	set.TC = iter_arr.DT_EPO;
	set.C3 = iter_arr.C3;
	set.AZ = LaunchAzimuth;
	set.RCAM = iter_arr.R_LLS + iter_arr.h_pl;
	set.LATM = iter_arr.lat_pl;
	set.LONM = iter_arr.lng_pl;
	set.TF = iter_arr.sv_PC.GMT - iter_arr.sv_TLI_Cutoff.GMT;

	return set;
}

SFPDataSet ApolloTrajectoryDesignProgram::WriteSFPTable(double LaunchAzimuth, int Window) const
{
	SFPDataSet set;

	set.Year = Year;
	set.Day = RefDay;
	set.Opportunity = iter_arr.FirstOpportunity ? 1 : 2;
	set.LaunchAzimuth = LaunchAzimuth;
	set.Window = Window;
	set.lat_PC1 = set.lat_PC2 = set.lat_ND = iter_arr.lat_pl;
	set.lng_PC1 = set.lng_PC2 = set.lng_ND = iter_arr.lng_pl;
	set.h_PC1 = set.h_PC2 = set.h_ND = iter_arr.h_pl;
	set.GET_TLI = iter_arr.sv_TLI_Ignition.GMT - iter_arr.T_L;
	set.dpsi_LOI = iter_arr.dpsi_LOI;
	set.gamma_LOI = 0.0; //TBD
	set.T_LO = iter_arr.sv_TEI.GMT - iter_arr.sv_PC.GMT;
	set.DT_LLS = iter_arr.sv_LLS.GMT - iter_arr.sv_PC.GMT;
	set.ApproachAzimuth = iter_arr.psi_DS;
	set.lat_LLS = iter_arr.lat_LLS;
	set.lng_LLS = iter_arr.lng_LLS;
	set.R_LLS = iter_arr.R_LLS;
	set.dpsi_TEI = iter_arr.dpsi_TEI;
	set.dv_TEI = iter_arr.dv_TEI;
	set.T_TE = iter_arr.sv_EI.GMT - iter_arr.sv_TEI.GMT;
	set.I_FR = 0.0; //TBD
	set.GMT_PC1 = set.GMT_PC2 = set.GMT_ND = iter_arr.sv_PC.GMT;

	//Additional data not technically on the SFP
	set.lat_spl = iter_arr.lat_ip_pr;
	set.lng_spl = iter_arr.lng_ip_pr;

	if (set.lng_spl > PI)
	{
		set.lng_spl -= PI2;
	}

	return set;
}

void ApolloTrajectoryDesignProgram::ExportLVDataSet(std::string project) const
{
	std::vector<std::string> data;
	char Buff[128];
	std::string tempstr, ID;
	std::string columns[5];
	double ss;
	int hh, mm;
	unsigned i, j;

	std::vector<LVTargetingObjectives> set;

	//Sort by opportunity (2 and then 1), discard any data for which both opportunities haven't been written
	for (i = 0; i < LVTargetingDataTable.size(); i++)
	{
		if (LVTargetingDataTable[i].Opportunity == 2)
		{
			//It's a second opportunity, find the first opportunity to it and then write both
			for (j = 0; j < LVTargetingDataTable.size(); j++)
			{
				if (LVTargetingDataTable[j].Opportunity == 1 && round(LVTargetingDataTable[i].AZ*DEG) == round(LVTargetingDataTable[j].AZ*DEG))
				{
					set.push_back(LVTargetingDataTable[i]);
					set.push_back(LVTargetingDataTable[j]);
					break;
				}
			}
		}
	}

	const double ER2HR2ToKM2SEC2 = pow(OrbMech::R_Earth / 3600000.0, 2);

	for (i = 0; i < set.size(); i++)
	{
		//ID
		snprintf(Buff, 17, "%03.0lf%d%02d%02d%02d", set.at(i).AZ * DEG, set.at(i).Opportunity, set.at(i).Year % 100, set.at(i).Month, set.at(i).Day);
		ID.assign(Buff);
		ID += "000";

		//Card 1
		OrbMech::SStoHHMMSS(set.at(i).LTS * 3600.0, hh, mm, ss);
		snprintf(Buff, 17, "%02d%02d", hh, mm);
		columns[0].assign(Buff);
		snprintf(Buff, 17, "%.9lf", ss);
		tempstr.assign(Buff);
		columns[0] += OrbMech::FixedWidthString(tempstr, 12U);

		snprintf(Buff, 17, "%.8E", set.at(i).TC * 3600.0);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 16U);

		snprintf(Buff, 17, "%.8E", set.at(i).C3 * ER2HR2ToKM2SEC2);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 16U);

		snprintf(Buff, 17, "%.8E", set.at(i).AZ * DEG);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 16U);

		tempstr = ID + "01";
		columns[4] = OrbMech::FixedWidthString(tempstr, 16U);

		data.push_back(columns[0] + columns[1] + columns[2] + columns[3] + columns[4]);

		//Card 2
		tempstr = "";
		columns[0] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = "";
		columns[1] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = "";
		columns[2] = OrbMech::FixedWidthString(tempstr, 16U);
		snprintf(Buff, 17, "%.8E", set.at(i).LONM * DEG);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = ID + "02";
		columns[4] = OrbMech::FixedWidthString(tempstr, 16U);

		data.push_back(columns[0] + columns[1] + columns[2] + columns[3] + columns[4]);

		//Card 3
		tempstr = "";
		columns[0] = OrbMech::FixedWidthString(tempstr, 16U);
		snprintf(Buff, 17, "%.8E", set.at(i).RCAM * OrbMech::R_Earth / 1000.0);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 16U);
		snprintf(Buff, 17, "%.8E", set.at(i).LATM * DEG);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = "";
		columns[3] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = ID + "03";
		columns[4] = OrbMech::FixedWidthString(tempstr, 16U);

		data.push_back(columns[0] + columns[1] + columns[2] + columns[3] + columns[4]);

		//Card 4
		tempstr = "";
		columns[0] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = "";
		columns[1] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = "";
		columns[2] = OrbMech::FixedWidthString(tempstr, 16U);
		snprintf(Buff, 17, "%.8E", set.at(i).TF);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 16U);
		tempstr = ID + "04";
		columns[4] = OrbMech::FixedWidthString(tempstr, 16U);

		data.push_back(columns[0] + columns[1] + columns[2] + columns[3] + columns[4]);
	}

	//Output to file
	std::ofstream myfile;

	snprintf(Buff, 127, "./Projects/%s-LVTargetingObjectives.txt", project.c_str());
	myfile.open(Buff);

	for (unsigned i = 0; i < data.size(); i++)
	{
		myfile << data[i] << std::endl;
	}

	myfile.close();
}

void ApolloTrajectoryDesignProgram::SaveData()
{
	SFPDataSets.push_back(SFPData);
	LVTargetingDataTable.push_back(LVTargetingData);
}

void ApolloTrajectoryDesignProgram::DeleteStoredData()
{
	if (SFPDataSets.size() > 0) SFPDataSets.pop_back();
	if (LVTargetingDataTable.size() > 0) LVTargetingDataTable.pop_back();
}

void ApolloTrajectoryDesignProgram::ExportRTCCInitFile(int Year, int Month, int Day, double revs1, int revs2, int m, int n) const
{
	//Take data from first SFP set, if it exists
	if (SFPDataSets.size() == 0) return;
	if (LVTargetingDataTable.size() == 0) return;

	char Buff[128];
	std::ofstream myfile;

	snprintf(Buff, 128, "./Projects/%04d-%02d-%02d Init.txt", Year, Month, Day);
	myfile.open(Buff);

	if (myfile.is_open() == false) return;

	snprintf(Buff, 128, "LSLat %.3lf", SFPDataSets[0].lat_LLS * DEG);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "LSLng %.3lf", SFPDataSets[0].lng_LLS * DEG);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "LSRad %.3lf", SFPDataSets[0].R_LLS * OrbMech::ER2NM);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "TLAND %.3lf", SFPDataSets[0].GMT_ND + SFPDataSets[0].DT_LLS - LVTargetingDataTable[0].LTS);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "LOI_psi_DS %.3lf", SFPDataSets[0].ApproachAzimuth * DEG + 360.0);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "TLCC_AZ_min %.3lf", SFPDataSets[0].ApproachAzimuth * DEG);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "TLCC_AZ_max %.3lf", SFPDataSets[0].ApproachAzimuth * DEG);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "REVS1 %.2lf", revs1);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "REVS2 %d", revs2);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "LOPC_M %d", m);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "LOPC_N %d", n);
	myfile << Buff << std::endl;
	snprintf(Buff, 128, "PTPSite 0 EOM %.3lf %.3lf", SFPDataSets[0].lat_spl * DEG, SFPDataSets[0].lng_spl * DEG);
	myfile << Buff << std::endl;

	myfile.close();
}

void ApolloTrajectoryDesignProgram::ExportSFPDataSets(std::string project) const
{
	char Buff[128];
	std::ofstream myfile;

	snprintf(Buff, 128, "./Projects/%s-SFP.txt", project.c_str());
	myfile.open(Buff);

	for (unsigned i = 0; i < SFPDataSets.size(); i++)
	{
		ExportSFPDataSet(myfile, project, SFPDataSets[i]);
	}

	myfile.close();
}

void ApolloTrajectoryDesignProgram::ExportSFPDataSet(std::ofstream& file, std::string project, const SFPDataSet& set) const
{
	char Buff[128];
	std::string tempstr, ID;
	std::string columns[5];
	std::string cards[8];

	//ID
	ID = " " + std::to_string(set.Year % 100);
	snprintf(Buff, 17, "%03d", set.Day);
	ID.append(Buff);
	ID += std::to_string(set.Opportunity);
	snprintf(Buff, 17, "%03.0lf", set.LaunchAzimuth*DEG);
	ID.append(Buff);
	if (set.Window == 1)
	{
		tempstr = "P";
	}
	else
	{
		tempstr = "A";
	}
	ID += tempstr;

	//Card 1
	tempstr = std::to_string(set.Day);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);
	tempstr = std::to_string(set.Opportunity);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.LaunchAzimuth*DEG);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	if (set.Window == 1)
	{
		tempstr = "P";
	}
	else
	{
		tempstr = "A";
	}
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);
	columns[4] = ID + "1";

	cards[0] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 2
	snprintf(Buff, 17, "%.9E", set.lat_PC1 * DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.lng_PC1 * DEG);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.h_PC1 * OrbMech::ER2NM);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.GET_TLI);
	tempstr.assign(Buff);
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "2";

	cards[1] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 3
	snprintf(Buff, 17, "%.9E", set.lat_PC2 * DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.lng_PC2 * DEG);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.h_PC2 * OrbMech::ER2NM);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	tempstr = "";
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "3";

	cards[2] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 4
	snprintf(Buff, 17, "%.9E", set.dpsi_LOI * DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.gamma_LOI * DEG);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.T_LO);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.DT_LLS);
	tempstr.assign(Buff);
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "4";

	cards[3] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 5
	snprintf(Buff, 17, "%.9E", set.ApproachAzimuth * DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.lat_LLS * DEG);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.lng_LLS * DEG);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.R_LLS * OrbMech::ER2NM);
	tempstr.assign(Buff);
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "5";

	cards[4] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 6
	snprintf(Buff, 17, "%.9E", set.dpsi_TEI * DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.dv_TEI / OrbMech::FPS2ERPH);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.T_TE);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.I_FR*DEG);
	tempstr.assign(Buff);
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "6";

	cards[5] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 7
	snprintf(Buff, 17, "%.9E", set.GMT_PC1);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.GMT_PC2);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.GMT_ND);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	tempstr = "";
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "7";

	cards[6] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Card 8
	snprintf(Buff, 17, "%.9E", set.lat_ND*DEG);
	tempstr.assign(Buff);
	columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.lng_ND* DEG);
	tempstr.assign(Buff);
	columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

	snprintf(Buff, 17, "%.9E", set.h_ND*OrbMech::ER2NM);
	tempstr.assign(Buff);
	columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

	tempstr = "";
	columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

	columns[4] = ID + "8";

	cards[7] = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];

	//Output to file
	for (unsigned i = 0; i < 8; i++)
	{
		file << cards[i] << std::endl;
	}
}

bool ApolloTrajectoryDesignProgram::LoadMSFCPresetTape(std::string filename, MSFCPresetTape& tape) const
{
	//Return true if successful

	std::ifstream myfile;
	std::string fullpath, line;
	int i;

	fullpath = "./Projects/" + filename;

	myfile.open(fullpath);

	if (myfile.is_open() == false) return false;

	i = 1;

	while (std::getline(myfile, line))
	{
		if (i == 1) tape.Day = std::stoi(line);
		else if (i == 2) tape.Month = std::stoi(line);
		else if (i == 3) tape.Year = std::stoi(line);
		else if (i == 4) tape.TLO = std::stod(line);
		else if (i == 5) tape.RAO = std::stod(line);
		else if (i >= 8 && i <= 60) tape.AZ[i - 8] = std::stod(line);
		else if (i >= 61 && i <= 113) tape.TD_AZ[i - 61] = std::stod(line);
		else if (i >= 114 && i <= 128) tape.RAS1[i - 114] = std::stod(line);
		else if (i >= 129 && i <= 143) tape.DEC1[i - 129] = std::stod(line);
		else if (i >= 144 && i <= 158) tape.C31[i - 144] = std::stod(line);
		else if (i >= 159 && i <= 173) tape.COS1[i - 159] = std::stod(line);
		else if (i >= 174 && i <= 188) tape.EN1[i - 174] = std::stod(line);
		else if (i >= 191 && i <= 205) tape.RAS2[i - 191] = std::stod(line);
		else if (i >= 206 && i <= 220) tape.DEC2[i - 206] = std::stod(line);
		else if (i >= 221 && i <= 235) tape.C32[i - 221] = std::stod(line);
		else if (i >= 236 && i <= 250) tape.COS2[i - 236] = std::stod(line);
		else if (i >= 251 && i <= 265) tape.EN2[i - 251] = std::stod(line);
		else if (i == 266) tape.NumAzi = std::stoi(line);
		else if (i >= 268 && i <= 282) tape.TP[i - 268] = std::stod(line);
		else if (i == 283) tape.ALPHATS1 = std::stod(line);
		else if (i == 284) tape.BETA1 = std::stod(line);
		else if (i == 285) tape.RN1 = std::stod(line);
		else if (i == 286) tape.TST1 = std::stod(line);
		else if (i == 287) tape.F1 = std::stod(line);
		else if (i == 288) tape.T3R1 = std::stod(line);
		else if (i == 289) tape.TAU3R1 = std::stod(line);
		else if (i == 290) tape.DVBR1 = std::stod(line);
		else if (i == 293) tape.ALPHATS2 = std::stod(line);
		else if (i == 294) tape.BETA2 = std::stod(line);
		else if (i == 295) tape.RN2 = std::stod(line);
		else if (i == 296) tape.TST2 = std::stod(line);
		else if (i == 297) tape.F2 = std::stod(line);
		else if (i == 298) tape.T3R2 = std::stod(line);
		else if (i == 299) tape.TAU3R2 = std::stod(line);
		else if (i == 300) tape.DVBR2 = std::stod(line);
		else if (i == 304) tape.AZO = std::stod(line);
		else if (i == 305) tape.AZS = std::stod(line);
		else if (i >= 306 && i <= 312) tape.f[i - 306] = std::stod(line);
		else if (i >= 313 && i <= 319) tape.g[i - 313] = std::stod(line);
		else if (i >= 320 && i <= 324) tape.h1[i - 320] = std::stod(line);
		else if (i >= 325 && i <= 329) tape.h2[i - 325] = std::stod(line);
		else if (i >= 330 && i <= 334) tape.h3[i - 330] = std::stod(line);
		else if (i == 335) tape.TDS1 = std::stod(line);
		else if (i == 336) tape.TDS2 = std::stod(line);
		else if (i == 337) tape.TDS3 = std::stod(line);
		else if (i == 338) tape.TD1 = std::stod(line);
		else if (i == 339) tape.TD2 = std::stod(line);
		else if (i == 340) tape.TD3 = std::stod(line);
		else if (i == 341) tape.TSD1 = std::stod(line);
		else if (i == 342) tape.TSD2 = std::stod(line);
		else if (i == 343) tape.TSD3 = std::stod(line);

		i++;
	}

	return true;
}

std::string FormatID(std::string ID, int Opp, int Card)
{
	char Buff[8];
	std::string tempstr;

	snprintf(Buff, 7, "%01d%03d", Opp, Card);
	tempstr.assign(Buff);

	return OrbMech::FixedWidthString(ID + tempstr, 12U);
}

void ApolloTrajectoryDesignProgram::ConvertPresetTapeToRTCCPunchCards(PerformanceData perf, const std::vector<MSFCPresetTape>& tapes, std::string filename) const
{
	MSFCPresetTape tape;
	std::ofstream myfile;
	unsigned i, j;
	int RefDay;

	myfile.open(filename.c_str());

	if (myfile.is_open() == false) return;

	char Buff[18];
	std::string tempstr, ID;
	std::string columns[5], card;
	int opp, cardnum;

	const double ER2HR2ToKM2SEC2 = pow(OrbMech::R_Earth / 3600000.0, 2);

	cardnum = 1;

	for (j = 0; j < 10; j++)
	{
		if (j < tapes.size())
		{
			tape = tapes[j];
		}
		else
		{
			break;
		}

		if (tape.Year != 0)
		{
			RefDay = OrbMech::DayOfYear(tape.Year, tape.Month, tape.Day);
		}
		else
		{
			RefDay = 0;
		}

		snprintf(Buff, 17, "%02d%03d", tape.Year % 100, RefDay);
		ID.assign(Buff);

		opp = 1;

		tempstr = std::to_string(RefDay);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		tempstr = std::to_string(opp);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TP[0] / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.COS1[0]);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;

		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.C31[0] / ER2HR2ToKM2SEC2);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.EN1[0]);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.RAS1[0] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.DEC1[0] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		for (i = 0; i < 7; i++)
		{
			snprintf(Buff, 17, "%.8E", tape.TP[2 * i + 1] / HRS);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.COS1[2 * i + 1]);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.C31[2 * i + 1] / ER2HR2ToKM2SEC2);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.EN1[2 * i + 1]);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;

			snprintf(Buff, 17, "%.8E", tape.RAS1[2 * i + 1] * RAD);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.DEC1[2 * i + 1] * RAD);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.TP[2 * i + 2] / HRS);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.COS1[2 * i + 2]);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;

			snprintf(Buff, 17, "%.8E", tape.C31[2 * i + 2] / ER2HR2ToKM2SEC2);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.EN1[2 * i + 2]);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.RAS1[2 * i + 2] * RAD);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.DEC1[2 * i + 2] * RAD);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;
		}

		//Cards 24-46
		opp = 2;

		tempstr = std::to_string(RefDay);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		tempstr = std::to_string(opp);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TP[0] / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.COS2[0]);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;

		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.C32[0] / ER2HR2ToKM2SEC2);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.EN2[0]);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.RAS2[0] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.DEC2[0] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		for (i = 0; i < 7; i++)
		{
			snprintf(Buff, 17, "%.8E", tape.TP[2 * i + 1] / HRS);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.COS2[2 * i + 1]);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.C32[2 * i + 1] / ER2HR2ToKM2SEC2);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.EN2[2 * i + 1]);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;

			snprintf(Buff, 17, "%.8E", tape.RAS2[2 * i + 1] * RAD);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.DEC2[2 * i + 1] * RAD);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.TP[2 * i + 2] / HRS);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.COS2[2 * i + 2]);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;

			snprintf(Buff, 17, "%.8E", tape.C32[2 * i + 2] / ER2HR2ToKM2SEC2);
			tempstr.assign(Buff);
			columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.EN2[2 * i + 2]);
			tempstr.assign(Buff);
			columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.RAS2[2 * i + 2] * RAD);
			tempstr.assign(Buff);
			columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

			snprintf(Buff, 17, "%.8E", tape.DEC2[2 * i + 2] * RAD);
			tempstr.assign(Buff);
			columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

			columns[4] = FormatID(ID, opp, cardnum);

			card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
			myfile << card << std::endl;
			cardnum++;
		}
	}

	//Cards 461+

	cardnum = 461;

	for (j = 0; j < 10; j++)
	{
		if (j < tapes.size())
		{
			tape = tapes[j];
		}
		else
		{
			break;
		}

		if (tape.Year != 0)
		{
			RefDay = OrbMech::DayOfYear(tape.Year, tape.Month, tape.Day);
		}
		else
		{
			RefDay = 0;
		}

		opp = 1;

		tempstr = std::to_string(RefDay);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		tempstr = std::to_string(opp);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TST1 / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.BETA1 * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.ALPHATS1 * RAD);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.F1 * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.RN1 * 0.3048 / OrbMech::R_Earth);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.T3R1 / HRS);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.TAU3R1 / HRS);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		if (opp == 1)
		{
			snprintf(Buff, 17, "%.8E", perf.T_MRS);
		}
		else
		{
			snprintf(Buff, 17, "%.8E", 1.0 / HRS);
		}
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", (perf.TH1 / perf.MF1) * OrbMech::G0ER);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", perf.MF1);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.DVBR1 * HRS / OrbMech::R_Earth);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		if (opp == 1)
		{
			snprintf(Buff, 17, "%.8E", tape.TAU3R1 / HRS + perf.T_MRS);
		}
		else
		{
			snprintf(Buff, 17, "%.8E", (tape.TAU3R1 + 1.0) / HRS);
		}
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", 0.0);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", 0.0);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		opp = 2;

		tempstr = std::to_string(RefDay);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		tempstr = std::to_string(opp);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TST2 / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.BETA2 * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.ALPHATS2 * RAD);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.F2 * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.RN2 * 0.3048 / OrbMech::R_Earth);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.T3R2 / HRS);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.TAU3R2 / HRS);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		if (opp == 1)
		{
			snprintf(Buff, 17, "%.8E", perf.T_MRS);
		}
		else
		{
			snprintf(Buff, 17, "%.8E", 1.0 / HRS);
		}
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", (perf.TH1 / perf.MF1) * OrbMech::G0ER);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", perf.MF1);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.DVBR2 * HRS / OrbMech::R_Earth);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		if (opp == 1)
		{
			snprintf(Buff, 17, "%.8E", tape.TAU3R1 / HRS + perf.T_MRS);
		}
		else
		{
			snprintf(Buff, 17, "%.8E", (tape.TAU3R1 + 1.0) / HRS);
		}
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", 0.0);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", 0.0);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;
	}

	//Cards 541+

	cardnum = 541;
	opp = 0;

	for (j = 0; j < 10; j++)
	{
		if (j < tapes.size())
		{
			tape = tapes[j];
		}
		else
		{
			break;
		}

		if (tape.Year != 0)
		{
			RefDay = OrbMech::DayOfYear(tape.Year, tape.Month, tape.Day);
		}
		else
		{
			RefDay = 0;
		}

		tempstr = std::to_string(RefDay);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TLO / HRS);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.RAO * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", RefDay == 0 ? 0.0 : OrbMech::w_Earth);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", 0.0);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.TDS1 / HRS);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TDS2 / HRS);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TDS3 / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h1[0] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.h1[1] * RAD);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h1[2] * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h1[3] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h1[4] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.TD1 / HRS);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TSD1 / HRS);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h2[0] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h2[1] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.h2[2] * RAD);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h2[3] * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h2[4] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TD2 / HRS);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.TSD2 / HRS);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h3[0] * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h3[1] * RAD);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h3[2] * RAD);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;

		snprintf(Buff, 17, "%.8E", tape.h3[3] * RAD);
		tempstr.assign(Buff);
		columns[0] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.h3[4] * RAD);
		tempstr.assign(Buff);
		columns[1] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TD3 / HRS);
		tempstr.assign(Buff);
		columns[2] = OrbMech::FixedWidthString(tempstr, 17U);

		snprintf(Buff, 17, "%.8E", tape.TSD3 / HRS);
		tempstr.assign(Buff);
		columns[3] = OrbMech::FixedWidthString(tempstr, 17U);

		columns[4] = FormatID(ID, opp, cardnum);

		card = columns[0] + columns[1] + columns[2] + columns[3] + columns[4];
		myfile << card << std::endl;
		cardnum++;
	}

	myfile.close();
}

double ApolloTrajectoryDesignProgram::VariableLaunchAzimith(const MSFCPresetTape& tape, double T_L) const
{
	double T_D, A_Z;
	int i, N;

	T_D = T_L - tape.TLO;

	//Check if outside limits
	if (T_D < tape.TDS1)
	{
		i = 0;

		if (T_D < 0.0)
		{
			T_D = 0.0;
		}
	}
	else if (T_D < tape.TDS2)
	{
		i = 1;
	}
	else if (T_D <= tape.TDS3)
	{
		i = 2;
	}
	else
	{
		T_D = tape.TDS3;
		i = 2;
	}

	//Compute launch azimuth
	A_Z = 0.0;

	switch (i)
	{
	case 0:
		for (N = 0; N < 5; N++)
		{
			A_Z += tape.h1[N] * pow((T_D - tape.TD1) / tape.TSD1, N);
		}
		break;
	case 1:
		for (N = 0; N < 5; N++)
		{
			A_Z += tape.h2[N] * pow((T_D - tape.TD2) / tape.TSD2, N);
		}
		break;
	default:
		for (N = 0; N < 5; N++)
		{
			A_Z += tape.h3[N] * pow((T_D - tape.TD3) / tape.TSD3, N);
		}
		break;
	}

	return A_Z * RAD;
}