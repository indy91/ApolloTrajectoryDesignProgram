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
#include "OrbMech.h"
#include "EnckeIntegrator.h"
#include "GeneralizedIterator.h"
#include "QuickResponseTargetingProgram.h"

/*#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"*/

QuickResponseTargetingProgram::QuickResponseTargetingProgram()
{
	lambda_L = -80.604133 * RAD;
}

void QuickResponseTargetingProgram::RunTargetingOption(std::string project, PerformanceData perf, int day, bool freereturn, int splittime, int AltitudeOption, MSFCPresetTape& presets)
{
	unsigned i;

	//Get all cases for this launch day, sorted in pairs of TLI opportunity
	std::vector<LVTargetingObjectivesSet> objectives_day;

	for (i = 0; i < objectives.size(); i++)
	{
		if (objectives[i].set[0].Day == day)
		{
			objectives_day.push_back(objectives[i]);
		}
	}

	//Check if there are enough cases
	if (objectives_day.size() <= 1)
	{
		//Error
		return;
	}

	//Setup with numbers from first case
	if (SetupBasics(objectives_day[0].set[0].Year, objectives_day[0].set[0].Month, objectives_day[0].set[0].Day))
	{
		//Error
		return;
	}

	EnckeIntegratorInputTable enckein;
	LVTargetingObjectivesSet obj;
	double TLO;

	IsFreeReturn = freereturn;
	SplitTimeOption = splittime;

	//Constants
	iter_arr.F_I_SIVB = perf.TH1;
	iter_arr.F_SIVB = perf.TH2;
	iter_arr.WDOT_SIVB = perf.MF1;
	iter_arr.T_MRS = perf.T_MRS;
	iter_arr.Weight_Insertion = perf.SIVBWeight + perf.CSMWeight + perf.LMWeight;
	iter_arr.Area_SIVB = 365.0;
	iter_arr.RCA_E = 1.0 + 24.0 / OrbMech::ER2NM; //24 NM vacuum perigee

	EOIProcessorDataSet EOIData = OrbMech::GetEOIProcessorData(0, AltitudeOption);

	//Run all cases
	for (i = 0; i < objectives_day.size(); i++)
	{
		obj = objectives_day[i];

		//Simulate launch to orbit
		iter_arr.AZ = obj.set[0].AZ;
		OrbMech::EOIProcessor(EOIData, iter_arr.AZ, iter_arr.coord_EOI, iter_arr.DT_L);

		//TBD: CoV weight optimization

		//TRAJECTORY SELECTION (COPLANAR FREE RETURN AND HYPERSURFACE)
		//Search III: first or second opportunity coplanar
		iter_arr.sigma = 7.0 * RAD;
		if (SplitTimeOption == 1)
		{
			GETSET(1, obj);
		}
		else
		{
			GETSET(2, obj);
		}

		if (CoplanarSearch())
		{
			//Error
			continue;
		}

		//Search IV: Hypersurface
		/*HYPSRF(0);

		if (HypersurfaceSearch())
		{
			//Error
			continue;
		}
		HYPSRF(1);*/

		//Store and punch TLI presetting data
		if (SplitTimeOption == 1)
		{
			//Store data for 1st opportunity
			OPPEND(i, 1);
		}
		else if (SplitTimeOption == 2)
		{
			//Store data for 2nd opportunity
			OPPEND(i, 2);
		}

		//SPLIT LAUNCH TIME CALCULATION

		//Determine split launch time
		if (SplitTimeOption == 1 || SplitTimeOption == 2)
		{
			TLO = iter_arr.LTS; //Optimum 1st or 2nd TLI
		}
		else
		{
			//Run coplanar 1st TLI solution. For now, TLO is then at average value of LTS
			double TLO2 = iter_arr.LTS;

			GETSET(1, obj);

			if (CoplanarSearch())
			{
				//Error
				continue;
			}

			TLO = (TLO2 + iter_arr.LTS) / 2.0;
		}

		//TRAJECTORY SELECTION (PLANE CHANGE FREE RETURN AND HYPERSURFACE)
		if (SplitTimeOption == 1)
		{
			//Run second opportunity plane change
			GETSET(2, obj);
			iter_arr.LTS = TLO;

			if (PlaneChangeSearch())
			{
				//Error
				continue;
			}
			/*if (HypersurfaceSearch())
			{
				//Error
				continue;
			}*/
			OPPEND(i, 2);
		}
		else if (SplitTimeOption == 2)
		{
			//Run first opportunity plane change
			GETSET(1, obj);
			iter_arr.LTS = TLO;

			if (PlaneChangeSearch())
			{
				//Error
				continue;
			}
			/*if (HypersurfaceSearch())
			{
				//Error
				continue;
			}*/
			OPPEND(i, 1);
		}
		else
		{
			//Run both opportunities with plane change
			GETSET(1, obj);
			iter_arr.LTS = TLO;

			if (PlaneChangeSearch())
			{
				//Error
				continue;
			}
			/*if (HypersurfaceSearch())
			{
				//Error
				continue;
			}*/

			OPPEND(i, 1);

			GETSET(2, obj);
			iter_arr.LTS = TLO;

			if (PlaneChangeSearch())
			{
				//Error
				continue;
			}
			/*if (HypersurfaceSearch())
			{
				//Error
				continue;
			}*/

			OPPEND(i, 2);
		}
	}

	//All cases run, generate presettings

	unsigned j;

	//Find number of cases that converged
	OUTPUT.num = 0;
	while (OUTPUT.num < 14 && OUTPUT.data[0][OUTPUT.num].LaunchAzimuth != 0.0)
	{
		OUTPUT.num++;
	}

	if (OUTPUT.num == 0)
	{
		//Error
		return;
	}

	//Store presettings for LVDC and RTCC
	MSFCPresetTape presettings;

	presettings.Day = iter_arr.Day;
	presettings.Month = iter_arr.Month;
	presettings.Year = iter_arr.Year;
	presettings.TLO = OUTPUT.data[0][0].LaunchTime;
	presettings.RAO = OUTPUT.data[0][0].RAO;

	//TBD: Analytical launch azimuth model
	j = 0;
	for (i = 0; i < OUTPUT.num; i++)
	{
		presettings.AZ[i] = OUTPUT.data[0][i].LaunchAzimuth;
		presettings.TD_AZ[i] = OUTPUT.data[0][i].LaunchTime - OUTPUT.data[0][0].LaunchTime;
	}

	for (j = 0; j < OUTPUT.num; j++)
	{
		presettings.RAS1[j] = OUTPUT.data[0][j].RAS;
		presettings.DEC1[j] = OUTPUT.data[0][j].DEC;
		presettings.C31[j] = OUTPUT.data[0][j].C3_KM;
		presettings.COS1[j] = OUTPUT.data[0][j].COS_SIGMA;
		presettings.EN1[j] = OUTPUT.data[0][j].EN;

		presettings.RAS2[j] = OUTPUT.data[1][j].RAS;
		presettings.DEC2[j] = OUTPUT.data[1][j].DEC;
		presettings.C32[j] = OUTPUT.data[1][j].C3_KM;
		presettings.COS2[j] = OUTPUT.data[1][j].COS_SIGMA;
		presettings.EN2[j] = OUTPUT.data[1][j].EN;

		presettings.TP[j] = OUTPUT.data[0][j].LaunchTime - OUTPUT.data[0][0].LaunchTime;
	}
	
	//Use data from first launch azimuth
	presettings.F1 = OUTPUT.data[0][0].F;
	presettings.ALPHATS1 = OUTPUT.data[0][0].ALPHA;
	presettings.BETA1 = OUTPUT.data[0][0].BETA;
	presettings.RN1 = OUTPUT.data[0][0].RN;
	presettings.TAU3R1 = OUTPUT.data[0][0].TAU;
	presettings.T3R1 = OUTPUT.data[0][0].DT_MRS_TLI;
	presettings.TST1 = OUTPUT.data[0][0].DT_EOI_TB6 - DT_TST_TB6;

	presettings.F2 = OUTPUT.data[1][0].F;
	presettings.ALPHATS2 = OUTPUT.data[1][0].ALPHA;
	presettings.BETA2 = OUTPUT.data[1][0].BETA;
	presettings.RN2 = OUTPUT.data[1][0].RN;
	presettings.TAU3R2 = OUTPUT.data[1][0].TAU;
	presettings.T3R2 = OUTPUT.data[1][0].DT_MRS_TLI;
	presettings.TST2 = OUTPUT.data[1][0].DT_EOI_TB6 - DT_TST_TB6;

	presettings.NumAzi = OUTPUT.num;
	presettings.DVBR1 = presettings.DVBR2 = 3.44; //TBD
	presettings.AZO = 72.0;
	presettings.AZS = 36.0;
	presettings.f[0] = 32.55754; presettings.f[1] = -15.84615; presettings.f[2] = 11.6478; presettings.f[3] = 9.89097; presettings.f[4] = -5.11143;
	presettings.g[0] = 123.1935; presettings.g[1] = -55.06485; presettings.g[2] = 26.01324; presettings.g[3] = 26.01324; presettings.g[4] = -1.47591;

	//For now:
	presettings.h1[0] = presettings.AZ[0];
	presettings.h1[1] = presettings.AZ[1] - presettings.AZ[0];

	presettings.h2[0] = presettings.AZ[1];
	presettings.h2[1] = presettings.AZ[2] - presettings.AZ[1];

	presettings.TDS1 = presettings.TP[1];
	presettings.TDS2 = presettings.TP[2];
	presettings.TDS3 = presettings.TP[3];
	presettings.TD1 = 0.0;
	presettings.TD2 = presettings.TDS1;
	presettings.TD3 = presettings.TDS2;
	presettings.TSD1 = presettings.TDS1;
	presettings.TSD2 = presettings.TDS2 - presettings.TDS1;
	if (presettings.TDS3 != 0.0)
	{
		presettings.TSD3 = presettings.TDS3 - presettings.TDS2;
	}

	presets = presettings;

	char Buff[128];

	//Write preset tape
	sprintf_s(Buff, "./Projects/%s-Preset-Tape-%04d-%02d-%02d.txt", project.c_str(), iter_arr.Year, iter_arr.Month, iter_arr.Day);
	WritePresetTape(presettings, Buff);

	//Write LVDC presettings
	sprintf_s(Buff, "./Projects/%s-Presettings-%04d-%02d-%02d.txt", project.c_str(), iter_arr.Year, iter_arr.Month, iter_arr.Day);
	WritePresettings(AltitudeOption, presettings, Buff);

	//Write plots
	PLOT();
}

void QuickResponseTargetingProgram::RunTLMCOption()
{

}

bool QuickResponseTargetingProgram::SetupBasics(int Year, int Month, int Day)
{
	//Set up date related numbers
	iter_arr.Year = Year;
	iter_arr.Month = Month;
	iter_arr.Day = Day;

	//Error checking
	if (Year < 1951 || Year > 1999)
	{
		return true;
	}
	if (Month < 1 || Month > 12)
	{
		return true;
	}
	if (Day < 1 || Day > 31)
	{
		return true;
	}

	//Epoch is the closest turn of year
	iter_arr.Epoch = Year;
	if (Month >= 7)
	{
		iter_arr.Epoch++;
	}

	//Calculate day of the year
	iter_arr.RefDay = OrbMech::DayOfYear(Year, Month, Day);
	//Calculate MJD at midnight of launch
	double J_D = OrbMech::TJUDAT(Year, Month, Day);
	iter_arr.GMTBASE = J_D - 2400000.5;
	//Calculate hour angle
	iter_arr.BHA = OrbMech::HANGLE(iter_arr.Epoch, Year, iter_arr.RefDay);

	//Initialize ephemerides
	if (OrbMech::InitializeEphemerides(iter_arr.Epoch, iter_arr.GMTBASE, MDGSUN))
	{
		return true;
	}
	return false;
}

void QuickResponseTargetingProgram::LoadLVTargetingObjectives(std::string filename)
{
	std::string fullpath = "./Projects/" + filename;
	std::ifstream myfile(fullpath);

	if (myfile.is_open() == false) return;

	LVTargetingObjectives temp;
	std::string line, ID;
	double ss;
	int cardnum, hh, mm;

	objectives.clear();

	while (std::getline(myfile, line))
	{
		if (line.length() != 80U) continue; //Error

		ID = line.substr(65, 15);

		cardnum = std::stoi(ID.substr(14, 1));

		switch (cardnum)
		{
		case 1:
			hh = std::stoi(line.substr(0, 2));
			mm = std::stoi(line.substr(2, 2));
			ss = std::stod(line.substr(4, 12));
			temp.LTS = (double)hh + ((double)mm) / 60.0 + ss / 3600.0;
			temp.TC = std::stod(line.substr(17, 16)) / 3600.0;
			temp.C3 = std::stod(line.substr(33, 16)) / ER2HR2ToKM2SEC2;
			temp.AZ = std::stod(line.substr(49, 16)) * RAD;
			break;
		case 2:
			break;
		case 3:
			temp.RCAM = std::stod(line.substr(17, 16)) * 1000.0 / OrbMech::R_Earth;
			temp.LATM = std::stod(line.substr(33, 16)) * RAD;
			break;
		case 4:
			temp.TF = std::stod(line.substr(49, 16));
			break;
		}

		if (cardnum == 4)
		{
			//Final processing

			temp.Year = 1900 + std::stoi(ID.substr(4, 2));
			temp.Month = std::stoi(ID.substr(6, 2));
			temp.Day = std::stoi(ID.substr(8, 2));
			temp.Opportunity = std::stoi(ID.substr(3, 1));

			if (temp.Opportunity == 2)
			{
				objectives.push_back(LVTargetingObjectivesSet());
				objectives.back().set[0] = temp;
			}
			else
			{
				objectives.back().set[1] = temp;
			}
		}
	}

	myfile.close();
}

void QuickResponseTargetingProgram::GetPreprocessorPlotData(int type, std::vector<double>& xdata1, std::vector<double>& ydata1, std::vector<double>& xdata2, std::vector<double>& ydata2) const
{
	xdata1.clear();
	ydata1.clear();
	xdata2.clear();
	ydata2.clear();

	for (unsigned i = 0; i < objectives.size(); i++)
	{
		if (type == 0)
		{
			xdata1.push_back(objectives[i].set[1].AZ * DEG);
			ydata1.push_back(objectives[i].set[1].LATM * DEG);
			xdata2.push_back(objectives[i].set[0].AZ * DEG);
			ydata2.push_back(objectives[i].set[0].LATM * DEG);
		}
		else
		{
			xdata1.push_back(objectives[i].set[1].LTS);
			ydata1.push_back(objectives[i].set[1].AZ * DEG);
			xdata2.push_back(objectives[i].set[0].LTS);
			ydata2.push_back(objectives[i].set[0].AZ * DEG);
		}
	}
}

bool QuickResponseTargetingProgram::CoplanarSearch()
{
	iter_arr.HyperSurfaceSearch = false;

	void* constPtr;
	constPtr = &iter_arr;

	bool TLITrajectoryProcessorPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &TLITrajectoryProcessorPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[0] = true;
	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;

	block.IndVarGuess[0] = iter_arr.LTS;
	block.IndVarGuess[1] = iter_arr.TC;
	block.IndVarGuess[2] = iter_arr.C3;
	block.IndVarGuess[3] = iter_arr.sigma;
	block.IndVarGuess[4] = 0.0; //Delta

	block.IndVarStep[0] = 0.001;
	block.IndVarStep[1] = 0.001;
	block.IndVarStep[2] = 0.00001;
	block.IndVarStep[3] = 0.00001;

	block.IndVarWeight[0] = 512.0;
	block.IndVarWeight[1] = 512.0;
	block.IndVarWeight[2] = 512.0;
	block.IndVarWeight[3] = 512.0;

	block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;
	if (IsFreeReturn)
	{
		block.DepVarSwitch[2] = true; //Free return
	}
	else
	{
		block.DepVarSwitch[5] = true; //TLI to LOI time
	}
	//block.DepVarSwitch[3] = true; //TLI weight

	block.DepVarLowerLimit[0] = -0.0001;
	block.DepVarLowerLimit[1] = -0.0001;
	block.DepVarLowerLimit[2] = -0.0001;
	block.DepVarLowerLimit[3] = iter_arr.Weight_Insertion - 10000.0;
	block.DepVarLowerLimit[5] = iter_arr.DT_TLI_LOI - 0.0001;

	block.DepVarUpperLimit[0] = 0.0001;
	block.DepVarUpperLimit[1] = 0.0001;
	block.DepVarUpperLimit[2] = 0.0001;
	block.DepVarUpperLimit[3] = iter_arr.Weight_Insertion - 10000.0;
	block.DepVarUpperLimit[5] = iter_arr.DT_TLI_LOI + 0.0001;

	block.DepVarClass[0] = 1;
	block.DepVarClass[1] = 1;
	block.DepVarClass[2] = 1;
	block.DepVarClass[3] = 3;
	block.DepVarClass[5] = 1;

	block.DepVarWeight[3] = 1.0;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool QuickResponseTargetingProgram::HypersurfaceSearch()
{
	//Save data from previous converged run
	HBLOCK.sv_TLI_Ignition = iter_arr.sv_TLI_Ignition;
	HBLOCK.sv_TLI_Cutoff = iter_arr.sv_TLI_Cutoff;

	iter_arr.HyperSurfaceSearch = true;

	void* constPtr;
	constPtr = &iter_arr;

	bool TLITrajectoryProcessorPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &TLITrajectoryProcessorPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[4] = true;

	block.IndVarGuess[0] = iter_arr.LTS;
	block.IndVarGuess[1] = 0.0;// iter_arr.TC;
	block.IndVarGuess[2] = iter_arr.C3;
	block.IndVarGuess[3] = iter_arr.sigma;
	block.IndVarGuess[4] = 0.0; //Delta

	block.IndVarStep[0] = 0.001;
	block.IndVarStep[1] = 0.001;
	block.IndVarStep[2] = 0.00001;
	block.IndVarStep[3] = 0.00001;
	block.IndVarStep[4] = 0.00001;

	block.IndVarWeight[0] = 512.0;
	block.IndVarWeight[1] = 512.0;
	block.IndVarWeight[2] = 512.0;
	block.IndVarWeight[3] = 512.0;
	block.IndVarWeight[4] = 512.0;

	block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;

	block.DepVarLowerLimit[0] = -0.0001;
	block.DepVarLowerLimit[1] = -0.0001;

	block.DepVarUpperLimit[0] = 0.0001;
	block.DepVarUpperLimit[1] = 0.0001;

	block.DepVarClass[0] = 1;
	block.DepVarClass[1] = 1;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool QuickResponseTargetingProgram::PlaneChangeSearch()
{
	iter_arr.HyperSurfaceSearch = false;

	void* constPtr;
	constPtr = &iter_arr;

	bool TLITrajectoryProcessorPointer(void* data, std::vector<double>&var, void* varPtr, std::vector<double>&arr, bool mode);
	bool(*fptr)(void*, std::vector<double>&, void*, std::vector<double>&, bool) = &TLITrajectoryProcessorPointer;

	GenIterator::GeneralizedIteratorBlock block;

	block.IndVarSwitch[1] = true;
	block.IndVarSwitch[2] = true;
	block.IndVarSwitch[4] = true;

	block.IndVarGuess[0] = iter_arr.LTS;
	block.IndVarGuess[1] = iter_arr.TC;
	block.IndVarGuess[2] = iter_arr.C3;
	block.IndVarGuess[3] = iter_arr.sigma;
	block.IndVarGuess[4] = 0.0; //Delta

	block.IndVarStep[1] = 0.001;
	block.IndVarStep[2] = 0.00001;
	block.IndVarStep[4] = 0.00001;

	block.IndVarWeight[1] = 512.0;
	block.IndVarWeight[2] = 512.0;
	block.IndVarWeight[4] = 512.0;

	block.DepVarSwitch[0] = true;
	block.DepVarSwitch[1] = true;
	if (IsFreeReturn)
	{
		block.DepVarSwitch[2] = true; //Free return
	}
	else
	{
		block.DepVarSwitch[5] = true; //TLI to LOI time
	}

	block.DepVarLowerLimit[0] = -0.0001;
	block.DepVarLowerLimit[1] = -0.0001;
	block.DepVarLowerLimit[2] = -0.0001;
	block.DepVarLowerLimit[5] = iter_arr.DT_TLI_LOI - 0.0001;

	block.DepVarUpperLimit[0] = 0.0001;
	block.DepVarUpperLimit[1] = 0.0001;
	block.DepVarUpperLimit[2] = 0.0001;
	block.DepVarUpperLimit[5] = iter_arr.DT_TLI_LOI + 0.0001;

	block.DepVarClass[0] = 1;
	block.DepVarClass[1] = 1;
	block.DepVarClass[2] = 1;
	block.DepVarClass[5] = 1;

	std::vector<double> result;
	std::vector<double> y_vals;
	return GenIterator::GeneralizedIterator(fptr, block, constPtr, (void*)this, result, y_vals);
}

bool TLITrajectoryProcessorPointer(void* data, std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	return ((QuickResponseTargetingProgram*)data)->TLITrajectoryProcessor(var, varPtr, arr, mode);
}

bool QuickResponseTargetingProgram::TLITrajectoryProcessor(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	//Independent Variables:
	//0: Launch time, hours
	//1: Coast time in earth parking orbit, hours
	//2: TLI specific energy, (Er/hr)^2
	//3: perigee half angle
	//4: TLI plane change
	//Dependent variables:
	//0: D_BT
	//1: D_BR
	//2: Earth return error
	//3: TLI cutoff weight
	//4: Time of pericynthion
	//5: Time from TLI (cutoff) to pericynthion

	QRTPTLIGeneralizedIteratorArray* vars;
	vars = static_cast<QRTPTLIGeneralizedIteratorArray*>(varPtr);

	OrbMech::TLIBRNOutput tliout;
	EnckeIntegratorInputTable enckein;
	VECTOR3 V_EM;
	double T_INS, lng_iner;

	vars->LTS = var[0];
	vars->TC = var[1];
	vars->C3 = var[2];
	vars->sigma = var[3];
	vars->delta = var[4];

	//Setup generic integrator inputs
	enckein.GMTBASE = vars->GMTBASE;
	enckein.GMTLO = vars->LTS;
	enckein.Area = vars->Area_SIVB;
	enckein.DensityMultiplier = 1.0;
	enckein.MDGSUN = &MDGSUN;

	if (vars->HyperSurfaceSearch)
	{
		//Integrate to TLI ignition
		enckein.AnchorVector = vars->sv_TLI_Ignition_Hyper;
		enckein.VentPerturbationFactor = -1.0;
		enckein.Weight = vars->Weight_TLI_Ignition;
		enckein.MinIntegTime = 0.0;
		enckein.MaxIntegTime = abs(vars->TC);
		enckein.IsForwardIntegration = vars->TC >= 0.0 ? 1.0 : -1.0;
		enckein.CutoffIndicator = 1;
		enckein.Area = vars->Area_SIVB;

		encke.Propagate(enckein);

		if (enckein.TerminationCode != 1)
		{
			return true;
		}

		vars->sv_TLI_Ignition = enckein.sv_cutoff;
		vars->Weight_TLI_Ignition = enckein.Weight_cutoff;
	}
	else
	{
		//Convert spherical coordinates to inertial
		T_INS = vars->LTS + vars->DT_L;
		lng_iner = vars->BHA + vars->coord_EOI.lng + OrbMech::w_Earth * T_INS;
		OrbMech::adbar_from_rv(vars->coord_EOI.rad, vars->coord_EOI.vel, lng_iner, vars->coord_EOI.lat, vars->coord_EOI.fpa + PI05, vars->coord_EOI.azi, vars->sv_EOI.R, vars->sv_EOI.V);
		vars->sv_EOI.GMT = T_INS;
		vars->sv_EOI.RBI = OrbMech::Bodies::Body_Earth;

		//Integrate to TLI ignition
		enckein.AnchorVector = vars->sv_EOI;
		enckein.VentPerturbationFactor = 1.0;
		enckein.VentTable = &VentTable;
		enckein.Weight = vars->Weight_Insertion;
		enckein.MinIntegTime = 0.0;
		enckein.MaxIntegTime = abs(vars->TC);
		enckein.IsForwardIntegration = vars->TC >= 0.0 ? 1.0 : -1.0;
		enckein.CutoffIndicator = 1;
		enckein.Area = vars->Area_SIVB;

		encke.Propagate(enckein);

		if (enckein.TerminationCode != 1)
		{
			return true;
		}

		vars->sv_TLI_Ignition = enckein.sv_cutoff;
		vars->Weight_TLI_Ignition = enckein.Weight_cutoff;
	}

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

	TLIBRN(vars->sv_TLI_Ignition, vars->C3, vars->sigma, vars->delta, vars->F_SIVB, vars->F_I_SIVB, vars->Weight_TLI_Ignition, vars->WDOT_SIVB, T_MRS, tliout);
	vars->sv_TLI_Cutoff = tliout.sv2;
	vars->Weight_TLI_Cutoff = tliout.W2;
	vars->T_u = tliout.T;
	vars->alpha = tliout.alpha;
	vars->beta = tliout.beta;

	arr[3] = vars->Weight_TLI_Cutoff;

	//Propagate to pericynthion
	enckein.AnchorVector = vars->sv_TLI_Cutoff;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = 10.0 * 24.0;
	enckein.IsForwardIntegration = 1.0;
	enckein.EarthRelStopParam = 0.0;
	enckein.MoonRelStopParam = 0.0;
	enckein.StopParamRefFrame = 1;
	enckein.VentPerturbationFactor = -1.0;
	enckein.CutoffIndicator = 4;
	enckein.Weight = vars->Weight_TLI_Cutoff;

	encke.Propagate(enckein);

	if (enckein.TerminationCode != 4)
	{
		return true;
	}

	vars->sv_PC = enckein.sv_cutoff;

	//Calculate error in b-plane
	OrbMech::BPlaneTargetingPC(MDGSUN, vars->GMTBASE, vars->sv_PC, vars->RCA_M, vars->phi_star, vars->D_BT_M, vars->D_BR_M, vars->sv_PC2, vars->R_EM, V_EM);

	//Test
	/*MATRIX3 A;
	VECTOR3 i, j, k, R_ME, V_ME;
	VECTOR3 R_temp = vars->sv_PC.R;
	OrbMech::JPLEPH(MDGSUN, iter_arr.GMTBASE, 4, vars->sv_PC.GMT, &R_ME, &V_ME, NULL, NULL);

	i = -unit(R_ME);
	k = unit(crossp(R_ME, V_ME));
	j = unit(crossp(k, i));

	A = _M(i.x, j.x, k.x, i.y, j.y, k.y, i.z, j.z, k.z);
	R_temp = tmul(A, R_temp);
	double lat_pl = asin(R_temp.z / length(R_temp)); //atan(R_temp.z / sqrt(R_temp.x * R_temp.x + R_temp.y * R_temp.y));*/


	arr[0] = vars->D_BT_M;
	arr[1] = vars->D_BR_M;
	arr[4] = vars->sv_PC.GMT;
	arr[5] = vars->sv_PC.GMT - vars->sv_TLI_Cutoff.GMT;

	if (IsFreeReturn == false || vars->HyperSurfaceSearch)
	{
		return false;
	}

	//Propagate to perigee
	enckein.AnchorVector = vars->sv_PC2;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = 10.0 * 24.0;
	enckein.IsForwardIntegration = 1.0;
	enckein.EarthRelStopParam = 0.0;
	enckein.MoonRelStopParam = 0.0;
	enckein.StopParamRefFrame = 0;
	enckein.DensityMultiplier = 0.0;
	enckein.VentPerturbationFactor = -1.0;
	enckein.CutoffIndicator = 4;
	enckein.Area = vars->Area_SIVB;
	enckein.Weight = vars->Weight_TLI_Cutoff;

	encke.Propagate(enckein);

	if (enckein.CutoffIndicator != 4)
	{
		return true;
	}

	vars->sv_PG = enckein.sv_cutoff;

	//Found apogee
	//if (length(vars->sv_PG.R) > 30.0)
	//{
	//	return false;
	//}

	OrbMech::BPlaneTargetingPG(vars->sv_PG, vars->R_EM, V_EM, vars->RCA_E, vars->D_BT_E);

	arr[2] = vars->D_BT_E;

	return false;
}

bool TLMCTrajectoryProcessorPointer(void* data, std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	return ((QuickResponseTargetingProgram*)data)->TLMCTrajectoryProcessor(var, varPtr, arr, mode);
}

bool QuickResponseTargetingProgram::TLMCTrajectoryProcessor(std::vector<double>& var, void* varPtr, std::vector<double>& arr, bool mode)
{
	//Independent Variables:
	//0: Delta velocity at TLMC, Er/hr
	//1: Delta flight path angle at TLMC, rad
	//2: Delta azimuth at TLMC, rad
	//Dependent variables:
	//0: Radius of node, Er
	//1: Latitude of node, rad
	//2: Longitude of node, rad

	QRTPTLMCGeneralizedIteratorArray* vars;
	vars = static_cast<QRTPTLMCGeneralizedIteratorArray*>(varPtr);

	vars->dv_TLMC = var[0];
	vars->dgamma_TLMC = var[1];
	vars->dpsi_TLMC = var[2];

	EnckeIntegratorInputTable enckein;
	VECTOR3 R_temp, V_temp;
	double mfm0;

	OrbMech::BURN(vars->sv_TLMC.R, vars->sv_TLMC.V, vars->dv_TLMC, vars->dgamma_TLMC, vars->dpsi_TLMC, 1000.0, vars->dv_T_TLMC, mfm0, vars->sv_TLMC_apo.R, vars->sv_TLMC_apo.V);

	//Propagate to time of node
	enckein.AnchorVector = vars->sv_TLMC_apo;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = vars->GMT_nd;
	enckein.StopParamRefFrame = 0;
	enckein.CutoffIndicator = 1;
	enckein.IsForwardIntegration = 1.0;

	vars->sv_nd = enckein.sv_cutoff;

	//Calculate EMP conditions

	MATRIX3 A;
	VECTOR3 R_ME, V_ME, i, j, k;
	OrbMech::JPLEPH(MDGSUN, iter_arr.GMTBASE, 4, vars->sv_nd.GMT, &R_ME, &V_ME, NULL, NULL);

	i = -unit(R_ME);
	k = unit(crossp(R_ME, V_ME));
	j = unit(crossp(k, i));

	A = _M(i.x, j.x, k.x, i.y, j.y, k.y, i.z, j.z, k.z);

	R_temp = tmul(A, vars->sv_nd.R);
	V_temp = tmul(A, vars->sv_nd.V);

	vars->lat_nd = atan(R_temp.z / sqrt(R_temp.x * R_temp.x + R_temp.y * R_temp.y));
	vars->lng_nd = atan2(R_temp.y, R_temp.x);
	vars->r_nd = length(R_temp);

	var[0] = vars->r_nd;
	var[1] = vars->lat_nd;
	var[2] = vars->lng_nd;

	//if (IsFreeReturn == false)
	//{
		return false;
	//}

	/*
	//Propagate to perigee
	enckein.AnchorVector = vars->sv_nd;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = 10.0 * 24.0;
	enckein.IsForwardIntegration = 1.0;
	enckein.EarthRelStopParam = 0.0;
	enckein.MoonRelStopParam = 0.0;
	enckein.StopParamRefFrame = 0;
	enckein.DensityMultiplier = 0.0;
	enckein.VentPerturbationFactor = -1.0;
	enckein.CutoffIndicator = 4;
	enckein.Area = 0.0;
	enckein.Weight = 1.0;

	encke.Propagate(enckein);

	if (enckein.CutoffIndicator != 4)
	{
		return true;
	}

	vars->sv_PG = enckein.sv_cutoff;

	//Found apogee
	//if (length(vars->sv_PG.R) > 30.0)
	//{
	//	return false;
	//}

	//OrbMech::BPlaneTargetingPG(vars->sv_PG, vars->R_EM, vars->RCA_E, vars->D_BT_E);

	arr[2] = vars->D_BT_E;

	return false;*/
}

void QuickResponseTargetingProgram::GETSET(int entry, const LVTargetingObjectivesSet& obj)
{
	//1 = 1st opportunity, 2 = 2nd opportunity
	if (entry == 1)
	{
		//If first TLI opportunity is to be coplanar, set up inputs for it
		iter_arr.LTS = obj.set[1].LTS;
		iter_arr.TC = obj.set[1].TC;
		iter_arr.C3 = obj.set[1].C3;
		iter_arr.delta = 0.0;
		iter_arr.RCA_M = obj.set[1].RCAM;
		iter_arr.phi_star = obj.set[1].LATM;
		iter_arr.DT_TLI_LOI = obj.set[1].TF;
		iter_arr.FirstOpportunity = true;
	}
	else
	{
		//Otherwise, simulate second TLI opportunity as coplanar
		iter_arr.LTS = obj.set[0].LTS;
		iter_arr.TC = obj.set[0].TC;
		iter_arr.C3 = obj.set[0].C3;
		iter_arr.delta = 0.0;
		iter_arr.RCA_M = obj.set[0].RCAM;
		iter_arr.phi_star = obj.set[0].LATM;
		iter_arr.DT_TLI_LOI = obj.set[0].TF;
		iter_arr.FirstOpportunity = false;
	}
}

void QuickResponseTargetingProgram::HYPSRF(int entry)
{
	if (entry == 0)
	{
		//Set up perturbed TLI ignition state and save free-return data
		VECTOR3 R, V;
		VECTOR3 R_N; //Nominal reignition radius vector
		VECTOR3 V_N; //Nominal reignition velocity vector
		VECTOR3 N_N; //Nominal reignition normal vector
		VECTOR3 N_apo; //Rotated reignition normal vector
		VECTOR3 R_u_apo;
		VECTOR3 E;
		VECTOR3 V_u_apo;
		double Gamma;

		const double EPS = 0.1 * RAD; //Angle of rotation

		R = iter_arr.sv_TLI_Ignition.R;
		V = iter_arr.sv_TLI_Ignition.V;
		R_N = unit(R);
		V_N = unit(V);

		N_N = unit(crossp(R_N, V_N));
		N_apo = N_N * cos(EPS) + R_N * sin(EPS);
		R_u_apo = R_N * cos(EPS) - N_N * sin(EPS);
		E = unit(crossp(N_apo, R_u_apo));
		Gamma = asin(dotp(R_N, V_N));
		V_u_apo = E * cos(Gamma) + R_u_apo * sin(Gamma);

		//Save state vector
		iter_arr.sv_TLI_Ignition_Hyper = iter_arr.sv_TLI_Ignition;
		iter_arr.sv_TLI_Ignition_Hyper.R = R_u_apo * length(R);
		iter_arr.sv_TLI_Ignition_Hyper.V = V_u_apo * length(V);

		//Save free-return vectors
		VECTOR3 ECC, H;

		H = crossp(iter_arr.sv_TLI_Cutoff.R, iter_arr.sv_TLI_Cutoff.V);
		N_1 = unit(H);
		ECC = crossp(iter_arr.sv_TLI_Cutoff.V, H) / OrbMech::mu_Earth - unit(iter_arr.sv_TLI_Cutoff.R);
		P_E = unit(ECC);

		//Estimate target vector and sigma
		//VECTOR3 R_ME = unit(-iter_arr.R_EM);
		//double phi_apo = asin(dotp(N_1, R_ME));
		//VECTOR3 T_g = (R_ME - N_1 * sin(phi_apo)) / cos(phi_apo);
		//double sigma_g = acos(dotp(T_g, P_E)); //Document said asin
	}
	else
	{
		//Calculate sigma
		VECTOR3 A, T;
		//Translunar ellipse normal vector from perturnbed trajectory search
		VECTOR3 N_P;
		double EPS1;

		N_P = unit(crossp(iter_arr.sv_TLI_Cutoff.R, iter_arr.sv_TLI_Cutoff.V));
		EPS1 = acos(dotp(N_1, N_P));
		A = (N_P - N_1 * cos(EPS1)) / sin(EPS1);
		T = unit(crossp(A, N_1));
		iter_arr.sigma = acos(dotp(P_E, T));
	}
}

void QuickResponseTargetingProgram::OPPEND(int LAZ, int opp)
{
	if (LAZ >= 15) return;

	EnckeIntegratorInputTable enckein;
	OrbMech::EphemerisData sv_TB6;
	VECTOR3 R, V, H, E;
	double DEC, RAS, r, v, C3, e, f, alpha, beta, T_MRS;

	if (opp == 1)
	{
		T_MRS = iter_arr.T_MRS;
	}
	else
	{
		T_MRS = 1.0 / HRS;
	}

	//Integrate backwards to restart prep
	enckein.AnchorVector = iter_arr.sv_TLI_Ignition;
	enckein.GMTBASE = iter_arr.GMTBASE;
	enckein.GMTLO = iter_arr.LTS;
	enckein.MinIntegTime = 0.0;
	enckein.MaxIntegTime = DT_TB6_TIG;
	enckein.IsForwardIntegration = -1.0;
	enckein.DensityMultiplier = 1.0;
	enckein.VentPerturbationFactor = 1.0;
	enckein.CutoffIndicator = 1;
	enckein.Area = iter_arr.Area_SIVB;
	enckein.Weight = iter_arr.Weight_Insertion;
	enckein.VentTable = &VentTable;
	enckein.MDGSUN = &MDGSUN;

	encke.Propagate(enckein);

	sv_TB6 = enckein.sv_cutoff;

	//Convert target vector
	OrbMech::latlong_from_r(iter_arr.T_u, DEC, RAS);

	R = iter_arr.sv_TLI_Cutoff.R;
	V = iter_arr.sv_TLI_Cutoff.V;
	H = crossp(R, V);
	r = length(R);
	v = length(V);
	C3 = v * v - 2.0 * OrbMech::mu_Earth / r;

	E = crossp(V, H) / OrbMech::mu_Earth - unit(R);
	e = length(E);
	f = acos(dotp(unit(E), unit(R)));

	alpha = iter_arr.alpha + iter_arr.sigma;
	beta = alpha + acos(dotp(iter_arr.T_u, unit(sv_TB6.R)));

	OUTPUT.data[opp - 1][LAZ].LaunchAzimuth = iter_arr.AZ * DEG;
	OUTPUT.data[opp - 1][LAZ].LaunchTime = iter_arr.LTS * HRS;

	OUTPUT.data[opp - 1][LAZ].RAO = iter_arr.BHA + lambda_L + iter_arr.LTS * OrbMech::w_Earth;
	OrbMech::normalizeAngle(OUTPUT.data[opp - 1][LAZ].RAO, false);
	OUTPUT.data[opp - 1][LAZ].RAO *= DEG;

	OUTPUT.data[opp - 1][LAZ].DEC = DEC * DEG;
	OUTPUT.data[opp - 1][LAZ].RAS = RAS * DEG;
	OUTPUT.data[opp - 1][LAZ].COS_SIGMA = cos(iter_arr.sigma);
	OUTPUT.data[opp - 1][LAZ].C3_KM = C3 * ER2HR2ToKM2SEC2;
	OUTPUT.data[opp - 1][LAZ].C3_FT = OUTPUT.data[opp - 1][LAZ].C3_KM * pow(1000.0 / OrbMech::FT2M, 2);
	OUTPUT.data[opp - 1][LAZ].EN = e;

	OUTPUT.data[opp - 1][LAZ].RN = length(iter_arr.sv_TLI_Ignition.R) * OrbMech::R_Earth / OrbMech::FT2M;
	OUTPUT.data[opp - 1][LAZ].DT_EOI_TB6 = ((iter_arr.sv_TLI_Ignition.GMT - DT_TB6_TIG) - (iter_arr.LTS + iter_arr.DT_L)) * HRS;
	OUTPUT.data[opp - 1][LAZ].DT_MRS_TLI = (iter_arr.sv_TLI_Cutoff.GMT - (iter_arr.sv_TLI_Ignition.GMT + iter_arr.T_MRS)) * HRS;

	OUTPUT.data[opp - 1][LAZ].TAU = (iter_arr.Weight_TLI_Ignition / iter_arr.WDOT_SIVB - T_MRS) * HRS;
	OUTPUT.data[opp - 1][LAZ].T_u = iter_arr.T_u;

	OUTPUT.data[opp - 1][LAZ].F = f*DEG;
	OUTPUT.data[opp - 1][LAZ].ALPHA = alpha * DEG;
	OUTPUT.data[opp - 1][LAZ].BETA = beta * DEG;
}

void QuickResponseTargetingProgram::PLOTUtility(FILE* file, std::string filename, std::string title, std::string xname, int xtype, std::string yname, int ytype, int opp) const
{
	double xdata[15], ydata[15];
	unsigned i;

	for (i = 0; i < 15; i++)
	{
		xdata[i] = ydata[i] = 0.0;
	}

	if (xtype == 1)
	{
		for (i = 0; i < OUTPUT.num; i++)
		{
			xdata[i] = OUTPUT.data[opp - 1][i].LaunchTime - OUTPUT.data[opp - 1][0].LaunchTime;
		}
	}
	else
	{
		for (i = 0; i < OUTPUT.num; i++)
		{
			xdata[i] = OUTPUT.data[opp - 1][i].LaunchAzimuth;
		}
	}

	for (i = 0; i < OUTPUT.num; i++)
	{
		switch (ytype)
		{
		case 1:
			ydata[i] = OUTPUT.data[opp - 1][i].DEC;
			break;
		case 2:
			ydata[i] = OUTPUT.data[opp - 1][i].RAS;
			break;
		case 3:
			ydata[i] = OUTPUT.data[opp - 1][i].COS_SIGMA;
			break;
		case 4:
			ydata[i] = OUTPUT.data[opp - 1][i].C3_KM;
			break;
		case 5:
			ydata[i] = OUTPUT.data[opp - 1][i].EN;
			break;
		case 6:
			ydata[i] = OUTPUT.data[opp - 1][i].T_u.x;
			break;
		case 7:
			ydata[i] = OUTPUT.data[opp - 1][i].T_u.y;
			break;
		case 8:
			ydata[i] = OUTPUT.data[opp - 1][i].T_u.z;
			break;
		default:
			ydata[i] = 0.0;
			break;
		}
	}

	char Buff[128];
	/*std::ofstream myfile;

	myfile.open("Plots/tempplotdata.txt");

	sprintf_s(Buff, 128, "#%s %s", xname.c_str(), yname.c_str());
	myfile << Buff << std::endl;

	for (i = 0; i < OUTPUT.num; i++)
	{
		sprintf_s(Buff, 128, "%lf %lf", xdata[i], ydata[i]);
		myfile << Buff << std::endl;
	}

	myfile.close();*/

	//Reset
	//fprintf(file, "reset;\n");
	//Set up data save
	fprintf(file, "set terminal png size 640,480;\n");
	//Set file name
	sprintf_s(Buff, 128, "set output 'Plots/%s.png';\n", filename.c_str());
	fprintf(file, Buff);
	//Set title
	sprintf_s(Buff, 128, "set title '%s';\n", title.c_str());
	fprintf(file, Buff);
	//No legend
	fprintf(file, "unset key;\n");
	//Set labels
	sprintf_s(Buff, 128, "set xlabel '%s';\n", xname.c_str());
	fprintf(file, Buff);
	sprintf_s(Buff, 128, "set ylabel '%s';\n", yname.c_str());
	fprintf(file, Buff);
	//Plot
	fprintf(file, "plot '-' with lines\n");

	for (i = 0; i < OUTPUT.num; i++)
	{
		fprintf(file, "%lf %lf\n", xdata[i], ydata[i]);
	}

	// finally, e
	fprintf(file, "e\n");
	fflush(file);
}

void QuickResponseTargetingProgram::PLOT()
{
	//Open gnuplot
	FILE* pipe = _popen("gnuplot", "w"); // -persist

	if (pipe == NULL) return;

	//Save 32 plots
	PLOTUtility(pipe, "DELTL1 vs DECT1", "Target Declination (1st opportunity)", "DELTL1", 1, "DECT1", 1, 1);
	PLOTUtility(pipe, "DELTL2 vs DECT2", "Target Declination (2nd opportunity)", "DELTL2", 1, "DECT2", 1, 2);
	PLOTUtility(pipe, "DELTL1 vs RAST1", "Target Right Ascension (1st opportunity)", "DELTL1", 1, "RAST1", 2, 1);
	PLOTUtility(pipe, "DELTL2 vs RAST2", "Target Right Ascension (2nd opportunity)", "DELTL2", 1, "RAST2", 2, 2);
	PLOTUtility(pipe, "DELTL1 vs COSG1", "Cosine of Perigee Angle (1st opportunity)", "DELTL1", 1, "COSG1", 3, 1);
	PLOTUtility(pipe, "DELTL2 vs COSG2", "Cosine of Perigee Angle (2nd opportunity)", "DELTL2", 1, "COSG2", 3, 2);
	PLOTUtility(pipe, "DELTL1 vs C3KM1", "Twice Specific Energy (1st opportunity)", "DELTL1", 1, "C3KM1", 4, 1);
	PLOTUtility(pipe, "DELTL2 vs C3KM2", "Twice Specific Energy (2nd opportunity)", "DELTL2", 1, "C3KM2", 4, 2);
	PLOTUtility(pipe, "DELTL1 vs PLEN1", "Eccentricity at TLI (1st opportunity)", "DELTL1", 1, "PLEN1", 5, 1);
	PLOTUtility(pipe, "DELTL2 vs PLEN2", "Eccentricity at TLI (2nd opportunity)", "DELTL2", 1, "PLEN2", 5, 2);
	PLOTUtility(pipe, "DELTL1 vs PLTX1", "X Component of Target Vector (1st opportunity)", "DELTL1", 1, "PLTX1", 6, 1);
	PLOTUtility(pipe, "DELTL2 vs PLTX2", "X Component of Target Vector (2nd opportunity)", "DELTL2", 1, "PLTX2", 6, 2);
	PLOTUtility(pipe, "DELTL1 vs PLTY1", "Y Component of Target Vector (1st opportunity)", "DELTL1", 1, "PLTY1", 7, 1);
	PLOTUtility(pipe, "DELTL2 vs PLTY2", "Y Component of Target Vector (2nd opportunity)", "DELTL2", 1, "PLTY2", 7, 2);
	PLOTUtility(pipe, "DELTL1 vs PLTZ1", "Z Component of Target Vector (1st opportunity)", "DELTL1", 1, "PLTZ1", 8, 1);
	PLOTUtility(pipe, "DELTL2 vs PLTZ2", "Z Component of Target Vector (2nd opportunity)", "DELTL2", 1, "PLTZ2", 8, 2);

	PLOTUtility(pipe, "WELTL1 vs DECT1", "Target Declination (1st opportunity)", "WELTL1", 2, "DECT1", 1, 1);
	PLOTUtility(pipe, "WELTL2 vs DECT2", "Target Declination (2nd opportunity)", "WELTL2", 2, "DECT2", 1, 2);
	PLOTUtility(pipe, "WELTL1 vs RAST1", "Target Right Ascension (1st opportunity)", "WELTL1", 2, "RAST1", 2, 1);
	PLOTUtility(pipe, "WELTL2 vs RAST2", "Target Right Ascension (2nd opportunity)", "WELTL2", 2, "RAST2", 2, 2);
	PLOTUtility(pipe, "WELTL1 vs COSG1", "Cosine of Perigee Angle (1st opportunity)", "WELTL1", 2, "COSG1", 3, 1);
	PLOTUtility(pipe, "WELTL2 vs COSG2", "Cosine of Perigee Angle (2nd opportunity)", "WELTL2", 2, "COSG2", 3, 2);
	PLOTUtility(pipe, "WELTL1 vs C3KM1", "Twice Specific Energy (1st opportunity)", "WELTL1", 2, "C3KM1", 4, 1);
	PLOTUtility(pipe, "WELTL2 vs C3KM2", "Twice Specific Energy (2nd opportunity)", "WELTL2", 2, "C3KM2", 4, 2);
	PLOTUtility(pipe, "WELTL1 vs PLEN1", "Eccentricity at TLI (1st opportunity)", "WELTL1", 2, "PLEN1", 5, 1);
	PLOTUtility(pipe, "WELTL2 vs PLEN2", "Eccentricity at TLI (2nd opportunity)", "WELTL2", 2, "PLEN2", 5, 2);
	PLOTUtility(pipe, "WELTL1 vs PLTX1", "X Component of Target Vector (1st opportunity)", "WELTL1", 2, "PLTX1", 6, 1);
	PLOTUtility(pipe, "WELTL2 vs PLTX2", "X Component of Target Vector (2nd opportunity)", "WELTL2", 2, "PLTX2", 6, 2);
	PLOTUtility(pipe, "WELTL1 vs PLTY1", "Y Component of Target Vector (1st opportunity)", "WELTL1", 2, "PLTY1", 7, 1);
	PLOTUtility(pipe, "WELTL2 vs PLTY2", "Y Component of Target Vector (2nd opportunity)", "WELTL2", 2, "PLTY2", 7, 2);
	PLOTUtility(pipe, "WELTL1 vs PLTZ1", "Z Component of Target Vector (1st opportunity)", "WELTL1", 2, "PLTZ1", 8, 1);
	PLOTUtility(pipe, "WELTL2 vs PLTZ2", "Z Component of Target Vector (2nd opportunity)", "WELTL2", 2, "PLTZ2", 8, 2);

	//fprintf(pipe, "load QRTP_Plot_Script\n");

	//fprintf(pipe, "plot sin(x)\n");
	//fprintf(pipe, "plot tan(x)\n");

	_pclose(pipe);
}

void QuickResponseTargetingProgram::WritePresetTape(const MSFCPresetTape& tape, std::string filename)
{
	std::ofstream myfile;
	unsigned i;

	myfile.open(filename.c_str());

	if (myfile.is_open() == false) return;

	myfile << tape.Day << std::endl;
	myfile << tape.Month << std::endl;
	myfile << tape.Year << std::endl;
	myfile << tape.TLO << std::endl;
	myfile << tape.RAO << std::endl;
	myfile << std::endl << std::endl; //2 blanks
	for (i = 0; i < 53; i++)
	{
		myfile << tape.AZ[i] << std::endl;
	}
	for (i = 0; i < 53; i++)
	{
		myfile << tape.TD_AZ[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.RAS1[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.DEC1[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.C31[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.COS1[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.EN1[i] << std::endl;
	}
	myfile << std::endl << std::endl; //2 blanks
	for (i = 0; i < 15; i++)
	{
		myfile << tape.RAS2[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.DEC2[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.C32[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.COS2[i] << std::endl;
	}
	for (i = 0; i < 15; i++)
	{
		myfile << tape.EN2[i] << std::endl;
	}
	myfile << tape.NumAzi << std::endl;
	myfile << std::endl; //1 blank
	for (i = 0; i < 15; i++)
	{
		myfile << tape.TP[i] << std::endl;
	}
	myfile << tape.ALPHATS1 << std::endl;
	myfile << tape.BETA1 << std::endl;
	myfile << tape.RN1 << std::endl;
	myfile << tape.TST1 << std::endl;
	myfile << tape.F1 << std::endl;
	myfile << tape.T3R1 << std::endl;
	myfile << tape.TAU3R1 << std::endl;
	myfile << tape.DVBR1 << std::endl;
	myfile << std::endl << std::endl; //2 blanks
	myfile << tape.ALPHATS2 << std::endl;
	myfile << tape.BETA2 << std::endl;
	myfile << tape.RN2 << std::endl;
	myfile << tape.TST2 << std::endl;
	myfile << tape.F2 << std::endl;
	myfile << tape.T3R2 << std::endl;
	myfile << tape.TAU3R2 << std::endl;
	myfile << tape.DVBR2 << std::endl;
	myfile << std::endl << std::endl << std::endl; //3 blanks
	myfile << tape.AZO << std::endl;
	myfile << tape.AZS << std::endl;
	for (i = 0; i < 7; i++)
	{
		myfile << tape.f[i] << std::endl;
	}
	for (i = 0; i < 7; i++)
	{
		myfile << tape.g[i] << std::endl;
	}
	for (i = 0; i < 5; i++)
	{
		myfile << tape.h1[i] << std::endl;
	}
	for (i = 0; i < 5; i++)
	{
		myfile << tape.h2[i] << std::endl;
	}
	for (i = 0; i < 5; i++)
	{
		myfile << tape.h3[i] << std::endl;
	}
	myfile << tape.TDS1 << std::endl;
	myfile << tape.TDS2 << std::endl;
	myfile << tape.TDS3 << std::endl;
	myfile << tape.TD1 << std::endl;
	myfile << tape.TD2 << std::endl;
	myfile << tape.TD3 << std::endl;
	myfile << tape.TSD1 << std::endl;
	myfile << tape.TSD2 << std::endl;
	myfile << tape.TSD3 << std::endl;
	myfile << std::endl << std::endl << std::endl; //3 blanks

	myfile.close();
}

void QuickResponseTargetingProgram::WritePresettings(int AltitudeOption, const MSFCPresetTape& tape, std::string filename)
{
	//Altitude: 1 = 90 NM, 2 = 100 NM

	std::ofstream myfile;
	unsigned i;
	char Buff[128];

	myfile.open(filename.c_str());

	if (myfile.is_open() == false) return;

	if (AltitudeOption == 1)
	{
		myfile << "LVDC_R_T 6544846.0" << std::endl;
		myfile << "LVDC_V_T 7804.0613" << std::endl;
	}
	else
	{
		myfile << "LVDC_R_T 6563366.0" << std::endl;
		myfile << "LVDC_V_T 7793.0429" << std::endl;
	}

	sprintf_s(Buff, 128, "LVDC_T_LO %e", tape.TLO - DT_GRR * HRS);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_THTEO %e", tape.RAO * RAD - DT_GRR * OrbMech::w_Earth);
	myfile << Buff << std::endl;

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_RASA%d %e", i, tape.RAS1[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_DECA%d %e", i, tape.DEC1[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_C3A%d %e", i, tape.C31[i] * pow(1000.0, 2));
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_COSA%d %e", i, tape.COS1[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_ENA%d %e", i, tape.EN1[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_TPA%d %e", i, tape.TP[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_RASB%d %e", i, tape.RAS2[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_DECB%d %e", i, tape.DEC2[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_C3B%d %e", i, tape.C32[i] * pow(1000.0, 2));
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_COSB%d %e", i, tape.COS2[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_ENB%d %e", i, tape.EN2[i]);
		myfile << Buff << std::endl;
	}

	for (i = 0; i < 15; i++)
	{
		sprintf_s(Buff, 128, "LVDC_TPB%d %e", i, tape.TP[i]);
		myfile << Buff << std::endl;
	}

	sprintf_s(Buff, 128, "LVDC_ALFTSA %e", tape.ALPHATS1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_BETAA %e", tape.BETA1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_RNA %e", tape.RN1 * 0.3048);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_TSTA %e", tape.TST1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_FA %e", tape.F1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_T3PRA %e", tape.T3R1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_TAU3RA %e", tape.TAU3R1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_DVBRA %e", tape.DVBR1);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_ALFTSB %e", tape.ALPHATS2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_BETAB %e", tape.BETA2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_RNB %e", tape.RN2 * 0.3048);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_TSTB %e", tape.TST2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_FB %e", tape.F2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_T3PRB %e", tape.T3R2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_TAU3RB %e", tape.TAU3R2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_DVBRB %e", tape.DVBR2);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_AZO %e", tape.AZO);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_AZS %e", tape.AZS);
	myfile << Buff << std::endl;

	for (i = 0; i < 5; i++)
	{
		sprintf_s(Buff, 128, "LVDC_hx[0][%d] %e", i, tape.h1[i]);
		myfile << Buff << std::endl;
	}
	for (i = 0; i < 5; i++)
	{
		sprintf_s(Buff, 128, "LVDC_hx[1][%d] %e", i, tape.h2[i]);
		myfile << Buff << std::endl;
	}
	for (i = 0; i < 5; i++)
	{
		sprintf_s(Buff, 128, "LVDC_hx[2][%d] %e", i, tape.h3[i]);
		myfile << Buff << std::endl;
	}

	sprintf_s(Buff, 128, "LVDC_t_DS1 %e", tape.TDS1);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_DS2 %e", tape.TDS2);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_DS3 %e", tape.TDS3);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_t_D1 %e", tape.TD1);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_D2 %e", tape.TD2);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_D3 %e", tape.TD3);
	myfile << Buff << std::endl;

	sprintf_s(Buff, 128, "LVDC_t_SD1 %e", tape.TSD1);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_SD2 %e", tape.TSD2);
	myfile << Buff << std::endl;
	sprintf_s(Buff, 128, "LVDC_t_SD3 %e", tape.TSD3);
	myfile << Buff << std::endl;

	myfile.close();
}
