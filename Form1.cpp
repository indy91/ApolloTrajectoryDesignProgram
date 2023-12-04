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
#include "OrbMech.h"
#include "Form1.h"
#include "QuickResponseTargetingProgram.h"
#include <msclr\marshal_cppstd.h>
#include <fstream>

namespace CppCLRWinFormsProject {

	void Form1::CalculateElevationAngle()
	{
		double lat = Double::Parse(this->textBox1->Text);
		double lng = Double::Parse(this->textBox2->Text);
		int Year = Int32::Parse(this->textBox3->Text);
		int Month = Int32::Parse(this->textBox4->Text);
		int Day = Int32::Parse(this->textBox5->Text);
		double Hour = (double)Int32::Parse(this->textBox7->Text);

		ApolloTrajectoryDesignProgram traj;

		double Elev = traj.LunarSunElevationAngle(Year, Month, Day, Hour, lat, lng) * DEG;

		this->textBox6->Text = Elev.ToString("F2");

		//Copy inputs over to next tab
		this->textBox10->Text = this->textBox3->Text;
		this->textBox9->Text = this->textBox4->Text;

		int LaunchDay = (Int32::Parse(this->textBox5->Text) - 4);

		this->textBox8->Text = LaunchDay.ToString();
		this->textBox14->Text = this->textBox1->Text;
		this->textBox12->Text = this->textBox2->Text;
	}

	void Form1::CalculateTLIFirstGuess()
	{
		int Year = Int32::Parse(this->textBox10->Text);
		int Month = Int32::Parse(this->textBox9->Text);
		int Day = Int32::Parse(this->textBox8->Text);
		double Azi = Double::Parse(this->textBox13->Text);
		int AlitudeOption = this->comboBox4->SelectedIndex + 1;
		double lat = Double::Parse(this->textBox14->Text);
		double lng = Double::Parse(this->textBox12->Text);
		int LunarOrbits = Int32::Parse(this->textBox11->Text);
		int Window = this->comboBox1->SelectedIndex + 1;
		int Opportunity = System::Decimal::ToInt32(this->numericUpDown1->Value);

		ApolloTrajectoryDesignProgram traj;

		FirstGuessLogicDisplay disp = traj.CalculateFirstGuessLogic(Year, Month, Day, Azi, AlitudeOption, Window, Opportunity, LunarOrbits, lat, lng);

		this->richTextBox1->Text = "Launch: " + disp.Launchtime.ToString("F5") + "h GMT" + "\r\n";
		this->richTextBox1->Text += "TLI: " + disp.TLItime.ToString("F5") + "h GET" + "\r\n";
		this->richTextBox1->Text += "PC: " + disp.PCtime.ToString("F5") + "h GET" + "\r\n";
		this->richTextBox1->Text += "Landing: " + disp.LandingTime.ToString("F5") + "h GET" + "\r\n";
		this->richTextBox1->Text += "Sun Elevation: " + disp.LandingSunElevation.ToString("F5") + " deg";

		//Copy inputs over
		this->textBox18->Text = this->textBox10->Text;
		this->textBox17->Text = this->textBox9->Text;
		this->textBox16->Text = this->textBox8->Text;
		this->textBox15->Text = this->textBox13->Text;
		this->textBox19->Text = this->textBox14->Text;
		this->textBox20->Text = this->textBox12->Text;
		this->comboBox2->SelectedIndex = this->comboBox1->SelectedIndex;
	}

	void Form1::ConvergeOptimizedMission()
	{
		PerformanceData perf = ReadPerformanceData();

		int Year = Int32::Parse(this->textBox18->Text);
		int Month = Int32::Parse(this->textBox17->Text);
		int Day = Int32::Parse(this->textBox16->Text);
		double Azi = Double::Parse(this->textBox15->Text);

		OptimizedFullMissionInputs in;

		in.Year = Year;
		in.Month = Month;
		in.Day = Day;
		in.Azi = Azi;
		in.AlitudeOption = this->comboBox4->SelectedIndex + 1;
		in.Opportunity = System::Decimal::ToInt32(this->numericUpDown2->Value);
		in.Window = this->comboBox2->SelectedIndex + 1;
		in.lat_LLS = Double::Parse(this->textBox19->Text);
		in.lng_LLS = Double::Parse(this->textBox20->Text);
		in.H_LLS = Double::Parse(this->textBox21->Text);
		in.DW = -15.0;
		in.H_ALPO = 170.0;
		in.H_PLPO = 60.0;
		in.R1 = Int32::Parse(this->textBox24->Text);
		in.h_a = 60.0;
		in.h_p = 60.0;
		in.R2 = Int32::Parse(this->textBox25->Text);
		in.psi_DS = Double::Parse(this->textBox22->Text);
		in.DT_TLI_LOI = Double::Parse(this->textBox23->Text);
		in.FreeReturn = this->checkBox2->Checked;
		in.REVS3 = Int32::Parse(this->textBox26->Text);
		in.REVS4 = Int32::Parse(this->textBox27->Text);
		in.REVS5 = Int32::Parse(this->textBox28->Text);
		in.MaxInclination = Double::Parse(this->textBox29->Text);
		in.lambda_IP = Double::Parse(this->textBox30->Text);
		in.DT_TEC = Double::Parse(this->textBox31->Text);

		ConvergedMissionDisplay disp;
		
		int err = atdp->OptimizedFullMission(perf, in, disp);

		if (err)
		{
			this->richTextBox2->Text = "Error " + err.ToString("D");
		}
		else
		{
			this->richTextBox2->Text = "Launch: " + disp.Launchtime.ToString("F5") + "h GMT" + "\r\n";
			this->richTextBox2->Text += "TLI Ignition: " + disp.TLIIgnitionTime.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "TLI Cutoff: " + disp.TLICutoffTime.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "PC: " + disp.GMT_PC.ToString("F5") + "h GMT" + "\r\n";
			this->richTextBox2->Text += "PC: " + disp.GET_PC.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "Latitude of PC: " + disp.lat_PC.ToString("F2") + " deg" + "\r\n";
			this->richTextBox2->Text += "LOI-1 DV: " + disp.dv_LOI1.ToString("F1") + " ft/s" + "\r\n";
			this->richTextBox2->Text += "LOI-1 Plane Change: " + disp.dpsi_LOI1.ToString("F3") + " deg" + "\r\n";
			this->richTextBox2->Text += "Mass after LOI-1: " + disp.m_LOI1.ToString("F0") + " lbs" + "\r\n";
			this->richTextBox2->Text += "Time of LOI-2: " + disp.T_LOI2.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "LOI-2 DV: " + disp.dv_LOI2.ToString("F1") + " ft/s" + "\r\n";
			this->richTextBox2->Text += "Mass after LOI-2: " + disp.m_LOI2.ToString("F0") + " lbs" + "\r\n";
			this->richTextBox2->Text += "Landing: " + disp.LandingTime.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "Sun Elevation: " + disp.LandingSunElevation.ToString("F1") + " deg" + "\r\n";
			this->richTextBox2->Text += "Approach Azimuth: " + disp.azi_approach.ToString("F2") + " deg" + "\r\n";
			this->richTextBox2->Text += "Time of LOPC: " + disp.LOPCTime.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "LOPC DV: " + disp.dv_LOPC.ToString("F1") + " ft/s" + "\r\n";
			this->richTextBox2->Text += "Mass after LOPC: " + disp.m_LOPC.ToString("F0") + " lbs" + "\r\n";
			this->richTextBox2->Text += "Lunar liftoff: " + disp.AscentTime.ToString("F5") + " h GET" + "\r\n";
			this->richTextBox2->Text += "Time of TEI: " + disp.TEITime.ToString("F5") + "h GET" + "\r\n";
			this->richTextBox2->Text += "TEI DV: " + disp.dv_TEI.ToString("F1") + " ft/s" + "\r\n";
			this->richTextBox2->Text += "TEI Plane Change: " + disp.dpsi_TEI.ToString("F3") + " deg" + "\r\n";
			this->richTextBox2->Text += "Mass after TEI: " + disp.m_TEI.ToString("F0") + " lbs" + "\r\n";
			this->richTextBox2->Text += "Time of EI: " + disp.EITime.ToString("F5") + " h GET" + "\r\n";
			this->richTextBox2->Text += "Inclination of EI: " + disp.EIInclination.ToString("F2") + " deg" + "\r\n";
			this->richTextBox2->Text += "Splashdown latitude: " + disp.lat_spl.ToString("F2") + " deg" + "\r\n";
			this->richTextBox2->Text += "Splashdown longitude: " + disp.lng_spl.ToString("F2") + " deg" + "\r\n";

			this->richTextBox2->Text += "\r\n" + "DPS Abort (PC+2): " + "\r\n";
			this->richTextBox2->Text += "DV: " + disp.dv_PC2.ToString("F1") + " ft/s" + "\r\n";
			this->richTextBox2->Text += "Return inclination: " + disp.EIInclination_PC2.ToString("F2") + " deg" + "\r\n";

			this->richTextBox2->Text += "\r\n" + "Remaining DV: " + disp.dv_remain.ToString("F1") + " ft/s" + "\r\n";
		}
	}

	void Form1::ExportDataSets()
	{
		std::string project = msclr::interop::marshal_as<std::string>(this->textBox42->Text);

		atdp->ExportSFPDataSets(project);
		atdp->ExportLVDataSet(project);

		this->richTextBox2->Text = "SFP saved under " + this->textBox42->Text + "-SFP.txt" + "\r\n";
		this->richTextBox2->Text += "LV targeting objectives saved under " + this->textBox42->Text + "-LVTargetingObjectives.txt" + "\r\n";
	}

	void Form1::UpdateDisplayedLVTargeting()
	{
		std::vector<LVTargetingObjectives>* LVTargetingDataTable = atdp->GetLVTargetingDataTable();

		this->richTextBox3->Text = "Sets:" + "\r\n";

		for (unsigned i = 0; i < LVTargetingDataTable->size(); i++)
		{
			this->richTextBox3->Text += "Opp: " + LVTargetingDataTable->at(i).Opportunity + " Azi: " + (LVTargetingDataTable->at(i).AZ * DEG).ToString("F2") + "\r\n";
		}
	}

	using namespace System::Windows::Forms::DataVisualization::Charting;

	void Form1::LoadLVTargetingObjectives()
	{
		std::vector<double> xdata1, ydata1, xdata2, ydata2;

		std::string name = msclr::interop::marshal_as<std::string>(this->textBox32->Text);

		qrtp->LoadLVTargetingObjectives(name);

		//Launch azimuth
		qrtp->GetPreprocessorPlotData(1, xdata1, ydata1, xdata2, ydata2);

		this->chart2->Series->Clear();


		Series^ series3 = gcnew Series;

		series3->ChartType = SeriesChartType::Line;
		series3->Name = "AZ1 vs LT1";

		for (unsigned i = 0; i < xdata1.size(); i++)
		{
			series3->Points->AddXY(xdata1[i], ydata1[i]);
		}

		this->chart2->Series->Add(series3);

		Series^ series4 = gcnew Series;

		series4->ChartType = SeriesChartType::Line;
		series4->Name = "AZ2 vs LT2";

		for (unsigned i = 0; i < xdata2.size(); i++)
		{
			series4->Points->AddXY(xdata2[i], ydata2[i]);
		}

		this->chart2->Series->Add(series4);

		this->chart2->ChartAreas[0]->AxisY->Minimum = 72.0;
		this->chart2->ChartAreas[0]->AxisY->Maximum = 108.0;

		//Latitude of pericynthion
		qrtp->GetPreprocessorPlotData(0, xdata1, ydata1, xdata2, ydata2);

		this->chart1->Series->Clear();

		Series^ series1 = gcnew Series;

		series1->ChartType = SeriesChartType::Line;
		series1->Name = "LAT Opp 1";

		for (unsigned i = 0; i < xdata1.size(); i++)
		{
			series1->Points->AddXY(xdata1[i], ydata1[i]);
		}

		this->chart1->Series->Add(series1);

		Series^ series2 = gcnew Series;

		series2->ChartType = SeriesChartType::Line;
		series2->Name = "LAT Opp 2";

		for (unsigned i = 0; i < xdata2.size(); i++)
		{
			series2->Points->AddXY(xdata2[i], ydata2[i]);
		}

		this->chart1->Series->Add(series2);

		//delete series1;

		this->richTextBox4->Text = "";

		for (unsigned i = 0; i < qrtp->objectives.size(); i++)
		{
			this->richTextBox4->Text += "Day: " + qrtp->objectives.at(i).set[0].Day + ", Azi: " + (qrtp->objectives.at(i).set[0].AZ * DEG).ToString("F2") + "\r\n";
		}
	}

	void Form1::RunQRTPCases()
	{
		MSFCPresetTape presets;
		PerformanceData perf = ReadPerformanceData();

		std::string project = msclr::interop::marshal_as<std::string>(this->textBox42->Text);
		int Day = Int32::Parse(this->textBox33->Text);
		bool IsFreeReturn = this->checkBox1->Checked;
		int SplitTimeOption = this->comboBox3->SelectedIndex + 1;
		int AltitudeOption = this->comboBox4->SelectedIndex + 1;

		qrtp->RunTargetingOption(project, perf, Day, IsFreeReturn, SplitTimeOption, AltitudeOption, presets);

		//Preload preset tape name
		char Buffer[128];
		snprintf(Buffer, 127, "%s-Preset-Tape-%04d-%02d-%02d.txt", project.c_str(), presets.Year, presets.Month, presets.Day);
		this->textBox44->Text = gcnew String(Buffer);

		snprintf(Buffer, 127, "%s-Presettings-%04d-%02d-%02d.txt", project.c_str(), presets.Year, presets.Month, presets.Day);
		String^ tempstr = gcnew String(Buffer);

		//Text output
		double ss;
		int hh, mm;
		OrbMech::SStoHHMMSS(presets.TLO, hh, mm, ss);

		this->richTextBox5->Text = "";
		this->richTextBox5->Text += "Launch window opening: " + hh.ToString("D2") + ":" + mm.ToString("D2") + ":" + ss.ToString("F0") + "\r\n";
		this->richTextBox5->Text += "Launch azimuth: " + presets.AZ[0].ToString("F3") + " deg" + "\r\n";
		this->richTextBox5->Text += "LVDC presettings saved under " + tempstr + "\r\n";
		this->richTextBox5->Text += "Preset tape saved under " + this->textBox44->Text + "\r\n";

		
	}

	PerformanceData Form1::ReadPerformanceData()
	{
		PerformanceData data;

		data.CSMWeight = Double::Parse(this->textBox34->Text);
		data.SPSPropellant = Double::Parse(this->textBox43->Text);
		data.LMWeight = Double::Parse(this->textBox35->Text);
		data.SIVBWeight = Double::Parse(this->textBox36->Text);
		data.T_MRS = Double::Parse(this->textBox37->Text) / HRS;
		data.TH1 = Double::Parse(this->textBox38->Text);
		data.TH2 = Double::Parse(this->textBox39->Text);
		data.MF1 = Double::Parse(this->textBox40->Text) * HRS;
		data.MF2 = Double::Parse(this->textBox41->Text) * HRS;

		return data;
	}

	void Form1::GenerateFinalLaunchData()
	{
		PerformanceData perf = ReadPerformanceData();
		std::string project = msclr::interop::marshal_as<std::string>(this->textBox42->Text);
		std::string preset = msclr::interop::marshal_as<std::string>(this->textBox44->Text);

		MSFCPresetTape presettings;
		double data[3];
		char Buffer[128];

		atdp->CalculateLaunchData(perf, project, preset, presettings, data);

		double ss;
		int hh, mm;
		OrbMech::SStoHHMMSS(data[0], hh, mm, ss);

		sprintf_s(Buffer, "%s-RTCC-%04d-%02d-%02d.txt", project.c_str(), presettings.Year, presettings.Month, presettings.Day);
		String^ tempstr = gcnew String(Buffer);

		this->richTextBox6->Text = "";
		this->richTextBox6->Text += "Preferred launch time: " + hh.ToString("D2") + ":" + mm.ToString("D2") + ":" + ss.ToString("F0") + "\r\n";
		this->richTextBox6->Text += "Launch azimuth: " + data[1].ToString("F3") + " deg" + "\r\n";
		this->richTextBox6->Text += "MJD at T-4h: " + data[2].ToString("F9") + "\r\n";
		this->richTextBox6->Text += "RTCC TLI File saved under " + tempstr + "\r\n";
	}

	void Form1::SaveToFile()
	{
		std::string project = msclr::interop::marshal_as<std::string>(this->textBox42->Text);
		std::string projectfile = "Saves/" + project + "-Project.sav";
		std::ofstream myfile(projectfile.c_str(), std::ios::out | std::ios::binary);

		if (myfile.is_open() == false) return;

		SaveFileData data;

		data.dValues[0] = Double::Parse(this->textBox1->Text); //Lighting - LSLat
		data.dValues[1] = Double::Parse(this->textBox2->Text); //Lighting - LSLng
		data.dValues[2] = Double::Parse(this->textBox12->Text); //First Guess - LSLng
		data.dValues[3] = Double::Parse(this->textBox13->Text); //First Guess - Launch Azimuth
		data.dValues[4] = Double::Parse(this->textBox14->Text); //First Guess - LSLat
		data.dValues[5] = Double::Parse(this->textBox15->Text); //Target Objectives - Launch Azimuth
		data.dValues[6] = Double::Parse(this->textBox19->Text); //Target Objectives - LSLat
		data.dValues[7] = Double::Parse(this->textBox20->Text); //Target Objectives - LSLng
		data.dValues[8] = Double::Parse(this->textBox21->Text); //Target Objectives - LSAlt
		data.dValues[9] = Double::Parse(this->textBox22->Text); //Target Objectives - LSAzi
		data.dValues[10] = Double::Parse(this->textBox23->Text); //Target Objectives - DT TLI PC
		data.dValues[11] = Double::Parse(this->textBox29->Text); //Target Objectives - Max Inclination
		data.dValues[12] = Double::Parse(this->textBox30->Text); //Target Objectives - Splashdown longitude
		data.dValues[13] = Double::Parse(this->textBox31->Text); //Target Objectives - TEC time
		data.dValues[14] = Double::Parse(this->textBox34->Text); //Init - CSM weight
		data.dValues[15] = Double::Parse(this->textBox35->Text); //Init - LM weight
		data.dValues[16] = Double::Parse(this->textBox36->Text); //Init - S-IVB weight
		data.dValues[17] = Double::Parse(this->textBox37->Text); //Init - Time of MRS
		data.dValues[18] = Double::Parse(this->textBox38->Text); //Init - S-IVB thrust (pre MRS)
		data.dValues[19] = Double::Parse(this->textBox39->Text); //Init - S-IVB thrust (post MRS)
		data.dValues[20] = Double::Parse(this->textBox40->Text); //Init - S-IVB MFR (pre MRS)
		data.dValues[21] = Double::Parse(this->textBox41->Text); //Init - S-IVB MFR (post MRS)
		data.dValues[22] = Double::Parse(this->textBox43->Text); //Init - SPS propellant
		for (int i = 23; i < 30; i++)
		{
			data.dValues[i] = 0.0;
		}

		data.iValues[0] = Int32::Parse(this->textBox3->Text); //Lighting - Year
		data.iValues[1] = Int32::Parse(this->textBox4->Text); //Lighting - Month
		data.iValues[2] = Int32::Parse(this->textBox5->Text); //Lighting - Day
		data.iValues[3] = Int32::Parse(this->textBox7->Text); //Lighting - Hour
		data.iValues[4] = Int32::Parse(this->textBox8->Text); //First Guess - Day
		data.iValues[5] = Int32::Parse(this->textBox9->Text); //First Guess - Month
		data.iValues[6] = Int32::Parse(this->textBox10->Text); //First Guess - Year
		data.iValues[7] = Int32::Parse(this->textBox11->Text); //First Guess - Orbits from LOI to landing
		data.iValues[8] = Int32::Parse(this->textBox16->Text); //Target Objectives - Day
		data.iValues[9] = Int32::Parse(this->textBox17->Text); //Target Objectives - Month
		data.iValues[10] = Int32::Parse(this->textBox18->Text); //Target Objectives - Year
		data.iValues[11] = Int32::Parse(this->textBox24->Text); //Target Objectives - Revs LOI-1 to LOI-2
		data.iValues[12] = Int32::Parse(this->textBox25->Text); //Target Objectives - Revs LOI-2 to LLS
		data.iValues[13] = Int32::Parse(this->textBox26->Text); //Target Objectives - Revs LLS to LOPC
		data.iValues[14] = Int32::Parse(this->textBox27->Text); //Target Objectives - Revs LOPC to LLS2
		data.iValues[15] = Int32::Parse(this->textBox28->Text); //Target Objectives - Revs LLS2 to TEI
		data.iValues[16] = Int32::Parse(this->textBox33->Text); //LV Targeting - Day
		data.iValues[17] = this->comboBox1->SelectedIndex; //First Guess - Window
		data.iValues[18] = this->comboBox2->SelectedIndex; //Target Objectives - Window
		data.iValues[19] = this->comboBox3->SelectedIndex; //LV Targeting - Split Launch Window
		data.iValues[20] = this->comboBox4->SelectedIndex; //Init - Parking orbit
		data.iValues[21] = System::Decimal::ToInt32(this->numericUpDown1->Value); //First Guess - Opportunity
		data.iValues[22] = System::Decimal::ToInt32(this->numericUpDown2->Value); //Target Objectives - Opportunity
		data.iValues[23] = this->checkBox1->Checked ? 1 : 0; //LV targeting - free return
		data.iValues[24] = this->checkBox2->Checked ? 1 : 0; //Target objectives - free return
		for (int i = 25; i < 30; i++)
		{
			data.iValues[i] = 0;
		}

		std::string lvtarg = msclr::interop::marshal_as<std::string>(this->textBox32->Text);
		snprintf(data.LVTargetingObjectivesFile, 63, "%s", lvtarg.c_str());

		std::string preset = msclr::interop::marshal_as<std::string>(this->textBox44->Text);
		snprintf(data.PresetTapeFile, 63, "%s", preset.c_str());


		myfile.write((char*)&data, sizeof(struct SaveFileData));
		myfile.close();
	}

	void Form1::LoadFromFile()
	{
		std::string project = msclr::interop::marshal_as<std::string>(this->textBox42->Text);
		std::string projectfile = "Saves/" + project + "-Project.sav";
		std::ifstream myfile(projectfile.c_str());

		if (myfile.is_open() == false) return;

		SaveFileData data;

		myfile.read((char*)&data, sizeof(struct SaveFileData));
		myfile.close();

		this->textBox1->Text = data.dValues[0].ToString();
		this->textBox2->Text = data.dValues[1].ToString();
		this->textBox12->Text = data.dValues[2].ToString();
		this->textBox13->Text = data.dValues[3].ToString();
		this->textBox14->Text = data.dValues[4].ToString();
		this->textBox15->Text = data.dValues[5].ToString();
		this->textBox19->Text = data.dValues[6].ToString();
		this->textBox20->Text = data.dValues[7].ToString();
		this->textBox21->Text = data.dValues[8].ToString();
		this->textBox22->Text = data.dValues[9].ToString();
		this->textBox23->Text = data.dValues[10].ToString();
		this->textBox29->Text = data.dValues[11].ToString();
		this->textBox30->Text = data.dValues[12].ToString();
		this->textBox31->Text = data.dValues[13].ToString();
		this->textBox34->Text = data.dValues[14].ToString();
		this->textBox35->Text = data.dValues[15].ToString();
		this->textBox36->Text = data.dValues[16].ToString();
		this->textBox37->Text = data.dValues[17].ToString();
		this->textBox38->Text = data.dValues[18].ToString();
		this->textBox39->Text = data.dValues[19].ToString();
		this->textBox40->Text = data.dValues[20].ToString();
		this->textBox41->Text = data.dValues[21].ToString();
		this->textBox43->Text = data.dValues[22].ToString();

		this->textBox3->Text = data.iValues[0].ToString();
		this->textBox4->Text = data.iValues[1].ToString();
		this->textBox5->Text = data.iValues[2].ToString();
		this->textBox7->Text = data.iValues[3].ToString();
		this->textBox8->Text = data.iValues[4].ToString();
		this->textBox9->Text = data.iValues[5].ToString();
		this->textBox10->Text = data.iValues[6].ToString();
		this->textBox11->Text = data.iValues[7].ToString();
		this->textBox16->Text = data.iValues[8].ToString();
		this->textBox17->Text = data.iValues[9].ToString();
		this->textBox18->Text = data.iValues[10].ToString();
		this->textBox24->Text = data.iValues[11].ToString();
		this->textBox25->Text = data.iValues[12].ToString();
		this->textBox26->Text = data.iValues[13].ToString();
		this->textBox27->Text = data.iValues[14].ToString();
		this->textBox28->Text = data.iValues[15].ToString();
		this->textBox33->Text = data.iValues[16].ToString();
		this->comboBox1->SelectedIndex = data.iValues[17];
		this->comboBox2->SelectedIndex = data.iValues[18];
		this->comboBox3->SelectedIndex = data.iValues[19];
		this->comboBox4->SelectedIndex = data.iValues[20];
		this->numericUpDown1->Value = data.iValues[21];
		this->numericUpDown2->Value = data.iValues[22];
		this->checkBox1->Checked = data.iValues[23] == 1 ? true : false;
		this->checkBox2->Checked = data.iValues[24] == 1 ? true : false;

		this->textBox32->Text = gcnew String(data.LVTargetingObjectivesFile);
		this->textBox44->Text = gcnew String(data.PresetTapeFile);

	}
}