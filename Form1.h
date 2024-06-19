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

#include "ApolloTrajectoryDesignProgram.h"
#include "QuickResponseTargetingProgram.h"

namespace CppCLRWinFormsProject {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for Form1
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			atdp = new ApolloTrajectoryDesignProgram;
			qrtp = new QuickResponseTargetingProgram;

			if (this->comboBox1->SelectedIndex == -1)
			{
				this->comboBox1->SelectedIndex = this->comboBox1->FindStringExact("Pacific");
			}
			if (this->comboBox2->SelectedIndex == -1)
			{
				this->comboBox2->SelectedIndex = this->comboBox2->FindStringExact("Pacific");
			}
			if (this->comboBox3->SelectedIndex == -1)
			{
				this->comboBox3->SelectedIndex = this->comboBox3->FindStringExact("Optimum 1st Opp");
			}
			if (this->comboBox4->SelectedIndex == -1)
			{
				this->comboBox4->SelectedIndex = this->comboBox4->FindStringExact("100 NM");
			}
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
			delete atdp;
			delete qrtp;
		}

		void CalculateElevationAngle();
		void CalculateTLIFirstGuess();
		void ConvergeOptimizedMission();
		void ExportDataSets();
		void UpdateDisplayedLVTargeting();
		void LoadLVTargetingObjectives();
		void RunQRTPCases();
		void SaveToFile();
		void LoadFromFile();
		PerformanceData ReadPerformanceData();
		void GenerateFinalLaunchData();

	private: ApolloTrajectoryDesignProgram *atdp;
	private: QuickResponseTargetingProgram* qrtp;

	private: System::Windows::Forms::TabControl^ tabControl1;
	protected:
	private: System::Windows::Forms::TabPage^ tabPage1;
	private: System::Windows::Forms::TabPage^ tabPage2;
	private: System::Windows::Forms::Label^ label8;
	private: System::Windows::Forms::Label^ label7;
	private: System::Windows::Forms::TextBox^ textBox7;
	private: System::Windows::Forms::Label^ label6;
	private: System::Windows::Forms::TextBox^ textBox6;
	private: System::Windows::Forms::TextBox^ textBox5;
	private: System::Windows::Forms::TextBox^ textBox4;
	private: System::Windows::Forms::TextBox^ textBox3;
	private: System::Windows::Forms::Label^ label5;
	private: System::Windows::Forms::Label^ label4;
	private: System::Windows::Forms::Label^ label3;
	private: System::Windows::Forms::TextBox^ textBox2;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::TextBox^ textBox1;
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::Label^ label16;


	private: System::Windows::Forms::Label^ label13;
	private: System::Windows::Forms::TextBox^ textBox8;
	private: System::Windows::Forms::TextBox^ textBox9;
	private: System::Windows::Forms::TextBox^ textBox10;
	private: System::Windows::Forms::Label^ label10;
	private: System::Windows::Forms::Label^ label11;
	private: System::Windows::Forms::Label^ label12;
	private: System::Windows::Forms::Label^ label9;
	private: System::Windows::Forms::TextBox^ textBox13;


	private: System::Windows::Forms::Button^ button2;
	private: System::Windows::Forms::RichTextBox^ richTextBox1;
	private: System::Windows::Forms::TextBox^ textBox11;
	private: System::Windows::Forms::Label^ label15;
	private: System::Windows::Forms::Label^ label18;
	private: System::Windows::Forms::TextBox^ textBox12;
	private: System::Windows::Forms::Label^ label14;
	private: System::Windows::Forms::Label^ label17;
	private: System::Windows::Forms::TextBox^ textBox14;
	private: System::Windows::Forms::TabPage^ tabPage3;
	private: System::Windows::Forms::Button^ button3;
	private: System::Windows::Forms::TextBox^ textBox15;
	private: System::Windows::Forms::Label^ label19;
	private: System::Windows::Forms::Label^ label20;
	private: System::Windows::Forms::TextBox^ textBox16;
	private: System::Windows::Forms::TextBox^ textBox17;
	private: System::Windows::Forms::TextBox^ textBox18;
	private: System::Windows::Forms::Label^ label21;
	private: System::Windows::Forms::Label^ label22;
	private: System::Windows::Forms::Label^ label23;
	private: System::Windows::Forms::Label^ label24;
	private: System::Windows::Forms::RichTextBox^ richTextBox2;
	private: System::Windows::Forms::TextBox^ textBox20;
	private: System::Windows::Forms::TextBox^ textBox19;
	private: System::Windows::Forms::Label^ label27;
	private: System::Windows::Forms::Label^ label26;
	private: System::Windows::Forms::Label^ label25;
	private: System::Windows::Forms::TextBox^ textBox21;
	private: System::Windows::Forms::Label^ label28;
	private: System::Windows::Forms::TextBox^ textBox23;
	private: System::Windows::Forms::TextBox^ textBox22;
	private: System::Windows::Forms::Label^ label31;
	private: System::Windows::Forms::Label^ label30;

	private: System::Windows::Forms::ComboBox^ comboBox1;
	private: System::Windows::Forms::Label^ label32;
	private: System::Windows::Forms::ComboBox^ comboBox2;
	private: System::Windows::Forms::Label^ label33;
	private: System::Windows::Forms::TextBox^ textBox28;
	private: System::Windows::Forms::Label^ label39;
	private: System::Windows::Forms::TextBox^ textBox27;
	private: System::Windows::Forms::Label^ label38;
	private: System::Windows::Forms::TextBox^ textBox26;
	private: System::Windows::Forms::Label^ label37;
	private: System::Windows::Forms::TextBox^ textBox25;
	private: System::Windows::Forms::Label^ label36;
	private: System::Windows::Forms::TextBox^ textBox24;
	private: System::Windows::Forms::Label^ label35;
	private: System::Windows::Forms::Label^ label34;
private: System::Windows::Forms::TextBox^ textBox29;
private: System::Windows::Forms::Label^ label41;
private: System::Windows::Forms::Label^ label40;
private: System::Windows::Forms::TextBox^ textBox31;
private: System::Windows::Forms::TextBox^ textBox30;
private: System::Windows::Forms::Label^ label43;
private: System::Windows::Forms::Label^ label42;
private: System::Windows::Forms::NumericUpDown^ numericUpDown1;
private: System::Windows::Forms::Label^ label44;
private: System::Windows::Forms::NumericUpDown^ numericUpDown2;
private: System::Windows::Forms::Label^ label45;
private: System::Windows::Forms::Label^ label29;
private: System::Windows::Forms::Button^ button4;
private: System::Windows::Forms::RichTextBox^ richTextBox3;
private: System::Windows::Forms::Button^ button5;
private: System::Windows::Forms::Button^ button6;
private: System::Windows::Forms::TabPage^ tabPage4;
private: System::Windows::Forms::DataVisualization::Charting::Chart^ chart1;
private: System::Windows::Forms::Label^ label46;
private: System::Windows::Forms::TextBox^ textBox32;
private: System::Windows::Forms::Button^ button7;
private: System::Windows::Forms::TabPage^ tabPage5;
private: System::Windows::Forms::Label^ label47;
private: System::Windows::Forms::RichTextBox^ richTextBox4;
private: System::Windows::Forms::ComboBox^ comboBox3;
private: System::Windows::Forms::Label^ label49;

private: System::Windows::Forms::Label^ label48;
private: System::Windows::Forms::Button^ button8;
private: System::Windows::Forms::TextBox^ textBox33;
private: System::Windows::Forms::Label^ label50;
private: System::Windows::Forms::CheckBox^ checkBox1;
private: System::Windows::Forms::TabPage^ tabPage6;
private: System::Windows::Forms::Label^ label55;
private: System::Windows::Forms::TextBox^ textBox35;
private: System::Windows::Forms::Label^ label54;
private: System::Windows::Forms::Label^ label53;
private: System::Windows::Forms::TextBox^ textBox34;
private: System::Windows::Forms::Label^ label52;
private: System::Windows::Forms::Label^ label51;
private: System::Windows::Forms::Label^ label57;
private: System::Windows::Forms::TextBox^ textBox36;
private: System::Windows::Forms::Label^ label56;
private: System::Windows::Forms::Label^ label59;
private: System::Windows::Forms::TextBox^ textBox37;
private: System::Windows::Forms::Label^ label58;
private: System::Windows::Forms::Label^ label67;
private: System::Windows::Forms::Label^ label66;
private: System::Windows::Forms::TextBox^ textBox41;
private: System::Windows::Forms::TextBox^ textBox40;
private: System::Windows::Forms::Label^ label65;
private: System::Windows::Forms::Label^ label64;
private: System::Windows::Forms::Label^ label63;
private: System::Windows::Forms::Label^ label62;
private: System::Windows::Forms::TextBox^ textBox39;
private: System::Windows::Forms::TextBox^ textBox38;
private: System::Windows::Forms::Label^ label61;
private: System::Windows::Forms::Label^ label60;
private: System::Windows::Forms::ComboBox^ comboBox4;
private: System::Windows::Forms::Label^ label68;
private: System::Windows::Forms::RichTextBox^ richTextBox5;
private: System::Windows::Forms::TabPage^ tabPage7;
private: System::Windows::Forms::GroupBox^ groupBox1;
private: System::Windows::Forms::TextBox^ textBox42;
private: System::Windows::Forms::Button^ button10;
private: System::Windows::Forms::Button^ button9;
private: System::Windows::Forms::Label^ label69;
private: System::Windows::Forms::CheckBox^ checkBox2;
private: System::Windows::Forms::Label^ label70;
private: System::Windows::Forms::Label^ label72;
private: System::Windows::Forms::TextBox^ textBox43;
private: System::Windows::Forms::Label^ label71;
private: System::Windows::Forms::RichTextBox^ richTextBox6;
private: System::Windows::Forms::Button^ button11;
private: System::Windows::Forms::TextBox^ textBox44;
private: System::Windows::Forms::Label^ label73;
private: System::Windows::Forms::Label^ label76;
private: System::Windows::Forms::Label^ label75;
private: System::Windows::Forms::Label^ label74;
private: System::Windows::Forms::Label^ label80;
private: System::Windows::Forms::Label^ label79;
private: System::Windows::Forms::Label^ label78;
private: System::Windows::Forms::Label^ label77;
private: System::Windows::Forms::DataVisualization::Charting::Chart^ chart2;
private: System::Windows::Forms::Label^ label81;
private: System::Windows::Forms::Label^ label82;
private: System::Windows::Forms::TextBox^ textBox45;

private: System::Windows::Forms::Label^ label83;
private: System::Windows::Forms::Label^ label84;
private: System::Windows::Forms::Label^ label85;
private: System::Windows::Forms::TextBox^ textBox46;


	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::Windows::Forms::DataVisualization::Charting::ChartArea^ chartArea3 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^ legend3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^ series3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^ chartArea4 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^ legend4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^ series4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
			this->tabPage6 = (gcnew System::Windows::Forms::TabPage());
			this->label72 = (gcnew System::Windows::Forms::Label());
			this->textBox43 = (gcnew System::Windows::Forms::TextBox());
			this->label71 = (gcnew System::Windows::Forms::Label());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label69 = (gcnew System::Windows::Forms::Label());
			this->textBox42 = (gcnew System::Windows::Forms::TextBox());
			this->button10 = (gcnew System::Windows::Forms::Button());
			this->button9 = (gcnew System::Windows::Forms::Button());
			this->comboBox4 = (gcnew System::Windows::Forms::ComboBox());
			this->label68 = (gcnew System::Windows::Forms::Label());
			this->label67 = (gcnew System::Windows::Forms::Label());
			this->label66 = (gcnew System::Windows::Forms::Label());
			this->textBox41 = (gcnew System::Windows::Forms::TextBox());
			this->textBox40 = (gcnew System::Windows::Forms::TextBox());
			this->label65 = (gcnew System::Windows::Forms::Label());
			this->label64 = (gcnew System::Windows::Forms::Label());
			this->label63 = (gcnew System::Windows::Forms::Label());
			this->label62 = (gcnew System::Windows::Forms::Label());
			this->textBox39 = (gcnew System::Windows::Forms::TextBox());
			this->textBox38 = (gcnew System::Windows::Forms::TextBox());
			this->label61 = (gcnew System::Windows::Forms::Label());
			this->label60 = (gcnew System::Windows::Forms::Label());
			this->label59 = (gcnew System::Windows::Forms::Label());
			this->textBox37 = (gcnew System::Windows::Forms::TextBox());
			this->label58 = (gcnew System::Windows::Forms::Label());
			this->label57 = (gcnew System::Windows::Forms::Label());
			this->textBox36 = (gcnew System::Windows::Forms::TextBox());
			this->label56 = (gcnew System::Windows::Forms::Label());
			this->label55 = (gcnew System::Windows::Forms::Label());
			this->textBox35 = (gcnew System::Windows::Forms::TextBox());
			this->label54 = (gcnew System::Windows::Forms::Label());
			this->label53 = (gcnew System::Windows::Forms::Label());
			this->textBox34 = (gcnew System::Windows::Forms::TextBox());
			this->label52 = (gcnew System::Windows::Forms::Label());
			this->label51 = (gcnew System::Windows::Forms::Label());
			this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
			this->label84 = (gcnew System::Windows::Forms::Label());
			this->label83 = (gcnew System::Windows::Forms::Label());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->textBox7 = (gcnew System::Windows::Forms::TextBox());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->textBox6 = (gcnew System::Windows::Forms::TextBox());
			this->textBox5 = (gcnew System::Windows::Forms::TextBox());
			this->textBox4 = (gcnew System::Windows::Forms::TextBox());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
			this->numericUpDown1 = (gcnew System::Windows::Forms::NumericUpDown());
			this->label44 = (gcnew System::Windows::Forms::Label());
			this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
			this->label32 = (gcnew System::Windows::Forms::Label());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->textBox12 = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->textBox14 = (gcnew System::Windows::Forms::TextBox());
			this->textBox11 = (gcnew System::Windows::Forms::TextBox());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->richTextBox1 = (gcnew System::Windows::Forms::RichTextBox());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->textBox13 = (gcnew System::Windows::Forms::TextBox());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->textBox8 = (gcnew System::Windows::Forms::TextBox());
			this->textBox9 = (gcnew System::Windows::Forms::TextBox());
			this->textBox10 = (gcnew System::Windows::Forms::TextBox());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->tabPage3 = (gcnew System::Windows::Forms::TabPage());
			this->label82 = (gcnew System::Windows::Forms::Label());
			this->textBox45 = (gcnew System::Windows::Forms::TextBox());
			this->label81 = (gcnew System::Windows::Forms::Label());
			this->label80 = (gcnew System::Windows::Forms::Label());
			this->label79 = (gcnew System::Windows::Forms::Label());
			this->label78 = (gcnew System::Windows::Forms::Label());
			this->label77 = (gcnew System::Windows::Forms::Label());
			this->label76 = (gcnew System::Windows::Forms::Label());
			this->label75 = (gcnew System::Windows::Forms::Label());
			this->label74 = (gcnew System::Windows::Forms::Label());
			this->checkBox2 = (gcnew System::Windows::Forms::CheckBox());
			this->label70 = (gcnew System::Windows::Forms::Label());
			this->button6 = (gcnew System::Windows::Forms::Button());
			this->richTextBox3 = (gcnew System::Windows::Forms::RichTextBox());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->numericUpDown2 = (gcnew System::Windows::Forms::NumericUpDown());
			this->label45 = (gcnew System::Windows::Forms::Label());
			this->textBox31 = (gcnew System::Windows::Forms::TextBox());
			this->textBox30 = (gcnew System::Windows::Forms::TextBox());
			this->label43 = (gcnew System::Windows::Forms::Label());
			this->label42 = (gcnew System::Windows::Forms::Label());
			this->textBox29 = (gcnew System::Windows::Forms::TextBox());
			this->label41 = (gcnew System::Windows::Forms::Label());
			this->label40 = (gcnew System::Windows::Forms::Label());
			this->textBox28 = (gcnew System::Windows::Forms::TextBox());
			this->label39 = (gcnew System::Windows::Forms::Label());
			this->textBox27 = (gcnew System::Windows::Forms::TextBox());
			this->label38 = (gcnew System::Windows::Forms::Label());
			this->textBox26 = (gcnew System::Windows::Forms::TextBox());
			this->label37 = (gcnew System::Windows::Forms::Label());
			this->textBox25 = (gcnew System::Windows::Forms::TextBox());
			this->label36 = (gcnew System::Windows::Forms::Label());
			this->textBox24 = (gcnew System::Windows::Forms::TextBox());
			this->label35 = (gcnew System::Windows::Forms::Label());
			this->label34 = (gcnew System::Windows::Forms::Label());
			this->comboBox2 = (gcnew System::Windows::Forms::ComboBox());
			this->label33 = (gcnew System::Windows::Forms::Label());
			this->textBox23 = (gcnew System::Windows::Forms::TextBox());
			this->textBox22 = (gcnew System::Windows::Forms::TextBox());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->textBox21 = (gcnew System::Windows::Forms::TextBox());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->textBox20 = (gcnew System::Windows::Forms::TextBox());
			this->textBox19 = (gcnew System::Windows::Forms::TextBox());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->richTextBox2 = (gcnew System::Windows::Forms::RichTextBox());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->textBox15 = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->textBox16 = (gcnew System::Windows::Forms::TextBox());
			this->textBox17 = (gcnew System::Windows::Forms::TextBox());
			this->textBox18 = (gcnew System::Windows::Forms::TextBox());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->tabPage4 = (gcnew System::Windows::Forms::TabPage());
			this->chart2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->button7 = (gcnew System::Windows::Forms::Button());
			this->label46 = (gcnew System::Windows::Forms::Label());
			this->textBox32 = (gcnew System::Windows::Forms::TextBox());
			this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->tabPage5 = (gcnew System::Windows::Forms::TabPage());
			this->richTextBox5 = (gcnew System::Windows::Forms::RichTextBox());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->textBox33 = (gcnew System::Windows::Forms::TextBox());
			this->label50 = (gcnew System::Windows::Forms::Label());
			this->button8 = (gcnew System::Windows::Forms::Button());
			this->comboBox3 = (gcnew System::Windows::Forms::ComboBox());
			this->label49 = (gcnew System::Windows::Forms::Label());
			this->label48 = (gcnew System::Windows::Forms::Label());
			this->richTextBox4 = (gcnew System::Windows::Forms::RichTextBox());
			this->label47 = (gcnew System::Windows::Forms::Label());
			this->tabPage7 = (gcnew System::Windows::Forms::TabPage());
			this->button11 = (gcnew System::Windows::Forms::Button());
			this->textBox44 = (gcnew System::Windows::Forms::TextBox());
			this->label73 = (gcnew System::Windows::Forms::Label());
			this->richTextBox6 = (gcnew System::Windows::Forms::RichTextBox());
			this->label85 = (gcnew System::Windows::Forms::Label());
			this->textBox46 = (gcnew System::Windows::Forms::TextBox());
			this->tabControl1->SuspendLayout();
			this->tabPage6->SuspendLayout();
			this->groupBox1->SuspendLayout();
			this->tabPage1->SuspendLayout();
			this->tabPage2->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->BeginInit();
			this->tabPage3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown2))->BeginInit();
			this->tabPage4->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
			this->tabPage5->SuspendLayout();
			this->tabPage7->SuspendLayout();
			this->SuspendLayout();
			// 
			// tabControl1
			// 
			this->tabControl1->Controls->Add(this->tabPage6);
			this->tabControl1->Controls->Add(this->tabPage1);
			this->tabControl1->Controls->Add(this->tabPage2);
			this->tabControl1->Controls->Add(this->tabPage3);
			this->tabControl1->Controls->Add(this->tabPage4);
			this->tabControl1->Controls->Add(this->tabPage5);
			this->tabControl1->Controls->Add(this->tabPage7);
			this->tabControl1->Location = System::Drawing::Point(-1, 1);
			this->tabControl1->Name = L"tabControl1";
			this->tabControl1->SelectedIndex = 0;
			this->tabControl1->Size = System::Drawing::Size(1140, 651);
			this->tabControl1->TabIndex = 0;
			// 
			// tabPage6
			// 
			this->tabPage6->BackColor = System::Drawing::SystemColors::Control;
			this->tabPage6->Controls->Add(this->textBox46);
			this->tabPage6->Controls->Add(this->label85);
			this->tabPage6->Controls->Add(this->label72);
			this->tabPage6->Controls->Add(this->textBox43);
			this->tabPage6->Controls->Add(this->label71);
			this->tabPage6->Controls->Add(this->groupBox1);
			this->tabPage6->Controls->Add(this->comboBox4);
			this->tabPage6->Controls->Add(this->label68);
			this->tabPage6->Controls->Add(this->label67);
			this->tabPage6->Controls->Add(this->label66);
			this->tabPage6->Controls->Add(this->textBox41);
			this->tabPage6->Controls->Add(this->textBox40);
			this->tabPage6->Controls->Add(this->label65);
			this->tabPage6->Controls->Add(this->label64);
			this->tabPage6->Controls->Add(this->label63);
			this->tabPage6->Controls->Add(this->label62);
			this->tabPage6->Controls->Add(this->textBox39);
			this->tabPage6->Controls->Add(this->textBox38);
			this->tabPage6->Controls->Add(this->label61);
			this->tabPage6->Controls->Add(this->label60);
			this->tabPage6->Controls->Add(this->label59);
			this->tabPage6->Controls->Add(this->textBox37);
			this->tabPage6->Controls->Add(this->label58);
			this->tabPage6->Controls->Add(this->label57);
			this->tabPage6->Controls->Add(this->textBox36);
			this->tabPage6->Controls->Add(this->label56);
			this->tabPage6->Controls->Add(this->label55);
			this->tabPage6->Controls->Add(this->textBox35);
			this->tabPage6->Controls->Add(this->label54);
			this->tabPage6->Controls->Add(this->label53);
			this->tabPage6->Controls->Add(this->textBox34);
			this->tabPage6->Controls->Add(this->label52);
			this->tabPage6->Controls->Add(this->label51);
			this->tabPage6->Location = System::Drawing::Point(4, 22);
			this->tabPage6->Name = L"tabPage6";
			this->tabPage6->Padding = System::Windows::Forms::Padding(3);
			this->tabPage6->Size = System::Drawing::Size(1132, 625);
			this->tabPage6->TabIndex = 5;
			this->tabPage6->Text = L"Init";
			// 
			// label72
			// 
			this->label72->AutoSize = true;
			this->label72->Location = System::Drawing::Point(988, 296);
			this->label72->Name = L"label72";
			this->label72->Size = System::Drawing::Size(20, 13);
			this->label72->TabIndex = 30;
			this->label72->Text = L"lbs";
			// 
			// textBox43
			// 
			this->textBox43->Location = System::Drawing::Point(870, 293);
			this->textBox43->Name = L"textBox43";
			this->textBox43->Size = System::Drawing::Size(100, 20);
			this->textBox43->TabIndex = 29;
			this->textBox43->Text = L"40202";
			// 
			// label71
			// 
			this->label71->AutoSize = true;
			this->label71->Location = System::Drawing::Point(742, 296);
			this->label71->Name = L"label71";
			this->label71->Size = System::Drawing::Size(78, 13);
			this->label71->TabIndex = 28;
			this->label71->Text = L"SPS Propellant";
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label69);
			this->groupBox1->Controls->Add(this->textBox42);
			this->groupBox1->Controls->Add(this->button10);
			this->groupBox1->Controls->Add(this->button9);
			this->groupBox1->Location = System::Drawing::Point(79, 73);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(326, 285);
			this->groupBox1->TabIndex = 27;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Project";
			// 
			// label69
			// 
			this->label69->AutoSize = true;
			this->label69->Location = System::Drawing::Point(44, 144);
			this->label69->Name = L"label69";
			this->label69->Size = System::Drawing::Size(42, 13);
			this->label69->TabIndex = 3;
			this->label69->Text = L"Mission";
			// 
			// textBox42
			// 
			this->textBox42->Location = System::Drawing::Point(156, 141);
			this->textBox42->Name = L"textBox42";
			this->textBox42->Size = System::Drawing::Size(100, 20);
			this->textBox42->TabIndex = 2;
			this->textBox42->Text = L"Default";
			// 
			// button10
			// 
			this->button10->Location = System::Drawing::Point(204, 232);
			this->button10->Name = L"button10";
			this->button10->Size = System::Drawing::Size(75, 23);
			this->button10->TabIndex = 1;
			this->button10->Text = L"Load";
			this->button10->UseVisualStyleBackColor = true;
			this->button10->Click += gcnew System::EventHandler(this, &Form1::button10_Click);
			// 
			// button9
			// 
			this->button9->Location = System::Drawing::Point(47, 232);
			this->button9->Name = L"button9";
			this->button9->Size = System::Drawing::Size(75, 23);
			this->button9->TabIndex = 0;
			this->button9->Text = L"Save";
			this->button9->UseVisualStyleBackColor = true;
			this->button9->Click += gcnew System::EventHandler(this, &Form1::button9_Click);
			// 
			// comboBox4
			// 
			this->comboBox4->FormattingEnabled = true;
			this->comboBox4->Items->AddRange(gcnew cli::array< System::Object^  >(2) { L"90 NM", L"100 NM" });
			this->comboBox4->Location = System::Drawing::Point(870, 86);
			this->comboBox4->Name = L"comboBox4";
			this->comboBox4->Size = System::Drawing::Size(121, 21);
			this->comboBox4->TabIndex = 26;
			// 
			// label68
			// 
			this->label68->AutoSize = true;
			this->label68->Location = System::Drawing::Point(742, 89);
			this->label68->Name = L"label68";
			this->label68->Size = System::Drawing::Size(68, 13);
			this->label68->TabIndex = 25;
			this->label68->Text = L"Parking Orbit";
			// 
			// label67
			// 
			this->label67->AutoSize = true;
			this->label67->Location = System::Drawing::Point(988, 550);
			this->label67->Name = L"label67";
			this->label67->Size = System::Drawing::Size(42, 13);
			this->label67->TabIndex = 24;
			this->label67->Text = L"lbs/sec";
			// 
			// label66
			// 
			this->label66->AutoSize = true;
			this->label66->Location = System::Drawing::Point(988, 520);
			this->label66->Name = L"label66";
			this->label66->Size = System::Drawing::Size(42, 13);
			this->label66->TabIndex = 23;
			this->label66->Text = L"lbs/sec";
			// 
			// textBox41
			// 
			this->textBox41->Location = System::Drawing::Point(870, 547);
			this->textBox41->Name = L"textBox41";
			this->textBox41->Size = System::Drawing::Size(100, 20);
			this->textBox41->TabIndex = 22;
			this->textBox41->Text = L"472.121";
			// 
			// textBox40
			// 
			this->textBox40->Location = System::Drawing::Point(870, 517);
			this->textBox40->Name = L"textBox40";
			this->textBox40->Size = System::Drawing::Size(100, 20);
			this->textBox40->TabIndex = 21;
			this->textBox40->Text = L"412.167";
			// 
			// label65
			// 
			this->label65->AutoSize = true;
			this->label65->Location = System::Drawing::Point(742, 550);
			this->label65->Name = L"label65";
			this->label65->Size = System::Drawing::Size(116, 13);
			this->label65->TabIndex = 20;
			this->label65->Text = L"S-IVB MFR (post-MRS)";
			// 
			// label64
			// 
			this->label64->AutoSize = true;
			this->label64->Location = System::Drawing::Point(742, 520);
			this->label64->Name = L"label64";
			this->label64->Size = System::Drawing::Size(111, 13);
			this->label64->TabIndex = 19;
			this->label64->Text = L"S-IVB MFR (pre-MRS)";
			// 
			// label63
			// 
			this->label63->AutoSize = true;
			this->label63->Location = System::Drawing::Point(988, 490);
			this->label63->Name = L"label63";
			this->label63->Size = System::Drawing::Size(18, 13);
			this->label63->TabIndex = 18;
			this->label63->Text = L"lbf";
			// 
			// label62
			// 
			this->label62->AutoSize = true;
			this->label62->Location = System::Drawing::Point(988, 460);
			this->label62->Name = L"label62";
			this->label62->Size = System::Drawing::Size(18, 13);
			this->label62->TabIndex = 17;
			this->label62->Text = L"lbf";
			// 
			// textBox39
			// 
			this->textBox39->Location = System::Drawing::Point(870, 487);
			this->textBox39->Name = L"textBox39";
			this->textBox39->Size = System::Drawing::Size(100, 20);
			this->textBox39->TabIndex = 16;
			this->textBox39->Text = L"202097";
			// 
			// textBox38
			// 
			this->textBox38->Location = System::Drawing::Point(870, 457);
			this->textBox38->Name = L"textBox38";
			this->textBox38->Size = System::Drawing::Size(100, 20);
			this->textBox38->TabIndex = 15;
			this->textBox38->Text = L"178147";
			// 
			// label61
			// 
			this->label61->AutoSize = true;
			this->label61->Location = System::Drawing::Point(742, 490);
			this->label61->Name = L"label61";
			this->label61->Size = System::Drawing::Size(123, 13);
			this->label61->TabIndex = 14;
			this->label61->Text = L"S-IVB Thrust (post MRS)";
			// 
			// label60
			// 
			this->label60->AutoSize = true;
			this->label60->Location = System::Drawing::Point(742, 460);
			this->label60->Name = L"label60";
			this->label60->Size = System::Drawing::Size(118, 13);
			this->label60->TabIndex = 13;
			this->label60->Text = L"S-IVB Thrust (pre MRS)";
			// 
			// label59
			// 
			this->label59->AutoSize = true;
			this->label59->Location = System::Drawing::Point(988, 430);
			this->label59->Name = L"label59";
			this->label59->Size = System::Drawing::Size(24, 13);
			this->label59->TabIndex = 12;
			this->label59->Text = L"sec";
			// 
			// textBox37
			// 
			this->textBox37->Location = System::Drawing::Point(870, 427);
			this->textBox37->Name = L"textBox37";
			this->textBox37->Size = System::Drawing::Size(100, 20);
			this->textBox37->TabIndex = 11;
			this->textBox37->Text = L"50";
			// 
			// label58
			// 
			this->label58->AutoSize = true;
			this->label58->Location = System::Drawing::Point(742, 430);
			this->label58->Name = L"label58";
			this->label58->Size = System::Drawing::Size(115, 13);
			this->label58->TabIndex = 10;
			this->label58->Text = L"Time of MRS (1st Opp)";
			// 
			// label57
			// 
			this->label57->AutoSize = true;
			this->label57->Location = System::Drawing::Point(988, 400);
			this->label57->Name = L"label57";
			this->label57->Size = System::Drawing::Size(20, 13);
			this->label57->TabIndex = 9;
			this->label57->Text = L"lbs";
			// 
			// textBox36
			// 
			this->textBox36->Location = System::Drawing::Point(870, 397);
			this->textBox36->Name = L"textBox36";
			this->textBox36->Size = System::Drawing::Size(100, 20);
			this->textBox36->TabIndex = 8;
			this->textBox36->Text = L"201624";
			// 
			// label56
			// 
			this->label56->AutoSize = true;
			this->label56->Location = System::Drawing::Point(742, 400);
			this->label56->Name = L"label56";
			this->label56->Size = System::Drawing::Size(98, 13);
			this->label56->TabIndex = 7;
			this->label56->Text = L"S-IVB Weight (EOI)";
			// 
			// label55
			// 
			this->label55->AutoSize = true;
			this->label55->Location = System::Drawing::Point(988, 258);
			this->label55->Name = L"label55";
			this->label55->Size = System::Drawing::Size(20, 13);
			this->label55->TabIndex = 6;
			this->label55->Text = L"lbs";
			// 
			// textBox35
			// 
			this->textBox35->Location = System::Drawing::Point(870, 255);
			this->textBox35->Name = L"textBox35";
			this->textBox35->Size = System::Drawing::Size(100, 20);
			this->textBox35->TabIndex = 5;
			this->textBox35->Text = L"30000";
			// 
			// label54
			// 
			this->label54->AutoSize = true;
			this->label54->Location = System::Drawing::Point(742, 258);
			this->label54->Name = L"label54";
			this->label54->Size = System::Drawing::Size(59, 13);
			this->label54->TabIndex = 4;
			this->label54->Text = L"LM Weight";
			// 
			// label53
			// 
			this->label53->AutoSize = true;
			this->label53->Location = System::Drawing::Point(988, 219);
			this->label53->Name = L"label53";
			this->label53->Size = System::Drawing::Size(20, 13);
			this->label53->TabIndex = 3;
			this->label53->Text = L"lbs";
			// 
			// textBox34
			// 
			this->textBox34->Location = System::Drawing::Point(870, 216);
			this->textBox34->Name = L"textBox34";
			this->textBox34->Size = System::Drawing::Size(100, 20);
			this->textBox34->TabIndex = 2;
			this->textBox34->Text = L"66000";
			// 
			// label52
			// 
			this->label52->AutoSize = true;
			this->label52->Location = System::Drawing::Point(742, 219);
			this->label52->Name = L"label52";
			this->label52->Size = System::Drawing::Size(67, 13);
			this->label52->TabIndex = 1;
			this->label52->Text = L"CSM Weight";
			// 
			// label51
			// 
			this->label51->AutoSize = true;
			this->label51->Location = System::Drawing::Point(742, 178);
			this->label51->Name = L"label51";
			this->label51->Size = System::Drawing::Size(93, 13);
			this->label51->TabIndex = 0;
			this->label51->Text = L"Performance Data";
			// 
			// tabPage1
			// 
			this->tabPage1->Controls->Add(this->label84);
			this->tabPage1->Controls->Add(this->label83);
			this->tabPage1->Controls->Add(this->button1);
			this->tabPage1->Controls->Add(this->label8);
			this->tabPage1->Controls->Add(this->label7);
			this->tabPage1->Controls->Add(this->textBox7);
			this->tabPage1->Controls->Add(this->label6);
			this->tabPage1->Controls->Add(this->textBox6);
			this->tabPage1->Controls->Add(this->textBox5);
			this->tabPage1->Controls->Add(this->textBox4);
			this->tabPage1->Controls->Add(this->textBox3);
			this->tabPage1->Controls->Add(this->label5);
			this->tabPage1->Controls->Add(this->label4);
			this->tabPage1->Controls->Add(this->label3);
			this->tabPage1->Controls->Add(this->textBox2);
			this->tabPage1->Controls->Add(this->label2);
			this->tabPage1->Controls->Add(this->label1);
			this->tabPage1->Controls->Add(this->textBox1);
			this->tabPage1->Location = System::Drawing::Point(4, 22);
			this->tabPage1->Name = L"tabPage1";
			this->tabPage1->Padding = System::Windows::Forms::Padding(3);
			this->tabPage1->Size = System::Drawing::Size(1132, 625);
			this->tabPage1->TabIndex = 0;
			this->tabPage1->Text = L"Lighting";
			// 
			// label84
			// 
			this->label84->AutoSize = true;
			this->label84->Location = System::Drawing::Point(568, 288);
			this->label84->Name = L"label84";
			this->label84->Size = System::Drawing::Size(33, 13);
			this->label84->TabIndex = 34;
			this->label84->Text = L"None";
			// 
			// label83
			// 
			this->label83->AutoSize = true;
			this->label83->Location = System::Drawing::Point(514, 288);
			this->label83->Name = L"label83";
			this->label83->Size = System::Drawing::Size(37, 13);
			this->label83->TabIndex = 32;
			this->label83->Text = L"Status";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(453, 378);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(75, 23);
			this->button1->TabIndex = 31;
			this->button1->Text = L"Calculate";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(637, 254);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(30, 13);
			this->label8->TabIndex = 30;
			this->label8->Text = L"DEG";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(514, 254);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(51, 13);
			this->label7->TabIndex = 29;
			this->label7->Text = L"Elevation";
			// 
			// textBox7
			// 
			this->textBox7->Location = System::Drawing::Point(366, 316);
			this->textBox7->Name = L"textBox7";
			this->textBox7->Size = System::Drawing::Size(100, 20);
			this->textBox7->TabIndex = 28;
			this->textBox7->Text = L"20";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(313, 319);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(30, 13);
			this->label6->TabIndex = 27;
			this->label6->Text = L"Hour";
			// 
			// textBox6
			// 
			this->textBox6->Location = System::Drawing::Point(571, 251);
			this->textBox6->Name = L"textBox6";
			this->textBox6->ReadOnly = true;
			this->textBox6->Size = System::Drawing::Size(49, 20);
			this->textBox6->TabIndex = 26;
			this->textBox6->TabStop = false;
			// 
			// textBox5
			// 
			this->textBox5->Location = System::Drawing::Point(366, 285);
			this->textBox5->Name = L"textBox5";
			this->textBox5->Size = System::Drawing::Size(100, 20);
			this->textBox5->TabIndex = 25;
			this->textBox5->Text = L"20";
			// 
			// textBox4
			// 
			this->textBox4->Location = System::Drawing::Point(366, 258);
			this->textBox4->Name = L"textBox4";
			this->textBox4->Size = System::Drawing::Size(100, 20);
			this->textBox4->TabIndex = 24;
			this->textBox4->Text = L"7";
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(366, 232);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(100, 20);
			this->textBox3->TabIndex = 23;
			this->textBox3->Text = L"1969";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(317, 288);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(26, 13);
			this->label5->TabIndex = 22;
			this->label5->Text = L"Day";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(306, 261);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(37, 13);
			this->label4->TabIndex = 21;
			this->label4->Text = L"Month";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(314, 235);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(29, 13);
			this->label3->TabIndex = 20;
			this->label3->Text = L"Year";
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(366, 186);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(100, 20);
			this->textBox2->TabIndex = 19;
			this->textBox2->Text = L"23.7169";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(289, 189);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(54, 13);
			this->label2->TabIndex = 18;
			this->label2->Text = L"Longitude";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(298, 163);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(45, 13);
			this->label1->TabIndex = 17;
			this->label1->Text = L"Latitude";
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(366, 160);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(100, 20);
			this->textBox1->TabIndex = 16;
			this->textBox1->Text = L"0.6914";
			// 
			// tabPage2
			// 
			this->tabPage2->Controls->Add(this->numericUpDown1);
			this->tabPage2->Controls->Add(this->label44);
			this->tabPage2->Controls->Add(this->comboBox1);
			this->tabPage2->Controls->Add(this->label32);
			this->tabPage2->Controls->Add(this->label18);
			this->tabPage2->Controls->Add(this->textBox12);
			this->tabPage2->Controls->Add(this->label14);
			this->tabPage2->Controls->Add(this->label17);
			this->tabPage2->Controls->Add(this->textBox14);
			this->tabPage2->Controls->Add(this->textBox11);
			this->tabPage2->Controls->Add(this->label15);
			this->tabPage2->Controls->Add(this->richTextBox1);
			this->tabPage2->Controls->Add(this->button2);
			this->tabPage2->Controls->Add(this->textBox13);
			this->tabPage2->Controls->Add(this->label16);
			this->tabPage2->Controls->Add(this->label13);
			this->tabPage2->Controls->Add(this->textBox8);
			this->tabPage2->Controls->Add(this->textBox9);
			this->tabPage2->Controls->Add(this->textBox10);
			this->tabPage2->Controls->Add(this->label10);
			this->tabPage2->Controls->Add(this->label11);
			this->tabPage2->Controls->Add(this->label12);
			this->tabPage2->Controls->Add(this->label9);
			this->tabPage2->Location = System::Drawing::Point(4, 22);
			this->tabPage2->Name = L"tabPage2";
			this->tabPage2->Padding = System::Windows::Forms::Padding(3);
			this->tabPage2->Size = System::Drawing::Size(1132, 625);
			this->tabPage2->TabIndex = 1;
			this->tabPage2->Text = L"First Guess";
			// 
			// numericUpDown1
			// 
			this->numericUpDown1->Location = System::Drawing::Point(113, 268);
			this->numericUpDown1->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 2, 0, 0, 0 });
			this->numericUpDown1->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			this->numericUpDown1->Name = L"numericUpDown1";
			this->numericUpDown1->Size = System::Drawing::Size(120, 20);
			this->numericUpDown1->TabIndex = 52;
			this->numericUpDown1->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			// 
			// label44
			// 
			this->label44->AutoSize = true;
			this->label44->Location = System::Drawing::Point(40, 270);
			this->label44->Name = L"label44";
			this->label44->Size = System::Drawing::Size(61, 13);
			this->label44->TabIndex = 51;
			this->label44->Text = L"Opportunity";
			// 
			// comboBox1
			// 
			this->comboBox1->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBox1->FormattingEnabled = true;
			this->comboBox1->Items->AddRange(gcnew cli::array< System::Object^  >(2) { L"Pacific", L"Atlantic" });
			this->comboBox1->Location = System::Drawing::Point(112, 235);
			this->comboBox1->Name = L"comboBox1";
			this->comboBox1->Size = System::Drawing::Size(121, 21);
			this->comboBox1->TabIndex = 50;
			// 
			// label32
			// 
			this->label32->AutoSize = true;
			this->label32->Location = System::Drawing::Point(40, 238);
			this->label32->Name = L"label32";
			this->label32->Size = System::Drawing::Size(46, 13);
			this->label32->TabIndex = 49;
			this->label32->Text = L"Window";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(109, 330);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(59, 13);
			this->label18->TabIndex = 48;
			this->label18->Text = L"Lunar Orbit";
			// 
			// textBox12
			// 
			this->textBox12->Location = System::Drawing::Point(112, 415);
			this->textBox12->Name = L"textBox12";
			this->textBox12->Size = System::Drawing::Size(100, 20);
			this->textBox12->TabIndex = 47;
			this->textBox12->Text = L"23.7169";
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(40, 418);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(54, 13);
			this->label14->TabIndex = 46;
			this->label14->Text = L"Longitude";
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(40, 392);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(45, 13);
			this->label17->TabIndex = 45;
			this->label17->Text = L"Latitude";
			// 
			// textBox14
			// 
			this->textBox14->Location = System::Drawing::Point(112, 389);
			this->textBox14->Name = L"textBox14";
			this->textBox14->Size = System::Drawing::Size(100, 20);
			this->textBox14->TabIndex = 44;
			this->textBox14->Text = L"0.6914";
			// 
			// textBox11
			// 
			this->textBox11->Location = System::Drawing::Point(179, 357);
			this->textBox11->Name = L"textBox11";
			this->textBox11->Size = System::Drawing::Size(100, 20);
			this->textBox11->TabIndex = 43;
			this->textBox11->Text = L"13";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(35, 360);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(126, 13);
			this->label15->TabIndex = 42;
			this->label15->Text = L"Orbits from LOI to landing";
			// 
			// richTextBox1
			// 
			this->richTextBox1->Location = System::Drawing::Point(590, 95);
			this->richTextBox1->Name = L"richTextBox1";
			this->richTextBox1->Size = System::Drawing::Size(246, 170);
			this->richTextBox1->TabIndex = 40;
			this->richTextBox1->Text = L"";
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(99, 496);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(75, 23);
			this->button2->TabIndex = 39;
			this->button2->Text = L"Calculate";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// textBox13
			// 
			this->textBox13->Location = System::Drawing::Point(112, 203);
			this->textBox13->Name = L"textBox13";
			this->textBox13->Size = System::Drawing::Size(100, 20);
			this->textBox13->TabIndex = 38;
			this->textBox13->Text = L"72.0";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(40, 206);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(44, 13);
			this->label16->TabIndex = 35;
			this->label16->Text = L"Azimuth";
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(109, 176);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(43, 13);
			this->label13->TabIndex = 32;
			this->label13->Text = L"Launch";
			// 
			// textBox8
			// 
			this->textBox8->Location = System::Drawing::Point(112, 129);
			this->textBox8->Name = L"textBox8";
			this->textBox8->Size = System::Drawing::Size(100, 20);
			this->textBox8->TabIndex = 31;
			this->textBox8->Text = L"6";
			// 
			// textBox9
			// 
			this->textBox9->Location = System::Drawing::Point(112, 101);
			this->textBox9->Name = L"textBox9";
			this->textBox9->Size = System::Drawing::Size(100, 20);
			this->textBox9->TabIndex = 30;
			this->textBox9->Text = L"2";
			// 
			// textBox10
			// 
			this->textBox10->Location = System::Drawing::Point(112, 73);
			this->textBox10->Name = L"textBox10";
			this->textBox10->Size = System::Drawing::Size(100, 20);
			this->textBox10->TabIndex = 29;
			this->textBox10->Text = L"1973";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(40, 132);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(26, 13);
			this->label10->TabIndex = 28;
			this->label10->Text = L"Day";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(40, 104);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(37, 13);
			this->label11->TabIndex = 27;
			this->label11->Text = L"Month";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(40, 76);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(29, 13);
			this->label12->TabIndex = 26;
			this->label12->Text = L"Year";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(109, 46);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(65, 13);
			this->label9->TabIndex = 0;
			this->label9->Text = L"Launch Day";
			// 
			// tabPage3
			// 
			this->tabPage3->BackColor = System::Drawing::SystemColors::Control;
			this->tabPage3->Controls->Add(this->label82);
			this->tabPage3->Controls->Add(this->textBox45);
			this->tabPage3->Controls->Add(this->label81);
			this->tabPage3->Controls->Add(this->label80);
			this->tabPage3->Controls->Add(this->label79);
			this->tabPage3->Controls->Add(this->label78);
			this->tabPage3->Controls->Add(this->label77);
			this->tabPage3->Controls->Add(this->label76);
			this->tabPage3->Controls->Add(this->label75);
			this->tabPage3->Controls->Add(this->label74);
			this->tabPage3->Controls->Add(this->checkBox2);
			this->tabPage3->Controls->Add(this->label70);
			this->tabPage3->Controls->Add(this->button6);
			this->tabPage3->Controls->Add(this->richTextBox3);
			this->tabPage3->Controls->Add(this->button5);
			this->tabPage3->Controls->Add(this->button4);
			this->tabPage3->Controls->Add(this->label29);
			this->tabPage3->Controls->Add(this->numericUpDown2);
			this->tabPage3->Controls->Add(this->label45);
			this->tabPage3->Controls->Add(this->textBox31);
			this->tabPage3->Controls->Add(this->textBox30);
			this->tabPage3->Controls->Add(this->label43);
			this->tabPage3->Controls->Add(this->label42);
			this->tabPage3->Controls->Add(this->textBox29);
			this->tabPage3->Controls->Add(this->label41);
			this->tabPage3->Controls->Add(this->label40);
			this->tabPage3->Controls->Add(this->textBox28);
			this->tabPage3->Controls->Add(this->label39);
			this->tabPage3->Controls->Add(this->textBox27);
			this->tabPage3->Controls->Add(this->label38);
			this->tabPage3->Controls->Add(this->textBox26);
			this->tabPage3->Controls->Add(this->label37);
			this->tabPage3->Controls->Add(this->textBox25);
			this->tabPage3->Controls->Add(this->label36);
			this->tabPage3->Controls->Add(this->textBox24);
			this->tabPage3->Controls->Add(this->label35);
			this->tabPage3->Controls->Add(this->label34);
			this->tabPage3->Controls->Add(this->comboBox2);
			this->tabPage3->Controls->Add(this->label33);
			this->tabPage3->Controls->Add(this->textBox23);
			this->tabPage3->Controls->Add(this->textBox22);
			this->tabPage3->Controls->Add(this->label31);
			this->tabPage3->Controls->Add(this->label30);
			this->tabPage3->Controls->Add(this->textBox21);
			this->tabPage3->Controls->Add(this->label28);
			this->tabPage3->Controls->Add(this->textBox20);
			this->tabPage3->Controls->Add(this->textBox19);
			this->tabPage3->Controls->Add(this->label27);
			this->tabPage3->Controls->Add(this->label26);
			this->tabPage3->Controls->Add(this->label25);
			this->tabPage3->Controls->Add(this->richTextBox2);
			this->tabPage3->Controls->Add(this->button3);
			this->tabPage3->Controls->Add(this->textBox15);
			this->tabPage3->Controls->Add(this->label19);
			this->tabPage3->Controls->Add(this->label20);
			this->tabPage3->Controls->Add(this->textBox16);
			this->tabPage3->Controls->Add(this->textBox17);
			this->tabPage3->Controls->Add(this->textBox18);
			this->tabPage3->Controls->Add(this->label21);
			this->tabPage3->Controls->Add(this->label22);
			this->tabPage3->Controls->Add(this->label23);
			this->tabPage3->Controls->Add(this->label24);
			this->tabPage3->Location = System::Drawing::Point(4, 22);
			this->tabPage3->Name = L"tabPage3";
			this->tabPage3->Padding = System::Windows::Forms::Padding(3);
			this->tabPage3->Size = System::Drawing::Size(1132, 625);
			this->tabPage3->TabIndex = 2;
			this->tabPage3->Text = L"Mission Planning";
			// 
			// label82
			// 
			this->label82->AutoSize = true;
			this->label82->Location = System::Drawing::Point(371, 100);
			this->label82->Name = L"label82";
			this->label82->Size = System::Drawing::Size(21, 13);
			this->label82->TabIndex = 101;
			this->label82->Text = L"hrs";
			// 
			// textBox45
			// 
			this->textBox45->Location = System::Drawing::Point(323, 97);
			this->textBox45->Name = L"textBox45";
			this->textBox45->Size = System::Drawing::Size(42, 20);
			this->textBox45->TabIndex = 100;
			this->textBox45->Text = L"12";
			// 
			// label81
			// 
			this->label81->AutoSize = true;
			this->label81->Location = System::Drawing::Point(251, 100);
			this->label81->Name = L"label81";
			this->label81->Size = System::Drawing::Size(61, 13);
			this->label81->TabIndex = 99;
			this->label81->Text = L"Estim. Time";
			// 
			// label80
			// 
			this->label80->AutoSize = true;
			this->label80->Location = System::Drawing::Point(556, 548);
			this->label80->Name = L"label80";
			this->label80->Size = System::Drawing::Size(21, 13);
			this->label80->TabIndex = 98;
			this->label80->Text = L"hrs";
			// 
			// label79
			// 
			this->label79->AutoSize = true;
			this->label79->Location = System::Drawing::Point(556, 511);
			this->label79->Name = L"label79";
			this->label79->Size = System::Drawing::Size(25, 13);
			this->label79->TabIndex = 97;
			this->label79->Text = L"deg";
			// 
			// label78
			// 
			this->label78->AutoSize = true;
			this->label78->Location = System::Drawing::Point(556, 474);
			this->label78->Name = L"label78";
			this->label78->Size = System::Drawing::Size(25, 13);
			this->label78->TabIndex = 96;
			this->label78->Text = L"deg";
			// 
			// label77
			// 
			this->label77->AutoSize = true;
			this->label77->Location = System::Drawing::Point(229, 545);
			this->label77->Name = L"label77";
			this->label77->Size = System::Drawing::Size(25, 13);
			this->label77->TabIndex = 95;
			this->label77->Text = L"deg";
			// 
			// label76
			// 
			this->label76->AutoSize = true;
			this->label76->Location = System::Drawing::Point(229, 510);
			this->label76->Name = L"label76";
			this->label76->Size = System::Drawing::Size(24, 13);
			this->label76->TabIndex = 94;
			this->label76->Text = L"NM";
			// 
			// label75
			// 
			this->label75->AutoSize = true;
			this->label75->Location = System::Drawing::Point(229, 482);
			this->label75->Name = L"label75";
			this->label75->Size = System::Drawing::Size(25, 13);
			this->label75->TabIndex = 93;
			this->label75->Text = L"deg";
			// 
			// label74
			// 
			this->label74->AutoSize = true;
			this->label74->Location = System::Drawing::Point(229, 454);
			this->label74->Name = L"label74";
			this->label74->Size = System::Drawing::Size(25, 13);
			this->label74->TabIndex = 92;
			this->label74->Text = L"deg";
			// 
			// checkBox2
			// 
			this->checkBox2->AutoSize = true;
			this->checkBox2->Location = System::Drawing::Point(159, 322);
			this->checkBox2->Name = L"checkBox2";
			this->checkBox2->Size = System::Drawing::Size(15, 14);
			this->checkBox2->TabIndex = 91;
			this->checkBox2->UseVisualStyleBackColor = true;
			this->checkBox2->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBox2_CheckedChanged);
			// 
			// label70
			// 
			this->label70->AutoSize = true;
			this->label70->Location = System::Drawing::Point(31, 322);
			this->label70->Name = L"label70";
			this->label70->Size = System::Drawing::Size(63, 13);
			this->label70->TabIndex = 90;
			this->label70->Text = L"Free Return";
			// 
			// button6
			// 
			this->button6->Location = System::Drawing::Point(418, 595);
			this->button6->Name = L"button6";
			this->button6->Size = System::Drawing::Size(75, 23);
			this->button6->TabIndex = 89;
			this->button6->Text = L"Clear";
			this->button6->UseVisualStyleBackColor = true;
			this->button6->Click += gcnew System::EventHandler(this, &Form1::button6_Click);
			// 
			// richTextBox3
			// 
			this->richTextBox3->Location = System::Drawing::Point(673, 512);
			this->richTextBox3->Name = L"richTextBox3";
			this->richTextBox3->Size = System::Drawing::Size(270, 96);
			this->richTextBox3->TabIndex = 88;
			this->richTextBox3->Text = L"";
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(306, 595);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(75, 23);
			this->button5->TabIndex = 87;
			this->button5->Text = L"Store";
			this->button5->UseVisualStyleBackColor = true;
			this->button5->Click += gcnew System::EventHandler(this, &Form1::button5_Click);
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(1029, 540);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(75, 23);
			this->button4->TabIndex = 86;
			this->button4->Text = L"Export";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &Form1::button4_Click);
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->Location = System::Drawing::Point(109, 261);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(57, 13);
			this->label29->TabIndex = 85;
			this->label29->Text = L"Translunar";
			// 
			// numericUpDown2
			// 
			this->numericUpDown2->Location = System::Drawing::Point(113, 224);
			this->numericUpDown2->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 2, 0, 0, 0 });
			this->numericUpDown2->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			this->numericUpDown2->Name = L"numericUpDown2";
			this->numericUpDown2->Size = System::Drawing::Size(120, 20);
			this->numericUpDown2->TabIndex = 84;
			this->numericUpDown2->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			// 
			// label45
			// 
			this->label45->AutoSize = true;
			this->label45->Location = System::Drawing::Point(40, 226);
			this->label45->Name = L"label45";
			this->label45->Size = System::Drawing::Size(61, 13);
			this->label45->TabIndex = 83;
			this->label45->Text = L"Opportunity";
			// 
			// textBox31
			// 
			this->textBox31->Location = System::Drawing::Point(441, 545);
			this->textBox31->Name = L"textBox31";
			this->textBox31->Size = System::Drawing::Size(100, 20);
			this->textBox31->TabIndex = 82;
			this->textBox31->Text = L"65.0";
			// 
			// textBox30
			// 
			this->textBox30->Location = System::Drawing::Point(441, 509);
			this->textBox30->Name = L"textBox30";
			this->textBox30->Size = System::Drawing::Size(100, 20);
			this->textBox30->TabIndex = 81;
			this->textBox30->Text = L"-170.0";
			// 
			// label43
			// 
			this->label43->AutoSize = true;
			this->label43->Location = System::Drawing::Point(353, 548);
			this->label43->Name = L"label43";
			this->label43->Size = System::Drawing::Size(81, 13);
			this->label43->TabIndex = 80;
			this->label43->Text = L"Estim. TEC time";
			// 
			// label42
			// 
			this->label42->AutoSize = true;
			this->label42->Location = System::Drawing::Point(369, 512);
			this->label42->Name = L"label42";
			this->label42->Size = System::Drawing::Size(54, 13);
			this->label42->TabIndex = 79;
			this->label42->Text = L"Longitude";
			// 
			// textBox29
			// 
			this->textBox29->Location = System::Drawing::Point(441, 471);
			this->textBox29->Name = L"textBox29";
			this->textBox29->Size = System::Drawing::Size(100, 20);
			this->textBox29->TabIndex = 78;
			this->textBox29->Text = L"40.0";
			// 
			// label41
			// 
			this->label41->AutoSize = true;
			this->label41->Location = System::Drawing::Point(353, 474);
			this->label41->Name = L"label41";
			this->label41->Size = System::Drawing::Size(78, 13);
			this->label41->TabIndex = 77;
			this->label41->Text = L"Max Inclination";
			// 
			// label40
			// 
			this->label40->AutoSize = true;
			this->label40->Location = System::Drawing::Point(438, 445);
			this->label40->Name = L"label40";
			this->label40->Size = System::Drawing::Size(65, 13);
			this->label40->TabIndex = 76;
			this->label40->Text = L"Splashdown";
			// 
			// textBox28
			// 
			this->textBox28->Location = System::Drawing::Point(492, 381);
			this->textBox28->Name = L"textBox28";
			this->textBox28->Size = System::Drawing::Size(49, 20);
			this->textBox28->TabIndex = 75;
			this->textBox28->Text = L"6";
			// 
			// label39
			// 
			this->label39->AutoSize = true;
			this->label39->Location = System::Drawing::Point(394, 384);
			this->label39->Name = L"label39";
			this->label39->Size = System::Drawing::Size(64, 13);
			this->label39->TabIndex = 74;
			this->label39->Text = L"LLS2 to TEI";
			// 
			// textBox27
			// 
			this->textBox27->Location = System::Drawing::Point(492, 347);
			this->textBox27->Name = L"textBox27";
			this->textBox27->Size = System::Drawing::Size(49, 20);
			this->textBox27->TabIndex = 73;
			this->textBox27->Text = L"8";
			// 
			// label38
			// 
			this->label38->AutoSize = true;
			this->label38->Location = System::Drawing::Point(394, 350);
			this->label38->Name = L"label38";
			this->label38->Size = System::Drawing::Size(75, 13);
			this->label38->TabIndex = 72;
			this->label38->Text = L"LOPC to LLS2";
			// 
			// textBox26
			// 
			this->textBox26->Location = System::Drawing::Point(492, 315);
			this->textBox26->Name = L"textBox26";
			this->textBox26->Size = System::Drawing::Size(49, 20);
			this->textBox26->TabIndex = 71;
			this->textBox26->Text = L"3";
			// 
			// label37
			// 
			this->label37->AutoSize = true;
			this->label37->Location = System::Drawing::Point(394, 318);
			this->label37->Name = L"label37";
			this->label37->Size = System::Drawing::Size(69, 13);
			this->label37->TabIndex = 70;
			this->label37->Text = L"LLS to LOPC";
			// 
			// textBox25
			// 
			this->textBox25->Location = System::Drawing::Point(492, 284);
			this->textBox25->Name = L"textBox25";
			this->textBox25->Size = System::Drawing::Size(49, 20);
			this->textBox25->TabIndex = 69;
			this->textBox25->Text = L"11";
			// 
			// label36
			// 
			this->label36->AutoSize = true;
			this->label36->Location = System::Drawing::Point(394, 287);
			this->label36->Name = L"label36";
			this->label36->Size = System::Drawing::Size(67, 13);
			this->label36->TabIndex = 68;
			this->label36->Text = L"LOI-2 to LLS";
			// 
			// textBox24
			// 
			this->textBox24->Location = System::Drawing::Point(492, 254);
			this->textBox24->Name = L"textBox24";
			this->textBox24->Size = System::Drawing::Size(49, 20);
			this->textBox24->TabIndex = 67;
			this->textBox24->Text = L"2";
			// 
			// label35
			// 
			this->label35->AutoSize = true;
			this->label35->Location = System::Drawing::Point(394, 257);
			this->label35->Name = L"label35";
			this->label35->Size = System::Drawing::Size(74, 13);
			this->label35->TabIndex = 66;
			this->label35->Text = L"LOI-1 to LOI-2";
			// 
			// label34
			// 
			this->label34->AutoSize = true;
			this->label34->Location = System::Drawing::Point(434, 231);
			this->label34->Name = L"label34";
			this->label34->Size = System::Drawing::Size(107, 13);
			this->label34->TabIndex = 65;
			this->label34->Text = L"Lunar Orbit Geometry";
			// 
			// comboBox2
			// 
			this->comboBox2->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBox2->FormattingEnabled = true;
			this->comboBox2->Items->AddRange(gcnew cli::array< System::Object^  >(2) { L"Pacific", L"Atlantic" });
			this->comboBox2->Location = System::Drawing::Point(112, 191);
			this->comboBox2->Name = L"comboBox2";
			this->comboBox2->Size = System::Drawing::Size(121, 21);
			this->comboBox2->TabIndex = 64;
			// 
			// label33
			// 
			this->label33->AutoSize = true;
			this->label33->Location = System::Drawing::Point(40, 194);
			this->label33->Name = L"label33";
			this->label33->Size = System::Drawing::Size(46, 13);
			this->label33->TabIndex = 63;
			this->label33->Text = L"Window";
			// 
			// textBox23
			// 
			this->textBox23->Location = System::Drawing::Point(112, 284);
			this->textBox23->Name = L"textBox23";
			this->textBox23->Size = System::Drawing::Size(100, 20);
			this->textBox23->TabIndex = 62;
			this->textBox23->Text = L"73.13";
			// 
			// textBox22
			// 
			this->textBox22->Location = System::Drawing::Point(138, 542);
			this->textBox22->Name = L"textBox22";
			this->textBox22->Size = System::Drawing::Size(74, 20);
			this->textBox22->TabIndex = 61;
			this->textBox22->Text = L"-91.0";
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->Location = System::Drawing::Point(24, 287);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(70, 13);
			this->label31->TabIndex = 60;
			this->label31->Text = L"DT TLI to PC";
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->Location = System::Drawing::Point(40, 545);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(92, 13);
			this->label30->TabIndex = 59;
			this->label30->Text = L"Approach azimuth";
			// 
			// textBox21
			// 
			this->textBox21->Location = System::Drawing::Point(112, 507);
			this->textBox21->Name = L"textBox21";
			this->textBox21->Size = System::Drawing::Size(100, 20);
			this->textBox21->TabIndex = 57;
			this->textBox21->Text = L"-1.44";
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->Location = System::Drawing::Point(40, 510);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(51, 13);
			this->label28->TabIndex = 56;
			this->label28->Text = L"Elevation";
			// 
			// textBox20
			// 
			this->textBox20->Location = System::Drawing::Point(112, 479);
			this->textBox20->Name = L"textBox20";
			this->textBox20->Size = System::Drawing::Size(100, 20);
			this->textBox20->TabIndex = 55;
			this->textBox20->Text = L"23.7169";
			// 
			// textBox19
			// 
			this->textBox19->Location = System::Drawing::Point(112, 451);
			this->textBox19->Name = L"textBox19";
			this->textBox19->Size = System::Drawing::Size(100, 20);
			this->textBox19->TabIndex = 54;
			this->textBox19->Text = L"0.6914";
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->Location = System::Drawing::Point(40, 482);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(54, 13);
			this->label27->TabIndex = 53;
			this->label27->Text = L"Longitude";
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->Location = System::Drawing::Point(40, 454);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(45, 13);
			this->label26->TabIndex = 52;
			this->label26->Text = L"Latitude";
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->Location = System::Drawing::Point(109, 428);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(66, 13);
			this->label25->TabIndex = 51;
			this->label25->Text = L"Landing Site";
			// 
			// richTextBox2
			// 
			this->richTextBox2->Location = System::Drawing::Point(673, 14);
			this->richTextBox2->Name = L"richTextBox2";
			this->richTextBox2->Size = System::Drawing::Size(440, 473);
			this->richTextBox2->TabIndex = 50;
			this->richTextBox2->Text = L"";
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(171, 595);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(75, 23);
			this->button3->TabIndex = 49;
			this->button3->Text = L"Calculate";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
			// 
			// textBox15
			// 
			this->textBox15->Location = System::Drawing::Point(112, 159);
			this->textBox15->Name = L"textBox15";
			this->textBox15->Size = System::Drawing::Size(100, 20);
			this->textBox15->TabIndex = 48;
			this->textBox15->Text = L"72.0";
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Location = System::Drawing::Point(40, 162);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(44, 13);
			this->label19->TabIndex = 47;
			this->label19->Text = L"Azimuth";
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Location = System::Drawing::Point(109, 132);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(43, 13);
			this->label20->TabIndex = 46;
			this->label20->Text = L"Launch";
			// 
			// textBox16
			// 
			this->textBox16->Location = System::Drawing::Point(112, 97);
			this->textBox16->Name = L"textBox16";
			this->textBox16->Size = System::Drawing::Size(100, 20);
			this->textBox16->TabIndex = 45;
			this->textBox16->Text = L"16";
			// 
			// textBox17
			// 
			this->textBox17->Location = System::Drawing::Point(112, 69);
			this->textBox17->Name = L"textBox17";
			this->textBox17->Size = System::Drawing::Size(100, 20);
			this->textBox17->TabIndex = 44;
			this->textBox17->Text = L"7";
			// 
			// textBox18
			// 
			this->textBox18->Location = System::Drawing::Point(112, 41);
			this->textBox18->Name = L"textBox18";
			this->textBox18->Size = System::Drawing::Size(100, 20);
			this->textBox18->TabIndex = 43;
			this->textBox18->Text = L"1969";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(40, 100);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(26, 13);
			this->label21->TabIndex = 42;
			this->label21->Text = L"Day";
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(40, 72);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(37, 13);
			this->label22->TabIndex = 41;
			this->label22->Text = L"Month";
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->Location = System::Drawing::Point(40, 44);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(29, 13);
			this->label23->TabIndex = 40;
			this->label23->Text = L"Year";
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->Location = System::Drawing::Point(109, 14);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(65, 13);
			this->label24->TabIndex = 39;
			this->label24->Text = L"Launch Day";
			// 
			// tabPage4
			// 
			this->tabPage4->BackColor = System::Drawing::SystemColors::Control;
			this->tabPage4->Controls->Add(this->chart2);
			this->tabPage4->Controls->Add(this->button7);
			this->tabPage4->Controls->Add(this->label46);
			this->tabPage4->Controls->Add(this->textBox32);
			this->tabPage4->Controls->Add(this->chart1);
			this->tabPage4->Location = System::Drawing::Point(4, 22);
			this->tabPage4->Name = L"tabPage4";
			this->tabPage4->Padding = System::Windows::Forms::Padding(3);
			this->tabPage4->Size = System::Drawing::Size(1132, 625);
			this->tabPage4->TabIndex = 3;
			this->tabPage4->Text = L"LV Preprocessor";
			// 
			// chart2
			// 
			chartArea3->Name = L"ChartArea1";
			this->chart2->ChartAreas->Add(chartArea3);
			legend3->Name = L"Legend1";
			this->chart2->Legends->Add(legend3);
			this->chart2->Location = System::Drawing::Point(669, 50);
			this->chart2->Name = L"chart2";
			series3->ChartArea = L"ChartArea1";
			series3->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
			series3->Legend = L"Legend1";
			series3->Name = L"Launch Azimuth";
			this->chart2->Series->Add(series3);
			this->chart2->Size = System::Drawing::Size(452, 274);
			this->chart2->TabIndex = 4;
			this->chart2->Text = L"chart2";
			// 
			// button7
			// 
			this->button7->Location = System::Drawing::Point(398, 31);
			this->button7->Name = L"button7";
			this->button7->Size = System::Drawing::Size(75, 23);
			this->button7->TabIndex = 3;
			this->button7->Text = L"Load";
			this->button7->UseVisualStyleBackColor = true;
			this->button7->Click += gcnew System::EventHandler(this, &Form1::button7_Click);
			// 
			// label46
			// 
			this->label46->AutoSize = true;
			this->label46->Location = System::Drawing::Point(38, 36);
			this->label46->Name = L"label46";
			this->label46->Size = System::Drawing::Size(76, 13);
			this->label46->TabIndex = 2;
			this->label46->Text = L"Objectives File";
			// 
			// textBox32
			// 
			this->textBox32->Location = System::Drawing::Point(144, 33);
			this->textBox32->Name = L"textBox32";
			this->textBox32->Size = System::Drawing::Size(218, 20);
			this->textBox32->TabIndex = 1;
			this->textBox32->Text = L"LVTargetingObjectives.txt";
			// 
			// chart1
			// 
			chartArea4->Name = L"ChartArea1";
			this->chart1->ChartAreas->Add(chartArea4);
			legend4->Name = L"Legend1";
			this->chart1->Legends->Add(legend4);
			this->chart1->Location = System::Drawing::Point(669, 344);
			this->chart1->Name = L"chart1";
			series4->ChartArea = L"ChartArea1";
			series4->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
			series4->Legend = L"Legend1";
			series4->Name = L"LATM";
			this->chart1->Series->Add(series4);
			this->chart1->Size = System::Drawing::Size(452, 274);
			this->chart1->TabIndex = 0;
			this->chart1->Text = L"chart1";
			// 
			// tabPage5
			// 
			this->tabPage5->BackColor = System::Drawing::SystemColors::Control;
			this->tabPage5->Controls->Add(this->richTextBox5);
			this->tabPage5->Controls->Add(this->checkBox1);
			this->tabPage5->Controls->Add(this->textBox33);
			this->tabPage5->Controls->Add(this->label50);
			this->tabPage5->Controls->Add(this->button8);
			this->tabPage5->Controls->Add(this->comboBox3);
			this->tabPage5->Controls->Add(this->label49);
			this->tabPage5->Controls->Add(this->label48);
			this->tabPage5->Controls->Add(this->richTextBox4);
			this->tabPage5->Controls->Add(this->label47);
			this->tabPage5->Location = System::Drawing::Point(4, 22);
			this->tabPage5->Name = L"tabPage5";
			this->tabPage5->Padding = System::Windows::Forms::Padding(3);
			this->tabPage5->Size = System::Drawing::Size(1132, 625);
			this->tabPage5->TabIndex = 4;
			this->tabPage5->Text = L"LV Targeting";
			// 
			// richTextBox5
			// 
			this->richTextBox5->Location = System::Drawing::Point(683, 61);
			this->richTextBox5->Name = L"richTextBox5";
			this->richTextBox5->Size = System::Drawing::Size(371, 230);
			this->richTextBox5->TabIndex = 10;
			this->richTextBox5->Text = L"";
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Location = System::Drawing::Point(220, 216);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(15, 14);
			this->checkBox1->TabIndex = 9;
			this->checkBox1->UseVisualStyleBackColor = true;
			// 
			// textBox33
			// 
			this->textBox33->Location = System::Drawing::Point(182, 184);
			this->textBox33->Name = L"textBox33";
			this->textBox33->Size = System::Drawing::Size(100, 20);
			this->textBox33->TabIndex = 8;
			this->textBox33->Text = L"16";
			// 
			// label50
			// 
			this->label50->AutoSize = true;
			this->label50->Location = System::Drawing::Point(53, 187);
			this->label50->Name = L"label50";
			this->label50->Size = System::Drawing::Size(26, 13);
			this->label50->TabIndex = 7;
			this->label50->Text = L"Day";
			// 
			// button8
			// 
			this->button8->Location = System::Drawing::Point(93, 502);
			this->button8->Name = L"button8";
			this->button8->Size = System::Drawing::Size(75, 23);
			this->button8->TabIndex = 6;
			this->button8->Text = L"Run";
			this->button8->UseVisualStyleBackColor = true;
			this->button8->Click += gcnew System::EventHandler(this, &Form1::button8_Click);
			// 
			// comboBox3
			// 
			this->comboBox3->FormattingEnabled = true;
			this->comboBox3->Items->AddRange(gcnew cli::array< System::Object^  >(3) { L"Optimum 1st Opp", L"Optimum 2nd Opp", L"Split Time" });
			this->comboBox3->Location = System::Drawing::Point(182, 275);
			this->comboBox3->Name = L"comboBox3";
			this->comboBox3->Size = System::Drawing::Size(121, 21);
			this->comboBox3->TabIndex = 5;
			// 
			// label49
			// 
			this->label49->AutoSize = true;
			this->label49->Location = System::Drawing::Point(53, 278);
			this->label49->Name = L"label49";
			this->label49->Size = System::Drawing::Size(92, 13);
			this->label49->TabIndex = 4;
			this->label49->Text = L"Split Launch Time";
			// 
			// label48
			// 
			this->label48->AutoSize = true;
			this->label48->Location = System::Drawing::Point(53, 216);
			this->label48->Name = L"label48";
			this->label48->Size = System::Drawing::Size(63, 13);
			this->label48->TabIndex = 2;
			this->label48->Text = L"Free Return";
			// 
			// richTextBox4
			// 
			this->richTextBox4->Location = System::Drawing::Point(56, 61);
			this->richTextBox4->Name = L"richTextBox4";
			this->richTextBox4->Size = System::Drawing::Size(206, 96);
			this->richTextBox4->TabIndex = 1;
			this->richTextBox4->Text = L"";
			// 
			// label47
			// 
			this->label47->AutoSize = true;
			this->label47->Location = System::Drawing::Point(53, 30);
			this->label47->Name = L"label47";
			this->label47->Size = System::Drawing::Size(39, 13);
			this->label47->TabIndex = 0;
			this->label47->Text = L"Cases:";
			// 
			// tabPage7
			// 
			this->tabPage7->BackColor = System::Drawing::SystemColors::Control;
			this->tabPage7->Controls->Add(this->button11);
			this->tabPage7->Controls->Add(this->textBox44);
			this->tabPage7->Controls->Add(this->label73);
			this->tabPage7->Controls->Add(this->richTextBox6);
			this->tabPage7->Location = System::Drawing::Point(4, 22);
			this->tabPage7->Name = L"tabPage7";
			this->tabPage7->Padding = System::Windows::Forms::Padding(3);
			this->tabPage7->Size = System::Drawing::Size(1132, 625);
			this->tabPage7->TabIndex = 6;
			this->tabPage7->Text = L"Scenario";
			// 
			// button11
			// 
			this->button11->Location = System::Drawing::Point(103, 139);
			this->button11->Name = L"button11";
			this->button11->Size = System::Drawing::Size(75, 23);
			this->button11->TabIndex = 3;
			this->button11->Text = L"Load";
			this->button11->UseVisualStyleBackColor = true;
			this->button11->Click += gcnew System::EventHandler(this, &Form1::button11_Click);
			// 
			// textBox44
			// 
			this->textBox44->Location = System::Drawing::Point(189, 41);
			this->textBox44->Name = L"textBox44";
			this->textBox44->Size = System::Drawing::Size(202, 20);
			this->textBox44->TabIndex = 2;
			this->textBox44->Text = L"Default-Preset-Tape-1969-07-16.txt";
			// 
			// label73
			// 
			this->label73->AutoSize = true;
			this->label73->Location = System::Drawing::Point(79, 44);
			this->label73->Name = L"label73";
			this->label73->Size = System::Drawing::Size(56, 13);
			this->label73->TabIndex = 1;
			this->label73->Text = L"Preset File";
			// 
			// richTextBox6
			// 
			this->richTextBox6->Location = System::Drawing::Point(630, 44);
			this->richTextBox6->Name = L"richTextBox6";
			this->richTextBox6->Size = System::Drawing::Size(333, 232);
			this->richTextBox6->TabIndex = 0;
			this->richTextBox6->Text = L"";
			// 
			// label85
			// 
			this->label85->AutoSize = true;
			this->label85->Location = System::Drawing::Point(742, 122);
			this->label85->Name = L"label85";
			this->label85->Size = System::Drawing::Size(88, 13);
			this->label85->TabIndex = 31;
			this->label85->Text = L"Orbit 1st TLI Opp";
			// 
			// textBox46
			// 
			this->textBox46->Location = System::Drawing::Point(870, 119);
			this->textBox46->Name = L"textBox46";
			this->textBox46->Size = System::Drawing::Size(100, 20);
			this->textBox46->TabIndex = 32;
			this->textBox46->Text = L"2";
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1136, 653);
			this->Controls->Add(this->tabControl1);
			this->Name = L"Form1";
			this->Text = L"Apollo Trajectory Design Program";
			this->tabControl1->ResumeLayout(false);
			this->tabPage6->ResumeLayout(false);
			this->tabPage6->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->tabPage1->ResumeLayout(false);
			this->tabPage1->PerformLayout();
			this->tabPage2->ResumeLayout(false);
			this->tabPage2->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->EndInit();
			this->tabPage3->ResumeLayout(false);
			this->tabPage3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown2))->EndInit();
			this->tabPage4->ResumeLayout(false);
			this->tabPage4->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
			this->tabPage5->ResumeLayout(false);
			this->tabPage5->PerformLayout();
			this->tabPage7->ResumeLayout(false);
			this->tabPage7->PerformLayout();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
		CalculateElevationAngle();
	}
private: System::Void button2_Click(System::Object^ sender, System::EventArgs^ e) {
	CalculateTLIFirstGuess();
}
private: System::Void button3_Click(System::Object^ sender, System::EventArgs^ e) {
	ConvergeOptimizedMission();
}
private: System::Void button4_Click(System::Object^ sender, System::EventArgs^ e) {
	ExportDataSets();
}
private: System::Void button5_Click(System::Object^ sender, System::EventArgs^ e) {
	atdp->SaveData();
	UpdateDisplayedLVTargeting();

	this->textBox32->Text = this->textBox42->Text + "-LVTargetingObjectives.txt";
}
private: System::Void button6_Click(System::Object^ sender, System::EventArgs^ e) {
	atdp->DeleteStoredData();
	UpdateDisplayedLVTargeting();
}
private: System::Void button7_Click(System::Object^ sender, System::EventArgs^ e) {
	LoadLVTargetingObjectives();
}
private: System::Void button8_Click(System::Object^ sender, System::EventArgs^ e) {
	RunQRTPCases();
}
private: System::Void checkBox2_CheckedChanged(System::Object^ sender, System::EventArgs^ e) {

	if (this->checkBox2->Checked)
	{
		this->textBox23->ReadOnly = true;
	}
	else
	{
		this->textBox23->ReadOnly = false;
	}
}
private: System::Void button9_Click(System::Object^ sender, System::EventArgs^ e) {
	SaveToFile();
}
private: System::Void button10_Click(System::Object^ sender, System::EventArgs^ e) {
	LoadFromFile();
}
private: System::Void button11_Click(System::Object^ sender, System::EventArgs^ e) {
	GenerateFinalLaunchData();
}
};
}
