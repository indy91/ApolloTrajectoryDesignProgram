A tool to generate Translunar Injection (TLI) targeting data for Project Apollo - NASSP.

Installation:

Optional: To get the various output plots by the LV targeting, install Gnuplot and make sure it has been added to the PATH system variable.

Instructions:

Perform planning with the ATDP, check the NonTargetingParameters.txt file for any default LVDC settings. These numbers are not used by the ATDP itself, they are just combined with the targeting presettings calculated by the ATDP into a text file that then contains the entire LVDC scenario section for a launch scenario. For variables descriptions check: https://nassp.space/index.php/Scenario_File_Options#LVDC_options

Init page. On the init page a project can file be saved and loaded.

Lighting page. For a first evaluation of the lighting conditions at a desired landing site, the coordinates can be checked, together with a date of landing, to see if a landing is feasible on a given date. The ideal lighting conditions vary from 5° to 20°. For some landing sites, with high nearby terrain, a higher elevation might be required. After a calculation on this page the coordinates are automatically transferred to the first guess tab. An estimate for the day of launch is also transferred.

First guess page. The primary purpose of this page is to select the launch day. An analytic first guess logic is run to determine an estimate for time of launch, time of pericynthion passage with near free return conditions, as well as the sun elevation angle a few orbits after perincythion. The resulting values from this page are not very accurate yet, but should be close enough to determine if the lighting at the landing site would support a launch on this day. If not the day of launch can be adjusted and another day can be tried. After each calculation on this page, the day of launch is transferred to the mission planning page.

There isn't an easy way to determine if the Pacific or Atlantic TLI window is the best for a given lunar mission. Often the Atlantic launch windows will lead to a night launch. Certain lunar landing site latitudes can be reached better by the Atlantic window. The Pacific window was used by all Apollo missions except for Apollo 17.

Mission planning page.

The results from this page are written to three files, two of which are loaded by the RTCC in NASSP. These files need to be copied into the Config\ProjectApollo\RTCC folder in your NASSP install. The third file contains the launch vehicle targeting objectives for the actual TLI planning in the ATDP. The ATDP is split up in completely separate sections, which can be used individually. All of the mission planing would have been done by the Manned Spacecraft Center. The result of this planning would then be sent to Marshall Space Flight Center, where the actual launch vehicle targeting is done. The LV targeting objectives file is this interface between the two NASA centers. When MSC mission planning is complete, the planning process doesn't have to be started from scratch if launch vehicle only changes are done. Instead the LV targeting objectives file can be loaded on the LV preprocessor page and the process can continue from there on.

Currently only one mission profile is supported, which has the standard LOI-1 followed by a LOI-2 circularization burns. This profile was used until Apollo 12. Hybrid trajectory missions are not supported yet. Both free return and non-free return missions can be planned.

The mission planning process consists of planning not just one mission, but the whole daily launch window, using launch azimuths from 72° to 108°, with two possible TLI opportunities. For a complete set of targeting data at least two launch azimuths and both TLI opportunities need to be planned and stored. The suggest number of azimuths is 5, specifically 72°, 81°, 90°, 99° and 108°. They should be planned in this order, with the second TLI opportunity after each first TLI opportunity. The various parameters on this page can be tweaked until the desired profile is created. With the Calculate button the mission can then be simulated. If the results are good, the Save button is used to store this set of trajectory data for further processing. Then the process should be repeated for the next TLI opportunity or the next launch azimuth. At the end of this, 10 sets of data should have been saved. When everything looks good the Export button is used to write the RTCC files and the LV objectives file.

Here follow descriptions of each input:

-Year, Month, Day: Date on which the launch will occur.
-Azimuth: In degrees, should be a number between 72 and 108.
-Window. Pacific and Atlantic launch windows are possible.
-Opportunity: Only first and second TLI opportunities are supported, happening on the second and third orbit after launch, respectively.
-DT TLI to PC: In hours. Should no free return constraint be desired, this parameter needs to be adjusted to constrain the time from TLI cutoff to pericynthion.
-Free Return: For enforcing a free return constraint on the translunar trajectory.
-Landing Site Latitude, Longitude, Elevation: Select the desired lunar landing site coordinates.
-Approach azimuth: This is the direction from which the landing site is being approached. A typical value is -90°, a normal range of values can be -70° to -110°. This value is the primary way to optimize the DV remaining after TEI. When every other input parameter is as desired, this number needs to be manually tweaked for optimization.
-Lunar Orbit Geometry: Here select the number of orbits between each event in lunar orbit. The events are LOI-1 (orbit insertion into a 60 x 170 NM altitude orbit). LOI-2, circularization to 60 NM altitude. LOI-2 to LLS, number of full orbits from LOI-2 to the lunar landing. The internal logic will account for the remaining angle necessary to reach the landing site. LLS to LOPC, number of full orbits between the lunar landing the lunar orbit plane change burn, which is done by the CSM to fly over the site for a second time, for lunar ascent by the LM. LOPC to LLS2, number of full orbits between the LOPC burn and lunar ascent. LLS2 to TEI, number of full orbits from lunar ascent to TEI.
-Splashdown maximum inclination: For TEI the return (Earth relative) inclination was contrained to 40° for the early Apollo missions. Later it got relaxed to 75°. This contraint is not enforce for the Pericynthion plus 2 DPS abort burn simulation.
-Splashdown longitude: The targeted longitude returning from the Moon.
-Estimate TEC time: An estimate time for the transearth coast needs to be given so that a TEI of desirable magnitude can be calculated.

Outputs:

LV Preprocessor page.

On this page the LV targeting objectives file is loaded and two output plots, for launch azimuth and the target latitude of pericynthion, are shown.

LV Targeting page.

The first step on this page is to make sure that the displayed day under Cases is the correct day of the month for the mission. Also make sure the 2-5 launch azimuths have been correctly loaded. In theory the LV targeting objectives file could contain multiple launch days, but this is not supported yet. Make sure the text box for the launch day is consistent with what is shown under Cases. Only two other options are available at this point. A free return constraint can optionally be used for the mission. The other option is for the launch time. The optimum launch times for the two TLI opportunities (for one launch azimuth), where the TLI burn is completely in-plane, are not identical. So if both opportunities are considered equal, a compromise launch day has to be chose that works well for both opportunities. If this is not required the first TLI opportunity should be selected as optimum. The second TLI opportunity can also be made optimum. The output from this page will show the converged launch time for the initial launch azimuth. Also a file with LVDC presettings is written.

Scenario page.

In the future this page will contain more numbers that can be used to create T-4h launch scenarios. At the moment only the RTCC TLI file is written on this page. The TLI targeting data are send from MSFC to MSC, where they get converted to a different format and units and then written to a tape, to be loaded by the RTCC during the mission. Because it is the goal for the ATDP to keep MSC and MSFC tasks separate, this file is written here and not on the LV Targeting page.
