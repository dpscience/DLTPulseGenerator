/*******************************************************************************************
**
** Copyright (c) 2017, 2018 Danny Petschke. All rights reserved.
** 
** Redistribution and use in source and binary forms, with or without modification, 
** are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice, 
**	  this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice, 
**    this list of conditions and the following disclaimer in the documentation 
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder "Danny Petschke" nor the names of its  
**    contributors may be used to endorse or promote products derived from this software  
**    without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
** OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
** COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
** EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
** TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
** Contact: danny.petschke@uni-wuerzburg.de
**
***********************************************************************************************/

#include "stdafx.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>

#include "DLTPulseGeneratorApp.h" //<- includes dltpulsegenerator.h

using namespace std;

void showLicense();
void showParameterInformation(const DLTSetup& setup, const DLTPulse& pulse, const DLTPHS& phs, const DLTSimulationInput& simulationInput);

int main() {
	printf("This Software shows the Application of 'DLTPulseGenerator' library.\n\n");
	
	showLicense();
	
	DLTSetup		   setup		   = DLTSetup_DEMO; 
	DLTPulse		   pulse		   = DLTPulse_DEMO;
	DLTPHS			   phs			   = DLTPHS_DEMO;
	DLTSimulationInput simulationInput = DLTSimulationInput_DEMO;

	showParameterInformation(setup, pulse, phs, simulationInput);

	string yesNo;
	cout << "\n\nReady to start DLTPulseGenerator? Press any Button ...";
	getline(cin, yesNo);

	/* Wrapper-class which inherits from class DLifeTime::DLTCallback to manage the Error-Handling */
	DLTPulseGeneratorApply generator(simulationInput, phs, setup, pulse);

	generator.startGenerating();

	printf("\nDLTPulseGenerator was initialized with Error-Code: \n%s\n\n", generator.getErrorString().c_str());

	if (!generator.wasInitilizationSuccessful()) { //on Error...
		printf("Please check your Input and start the Simulation again!");
		getline(cin, yesNo);
		return 0;
	}

	double valueA;
	double valueB;

	if ( pulse.isPositiveSignalPolarity )
		printf("Please type the Trigger-Level for branch A (%fmV < valueA < %fmV): value?  ", 0.0f, pulse.amplitude);
	else
		printf("Please type the Trigger-Level for branch A (%fmV > valueA > %fmV): value?  ", pulse.amplitude, 0.0f);

	cin >> valueA;

	if (pulse.isPositiveSignalPolarity)
		printf("Please type the Trigger-Level for branch B (%fmV < valueB < %fmV): value?  ", 0.0f, pulse.amplitude);
	else
		printf("Please type the Trigger-Level for branch B (%fmV > valueB > %fmV): value?  ", pulse.amplitude, 0.0f);

	cin >> valueB;

	printf("\nOK! Trigger-Levels set to: A -> %f [mV] B -> %f [mV]\n\n", valueA, valueB);
	printf("... waiting for Results ...\n\n");

	//Show up the average of generated pulses per second:
	time_t start;
	time_t stop;
	int counter = 0;

	while (1) {
		time(&start);
		time(&stop);
		counter = 0;

		while ( difftime(stop, start) < 5 ) { //averaging all 5 seconds.
			time(&stop);

			DLTPulseF pulseA, pulseB;

			//obtain the pulses:
			generator.generatePulses(&pulseA, &pulseB, valueA, valueB);


			/* all algorithms for exact timing determination and lifetime calculation, respectively, have to be placed here! */
			
			//const double timingA = CFD(pulseA);
			//const double timingB = CFD(pulseA);

			//const double lifetime = calcDifference(timingA, timingB);

			// -> binning the lifetimes (MCA).

			counter++;
		}

		time(&stop); //getting timestamp for logging
		const double avg = (double)counter/difftime(stop, start);

		printf("%s - Average Number of generated Pulses/s: %f\n", asctime(localtime(&stop)), avg);
	}

    return 0;
}

void showParameterInformation(const DLTSetup& setup, const DLTPulse& pulse, const DLTPHS& phs, const DLTSimulationInput& simulationInput) {
	printf("Simulation parameters:\n\n");

	printf("Setup:\n-----------------------------------------------\n");
	DLTIRF irf = setup.irfA.irf1PDS;
	printf("IRF PDS-A (1) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfA.irf2PDS;
	printf("IRF PDS-A (2) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfA.irf3PDS;
	printf("IRF PDS-A (3) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfA.irf4PDS;
	printf("IRF PDS-A (4) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfA.irf5PDS;
	printf("IRF PDS-A (5) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfB.irf1PDS;
	printf("IRF PDS-B (1) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfB.irf2PDS;
	printf("IRF PDS-B (2) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfB.irf3PDS;
	printf("IRF PDS-B (3) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfB.irf4PDS;
	printf("IRF PDS-B (4) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfB.irf5PDS;
	printf("IRF PDS-B (5) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfMU.irf1MU;
	printf("IRF MU (1) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfMU.irf2MU;
	printf("IRF MU (2) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfMU.irf3MU;
	printf("IRF MU (3) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfMU.irf4MU;
	printf("IRF MU (4) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	irf = setup.irfMU.irf5MU;
	printf("IRF MU (5) enabled?:		%i\n", irf.enabled);
	if (irf.enabled) {
		const char* name = {};
		switch (irf.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			name = { "Gaussian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			name = { "Log-Normal" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			name = { "Cauchy/Lorentzian" };
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			name = { "Gaussian" };
			break;
		}

		printf("--------------------------\n");
		printf("Function-Type:			%s\n", name);
		printf("Uncertainty:			%f [ns]\n", irf.uncertainty);
		printf("Intensity:			%f\n", irf.intensity);
		printf("Relative Shift:			%f [ns]\n", irf.relativeShift);
		printf("--------------------------\n");
	}

	printf("Number of Cells:		%i \n", setup.numberOfCells);
	printf("Sweep:				%f [ns]\n", setup.sweep);
	printf("Arrival-Time Spread - ATS:	%f [ns]\n", setup.ATS);

	printf("\nPulse-Definition:\n-----------------------------------------------\n");
	printf("Rise-Time:			%f [ns]\n", pulse.riseTime);
	printf("Pulse-Width:			%f [ns]\n", pulse.pulseWidth);
	printf("Amplitude:			%f [mV]\n", pulse.amplitude);
	printf("Polarity (+?):			%i \n", pulse.isPositiveSignalPolarity);
	printf("Delay:				%f [ns]\n", pulse.delay);

	printf("\nPHS:\n-----------------------------------------------\n");
	printf("Mean Start of A:		%f [ns]\n", phs.meanOfStartA);
	printf("Mean Stop of A:			%f [ns]\n", phs.meanOfStopA);
	printf("Mean Start of B:		%f [ns]\n", phs.meanOfStartB);
	printf("Mean Stop of B:			%f [ns]\n", phs.meanOfStopB);
	printf("Uncertainty Start of A:		%f [ns]\n", phs.stddevOfStartA);
	printf("Uncertainty Stop of A:		%f [ns]\n", phs.stddevOfStopA);
	printf("Uncertainty Start of B:		%f [ns]\n", phs.stddevOfStartB);
	printf("Uncertainty Stop of B:		%f [ns]\n", phs.stddevOfStopB);

	printf("\nLifetimes - Simulation input:\n-----------------------------------------------\n");
	if (simulationInput.lt1_activated) {
		printf("tau 1:				%f [ns]\n", simulationInput.tau1);
		printf("I 1:				%f\n", simulationInput.intensity1);
		printf("Discrete Lifetime 1?:		%i\n", !simulationInput.tau1Distribution.enabled);

		if (simulationInput.tau1Distribution.enabled) {
			const char* name = {};
			switch (simulationInput.tau1Distribution.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				name = { "Gaussian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				name = { "Log-Normal" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				name = { "Cauchy/Lorentzian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				name = { "Gaussian" };
				break;
			}

			printf("Function-Type:			%s\n", name);
			printf("Uncertainty:			%f [ns]\n", simulationInput.tau1Distribution.param1);
			printf("Grid-Count:			%i\n", simulationInput.tau1Distribution.gridNumber);
			printf("Grid-Increment:			%f [ns]\n", simulationInput.tau1Distribution.gridIncrement);
		}
	}

	if (simulationInput.lt2_activated) {
		printf("tau 2:				%f [ns]\n", simulationInput.tau2);
		printf("I 2:				%f\n", simulationInput.intensity2);

		printf("Discrete Lifetime 2?:		%i\n", !simulationInput.tau2Distribution.enabled);

		if (simulationInput.tau2Distribution.enabled) {
			const char* name = {};
			switch (simulationInput.tau2Distribution.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				name = { "Gaussian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				name = { "Log-Normal" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				name = { "Cauchy/Lorentzian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				name = { "Gaussian" };
				break;
			}

			printf("Function-Type:			%s\n", name);
			printf("Uncertainty:			%f [ns]\n", simulationInput.tau2Distribution.param1);
			printf("Grid-Count:			%i\n", simulationInput.tau2Distribution.gridNumber);
			printf("Grid-Increment:			%f [ns]\n", simulationInput.tau2Distribution.gridIncrement);
		}
	}

	if (simulationInput.lt3_activated) {
		printf("tau 3:				%f [ns]\n", simulationInput.tau3);
		printf("I 3:				%f\n", simulationInput.intensity3);

		printf("Discrete Lifetime 3?:		%i\n", !simulationInput.tau3Distribution.enabled);

		if (simulationInput.tau3Distribution.enabled) {
			const char* name = {};
			switch (simulationInput.tau3Distribution.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				name = { "Gaussian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				name = { "Log-Normal" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				name = { "Cauchy/Lorentzian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				name = { "Gaussian" };
				break;
			}

			printf("Function-Type:			%s\n", name);
			printf("Uncertainty:			%f [ns]\n", simulationInput.tau3Distribution.param1);
			printf("Grid-Count:			%i\n", simulationInput.tau3Distribution.gridNumber);
			printf("Grid-Increment:			%f [ns]\n", simulationInput.tau3Distribution.gridIncrement);
		}
	}

	if (simulationInput.lt4_activated) {
		printf("tau 4:				%f [ns]\n", simulationInput.tau4);
		printf("I 4:				%f\n", simulationInput.intensity4);

		printf("Discrete Lifetime 4?:		%i\n", !simulationInput.tau4Distribution.enabled);

		if (simulationInput.tau4Distribution.enabled) {
			const char* name = {};
			switch (simulationInput.tau4Distribution.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				name = { "Gaussian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				name = { "Log-Normal" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				name = { "Cauchy/Lorentzian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				name = { "Gaussian" };
				break;
			}

			printf("Function-Type:			%s\n", name);
			printf("Uncertainty:			%f [ns]\n", simulationInput.tau4Distribution.param1);
			printf("Grid-Count:			%i\n", simulationInput.tau4Distribution.gridNumber);
			printf("Grid-Increment:			%f [ns]\n", simulationInput.tau4Distribution.gridIncrement);
		}
	}

	if (simulationInput.lt5_activated) {
		printf("tau 5:				%f [ns]\n", simulationInput.tau5);
		printf("I 5:				%f\n", simulationInput.intensity5);

		printf("Discrete Lifetime 5?:		%i\n", !simulationInput.tau5Distribution.enabled);

		if (simulationInput.tau5Distribution.enabled) {
			const char* name = {};
			switch (simulationInput.tau5Distribution.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				name = { "Gaussian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				name = { "Log-Normal" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				name = { "Cauchy/Lorentzian" };
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				name = { "Gaussian" };
				break;
			}

			printf("Function-Type:			%s\n", name);
			printf("Uncertainty:			%f [ns]\n", simulationInput.tau5Distribution.param1);
			printf("Grid-Count:			%i\n", simulationInput.tau5Distribution.gridNumber);
			printf("Grid-Increment:			%f [ns]\n", simulationInput.tau5Distribution.gridIncrement);
		}
	}

	printf("Prompt:      			%f [ns]\n", simulationInput.intensityOfPromtOccurrance);
	printf("Background:			%f [ns]\n", simulationInput.intensityOfBackgroundOccurrance);
	printf("Alternating A-B:		%i\n", simulationInput.isStartStopAlternating);
}

void showLicense() {
	printf("/*****************************  BSD 3-clause license  **************************************\
\n**\n\
** Copyright(c) 2017, 2018 Danny Petschke. All rights reserved.\n\
**\n\
** Redistribution and use in source and binary forms, with or without modification,\n\
** are permitted provided that the following conditions are met :\n\
**\n\
** 1. Redistributions of source code must retain the above copyright notice,\n\
**    this list of conditions and the following disclaimer.\n\
**\n\
** 2. Redistributions in binary form must reproduce the above copyright notice,\n\
**    this list of conditions and the following disclaimer in the documentation\n\
**    and / or other materials provided with the distribution.\n\
**\n\
** 3. Neither the name of the copyright holder \"Danny Petschke\" nor the names of its\n\
**    contributors may be used to endorse or promote products derived from this software\n\
**    without specific prior written permission.\n\
**\n\
**\n\
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS\n\
** OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF\n\
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE\n\
** COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,\n\
** EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF\n\
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)\n\
** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR\n\
** TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,\n\
** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n\
**\n\
** Contact: danny.petschke@uni-wuerzburg.de\n\
**\n\
***********************************************************************************************/\n\n");
}
