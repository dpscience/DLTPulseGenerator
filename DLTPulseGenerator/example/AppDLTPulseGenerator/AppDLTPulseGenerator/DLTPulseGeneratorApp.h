/*******************************************************************************************
**
** Copyright (c) 2017 Danny Petschke. All rights reserved.
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

/* Wrapper-class which inherits from class DLifeTime::DLTCallback to manage the Error-Handling */

#include "dltpulsegenerator.h"

#include <string>

using namespace DLifeTime;
using namespace std;

class DLTPulseGeneratorApply : public DLTCallback {
	DLTPulseGenerator *m_pulseGenerator;
	DLTError		   m_error;

	DLTSetup		   m_setup;
	DLTPulse		   m_pulse;
	DLTPHS			   m_phs;
	DLTSimulationInput m_simulationInput;

public:
	DLTPulseGeneratorApply::DLTPulseGeneratorApply(
		const DLTSimulationInput& simulationInput = DLTSimulationInput_DEMO,
		const DLTPHS& phsDistribution = DLTPHS_DEMO,
		const DLTSetup& setupInfo = DLTSetup_DEMO,
		const DLTPulse& pulseInfo = DLTPulse_DEMO) :
		m_error(DLTErrorType::NONE_ERROR),
		m_simulationInput(simulationInput),
		m_phs(phsDistribution),
		m_setup(setupInfo),
		m_pulse(pulseInfo) {
		m_pulseGenerator = nullptr;
	}

	DLTPulseGeneratorApply::~DLTPulseGeneratorApply() {
		if (m_pulseGenerator) {
			delete m_pulseGenerator;
			m_pulseGenerator = nullptr;
		}
	}

	void DLTPulseGeneratorApply::startGenerating() {
		m_pulseGenerator = new DLTPulseGenerator(m_simulationInput, m_phs, m_setup, m_pulse, this);
	}

	bool DLTPulseGeneratorApply::wasInitilizationSuccessful() const {
		if (getLastError() == DLTErrorType::NONE_ERROR)
			return true;

		return false;
	}

	bool DLTPulseGeneratorApply::generatePulses(DLTPulseF *pulseA, DLTPulseF *pulseB, double triggerA_in_mV, double triggerB_in_mV) {
		if (!pulseA || !pulseB)
			return false;

		if (!m_pulseGenerator)
			return false;

		if (getLastError() != DLTErrorType::NONE_ERROR)
			return false;

		return m_pulseGenerator->emitPulses(pulseA, pulseB, triggerA_in_mV, triggerB_in_mV);
	}

	string DLTPulseGeneratorApply::getErrorString() const {
		if (m_error == DLTErrorType::NONE_ERROR)
			return string("- succeed");

		string errorStr = "";

		/*
		NONE_ERROR							= 0x00000000,

		NO_LIFETIMES_TO_SIMULATE			= 0x00000001,

		SWEEP_INVALID						= 0x00000002,
		NUMBER_OF_CELLS_INVALID				= 0x00000004,
		PDS_UNCERTAINTY_INVALID				= 0x00000008,
		MU_UNCERTAINTY_INVALID				= 0x00000010,
		PULSE_RISE_TIME_INVALID				= 0x00000020,
		PULSE_WIDTH_INVALID					= 0x00000040,
		DELAY_INVALID						= 0x00000080,
		DELAY_LARGER_THAN_SWEEP				= 0x00000100,
		INTENSITY_OF_LIFETIME_BELOW_ZERO	= 0x00000200,
		INTENSITY_OF_BKGRD_BELOW_ZERO		= 0x00000400,
		INTENSITY_OF_PROMT_BELOW_ZERO		= 0x00000800,
		INVALID_SUM_OF_WEIGTHS				= 0x00001000,
		AMPLITUDE_AND_PULSE_POLARITY_MISFIT = 0x00002000,
		AMPLITUDE_AND_PHS_MISFIT			= 0x00004000,
		*/

		if ((m_error & DLTErrorType::NO_LIFETIMES_TO_SIMULATE)) 
			errorStr += "- no lifetimes to simulate\n";
		if ((m_error & DLTErrorType::SWEEP_INVALID))
			errorStr += "- invalid sweep\n";
		if ((m_error & DLTErrorType::PDS_UNCERTAINTY_INVALID))
			errorStr += "- invalid PDS uncertainty\n";
		if ((m_error & DLTErrorType::MU_UNCERTAINTY_INVALID))
			errorStr += "- invalid MU uncertainty\n";
		if ((m_error & DLTErrorType::PULSE_RISE_TIME_INVALID))
			errorStr += "- invalid pulse rise time\n";
		if ((m_error & DLTErrorType::PULSE_WIDTH_INVALID))
			errorStr += "- invalid pulse width\n";
		if ((m_error & DLTErrorType::DELAY_INVALID))
			errorStr += "- invalid delay\n";
		if ((m_error & DLTErrorType::DELAY_LARGER_THAN_SWEEP))
			errorStr += "- delay > sweep\n";
		if ((m_error & DLTErrorType::INTENSITY_OF_LIFETIME_BELOW_ZERO))
			errorStr += "- negative intensity of all or one lifetime component\n";
		if ((m_error & DLTErrorType::INTENSITY_OF_BKGRD_BELOW_ZERO))
			errorStr += "- weight of background occurrences < 0\n";
		if ((m_error & DLTErrorType::INTENSITY_OF_PROMT_BELOW_ZERO))
			errorStr += "- weight of promt events < 0\n";
		if ((m_error & DLTErrorType::INVALID_SUM_OF_WEIGTHS))
			errorStr += "- no lifetimes to simulate\n";
		if ((m_error & DLTErrorType::AMPLITUDE_AND_PULSE_POLARITY_MISFIT))
			errorStr += "- amplitude does not fit the polarity\n";
		if ((m_error & DLTErrorType::NO_LIFETIMES_TO_SIMULATE))
			errorStr += "- all lifetimes are disabled\n";
		if ((m_error & DLTErrorType::AMPLITUDE_AND_PHS_MISFIT))
			errorStr += "- misfit of amplitude and PHS\n";

		return errorStr;
	}

private:
	DLTError DLTPulseGeneratorApply::getLastError() const {
		return m_error;
	}

protected:
	virtual void DLTPulseGeneratorApply::onEvent(DLTError error) {
		m_error = error;

		DLTCallback::onEvent(error);
	}

};
