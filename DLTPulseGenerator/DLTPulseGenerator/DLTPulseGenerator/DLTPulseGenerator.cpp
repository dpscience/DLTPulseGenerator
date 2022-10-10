/*******************************************************************************************
**
** Copyright (c) 2017 - 2022 Dr. Danny Petschke. All rights reserved.
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

#include "dltpulsegenerator.h"

using namespace std;

class DLifeTime::DLTPulseGenerator::DLTDistributionManager {
	friend class DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate;
	friend class DLifeTime::DLTPulseGenerator;

	bool m_isDiscreteLifetime;

	double m_startLifetime;
	double m_stopLifetime;
	double m_lifetimeROI;

	vector<exponential_distribution<double> > m_lifetimeExpDistribution;
	vector<default_random_engine> m_lifetimeExpDistributionGenerator;

	DLTDistributionManager(double startLifetime = 0.0f, double stopLifetime = 0.0f) :
		m_isDiscreteLifetime(false),
		m_startLifetime(startLifetime),
		m_stopLifetime(stopLifetime),
		m_lifetimeROI(m_stopLifetime - m_startLifetime) {}
	~DLTDistributionManager() {
		clear();
	}

	void init(double startLifetime, double stopLifetime, bool isDiscreteLifetime) {
		m_startLifetime = startLifetime;
		m_stopLifetime = stopLifetime;
		m_lifetimeROI = m_stopLifetime - m_startLifetime;
		m_isDiscreteLifetime = isDiscreteLifetime;
	}

	void add(double tau_in_ns) {
		exponential_distribution<double> expDistr(1.0f/tau_in_ns);
		
		default_random_engine rndEngine;
		rndEngine.seed(rand());

		m_lifetimeExpDistribution.push_back(expDistr);
		m_lifetimeExpDistributionGenerator.push_back(rndEngine);
	}

	void clear() {
		m_lifetimeExpDistribution.clear();
		m_lifetimeExpDistributionGenerator.clear();
	}

	void setAsDiscrete(bool isDiscrete) {
		m_isDiscreteLifetime = isDiscrete;
	}

	void setAsDistribution(bool isDistribution) {
		m_isDiscreteLifetime = !isDistribution;
	}

	bool isDiscreteLifetime() const {
		return m_isDiscreteLifetime;
	}

	bool isDistributedLifetime() const {
		return !m_isDiscreteLifetime;
	}

	int getSize() const {
		return m_lifetimeExpDistribution.size();
	}

	double mapLifetime(double lifetime, bool *valid) {
		if (getSize() <= 0) {
			if (valid)
				*valid = false;

			return 0.0f;
		}

		if (valid)
			*valid = true;

		if (isDiscreteLifetime())
			return m_lifetimeExpDistribution.at(0)(m_lifetimeExpDistributionGenerator.at(0));

		
		const int index = (int)(round(((lifetime - m_startLifetime) / m_lifetimeROI)*((double)getSize())));

		if (index >= getSize() || index < 0) {
			if (valid)
				*valid = false;

			return 0.0f;
		}

		return m_lifetimeExpDistribution.at(index)(m_lifetimeExpDistributionGenerator.at(index));
	}
};

class DLifeTime::DLTPulseGenerator::DLTInterpolator {
	friend class DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate;
	friend class DLifeTime::DLTPulseGenerator;

	std::vector<double> m_x, m_y, m_xm, m_z;

	DLTInterpolator() {}
	~DLTInterpolator() {}

	void clear() {
		m_x.clear();
		m_y.clear();
		m_z.clear();
		m_xm.clear();
	}

	void setPoints(const std::vector<double>& x, const std::vector<double>& y) {
		clear();

		const int iv = x.size();

		m_x.resize(iv + 1);
		m_y.resize(iv + 1);
		m_z.resize(iv + 1);
		m_xm.resize(iv + 4);

		for (int i = 0; i < iv; ++i) {
			m_x[i] = x[i];
			m_y[i] = y[i];
		}

		m_x[0] = 2.0*m_x[1] - m_x[2];

		for (int i = 1; i < iv; ++i)
			m_xm[i + 2] = (m_y[i + 1] - m_y[i]) / (m_x[i + 1] - m_x[i]);

		m_xm[iv + 2] = 2.0*m_xm[iv + 1] - m_xm[iv];
		m_xm[iv + 3] = 2.0*m_xm[iv + 2] - m_xm[iv + 1];
		m_xm[2] = 2.0*m_xm[3] - m_xm[4];
		m_xm[1] = 2.0*m_xm[2] - m_xm[3];

		double a = 0, b = 0;

		for (int i = 1; i < iv + 1; ++i) {
			a = fabs(m_xm[i + 3] - m_xm[i + 2]);
			b = fabs(m_xm[i + 1] - m_xm[i]);

			if (a + b != 0)
				m_z[i] = (a*m_xm[i + 1] + b * m_xm[i + 2]) / (a + b);
			else
				m_z[i] = (m_xm[i + 2] + m_xm[i + 1]) / 2.0;
		}
	}

	double operator() (double xx) const {
		const int iv = m_x.size() - 1;

		double yy = 0.0;

		if (xx < m_x[1]
			|| xx >= m_x[iv - 3])
			return yy;

		std::vector<double>::const_iterator it;
		it = std::lower_bound(m_x.begin(), m_x.end(), xx);

		int idx = std::max(int(it - m_x.begin()) - 1, 0);

		const double b = m_x[idx + 1] - m_x[idx];
		const double a = xx - m_x[idx];

		yy = m_y[idx] + m_z[idx] * a + (3.0*m_xm[idx + 2] - 2.0*m_z[idx] - m_z[idx + 1])*a*a / b;
		yy = yy + (m_z[idx] + m_z[idx + 1] - 2.0*m_xm[idx + 2])*a*a*a / (b*b);

		return yy;
	}
};

class DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate {
    friend class DLifeTime::DLTPulseGenerator;

	// DLTPulse

	default_random_engine m_generatorPulseBaselineJitterA;             // voltage domain
	normal_distribution<double> m_distributionPulseBaselineJitterA;

	default_random_engine m_generatorPulseRndNoiseA;	               // voltage domain
	normal_distribution<double> m_distributionPulseRndNoiseA;

	default_random_engine m_generatorPulseFixedPatternApertureJitterA; // time domain
	normal_distribution<double> m_distributionPulseFixedPatternApertureJitterA;

	vector<double> m_fixedPatternApertureJitterVecA;

	default_random_engine m_generatorPulseRndApertureJitterA;          // time domain
	normal_distribution<double> m_distributionPulseRndApertureJitterA;

	
	default_random_engine m_generatorPulseBaselineJitterB;             // voltage domain
	normal_distribution<double> m_distributionPulseBaselineJitterB;

	default_random_engine m_generatorPulseRndNoiseB;	               // voltage domain
	normal_distribution<double> m_distributionPulseRndNoiseB;

	default_random_engine m_generatorPulseFixedPatternApertureJitterB; // time domain
	normal_distribution<double> m_distributionPulseFixedPatternApertureJitterB;

	vector<double> m_fixedPatternApertureJitterVecB;

	default_random_engine m_generatorPulseRndApertureJitterB;          // time domain
	normal_distribution<double> m_distributionPulseRndApertureJitterB;

    // DLTPHS

	default_random_engine m_generatorStartA;
	default_random_engine m_generatorStopA;

	default_random_engine m_generatorStartB;
	default_random_engine m_generatorStopB;

	// if 'useGaussianModels' == true:

    normal_distribution<double> m_distributionStartA;
    normal_distribution<double> m_distributionStartB;

    normal_distribution<double> m_distributionStopA;
    normal_distribution<double> m_distributionStopB;

	// if 'useGaussianModels' == false:

	piecewise_constant_distribution<double> m_distributionCustomStartA;
	piecewise_constant_distribution<double> m_distributionCustomStartB;

	piecewise_constant_distribution<double> m_distributionCustomStopA;
	piecewise_constant_distribution<double> m_distributionCustomStopB;

    // DLTSetup

	void *m_distributionPDSA1;
	void *m_distributionPDSA2;
	void *m_distributionPDSA3;
	void *m_distributionPDSA4;
	void *m_distributionPDSA5;

	void *m_distributionPDSB1;
	void *m_distributionPDSB2;
	void *m_distributionPDSB3;
	void *m_distributionPDSB4;
	void *m_distributionPDSB5;

	void *m_distributionMU1;
	void *m_distributionMU2;
	void *m_distributionMU3;
	void *m_distributionMU4;
	void *m_distributionMU5;

	default_random_engine m_generator_PDSA1;
	default_random_engine m_generator_PDSA2;
	default_random_engine m_generator_PDSA3;
	default_random_engine m_generator_PDSA4;
	default_random_engine m_generator_PDSA5;

	default_random_engine m_generator_PDSA;
	piecewise_constant_distribution<double> m_distributionPDSA;

	default_random_engine m_generator_PDSB1;
	default_random_engine m_generator_PDSB2;
	default_random_engine m_generator_PDSB3;
	default_random_engine m_generator_PDSB4;
	default_random_engine m_generator_PDSB5;

	default_random_engine m_generator_PDSB;
	piecewise_constant_distribution<double> m_distributionPDSB;

	default_random_engine m_generator_MU1;
	default_random_engine m_generator_MU2;
	default_random_engine m_generator_MU3;
	default_random_engine m_generator_MU4;
	default_random_engine m_generator_MU5;

	default_random_engine m_generator_MU;
	piecewise_constant_distribution<double> m_distributionMU;

    default_random_engine m_generatorBackground;
    piecewise_constant_distribution<double> m_distributionBackground;

	// DLTSimulationInput

	void *m_distributionLT1;
	void *m_distributionLT2;
	void *m_distributionLT3;
	void *m_distributionLT4;
	void *m_distributionLT5;

	default_random_engine m_generator_lt1;
	default_random_engine m_generator_lt2;
	default_random_engine m_generator_lt3;
	default_random_engine m_generator_lt4;
	default_random_engine m_generator_lt5;

	// lifetime-distribution grid

	DLTDistributionManager m_lt1Manager;
	DLTDistributionManager m_lt2Manager;
	DLTDistributionManager m_lt3Manager;
	DLTDistributionManager m_lt4Manager;
	DLTDistributionManager m_lt5Manager;

	// LTSelector

	default_random_engine m_generatorLTSelector;
	piecewise_constant_distribution<double> m_distributionLTSelector;

	DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate() {
		// Lifetime distribution

		m_distributionLT1 = nullptr;
		m_distributionLT2 = nullptr;
		m_distributionLT3 = nullptr;
		m_distributionLT4 = nullptr;
		m_distributionLT5 = nullptr;

		// PDS and MU uncertainties

		m_distributionPDSA1 = nullptr;
		m_distributionPDSA2 = nullptr;
		m_distributionPDSA3 = nullptr;
		m_distributionPDSA4 = nullptr;
		m_distributionPDSA5 = nullptr;

		m_distributionPDSB1 = nullptr;
		m_distributionPDSB2 = nullptr;
		m_distributionPDSB3 = nullptr;
		m_distributionPDSB4 = nullptr;
		m_distributionPDSB5 = nullptr;

		m_distributionMU1 = nullptr;
		m_distributionMU2 = nullptr;
		m_distributionMU3 = nullptr;
		m_distributionMU4 = nullptr;
		m_distributionMU5 = nullptr;
	}
};

DLifeTime::DLTPulseGenerator::DLTPulseGenerator(const DLifeTime::DLTSimulationInput &simulationInput, 
	const DLifeTime::DLTPHS &phsDistribution, 
	const DLifeTime::DLTSetup &setupeInfo, 
	const DLifeTime::DLTPulse &pulseInfo, 
	DLifeTime::DLTCallback *callback) :
	m_privatePtr(new DLTPulseGeneratorPrivate),
	m_simulationInput(simulationInput),
	m_phsDistribution(phsDistribution),
	m_setupInfo(setupeInfo),
	m_pulseInfo(pulseInfo) {
	
	DLifeTime::DLTError error = NONE_ERROR;

    if ( m_setupInfo.sweep <= 10 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::SWEEP_INVALID;
    }

    if ( m_setupInfo.numberOfCells <= 10 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::NUMBER_OF_CELLS_INVALID;
    }

    if ( m_pulseInfo.pulseA.riseTime <= 0 
		|| m_pulseInfo.pulseB.riseTime <= 0) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::PULSE_RISE_TIME_INVALID;
    }

    if ( m_pulseInfo.pulseA.pulseWidth <= 0 
		|| m_pulseInfo.pulseB.pulseWidth <= 0) {
        if ( callback )
			error |= DLifeTime::DLTErrorType::PULSE_WIDTH_INVALID;
    }

    if (m_pulseInfo.delay < 0 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::DELAY_INVALID;
    }

    if ( m_pulseInfo.delay > m_setupInfo.sweep ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::DELAY_LARGER_THAN_SWEEP;
    }

    if ( m_simulationInput.intensity1 < 0
         || m_simulationInput.intensity2 < 0
         || m_simulationInput.intensity3 < 0
         || m_simulationInput.intensity4 < 0
         || m_simulationInput.intensity5 < 0) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::INTENSITY_OF_LIFETIME_BELOW_ZERO;
    }

    if ( m_simulationInput.intensityOfBackgroundOccurrance < 0 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::INTENSITY_OF_BKGRD_BELOW_ZERO;
    }

    if ( m_simulationInput.intensityOfPromptOccurrance < 0 ) {
        if ( callback )
			error |= DLifeTime::DLTErrorType::INTENSITY_OF_PROMT_BELOW_ZERO;
    }

    if ( !m_simulationInput.lt1_activated
         && !m_simulationInput.lt2_activated
         && !m_simulationInput.lt3_activated
         && !m_simulationInput.lt4_activated
         && !m_simulationInput.lt5_activated ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::NO_LIFETIMES_TO_SIMULATE;
    }

    if ( (m_pulseInfo.isPositiveSignalPolarity && m_pulseInfo.amplitude < 0)
         || (!m_pulseInfo.isPositiveSignalPolarity && m_pulseInfo.amplitude > 0) ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::AMPLITUDE_AND_PULSE_POLARITY_MISFIT;
    }

	// init event-counter

    m_eventCounter = 0;

    // init of DLTPHS related random-generators

	initPHS(&error, callback);

	// init of DLTPulse related random generators

	initPulseGenerator(&error, callback);

	// init of DLTSimulationInput related random generators

	initLTGenerator(&error, callback);

	// init of DLTSetup related random generators

	initIRFGenerator(&error, callback);
    
	// generate background 120x of x(FWHM) on the t0-left side

	const double estimatedFWHM = 120*estimateFWHM();

	const int leftSideBin = (estimatedFWHM / m_setupInfo.sweep)*((double)m_setupInfo.numberOfCells*(25000 / 1024)); 
	const int backgroundBinCount = ((double)m_setupInfo.numberOfCells)*(25000 / 1024);
	
	const int startBin = -leftSideBin;       // left of t0
	const int stopBin  = backgroundBinCount; // right of t0 -> sweep

	const int backgroundFullBinCount = abs(startBin) + stopBin + 1;

	const double backgroundLTIncr = (m_setupInfo.sweep+estimatedFWHM)/((double)backgroundFullBinCount);

    vector<double> intervalsBG;
    vector<double> weightsBG;

    weightsBG.resize(backgroundFullBinCount);
    intervalsBG.resize(backgroundFullBinCount + 1);

	int cnt = 0;

    for ( int i = startBin ; i <= stopBin; ++ i ) {
        intervalsBG[cnt] = (double)i*backgroundLTIncr;

        if ( i < stopBin )
           weightsBG[cnt] = 1.0f;

		cnt++;
    }

    m_privatePtr.get()->m_generatorBackground.seed(rand());
    m_privatePtr.get()->m_distributionBackground = piecewise_constant_distribution<double>(intervalsBG.begin(), intervalsBG.end(), weightsBG.begin());

    const double iLT1 = m_simulationInput.lt1_activated?m_simulationInput.intensity1*100.0f:0.0f; // [%]
    const double iLT2 = m_simulationInput.lt2_activated?m_simulationInput.intensity2*100.0f:0.0f; // [%]
    const double iLT3 = m_simulationInput.lt3_activated?m_simulationInput.intensity3*100.0f:0.0f; // [%]
    const double iLT4 = m_simulationInput.lt4_activated?m_simulationInput.intensity4*100.0f:0.0f; // [%]
    const double iLT5 = m_simulationInput.lt5_activated?m_simulationInput.intensity5*100.0f:0.0f; // [%]

    const double wBG  = m_simulationInput.intensityOfBackgroundOccurrance*100.0f;	// [%]
    const double wPro = m_simulationInput.intensityOfPromptOccurrance*100.0f;		// [%]
	const double wLT  = (100.0f - wBG - wPro); // [%]

	if (wLT < 0.0f 
		|| (wBG + wPro + wLT) > 100.0f) {
		if (callback) 
			error |= DLifeTime::DLTErrorType::INVALID_SUM_OF_WEIGTHS;
	}

    array<double, 8> intervalsLTSelector{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    array<double, 7> weightsLTSelector{iLT1*wLT*0.01f, iLT2*wLT*0.01f, iLT3*wLT*0.01f, iLT4*wLT*0.01f, iLT5*wLT*0.01f, wBG, wPro};

    m_privatePtr.get()->m_generatorLTSelector.seed(rand());
    m_privatePtr.get()->m_distributionLTSelector = piecewise_constant_distribution<double>(intervalsLTSelector.begin(), intervalsLTSelector.end(), weightsLTSelector.begin());

	if (callback)
		callback->onEvent(error);
}

void DLifeTime::DLTPulseGenerator::initPHS(DLTError *error, DLTCallback *callback) {
	if (m_phsDistribution.useGaussianModels) {
		if ((m_pulseInfo.isPositiveSignalPolarity
			&& !(m_phsDistribution.meanOfStartA > 0
				&& m_phsDistribution.meanOfStartB > 0
				&& m_phsDistribution.meanOfStopA > 0
				&& m_phsDistribution.meanOfStopB > 0
				&& m_phsDistribution.stddevOfStartA > 0
				&& m_phsDistribution.stddevOfStartB > 0
				&& m_phsDistribution.stddevOfStopA > 0
				&& m_phsDistribution.stddevOfStopB > 0))
			|| (!m_pulseInfo.isPositiveSignalPolarity
				&& !(m_phsDistribution.meanOfStartA < 0
					&& m_phsDistribution.meanOfStartB < 0
					&& m_phsDistribution.meanOfStopA < 0
					&& m_phsDistribution.meanOfStopB < 0
					&& m_phsDistribution.stddevOfStartA < 0
					&& m_phsDistribution.stddevOfStartB < 0
					&& m_phsDistribution.stddevOfStopA < 0
					&& m_phsDistribution.stddevOfStopB < 0))) {
			if (error && callback)
				*error |= DLifeTime::DLTErrorType::AMPLITUDE_AND_PHS_MISFIT;
		}
	}
	else {
		if (m_phsDistribution.gridNumberA < 10
			|| m_phsDistribution.gridNumberB < 10) {
			if (error && callback)
				*error |= DLifeTime::DLTErrorType::INVALID_PHS_GRID;
		}

		if (m_phsDistribution.resolutionMilliVoltPerStepA <= 0.
			|| m_phsDistribution.resolutionMilliVoltPerStepB <= 0.) {
			if (error && callback)
				*error |= DLifeTime::DLTErrorType::INVALID_PHS_RESOLUTION;
		}
	}

	m_privatePtr.get()->m_generatorStartA.seed(rand());
	m_privatePtr.get()->m_generatorStartB.seed(rand());

	m_privatePtr.get()->m_generatorStopA.seed(rand());
	m_privatePtr.get()->m_generatorStopB.seed(rand());

	if (m_phsDistribution.useGaussianModels) {
		m_privatePtr.get()->m_distributionStartA = normal_distribution<double>(m_phsDistribution.meanOfStartA, m_phsDistribution.stddevOfStartA);
		m_privatePtr.get()->m_distributionStartB = normal_distribution<double>(m_phsDistribution.meanOfStartB, m_phsDistribution.stddevOfStartB);

		m_privatePtr.get()->m_distributionStopA = normal_distribution<double>(m_phsDistribution.meanOfStopA, m_phsDistribution.stddevOfStopA);
		m_privatePtr.get()->m_distributionStopB = normal_distribution<double>(m_phsDistribution.meanOfStopB, m_phsDistribution.stddevOfStopB);
	}
	else {
		// just set it to any value eventhough we don`t need it ...

		m_privatePtr.get()->m_distributionStartA = normal_distribution<double>(0., 1.);
		m_privatePtr.get()->m_distributionStartB = normal_distribution<double>(0., 1.);

		m_privatePtr.get()->m_distributionStopA = normal_distribution<double>(0., 1.);
		m_privatePtr.get()->m_distributionStopB = normal_distribution<double>(0., 1.);

		const int phsSizeAStart = m_phsDistribution.distributionStartA.size();
		const int phsSizeAStop = m_phsDistribution.distributionStopA.size();

		const int phsSizeBStart = m_phsDistribution.distributionStartB.size();
		const int phsSizeBStop = m_phsDistribution.distributionStopB.size();

		std::vector<double> x;

		// start A

		for (int i = 0; i < phsSizeAStart; ++i)
			x.push_back(i*m_phsDistribution.resolutionMilliVoltPerStepA);

		DLTInterpolator startA;
		startA.setPoints(x, m_phsDistribution.distributionStartA);

		x.clear();

		// stop A

		for (int i = 0; i < phsSizeAStop; ++i)
			x.push_back(i*m_phsDistribution.resolutionMilliVoltPerStepA);

		DLTInterpolator stopA;
		stopA.setPoints(x, m_phsDistribution.distributionStopA);

		x.clear();

		// start B

		for (int i = 0; i < phsSizeBStart; ++i)
			x.push_back(i*m_phsDistribution.resolutionMilliVoltPerStepB);

		DLTInterpolator startB;
		startB.setPoints(x, m_phsDistribution.distributionStartB);

		x.clear();

		// stop B

		for (int i = 0; i < phsSizeBStop; ++i)
			x.push_back(i*m_phsDistribution.resolutionMilliVoltPerStepB);

		DLTInterpolator stopB;
		stopB.setPoints(x, m_phsDistribution.distributionStopB);

		x.clear();
		
		// re-arrange A (start & stop)

		const double incrementA = std::abs(m_pulseInfo.amplitude) / double(m_phsDistribution.gridNumberA);

		const double maxStartA = phsSizeAStart * m_phsDistribution.resolutionMilliVoltPerStepA;
		const double maxStopA = phsSizeAStop * m_phsDistribution.resolutionMilliVoltPerStepA;

		std::vector<double> _xStartStopA;

		std::vector<double> _yStartA;
		std::vector<double> _yStopA;

		double sumStartA = 0.;
		double sumStopA = 0.;

		for (int i = 0; i <= m_phsDistribution.gridNumberA; ++i) {
			const double xStartStop = incrementA * i;

			_xStartStopA.push_back(xStartStop);

			if (xStartStop <= maxStartA) {
				_yStartA.push_back(startA(xStartStop));

				sumStartA += _yStartA.at(_yStartA.size()-1);
			}
			else 
				_yStartA.push_back(0.);
			
			if (xStartStop <= maxStopA) {
				_yStopA.push_back(stopA(xStartStop));

				sumStopA += _yStopA.at(_yStopA.size() - 1);
			}
			else
				_yStopA.push_back(0.);
		}

		// normalize 

		for (int i = 0; i <= _yStartA.size(); ++i) {
			_yStartA[i] /= sumStartA;
			_yStopA[i] /= sumStopA;
		}

		// re-arrange B (start & stop)

		const double incrementB = std::abs(m_pulseInfo.amplitude) / double(m_phsDistribution.gridNumberB);

		const double maxStartB = phsSizeBStart * m_phsDistribution.resolutionMilliVoltPerStepB;
		const double maxStopB = phsSizeBStop * m_phsDistribution.resolutionMilliVoltPerStepB;

		std::vector<double> _xStartStopB;

		std::vector<double> _yStartB;
		std::vector<double> _yStopB;

		double sumStartB = 0.;
		double sumStopB = 0.;

		for (int i = 0; i <= m_phsDistribution.gridNumberB; ++i) {
			const double xStartStop = incrementB * i;

			_xStartStopB.push_back(xStartStop);

			if (xStartStop <= maxStartB) {
				_yStartB.push_back(startB(xStartStop));

				sumStartB += _yStartB.at(_yStartB.size() - 1);
			}
			else
				_yStartB.push_back(0.);

			if (xStartStop <= maxStopB) {
				_yStopB.push_back(stopB(xStartStop));

				sumStopB += _yStopB.at(_yStopB.size() - 1);
			}
			else
				_yStopB.push_back(0.);
		}

		// normalize 

		for (int i = 0; i <= _yStartB.size(); ++i) {
			_yStartB[i] /= sumStartB;
			_yStopB[i] /= sumStopB;
		}

		// setup piecewise distributions ...

		m_privatePtr.get()->m_distributionCustomStartA = piecewise_constant_distribution<double>(_xStartStopA.begin(), _xStartStopA.end(), _yStartA.begin());
		m_privatePtr.get()->m_distributionCustomStopA = piecewise_constant_distribution<double>(_xStartStopA.begin(), _xStartStopA.end(), _yStopA.begin());

		m_privatePtr.get()->m_distributionCustomStartB = piecewise_constant_distribution<double>(_xStartStopB.begin(), _xStartStopB.end(), _yStartB.begin());
		m_privatePtr.get()->m_distributionCustomStopB = piecewise_constant_distribution<double>(_xStartStopB.begin(), _xStartStopB.end(), _yStopB.begin());
	}
}

double DLifeTime::DLTPulseGenerator::estimateFWHM() {
	/* this estimation is based on a sum of wighted non-shifted gaussian distribution functions */
	double pdsA = 0.0f;
	double pdsB = 0.0f;
	double mu = 0.0f;

	/* PDS - A */
	if (m_setupInfo.irfA.irf1PDS.enabled) {
		pdsA += m_setupInfo.irfA.irf1PDS.uncertainty*m_setupInfo.irfA.irf1PDS.intensity;
	}

	if (m_setupInfo.irfA.irf2PDS.enabled) {
		pdsA += m_setupInfo.irfA.irf2PDS.uncertainty*m_setupInfo.irfA.irf2PDS.intensity;
	}

	if (m_setupInfo.irfA.irf3PDS.enabled) {
		pdsA += m_setupInfo.irfA.irf3PDS.uncertainty*m_setupInfo.irfA.irf3PDS.intensity;
	}

	if (m_setupInfo.irfA.irf4PDS.enabled) {
		pdsA += m_setupInfo.irfA.irf4PDS.uncertainty*m_setupInfo.irfA.irf4PDS.intensity;
	}

	if (m_setupInfo.irfA.irf5PDS.enabled) {
		pdsA += m_setupInfo.irfA.irf5PDS.uncertainty*m_setupInfo.irfA.irf5PDS.intensity;
	}

	/* PDS - B */
	if (m_setupInfo.irfB.irf1PDS.enabled) {
		pdsB += m_setupInfo.irfB.irf1PDS.uncertainty*m_setupInfo.irfB.irf1PDS.intensity;
	}

	if (m_setupInfo.irfB.irf2PDS.enabled) {
		pdsB += m_setupInfo.irfB.irf2PDS.uncertainty*m_setupInfo.irfB.irf2PDS.intensity;
	}

	if (m_setupInfo.irfB.irf3PDS.enabled) {
		pdsB += m_setupInfo.irfB.irf3PDS.uncertainty*m_setupInfo.irfB.irf3PDS.intensity;
	}

	if (m_setupInfo.irfB.irf4PDS.enabled) {
		pdsB += m_setupInfo.irfB.irf4PDS.uncertainty*m_setupInfo.irfB.irf4PDS.intensity;
	}

	if (m_setupInfo.irfB.irf5PDS.enabled) {
		pdsB += m_setupInfo.irfB.irf5PDS.uncertainty*m_setupInfo.irfB.irf5PDS.intensity;
	}

	/* MU */
	if (m_setupInfo.irfMU.irf1MU.enabled) {
		mu += m_setupInfo.irfMU.irf1MU.uncertainty*m_setupInfo.irfMU.irf1MU.intensity;
	}

	if (m_setupInfo.irfMU.irf2MU.enabled) {
		mu += m_setupInfo.irfMU.irf2MU.uncertainty*m_setupInfo.irfMU.irf2MU.intensity;
	}

	if (m_setupInfo.irfMU.irf3MU.enabled) {
		mu += m_setupInfo.irfMU.irf3MU.uncertainty*m_setupInfo.irfMU.irf3MU.intensity;
	}

	if (m_setupInfo.irfMU.irf4MU.enabled) {
		mu += m_setupInfo.irfMU.irf4MU.uncertainty*m_setupInfo.irfMU.irf4MU.intensity;
	}

	if (m_setupInfo.irfMU.irf5MU.enabled) {
		mu += m_setupInfo.irfMU.irf5MU.uncertainty*m_setupInfo.irfMU.irf5MU.intensity;
	}

	const double branchA = sqrt(pdsA*pdsA + mu*mu);
	const double branchB = sqrt(pdsB*pdsB + mu*mu);
	double estimatedFWHM = sqrt(branchA*branchA + branchB*branchB) * 2 * sqrt(2 * log(2));

	return estimatedFWHM;
}

DLifeTime::DLTPulseGenerator::~DLTPulseGenerator() {
	if (m_privatePtr.get()->m_distributionLT1) {
		delete m_privatePtr.get()->m_distributionLT1;
		m_privatePtr.get()->m_distributionLT1 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionLT2) {
		delete m_privatePtr.get()->m_distributionLT2;
		m_privatePtr.get()->m_distributionLT2 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionLT3) {
		delete m_privatePtr.get()->m_distributionLT3;
		m_privatePtr.get()->m_distributionLT3 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionLT4) {
		delete m_privatePtr.get()->m_distributionLT4;
		m_privatePtr.get()->m_distributionLT4 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionLT5) {
		delete m_privatePtr.get()->m_distributionLT5;
		m_privatePtr.get()->m_distributionLT5 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSA1) {
		delete m_privatePtr.get()->m_distributionPDSA1;
		m_privatePtr.get()->m_distributionPDSA1 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSA2) {
		delete m_privatePtr.get()->m_distributionPDSA2;
		m_privatePtr.get()->m_distributionPDSA2 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSA3) {
		delete m_privatePtr.get()->m_distributionPDSA3;
		m_privatePtr.get()->m_distributionPDSA3 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSA4) {
		delete m_privatePtr.get()->m_distributionPDSA4;
		m_privatePtr.get()->m_distributionPDSA4 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSA5) {
		delete m_privatePtr.get()->m_distributionPDSA5;
		m_privatePtr.get()->m_distributionPDSA5 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSB1) {
		delete m_privatePtr.get()->m_distributionPDSB1;
		m_privatePtr.get()->m_distributionPDSB1 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSB2) {
		delete m_privatePtr.get()->m_distributionPDSB2;
		m_privatePtr.get()->m_distributionPDSB2 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSB3) {
		delete m_privatePtr.get()->m_distributionPDSB3;
		m_privatePtr.get()->m_distributionPDSB3 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSB4) {
		delete m_privatePtr.get()->m_distributionPDSB4;
		m_privatePtr.get()->m_distributionPDSB4 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionPDSB5) {
		delete m_privatePtr.get()->m_distributionPDSB5;
		m_privatePtr.get()->m_distributionPDSB5 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionMU1) {
		delete m_privatePtr.get()->m_distributionMU1;
		m_privatePtr.get()->m_distributionMU1 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionMU2) {
		delete m_privatePtr.get()->m_distributionMU2;
		m_privatePtr.get()->m_distributionMU2 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionMU3) {
		delete m_privatePtr.get()->m_distributionMU3;
		m_privatePtr.get()->m_distributionMU3 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionMU4) {
		delete m_privatePtr.get()->m_distributionMU4;
		m_privatePtr.get()->m_distributionMU4 = nullptr;
	}

	if (m_privatePtr.get()->m_distributionMU5) {
		delete m_privatePtr.get()->m_distributionMU5;
		m_privatePtr.get()->m_distributionMU5 = nullptr;
	}

    m_privatePtr.release();
}

bool DLifeTime::DLTPulseGenerator::emitPulses(DLifeTime::DLTPulseF *pulseA,
	DLifeTime::DLTPulseF *pulseB,
	double triggerLevelA_in_milliVolt, 
	double triggerLevelB_in_milliVolt) {
    if (!pulseA 
		|| !pulseB)
        return false;

    if ( m_eventCounter > INT_MAX/2 )
        m_eventCounter = 0;

	if (!m_simulationInput.isStartStopAlternating)
		m_eventCounter = 1;

    bool isCoincidence = false;
	bool validLifetime = true;

	// pick up next lifetime from the LTSelector ...

    const double nextLT = nextLifeTime(&isCoincidence, &validLifetime); 

	if (!validLifetime)
		return false;

	/* add baseline jitter (V) */

	const double baselineA = (m_pulseInfo.pulseA.baselineOffsetJitterInfoV.enabled ? m_privatePtr.get()->m_distributionPulseBaselineJitterA(m_privatePtr.get()->m_generatorPulseBaselineJitterA) : 0.0);
	const double baselineB = (m_pulseInfo.pulseB.baselineOffsetJitterInfoV.enabled ? m_privatePtr.get()->m_distributionPulseBaselineJitterB(m_privatePtr.get()->m_generatorPulseBaselineJitterB) : 0.0);

	const double overallMeasurementRange = 2.0*abs(m_pulseInfo.amplitude);
	const double digiStepV = overallMeasurementRange / (pow(2, m_pulseInfo.digitizationInfo.digitizationDepth)); // [mV]

	// A (= Start) - B (= Stop) 

    if ( m_eventCounter%2 ) { 
		double amplitudeInMVA = 0.;
		double amplitudeInMVB = 0.;

		if (m_phsDistribution.useGaussianModels) {
			amplitudeInMVA = !isCoincidence ? m_privatePtr.get()->m_distributionStartA(m_privatePtr.get()->m_generatorStartA) : m_privatePtr.get()->m_distributionStopA(m_privatePtr.get()->m_generatorStopA);
			amplitudeInMVB = m_privatePtr.get()->m_distributionStopB(m_privatePtr.get()->m_generatorStopB);
		}
		else {
			amplitudeInMVA = !isCoincidence ? m_privatePtr.get()->m_distributionCustomStartA(m_privatePtr.get()->m_generatorStartA) : m_privatePtr.get()->m_distributionCustomStopA(m_privatePtr.get()->m_generatorStopA);
			amplitudeInMVB = m_privatePtr.get()->m_distributionCustomStopB(m_privatePtr.get()->m_generatorStopB);

			if (!m_pulseInfo.isPositiveSignalPolarity) {
				amplitudeInMVA *= -1.;
				amplitudeInMVB *= -1.;
			}
		}

		const double start_t_in_ns = m_pulseInfo.delay + uncertaintyA();
        const double stop_t_in_ns  = nextLT - m_setupInfo.ATS + uncertaintyB() + m_pulseInfo.delay;

		const double timeIncrInNS = m_setupInfo.sweep / ((double)m_setupInfo.numberOfCells);

        // trigger OK? (AND-logic only)

		const double levelA = amplitudeInMVA + baselineA;
		const double levelB = amplitudeInMVB + baselineB;

        if ( m_pulseInfo.isPositiveSignalPolarity ) {
            if ( !(triggerLevelA_in_milliVolt < levelA && triggerLevelA_in_milliVolt > baselineA
                   && triggerLevelB_in_milliVolt < levelB && triggerLevelB_in_milliVolt > baselineB) )
                return false;
        }
        else {
            if ( !(triggerLevelA_in_milliVolt > levelA && triggerLevelA_in_milliVolt < baselineA
                   && triggerLevelB_in_milliVolt > levelB && triggerLevelB_in_milliVolt < baselineB) )
                return false;
        }

		int runningIndex = 0;

		for (double tP_in_ns = 0.0; tP_in_ns < m_setupInfo.sweep; tP_in_ns += timeIncrInNS) {
            DLTPointF pA, pB;

			double tP_in_ns_A = tP_in_ns;
			double tP_in_ns_B = tP_in_ns;

			/* add non-linearity to time axis */

			if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.enabled) {
				tP_in_ns_A += m_privatePtr.get()->m_fixedPatternApertureJitterVecA.at(runningIndex) + m_privatePtr.get()->m_distributionPulseRndApertureJitterA(m_privatePtr.get()->m_generatorPulseRndApertureJitterA);
			}

			if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.enabled) {
				tP_in_ns_B += m_privatePtr.get()->m_fixedPatternApertureJitterVecB.at(runningIndex) + m_privatePtr.get()->m_distributionPulseRndApertureJitterB(m_privatePtr.get()->m_generatorPulseRndApertureJitterB);
			}

			runningIndex++;

			pA.setX(tP_in_ns_A);
			pB.setX(tP_in_ns_B);

            if ( tP_in_ns < start_t_in_ns ) {
				double ampl = baselineA;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseA.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseA(m_privatePtr.get()->m_generatorPulseRndNoiseA);
				}

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

				pA.setY(ampl);
			}
            else {
				const double tA = (tP_in_ns_A - start_t_in_ns);

                const double lnVal = log(tA/m_pulseInfo.pulseA.riseTime)/m_pulseInfo.pulseA.pulseWidth;

                double ampl = amplitudeInMVA*exp(-0.5f*(lnVal)*(lnVal)) + baselineA;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseA.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseA(m_privatePtr.get()->m_generatorPulseRndNoiseA);
				}

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

                pA.setY(ampl);
            }

            if ( tP_in_ns < stop_t_in_ns ) {
				double ampl = baselineB;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseB.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseB(m_privatePtr.get()->m_generatorPulseRndNoiseB);
				}

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

				pB.setY(ampl);
			}
            else {
				const double tB = (tP_in_ns_B - stop_t_in_ns);

                const double lnVal = log(tB/m_pulseInfo.pulseB.riseTime)/m_pulseInfo.pulseB.pulseWidth;

                double ampl = amplitudeInMVB*exp(-0.5f*(lnVal)*(lnVal)) + baselineB;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseB.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseB(m_privatePtr.get()->m_generatorPulseRndNoiseB);
				}

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

                pB.setY(ampl);
            }

            pulseA->append(pA);
            pulseB->append(pB);
        }
    }

	// B (=Start) - A (= Stop)

    else { 
		double amplitudeInMVA = 0.;
		double amplitudeInMVB = 0.;

		if (m_phsDistribution.useGaussianModels) {
			amplitudeInMVB = (!isCoincidence) ? m_privatePtr.get()->m_distributionStartB(m_privatePtr.get()->m_generatorStartB) : m_privatePtr.get()->m_distributionStopB(m_privatePtr.get()->m_generatorStopB);
			amplitudeInMVA = m_privatePtr.get()->m_distributionStopA(m_privatePtr.get()->m_generatorStopA);
		}
		else {
			amplitudeInMVB = (!isCoincidence) ? m_privatePtr.get()->m_distributionCustomStartB(m_privatePtr.get()->m_generatorStartB) : m_privatePtr.get()->m_distributionCustomStopB(m_privatePtr.get()->m_generatorStopB);
			amplitudeInMVA = m_privatePtr.get()->m_distributionCustomStopA(m_privatePtr.get()->m_generatorStopA);

			if (!m_pulseInfo.isPositiveSignalPolarity) {
				amplitudeInMVA *= -1.;
				amplitudeInMVB *= -1.;
			}
		}

        const double start_t_in_ns = m_pulseInfo.delay + uncertaintyB();
        const double stop_t_in_ns  = nextLT + m_setupInfo.ATS + uncertaintyA() + m_pulseInfo.delay;

		const double timeIncrInNS = m_setupInfo.sweep / ((double)m_setupInfo.numberOfCells);

        // trigger OK? (AND-logic only)

		const double levelA = amplitudeInMVA + baselineA;
		const double levelB = amplitudeInMVB + baselineB;

        if ( m_pulseInfo.isPositiveSignalPolarity ) {
            if ( !(triggerLevelA_in_milliVolt < amplitudeInMVA && triggerLevelA_in_milliVolt > baselineA
                   && triggerLevelB_in_milliVolt < amplitudeInMVB && triggerLevelB_in_milliVolt > baselineB) )
                return false;
        }
        else {
            if ( !(triggerLevelA_in_milliVolt > amplitudeInMVA && triggerLevelA_in_milliVolt < baselineA
                   && triggerLevelB_in_milliVolt > amplitudeInMVB && triggerLevelB_in_milliVolt < baselineB) )
                return false;
        }

		int runningIndex = 0;

        for ( double tP_in_ns = 0.0; tP_in_ns < m_setupInfo.sweep; tP_in_ns += timeIncrInNS ) {
            DLTPointF pA, pB;

			double tP_in_ns_A = tP_in_ns;
			double tP_in_ns_B = tP_in_ns;

			/* add non-linearity to time axis */

			if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.enabled) {
				tP_in_ns_A += m_privatePtr.get()->m_fixedPatternApertureJitterVecA.at(runningIndex) + m_privatePtr.get()->m_distributionPulseRndApertureJitterA(m_privatePtr.get()->m_generatorPulseRndApertureJitterA);
			}

			if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.enabled) {
				tP_in_ns_B += m_privatePtr.get()->m_fixedPatternApertureJitterVecB.at(runningIndex) + m_privatePtr.get()->m_distributionPulseRndApertureJitterB(m_privatePtr.get()->m_generatorPulseRndApertureJitterB);
			}

			runningIndex++;

			pA.setX(tP_in_ns_A);
			pB.setX(tP_in_ns_B);

            if ( tP_in_ns < start_t_in_ns ) {
				double ampl = baselineB;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseB.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseB(m_privatePtr.get()->m_generatorPulseRndNoiseB);
				}

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

				pB.setY(ampl);
			}
            else {
				const double tB = (tP_in_ns_B - start_t_in_ns);

                const double lnVal = log(tB / m_pulseInfo.pulseB.riseTime) / m_pulseInfo.pulseB.pulseWidth;

                double ampl = amplitudeInMVB*exp(-0.5f*(lnVal)*(lnVal)) + baselineB;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseB.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseB(m_privatePtr.get()->m_generatorPulseRndNoiseB);
				}

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

                pB.setY(ampl);
            }

			if ( tP_in_ns < stop_t_in_ns ) {
				double ampl = baselineA;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseA.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseA(m_privatePtr.get()->m_generatorPulseRndNoiseA);
				}

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

				pA.setY(ampl);
			}
            else {
				const double tA = (tP_in_ns_A - stop_t_in_ns);

                const double lnVal = log(tA/m_pulseInfo.pulseA.riseTime)/m_pulseInfo.pulseA.pulseWidth;

                double ampl = amplitudeInMVA*exp(-0.5f*(lnVal)*(lnVal)) + baselineA;

				/* add noise + baseline jitter (V) */

				if (m_pulseInfo.pulseA.randomNoiseInfoV.enabled) {
					ampl += m_privatePtr.get()->m_distributionPulseRndNoiseA(m_privatePtr.get()->m_generatorPulseRndNoiseA);
				}

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

				/* consider digitization depth */

				if (m_pulseInfo.digitizationInfo.enabled) {
					double voltScaled = 0.0;
					int ratio = 1;

					if (ampl < 0.0) {
						voltScaled = overallMeasurementRange*0.5 - abs(ampl);
						ratio = (int)(round(voltScaled / digiStepV));
					}
					else {
						voltScaled = overallMeasurementRange*0.5 + ampl;
						ratio = (int)(round(voltScaled / digiStepV));
					}

					ampl = (ratio*digiStepV) - overallMeasurementRange*0.5;
				}

                pA.setY(ampl);
            }

            pulseA->append(pA);
            pulseB->append(pB);
        }
    }

	if (m_simulationInput.isStartStopAlternating)
		m_eventCounter ++;

    return true;
}

void DLifeTime::DLTPulseGenerator::initPulseGenerator(DLifeTime::DLTError *error, DLifeTime::DLTCallback *callback) {
	/* voltage domain */

	if (m_pulseInfo.digitizationInfo.enabled) {
		if (m_pulseInfo.digitizationInfo.digitizationDepth <= 2) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_DIGITIZATION_DEPTH;
		}
	}

	/* init V - generators - A */
	m_privatePtr.get()->m_generatorPulseBaselineJitterA.seed(rand());
	m_privatePtr.get()->m_generatorPulseRndNoiseA.seed(rand());

	/* baseline jitter - A */
	if (m_pulseInfo.pulseA.baselineOffsetJitterInfoV.enabled) {
		if (m_pulseInfo.isPositiveSignalPolarity
			&& (m_pulseInfo.pulseA.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter >= m_pulseInfo.amplitude)) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}
		else if (!m_pulseInfo.isPositiveSignalPolarity
			&& (m_pulseInfo.pulseA.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter <= m_pulseInfo.amplitude)) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}

		if (m_pulseInfo.pulseA.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseBaselineJitterA = normal_distribution<double>(m_pulseInfo.pulseA.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter, m_pulseInfo.pulseA.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter);		
	}
	else {
		m_privatePtr.get()->m_distributionPulseBaselineJitterA = normal_distribution<double>(0.0, 0.0);
	}

	/* random noise - A */
	if (m_pulseInfo.pulseA.randomNoiseInfoV.enabled) {
		if (m_pulseInfo.pulseA.randomNoiseInfoV.rndNoise < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_RND_NOISE;
		}

		m_privatePtr.get()->m_distributionPulseRndNoiseA = normal_distribution<double>(0.0, m_pulseInfo.pulseA.randomNoiseInfoV.rndNoise);
	}
	else {
		m_privatePtr.get()->m_distributionPulseRndNoiseA = normal_distribution<double>(0.0, 0.0);
	}

	/* init V - generators - B */
	m_privatePtr.get()->m_generatorPulseBaselineJitterB.seed(rand());
	m_privatePtr.get()->m_generatorPulseRndNoiseB.seed(rand());

	/* baseline jitter - B */
	if (m_pulseInfo.pulseB.baselineOffsetJitterInfoV.enabled) {
		if (m_pulseInfo.isPositiveSignalPolarity
			&& (m_pulseInfo.pulseB.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter >= m_pulseInfo.amplitude)) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}
		else if (!m_pulseInfo.isPositiveSignalPolarity
			&& (m_pulseInfo.pulseB.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter <= m_pulseInfo.amplitude)) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}

		if (m_pulseInfo.pulseB.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_BASELINE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseBaselineJitterB = normal_distribution<double>(m_pulseInfo.pulseB.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter, m_pulseInfo.pulseB.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter);
	}
	else {
		m_privatePtr.get()->m_distributionPulseBaselineJitterB = normal_distribution<double>(0.0, 0.0);
	}

	/* random noise - B */
	if (m_pulseInfo.pulseB.randomNoiseInfoV.enabled) {
		if (m_pulseInfo.pulseB.randomNoiseInfoV.rndNoise < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_VOLTAGE_RND_NOISE;
		}

		m_privatePtr.get()->m_distributionPulseRndNoiseB = normal_distribution<double>(0.0, m_pulseInfo.pulseB.randomNoiseInfoV.rndNoise);
	}
	else {
		m_privatePtr.get()->m_distributionPulseRndNoiseB = normal_distribution<double>(0.0, 0.0);
	}


	/* time domain - non-linearity */

	/* init T - generators - A */
	m_privatePtr.get()->m_generatorPulseFixedPatternApertureJitterA.seed(rand());
	m_privatePtr.get()->m_generatorPulseRndApertureJitterA.seed(rand());

	/* fixed pattern aperture jitter - A */
	if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.enabled) {
		const double incrNanoSec = (m_setupInfo.sweep / (double)m_setupInfo.numberOfCells);

		if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter > 0.25*incrNanoSec
			|| m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_FIXED_APERTURE_JITTER;
		}

		if ((m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter + m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter) > 0.25*incrNanoSec) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_FIXED_APERTURE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterA = normal_distribution<double>(0.0, m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter);
	}
	else {
		m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterA = normal_distribution<double>(0.0, 0.0);
	}

	/* store the fixed values */
	m_privatePtr.get()->m_fixedPatternApertureJitterVecA.clear();

	const int regA = m_setupInfo.numberOfCells * 4; // 4 - to be save ...

	for (int i = 0; i < regA; ++i) {
		m_privatePtr.get()->m_fixedPatternApertureJitterVecA.push_back(m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterA(m_privatePtr.get()->m_generatorPulseFixedPatternApertureJitterA));
	}

	/* random aperture jitter - A */
	if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.enabled) {
		const double incrNanoSec = (m_setupInfo.sweep / (double)m_setupInfo.numberOfCells);

		if (m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter > 0.25*incrNanoSec
			|| m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_RND_APERTURE_JITTER;
		}

		if ((m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter + m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter) > 0.25*incrNanoSec) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_RND_APERTURE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseRndApertureJitterA = normal_distribution<double>(0.0, m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter);
	}
	else {
		m_privatePtr.get()->m_distributionPulseRndApertureJitterA = normal_distribution<double>(0.0, 0.0);
	}

	/* init T - generators - B */
	m_privatePtr.get()->m_generatorPulseFixedPatternApertureJitterB.seed(rand());
	m_privatePtr.get()->m_generatorPulseRndApertureJitterB.seed(rand());

	/* fixed pattern aperture jitter - B */
	if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.enabled) {
		const double incrNanoSec = (m_setupInfo.sweep / (double)m_setupInfo.numberOfCells);

		if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter > 0.25*incrNanoSec
			|| m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_FIXED_APERTURE_JITTER;
		}

		if ((m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter + m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter) > 0.25*incrNanoSec) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_FIXED_APERTURE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterB = normal_distribution<double>(0.0, m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter);
	}
	else {
		m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterB = normal_distribution<double>(0.0, 0.0);
	}

	/* store the fixed values */
	m_privatePtr.get()->m_fixedPatternApertureJitterVecB.clear();

	const int regB = m_setupInfo.numberOfCells * 4; // 4 - to be save ...

	for (int i = 0; i < regB; ++i) {
		m_privatePtr.get()->m_fixedPatternApertureJitterVecB.push_back(m_privatePtr.get()->m_distributionPulseFixedPatternApertureJitterB(m_privatePtr.get()->m_generatorPulseFixedPatternApertureJitterB));
	}

	/* random aperture jitter - B */
	if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.enabled) {
		const double incrNanoSec = (m_setupInfo.sweep / (double)m_setupInfo.numberOfCells);

		if (m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter > 0.25*incrNanoSec
			|| m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter < 0.0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_RND_APERTURE_JITTER;
		}

		if ((m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter + m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter) > 0.25*incrNanoSec) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::INVALID_TIME_NONLINEARITY_RND_APERTURE_JITTER;
		}

		m_privatePtr.get()->m_distributionPulseRndApertureJitterB = normal_distribution<double>(0.0, m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter);
	}
	else {
		m_privatePtr.get()->m_distributionPulseRndApertureJitterB = normal_distribution<double>(0.0, 0.0);
	}	
}

void DLifeTime::DLTPulseGenerator::initLTGenerator(DLifeTime::DLTError *error, DLifeTime::DLTCallback *callback) {
	if (m_simulationInput.lt1_activated) {
		if (m_simulationInput.tau1Distribution.enabled) {
			if (m_simulationInput.tau1Distribution.gridIncrement <= 0.0f
				|| m_simulationInput.tau1Distribution.gridNumber <= 0
				|| m_simulationInput.tau1Distribution.param1 <= 0.0f) {
				if (callback && error)
					*error |= DLifeTime::DLTErrorType::INVALID_LIFETIME_DISTRIBUTION_INPUT;
			}
		}
	}

	if (m_simulationInput.lt2_activated) {
		if (m_simulationInput.tau2Distribution.enabled) {
			if (m_simulationInput.tau2Distribution.gridIncrement <= 0.0f
				|| m_simulationInput.tau2Distribution.gridNumber <= 0
				|| m_simulationInput.tau2Distribution.param1 <= 0.0f) {
				if (callback && error)
					*error |= DLifeTime::DLTErrorType::INVALID_LIFETIME_DISTRIBUTION_INPUT;
			}
		}
	}

	if (m_simulationInput.lt3_activated) {
		if (m_simulationInput.tau3Distribution.enabled) {
			if (m_simulationInput.tau3Distribution.gridIncrement <= 0.0f
				|| m_simulationInput.tau3Distribution.gridNumber <= 0
				|| m_simulationInput.tau3Distribution.param1 <= 0.0f) {
				if (callback && error)
					*error |= DLifeTime::DLTErrorType::INVALID_LIFETIME_DISTRIBUTION_INPUT;
			}
		}
	}

	if (m_simulationInput.lt4_activated) {
		if (m_simulationInput.tau4Distribution.enabled) {
			if (m_simulationInput.tau4Distribution.gridIncrement <= 0.0f
				|| m_simulationInput.tau4Distribution.gridNumber <= 0
				|| m_simulationInput.tau4Distribution.param1 <= 0.0f) {
				if (callback && error)
					*error |= DLifeTime::DLTErrorType::INVALID_LIFETIME_DISTRIBUTION_INPUT;
			}
		}
	}

	if (m_simulationInput.lt5_activated) {
		if (m_simulationInput.tau5Distribution.enabled) {
			if (m_simulationInput.tau5Distribution.gridIncrement <= 0.0f
				|| m_simulationInput.tau5Distribution.gridNumber <= 0
				|| m_simulationInput.tau5Distribution.param1 <= 0.0f) {
				if (callback && error)
					*error |= DLifeTime::DLTErrorType::INVALID_LIFETIME_DISTRIBUTION_INPUT;
			}
		}
	}

	//lifetime 1:
	if (m_privatePtr.get()->m_distributionLT1) {
		delete m_privatePtr.get()->m_distributionLT1;
		m_privatePtr.get()->m_distributionLT1 = nullptr;
	}

	m_privatePtr.get()->m_lt1Manager.clear();

	if (m_simulationInput.lt1_activated) { // lifetime activated?
		m_privatePtr.get()->m_generator_lt1.seed(rand());

		if (m_simulationInput.tau1Distribution.enabled) { //distribution?
			switch (m_simulationInput.tau1Distribution.functionType)
			{
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				m_privatePtr.get()->m_distributionLT1 = new normal_distribution<double>(m_simulationInput.tau1, m_simulationInput.tau1Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				m_privatePtr.get()->m_distributionLT1 = new lognormal_distribution<double>(m_simulationInput.tau1, m_simulationInput.tau1Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				m_privatePtr.get()->m_distributionLT1 = new cauchy_distribution<double>(m_simulationInput.tau1, m_simulationInput.tau1Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				m_privatePtr.get()->m_distributionLT1 = new normal_distribution<double>(m_simulationInput.tau1, m_simulationInput.tau1Distribution.param1);
				break;
			}

			const double lt_region_in_ns = ((double)(m_simulationInput.tau1Distribution.gridNumber - 1))*m_simulationInput.tau1Distribution.gridIncrement;
			double start_lt_in_ns = m_simulationInput.tau1 - lt_region_in_ns*0.5f;

			if (start_lt_in_ns <= 0.0f)
				start_lt_in_ns += fabs(start_lt_in_ns);

			m_privatePtr.get()->m_lt1Manager.init(start_lt_in_ns, start_lt_in_ns + lt_region_in_ns, false);

			for (int i = 0; i < m_simulationInput.tau1Distribution.gridNumber; ++i) {
				const double lt_in_ns = start_lt_in_ns + ((double)i)*m_simulationInput.tau1Distribution.gridIncrement;

				m_privatePtr.get()->m_lt1Manager.add(lt_in_ns);
			}
		}
		else { //discrete!
			m_simulationInput.tau1Distribution.functionType = DLifeTime::DLTDistributionFunction::Function::GAUSSIAN;

			m_privatePtr.get()->m_distributionLT1 = new normal_distribution<double>(m_simulationInput.tau1, 0.0f);

			m_privatePtr.get()->m_lt1Manager.init(m_simulationInput.tau1, m_simulationInput.tau1, true);
			m_privatePtr.get()->m_lt1Manager.add(m_simulationInput.tau1);
		}
	}

	//lifetime 2:
	if (m_privatePtr.get()->m_distributionLT2) {
		delete m_privatePtr.get()->m_distributionLT2;
		m_privatePtr.get()->m_distributionLT2 = nullptr;
	}

	m_privatePtr.get()->m_lt2Manager.clear();

	if (m_simulationInput.lt2_activated) { // lifetime activated?
		m_privatePtr.get()->m_generator_lt2.seed(rand());

		if (m_simulationInput.tau2Distribution.enabled) { //distribution?
			switch (m_simulationInput.tau2Distribution.functionType)
			{
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				m_privatePtr.get()->m_distributionLT2 = new normal_distribution<double>(m_simulationInput.tau2, m_simulationInput.tau2Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				m_privatePtr.get()->m_distributionLT2 = new lognormal_distribution<double>(m_simulationInput.tau2, m_simulationInput.tau2Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				m_privatePtr.get()->m_distributionLT2 = new cauchy_distribution<double>(m_simulationInput.tau2, m_simulationInput.tau2Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				m_privatePtr.get()->m_distributionLT2 = new normal_distribution<double>(m_simulationInput.tau2, m_simulationInput.tau2Distribution.param1);
				break;
			}

			const double lt_region_in_ns = ((double)(m_simulationInput.tau2Distribution.gridNumber - 1))*m_simulationInput.tau2Distribution.gridIncrement;
			double start_lt_in_ns = m_simulationInput.tau2 - lt_region_in_ns*0.5f;

			if (start_lt_in_ns <= 0.0f)
				start_lt_in_ns += fabs(start_lt_in_ns);

			m_privatePtr.get()->m_lt2Manager.init(start_lt_in_ns, start_lt_in_ns + lt_region_in_ns, false);

			for (int i = 0; i < m_simulationInput.tau2Distribution.gridNumber; ++i) {
				const double lt_in_ns = start_lt_in_ns + ((double)i)*m_simulationInput.tau2Distribution.gridIncrement;

				m_privatePtr.get()->m_lt2Manager.add(lt_in_ns);
			}
		}
		else { //discrete!
			m_simulationInput.tau2Distribution.functionType = DLifeTime::DLTDistributionFunction::Function::GAUSSIAN;

			m_privatePtr.get()->m_distributionLT2 = new normal_distribution<double>(m_simulationInput.tau2, 0.0f);

			m_privatePtr.get()->m_lt2Manager.init(m_simulationInput.tau2, m_simulationInput.tau2, true);
			m_privatePtr.get()->m_lt2Manager.add(m_simulationInput.tau2);
		}
	}

	//lifetime 3:
	if (m_privatePtr.get()->m_distributionLT3) {
		delete m_privatePtr.get()->m_distributionLT3;
		m_privatePtr.get()->m_distributionLT3 = nullptr;
	}

	m_privatePtr.get()->m_lt3Manager.clear();

	if (m_simulationInput.lt3_activated) { // lifetime activated?
		m_privatePtr.get()->m_generator_lt3.seed(rand());

		if (m_simulationInput.tau3Distribution.enabled) { //distribution?
			switch (m_simulationInput.tau3Distribution.functionType)
			{
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				m_privatePtr.get()->m_distributionLT3 = new normal_distribution<double>(m_simulationInput.tau3, m_simulationInput.tau3Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				m_privatePtr.get()->m_distributionLT3 = new lognormal_distribution<double>(m_simulationInput.tau3, m_simulationInput.tau3Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				m_privatePtr.get()->m_distributionLT3 = new cauchy_distribution<double>(m_simulationInput.tau3, m_simulationInput.tau3Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				m_privatePtr.get()->m_distributionLT3 = new normal_distribution<double>(m_simulationInput.tau3, m_simulationInput.tau3Distribution.param1);
				break;
			}

			const double lt_region_in_ns = ((double)(m_simulationInput.tau3Distribution.gridNumber - 1))*m_simulationInput.tau3Distribution.gridIncrement;
			double start_lt_in_ns = m_simulationInput.tau3 - lt_region_in_ns*0.5f;

			if (start_lt_in_ns <= 0.0f)
				start_lt_in_ns += fabs(start_lt_in_ns);

			m_privatePtr.get()->m_lt3Manager.init(start_lt_in_ns, start_lt_in_ns + lt_region_in_ns, false);

			for (int i = 0; i < m_simulationInput.tau3Distribution.gridNumber; ++i) {
				const double lt_in_ns = start_lt_in_ns + ((double)i)*m_simulationInput.tau3Distribution.gridIncrement;

				m_privatePtr.get()->m_lt3Manager.add(lt_in_ns);
			}
		}
		else { //discrete!
			m_simulationInput.tau3Distribution.functionType = DLifeTime::DLTDistributionFunction::Function::GAUSSIAN;

			m_privatePtr.get()->m_distributionLT3 = new normal_distribution<double>(m_simulationInput.tau3, 0.0f);

			m_privatePtr.get()->m_lt3Manager.init(m_simulationInput.tau3, m_simulationInput.tau3, true);
			m_privatePtr.get()->m_lt3Manager.add(m_simulationInput.tau3);
		}
	}

	//lifetime 4:
	if (m_privatePtr.get()->m_distributionLT4) {
		delete m_privatePtr.get()->m_distributionLT4;
		m_privatePtr.get()->m_distributionLT4 = nullptr;
	}

	m_privatePtr.get()->m_lt4Manager.clear();

	if (m_simulationInput.lt4_activated) { // lifetime activated?
		m_privatePtr.get()->m_generator_lt4.seed(rand());

		if (m_simulationInput.tau4Distribution.enabled) { //distribution?
			switch (m_simulationInput.tau4Distribution.functionType)
			{
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				m_privatePtr.get()->m_distributionLT4 = new normal_distribution<double>(m_simulationInput.tau4, m_simulationInput.tau4Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				m_privatePtr.get()->m_distributionLT4 = new lognormal_distribution<double>(m_simulationInput.tau4, m_simulationInput.tau4Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				m_privatePtr.get()->m_distributionLT4 = new cauchy_distribution<double>(m_simulationInput.tau4, m_simulationInput.tau4Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				m_privatePtr.get()->m_distributionLT4 = new normal_distribution<double>(m_simulationInput.tau4, m_simulationInput.tau4Distribution.param1);
				break;
			}

			const double lt_region_in_ns = ((double)(m_simulationInput.tau4Distribution.gridNumber - 1))*m_simulationInput.tau4Distribution.gridIncrement;
			double start_lt_in_ns = m_simulationInput.tau4 - lt_region_in_ns*0.5f;

			if (start_lt_in_ns <= 0.0f)
				start_lt_in_ns += fabs(start_lt_in_ns);

			m_privatePtr.get()->m_lt4Manager.init(start_lt_in_ns, start_lt_in_ns + lt_region_in_ns, false);

			for (int i = 0; i < m_simulationInput.tau4Distribution.gridNumber; ++i) {
				const double lt_in_ns = start_lt_in_ns + ((double)i)*m_simulationInput.tau4Distribution.gridIncrement;

				m_privatePtr.get()->m_lt4Manager.add(lt_in_ns);
			}
		}
		else { //discrete!
			m_simulationInput.tau4Distribution.functionType = DLifeTime::DLTDistributionFunction::Function::GAUSSIAN;

			m_privatePtr.get()->m_distributionLT4 = new normal_distribution<double>(m_simulationInput.tau4, 0.0f);

			m_privatePtr.get()->m_lt4Manager.init(m_simulationInput.tau4, m_simulationInput.tau4, true);
			m_privatePtr.get()->m_lt4Manager.add(m_simulationInput.tau4);
		}
	}

	//lifetime 5:
	if (m_privatePtr.get()->m_distributionLT5) {
		delete m_privatePtr.get()->m_distributionLT5;
		m_privatePtr.get()->m_distributionLT5 = nullptr;
	}

	m_privatePtr.get()->m_lt5Manager.clear();

	if (m_simulationInput.lt5_activated) { // lifetime activated?
		m_privatePtr.get()->m_generator_lt5.seed(rand());

		if (m_simulationInput.tau5Distribution.enabled) { //distribution?
			switch (m_simulationInput.tau5Distribution.functionType)
			{
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				m_privatePtr.get()->m_distributionLT5 = new normal_distribution<double>(m_simulationInput.tau5, m_simulationInput.tau5Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				m_privatePtr.get()->m_distributionLT5 = new lognormal_distribution<double>(m_simulationInput.tau5, m_simulationInput.tau5Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				m_privatePtr.get()->m_distributionLT5 = new cauchy_distribution<double>(m_simulationInput.tau5, m_simulationInput.tau5Distribution.param1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				m_privatePtr.get()->m_distributionLT5 = new normal_distribution<double>(m_simulationInput.tau5, m_simulationInput.tau5Distribution.param1);
				break;
			}

			const double lt_region_in_ns = ((double)(m_simulationInput.tau5Distribution.gridNumber - 1))*m_simulationInput.tau5Distribution.gridIncrement;
			double start_lt_in_ns = m_simulationInput.tau5 - lt_region_in_ns*0.5f;

			if (start_lt_in_ns <= 0.0f)
				start_lt_in_ns += fabs(start_lt_in_ns);

			m_privatePtr.get()->m_lt5Manager.init(start_lt_in_ns, start_lt_in_ns + lt_region_in_ns, false);

			for (int i = 0; i < m_simulationInput.tau5Distribution.gridNumber; ++i) {
				const double lt_in_ns = start_lt_in_ns + ((double)i)*m_simulationInput.tau5Distribution.gridIncrement;

				m_privatePtr.get()->m_lt5Manager.add(lt_in_ns);
			}
		}
		else { //discrete!
			m_simulationInput.tau5Distribution.functionType = DLifeTime::DLTDistributionFunction::Function::GAUSSIAN;

			m_privatePtr.get()->m_distributionLT5 = new normal_distribution<double>(m_simulationInput.tau5, 0.0f);

			m_privatePtr.get()->m_lt5Manager.init(m_simulationInput.tau5, m_simulationInput.tau5, true);
			m_privatePtr.get()->m_lt5Manager.add(m_simulationInput.tau5);
		}
	}
}

void DLifeTime::DLTPulseGenerator::initIRFGenerator(DLifeTime::DLTError *error, DLifeTime::DLTCallback *callback) {
	/* irf PDS - A */
	if (m_setupInfo.irfA.irf1PDS.enabled) {
		if (m_setupInfo.irfA.irf1PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfA.irf1PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfA.irf2PDS.enabled) {
		if (m_setupInfo.irfA.irf2PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfA.irf2PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfA.irf3PDS.enabled) {
		if (m_setupInfo.irfA.irf3PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfA.irf3PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfA.irf4PDS.enabled) {
		if (m_setupInfo.irfA.irf4PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfA.irf4PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfA.irf5PDS.enabled) {
		if (m_setupInfo.irfA.irf5PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfA.irf5PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	/* irf PDS - B */
	if (m_setupInfo.irfB.irf1PDS.enabled) {
		if (m_setupInfo.irfB.irf1PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfB.irf1PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfB.irf2PDS.enabled) {
		if (m_setupInfo.irfB.irf2PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfB.irf2PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfB.irf3PDS.enabled) {
		if (m_setupInfo.irfB.irf3PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfB.irf3PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfB.irf4PDS.enabled) {
		if (m_setupInfo.irfB.irf4PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfB.irf4PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfB.irf5PDS.enabled) {
		if (m_setupInfo.irfB.irf5PDS.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfB.irf5PDS.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
		}
	}

	/* irf MU */
	if (m_setupInfo.irfMU.irf1MU.enabled) {
		if (m_setupInfo.irfMU.irf1MU.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfMU.irf1MU.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfMU.irf2MU.enabled) {
		if (m_setupInfo.irfMU.irf2MU.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfMU.irf2MU.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfMU.irf3MU.enabled) {
		if (m_setupInfo.irfMU.irf3MU.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfMU.irf3MU.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfMU.irf4MU.enabled) {
		if (m_setupInfo.irfMU.irf4MU.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfMU.irf4MU.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}
	}

	if (m_setupInfo.irfMU.irf5MU.enabled) {
		if (m_setupInfo.irfMU.irf5MU.uncertainty < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}

		if (m_setupInfo.irfMU.irf5MU.intensity < 0) {
			if (callback && error)
				*error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
		}
	}

	/* irf PDS_A - 1 */
	if (m_privatePtr.get()->m_distributionPDSA1) {
		delete m_privatePtr.get()->m_distributionPDSA1;
		m_privatePtr.get()->m_distributionPDSA1 = nullptr;
	}

	DLifeTime::DLTIRF irf = m_setupInfo.irfA.irf1PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSA1.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSA1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSA1 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSA1 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSA1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_A - 2 */
	if (m_privatePtr.get()->m_distributionPDSA2) {
		delete m_privatePtr.get()->m_distributionPDSA2;
		m_privatePtr.get()->m_distributionPDSA2 = nullptr;
	}

	irf = m_setupInfo.irfA.irf2PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSA2.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSA2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSA2 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSA2 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSA2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_A - 3 */
	if (m_privatePtr.get()->m_distributionPDSA3) {
		delete m_privatePtr.get()->m_distributionPDSA3;
		m_privatePtr.get()->m_distributionPDSA3 = nullptr;
	}

	irf = m_setupInfo.irfA.irf3PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSA3.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSA3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSA3 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSA3 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSA3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_A - 4 */
	if (m_privatePtr.get()->m_distributionPDSA4) {
		delete m_privatePtr.get()->m_distributionPDSA4;
		m_privatePtr.get()->m_distributionPDSA4 = nullptr;
	}

	irf = m_setupInfo.irfA.irf4PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSA4.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSA4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSA4 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSA4 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSA4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_A - 5 */
	if (m_privatePtr.get()->m_distributionPDSA5) {
		delete m_privatePtr.get()->m_distributionPDSA5;
		m_privatePtr.get()->m_distributionPDSA5 = nullptr;
	}

	irf = m_setupInfo.irfA.irf5PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSA5.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSA5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSA5 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSA5 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSA5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_B - 1 */
	if (m_privatePtr.get()->m_distributionPDSB1) {
		delete m_privatePtr.get()->m_distributionPDSB1;
		m_privatePtr.get()->m_distributionPDSB1 = nullptr;
	}

	irf = m_setupInfo.irfB.irf1PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSB1.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSB1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSB1 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSB1 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSB1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_B - 2 */
	if (m_privatePtr.get()->m_distributionPDSB2) {
		delete m_privatePtr.get()->m_distributionPDSB2;
		m_privatePtr.get()->m_distributionPDSB2 = nullptr;
	}

	irf = m_setupInfo.irfB.irf2PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSB2.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSB2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSB2 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSB2 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSB2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_B - 3 */
	if (m_privatePtr.get()->m_distributionPDSB3) {
		delete m_privatePtr.get()->m_distributionPDSB3;
		m_privatePtr.get()->m_distributionPDSB3 = nullptr;
	}

	irf = m_setupInfo.irfB.irf3PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSB3.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSB3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSB3 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSB3 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSB3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_B - 4 */
	if (m_privatePtr.get()->m_distributionPDSB4) {
		delete m_privatePtr.get()->m_distributionPDSB4;
		m_privatePtr.get()->m_distributionPDSB4 = nullptr;
	}

	irf = m_setupInfo.irfB.irf4PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSB4.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSB4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSB4 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSB4 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSB4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf PDS_B - 5 */
	if (m_privatePtr.get()->m_distributionPDSB5) {
		delete m_privatePtr.get()->m_distributionPDSB5;
		m_privatePtr.get()->m_distributionPDSB5 = nullptr;
	}

	irf = m_setupInfo.irfB.irf5PDS;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_PDSB5.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionPDSB5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionPDSB5 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionPDSB5 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionPDSB5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf MU - 1 */
	if (m_privatePtr.get()->m_distributionMU1) {
		delete m_privatePtr.get()->m_distributionMU1;
		m_privatePtr.get()->m_distributionMU1 = nullptr;
	}

	irf = m_setupInfo.irfMU.irf1MU;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_MU1.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionMU1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionMU1 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionMU1 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionMU1 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf MU - 2 */
	if (m_privatePtr.get()->m_distributionMU2) {
		delete m_privatePtr.get()->m_distributionMU2;
		m_privatePtr.get()->m_distributionMU2 = nullptr;
	}

	irf = m_setupInfo.irfMU.irf2MU;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_MU2.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionMU2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionMU2 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionMU2 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionMU2 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf MU - 3 */
	if (m_privatePtr.get()->m_distributionMU3) {
		delete m_privatePtr.get()->m_distributionMU3;
		m_privatePtr.get()->m_distributionMU3 = nullptr;
	}

	irf = m_setupInfo.irfMU.irf3MU;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_MU3.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionMU3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionMU3 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionMU3 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionMU3 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf MU - 4 */
	if (m_privatePtr.get()->m_distributionMU4) {
		delete m_privatePtr.get()->m_distributionMU4;
		m_privatePtr.get()->m_distributionMU4 = nullptr;
	}

	irf = m_setupInfo.irfMU.irf4MU;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_MU4.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionMU4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionMU4 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionMU4 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionMU4 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	/* irf MU - 5 */
	if (m_privatePtr.get()->m_distributionMU5) {
		delete m_privatePtr.get()->m_distributionMU5;
		m_privatePtr.get()->m_distributionMU5 = nullptr;
	}

	irf = m_setupInfo.irfMU.irf5MU;

	if (irf.enabled) { //enabled?
		m_privatePtr.get()->m_generator_MU5.seed(rand());

		switch (irf.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			m_privatePtr.get()->m_distributionMU5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			m_privatePtr.get()->m_distributionMU5 = new lognormal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			m_privatePtr.get()->m_distributionMU5 = new cauchy_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			m_privatePtr.get()->m_distributionMU5 = new normal_distribution<double>(irf.relativeShift, irf.uncertainty);
			break;
		}
	}

	double wIRF_1 = 0.0;
	double wIRF_2 = 0.0;
	double wIRF_3 = 0.0;
	double wIRF_4 = 0.0;
	double wIRF_5 = 0.0;

	/* setup MU - selector */
	if (m_setupInfo.irfMU.irf1MU.enabled)
		wIRF_1 = m_setupInfo.irfMU.irf1MU.intensity * 100;

	if (m_setupInfo.irfMU.irf2MU.enabled)
		wIRF_2 = m_setupInfo.irfMU.irf2MU.intensity * 100;

	if (m_setupInfo.irfMU.irf3MU.enabled)
		wIRF_3 = m_setupInfo.irfMU.irf3MU.intensity * 100;

	if (m_setupInfo.irfMU.irf4MU.enabled)
		wIRF_4 = m_setupInfo.irfMU.irf4MU.intensity * 100;

	if (m_setupInfo.irfMU.irf5MU.enabled)
		wIRF_5 = m_setupInfo.irfMU.irf5MU.intensity * 100;

	if (wIRF_1 + wIRF_2 + wIRF_3 + wIRF_4 + wIRF_5 > 100.0) {
		if (callback && error)
			*error |= DLifeTime::DLTErrorType::INVALID_SUM_OF_MU_IRF_INTENSITIES;
	}

	array<double, 6> intervalsMUSelector{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 };
	array<double, 5> weightsMUSelector{ wIRF_1, wIRF_2, wIRF_3, wIRF_4, wIRF_5 };

	m_privatePtr.get()->m_generator_MU.seed(rand());
	m_privatePtr.get()->m_distributionMU = piecewise_constant_distribution<double>(intervalsMUSelector.begin(), intervalsMUSelector.end(), weightsMUSelector.begin());


	wIRF_1 = 0.0;
	wIRF_2 = 0.0;
	wIRF_3 = 0.0;
	wIRF_4 = 0.0;
	wIRF_5 = 0.0;

	/* setup PDS - A - selector */
	if (m_setupInfo.irfA.irf1PDS.enabled)
		wIRF_1 = m_setupInfo.irfA.irf1PDS.intensity * 100;

	if (m_setupInfo.irfA.irf2PDS.enabled)
		wIRF_2 = m_setupInfo.irfA.irf2PDS.intensity * 100;

	if (m_setupInfo.irfA.irf3PDS.enabled)
		wIRF_3 = m_setupInfo.irfA.irf3PDS.intensity * 100;

	if (m_setupInfo.irfA.irf4PDS.enabled)
		wIRF_4 = m_setupInfo.irfA.irf4PDS.intensity * 100;

	if (m_setupInfo.irfA.irf5PDS.enabled)
		wIRF_5 = m_setupInfo.irfA.irf5PDS.intensity * 100;

	if (wIRF_1 + wIRF_2 + wIRF_3 + wIRF_4 + wIRF_5 > 100.0) {
		if (callback && error)
			*error |= DLifeTime::DLTErrorType::INVALID_SUM_OF_PDS_IRF_INTENSITIES;
	}

	array<double, 6> intervalsPDSASelector{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 };
	array<double, 5> weightsPDSASelector{ wIRF_1, wIRF_2, wIRF_3, wIRF_4, wIRF_5 };

	m_privatePtr.get()->m_generator_PDSA.seed(rand());
	m_privatePtr.get()->m_distributionPDSA = piecewise_constant_distribution<double>(intervalsPDSASelector.begin(), intervalsPDSASelector.end(), weightsPDSASelector.begin());


	wIRF_1 = 0.0;
	wIRF_2 = 0.0;
	wIRF_3 = 0.0;
	wIRF_4 = 0.0;
	wIRF_5 = 0.0;

	/* setup PDS - B - selector */
	if (m_setupInfo.irfB.irf1PDS.enabled)
		wIRF_1 = m_setupInfo.irfB.irf1PDS.intensity * 100;

	if (m_setupInfo.irfB.irf2PDS.enabled)
		wIRF_2 = m_setupInfo.irfB.irf2PDS.intensity * 100;

	if (m_setupInfo.irfB.irf3PDS.enabled)
		wIRF_3 = m_setupInfo.irfB.irf3PDS.intensity * 100;

	if (m_setupInfo.irfB.irf4PDS.enabled)
		wIRF_4 = m_setupInfo.irfB.irf4PDS.intensity * 100;

	if (m_setupInfo.irfB.irf5PDS.enabled)
		wIRF_5 = m_setupInfo.irfB.irf5PDS.intensity * 100;

	if (wIRF_1 + wIRF_2 + wIRF_3 + wIRF_4 + wIRF_5 > 100.0) {
		if (callback && error)
			*error |= DLifeTime::DLTErrorType::INVALID_SUM_OF_PDS_IRF_INTENSITIES;
	}

	array<double, 6> intervalsPDSBSelector{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 };
	array<double, 5> weightsPDSBSelector{ wIRF_1, wIRF_2, wIRF_3, wIRF_4, wIRF_5 };

	m_privatePtr.get()->m_generator_PDSB.seed(rand());
	m_privatePtr.get()->m_distributionPDSB = piecewise_constant_distribution<double>(intervalsPDSBSelector.begin(), intervalsPDSBSelector.end(), weightsPDSBSelector.begin());	
}

double DLifeTime::DLTPulseGenerator::nextLifeTime(bool *bPromt, bool *bValid) {
    if ( !bPromt || !bValid )
        return 0.0f;

    *bPromt = false;
	*bValid = true;

    const int selector = m_privatePtr.get()->m_distributionLTSelector(m_privatePtr.get()->m_generatorLTSelector);

	double lt = 0.0f;

    switch ( selector ) {
	case 0: { //lifetime 1:
		switch (m_simulationInput.tau1Distribution.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			lt = m_privatePtr.get()->m_lt1Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT1)(m_privatePtr.get()->m_generator_lt1), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			lt = m_privatePtr.get()->m_lt1Manager.mapLifetime((*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionLT1)(m_privatePtr.get()->m_generator_lt1), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			lt = m_privatePtr.get()->m_lt1Manager.mapLifetime((*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionLT1)(m_privatePtr.get()->m_generator_lt1), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			lt = m_privatePtr.get()->m_lt1Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT1)(m_privatePtr.get()->m_generator_lt1), bValid);
			break;
		}
	}
		break;

	case 1: { //lifetime 2:
		switch (m_simulationInput.tau2Distribution.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			lt = m_privatePtr.get()->m_lt2Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT2)(m_privatePtr.get()->m_generator_lt2), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			lt = m_privatePtr.get()->m_lt2Manager.mapLifetime((*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionLT2)(m_privatePtr.get()->m_generator_lt2), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			lt = m_privatePtr.get()->m_lt2Manager.mapLifetime((*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionLT2)(m_privatePtr.get()->m_generator_lt2), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			lt = m_privatePtr.get()->m_lt2Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT2)(m_privatePtr.get()->m_generator_lt2), bValid);
			break;
		}
	}
		break;

	case 2: { //lifetime 3:
		switch (m_simulationInput.tau3Distribution.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			lt = m_privatePtr.get()->m_lt3Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT3)(m_privatePtr.get()->m_generator_lt3), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			lt = m_privatePtr.get()->m_lt3Manager.mapLifetime((*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionLT3)(m_privatePtr.get()->m_generator_lt3), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			lt = m_privatePtr.get()->m_lt3Manager.mapLifetime((*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionLT3)(m_privatePtr.get()->m_generator_lt3), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			lt = m_privatePtr.get()->m_lt3Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT3)(m_privatePtr.get()->m_generator_lt3), bValid);
			break;
		}
	}
		break;

	case 3: { //lifetime 4:
		switch (m_simulationInput.tau4Distribution.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			lt = m_privatePtr.get()->m_lt4Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT4)(m_privatePtr.get()->m_generator_lt4), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			lt = m_privatePtr.get()->m_lt4Manager.mapLifetime((*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionLT4)(m_privatePtr.get()->m_generator_lt4), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			lt = m_privatePtr.get()->m_lt4Manager.mapLifetime((*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionLT4)(m_privatePtr.get()->m_generator_lt4), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			lt = m_privatePtr.get()->m_lt4Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT4)(m_privatePtr.get()->m_generator_lt4), bValid);
			break;
		}
	}
		break;

	case 4: { //lifetime 5:
		switch (m_simulationInput.tau5Distribution.functionType)
		{
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			lt = m_privatePtr.get()->m_lt5Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT5)(m_privatePtr.get()->m_generator_lt5), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			lt = m_privatePtr.get()->m_lt5Manager.mapLifetime((*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionLT5)(m_privatePtr.get()->m_generator_lt5), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			lt = m_privatePtr.get()->m_lt5Manager.mapLifetime((*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionLT5)(m_privatePtr.get()->m_generator_lt5), bValid);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			lt = m_privatePtr.get()->m_lt5Manager.mapLifetime((*(normal_distribution<double>*)m_privatePtr.get()->m_distributionLT5)(m_privatePtr.get()->m_generator_lt5), bValid);
			break;
		}
	}
		break;

    case 5: //background
        lt = m_privatePtr.get()->m_distributionBackground(m_privatePtr.get()->m_generatorBackground);
		break;

    case 6: //promt
	default:
        *bPromt = true;
		break;
    }

    return lt;
}

double DLifeTime::DLTPulseGenerator::uncertaintyA() {
	if (!m_setupInfo.irfA.irf1PDS.enabled
		&& !m_setupInfo.irfA.irf2PDS.enabled
		&& !m_setupInfo.irfA.irf3PDS.enabled
		&& !m_setupInfo.irfA.irf4PDS.enabled
		&& !m_setupInfo.irfA.irf5PDS.enabled) 
		return 0.;

	const int fIndex = (int)(m_privatePtr.get()->m_distributionPDSA(m_privatePtr.get()->m_generator_PDSA));

	double val = 0.0f;

	switch (fIndex) {
	case 0: {
		switch (m_setupInfo.irfA.irf1PDS.functionType) {
			case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
				val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA1)(m_privatePtr.get()->m_generator_PDSA1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
				val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA1)(m_privatePtr.get()->m_generator_PDSA1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
				val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSA1)(m_privatePtr.get()->m_generator_PDSA1);
				break;

			case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
			default:
				val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA1)(m_privatePtr.get()->m_generator_PDSA1);
				break;
		}
	}
			break;

	case 1: {
		switch (m_setupInfo.irfA.irf2PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA2)(m_privatePtr.get()->m_generator_PDSA2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA2)(m_privatePtr.get()->m_generator_PDSA2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSA2)(m_privatePtr.get()->m_generator_PDSA2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA2)(m_privatePtr.get()->m_generator_PDSA2);
			break;
		}
	}
			break;

	case 2: {
		switch (m_setupInfo.irfA.irf3PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA3)(m_privatePtr.get()->m_generator_PDSA3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA3)(m_privatePtr.get()->m_generator_PDSA3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSA3)(m_privatePtr.get()->m_generator_PDSA3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA3)(m_privatePtr.get()->m_generator_PDSA3);
			break;
		}
	}
			break;

	case 3: {
		switch (m_setupInfo.irfA.irf4PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA4)(m_privatePtr.get()->m_generator_PDSA4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA4)(m_privatePtr.get()->m_generator_PDSA4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSA4)(m_privatePtr.get()->m_generator_PDSA4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA4)(m_privatePtr.get()->m_generator_PDSA4);
			break;
		}
	}
			break;

	case 4: {
		switch (m_setupInfo.irfA.irf5PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA5)(m_privatePtr.get()->m_generator_PDSA5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA5)(m_privatePtr.get()->m_generator_PDSA5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSA5)(m_privatePtr.get()->m_generator_PDSA5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSA5)(m_privatePtr.get()->m_generator_PDSA5);
			break;
		}
	}
			break;

	default:
		val += 0;
		break;
	}

	addUncertaintyMU(&val);

	return val;
}

double DLifeTime::DLTPulseGenerator::uncertaintyB() {
	if (!m_setupInfo.irfB.irf1PDS.enabled
		&& !m_setupInfo.irfB.irf2PDS.enabled
		&& !m_setupInfo.irfB.irf3PDS.enabled
		&& !m_setupInfo.irfB.irf4PDS.enabled
		&& !m_setupInfo.irfB.irf5PDS.enabled)
		return 0.;

	const int fIndex = (int)(m_privatePtr.get()->m_distributionPDSB(m_privatePtr.get()->m_generator_PDSB));

	double val = 0.0f;

	switch (fIndex) {
	case 0: {
		switch (m_setupInfo.irfB.irf1PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB1)(m_privatePtr.get()->m_generator_PDSB1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB1)(m_privatePtr.get()->m_generator_PDSB1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSB1)(m_privatePtr.get()->m_generator_PDSB1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB1)(m_privatePtr.get()->m_generator_PDSB1);
			break;
		}
	}
			break;

	case 1: {
		switch (m_setupInfo.irfB.irf2PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB2)(m_privatePtr.get()->m_generator_PDSB2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB2)(m_privatePtr.get()->m_generator_PDSB2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSB2)(m_privatePtr.get()->m_generator_PDSB2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB2)(m_privatePtr.get()->m_generator_PDSB2);
			break;
		}
	}
			break;

	case 2: {
		switch (m_setupInfo.irfB.irf3PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB3)(m_privatePtr.get()->m_generator_PDSB3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB3)(m_privatePtr.get()->m_generator_PDSB3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSB3)(m_privatePtr.get()->m_generator_PDSB3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB3)(m_privatePtr.get()->m_generator_PDSB3);
			break;
		}
	}
			break;

	case 3: {
		switch (m_setupInfo.irfB.irf4PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB4)(m_privatePtr.get()->m_generator_PDSB4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB4)(m_privatePtr.get()->m_generator_PDSB4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSB4)(m_privatePtr.get()->m_generator_PDSB4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB4)(m_privatePtr.get()->m_generator_PDSB4);
			break;
		}
	}
			break;

	case 4: {
		switch (m_setupInfo.irfB.irf5PDS.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB5)(m_privatePtr.get()->m_generator_PDSB5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB5)(m_privatePtr.get()->m_generator_PDSB5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionPDSB5)(m_privatePtr.get()->m_generator_PDSB5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionPDSB5)(m_privatePtr.get()->m_generator_PDSB5);
			break;
		}
	}
			break;

	default:
		val += 0;
		break;
	}

	addUncertaintyMU(&val);

	return val;
}

void DLifeTime::DLTPulseGenerator::addUncertaintyMU(double *val) {
	if (!val)
		return;

	if (!m_setupInfo.irfMU.irf1MU.enabled
		&& !m_setupInfo.irfMU.irf2MU.enabled
		&& !m_setupInfo.irfMU.irf3MU.enabled
		&& !m_setupInfo.irfMU.irf4MU.enabled
		&& !m_setupInfo.irfMU.irf5MU.enabled)
		return;

	const int fIndex = (int)(m_privatePtr.get()->m_distributionMU(m_privatePtr.get()->m_generator_MU));

	switch (fIndex) {
	case 0: {
		switch (m_setupInfo.irfMU.irf1MU.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU1)(m_privatePtr.get()->m_generator_MU1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			*val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionMU1)(m_privatePtr.get()->m_generator_MU1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			*val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionMU1)(m_privatePtr.get()->m_generator_MU1);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU1)(m_privatePtr.get()->m_generator_MU1);
			break;
		}
	}
			break;

	case 1: {
		switch (m_setupInfo.irfMU.irf2MU.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU2)(m_privatePtr.get()->m_generator_MU2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			*val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionMU2)(m_privatePtr.get()->m_generator_MU2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			*val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionMU2)(m_privatePtr.get()->m_generator_MU2);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU2)(m_privatePtr.get()->m_generator_MU2);
			break;
		}
	}
			break;

	case 2: {
		switch (m_setupInfo.irfMU.irf3MU.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU3)(m_privatePtr.get()->m_generator_MU3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			*val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionMU3)(m_privatePtr.get()->m_generator_MU3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			*val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionMU3)(m_privatePtr.get()->m_generator_MU3);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU3)(m_privatePtr.get()->m_generator_MU3);
			break;
		}
	}
			break;

	case 3: {
		switch (m_setupInfo.irfMU.irf4MU.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU4)(m_privatePtr.get()->m_generator_MU4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			*val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionMU4)(m_privatePtr.get()->m_generator_MU4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			*val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionMU4)(m_privatePtr.get()->m_generator_MU4);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU4)(m_privatePtr.get()->m_generator_MU4);
			break;
		}
	}
			break;

	case 4: {
		switch (m_setupInfo.irfMU.irf5MU.functionType) {
		case DLifeTime::DLTDistributionFunction::Function::GAUSSIAN:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU5)(m_privatePtr.get()->m_generator_MU5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LOG_NORMAL:
			*val += (*(lognormal_distribution<double>*)m_privatePtr.get()->m_distributionMU5)(m_privatePtr.get()->m_generator_MU5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::LORENTZIAN_CAUCHY:
			*val += (*(cauchy_distribution<double>*)m_privatePtr.get()->m_distributionMU5)(m_privatePtr.get()->m_generator_MU5);
			break;

		case DLifeTime::DLTDistributionFunction::Function::UNKNOWN:
		default:
			*val += (*(normal_distribution<double>*)m_privatePtr.get()->m_distributionMU5)(m_privatePtr.get()->m_generator_MU5);
			break;
		}
	}
			break;

	default:
		*val += 0;
		break;
	}
}

DLifeTime::DLTPointF::DLTPointF() : 
	m_x(0.0f), 
	m_y(0.0f) {}

DLifeTime::DLTPointF::~DLTPointF() {}

void DLifeTime::DLTPointF::setX(double x) {
    m_x = x;
}

void DLifeTime::DLTPointF::setY(double y) {
    m_y = y;
}

void DLifeTime::DLTPointF::setPoint(double x, double y) {
    m_x = x;
    m_y = y;
}

void DLifeTime::DLTPointF::setPoint(const DLTPointF &point) {
    m_x = point.x();
    m_y = point.y();
}

double DLifeTime::DLTPointF::x() const {
    return m_x;
}

double DLifeTime::DLTPointF::y() const {
    return m_y;
}

DLifeTime::DLTPulseF::DLTPulseF(double *time, double *voltage) :
	m_counter(0),
	m_time(nullptr),
	m_voltage(nullptr) {
	m_vector = new vector<DLifeTime::DLTPointF>();

	if (time && voltage) {
		m_time = time;
		m_voltage = voltage;
	}
}

DLifeTime::DLTPulseF::~DLTPulseF() {
	m_vector->clear();

	delete m_vector;
	m_vector = nullptr;
}

void DLifeTime::DLTPulseF::append(const DLTPointF &dataPoint) {
	m_vector->push_back(dataPoint);

	if (m_voltage && m_time)
	{
		m_time[m_counter] = dataPoint.x();
		m_voltage[m_counter] = dataPoint.y();

		m_counter ++;
	}
}

void DLifeTime::DLTPulseF::clear() {
	m_vector->clear();

	m_counter = 0;
}

int DLifeTime::DLTPulseF::size() const {
	return m_vector->size();
}

DLifeTime::DLTPointF DLifeTime::DLTPulseF::at(int index) const {
	if (index >= size()
		|| index < 0)
		return DLifeTime::DLTPointF();

	return m_vector->at(index);
}

/** The class DLT_C_WRAPPER is used as singleton pattern to access from Ansi C functions.
**	It provides an interface to different programming languages such as matlab (mex-compiler) or
**  python (ctypes-library) is provided.
**
**	An example how to use class DLTPulseGenerator in combination with class DLT_C_WRAPPER
**  is given in python: pyDLTPulseGenerator
**/

DLT_C_WRAPPER *__sharedAccessPtr = nullptr;

DLT_C_WRAPPER::DLT_C_WRAPPER() {
	m_setupInfo		  = DLTSetup_DEMO;
	m_pulseInfo		  = DLTPulse_DEMO;
	m_phsDistribution = DLTPHS_DEMO;
	m_simulationInput = DLTSimulationInput_DEMO;

	m_lastError	= DLifeTime::DLTErrorType::NONE_ERROR;

	m_accessPtr = new DLifeTime::DLTPulseGenerator(DLTPulseGeneratorDEMO, this);

	m_pulseA = new DLifeTime::DLTPulseF;
	m_pulseB = new DLifeTime::DLTPulseF;
}

DLT_C_WRAPPER::~DLT_C_WRAPPER() {
	if (m_pulseA) {
		delete m_pulseA;
		m_pulseA = nullptr;
	}

	if (m_pulseB) {
		delete m_pulseB;
		m_pulseB = nullptr;
	}

	if (m_accessPtr) {
		delete m_accessPtr;
		m_accessPtr = nullptr;
	}
}

DLT_C_WRAPPER *DLT_C_WRAPPER::sharedInstance() {
	if (!__sharedAccessPtr)
		__sharedAccessPtr = new DLT_C_WRAPPER;

	return __sharedAccessPtr;
}

void init() {
	if (DLT_C_WRAPPER::sharedInstance()) {} //Call once for initialization!
}

void DLT_C_WRAPPER::reinit() {
	if (m_accessPtr) {
		delete m_accessPtr;
		m_accessPtr = nullptr;
	}

	if (m_pulseA) {
		delete m_pulseA;
		m_pulseA = nullptr;
	}

	if (m_pulseB) {
		delete m_pulseB;
		m_pulseB = nullptr;
	}

	m_lastError = DLifeTime::DLTErrorType::NONE_ERROR;

	m_pulseA = new DLifeTime::DLTPulseF;
	m_pulseB = new DLifeTime::DLTPulseF;

	m_accessPtr = new DLifeTime::DLTPulseGenerator(m_simulationInput, m_phsDistribution, m_setupInfo, m_pulseInfo, this);
}

void update() {
	DLT_C_WRAPPER::sharedInstance()->reinit();
}

void DLT_C_WRAPPER::onEvent(DLifeTime::DLTError error) {
	m_lastError = error;

	DLTCallback::onEvent(error);
}

bool DLT_C_WRAPPER::emitPulse(double trigger_in_mV_A, double trigger_in_mV_B) {
	if (!m_accessPtr)
		return false;

	if (!m_pulseA || !m_pulseB)
		return false;

	m_pulseA->clear();
	m_pulseB->clear();

	return m_accessPtr->emitPulses(m_pulseA, m_pulseB, trigger_in_mV_A, trigger_in_mV_B);
}

bool emitPulse(double trigger_in_mV_A, double trigger_in_mV_B) {
	return DLT_C_WRAPPER::sharedInstance()->emitPulse(trigger_in_mV_A, trigger_in_mV_B);
}

double getTimeA(int index) {
	if (index >= DLT_C_WRAPPER::sharedInstance()->m_pulseA->size()
		|| index < 0)
		return 0.0f;

	return static_cast<double>(DLT_C_WRAPPER::sharedInstance()->m_pulseA->at(index).x());
}

double getVoltageA(int index) {
	if (index >= DLT_C_WRAPPER::sharedInstance()->m_pulseA->size()
		|| index < 0)
		return 0.0f;

	return static_cast<double>(DLT_C_WRAPPER::sharedInstance()->m_pulseA->at(index).y());
}

double getTimeB(int index) {
	if (index >= DLT_C_WRAPPER::sharedInstance()->m_pulseB->size()
		|| index < 0)
		return 0.0f;

	return static_cast<double>(DLT_C_WRAPPER::sharedInstance()->m_pulseB->at(index).x());
}

double getVoltageB(int index) {
	if (index >= DLT_C_WRAPPER::sharedInstance()->m_pulseB->size()
		|| index < 0)
		return 0.0f;

	return static_cast<double>(DLT_C_WRAPPER::sharedInstance()->m_pulseB->at(index).y());
}

int getNumberOfCells() {
	return DLT_C_WRAPPER::sharedInstance()->m_setupInfo.numberOfCells;
}

DLifeTime::DLTError getLastError() {
	return DLT_C_WRAPPER::sharedInstance()->m_lastError;
}

void setLifeTime_1(bool lt1_activated, double tau1_in_nanoSeconds, double intensity1, bool lt1_distributionActivated, DLifeTime::DLTDistributionFunction::Function functionType, double param1, int gridNumber, double gridIncrement) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.lt1_activated = lt1_activated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1 = tau1_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.intensity1 = intensity1;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1Distribution.enabled = lt1_distributionActivated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1Distribution.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1Distribution.gridIncrement = gridIncrement;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1Distribution.gridNumber = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau1Distribution.param1 = param1;
}

void setLifeTime_2(bool lt2_activated, double tau2_in_nanoSeconds, double intensity2, bool lt2_distributionActivated, DLifeTime::DLTDistributionFunction::Function functionType, double param1, int gridNumber, double gridIncrement) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.lt2_activated = lt2_activated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2 = tau2_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.intensity2 = intensity2;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2Distribution.enabled = lt2_distributionActivated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2Distribution.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2Distribution.gridIncrement = gridIncrement;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2Distribution.gridNumber = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau2Distribution.param1 = param1;
}

void setLifeTime_3(bool lt3_activated, double tau3_in_nanoSeconds, double intensity3, bool lt3_distributionActivated, DLifeTime::DLTDistributionFunction::Function functionType, double param1, int gridNumber, double gridIncrement) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.lt3_activated = lt3_activated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3 = tau3_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.intensity3 = intensity3;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3Distribution.enabled = lt3_distributionActivated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3Distribution.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3Distribution.gridIncrement = gridIncrement;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3Distribution.gridNumber = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau3Distribution.param1 = param1;
}

void setLifeTime_4(bool lt4_activated, double tau4_in_nanoSeconds, double intensity4, bool lt4_distributionActivated, DLifeTime::DLTDistributionFunction::Function functionType, double param1, int gridNumber, double gridIncrement) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.lt4_activated = lt4_activated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4 = tau4_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.intensity4 = intensity4;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4Distribution.enabled = lt4_distributionActivated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4Distribution.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4Distribution.gridIncrement = gridIncrement;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4Distribution.gridNumber = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau4Distribution.param1 = param1;
}

void setLifeTime_5(bool lt5_activated, double tau5_in_nanoSeconds, double intensity5, bool lt5_distributionActivated, DLifeTime::DLTDistributionFunction::Function functionType, double param1, int gridNumber, double gridIncrement) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.lt5_activated = lt5_activated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5 = tau5_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.intensity5 = intensity5;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5Distribution.enabled = lt5_distributionActivated;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5Distribution.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5Distribution.gridIncrement = gridIncrement;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5Distribution.gridNumber = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.tau5Distribution.param1 = param1;
}

void setPHSFromGaussianModel(bool enabled) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.useGaussianModels = enabled;
}

void setStartOfA(double meanOfStart_A_in_milliVolt, double stddevOfStart_A_in_milliVolt) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.meanOfStartA = meanOfStart_A_in_milliVolt;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.stddevOfStartA = stddevOfStart_A_in_milliVolt;
}

void setStartOfB(double meanOfStart_B_in_milliVolt, double stddevOfStart_B_in_milliVolt) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.meanOfStartB = meanOfStart_B_in_milliVolt;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.stddevOfStartB = stddevOfStart_B_in_milliVolt;
}

void setStopOfA(double meanOfStop_A_in_milliVolt, double stddevOfStop_A_in_milliVolt) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.meanOfStopA = meanOfStop_A_in_milliVolt;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.stddevOfStopA = stddevOfStop_A_in_milliVolt;
}

void setStopOfB(double meanOfStop_B_in_milliVolt, double stddevOfStop_B_in_milliVolt) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.meanOfStopB = meanOfStop_B_in_milliVolt;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.stddevOfStopB = stddevOfStop_B_in_milliVolt;
}

void setPHSDistributionOfA(double resolution, int gridNumber, double distributionStart[], int sizeOfStart, double distributionStop[], int sizeOfStop) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.gridNumberA = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.resolutionMilliVoltPerStepA = resolution;

	std::vector<double> startPHS;

	for (int i = 0; i < sizeOfStart; ++i) 
		startPHS.push_back(distributionStart[i]);

	std::vector<double> stopPHS;

	for (int i = 0; i < sizeOfStop; ++i)
		stopPHS.push_back(distributionStop[i]);
	
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.distributionStartA = startPHS;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.distributionStopA = stopPHS;
}

void setPHSDistributionOfB(double resolution, int gridNumber, double distributionStart[], int sizeOfStart, double distributionStop[], int sizeOfStop) {
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.gridNumberB = gridNumber;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.resolutionMilliVoltPerStepB = resolution;

	std::vector<double> startPHS;

	for (int i = 0; i < sizeOfStart; ++i)
		startPHS.push_back(distributionStart[i]);

	std::vector<double> stopPHS;

	for (int i = 0; i < sizeOfStop; ++i)
		stopPHS.push_back(distributionStop[i]);

	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.distributionStartB = startPHS;
	DLT_C_WRAPPER::sharedInstance()->m_phsDistribution.distributionStopB = stopPHS;
}

void setIRF_PDS_A_1(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf1PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf1PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf1PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf1PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf1PDS.relativeShift = relativeShift;
}

void setIRF_PDS_A_2(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf2PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf2PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf2PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf2PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf2PDS.relativeShift = relativeShift;
}

void setIRF_PDS_A_3(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf3PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf3PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf3PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf3PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf3PDS.relativeShift = relativeShift;
}

void setIRF_PDS_A_4(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf4PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf4PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf4PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf4PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf4PDS.relativeShift = relativeShift;
}

void setIRF_PDS_A_5(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf5PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf5PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf5PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf5PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfA.irf5PDS.relativeShift = relativeShift;
}

void setIRF_PDS_B_1(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf1PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf1PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf1PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf1PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf1PDS.relativeShift = relativeShift;
}

void setIRF_PDS_B_2(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf2PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf2PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf2PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf2PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf2PDS.relativeShift = relativeShift;
}

void setIRF_PDS_B_3(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf3PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf3PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf3PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf3PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf3PDS.relativeShift = relativeShift;
}

void setIRF_PDS_B_4(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf4PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf4PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf4PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf4PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf4PDS.relativeShift = relativeShift;
}

void setIRF_PDS_B_5(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf5PDS.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf5PDS.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf5PDS.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf5PDS.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfB.irf5PDS.relativeShift = relativeShift;
}

void setIRF_MU_1(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf1MU.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf1MU.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf1MU.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf1MU.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf1MU.relativeShift = relativeShift;
}

void setIRF_MU_2(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf2MU.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf2MU.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf2MU.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf2MU.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf2MU.relativeShift = relativeShift;
}

void setIRF_MU_3(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf3MU.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf3MU.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf3MU.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf3MU.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf3MU.relativeShift = relativeShift;
}

void setIRF_MU_4(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf4MU.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf4MU.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf4MU.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf4MU.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf4MU.relativeShift = relativeShift;
}

void setIRF_MU_5(bool enabled, DLifeTime::DLTDistributionFunction::Function functionType, double intensity, double uncertainty, double relativeShift) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf5MU.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf5MU.functionType = functionType;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf5MU.intensity = intensity;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf5MU.uncertainty = uncertainty;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.irfMU.irf5MU.relativeShift = relativeShift;
}

void setATS(double ATS_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.ATS = ATS_in_nanoSeconds;
}

void setSweep(double sweep_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.sweep = sweep_in_nanoSeconds;
}

void setNumberOfCells(int numberOfCells) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.numberOfCells = numberOfCells;
}

void setRiseTimeA(double riseTime_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.riseTime = riseTime_in_nanoSeconds;
}

void setPulseWidthA(double pulseWidth) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.pulseWidth = pulseWidth;
}

void setPulseBaselineOffsetJitterA(bool enabled, double meanOfBaselineJitter_in_mV, double uncertaintyOfBaselineJitter_in_mV) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.baselineOffsetJitterInfoV.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter = meanOfBaselineJitter_in_mV;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter = uncertaintyOfBaselineJitter_in_mV;
}

void setPulseRandomNoiseA(bool enabled, double uncertainty_in_mV) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.randomNoiseInfoV.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.randomNoiseInfoV.rndNoise = uncertainty_in_mV;
}

void setPulseTimeAxisNonlinearityA(bool enabled, double uncertainty_fixedPatternApertureJitter_in_ns, double uncertainty_randomApertureJitter_in_ns) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.fixedPatternApertureJitter = uncertainty_fixedPatternApertureJitter_in_ns;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseA.timeAxisNonLinearityInfoT.rndApertureJitter = uncertainty_randomApertureJitter_in_ns;
}

void setRiseTimeB(double riseTime_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.riseTime = riseTime_in_nanoSeconds;
}

void setPulseWidthB(double pulseWidth) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.pulseWidth = pulseWidth;
}

void setPulseBaselineOffsetJitterB(bool enabled, double meanOfBaselineJitter_in_mV, double uncertaintyOfBaselineJitter_in_mV) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.baselineOffsetJitterInfoV.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.baselineOffsetJitterInfoV.meanOfBaselineOffsetJitter = meanOfBaselineJitter_in_mV;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.baselineOffsetJitterInfoV.stddevOfBaselineOffsetJitter = uncertaintyOfBaselineJitter_in_mV;
}

void setPulseRandomNoiseB(bool enabled, double uncertainty_in_mV) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.randomNoiseInfoV.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.randomNoiseInfoV.rndNoise = uncertainty_in_mV;
}

void setPulseTimeAxisNonlinearityB(bool enabled, double uncertainty_fixedPatternApertureJitter_in_ns, double uncertainty_randomApertureJitter_in_ns) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.fixedPatternApertureJitter = uncertainty_fixedPatternApertureJitter_in_ns;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseB.timeAxisNonLinearityInfoT.rndApertureJitter = uncertainty_randomApertureJitter_in_ns;
}

void setDigitizationInfo(bool enabled, unsigned int digitizationDepth_in_bit) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.digitizationInfo.enabled = enabled;
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.digitizationInfo.digitizationDepth = digitizationDepth_in_bit;
}

void setDelay(double delay_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.delay = delay_in_nanoSeconds;
}

void setAmplitude(double amplitude_in_milliVolt) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.amplitude = amplitude_in_milliVolt;
}

void setUsingPositiveSignalPolarity(bool isPositiveSignalPolarity) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.isPositiveSignalPolarity = isPositiveSignalPolarity;
}

void setStartStopAlternating(bool alternating) {
	DLT_C_WRAPPER::sharedInstance()->m_simulationInput.isStartStopAlternating = alternating;
}


