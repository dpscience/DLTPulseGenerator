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

class DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate {
    friend class DLifeTime::DLTPulseGenerator;

    //DLTPHS:
    default_random_engine m_generatorStartA;
    default_random_engine m_generatorStopA;

    default_random_engine m_generatorStartB;
    default_random_engine m_generatorStopB;

    normal_distribution<double> m_distributionStartA;
    normal_distribution<double> m_distributionStartB;

    normal_distribution<double> m_distributionStopA;
    normal_distribution<double> m_distributionStopB;

    //DLTSetup:
    default_random_engine m_generatorUncertaintyPDSDetectorA;
    normal_distribution<double> m_distributionUncertaintyPDSDetectorA;

    default_random_engine m_generatorUncertaintyPDSDetectorB;
    normal_distribution<double> m_distributionUncertaintyPDSDetectorB;

    default_random_engine m_generatorUncertaintyMU;
    normal_distribution<double> m_distributionUncertaintyMU;

    default_random_engine m_generatorBackground;
    piecewise_constant_distribution<double>m_distributionBackground;

	//DLTSimulationInput:
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

	//lifetime-distribution grid:
	DLTDistributionManager m_lt1Manager;
	DLTDistributionManager m_lt2Manager;
	DLTDistributionManager m_lt3Manager;
	DLTDistributionManager m_lt4Manager;
	DLTDistributionManager m_lt5Manager;

	//LTSelector:
	default_random_engine m_generatorLTSelector;
	piecewise_constant_distribution<double> m_distributionLTSelector;

	DLifeTime::DLTPulseGenerator::DLTPulseGeneratorPrivate() {
		m_distributionLT1 = nullptr;
		m_distributionLT2 = nullptr;
		m_distributionLT3 = nullptr;
		m_distributionLT4 = nullptr;
		m_distributionLT5 = nullptr;
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
	m_sampleScaleFactor = 15000.0f / 1024.0f; //(15000/1024)=(x/m_setupInfo.numberOfCells)

	DLifeTime::DLTError error = NONE_ERROR;

    if ( m_setupInfo.sweep <= 10 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::SWEEP_INVALID;
    }

    if ( m_setupInfo.numberOfCells <= 10 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::NUMBER_OF_CELLS_INVALID;
    }

    if ( m_setupInfo.PDSUncertaintyA < 0
         || m_setupInfo.PDSUncertaintyB < 0 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::PDS_UNCERTAINTY_INVALID;
    }

    if ( m_setupInfo.MUUncertainty < 0 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::MU_UNCERTAINTY_INVALID;
    }

    if ( m_pulseInfo.riseTime <= 0 ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::PULSE_RISE_TIME_INVALID;
    }

    if ( m_pulseInfo.pulseWidth <= 0 ) {
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

    if ( m_simulationInput.intensityOfPromtOccurrance < 0 ) {
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

    if ( (m_pulseInfo.isPositiveSignalPolarity
         && !( m_phsDistribution.meanOfStartA > 0
               && m_phsDistribution.meanOfStartB > 0
               && m_phsDistribution.meanOfStopA > 0
               && m_phsDistribution.meanOfStopB > 0
               && m_phsDistribution.stddevOfStartA > 0
               && m_phsDistribution.stddevOfStartB > 0
               && m_phsDistribution.stddevOfStopA > 0
               && m_phsDistribution.stddevOfStopB > 0))
         || (!m_pulseInfo.isPositiveSignalPolarity
             && !( m_phsDistribution.meanOfStartA < 0
                   && m_phsDistribution.meanOfStartB < 0
                   && m_phsDistribution.meanOfStopA < 0
                   && m_phsDistribution.meanOfStopB < 0
                   && m_phsDistribution.stddevOfStartA < 0
                   && m_phsDistribution.stddevOfStartB < 0
                   && m_phsDistribution.stddevOfStopA < 0
                   && m_phsDistribution.stddevOfStopB < 0)) ) {
        if ( callback ) 
			error |= DLifeTime::DLTErrorType::AMPLITUDE_AND_PHS_MISFIT;
    }

	//init event-counter:
    m_eventCounter = 0;

    //Init of DLTPHS related random-generators:
	m_privatePtr.get()->m_generatorStartA.seed(rand());
	m_privatePtr.get()->m_generatorStartB.seed(rand());

	m_privatePtr.get()->m_generatorStopA.seed(rand());
    m_privatePtr.get()->m_generatorStopB.seed(rand());

    m_privatePtr.get()->m_distributionStartA = normal_distribution<double>(m_phsDistribution.meanOfStartA, m_phsDistribution.stddevOfStartA);
    m_privatePtr.get()->m_distributionStartB = normal_distribution<double>(m_phsDistribution.meanOfStartB, m_phsDistribution.stddevOfStartB);

    m_privatePtr.get()->m_distributionStopA  = normal_distribution<double>(m_phsDistribution.meanOfStopA, m_phsDistribution.stddevOfStopA);
    m_privatePtr.get()->m_distributionStopB  = normal_distribution<double>(m_phsDistribution.meanOfStopB, m_phsDistribution.stddevOfStopB);

	//init of DLTSimulationInput related random-generators:
	initLTGenerator(&error, callback);
    
    //init of DLTSetup (PDS and MU) related random-generators:
    m_privatePtr.get()->m_generatorUncertaintyPDSDetectorA.seed(rand());
    m_privatePtr.get()->m_generatorUncertaintyPDSDetectorB.seed(rand());
    m_privatePtr.get()->m_generatorUncertaintyMU.seed(rand());

    m_privatePtr.get()->m_distributionUncertaintyPDSDetectorA = normal_distribution<double>(0.0f, m_setupInfo.PDSUncertaintyA);
    m_privatePtr.get()->m_distributionUncertaintyPDSDetectorB = normal_distribution<double>(0.0f, m_setupInfo.PDSUncertaintyB);
    m_privatePtr.get()->m_distributionUncertaintyMU			  = normal_distribution<double>(0.0f, m_setupInfo.MUUncertainty);

	const double branchA = sqrt(m_setupInfo.PDSUncertaintyA*m_setupInfo.PDSUncertaintyA + m_setupInfo.MUUncertainty*m_setupInfo.MUUncertainty);
	const double branchB = sqrt(m_setupInfo.PDSUncertaintyB*m_setupInfo.PDSUncertaintyB + m_setupInfo.MUUncertainty*m_setupInfo.MUUncertainty);
	double estimatedFWHM = sqrt(branchA*branchA + branchB*branchB)*2*sqrt(2*log(2));
	
	//generate background 12x of x(FWHM) on the t0-left side:
	estimatedFWHM *= 12;

	const int leftSideBin = (estimatedFWHM / m_setupInfo.sweep)*((double)m_setupInfo.numberOfCells*m_sampleScaleFactor);
	const int backgroundBinCount = ((double)m_setupInfo.numberOfCells)*m_sampleScaleFactor; 
	
	const int startBin = -leftSideBin; //left of t0
	const int stopBin  = backgroundBinCount; //right of t0 -> sweep

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
    const double wPro = m_simulationInput.intensityOfPromtOccurrance*100.0f;		// [%]
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

    m_privatePtr.release();
}

bool DLifeTime::DLTPulseGenerator::emitPulses(DLifeTime::DLTPulseF *pulseA,
	DLifeTime::DLTPulseF *pulseB,
	double triggerLevelA_in_milliVolt, 
	double triggerLevelB_in_milliVolt) {
    if ( !pulseA || !pulseB )
        return false;

    if ( m_eventCounter > INT_MAX/2 )
        m_eventCounter = 0;

	if (!m_simulationInput.isStartStopAlternating)
		m_eventCounter = 1;

    const double deeperSampleDepthFactor = ((double)m_setupInfo.numberOfCells*m_sampleScaleFactor); //(15000/1024)=(x/m_setupInfo.numberOfCells)
    const double deeperSampleDepth		 = ((double)m_setupInfo.numberOfCells*deeperSampleDepthFactor);

    bool isCoincidence = false;
	bool validLifetime = true;
    const double nextLT = nextLifeTime(&isCoincidence, &validLifetime); //next lifetime? (LTSelector)

	if (!validLifetime)
		return false;

    if ( m_eventCounter%2 ) { //A (Start)-B (Stop) 
        const double amplitudeInMVA = (!isCoincidence)?m_privatePtr.get()->m_distributionStartA(m_privatePtr.get()->m_generatorStartA):m_privatePtr.get()->m_distributionStopA(m_privatePtr.get()->m_generatorStopA);
        const double amplitudeInMVB = m_privatePtr.get()->m_distributionStopB(m_privatePtr.get()->m_generatorStopB);

		const double start_t_in_ns = m_pulseInfo.delay + m_privatePtr.get()->m_distributionUncertaintyPDSDetectorA(m_privatePtr.get()->m_generatorUncertaintyPDSDetectorA) + m_privatePtr.get()->m_distributionUncertaintyMU(m_privatePtr.get()->m_generatorUncertaintyMU);
        const double stop_t_in_ns  = nextLT - m_setupInfo.ATS + m_privatePtr.get()->m_distributionUncertaintyPDSDetectorB(m_privatePtr.get()->m_generatorUncertaintyPDSDetectorB) + m_privatePtr.get()->m_distributionUncertaintyMU(m_privatePtr.get()->m_generatorUncertaintyMU) + m_pulseInfo.delay;

        const int startCell = (start_t_in_ns/m_setupInfo.sweep)*deeperSampleDepth;
        const int stopCell  = (stop_t_in_ns/ m_setupInfo.sweep)*deeperSampleDepth;

        const double timeIncrInNS = m_setupInfo.sweep/deeperSampleDepth;

        //trigger OK? (AND-logic only)
        if ( m_pulseInfo.isPositiveSignalPolarity ) {
            if ( !(triggerLevelA_in_milliVolt < amplitudeInMVA && triggerLevelA_in_milliVolt > 0.0f
                   && triggerLevelB_in_milliVolt < amplitudeInMVB && triggerLevelB_in_milliVolt > 0.0f) )
                return false;
        }
        else {
            if ( !(triggerLevelA_in_milliVolt > amplitudeInMVA && triggerLevelA_in_milliVolt < 0.0f
                   && triggerLevelB_in_milliVolt > amplitudeInMVB && triggerLevelB_in_milliVolt < 0.0f) )
                return false;
        }

        for ( int cell = 0 ; cell < deeperSampleDepth ; cell += deeperSampleDepthFactor ) {
            DLTPointF pA, pB;

            pA.setX(cell*timeIncrInNS);
            pB.setX(cell*timeIncrInNS);

            if ( cell <= startCell )
                pA.setY(0.0f);
            else {
                const double lnVal = log(((double)(cell-startCell)*timeIncrInNS)/m_pulseInfo.riseTime)/m_pulseInfo.pulseWidth;

                double ampl = amplitudeInMVA*exp(-0.5f*(lnVal)*(lnVal));

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl < 0.0f )
                        ampl = 0.0f;
                    else if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl > 0.0f )
                        ampl = 0.0f;
                    else if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

                pA.setY(ampl);
            }

            if ( cell <= stopCell )
                pB.setY(0.0f);
            else {
                const double lnVal = log(((double)(cell-stopCell)*timeIncrInNS)/m_pulseInfo.riseTime)/m_pulseInfo.pulseWidth;

                double ampl = amplitudeInMVB*exp(-0.5f*(lnVal)*(lnVal));

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl < 0.0f )
                        ampl = 0.0f;
                    else if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl > 0.0f )
                        ampl = 0.0f;
                    else if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

                pB.setY(ampl);
            }

            pulseA->append(pA);
            pulseB->append(pB);
        }
    }
    else { //B (Start)-A (Stop)
        const double amplitudeInMVB = (!isCoincidence)?m_privatePtr.get()->m_distributionStartB(m_privatePtr.get()->m_generatorStartB):m_privatePtr.get()->m_distributionStopB(m_privatePtr.get()->m_generatorStopB);
        const double amplitudeInMVA = m_privatePtr.get()->m_distributionStopA(m_privatePtr.get()->m_generatorStopA);

        const double start_t_in_ns = m_pulseInfo.delay + m_privatePtr.get()->m_distributionUncertaintyPDSDetectorB(m_privatePtr.get()->m_generatorUncertaintyPDSDetectorB) + m_privatePtr.get()->m_distributionUncertaintyMU(m_privatePtr.get()->m_generatorUncertaintyMU);
        const double stop_t_in_ns  = nextLT + m_setupInfo.ATS + m_privatePtr.get()->m_distributionUncertaintyPDSDetectorA(m_privatePtr.get()->m_generatorUncertaintyPDSDetectorA) + m_privatePtr.get()->m_distributionUncertaintyMU(m_privatePtr.get()->m_generatorUncertaintyMU) + m_pulseInfo.delay;

        const int startCell = (start_t_in_ns/m_setupInfo.sweep)*deeperSampleDepth;
        const int stopCell  = (stop_t_in_ns/m_setupInfo.sweep)*deeperSampleDepth;

        const double timeIncrInNS = m_setupInfo.sweep/deeperSampleDepth;

        //trigger OK? (AND-logic only)
        if ( m_pulseInfo.isPositiveSignalPolarity ) {
            if ( !(triggerLevelA_in_milliVolt < amplitudeInMVA && triggerLevelA_in_milliVolt > 0.0f
                   && triggerLevelB_in_milliVolt < amplitudeInMVB && triggerLevelB_in_milliVolt > 0.0f) )
                return false;
        }
        else {
            if ( !(triggerLevelA_in_milliVolt > amplitudeInMVA && triggerLevelA_in_milliVolt < 0.0f
                   && triggerLevelB_in_milliVolt > amplitudeInMVB && triggerLevelB_in_milliVolt < 0.0f) )
                return false;
        }

        for ( int cell = 0 ; cell < deeperSampleDepth ; cell += deeperSampleDepthFactor ) {
            DLTPointF pA, pB;

            pA.setX(cell*timeIncrInNS);
            pB.setX(cell*timeIncrInNS);

            if ( cell <= startCell )
                pB.setY(0.0f);
            else {
                const double lnVal = log(((double)(cell-startCell)*timeIncrInNS)/m_pulseInfo.riseTime)/m_pulseInfo.pulseWidth;

                double ampl = amplitudeInMVB*exp(-0.5f*(lnVal)*(lnVal));

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl < 0.0f )
                        ampl = 0.0f;
                    else if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl > 0.0f )
                        ampl = 0.0f;
                    else if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }

                pB.setY(ampl);
            }

            if ( cell <= stopCell )
                pA.setY(0.0f);
            else {
                const double lnVal = log(((double)(cell-stopCell)*timeIncrInNS)/m_pulseInfo.riseTime)/m_pulseInfo.pulseWidth;

                double ampl = amplitudeInMVA*exp(-0.5f*(lnVal)*(lnVal));

                if ( m_pulseInfo.isPositiveSignalPolarity ) {
                    if ( ampl < 0.0f )
                        ampl = 0.0f;
                    else if ( ampl > m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
                }
                else {
                    if ( ampl > 0.0f )
                        ampl = 0.0f;
                    else if ( ampl < m_pulseInfo.amplitude )
                        ampl = m_pulseInfo.amplitude;
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

void DLifeTime::DLTPulseGenerator::initLTGenerator(DLifeTime::DLTError *error, DLifeTime::DLTCallback *callback)
{
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

/** The class DLT_C_WRAPPER is used as singleton-pattern class to access from Ansi C-functions:
**	This provides the access to other programming languages such as matlab (mex-compiler) or python (ctypes-library).
**
**	An example how to access the class DLTPulseGenerator via class DLT_C_WRAPPER is given in python:
**
**  - pyDLTPulseGenerator -
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

	if (__sharedAccessPtr) {
		delete __sharedAccessPtr;
		__sharedAccessPtr = nullptr;
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

void setUncertaintyOfPDSDetectors(double uncertaintyPDS_detector_A_in_nanoSeconds, double uncertaintyPDS_detector_B_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.PDSUncertaintyA = uncertaintyPDS_detector_A_in_nanoSeconds;
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.PDSUncertaintyB = uncertaintyPDS_detector_B_in_nanoSeconds;
}

void setUncertaintyOfMU(double uncertaintyMU_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_setupInfo.MUUncertainty = uncertaintyMU_in_nanoSeconds;
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

void setRiseTime(double riseTime_in_nanoSeconds) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.riseTime = riseTime_in_nanoSeconds;
}

void setPulseWidth(double pulseWidth) {
	DLT_C_WRAPPER::sharedInstance()->m_pulseInfo.pulseWidth = pulseWidth;
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


