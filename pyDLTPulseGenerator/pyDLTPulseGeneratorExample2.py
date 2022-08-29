#*************************************************************************************************
#**
#** Copyright (c) 2017-2022 Dr. Danny Petschke. All rights reserved.
#** 
#** Redistribution and use in source and binary forms, with or without modification, 
#** are permitted provided that the following conditions are met:
#**
#** 1. Redistributions of source code must retain the above copyright notice
#**    this list of conditions and the following disclaimer.
#**
#** 2. Redistributions in binary form must reproduce the above copyright notice, 
#**    this list of conditions and the following disclaimer in the documentation 
#**    and/or other materials provided with the distribution.
#**
#** 3. Neither the name of the copyright holder "Danny Petschke" nor the names of its  
#**    contributors may be used to endorse or promote products derived from this software  
#**    without specific prior written permission.
#**
#**
#** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
#** OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
#** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
#** COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
#** EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
#** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
#** TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
#** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#**
#** Contact: danny.petschke@uni-wuerzburg.de
#**
#*************************************************************************************************

PATH_TO_LIBRARY = 'DLTPulseGenerator.dll' 

import pyDLTPulseGenerator as dpg

import numpy as np
import matplotlib.pyplot as plt

numberOfPulses = 100000 # number of pulses to be generated ...

# set trigger levels for branches A & B [mV] ...
    
triggerA = -.1 # [mV]
triggerB = -.1 # [mV]

if __name__ == '__main__':
    dpg.__information__() 
    dpg.__licence__()
    
    # define your simulation input ...
    
    lt          = dpg.DLTSimulationInput()
    setupInfo   = dpg.DLTSetup()
    pulseInfo   = dpg.DLTPulse()
    
    phs         = dpg.DLTPHS()
    
    # import PHS data from file ...
    
    stopPHS,startPHS = np.loadtxt('phs.txt', delimiter='\t', skiprows=2, unpack=True, dtype='float')
    
    # show data for verification ...
    
    fig,ax = plt.subplots()
    
    plt.plot(stopPHS/np.sum(stopPHS),'r-',label="BC422-Q | 22-Na | 511 keV")
    plt.plot(startPHS/np.sum(startPHS),'b-',label="BC422-Q | 22-Na | 1274 keV")
    plt.plot(((startPHS/np.sum(startPHS))+(stopPHS/np.sum(stopPHS))),'g-',label="superimposed PHS input")
    
    plt.legend(loc='best')
    
    ax.set_ylabel('pdf [a.u.]')
    ax.set_xlabel('amplitude distribution [a.u.]')
    
    plt.show()
    
    # modify PHS data structure in order to apply the loaded distributions (PHS) ...
    
    phs.m_useGaussianModels = False # indicate the library that we want to apply our own distribution ...
    
    phs.m_distributionStartA = startPHS
    phs.m_distributionStopA = stopPHS
    phs.m_resolutionMilliVoltPerStepA = float(2/3)*np.abs(pulseInfo.m_amplitude_in_milliVolt)/len(startPHS) # fit PHS into the 2/3 of the max. amplitude ...
    phs.m_gridNumberA = 1024
    
    phs.m_distributionStartB = startPHS
    phs.m_distributionStopB = stopPHS
    phs.m_resolutionMilliVoltPerStepB = float(2/3)*np.abs(pulseInfo.m_amplitude_in_milliVolt)/len(startPHS) # fit PHS into the 2/3 of the max. amplitude ...
    phs.m_gridNumberB = 1024
    
    
    maxAmplitude  = pulseInfo.m_amplitude_in_milliVolt
    sweep_in_ns   = setupInfo.m_sweep_in_nanoSeconds
    
    # initalize pulse generator ...
    
    pulseGen = dpg.DLTPulseGenerator(phs,
                                     lt,
                                     setupInfo,
                                     pulseInfo,
                                     PATH_TO_LIBRARY)
    
    # catch errors ...
    
    if not pulseGen.printErrorDescription():
        quit() # kill process on error ...

    pulseA = dpg.DLTPulseF() # pulse of detector A
    pulseB = dpg.DLTPulseF() # pulse of detector B
    
    pulsesShown = False
    numberPHSBins = 1024 # define a binning for the PHS (DDRS4PALS is using 1024)
    
    phsA = [0]*numberPHSBins
    phsB = [0]*numberPHSBins
    
    for i in range(numberOfPulses):
        # generate pulses ...
        
        if not pulseGen.emitPulses(pulseA,pulseB,triggerA,triggerB):
            continue
        
        # determine phs indices ...
        
        if pulseInfo.m_amplitude_in_milliVolt > 0:
            phsAIndex = (pulseA.getMaximumVoltage()/maxAmplitude)*numberPHSBins
            phsBIndex = (pulseB.getMaximumVoltage()/maxAmplitude)*numberPHSBins
        else:
            phsAIndex = (pulseA.getMinimumVoltage()/(maxAmplitude))*numberPHSBins
            phsBIndex = (pulseB.getMinimumVoltage()/(maxAmplitude))*numberPHSBins

        if phsAIndex <= numberPHSBins-1 and phsAIndex >= 0:
            phsA[int(phsAIndex)] = phsA[int(phsAIndex)] + 1
            
        if phsBIndex <= numberPHSBins-1 and phsBIndex >= 0:
            phsB[int(phsBIndex)] = phsB[int(phsBIndex)] + 1
        
        if not pulsesShown: # show the first pulses ...
            pulsesShown = True
            
            fig,ax = plt.subplots()
            
            plt.plot(pulseA.getTime(),pulseA.getVoltage(),'r-',label="pulse-A")
            plt.plot(pulseB.getTime(),pulseB.getVoltage(),'b-',label="pulse-B")
            
            plt.legend(loc='best')
            
            ax.set_ylabel('amplitude [mV]')
            ax.set_xlabel('time [ns]')
            
            if pulseInfo.m_amplitude_in_milliVolt < 0.:
                ax.set_ylim([maxAmplitude,100.])
            else:
                ax.set_ylim([-100.,maxAmplitude])
                
            ax.set_xlim([0.,sweep_in_ns])
            
            plt.show()
            
    # show the resulting phs ...
    
    fig,ax = plt.subplots()
    
    voltOut   = []
    sumPHSOut  = 0
    normPHSOut = phsA/np.sum(phsA)
    
    for i in range(0,len(phsA)):
        voltOut.append(i*np.abs(maxAmplitude/numberPHSBins))
        sumPHSOut = sumPHSOut + np.abs(maxAmplitude/numberPHSBins)*normPHSOut[i]
    
    plt.plot(voltOut,normPHSOut/sumPHSOut,'rx',label="PHS output")
    
    voltIn    = []
    sumPHSIn  = 0
    normPHSIn = (startPHS+stopPHS)/(np.sum(startPHS) + np.sum(stopPHS))
    
    for i in range(0,len(startPHS)):
        voltIn.append(i*phs.m_resolutionMilliVoltPerStepA)
        sumPHSIn = sumPHSIn + normPHSIn[i]*phs.m_resolutionMilliVoltPerStepA
        
    plt.plot(voltIn,normPHSIn/sumPHSIn,'g-',label="PHS input")
       
    plt.legend(loc='best')
     
    ax.set_ylabel('pdf [a.u.]')
    ax.set_xlabel('amplitude distribution [mV]')
            
    ax.set_xlim([0.,np.abs(pulseInfo.m_amplitude_in_milliVolt)])
    
    plt.show()