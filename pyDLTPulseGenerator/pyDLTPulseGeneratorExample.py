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
    
triggerA = -25 # [mV]
triggerB = -25 # [mV]

if __name__ == '__main__':
    dpg.__information__() 
    dpg.__licence__()
    
    # define your simulation input ...
    
    lt          = dpg.DLTSimulationInput()
    phs         = dpg.DLTPHS()
    setupInfo   = dpg.DLTSetup()
    pulseInfo   = dpg.DLTPulse()
    
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
    
    plt.plot(phsA,'r-',label="PHS-A")
    plt.plot(phsB,'b-',label="PHS-B")
    
    plt.legend(loc='best')
     
    ax.set_ylabel('propability [a.u.]')
    ax.set_xlabel('amplitude distribution [{} mV]'.format(np.abs(maxAmplitude/numberPHSBins)))
            
    ax.set_xlim([0.,numberPHSBins-1])
            
    plt.show()