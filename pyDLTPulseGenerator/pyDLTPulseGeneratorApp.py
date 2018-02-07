#*************************************************************************************************
#**
#** Copyright (c) 2017 Danny Petschke. All rights reserved.
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

PATH_TO_LIBRARY = '.../DLTPulseGenerator.dll' #!full path + filename required!

import pyDLTPulseGenerator as dpg

import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#information on copyright and licence!
dpg.__information__() 
dpg.__licence__()

#setup your Trigger-Value [mV]:
triggerA = 50 #[mV]
triggerB = 50 #[mV]

#define input:
lt          = dpg.DLTSimulationInput()
phs         = dpg.DLTPHS()
setupInfo   = dpg.DLTSetup()
pulseInfo   = dpg.DLTPulse()

#some necessary values for data-visualization:
amplitudeA    = pulseInfo.m_amplitude_in_milliVolt
amplitudeB    = pulseInfo.m_amplitude_in_milliVolt
maxAmplitude  = pulseInfo.m_amplitude_in_milliVolt

sweep_in_ns   = setupInfo.m_sweep_in_nanoSeconds
numberOfCells = setupInfo.m_numberOfCells;

#initalize the Pulse-Generator (Note: The Pulse-Generator will only run in one Instance!!!) :
pulseGen    = dpg.DLTPulseGenerator(phs,
                                    lt,
                                    setupInfo,
                                    pulseInfo,
                                    PATH_TO_LIBRARY)

# enum DLTPULSEGENERATOR_EXPORT DLTErrorType : DLTError
    #NONE_ERROR                          = 0x00000000,
    #NO_LIFETIMES_TO_SIMULATE            = 0x00000001,
    #SWEEP_INVALID                       = 0x00000002,
    #NUMBER_OF_CELLS_INVALID             = 0x00000004,
    #PDS_UNCERTAINTY_INVALID             = 0x00000008,
    #MU_UNCERTAINTY_INVALID              = 0x00000010,
    #PULSE_RISE_TIME_INVALID             = 0x00000020,
    #PULSE_WIDTH_INVALID                 = 0x00000040,
    #DELAY_INVALID                       = 0x00000080,
    #DELAY_LARGER_THAN_SWEEP             = 0x00000100,
    #INTENSITY_OF_LIFETIME_BELOW_ZERO    = 0x00000200,
    #INTENSITY_OF_BKGRD_BELOW_ZERO       = 0x00000400,
    #INTENSITY_OF_PROMT_BELOW_ZERO       = 0x00000800,
    #INVALID_SUM_OF_WEIGTHS              = 0x00001000,
    #AMPLITUDE_AND_PULSE_POLARITY_MISFIT = 0x00002000,
    #AMPLITUDE_AND_PHS_MISFIT            = 0x00004000,
#};

print("DLTPulseGenerator initialized with error-code:");

if pulseGen.getError() != 0:
    if pulseGen.getError() & 1:
        print("- no lifetimes enabled.");

    if pulseGen.getError() & 2:
        print("- invalid sweep.");

    if pulseGen.getError() & 4:
        print("- invalid number of cells.")

    if pulseGen.getError() & 8:
        print("- invalid PDS uncertainty.");

    if pulseGen.getError() & 16:
        print("- invalid MU uncertainty.");

    if pulseGen.getError() & 32:
        print("- invalid pulse rise time.");

    if pulseGen.getError() & 64:
        print("- invalid pulse width.");

    if pulseGen.getError() & 128:
        print("- invalid delay.");

    if pulseGen.getError() & 256:
        print("- delay > sweep.");

    if pulseGen.getError() & 512:
        print("- negative lifetimes detected.");

    if pulseGen.getError() & 1024:
        print("- negative background detected.");

    if pulseGen.getError() & 2048:
        print("- negative promt detected.");

    if pulseGen.getError() & 5096:
        print("- invalid sum of weights.");

    if pulseGen.getError() & 10192:
        print("- amplitude and pulse polarity misfit.");

    if pulseGen.getError() & 20384:
        print("- amplitude and phs misfit.");

    quit(); #kill process on error!
else:
    print("- succeess.\n");


#plot the pulses:
fig = plt.figure()
axe = fig.add_subplot(1, 1, 1)

axes = plt.gca()

axes.yaxis.set_label("amplitude [mV]")
axes.xaxis.set_label("sweep [ns]")

#plot the phs:
figPHS = plt.figure()
axePHS = figPHS.add_subplot(1, 1, 1)

axesPHS = plt.gca()

phsA = [0]*numberOfCells
phsB = [0]*numberOfCells

phsA_x = [0]*numberOfCells
phsB_x = [0]*numberOfCells

for i in range(0, (numberOfCells-1)):
    phsA_x[i] = (float(i)/float((numberOfCells-1)))*maxAmplitude
    phsB_x[i] = (float(i)/float((numberOfCells-1)))*maxAmplitude
    
pulseA = dpg.DLTPulseF() #pulse of detector A
pulseB = dpg.DLTPulseF() #pulse of detector B

def updatePlotView(i):
    #The content of the Pulses will be manipulated by the function(except: Trigger-Levels: 'triggerA' & 'triggerB':
    if pulseGen.emitPulses(pulseA, pulseB, triggerA, triggerB):
        
        axe.clear()
        axe.plot(pulseA.getTime(), pulseA.getVoltage(), 'r--', pulseB.getTime(), pulseB.getVoltage(), 'b--')

        axes.set_xlim([0, sweep_in_ns])
        axes.set_ylim([-amplitudeA, amplitudeA])

        
        phsAIndex = (pulseA.getMaximumVoltage()/maxAmplitude)*numberOfCells
        phsBIndex = (pulseB.getMaximumVoltage()/maxAmplitude)*numberOfCells

        if (phsAIndex <= (numberOfCells-1) and phsAIndex >= 0) and (phsBIndex <= (numberOfCells-1) and phsBIndex >= 0):
            phsA[int(phsAIndex)] = phsA[int(phsAIndex)] + 1
            phsB[int(phsBIndex)] += 1
      
        axePHS.clear()
        
        axePHS.plot(phsA_x, phsA, 'r--')
        axePHS.plot(phsB_x, phsB, 'b--')
        
        axesPHS.set_xlim([0, maxAmplitude])
        axesPHS.set_ylim([0, max(max(phsA), max(phsB))])
    #else:
        #This happens only if the PHS-Value is lower than the Trigger-Value at the Channel:

    
a = animation.FuncAnimation(fig, updatePlotView, frames = 100, repeat = True)
b = animation.FuncAnimation(figPHS, updatePlotView, frames = 100, repeat = True)

plt.show()
