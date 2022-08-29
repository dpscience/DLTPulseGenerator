![badge-OS](https://img.shields.io/badge/OS-tested%20under%20Windows%2010-brightgreen)
![badge-OS](https://img.shields.io/badge/OS-tested%20under%20Windows%2011-brightgreen)

Support this project and keep always updated about recent software releases, bug fixes and major improvements by [following on researchgate](https://www.researchgate.net/project/DDRS4PALS-a-software-for-the-acquisition-and-simulation-of-positron-annihilation-lifetime-spectra-PALS-using-the-DRS4-evaluation-board) or [github](https://github.com/dpscience?tab=followers).

[![badge-researchGate](https://img.shields.io/badge/project-researchGate-brightgreen)](https://www.researchgate.net/project/DDRS4PALS-a-software-for-the-acquisition-and-simulation-of-positron-annihilation-lifetime-spectra-PALS-using-the-DRS4-evaluation-board)

![badge-followers](https://img.shields.io/github/followers/dpscience?style=social)
![badge-stars](https://img.shields.io/github/stars/dpscience/DLTPulseGenerator?style=social)
![badge-forks](https://img.shields.io/github/forks/dpscience/DLTPulseGenerator?style=social)

# DLTPulseGenerator

![badge-OS](https://img.shields.io/badge/OS-Windows-blue)
![badge-language](https://img.shields.io/badge/language-C++-blue)
![badge-language](https://img.shields.io/badge/language-Python-blue)
![badge-license](https://img.shields.io/badge/license-BSD-blue)

Copyright (c) 2016-2022 Dr. Danny Petschke (danny.petschke@uni-wuerzburg.de). All rights reserved.<br><br>
<b>DLTPulseGenerator</b> - A library for the simulation of lifetime spectra based on detector-output pulses.

### ``Software Applications using DLTPulseGenerator Library``

<b>[DDRS4PALS](https://github.com/dpscience/DDRS4PALS) software</b> written by Danny Petschke.

# Quickstart Guide

### ``C++ Example Code``

```c++
#define forever                 while(true)
#define DDELETE_SAFETY(__ptr__) { if (__ptr__) { delete __ptr__; __ptr__ = nullptr; } }

int main() {
  printf("let's go ...\n\n");

  /* 1a. define your virtual setup and desired spectrum to be simulated */
  
  DLTSetup setup                     = DLTSetup_DEMO; 
  DLTPulse pulse                     = DLTPulse_DEMO; 
  DLTPHS phs                         = DLTPHS_DEMO; 
  DLTSimulationInput simulationInput = DLTSimulationInput_DEMO; 
  
  /* 1b. set the trigger-levels for both detector branches A and B */
  
  const double triggerA_in_mV = 50.0;
  const double triggerB_in_mV = 50.0;
  
  /* 2. create an instance of DLTPulseGenerator class and pass your setup information and simulation input */
  
  DLTPulseGenerator *pulseGenerator = new DLTPulseGenerator(simulationInput, phs, setup, pulse, nullptr);
  
  /* pulses to be manipulated by calling 'emitPulses(..)' */
  
  DLTPulseF pulseA, pulseB;
  
  forever {
    /* receive the pulses */ 
    
    if (pulseGenerator->emitPulses(&pulseA, &pulseB, triggerA_in_mV, triggerB_in_mV)) {
    
      /* now its your turn ... use the generated pulses for whatever purposes ... for example ... */
      
      /* const double timingA = determineCFTiming(pulseA, level=50.);
	 const double timingB = determineCFTiming(pulseB, level=50.);

	 const double lifetime = calcTimeDifference(timingA, timingB);

	 ... add to the lifetime spectrum ... */
    }
    else
      break;
  }
  
  DDELETE_SAFETY(pulseGenerator)
}
```

### ``Python Example Code``

DLTPulseGenerator can be compiled as linked library enabling access from programming languages such as Python (or Matlab).

A library wrapper for Python ([pyDLTPulseGenerator.py](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator)) demonstrating the usage of [ctypes-library](https://docs.python.org/3/library/ctypes.html) is provided (see [pyDLTPulseGeneratorApp.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGenerator.py)). 

Examples can be found ([here](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator)).

```python
import pyDLTPulseGenerator as dpg # import 'pyDLTPulseGenerator' module
import matplotlib.pyplot as plt

numberOfPulses = 100000 # number of pulses to be generated ...

# set trigger levels for branches A & B [mV] ...
    
triggerA = -25. # [mV]
triggerB = -25. # [mV]

# define your simulation input (or apply the defaults) ...
    
lt          = dpg.DLTSimulationInput()
setupInfo   = dpg.DLTSetup()
pulseInfo   = dpg.DLTPulse()
phs         = dpg.DLTPHS()

# create an instance of DLTPulseGenerator ...
    
pulseGen = dpg.DLTPulseGenerator(phs,
                                 lt,
                                 setupInfo,
                                 pulseInfo,
                                 'dltpulsegenerator.dll')
				 			
# catch errors ...
    
if not pulseGen.printErrorDescription():
    quit() # kill process on error ...

pulseA = dpg.DLTPulseF() # pulse of detector A
pulseB = dpg.DLTPulseF() # pulse of detector B

for i in range(numberOfPulses):    
    if not pulseGen.emitPulses(pulseA,pulseB,triggerA,triggerB): # generate pulse pairs ...
        continue
	
    plt.plot(pulseA.getTime(),pulseA.getVoltage(),'r-',label="pulse-A")
    plt.plot(pulseB.getTime(),pulseB.getVoltage(),'b-',label="pulse-B")
    plt.show()
    
    // ...

```

# Related Presentation & Publications

### ``DLTPulseGenerator v1.0``

<b>DLTPulseGenerator v1.0</b><br>[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4630108.svg)](https://doi.org/10.5281/zenodo.7023157)<br>
[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.04.002-yellowgreen)](https://doi.org/10.1016/j.softx.2018.04.002)

This <b>[release v1.0](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.0)</b> refers to the <b>original paper by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b> 

[DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.04.002)<br>

and provides the simulation of lifetime spectra consisting of <i>discrete characteristic lifetimes</i> based on detector-output pulses modelled by a log-normal distribution function.

![SoftX_1_0](/images/softxPub.png)

### ``DLTPulseGenerator v1.1``

<b>DLTPulseGenerator v1.1</b><br>[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4630108.svg)](https://doi.org/10.5281/zenodo.7023157)<br>
[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.05.001-yellowgreen)](https://doi.org/10.1016/j.softx.2018.05.001)

This <b>[release v1.1](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.1)</b> refers to the <b>update paper (v1.1) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b> 

[Update (v1.1) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.05.001)<br>

and provides the simulation of lifetime spectra consisting of <i>distributed characteristic lifetimes</i> as can be found in porous materials such as polymers or glasses due to distributions in their pore-sizes.

![SoftX_1_1](/images/softxPub_1_1.png)

### ``DLTPulseGenerator v1.2``

<b>DLTPulseGenerator v1.2</b><br>[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4630108.svg)](https://doi.org/10.5281/zenodo.7023157)<br>
[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.06.003-yellowgreen)](https://doi.org/10.1016/j.softx.2018.06.003)

This <b>[release v1.2](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.2)</b> refers to the <b>update paper (v1.2) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b>

[Update (v1.2) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.06.003)<br>

and enables the simulation of lifetime spectra consisting of non-Gaussian or any-distributed and linearly combined Instrument Response Functions (IRF) for the PDS A/B and MU.

![SoftX_1_2](/images/softxPub_1_2.png)

### ``DLTPulseGenerator v1.3``

<b>DLTPulseGenerator v1.3</b><br>[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4630108.svg)](https://doi.org/10.5281/zenodo.7023157)<br>
[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2019.02.003-yellowgreen)](https://doi.org/10.1016/j.softx.2019.02.003)

This <b>[release v1.3](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.3)</b> refers to the <b>update paper (v1.3) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2019)</b>

[Update (v1.3) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2019.02.003)<br>

and provides the simulation of realistic hardware influences mainly originating from the parts of the A/D converter such as baseline-offset jitter, random noise, fixed pattern and random aperture jitters on the time axis or the digitization depth.

![SoftX_1_3](/images/softxPub_1_3.png)

### ``DLTPulseGenerator v1.4``

<b>DLTPulseGenerator v1.4</b><br>[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4630108.svg)](https://doi.org/10.5281/zenodo.7023157)<br>
This <b>[release v1.4](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.4)</b> enables the incorporation of pulse height spectra (PHS) from real data or
generated by Geant4, so that effects on the background contributing to the lifetime spectra with regard to the PHS window settings can be studied more realistically.

### ``Presentation at 18th International Conference on Positron Annihilation (ICPA-18) in Orlando (Aug. 2018)``

[ICPA-18 (Orlando): DLTPulseGenerator - A C/C++ library for the simulation of lifetime spectra based on detector output-pulses](https://www.researchgate.net/publication/329782711_ICPA-18_Orlando_DLTPulseGenerator_-_A_CC_library_for_the_simulation_of_lifetime_spectra_based_on_detector_output-pulses)<br>

# How to cite this Library?

* <b>You should at least cite the following publication.</b><br>

[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.04.002-yellowgreen)](https://doi.org/10.1016/j.softx.2018.04.002)

[DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses (SoftwareX 2018, Elsevier)](https://doi.org/10.1016/j.softx.2018.04.002)<br><br>

* <b>Additionally, you should cite the version of DLTPulseGenerator library used in your study.</b><br>

You can cite all released software versions by using the <b>DOI 10.5281/zenodo.3665474</b>. This DOI represents all versions, and will always resolve to the latest one.<br>

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3665474.svg)](https://doi.org/10.5281/zenodo.3665475)

# License of DLTPulseGenerator (BSD-3-Clause)

Copyright (c) 2016-2022 Dr. Danny Petschke (danny.petschke@uni-wuerzburg.de). All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification,<br> 
are permitted provided that the following conditions are met:<br><br>

 1. Redistributions of source code must retain the above copyright notice<br>
    this list of conditions and the following disclaimer.<br><br>

 2. Redistributions in binary form must reproduce the above copyright notice,<br> 
    this list of conditions and the following disclaimer in the documentation<br> 
    and/or other materials provided with the distribution.<br><br>

 3. Neither the name of the copyright holder "Danny Petschke" nor the names of<br> 
    its contributors may be used to endorse or promote products derived from <br>
    this software without specific prior written permission.<br><br>


 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS<br> 
 OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF<br> 
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE<br> 
 COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,<br> 
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF<br> 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)<br> 
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR<br> 
 TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,<br> 
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.<br>
 
 see also [BSD-3-Clause License](https://opensource.org/licenses/BSD-3-Clause)
