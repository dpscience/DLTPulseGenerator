![badge-OS](https://img.shields.io/badge/OS-tested%20under%20Windows%2010-brightgreen)

Support this project and keep always updated about recent software releases, bug fixes and major improvements by [following on researchgate](https://www.researchgate.net/project/DDRS4PALS-a-software-for-the-acquisition-and-simulation-of-positron-annihilation-lifetime-spectra-PALS-using-the-DRS4-evaluation-board) or [github](https://github.com/dpscience?tab=followers).

[![badge-researchGate](https://img.shields.io/badge/project-researchGate-brightgreen)](https://www.researchgate.net/project/DDRS4PALS-a-software-for-the-acquisition-and-simulation-of-positron-annihilation-lifetime-spectra-PALS-using-the-DRS4-evaluation-board)

![badge-followers](https://img.shields.io/github/followers/dpscience?style=social)
![badge-stars](https://img.shields.io/github/stars/dpscience/DLTPulseGenerator?style=social)
![badge-forks](https://img.shields.io/github/forks/dpscience/DLTPulseGenerator?style=social)

# DLTPulseGenerator

![badge-OS](https://img.shields.io/badge/OS-Windows-blue)
![badge-language](https://img.shields.io/badge/language-C++-blue)
![badge-license](https://img.shields.io/badge/license-BSD-blue)

Copyright (c) 2016-2019 Danny Petschke (danny.petschke@uni-wuerzburg.de). All rights reserved.<br><br>
<b>DLTPulseGenerator</b> - A library for the simulation of lifetime spectra based on detector-output pulses

# Related Presentation & Publications

### ``DLTPulseGenerator v1.0``

[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.04.002-yellowgreen)](https://doi.org/10.1016/j.softx.2018.04.002)

This <b>[release v1.0](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.0)</b> relates to the <b>original paper by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b> 

[DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.04.002)<br>

and provides the simulation of lifetime spectra consisting of <i>discrete characteristic lifetimes</i> based on detector-output pulses modelled by a log-normal distribution function.

### ``DLTPulseGenerator v1.1``

[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.05.001-yellowgreen)](https://doi.org/10.1016/j.softx.2018.05.001)

This <b>[release v1.1](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.1)</b> relates to the <b>update paper (v1.1) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b> 

[Update (v1.1) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.05.001)<br>

and provides the simulation of lifetime spectra consisting of <i>distributed characteristic lifetimes</i> as can be found in porous materials such as polymers or glasses due to distributions in their pore-sizes.

### ``DLTPulseGenerator v1.2``

[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2018.06.003-yellowgreen)](https://doi.org/10.1016/j.softx.2018.06.003)

This <b>[release v1.2](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.2)</b> relates to the <b>update paper (v1.2) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2018)</b>

[Update (v1.2) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2018.06.003)<br>

and enables the simulation of lifetime spectra consisting of non-Gaussian or any-distributed and linearly combined Instrument Response Functions (IRF) for the PDS A/B and MU.

### ``DLTPulseGenerator v1.3``

[![DOI](https://img.shields.io/badge/DOI-10.1016%2Fj.softx.2019.02.003-yellowgreen)](https://doi.org/10.1016/j.softx.2019.02.003)

This <b>[release v1.3](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.3)</b> relates to the <b>update paper (v1.3) by Petschke <i>et al.</i> published in SoftwareX (Elsevier, 2019)</b>

[Update (v1.3) to DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses](https://doi.org/10.1016/j.softx.2019.02.003)<br>

and provides the simulation of realistic hardware influences mainly originating from the parts of the A/D converter such as baseline-offset jitter, random noise, fixed pattern and random aperture jitters on the time axis or the digitization depth.

# How to cite this Library?

<b>You should at least cite the following publication:</b><br><br>
[DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses (SoftwareX 2018, Elsevier)](https://doi.org/10.1016/j.softx.2018.04.002)<br><br>



## Software Applications using DLTPulseGenerator library

<b>[DDRS4PALS](https://github.com/dpscience/DDRS4PALS) software</b> written by Danny Petschke

# Example using C++

```c++
#define forever                 while(true)
#define DDELETE_SAFETY(__ptr__) { if (__ptr__) { delete __ptr__; __ptr__ = nullptr; } }

int main() {
  printf("How to easily implement DLTPulseGenerator library?\n\n");

  /* 1a. Define structs: */
  DLTSetup setup                     = DLTSetup_DEMO; 
  DLTPulse pulse                     = DLTPulse_DEMO; 
  DLTPHS phs                         = DLTPHS_DEMO; 
  DLTSimulationInput simulationInput = DLTSimulationInput_DEMO; 
  
  /* 1b. Set trigger-levels for branch A and B: */
  const double triggerA_in_mV = 50.0;
  const double triggerB_in_mV = 50.0;
  
  /* 2. Initialize DLTPulseGenerator class: */
  DLTPulseGenerator *pulseGenerator = new DLTPulseGenerator(simulationInput, phs, setup, pulse, nullptr);
  
  /* 3. Receive pulses: */
  DLTPulseF pulseA, pulseB;
  
  forever {
    if ( pulseGenerator->emitPulses(&pulseA, &pulseB, triggerA_in_mV, triggerB_in_mV) ) {
      /* all algorithms for exact timing determination and lifetime calculation, respectively, have to be placed here! */
      /* const double timingA = CFD(pulseA, level);
	 const double timingB = CFD(pulseB, level);

	 const double lifetime = calcDifference(timingA, timingB);

	 -> binning the lifetimes (MCA). */
    }
    else
      break;
  }
  
  DDELETE_SAFETY(pulseGenerator)
}
```
Errors can be handled by inheriting <i>class DLTPulseGenerator</i> using the provided callback function: see [DLTPulseGeneratorApp.h/.cpp](https://github.com/dpscience/DLTPulseGenerator/blob/master/DLTPulseGenerator/example/AppDLTPulseGenerator/AppDLTPulseGenerator/DLTPulseGeneratorApp.h).  

# Example using Python

A <b>library wrapper</b> in <i>python</i> ([pyDLTPulseGenerator.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGenerator.py)) demonstrating the usage of [ctypes-library](https://docs.python.org/3/library/ctypes.html) by calling the functions of <b>DLTPulseGenerator.dll</b> ([x86](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator/x86)/[x64](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator/x64)) is provided. [pyDLTPulseGeneratorApp.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGeneratorApp.py) calls functions from the library (wrapper) [pyDLTPulseGenerator.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGenerator.py). The generated pulse pairs and the pulse height spectrum are displayed for demonstration purposes.<br>

#### Requirements:

- [ctypes](https://docs.python.org/3/library/ctypes.html) 
- [NumPy](http://www.numpy.org/) 
- [matplotlib](https://matplotlib.org/)<br>

#### [WinPython](https://sourceforge.net/projects/winpython/) meets all requirements. 

![Generated Pulses](/pyDLTPulseGenerator/PulsesPythonAndPHS.png)

# License (BSD-3-Clause)

Copyright (c) 2016-2019 Danny Petschke (danny.petschke@uni-wuerzburg.de). All rights reserved.<br>

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
