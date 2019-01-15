# DLTPulseGenerator
Copyright (c) 2016-2019 Danny Petschke (danny.petschke@uni-wuerzburg.de). All rights reserved.<br><br>
<b>DLTPulseGenerator</b> - A library for the simulation of lifetime spectra based on detector-output pulses

# Introduction

The quantitative analysis of lifetime spectra relevant in both life and materials sciences presents one of the ill-posed inverse problems and leads to the most stringent requirements on the hardware specifications and analysis algorithms.<br><br>
<b>DLTPulseGenerator is written in native C++ 11</b> (ISO/IEC 14882:2011) and provides the simulation of (e.g. positron or fluorescence) lifetime spectra according to the measurement setup: i.e. the kind of detectors (photomultipliers (PMT) or (avalanche) diodes) and the acquisition hardware (mostly the combination of ADC and FPGA). 
The simulation is based on pairs of non-TTL detector output pulses which require the constant fraction principle for the determination of the exact timing signal.<br><br>

DLTPulseGenerator library provides the compilation as <i>static</i> or <i>linked</i> library to make it easily accessible from other programming languages such as ...<br>
- <i>matlab</i> (for [mex-library](https://de.mathworks.com/help/matlab/matlab_external/standalone-example.html)) or<br>
- <i>python</i> (for [ctypes-library](https://docs.python.org/3/library/ctypes.html)). 

# How to cite this library?

Petschke D., Staab T.E.M. <i>DLTPulseGenerator: A library for the simulation of lifetime spectra based on detector-output pulses</i>. <b>SoftwareX (2018), https://doi.org/10.1016/j.softx.2018.04.002</b>.

## DLTPulseGenerator v1.0

This <b>[release v1.0](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.0)</b> refers to the <b>original paper [(Petschke and Staab (2018))](https://doi.org/10.1016/j.softx.2018.04.002)</b> and provides the simulation of lifetime spectra consisting of <i>discrete specific lifetimes</i>.

forked by <b>SoftwareX (Elsevier)</b>: [https://github.com/ElsevierSoftwareX/SOFTX-D-17-00077](https://github.com/ElsevierSoftwareX/SOFTX-D-17-00077) 

## DLTPulseGenerator v1.1

This <b>[release v1.1](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.1)</b> refers to the <b>update paper (v1.1) [(Petschke and Staab (2018))](https://doi.org/10.1016/j.softx.2018.05.001)</b> and was extended to provide the simulation of lifetime spectra consisting of <i>distributed specific lifetimes</i> as can be found in porous materials (polymers, glasses) using positron annihilation lifetime spectroscopy (PALS).

## DLTPulseGenerator v1.2

This <b>[release v1.2](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.2)</b> refers to the <b>update paper (v1.2) [(Petschke and Staab (2018))](https://doi.org/10.1016/j.softx.2018.06.003)</b> and was modified to allow the simulation of lifetime spectra consisting of non-Gaussian or any-distributed and linearly combined Instrument Response Functions (IRF) for the PDS A/B and MU. 

## DLTPulseGenerator v1.3

This <b>[release v1.3](https://github.com/dpscience/DLTPulseGenerator/releases/tag/1.3)</b> provides the simulation of additional hardware influences mainly originating from the parts of the A/D converter such as baseline-offset jitter, random noise, fixed pattern and random aperture jitters on the time axis and the digitization depth [bit].

## Applications using DLTPulseGenerator library

<b>[DDRS4PALS](https://github.com/dpscience/DDRS4PALS) software</b> written by Danny Petschke

# Example using C++

```c++
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
  
  while (1) {
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
}
```
Errors can be handled by inheriting from <i>class DLTPulseGenerator</i> using the provided callback function: see [DLTPulseGeneratorApp.h/.cpp](https://github.com/dpscience/DLTPulseGenerator/blob/master/DLTPulseGenerator/example/AppDLTPulseGenerator/AppDLTPulseGenerator/DLTPulseGeneratorApp.h).  

# Example using python

A <b>library wrapper</b> in <i>python</i> ([pyDLTPulseGenerator.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGenerator.py)) demonstrating the usage of [ctypes-library](https://docs.python.org/3/library/ctypes.html) by calling the functions of <b>DLTPulseGenerator.dll</b> ([x86](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator/x86)/[x64](https://github.com/dpscience/DLTPulseGenerator/tree/master/pyDLTPulseGenerator/x64)) is provided. [pyDLTPulseGeneratorApp.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGeneratorApp.py) calls functions from the library (wrapper) [pyDLTPulseGenerator.py](https://github.com/dpscience/DLTPulseGenerator/blob/master/pyDLTPulseGenerator/pyDLTPulseGenerator.py). The generated pulse pairs and the pulse height spectrum are displayed for demonstration purposes.<br>

#### requirements:
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
