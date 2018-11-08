#*************************************************************************************************
#**
#** Copyright (c) 2017, 2018 Danny Petschke. All rights reserved.
#** 
#** Redistribution and use in source and binary forms, with or without modification, 
#** are permitted provided that the following conditions are met:
#**
#** 1. Redistributions of source code must retain the above copyright notice, 
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

import ctypes
from ctypes import cdll

def __information__():
    print("#****************** pyDLTPulseGenerator 1.2 (05.06.2017) *******************")
    print("#**")
    print("#** Copyright (C) 2017, 2018 Danny Petschke")
    print("#**")
    print("#** Contact: danny.petschke@uni-wuerzburg.de")
    print("#**")
    print("#***************************************************************************\n")

def __licence__():
    print("#*************************************************************************************************")
    print("#**")
    print("#** Copyright (c) 2017, 2018 Danny Petschke. All rights reserved.")
    print("#**")
    print("#** Redistribution and use in source and binary forms, with or without modification,") 
    print("#** are permitted provided that the following conditions are met:")
    print("#**")
    print("#** 1. Redistributions of source code must retain the above copyright notice,")
    print("#**    this list of conditions and the following disclaimer.")
    print("#**")
    print("#** 2. Redistributions in binary form must reproduce the above copyright notice,") 
    print("#**    this list of conditions and the following disclaimer in the documentation") 
    print("#**    and/or other materials provided with the distribution.")
    print("#**")
    print("#** 3. Neither the name of the copyright holder ""Danny Petschke"" nor the names of its")  
    print("#**    contributors may be used to endorse or promote products derived from this software")  
    print("#**    without specific prior written permission.")
    print("#**")
    print("#**")
    print("#** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ""AS IS"" AND ANY EXPRESS") 
    print("#** OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF") 
    print("#** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE") 
    print("#** COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,") 
    print("#** EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF") 
    print("#** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)") 
    print("#** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR") 
    print("#** TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,") 
    print("#** EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.")
    print("#**")
    print("#** Contact: danny.petschke@uni-wuerzburg.de")
    print("#**")
    print("#*************************************************************************************************")

class DLTPointF:
    m_x = float(0.0)
    m_y = float(0.0)

    def __init__(self, x = float(0.0), y = float(0.0)):
        self.m_x = x
        self.m_y = y

    def __del__(self):
        del self.m_x
        del self.m_y
        
    def setX(self, x):
        self.m_x = x

    def setY(self, y):
        self.m_y = y

    def setPoint(self, x, y):
        self.m_x = x
        self.m_y = y

    def x(self):
        return self.m_x

    def y(self):
        return self.m_y

    def __print__(self):
        print("DLTPointF: (", self.m_x, ";", self.m_y, ")")


class DLTPulseF:
    m_x = []
    m_y = []

    def __init__(self):
        self.m_x = []
        self.m_y = []

    def __del__(self):
        del self.m_x
        del self.m_y

    def append(self, point):
        self.m_x.append(point.x())
        self.m_y.append(point.y())

    def clear(self):
        self.m_x.clear()
        self.m_y.clear()

    def count(self):
        return len(self.m_x)

    def getPoint(self, index): #returns DLTPointF
        if index >= self.count() or index < 0:
           raise ValueError('DLTPulseF: Index out of range!')

        return DLTPointF(self.m_x[index], self.m_y[index])

    def getTime(self):
        return self.m_x

    def getVoltage(self):
        return self.m_y

    def getMaximumVoltage(self):
        maximum = 0;
        for index in range(0, self.count()-1):
            if self.m_y[index] > maximum:
                maximum = self.m_y[index]

        return maximum

    def getMinimumVoltage(self):
        minimum = 1E9;
        for index in range(0, self.count()-1):
            if self.m_y[index] < minimum:
                minimum = self.m_y[index]

        return minimum
            

class DLTPHS:
    m_meanOfStart_A_in_milliVolt = 190.0
    m_meanOfStop_A_in_milliVolt = 90.0
    m_stddevOfStart_A_in_milliVolt = 150.0
    m_stddevOfStop_A_in_milliVolt = 25.0

    m_meanOfStart_B_in_milliVolt = 190.0
    m_meanOfStop_B_in_milliVolt = 90.0
    m_stddevOfStart_B_in_milliVolt = 150.0
    m_stddevOfStop_B_in_milliVolt = 25.0

    def __init__(self, meanOfStartA = 190.0, stddevOfStartA = 150.0, meanOfStopA = 90.0, stddevOfStopA = 25.0, meanOfStartB = 190.0, stddevOfStartB = 150.0, meanOfStopB = 90.0, stddevOfStopB = 25.0):
        self.m_meanOfStart_A_in_milliVolt = meanOfStartA
        self.m_meanOfStop_A_in_milliVolt = meanOfStopA
        self.m_stddevOfStart_A_in_milliVolt = stddevOfStartA
        self.m_stddevOfStop_A_in_milliVolt = stddevOfStopA

        self.m_meanOfStart_B_in_milliVolt = meanOfStartB
        self.m_meanOfStop_B_in_milliVolt = meanOfStopB
        self.m_stddevOfStart_B_in_milliVolt = stddevOfStartB
        self.m_stddevOfStop_B_in_milliVolt = stddevOfStopB

class DLTDistributionInfo:
    m_enabled = False

    #GAUSSIAN           = 0
    #LOG_NORMAL         = 1
    #LORENTZIAN_CAUCHY  = 2
    
    m_functionType = 0
    m_param1 = 0.0
    m_param2 = 0.0 #reserved for future implementations
    m_gridCount = 0
    m_gridIncrement = 0.0

    def __init__(self, enabled = False, functionType = 0, param1 = 0.0, gridCount = 0, gridIncrement = 0.0):
        self.m_enabled = enabled
        self.m_functionType = functionType
        self.m_param1 = param1
        self.m_param2 = 0.0
        self.m_gridCount = gridCount
        self.m_gridIncrement = gridIncrement
        
class DLTSimulationInput:
    m_lt1_activated = True
    m_lt2_activated = True
    m_lt3_activated = True
    m_lt4_activated = False
    m_lt5_activated = False

    m_tau1_in_nanoSeconds = 0.160
    m_tau2_in_nanoSeconds = 0.420
    m_tau3_in_nanoSeconds = 3.2
    m_tau4_in_nanoSeconds = 0.0
    m_tau5_in_nanoSeconds = 0.0

    m_distrInfo1 = DLTDistributionInfo()
    m_distrInfo2 = DLTDistributionInfo()
    m_distrInfo3 = DLTDistributionInfo()
    m_distrInfo4 = DLTDistributionInfo()
    m_distrInfo5 = DLTDistributionInfo()

    m_intensity1 = 0.25
    m_intensity2 = 0.25
    m_intensity3 = 0.5
    m_intensity4 = 0.0
    m_intensity5 = 0.0

    m_intensityOfPromtOccurrance = 0.2
    m_intensityOfBackgroundOccurrance = 0.05

    m_isStartStopAlternating = True;

    def __init__(self, lt1_activated = True, tau1_in_nanoSeconds = 0.160, intensity1 = 0.25, distrInfo1 = DLTDistributionInfo(),
               lt2_activated = True, tau2_in_nanoSeconds = 0.420, intensity2 = 0.25, distrInfo2 = DLTDistributionInfo(),
               lt3_activated = True, tau3_in_nanoSeconds = 3.2, intensity3 = 0.5, distrInfo3 = DLTDistributionInfo(),
               lt4_activated = False, tau4_in_nanoSeconds = 0.0, intensity4 = 0.0, distrInfo4 = DLTDistributionInfo(),
               lt5_activated = False, tau5_in_nanoSeconds = 0.0, intensity5 = 0.0, distrInfo5 = DLTDistributionInfo(),
               intensityOfPromtOccurrance = 0.2,
               intensityOfBackgroundOccurrance = 0.05,
               isStartStopAlternating = True):
        self.m_lt1_activated = lt1_activated
        self.m_lt2_activated = lt2_activated
        self.m_lt3_activated = lt3_activated
        self.m_lt4_activated = lt4_activated
        self.m_lt5_activated = lt5_activated

        self.m_tau1_in_nanoSeconds = tau1_in_nanoSeconds
        self.m_tau2_in_nanoSeconds = tau2_in_nanoSeconds
        self.m_tau3_in_nanoSeconds = tau3_in_nanoSeconds
        self.m_tau4_in_nanoSeconds = tau4_in_nanoSeconds
        self.m_tau5_in_nanoSeconds = tau5_in_nanoSeconds

        self.m_intensity1 = intensity1
        self.m_intensity2 = intensity2
        self.m_intensity3 = intensity3
        self.m_intensity4 = intensity4
        self.m_intensity5 = intensity5

        self.m_distrInfo1 = distrInfo1
        self.m_distrInfo2 = distrInfo2
        self.m_distrInfo3 = distrInfo3
        self.m_distrInfo4 = distrInfo4
        self.m_distrInfo5 = distrInfo5

        self.m_intensityOfPromtOccurrance = intensityOfPromtOccurrance
        self.m_intensityOfBackgroundOccurrance = intensityOfBackgroundOccurrance

        self.m_isStartStopAlternating = isStartStopAlternating;

class DLTIRF:
    m_enabled = False

    #GAUSSIAN           = 0
    #LOG_NORMAL         = 1
    #LORENTZIAN_CAUCHY  = 2
    
    m_functionType  = 0
    m_relativeShift = 0.0
    m_intensity     = 0.0 
    m_uncertainty   = 0.0
    
    def __init__(self, enabled = False, functionType = 0, relativeShift = 0.0, intensity = 0.0, uncertainty = 0.0):
        self.m_enabled = enabled
        self.m_functionType = functionType
        self.m_relativeShift = relativeShift
        self.m_intensity = intensity
        self.m_uncertainty = uncertainty
        
class DLTSetup:
    #PDS-A:
    m_irf_PDSA_1 = DLTIRF(True, 0, 0.0, 1.0, 0.084932901)
    m_irf_PDSA_2 = DLTIRF()
    m_irf_PDSA_3 = DLTIRF()
    m_irf_PDSA_4 = DLTIRF()
    m_irf_PDSA_5 = DLTIRF()

    #PDS-B:
    m_irf_PDSB_1 = DLTIRF(True, 0, 0.0, 1.0, 0.084932901)
    m_irf_PDSB_2 = DLTIRF()
    m_irf_PDSB_3 = DLTIRF()
    m_irf_PDSB_4 = DLTIRF()
    m_irf_PDSB_5 = DLTIRF()

    #MU:
    m_irf_MU_1 = DLTIRF(True, 0, 0.0, 1.0, 0.0025)
    m_irf_MU_2 = DLTIRF()
    m_irf_MU_3 = DLTIRF()
    m_irf_MU_4 = DLTIRF()
    m_irf_MU_5 = DLTIRF()

    m_ATS_in_nanoSeconds = 0.25

    m_sweep_in_nanoSeconds = 200.0
    m_numberOfCells = 1024

    def __init__(self,
                 irf_PDSA_1 = DLTIRF(True, 0, 0.0, 1.0, 0.084932901),
                 irf_PDSA_2 = DLTIRF(),
                 irf_PDSA_3 = DLTIRF(),
                 irf_PDSA_4 = DLTIRF(),
                 irf_PDSA_5 = DLTIRF(),
                 irf_PDSB_1 = DLTIRF(True, 0, 0.0, 1.0, 0.084932901),
                 irf_PDSB_2 = DLTIRF(),
                 irf_PDSB_3 = DLTIRF(),
                 irf_PDSB_4 = DLTIRF(),
                 irf_PDSB_5 = DLTIRF(),
                 irf_MU_1 = DLTIRF(True, 0, 0.0, 1.0, 0.0025),
                 irf_MU_2 = DLTIRF(),
                 irf_MU_3 = DLTIRF(),
                 irf_MU_4 = DLTIRF(),
                 irf_MU_5 = DLTIRF(),
                 ATS_in_nanoSeconds = 0.25,
                 sweep_in_nanoSeconds = 200.0,
                 numberOfCells = 1024):
        self.m_irf_PDSA_1 = irf_PDSA_1;
        self.m_irf_PDSA_2 = irf_PDSA_2;
        self.m_irf_PDSA_3 = irf_PDSA_3;
        self.m_irf_PDSA_4 = irf_PDSA_4;
        self.m_irf_PDSA_5 = irf_PDSA_5;

        self.m_irf_PDSB_1 = irf_PDSB_1;
        self.m_irf_PDSB_2 = irf_PDSB_2;
        self.m_irf_PDSB_3 = irf_PDSB_3;
        self.m_irf_PDSB_4 = irf_PDSB_4;
        self.m_irf_PDSB_5 = irf_PDSB_5;

        self.m_irf_MU_1 = irf_MU_1;
        self.m_irf_MU_2 = irf_MU_2;
        self.m_irf_MU_3 = irf_MU_3;
        self.m_irf_MU_4 = irf_MU_4;
        self.m_irf_MU_5 = irf_MU_5;
        
        self.m_ATS_in_nanoSeconds = ATS_in_nanoSeconds

        self.m_sweep_in_nanoSeconds = sweep_in_nanoSeconds
        self.m_numberOfCells = numberOfCells

class DLTPulse:
    m_riseTime_in_nanoSeconds = 5.0
    m_pulseWidth_in_nanoSeconds = 0.165
    m_delay_in_nanoSeconds = 65.0

    m_amplitude_in_milliVolt = 500.0
    m_isPositiveSignalPolarity = True
        
    def __init__(self, riseTime_in_nanoSeconds = 5.0,
                 pulseWidth_in_nanoSeconds = 0.165,
                 delay_in_nanoSeconds = 65.0,
                 amplitude_in_milliVolt = 500.0,
                 isPositiveSignalPolarity = True):
        self.m_riseTime_in_nanoSeconds = riseTime_in_nanoSeconds
        self.m_pulseWidth_in_nanoSeconds = pulseWidth_in_nanoSeconds
        self.m_delay_in_nanoSeconds = delay_in_nanoSeconds

        self.m_amplitude_in_milliVolt = amplitude_in_milliVolt
        self.m_isPositiveSignalPolarity = isPositiveSignalPolarity
                 
class DLTPulseGenerator:
    __dllPtr = 0

    m_getTimeA = 0
    m_getTimeB = 0
    m_getVoltageA = 0
    m_getVoltageB = 0
    
    m_error = 0
    m_numberOfCells = 0

    m_emitPulse = 0

    def __init__(self, dLTPHSDistribution = DLTPHS(),
                 dLTSimulationInput       = DLTSimulationInput(),
                 dLTSetupInfo             = DLTSetup(),
                 dLTPulseInfo             = DLTPulse(),
                 path = "DLTPulseGenerator.dll"):
        self.__dllPtr = cdll.LoadLibrary(path)
        self.__dllPtr.init() #initialize the Pulse-Generator once!

        #DLTSimulationInput:
        self.__dllPtr.setLifeTime_1(ctypes.c_bool(dLTSimulationInput.m_lt1_activated),
                                    ctypes.c_double(dLTSimulationInput.m_tau1_in_nanoSeconds),
                                    ctypes.c_double(dLTSimulationInput.m_intensity1),
                                    ctypes.c_bool(dLTSimulationInput.m_distrInfo1.m_enabled),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo1.m_functionType),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo1.m_param1),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo1.m_gridCount),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo1.m_gridIncrement))
        self.__dllPtr.setLifeTime_2(ctypes.c_bool(dLTSimulationInput.m_lt2_activated),
                                    ctypes.c_double(dLTSimulationInput.m_tau2_in_nanoSeconds),
                                    ctypes.c_double(dLTSimulationInput.m_intensity2),
                                    ctypes.c_bool(dLTSimulationInput.m_distrInfo2.m_enabled),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo2.m_functionType),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo2.m_param1),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo2.m_gridCount),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo2.m_gridIncrement))
        self.__dllPtr.setLifeTime_3(ctypes.c_bool(dLTSimulationInput.m_lt3_activated),
                                    ctypes.c_double(dLTSimulationInput.m_tau3_in_nanoSeconds),
                                    ctypes.c_double(dLTSimulationInput.m_intensity3),
                                    ctypes.c_bool(dLTSimulationInput.m_distrInfo3.m_enabled),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo3.m_functionType),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo3.m_param1),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo3.m_gridCount),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo3.m_gridIncrement))
        self.__dllPtr.setLifeTime_4(ctypes.c_bool(dLTSimulationInput.m_lt4_activated),
                                    ctypes.c_double(dLTSimulationInput.m_tau4_in_nanoSeconds),
                                    ctypes.c_double(dLTSimulationInput.m_intensity4),
                                    ctypes.c_bool(dLTSimulationInput.m_distrInfo4.m_enabled),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo4.m_functionType),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo4.m_param1),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo4.m_gridCount),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo4.m_gridIncrement))
        self.__dllPtr.setLifeTime_5(ctypes.c_bool(dLTSimulationInput.m_lt5_activated),
                                    ctypes.c_double(dLTSimulationInput.m_tau5_in_nanoSeconds),
                                    ctypes.c_double(dLTSimulationInput.m_intensity5),
                                    ctypes.c_bool(dLTSimulationInput.m_distrInfo5.m_enabled),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo5.m_functionType),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo5.m_param1),
                                    ctypes.c_int(dLTSimulationInput.m_distrInfo5.m_gridCount),
                                    ctypes.c_double(dLTSimulationInput.m_distrInfo5.m_gridIncrement))
        self.__dllPtr.setStartStopAlternating(ctypes.c_bool(dLTSimulationInput.m_isStartStopAlternating))

        #DLTPHS:
        self.__dllPtr.setStartOfA(ctypes.c_double(dLTPHSDistribution.m_meanOfStart_A_in_milliVolt),
                                  ctypes.c_double(dLTPHSDistribution.m_stddevOfStart_A_in_milliVolt))
        self.__dllPtr.setStopOfA(ctypes.c_double(dLTPHSDistribution.m_meanOfStop_A_in_milliVolt),
                                  ctypes.c_double(dLTPHSDistribution.m_stddevOfStop_A_in_milliVolt))
        self.__dllPtr.setStartOfB(ctypes.c_double(dLTPHSDistribution.m_meanOfStart_B_in_milliVolt),
                                  ctypes.c_double(dLTPHSDistribution.m_stddevOfStart_B_in_milliVolt))
        self.__dllPtr.setStopOfB(ctypes.c_double(dLTPHSDistribution.m_meanOfStop_B_in_milliVolt),
                                  ctypes.c_double(dLTPHSDistribution.m_stddevOfStop_B_in_milliVolt))
        
        #DLTSetup:
        self.__dllPtr.setIRF_PDS_A_1(ctypes.c_bool(dLTSetupInfo.m_irf_PDSA_1.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSA_1.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_1.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_1.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_1.m_relativeShift))

        self.__dllPtr.setIRF_PDS_A_2(ctypes.c_bool(dLTSetupInfo.m_irf_PDSA_2.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSA_2.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_2.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_2.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_2.m_relativeShift))

        self.__dllPtr.setIRF_PDS_A_3(ctypes.c_bool(dLTSetupInfo.m_irf_PDSA_3.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSA_3.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_3.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_3.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_3.m_relativeShift))

        self.__dllPtr.setIRF_PDS_A_4(ctypes.c_bool(dLTSetupInfo.m_irf_PDSA_4.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSA_4.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_4.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_4.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_4.m_relativeShift))

        self.__dllPtr.setIRF_PDS_A_5(ctypes.c_bool(dLTSetupInfo.m_irf_PDSA_5.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSA_5.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_5.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_5.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSA_5.m_relativeShift))


        self.__dllPtr.setIRF_PDS_B_1(ctypes.c_bool(dLTSetupInfo.m_irf_PDSB_1.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSB_1.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_1.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_1.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_1.m_relativeShift))

        self.__dllPtr.setIRF_PDS_B_2(ctypes.c_bool(dLTSetupInfo.m_irf_PDSB_2.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSB_2.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_2.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_2.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_2.m_relativeShift))

        self.__dllPtr.setIRF_PDS_B_3(ctypes.c_bool(dLTSetupInfo.m_irf_PDSB_3.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSB_3.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_3.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_3.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_3.m_relativeShift))

        self.__dllPtr.setIRF_PDS_B_4(ctypes.c_bool(dLTSetupInfo.m_irf_PDSB_4.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSB_4.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_4.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_4.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_4.m_relativeShift))

        self.__dllPtr.setIRF_PDS_B_5(ctypes.c_bool(dLTSetupInfo.m_irf_PDSB_5.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_PDSB_5.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_5.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_5.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_PDSB_5.m_relativeShift))


        self.__dllPtr.setIRF_MU_1(ctypes.c_bool(dLTSetupInfo.m_irf_MU_1.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_MU_1.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_1.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_1.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_1.m_relativeShift))

        self.__dllPtr.setIRF_MU_2(ctypes.c_bool(dLTSetupInfo.m_irf_MU_2.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_MU_2.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_2.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_2.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_2.m_relativeShift))

        self.__dllPtr.setIRF_MU_3(ctypes.c_bool(dLTSetupInfo.m_irf_MU_3.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_MU_3.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_3.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_3.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_3.m_relativeShift))

        self.__dllPtr.setIRF_MU_4(ctypes.c_bool(dLTSetupInfo.m_irf_MU_4.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_MU_4.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_4.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_4.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_4.m_relativeShift))

        self.__dllPtr.setIRF_MU_5(ctypes.c_bool(dLTSetupInfo.m_irf_MU_5.m_enabled),
                                     ctypes.c_int(dLTSetupInfo.m_irf_MU_5.m_functionType),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_5.m_intensity),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_5.m_uncertainty),
                                     ctypes.c_double(dLTSetupInfo.m_irf_MU_5.m_relativeShift))
        

        
        self.__dllPtr.setATS(ctypes.c_double(dLTSetupInfo.m_ATS_in_nanoSeconds))
        self.__dllPtr.setSweep(ctypes.c_double(dLTSetupInfo.m_sweep_in_nanoSeconds))
        self.__dllPtr.setNumberOfCells(ctypes.c_int(dLTSetupInfo.m_numberOfCells))

        #DLTPulse:
        self.__dllPtr.setRiseTime(ctypes.c_double(dLTPulseInfo.m_riseTime_in_nanoSeconds))
        self.__dllPtr.setPulseWidth(ctypes.c_double(dLTPulseInfo.m_pulseWidth_in_nanoSeconds))
        self.__dllPtr.setDelay(ctypes.c_double(dLTPulseInfo.m_delay_in_nanoSeconds))
        self.__dllPtr.setAmplitude(ctypes.c_double(dLTPulseInfo.m_amplitude_in_milliVolt))
        self.__dllPtr.setUsingPositiveSignalPolarity(ctypes.c_bool(dLTPulseInfo.m_isPositiveSignalPolarity))

        self.__dllPtr.update()
        
        self.m_error = self.__dllPtr.getLastError
        self.m_numberOfCells = self.__dllPtr.getNumberOfCells
        self.m_emitPulse = self.__dllPtr.emitPulse

        self.m_getTimeA = self.__dllPtr.getTimeA
        self.m_getTimeB = self.__dllPtr.getTimeB

        self.m_getVoltageA = self.__dllPtr.getVoltageA
        self.m_getVoltageB = self.__dllPtr.getVoltageB

        self.m_getVoltageA.restype = ctypes.c_double #mV
        self.m_getVoltageB.restype = ctypes.c_double #mV

        self.m_getTimeA.restype = ctypes.c_double #ns
        self.m_getTimeB.restype = ctypes.c_double #ns
        
        self.m_error.restype = ctypes.c_int #error-code
        self.m_numberOfCells.restype = ctypes.c_int #[#]
        
        self.m_emitPulse.restype = ctypes.c_bool

    def __del__(self):
        del __dllPtr
        del m_getTimeA
        del m_getTimeB
        del m_getVoltageA
        del m_getVoltageB   
        del m_error
        del m_numberOfCells
        del m_emitPulse

    def getError(self):
        return self.__dllPtr.getLastError()
        
    def emitPulses(self, pulseA, pulseB, triggerA, triggerB):
        ok = self.m_emitPulse(ctypes.c_double(triggerA), ctypes.c_double(triggerB))

        if ok == False:
            return False

        
        maxCell = (self.m_numberOfCells() - 1)
        
        pulseA.clear()
        pulseB.clear()
        
        #read pulse-values:
        for cell in range(0, maxCell):
            pulseA.append(DLTPointF(self.m_getTimeA(ctypes.c_int(cell)), self.m_getVoltageA(ctypes.c_int(cell))))
            pulseB.append(DLTPointF(self.m_getTimeB(ctypes.c_int(cell)), self.m_getVoltageB(ctypes.c_int(cell))))

        return True
