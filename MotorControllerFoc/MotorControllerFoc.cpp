//------------------------------------------------------------------------------
//       _______    __                           ___
//      ||  ___ \  || |             __          //  |
//      || |  || | || |   _______  || |__      //   |    _____  ___
//      || |__|| | || |  // ___  | ||  __|    // _  |   ||  _ \/ _ \
//      ||  ____/  || | || |  || | || |      // /|| |   || |\\  /\\ \
//      || |       || | || |__|| | || |     // /_|| |_  || | || | || |
//      || |       || |  \\____  | || |__  //_____   _| || | || | || |
//      ||_|       ||_|       ||_|  \\___|       ||_|   ||_| ||_| ||_|
//
//
// The MIT License (MIT)
//
// Copyright (c) 2018 Benjamin Minerd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------

///
/// @file MotorControllerFocFoc.cpp
/// @author Ben Minerd
/// @date 8/16/2018
/// @brief MotorControllerFoc class source file.
///

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <Plat4m_Controls/MotorControllerFoc/MotorControllerFoc.h>

using Plat4m::Controls::MotorControllerFoc;
using Plat4m::Controls::MotorController;

using Plat4m::Module;

//------------------------------------------------------------------------------
// Private static data members
//------------------------------------------------------------------------------

const RealNumber

//------------------------------------------------------------------------------
// Public constructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
MotorControllerFoc::MotorControllerFoc(CurrentSensor& phaseACurrentSensor,
                                       CurrentSensor& phaseBCurrentSensor,
                                       CurrentSensor& phaseCCurrentSensor,
                                       RotaryEncoder& rotaryEncoder,
                                       Controller& currentQController,
                                       Controller& currentDController,
                                       PwmOutput& phaseAPwmOutput,
                                       PwmOutput& phaseBPwmOutput,
                                       PwmOutput& phaseCPwmOutput) :
    MotorController(),
    myPhaseACurrentSensor(phaseACurrentSensor),
    myPhaseBCurrentSensor(phaseBCurrentSensor),
    myPhaseCCurrentSensor(phaseCCurrentSensor),
    myRotaryEncoder(rotaryEncoder),
    myCurrentQController(currentQController),
    myCurrentDController(currentDController),
    myPhaseAPwmOutput(phaseAPwmOutput),
    myPhaseBPwmOutput(phaseBPwmOutput),
    myPhaseCPwmOutput(phaseCPwmOutput)
{
}

//------------------------------------------------------------------------------
// Public virtual destructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
MotorControllerFoc::~MotorControllerFoc()
{
}

//------------------------------------------------------------------------------
// Private methods implemented from Module
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
Module::Error MotorControllerFoc::driverSetEnabled(const bool enabled)
{
    return Module::Error(Module::ERROR_CODE_NONE);
}

//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void MotorControllerFoc::update()
{
    // Electrical angle
    AngleRadians electricalAngleRadians = myRotaryEncoder.getAngleRadiansFast();

    // Clarke transform


    // Park transform

    // Apply controllers

    // Inverse Park transform

    // Inverse Clarke transform

    // Set phase PWMs
    myPhaseAPwmOutput.setDutyCycleNormalizedFast(0.0);
    myPhaseBPwmOutput.setDutyCycleNormalizedFast(0.0);
    myPhaseCPwmOutput.setDutyCycleNormalizedFast(0.0);
}
