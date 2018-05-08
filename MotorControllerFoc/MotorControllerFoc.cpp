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
/// @date 5/2/2018
/// @brief MotorControllerFoc class source file.
///

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <Plat4m_Controls/MotorControllerFoc/MotorControllerFoc.h>

using Plat4m::Controls::MotorControllerFoc;
using Plat4m::Controls::MotorController;

using Plat4m::Module;

using namespace Plat4m;

//------------------------------------------------------------------------------
// Private static data members
//------------------------------------------------------------------------------

// Constants

const RealNumber MotorControllerFoc::myClarkeTransformMatrixValues[] =
{
    {          1.0,           0.0},
    {1.0/sqrt(3.0), 2.0/sqrt(3.0)}
};

const Math::Matrix<RealNumber>
     MotorControllerFoc::myClarkeTransformMatrix(myClarkeTransformMatrixValues);

const RealNumber MotorControllerFoc::myInverseClarkeTransformMatrixValues[] =
{
    {     1.0,            0.0},
    {-1.0/2.0,  sqrt(3.0)/2.0},
    {-1.0/2.0, -sqrt(3.0)/2.0}
};

const Math::Matrix<RealNumber>
     MotorControllerFoc::myInverseClarkeTransformMatrix(
                                          myInverseClarkeTransformMatrixValues);

//------------------------------------------------------------------------------
// Public constructors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
MotorControllerFoc::MotorControllerFoc(CurrentSensor& phaseACurrentSensor,
                                       CurrentSensor& phaseBCurrentSensor,
                                       RotaryEncoder& rotaryEncoder,
                                       Controller& currentQController,
                                       Controller& currentDController,
                                       PwmOutput& phaseAPwmOutput,
                                       PwmOutput& phaseBPwmOutput,
                                       PwmOutput& phaseCPwmOutput) :
    MotorController(),
    myPhaseACurrentSensor(phaseACurrentSensor),
    myPhaseBCurrentSensor(phaseBCurrentSensor),
    myRotaryEncoder(rotaryEncoder),
    myCurrentQController(currentQController),
    myCurrentDController(currentDController),
    myPhaseAPwmOutput(phaseAPwmOutput),
    myPhaseBPwmOutput(phaseBPwmOutput),
    myPhaseCPwmOutput(phaseCPwmOutput),
    myParkTransformMatrix(),
    myInverseParkTransformMatrix(),
    myCurrentABVector(),
    myCurrentAlphaBetaVector(),
    myCurrentDQVector(),
    myVoltageDQVector(),
    myVoltageAlphaBetaVector()
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
    // Get A+B phase currents

    myCurrentABVector(0) = myPhaseACurrentSensor.getCurrentAFast();
    myCurrentABVector(1) = myPhaseBCurrentSensor.getCurrentAFast();

    // Electrical angle

    AngleRadians electricalAngleRadians = myRotaryEncoder.getAngleRadiansFast();

    RealNumber cosTheta = cos(electricalAngleRadians);
    RealNumber sinTheta = sin(electricalAngleRadians);

    // Clarke transform

    myCurrentAlphaBetaVector = myClarkeTransformMatrix * myCurrentABVector;

    // Park transform

    myParkTransformMatrix(0, 0) =  cosTheta;
    myParkTransformMatrix(0, 1) =  sinTheta;
    myParkTransformMatrix(1, 0) = -sinTheta;
    myParkTransformMatrix(1, 1) =  cosTheta;

    myCurrentDQVector = myParkTransformMatrix * myCurrentAlphaBetaVector;

    // Apply controllers

    myVoltageDQVector(0) = myCurrentDController.update(myCurrentDQVector(0));
    myVoltageDQVector(1) = myCurrentQController.update(myCurrentDQVector(1));

    // Inverse Park transform

    myInverseParkTransformMatrix(0, 0) =  cosTheta;
    myInverseParkTransformMatrix(0, 1) = -sinTheta;
    myInverseParkTransformMatrix(1, 0) =  sinTheta;
    myInverseParkTransformMatrix(1, 1) =  cosTheta;

    myVoltageAlphaBetaVector = myInverseParkTransformMatrix * myVoltageDQVector;

    // Inverse Clarke transform

    myPhaseDutyCyclesVector =
                      myInverseClarkeTransformMatrix * myVoltageAlphaBetaVector;

    // Set phase PWMs
    myPhaseAPwmOutput.setDutyCycleNormalizedFast(myPhaseDutyCyclesVector(0));
    myPhaseBPwmOutput.setDutyCycleNormalizedFast(myPhaseDutyCyclesVector(1));
    myPhaseCPwmOutput.setDutyCycleNormalizedFast(myPhaseDutyCyclesVector(2));
}
