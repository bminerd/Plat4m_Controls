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
/// @file MotorControllerFoc.h
/// @author Ben Minerd
/// @date 5/2/2018
/// @brief MotorControllerFoc class header file.
///

#ifndef MOTOR_CONTROLLER_FOC_H
#define MOTOR_CONTROLLER_FOC_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <Plat4m_Core/Plat4m.h>
#include <Plat4m_Core/Module.h>
#include <Plat4m_Core/ErrorTemplate.h>
#include <Plat4m_Core/CurrentSensor.h>
#include <Plat4m_Core/RotaryEncoder.h>
#include <Plat4m_Core/PwmOutput.h>
#include <Plat4m_Controls/MotorController.h>
#include <Plat4m_Controls/Controller.h>
#include <Plat4m_Math/Matrix.h>
#include <Plat4m_Math/Vector.h>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

namespace Plat4m
{

namespace Controls
{

//------------------------------------------------------------------------------
// Classes
//------------------------------------------------------------------------------

// TODO: Could this be a template class with concepts for current measurement,
// control loops, etc.? (Remember this excludes generic base-classes)
class MotorControllerFoc : public MotorController
{
public:

    //--------------------------------------------------------------------------
    // Public types
    //--------------------------------------------------------------------------

    enum ErrorCode
    {
        ERROR_CODE_NONE = 0
    };

    typedef ErrorTemplate<ErrorCode> Error;

    //--------------------------------------------------------------------------
    // Public constructors
    //--------------------------------------------------------------------------

    MotorControllerFoc(CurrentSensor& phaseACurrentSensor,
                       CurrentSensor& phaseBCurrentSensor,
                       CurrentSensor& phaseCCurrentSensor,
                       RotaryEncoder& rotaryEncoder,
                       Controller& currentQController,
                       Controller& currentDController,
                       PwmOutput& phaseAPwmOutput,
                       PwmOutput& phaseBPwmOutput,
                       PwmOutput& phaseCPwmOutput);

    //--------------------------------------------------------------------------
    // Public virtual destructors
    //--------------------------------------------------------------------------

    virtual ~MotorControllerFoc();

    //--------------------------------------------------------------------------
    // Public methods
    //--------------------------------------------------------------------------

    AngleRadians getElectricalAngleRadians();

    AngleDegrees getElectricalAngleDegrees();

private:

    //--------------------------------------------------------------------------
    // Private static data members
    //--------------------------------------------------------------------------

    // Constants

    static const RealNumber myClarkeTransformMatrixValues[];

    static const Math::Matrix myClarkeTransformMatrix;

    static const RealNumber myParkTransformMatrixValues[];

    static const Math::Matrix myParkTransformMatrix;

    static const RealNumber myInverseParkTransformMatrixValues[];

    static const Math::Matrix myInverseParkTransformMatrix;

    static const RealNumber myInverseClarkeTransformMatrixValues[];

    static const Math::Matrix myInverseClarkeTransformMatrix;

    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------

    CurrentSensor& myPhaseACurrentSensor;
    CurrentSensor& myPhaseBCurrentSensor;
    CurrentSensor& myPhaseCCurrentSensor;

    RotaryEncoder& myRotaryEncoder;

    Controller& myCurrentQController;
    Controller& myCurrentDController;

    PwmOutput& myPhaseAPwmOutput;
    PwmOutput& myPhaseBPwmOutput;
    PwmOutput& myPhaseCPwmOutput;

    //--------------------------------------------------------------------------
    // Private methods implemented from Module
    //--------------------------------------------------------------------------

    Module::Error driverSetEnabled(const bool enabled);

    //--------------------------------------------------------------------------
    // Private methods
    //--------------------------------------------------------------------------

    void update();
};

}; // namespace Control

}; // namespace Plat4m

#endif // MOTOR_CONTROLLER_FOC_H
