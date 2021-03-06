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
// Copyright (c) 2017 Benjamin Minerd
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
/// @file ControllerUpdateMessage.h
/// @author Ben Minerd
/// @date 2/2/2017
/// @brief ControllerUpdateMessage struct header file.
///

#ifndef PLAT4M_CONTROLS_CONTROLLER_UPDATE_MESSAGE_H
#define PLAT4M_CONTROLS_CONTROLLER_UPDATE_MESSAGE_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <stdint.h>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

namespace Plat4m
{

namespace Controls
{

//------------------------------------------------------------------------------
// Structs
//------------------------------------------------------------------------------

struct ControllerUpdateMessage
{
public:

    //--------------------------------------------------------------------------
    // Types
    //--------------------------------------------------------------------------

    enum Parameter
    {
        PARAMETER_INDEX,
        PARAMETER_TIME_US,
        PARAMETER_SET_POINT,
        PARAMETER_FEEDBACK,
        PARAMETER_ERROR,
        PARAMETER_OUTPUT,

        // Don't add values below
        PARAMETER_COUNT
    };

    //--------------------------------------------------------------------------
    // Fields
    //--------------------------------------------------------------------------

    uint8_t index;
    uint32_t timeUs;
    float setPoint;
    float feedback;
    float error;
    float output;
};

}; // namespace Controls

}; // namespace Plat4m

#endif // PLAT4M_CONTROLS_CONTROLLER_UPDATE_MESSAGE_H
