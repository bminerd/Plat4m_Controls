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
// Copyright (c) 2013-2023 Benjamin Minerd
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
/// @file KalmanFilter.h
/// @author Ben Minerd
/// @date 3/28/2013
/// @brief KalmanFilter class header file.
///

#ifndef PLAT4M_CONTROLS_KALMAN_FILTER_H
#define PLAT4M_CONTROLS_KALMAN_FILTER_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <Plat4m_Controls/Estimator.h>

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

template<typename ValueType,
         std::uint32_t nStates,
         std::uint32_t nObservables,
         std::uint32_t nControlInputs>
class KalmanFilter :  public Estimator<ValueType, nStates, nControlInputs>
{
public:

    //--------------------------------------------------------------------------
    // Public types
    //--------------------------------------------------------------------------

    typedef Eigen::Matrix<ValueType, nStates, nStates> MatrixNbyN;

    typedef Eigen::Matrix<ValueType, nStates, nObservables> MatrixNbyM;

    typedef Eigen::Matrix<ValueType, nObservables, nStates> MatrixMbyN;

    typedef Eigen::Matrix<ValueType, nObservables, nObservables> MatrixMbyM;

    typedef Eigen::Matrix<ValueType, nStates, 1> VectorN;

    typedef Eigen::Matrix<ValueType, nObservables, 1> VectorM;

    //--------------------------------------------------------------------------
    // Public constructors
    //--------------------------------------------------------------------------

    KalmanFilter() :
        x(),
        f(),
        u(),
        B(),
        H(),
        P(),
        Q(),
        R()
    {
    }

    //--------------------------------------------------------------------------
    // Public methods
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    void updateMeasurement(const Estimator::VectorN& measurementVector)
    {
        u = measurementVector;

        // State estimate
        // X = (F * X) + (B * U)
        //
        // Where...
        // X = state vector
        // F = state transition matrix
        // B = control matrix
        // U = control input vector
        //
        x = (F * x) + (B * u);

        // Covariance estimate
        P = (F * P * F.transpose()) + Q;
    }

    //--------------------------------------------------------------------------
    void updateCorrection(const VectorM& correctionVector)
    {
        const VectorM& z = correctionVector;

        // Measurement
        const VectorM y = z - (H * x);

        // Residual covariance

        const MatrixMByM s = (x * P * H.transpose()) + R;

        const MatrixMbyM sInverse = pseudoInverse(s);

        // Kalman gain
        const MatrixMbyM K = P * H.transpose() * sInverse;

        // Updated state estimate
        x = x + K * y;
        // Updated estimate covariance
        P = (MatrixNbyN::Identity() - K * H) * P;
    }

    //--------------------------------------------------------------------------
    static MatrixMbyM pseudoInverse(const MatrixMbyM& matrix)
    {
        // Compute Moore-Penrose pseudo inverse

        const ValueType pinvTolerance =
                                      std::numeric_limits<ValueType>::epsilon();

        Eigen::JacobiSVD<MatrixMbyM> matrixSvd(matrix,
                                               Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
        VectorM inverseSvdValues;

        for (std::uint32_t i = 0; i < nObservables; i++)
        {
            if (matrixSvd.singularValues()(i) > pinvTolerance)
            {
                inverseSvdValues(i) = 1.0f / matrixSvd.singularValues()(i);
            }
            else
            {
                inverseSvdValues(i) = 0.0f;
            }
        }

        MatrixMbyM matrixInverse;

        matrixInverse = (matrixSvd.matrixV()           *
                         inverseSvdValues.asDiagonal() *
                         matrixSvd.matrixU().transpose());

        return matrixInverse;
    }

    //--------------------------------------------------------------------------
    VectorN& x()
    {
        return x;
    }

    //--------------------------------------------------------------------------
    VectorN& getStateVector()
    {
        return x();
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& f()
    {
        return f;
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& getStateTransitionMatrix()
    {
        return f();
    }

    //--------------------------------------------------------------------------
    Eigen::Matrix<ValueType, nControlInputs, 1>& u()
    {
        return u;
    }

    //--------------------------------------------------------------------------
    Eigen::Matrix<ValueType, nControlInputs, 1>& getControlInputVector()
    {
        return u();
    }

    //--------------------------------------------------------------------------
    Eigen::Matrix<ValueType, nStates, nControlInputs>& B()
    {
        return B;
    }

    //--------------------------------------------------------------------------
    Eigen::Matrix<ValueType, nStates, nControlInputs>& getControlMatrix()
    {
        return B();
    }

    //--------------------------------------------------------------------------
    MatrixMbyN& H()
    {
        return H;
    }

    //--------------------------------------------------------------------------
    MatrixMbyN& getObservationMatrix()
    {
        return H();
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& P()
    {
        return P;
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& getPredictedCovarianceMatrix()
    {
        return P();
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& Q()
    {
        return Q;
    }

    //--------------------------------------------------------------------------
    MatrixNbyN& getProcessErrorCovarianceMatrix()
    {
        return Q();
    }

    //--------------------------------------------------------------------------
    MatrixMbyM& R()
    {
        return R;
    }

    //--------------------------------------------------------------------------
    MatrixMbyM& getMeasurementErrorCovarianceMatrix()
    {
        return R();
    }

private:

    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------

    ///
    /// @brief Contains the current values of the filtered state variables. Also
    /// called state vector.
    ///
    VectorN x;

    ///
    /// @brief Defines how the state variables relate to each other. Also called
    /// state transition matrix.
    ///
    MatrixNbyN f;

    ///
    /// @brief Stores the control input values. Also called control input
    /// vector.
    ///
    Eigen::Matrix<ValueType, nControlInputs, 1> u;

    ///
    /// @brief Defines how the control inputs relate to the state variables.
    /// Also called control matrix.
    ///
    Eigen::Matrix<ValueType, nStates, nControlInputs> B;

    ///
    /// @brief Defines what state variables are physically observed or measured.
    /// Also called observation matrix.
    ///
    MatrixMbyN H;

    ///
    /// @brief Contains the current covariance estimate for each state variable.
    /// Also called predicted covariance matrix.
    ///
    MatrixNbyN P;

    ///
    /// @brief Defines the process error covariance estimates for each state
    /// variable. Also called process error covariance matrix.
    ///
    MatrixNbyN Q;

    ///
    /// @brief Defines the measurement error covariance estimates for each state
    /// variable. Also called measurement error covariance matrix.
    ///
    MatrixMbyM R;
};

}; // namespace Controls

}; // namespace Plat4m

#endif // PLAT4M_CONTROLS_KALMAN_FILTER_H
