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
// Copyright (c) 2013 Benjamin Minerd
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

#ifndef PLAT4M_KALMAN_FILTER_H
#define PLAT4M_KALMAN_FILTER_H

//------------------------------------------------------------------------------
// Include files
//------------------------------------------------------------------------------

#include <float.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

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
         uint32_t nStates,
         uint32_t nObservables,
         uint32_t nControlInputs>
class KalmanFilter
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
		myXVector(),
		myFMatrix(),
		myUVector(),
		myBMatrix(),
		myHMatrix(),
		myPMatrix(),
		myQMatrix(),
		myRMatrix()
	{
	}

    //--------------------------------------------------------------------------
    // Public methods
    //--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	void predict()
	{
		// State estimate
		// X = (F * X) + (B * U)
		//
		// Where...
		// X = state vector
		// F = state transition matrix
		// B = control matrix
		// U = control input vector
		//
        myXVector = myFMatrix * myXVector;

		// Covariance estimate
		// P = (F * P * F_T) + Q
		myPMatrix = (myFMatrix * myPMatrix * myFMatrix.transpose()) + myQMatrix;
	}

    //--------------------------------------------------------------------------
    void predict(const Eigen::Matrix<ValueType, nControlInputs, 1>& uVector)
    {
        // State estimate
        // X = (F * X) + (B * U)
        //
        // Where...
        // X = state vector
        // F = state transition matrix
        // B = control matrix
        // U = control input vector
        //
        myXVector = (myFMatrix * myXVector) + (myBMatrix * myUVector);

        // Covariance estimate
        // P = (F * P * F_T) + Q
        myPMatrix = (myFMatrix * myPMatrix * myFMatrix.transpose()) + myQMatrix;
    }

	//--------------------------------------------------------------------------
	void update(const VectorM& measurementVector)
	{
		VectorM yMatrix;
		MatrixMbyM sMatrix;
		MatrixMbyM sMatrixInverse;
		MatrixNbyM kMatrix;

		// Measurement
		yMatrix = measurementVector - (myHMatrix * myXVector);

		// Residual covariance

		sMatrix = (myHMatrix * myPMatrix * myHMatrix.transpose()) + myRMatrix;

		sMatrixInverse = pseudoInverse(sMatrix);

		// Kalman gain
		kMatrix = myPMatrix * myHMatrix.transpose() * sMatrixInverse;

		// Updated state estimate
		myXVector = myXVector + kMatrix * yMatrix;
		// Updated estimate covariance
		myPMatrix = (MatrixNbyN::Identity() - kMatrix * myHMatrix) * myPMatrix;
	}

	//--------------------------------------------------------------------------
	static MatrixMbyM pseudoInverse(const MatrixMbyM& matrix)
	{
        // Compute Moore-Penrose pseudo inverse

		ValueType pinvTolerance = FLT_EPSILON;

		Eigen::JacobiSVD<MatrixMbyM> matrixSvd(matrix,
		                                       Eigen::ComputeFullU |
											   Eigen::ComputeFullV);
		VectorM inverseSvdValues;

		for (uint32_t i = 0; i < nObservables; i++)
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
	VectorN& getXVector()
	{
		return myXVector;
	}

	//--------------------------------------------------------------------------
	VectorN& getStateVector()
	{
		return myXVector;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getFMatrix()
	{
		return myFMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getStateTransitionMatrix()
	{
		return myFMatrix;
	}

	//--------------------------------------------------------------------------
	Eigen::Matrix<ValueType, nControlInputs, 1>& getUVector()
	{
		return myUVector;
	}

	//--------------------------------------------------------------------------
	Eigen::Matrix<ValueType, nControlInputs, 1>& getControlInputVector()
	{
		return myUVector;
	}

	//--------------------------------------------------------------------------
	Eigen::Matrix<ValueType, nStates, nControlInputs>& getBMatrix()
	{
		return myBMatrix;
	}

	//--------------------------------------------------------------------------
	Eigen::Matrix<ValueType, nStates, nControlInputs>& getControlMatrix()
	{
		return myBMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixMbyN& getHMatrix()
	{
		return myHMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixMbyN& getObservationMatrix()
	{
		return myHMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getPMatrix()
	{
		return myPMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getPredictedCovarianceMatrix()
	{
		return myPMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getQMatrix()
	{
		return myQMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixNbyN& getProcessErrorCovarianceMatrix()
	{
		return myQMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixMbyM& getRMatrix()
	{
		return myRMatrix;
	}

	//--------------------------------------------------------------------------
	MatrixMbyM& getMeasurementErrorCovarianceMatrix()
	{
		return myRMatrix;
	}

private:

    //--------------------------------------------------------------------------
    // Private data members
    //--------------------------------------------------------------------------

	///
	/// @brief Contains the current values of the filtered state variables. Also
	/// called state vector.
	///
	VectorN myXVector;

	///
	/// @brief Defines how the state variables relate to each other. Also called
	/// state transition matrix.
	///
	MatrixNbyN myFMatrix;

	///
	/// @brief Stores the control input values. Also called control input
	/// vector.
	///
	Eigen::Matrix<ValueType, nControlInputs, 1> myUVector;

	///
	/// @brief Defines how the control inputs relate to the state variables.
	/// Also called control matrix.
	///
	Eigen::Matrix<ValueType, nStates, nControlInputs> myBMatrix;

	///
	/// @brief Defines what state variables are physically observed or measured.
	/// Also called observation matrix.
	///
	MatrixMbyN myHMatrix;

	///
	/// @brief Contains the current covariance estimate for each state variable.
	/// Also called predicted covariance matrix.
	///
	MatrixNbyN myPMatrix;

	///
	/// @brief Defines the process error covariance estimates for each state
	/// variable. Also called process error covariance matrix.
	///
	MatrixNbyN myQMatrix;

	///
	/// @brief Defines the measurement error covariance estimates for each state
	/// variable. Also called measurement error covariance matrix.
	///
	MatrixMbyM myRMatrix;
};

}; // namespace Controls

}; // namespace Plat4m

#endif // PLAT4M_KALMAN_FILTER_H
