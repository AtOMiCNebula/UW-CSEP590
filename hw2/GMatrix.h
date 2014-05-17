// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Modified for CSE P590

#pragma once

// Matrix operations are applied on the left.  For example, given a matrix M
// and a vector V, matrix-times-vector is M*V.  That is, V is treated as a
// column vector.  Some graphics APIs use V*M where V is treated as a row
// vector.  In this context the "M" matrix is really a transpose of the M as
// represented in Wild Magic.  Similarly, to apply two matrix operations M0
// and M1, in that order, you compute M1*M0 so that the transform of a vector
// is (M1*M0)*V = M1*(M0*V).  Some graphics APIs use M0*M1, but again these
// matrices are the transpose of those as represented in Wild Magic.  You
// must therefore be careful about how you interface the transformation code
// with graphics APIS.
//
// Matrices are stored in row-major order, matrix[row][col].

#include "GVector.h"

template <class Real>
class GMatrix
{
public:
    // construction and destruction
    GMatrix (int iRows = 0, int iCols = 0);
    GMatrix (int iRows, int iCols, const Real* afData);
    GMatrix (int iRows, int iCols, const Real** aafEntry);
    GMatrix (const GMatrix& rkM);
	GMatrix (const GVector<Real>& rkV);
    ~GMatrix ();

    // member access
    void setSize (int iRows, int iCols);
    void getSize (int& riRows, int& riCols) const;
    int getNumRows () const;
    int getNumCols () const;
    int getQuantity () const;
    operator const Real* () const;
    operator Real* ();
    const Real* operator[] (int iRow) const;
    Real* operator[] (int iRow);
    void swapRows (int iRow0, int iRow1);
    const Real& operator() (int iRow, int iCol) const;
    Real& operator() (int iRow, int iCol);
    void setRow (int iRow, const GVector<Real>& rkV);
    GVector<Real> getRow (int iRow) const;
    void setCol (int iCol, const GVector<Real>& rkV);
    GVector<Real> getCol (int iCol) const;
	GMatrix<Real> getBlock(int rowA, int rowB, int colA, int colB) const;
    void setMatrix (int iRows, int iCols, const Real* afEntry);
    void setMatrix (int iRows, int iCols, const Real** aafMatrix);
    void getColumnMajor (Real* afCMajor) const;

	void setConstant (Real fConstant);
	void setIdentity ();
	
    // assignment
    GMatrix& operator= (const GMatrix& rkM);

    // arithmetic operations
    GMatrix operator+ (const GMatrix& rkM) const;
    GMatrix operator- (const GMatrix& rkM) const;
    GMatrix operator* (const GMatrix& rkM) const;

    GMatrix operator+ (Real fScalar) const;
    GMatrix operator- (Real fScalar) const;
    GMatrix operator* (Real fScalar) const;
    GMatrix operator/ (Real fScalar) const;
    GMatrix operator- () const;

    // matrix products
    GMatrix transpose () const;  // M^T

    // matrix-vector operations
    GVector<Real> operator* (const GVector<Real>& rkV) const;  // M * v

    // Inversion.  The matrix must be square.  The function returns true
    // whenever the matrix is square and invertible.
    GMatrix<Real> inverse () const;

protected:
    // Support for allocation and deallocation.  The allocation call requires
    // m_iRows, m_iCols, and m_iQuantity to have already been correctly
    // initialized.
    void allocate (bool bSetToZero);
    void deallocate ();

    int m_iRows, m_iCols, m_iQuantity;

    // the matrix is stored in row-major form as a 1-dimensional array
    Real* m_afData;

    // An array of pointers to the rows of the matrix.  The separation of
    // row pointers and actual data supports swapping of rows in linear
    // algebraic algorithms such as solving linear systems of equations.
    Real** m_aafEntry;
};

// c * M
template <class Real>
GMatrix<Real> operator* (Real fScalar, const GMatrix<Real>& rkM);

// v^T * M
template <class Real>
GVector<Real> operator* (const GVector<Real>& rkV, const GMatrix<Real>& rkM);

// u * v^T
template <class Real>
GMatrix<Real> operator* (const GVector<Real>& rkU, const GVector<Real>& rkV);



//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::GMatrix (int iRows, int iCols)
{
    m_afData = 0;
    m_aafEntry = 0;
    setSize(iRows,iCols);
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::GMatrix (int iRows, int iCols, const Real* afEntry)
{
    m_afData = 0;
    m_aafEntry = 0;
    setMatrix(iRows,iCols,afEntry);
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::GMatrix (int iRows, int iCols, const Real** aafMatrix)
{
    m_afData = 0;
    m_aafEntry = 0;
    setMatrix(iRows,iCols,aafMatrix);
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::GMatrix (const GMatrix& rkM)
{
    m_iRows = 0;
    m_iCols = 0;
    m_iQuantity = 0;
    m_afData = 0;
    m_aafEntry = 0;
    *this = rkM;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::GMatrix (const GVector<Real>& rkV)
{
	// create column matrix

    m_afData = 0;
    m_aafEntry = 0;
    setSize(rkV.getSize(), 1);
	setCol(0, rkV);
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::~GMatrix ()
{
    deallocate();
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::allocate (bool bSetToZero)
{
    // assert:  m_iRows, m_iCols, and m_iQuantity already initialized

    m_afData = new Real[m_iQuantity];
    if (bSetToZero)
    {
		for (int i = 0; i < m_iQuantity; i++) { m_afData[i] = (Real)0.0; }
    }

    m_aafEntry = new Real*[m_iRows];
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        m_aafEntry[iRow] = &m_afData[iRow*m_iCols];
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::deallocate ()
{
    delete [] m_afData;
    delete [] m_aafEntry;
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setSize (int iRows, int iCols)
{
    deallocate();
    if (iRows > 0 && iCols > 0)
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = m_iRows*m_iCols;
        allocate(true);
    }
    else
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = 0;
        m_afData = 0;
        m_aafEntry = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::getSize (int& riRows, int& riCols) const
{
    riRows = m_iRows;
    riCols = m_iCols;
}
//----------------------------------------------------------------------------
template <class Real>
int GMatrix<Real>::getNumRows () const
{
    return m_iRows;
}
//----------------------------------------------------------------------------
template <class Real>
int GMatrix<Real>::getNumCols () const
{
    return m_iCols;
}
//----------------------------------------------------------------------------
template <class Real>
int GMatrix<Real>::getQuantity () const
{
    return m_iQuantity;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::operator const Real* () const
{
    return m_afData;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>::operator Real* ()
{
    return m_afData;
}
//----------------------------------------------------------------------------
template <class Real>
const Real* GMatrix<Real>::operator[] (int iRow) const
{
    assert(0 <= iRow && iRow < m_iRows);
    return m_aafEntry[iRow];
}
//----------------------------------------------------------------------------
template <class Real>
Real* GMatrix<Real>::operator[] (int iRow)
{
    assert(0 <= iRow && iRow < m_iRows);
    return m_aafEntry[iRow];
}
//----------------------------------------------------------------------------
template <class Real>
const Real& GMatrix<Real>::operator() (int iRow, int iCol) const
{
    return m_aafEntry[iRow][iCol];
}
//----------------------------------------------------------------------------
template <class Real>
Real& GMatrix<Real>::operator() (int iRow, int iCol)
{
    assert(0 <= iRow && iRow < m_iRows && 0 <= iCol && iCol <= m_iCols);
    return m_aafEntry[iRow][iCol];
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::swapRows (int iRow0, int iRow1)
{
    assert(0 <= iRow0 && iRow0 < m_iRows && 0 <= iRow1 && iRow1 < m_iRows);
    Real* afSave = m_aafEntry[iRow0];
    m_aafEntry[iRow0] = m_aafEntry[iRow1];
    m_aafEntry[iRow1] = afSave;
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setRow (int iRow, const GVector<Real>& rkV)
{
    assert((0 <= iRow && iRow < m_iRows) && (rkV.getSize() == m_iCols));
    for (int iCol = 0; iCol < m_iCols; iCol++)
    {
        m_aafEntry[iRow][iCol] = rkV[iCol];
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GMatrix<Real>::getRow (int iRow) const
{
    assert(0 <= iRow && iRow < m_iRows);
    GVector<Real> kV(m_iCols);
    for (int iCol = 0; iCol < m_iCols; iCol++)
    {
        kV[iCol] = m_aafEntry[iRow][iCol];
    }
    return kV;
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setCol (int iCol, const GVector<Real>& rkV)
{
    assert((0 <= iCol && iCol < m_iCols) && (rkV.getSize() == m_iRows));
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        m_aafEntry[iRow][iCol] = rkV[iRow];
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GMatrix<Real>::getCol (int iCol) const
{
    assert(0 <= iCol && iCol < m_iCols);
    GVector<Real> kV(m_iRows);
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        kV[iRow] = m_aafEntry[iRow][iCol];
    }
    return kV;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::getBlock(int rowA, int rowB, int colA, int colB) const
{
	assert(rowA <= m_iRows && rowB <= m_iRows && colA <= m_iCols && colB <= m_iCols);

	GMatrix<Real> A(rowB-rowA, colB-colA);

    for (int iRow = rowA; iRow < rowB; iRow++)
    {
        for (int iCol = colA; iCol < colB; iCol++)
        {
			A.m_aafEntry[iRow-rowA][iCol-colA] = m_aafEntry[iRow][iCol];
        }
    }

	return A;
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setMatrix (int iRows, int iCols, const Real* afData)
{
    deallocate();
    if (iRows > 0 && iCols > 0)
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = m_iRows*m_iCols;
        allocate(false);
	// unused
	//        size_t uiSize = m_iQuantity*sizeof(Real);
	for (int i = 0; i < m_iQuantity; i++) { m_afData[i] = afData[i]; }
    }
    else
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = 0;
        m_afData = 0;
        m_aafEntry = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setMatrix (int iRows, int iCols, const Real** aafEntry)
{
    deallocate();
    if (iRows > 0 && iCols > 0)
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = m_iRows*m_iCols;
        allocate(false);
        for (int iRow = 0; iRow < m_iRows; iRow++)
        {
            for (int iCol = 0; iCol < m_iCols; iCol++)
            {
                m_aafEntry[iRow][iCol] = aafEntry[iRow][iCol];
            }
        }
    }
    else
    {
        m_iRows = iRows;
        m_iCols = iCols;
        m_iQuantity = 0;
        m_afData = 0;
        m_aafEntry = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::getColumnMajor (Real* afCMajor) const
{
    for (int iRow = 0, i = 0; iRow < m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < m_iCols; iCol++)
        {
            m_aafEntry[iRow][iCol] = m_aafEntry[iCol][iRow];
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setConstant (Real fConstant)
{
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < m_iCols; iCol++)
        {
            m_aafEntry[iRow][iCol] = fConstant;
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
void GMatrix<Real>::setIdentity ()
{
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < m_iCols; iCol++)
        {
			m_aafEntry[iRow][iCol] = (iRow == iCol) ? (Real)1.0 : (Real)0.0;
        }
    }
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real>& GMatrix<Real>::operator= (const GMatrix& rkM)
{
    if (rkM.m_iQuantity > 0)
    {
        if (m_iRows != rkM.m_iRows || m_iCols != rkM.m_iCols)
        {
            deallocate();
            m_iRows = rkM.m_iRows;
            m_iCols = rkM.m_iCols;
            m_iQuantity = rkM.m_iQuantity;
            allocate(false);
        }
        for (int iRow = 0; iRow < m_iRows; iRow++)
        {
            for (int iCol = 0; iCol < m_iCols; iCol++)
            {
                m_aafEntry[iRow][iCol] = rkM.m_aafEntry[iRow][iCol];
            }
        }
    }
    else
    {
        deallocate();
        m_iRows = rkM.m_iRows;
        m_iCols = rkM.m_iCols;
        m_iQuantity = 0;
        m_afData = 0;
        m_aafEntry = 0;
    }
    return *this;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator+ (const GMatrix& rkM) const
{
	assert(m_iRows == rkM.m_iRows);
	assert(m_iCols == rkM.m_iCols);

    GMatrix<Real> kSum(rkM.m_iRows,rkM.m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kSum.m_afData[i] = m_afData[i] + rkM.m_afData[i];
    }
    return kSum;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator- (const GMatrix& rkM) const
{
	assert(m_iRows == rkM.m_iRows);
	assert(m_iCols == rkM.m_iCols);

    GMatrix<Real> kDiff(rkM.m_iRows,rkM.m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kDiff.m_afData[i] = m_afData[i] - rkM.m_afData[i];
    }
    return kDiff;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator* (const GMatrix& rkM) const
{
    // 'this' is RxN, 'M' is NxC, 'product = this*M' is RxC
    assert(m_iCols == rkM.m_iRows);
    GMatrix<Real> kProd(m_iRows,rkM.m_iCols);
    for (int iRow = 0; iRow < kProd.m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < kProd.m_iCols; iCol++)
        {
            for (int iMid = 0; iMid < m_iCols; iMid++)
            {
                kProd.m_aafEntry[iRow][iCol] += m_aafEntry[iRow][iMid] *
                    rkM.m_aafEntry[iMid][iCol];
            }
        }
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator+ (Real fScalar) const
{
    GMatrix<Real> kProd(m_iRows,m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kProd.m_afData[i] = m_afData[i]+fScalar;
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator- (Real fScalar) const
{
    GMatrix<Real> kProd(m_iRows,m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kProd.m_afData[i] = m_afData[i]-fScalar;
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator* (Real fScalar) const
{
    GMatrix<Real> kProd(m_iRows,m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kProd.m_afData[i] = fScalar*m_afData[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator/ (Real fScalar) const
{
    GMatrix<Real> kQuot(m_iRows,m_iCols);
    int i;

    if (fScalar != (Real)0.0)
    {
        Real fInvScalar = ((Real)1.0)/fScalar;
        for (i = 0; i < m_iQuantity; i++)
        {
            kQuot.m_afData[i] = fInvScalar*m_afData[i];
        }
    }
    else
    {
        for (i = 0; i < m_iQuantity; i++)
        {
            kQuot.m_afData[i] = FLT_MAX;
        }
    }

    return kQuot;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::operator- () const
{
    GMatrix<Real> kNeg(m_iRows,m_iCols);
    for (int i = 0; i < m_iQuantity; i++)
    {
        kNeg.m_afData[i] = -m_afData[i];
    }
    return kNeg;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> operator* (Real fScalar, const GMatrix<Real>& rkM)
{
    GMatrix<Real> kProd(rkM.getNumRows(),rkM.getNumCols());
    const Real* afMEntry = rkM;
    Real* afPEntry = kProd;
    for (int i = 0; i < rkM.getQuantity(); i++)
    {
        afPEntry[i] = fScalar*afMEntry[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::transpose () const
{
    GMatrix<Real> kTranspose(m_iCols,m_iRows);
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < m_iCols; iCol++)
        {
            kTranspose.m_aafEntry[iCol][iRow] = m_aafEntry[iRow][iCol];
        }
    }
    return kTranspose;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GMatrix<Real>::operator* (const GVector<Real>& rkV) const
{
    assert(rkV.getSize() == m_iCols);
    GVector<Real> kProd(m_iRows);
    for (int iRow = 0; iRow < m_iRows; iRow++)
    {
        for (int iCol = 0; iCol < m_iCols; iCol++)
        {
            kProd[iRow] += m_aafEntry[iRow][iCol]*rkV[iCol];
        }
            
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> operator* (const GVector<Real>& rkV, const GMatrix<Real>& rkM)
{
    assert(rkV.getSize() == rkM.getNumRows());
    GVector<Real> kProd(rkM.getNumCols());
    Real* afPEntry = kProd;
    for (int iCol = 0; iCol < rkM.getNumCols(); iCol++)
    {
        for (int iRow = 0; iRow < rkM.getNumRows(); iRow++)
        {
            afPEntry[iCol] += rkV[iRow]*rkM[iRow][iCol];
        }
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> operator* (const GVector<Real>& rkU, const GVector<Real>& rkV)
{
    GMatrix<Real> kProd(rkU.getSize(),rkV.getSize());
    for (int iRow = 0; iRow < kProd.getNumRows(); iRow++)
    {
        for (int iCol = 0; iCol < kProd.getNumCols(); iCol++)
        {
			kProd[iRow][iCol] = rkU[iRow] * rkV[iCol];
        }
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GMatrix<Real> GMatrix<Real>::inverse () const
{
	GMatrix<Real> rkInverse;

    // computations are performed in-place
    if (getNumRows() > 0 && getNumRows() != getNumCols())
    {
        return GMatrix<Real>();
    }

    int iSize = getNumRows();
    rkInverse = *this;

    int* aiColIndex = new int[iSize];
    int* aiRowIndex = new int[iSize];
    bool* abPivoted = new bool[iSize];
    memset(abPivoted,0,iSize*sizeof(bool));

    int i1, i2, iRow = 0, iCol = 0;
    Real fSave;

    // elimination by full pivoting
    for (int i0 = 0; i0 < iSize; i0++)
    {
        // search matrix (excluding pivoted rows) for maximum absolute entry
        Real fMax = (Real)0.0;
        for (i1 = 0; i1 < iSize; i1++)
        {
            if (!abPivoted[i1])
            {
                for (i2 = 0; i2 < iSize; i2++)
                {
                    if (!abPivoted[i2])
                    {
                        Real abs = fabs(rkInverse[i1][i2]);
                        if (abs > fMax)
                        {
                            fMax = abs;
                            iRow = i1;
                            iCol = i2;
                        }
                    }
                }
            }
        }

        if (fMax == (Real)0.0)
        {
            // matrix is not invertible
            delete [] aiColIndex;
            delete [] aiRowIndex;
            delete [] abPivoted;
            return GMatrix<Real>();
        }

        abPivoted[iCol] = true;

        // swap rows so that A[iCol][iCol] contains the pivot entry
        if (iRow != iCol)
        {
            rkInverse.swapRows(iRow,iCol);
        }

        // keep track of the permutations of the rows
        aiRowIndex[i0] = iRow;
        aiColIndex[i0] = iCol;

        // scale the row so that the pivot entry is 1
        Real fInv = ((Real)1.0)/rkInverse[iCol][iCol];
        rkInverse[iCol][iCol] = (Real)1.0;
        for (i2 = 0; i2 < iSize; i2++)
        {
            rkInverse[iCol][i2] *= fInv;
        }

        // zero out the pivot column locations in the other rows
        for (i1 = 0; i1 < iSize; i1++)
        {
            if (i1 != iCol)
            {
                fSave = rkInverse[i1][iCol];
                rkInverse[i1][iCol] = (Real)0.0;
                for (i2 = 0; i2 < iSize; i2++)
                {
                    rkInverse[i1][i2] -= rkInverse[iCol][i2]*fSave;
                }
            }
        }
    }

    // reorder rows so that A[][] stores the inverse of the original matrix
    for (i1 = iSize-1; i1 >= 0; i1--)
    {
        if (aiRowIndex[i1] != aiColIndex[i1])
        {
            for (i2 = 0; i2 < iSize; i2++)
            {
                fSave = rkInverse[i2][aiRowIndex[i1]];
                rkInverse[i2][aiRowIndex[i1]] =
                    rkInverse[i2][aiColIndex[i1]];
                rkInverse[i2][aiColIndex[i1]] = fSave;
            }
        }
    }

    delete [] aiColIndex;
    delete [] aiRowIndex;
    delete [] abPivoted;
    return rkInverse;
}