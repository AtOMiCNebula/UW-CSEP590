// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Modified for CSE P590

#pragma once

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

template <class Real>
class GVector
{
public:
    // construction
    GVector (int iSize = 0);
    GVector (int iSize, const Real* afTuple);
    GVector (const GVector& rkV);
    ~GVector ();

    // coordinate access
    void setSize (int iSize);
    int getSize () const;
    operator const Real* () const;
    operator Real* ();
    const Real& operator[] (int i) const;
    Real& operator[] (int i);

	void setConstant (Real fConstant);

    // assignment
    GVector& operator= (const GVector& rkV);
	
    // arithmetic operations
    GVector operator+ (const GVector& rkV) const;
    GVector operator- (const GVector& rkV) const;
    GVector operator+ (Real fScalar) const;
    GVector operator- (Real fScalar) const;
    GVector operator* (Real fScalar) const;
    GVector operator/ (Real fScalar) const;
    GVector operator- () const;
	
    // vector operations
    Real length () const;
    Real dot (const GVector& rkV) const;
    Real normalize ();
	
protected:

    int m_iSize;
    Real* m_afTuple;
};

//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::GVector (int iSize)
{
    if (iSize > 0)
    {
        m_iSize = iSize;
        m_afTuple = new Real[m_iSize];

		for (int i = 0; i < m_iSize; i++) { m_afTuple[i] = (Real)0.0; }
    }
    else
    {
        m_iSize = 0;
        m_afTuple = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::GVector (int iSize, const Real* afTuple)
{
    if (iSize > 0)
    {
        m_iSize = iSize;
        m_afTuple = new Real[m_iSize];
        size_t uiSize = m_iSize*sizeof(Real);

		for (int i = 0; i < m_iSize; i++) { m_afTuple[i] = afTuple[i]; }
    }
    else
    {
        m_iSize = 0;
        m_afTuple = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::GVector (const GVector& rkV)
{
    m_iSize = rkV.m_iSize;
    if (m_iSize > 0)
    {
        m_afTuple = new Real[m_iSize];
        // size_t uiSize = m_iSize*sizeof(Real);

	for (int i = 0; i < m_iSize; i++) { m_afTuple[i] = rkV.m_afTuple[i]; }
    }
    else
    {
        m_afTuple = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::~GVector ()
{
    delete [] m_afTuple;
}
//----------------------------------------------------------------------------
template <class Real>
void GVector<Real>::setSize (int iSize)
{
    delete [] m_afTuple;
    if (iSize > 0)
    {
        m_iSize = iSize;
        m_afTuple = new Real[m_iSize];
        for (int i = 0; i < m_iSize; i++) { m_afTuple[i] = (Real)0.0; }
    }
    else
    {
        m_iSize = 0;
        m_afTuple = 0;
    }
}
//----------------------------------------------------------------------------
template <class Real>
int GVector<Real>::getSize () const
{
    return m_iSize;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::operator const Real* () const
{
    return m_afTuple;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>::operator Real* ()
{
    return m_afTuple;
}
//----------------------------------------------------------------------------
template <class Real>
const Real& GVector<Real>::operator[] (int i) const
{
    assert(0 <= i && i < m_iSize);
    return m_afTuple[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real& GVector<Real>::operator[] (int i)
{
    assert(0 <= i && i < m_iSize);
    return m_afTuple[i];
}
//----------------------------------------------------------------------------
template <class Real>
void GVector<Real>::setConstant(Real fConstant)
{
    for (int i = 0; i < m_iSize; i++)
    {
        m_afTuple[i] = fConstant;
    }
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real>& GVector<Real>::operator= (const GVector& rkV)
{
    if (rkV.m_iSize > 0)
    {
        if (m_iSize != rkV.m_iSize)
        {
            if(m_iSize > 0) 
            {
                delete [] m_afTuple;
            }
            m_iSize = rkV.m_iSize;
            m_afTuple = new Real[m_iSize];
        }
	// unused
        // size_t uiSize = m_iSize*sizeof(Real);
	for (int i = 0; i < m_iSize; i++) { m_afTuple[i] = rkV.m_afTuple[i]; }
    }
    else
    {
        delete [] m_afTuple;
        m_iSize = 0;
        m_afTuple = 0;
    }
    return *this;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator+ (const GVector& rkV) const
{
	assert(m_iSize == rkV.m_iSize);

    GVector<Real> kSum(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kSum.m_afTuple[i] = m_afTuple[i] + rkV.m_afTuple[i];
    }
    return kSum;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator- (const GVector& rkV) const
{
	assert(m_iSize == rkV.m_iSize);

    GVector<Real> kDiff(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kDiff.m_afTuple[i] = m_afTuple[i] - rkV.m_afTuple[i];
    }
    return kDiff;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator+ (Real fScalar) const
{
    GVector<Real> kProd(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kProd.m_afTuple[i] = fScalar+m_afTuple[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator- (Real fScalar) const
{
    GVector<Real> kProd(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kProd.m_afTuple[i] = fScalar-m_afTuple[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator* (Real fScalar) const
{
    GVector<Real> kProd(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kProd.m_afTuple[i] = m_afTuple[i]*fScalar;
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator/ (Real fScalar) const
{
    GVector<Real> kQuot(m_iSize);
    int i;

    if (fScalar != (Real)0.0)
    {
        Real fInvScalar = ((Real)1.0)/fScalar;
        for (i = 0; i < m_iSize; i++)
        {
            kQuot.m_afTuple[i] = m_afTuple[i]*fInvScalar;
        }
    }
    else
    {
        for (i = 0; i < m_iSize; i++)
        {
            kQuot.m_afTuple[i] = FLT_MAX;
        }
    }

    return kQuot;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> GVector<Real>::operator- () const
{
    GVector<Real> kNeg(m_iSize);
    for (int i = 0; i < m_iSize; i++)
    {
        kNeg.m_afTuple[i] = -m_afTuple[i];
    }
    return kNeg;
}
//----------------------------------------------------------------------------
template <class Real>
GVector<Real> operator* (Real fScalar, const GVector<Real>& rkV)
{
    GVector<Real> kProd(rkV.getSize());
    for (int i = 0; i < rkV.getSize(); i++)
    {
        kProd[i] = fScalar*rkV[i];
    }
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
Real GVector<Real>::length () const
{
    Real fSqrLen = (Real)0.0;
    for (int i = 0; i < m_iSize; i++)
    {
        fSqrLen += m_afTuple[i]*m_afTuple[i];
    }
    return sqrt(fSqrLen);
}
//----------------------------------------------------------------------------
template <class Real>
Real GVector<Real>::dot (const GVector& rkV) const
{
	assert(m_iSize == rkV.m_iSize);

    Real fDot = (Real)0.0;
    for (int i = 0; i < m_iSize; i++)
    {
        fDot += m_afTuple[i]*rkV.m_afTuple[i];
    }
    return fDot;
}
//----------------------------------------------------------------------------
template <class Real>
Real GVector<Real>::normalize ()
{
    Real fLength = length();
    int i;

    if (fLength > FLT_EPSILON)
    {
        Real fInvLength = ((Real)1.0)/fLength;
        for (i = 0; i < m_iSize; i++)
        {
            m_afTuple[i] *= fInvLength;
        }
    }
    else
    {
        fLength = (Real)0.0;
        for (i = 0; i < m_iSize; i++)
        {
            m_afTuple[i] = (Real)0.0;
        }
    }

    return fLength;
}