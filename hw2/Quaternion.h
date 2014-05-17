// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Modified for CSE P590

#pragma once

#include "GVector.h"

template <class Real>
class Quaternion
{
public:
    // A quaternion is q = w + x*i + y*j + z*k where (w,x,y,z) is not
    // necessarily a unit length vector in 4D.

    // construction
    Quaternion ();  // uninitialized
    Quaternion (Real w, Real x, Real y, Real z);
	Quaternion (Real *wxyz);
    Quaternion (const Quaternion& q);

    // member access:  0 = w, 1 = x, 2 = y, 3 = z
    operator const Real* () const;
    operator Real* ();
    Real operator[] (int i) const;
    Real& operator[] (int i);
    Real w () const;
    Real& w ();
    Real x () const;
    Real& x ();
    Real y () const;
    Real& y ();
    Real z () const;
    Real& z ();

    // assignment and comparison
    Quaternion& operator= (const Quaternion& q);

    // arithmetic operations
    Quaternion operator+ (const Quaternion& q) const;
    Quaternion operator- (const Quaternion& q) const;
    Quaternion operator* (const Quaternion& q) const;
    Quaternion operator* (Real k) const;
    Quaternion operator/ (Real k) const;
    Quaternion operator- () const;

    // functions of a quaternion
    Real dot (const Quaternion& q) const;  // dot product
    Quaternion inverse () const;  // apply to non-zero quaternion
	Real length() const;
	Quaternion normalize () const;

protected:

    Real m_tuple[4];
};

// returns a 3d vector that is a difference between two quaternions qa and qb
template <class Real>
GVector<Real> quatdiff (const Quaternion<Real> &qa, const Quaternion<Real> &qb);


//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::Quaternion ()
{
    // the object is uninitialized
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::Quaternion (Real fW, Real fX, Real fY, Real fZ)
{
    m_tuple[0] = fW;
    m_tuple[1] = fX;
    m_tuple[2] = fY;
    m_tuple[3] = fZ;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::Quaternion (Real *fWXYZ)
{
	memcpy(m_tuple,fWXYZ,4*sizeof(Real));
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::Quaternion (const Quaternion& rkQ)
{
    memcpy(m_tuple,rkQ.m_tuple,4*sizeof(Real));
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::operator const Real* () const
{
    return m_tuple;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>::operator Real* ()
{
    return m_tuple;
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::operator[] (int i) const
{
    assert( 0 <= i && i < 4 );
    return m_tuple[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real& Quaternion<Real>::operator[] (int i)
{
    assert( 0 <= i && i < 4 );
    return m_tuple[i];
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::w () const
{
    return m_tuple[0];
}
//----------------------------------------------------------------------------
template <class Real>
Real& Quaternion<Real>::w ()
{
    return m_tuple[0];
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::x () const
{
    return m_tuple[1];
}
//----------------------------------------------------------------------------
template <class Real>
Real& Quaternion<Real>::x ()
{
    return m_tuple[1];
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::y () const
{
    return m_tuple[2];
}
//----------------------------------------------------------------------------
template <class Real>
Real& Quaternion<Real>::y ()
{
    return m_tuple[2];
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::z () const
{
    return m_tuple[3];
}
//----------------------------------------------------------------------------
template <class Real>
Real& Quaternion<Real>::z ()
{
    return m_tuple[3];
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real>& Quaternion<Real>::operator= (const Quaternion& rkQ)
{
    memcpy(m_tuple,rkQ.m_tuple,4*sizeof(Real));
    return *this;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator+ (const Quaternion& rkQ) const
{
    Quaternion<Real> kSum;
    for (int i = 0; i < 4; i++)
        kSum.m_tuple[i] = m_tuple[i] + rkQ.m_tuple[i];
    return kSum;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator- (const Quaternion& rkQ) const
{
    Quaternion<Real> kDiff;
    for (int i = 0; i < 4; i++)
        kDiff.m_tuple[i] = m_tuple[i] - rkQ.m_tuple[i];
    return kDiff;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator* (const Quaternion& rkQ) const
{
    // NOTE:  Multiplication is not generally commutative, so in most
    // cases p*q != q*p.

    Quaternion kProd;

    kProd.m_tuple[0] =
        m_tuple[0]*rkQ.m_tuple[0] -
        m_tuple[1]*rkQ.m_tuple[1] -
        m_tuple[2]*rkQ.m_tuple[2] -
        m_tuple[3]*rkQ.m_tuple[3];

    kProd.m_tuple[1] =
        m_tuple[0]*rkQ.m_tuple[1] +
        m_tuple[1]*rkQ.m_tuple[0] +
        m_tuple[2]*rkQ.m_tuple[3] -
        m_tuple[3]*rkQ.m_tuple[2];

    kProd.m_tuple[2] =
        m_tuple[0]*rkQ.m_tuple[2] +
        m_tuple[2]*rkQ.m_tuple[0] +
        m_tuple[3]*rkQ.m_tuple[1] -
        m_tuple[1]*rkQ.m_tuple[3];

    kProd.m_tuple[3] =
        m_tuple[0]*rkQ.m_tuple[3] +
        m_tuple[3]*rkQ.m_tuple[0] +
        m_tuple[1]*rkQ.m_tuple[2] -
        m_tuple[2]*rkQ.m_tuple[1];

    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator* (Real fScalar) const
{
    Quaternion<Real> kProd;
    for (int i = 0; i < 4; i++)
        kProd.m_tuple[i] = fScalar*m_tuple[i];
    return kProd;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator/ (Real fScalar) const
{
    Quaternion<Real> kQuot;
    int i;

    if ( fScalar != (Real)0.0 )
    {
        Real fInvScalar = ((Real)1.0)/fScalar;
        for (i = 0; i < 4; i++)
            kQuot.m_tuple[i] = fInvScalar*m_tuple[i];
    }
    else
    {
        for (i = 0; i < 4; i++)
            kQuot.m_tuple[i] = FLT_MAX;
    }

    return kQuot;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::operator- () const
{
    Quaternion<Real> kNeg;
    for (int i = 0; i < 4; i++)
        kNeg.m_tuple[i] = -m_tuple[i];
    return kNeg;
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::dot (const Quaternion& rkQ) const
{
    Real fDot = (Real)0.0;
    for (int i = 0; i < 4; i++)
        fDot += m_tuple[i]*rkQ.m_tuple[i];
    return fDot;
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::inverse () const
{
    Quaternion<Real> kInverse;

    Real fNorm = (Real)0.0;
    int i;
    for (i = 0; i < 4; i++)
        fNorm += m_tuple[i]*m_tuple[i];

    if ( fNorm > (Real)0.0 )
    {
        Real fInvNorm = ((Real)1.0)/fNorm;
        kInverse.m_tuple[0] = m_tuple[0]*fInvNorm;
        kInverse.m_tuple[1] = -m_tuple[1]*fInvNorm;
        kInverse.m_tuple[2] = -m_tuple[2]*fInvNorm;
        kInverse.m_tuple[3] = -m_tuple[3]*fInvNorm;
    }
    else
    {
        // return an invalid result to flag the error
        for (i = 0; i < 4; i++)
            kInverse.m_tuple[i] = (Real)0.0;
    }

    return kInverse;
}
//----------------------------------------------------------------------------
template <class Real>
Real Quaternion<Real>::length () const
{
    return sqrt(
        m_tuple[0]*m_tuple[0] +
        m_tuple[1]*m_tuple[1] +
        m_tuple[2]*m_tuple[2] +
        m_tuple[3]*m_tuple[3]);
}
//----------------------------------------------------------------------------
template <class Real>
Quaternion<Real> Quaternion<Real>::normalize () const
{
	Quaternion<Real> Q = *this;

    Real fLength = length();

    if (fLength > FLT_EPSILON)
    {
        Real fInvLength = ((Real)1.0)/fLength;
        Q.m_tuple[0] *= fInvLength;
        Q.m_tuple[1] *= fInvLength;
        Q.m_tuple[2] *= fInvLength;
        Q.m_tuple[3] *= fInvLength;
    }
    else
    {
        fLength = (Real)0.0;
        Q.m_tuple[0] = (Real)1.0;
        Q.m_tuple[1] = (Real)0.0;
        Q.m_tuple[2] = (Real)0.0;
        Q.m_tuple[3] = (Real)0.0;
    }

    return Q;
}

//----------------------------------------------------------------------------
template <class Real>
GVector<Real> quatdiff (const Quaternion<Real> &qa, const Quaternion<Real> &qb)
{
	Quaternion<Real> qdiff = (qb * qa.inverse());
	GVector<Real> vdiff(3);
	vdiff[0] = qdiff.x();
	vdiff[1] = qdiff.y();
	vdiff[2] = qdiff.z();

	Real sina2 = vdiff.normalize();
	Real speed = (Real)2.0 * atan2(sina2, qdiff.w());
	return vdiff * speed;
}