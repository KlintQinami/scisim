// RigidBodyPolyline.cpp
//
// Klint Qinami
// Last updated: 03/25/2017

#include "RigidBodyPolyline.h"

#include <iostream>


RigidBodyPolyline::RigidBodyPolyline( std::istream& input_stream ) {
    std::cerr << "Code up RigidBodyPolyline input stream constructor" << std::endl;
    std::exit( EXIT_FAILURE );
}


RigidBodyPolyline::RigidBodyPolyline( const VectorXs& vertices, const scalar& r )
: m_r ( r )
, m_p (  ) {
    assert( r > 0 );
    assert( vertices.size() % 3 == 0 );
    m_p.resize( vertices.size() / 3 );
    for (int i = 0; i < m_p.size(); i++) {
        m_p[i] << vertices[3*i], vertices[3*i + 1], vertices[3*i + 2];
    }
}


RigidBodyPolyline::RigidBodyPolyline( const RigidBodyPolyline& other ) 
: m_p ( other.m_p )
, m_r ( other.m_r )
{}


RigidBodyPolyline::~RigidBodyPolyline()
{}


RigidBodyGeometryType RigidBodyPolyline::getType() const {
    return RigidBodyGeometryType::POLYLINE;
} 


std::unique_ptr<RigidBodyGeometry> RigidBodyPolyline::clone() const {
    return std::unique_ptr<RigidBodyGeometry>{ new RigidBodyPolyline{ *this } };
}


void RigidBodyPolyline::computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const {
    min.setConstant(  std::numeric_limits<scalar>::infinity() );
    max.setConstant( -std::numeric_limits<scalar>::infinity() );

    for( int i = 0; i < m_p.size(); i++ )
    {
        const Array3s transformed_vertex{ R * m_p[i] + cm };
        min = min.min( transformed_vertex );
        max = max.max( transformed_vertex );
    }
    assert( ( min < max ).all() );
}


void RigidBodyPolyline::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const {
    I.setIdentity();
    R.setIdentity();
    CM.setZero();
    for (int i = 0; i < m_p.size(); i++) {
       CM += m_p[i];
    }
    CM /= m_p.size(); 
    CM .setZero();
    M = 1.0;
}


std::string RigidBodyPolyline::name() const {
    return "polyline";
}


void RigidBodyPolyline::serialize( std::ostream& output_stream ) const {
    std::cerr << "Code up RigidBodyPolyline serialization" << std::endl;
    std::exit( EXIT_FAILURE );
}


scalar RigidBodyPolyline::volume() const {
    std::cerr << "Code up RigidBodyPolyline volume" << std::endl;
    std::exit( EXIT_FAILURE );
}


void RigidBodyPolyline::computeInertia( const scalar& M, Vector3s& I ) const {
    I.setIdentity();
}


const std::vector<Vector3s>& RigidBodyPolyline::points() const {
  return m_p;
}


const scalar& RigidBodyPolyline::r() const
{
  return m_r;
}

