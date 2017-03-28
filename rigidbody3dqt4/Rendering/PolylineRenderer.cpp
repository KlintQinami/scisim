#include "PolylineRenderer.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <iostream>

static std::vector<Eigen::Matrix<GLfloat,3,1>> generatePointVector( const std::vector<Vector3s>& points )
{
  std::vector<Eigen::Matrix<GLfloat,3,1>> new_points( points.size() );
  for( std::vector<Vector3s>::size_type i = 0; i < points.size(); ++i )
  {
    new_points[i] = points[i].cast<GLfloat>();
  }
  return new_points;
}

PolylineRenderer::PolylineRenderer( const int num_subdivs, const std::vector<Vector3s>& points, const scalar& r )
: m_num_samples( computeNumSamples( num_subdivs ) )
, m_points( generatePointVector(points) )
, m_r( GLfloat( r ) )
, m_sphere_renderer( num_subdivs )
, m_cylinder_verts()
, m_cylinder_normals()
{
  assert( num_subdivs >= 0 );
  assert( m_r > 0.0f );

  initializeCylinderMemory();
}

PolylineRenderer::~PolylineRenderer()
{}

void PolylineRenderer::renderBody( const Eigen::Matrix<GLfloat,3,1>& color )
{
  GLfloat mcolorambient[] = { GLfloat( 0.3 ) * color.x(), GLfloat( 0.3 ) * color.y(), GLfloat( 0.3 ) * color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { color.x(), color.y(), color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  for( int i = 0; i < m_points.size(); ++i )
  {
    // Cap the vertices of the polyline
    glPushMatrix();
    glTranslatef( m_points[i].x(), m_points[i].y(), m_points[i].z() );
    glScalef( m_r, m_r, m_r );
    m_sphere_renderer.drawVertexArray( color, color );
    glPopMatrix();
  }

  for( int i = 0; i < m_points.size()-1; ++i ) {
    // Draw the cylinders connecting vertices
    glPushMatrix();
    Eigen::Matrix<GLfloat, 3, 1> e = m_points[i+1] - m_points[i]; 
    // Translate to midpoint of edge
    glTranslatef(m_points[i+1].x()+0.5f*(m_points[i].x()-m_points[i+1].x()), 
                 m_points[i+1].y()+0.5f*(m_points[i].y()-m_points[i+1].y()), 
                 m_points[i+1].z()+0.5f*(m_points[i].z()-m_points[i+1].z()));
    // Rotate about normal of plane spanning vectors with magnitude equal to 
    // inner product. We have e x [1 0 0] = [0 -e2 e1].
    float vx = m_points[i+1].x()-m_points[i].x();
    float vy = m_points[i+1].y()-m_points[i].y();
    float vz = m_points[i+1].z()-m_points[i].z();
    float norm = sqrt(vx*vx + vy*vy + vz*vz);
    float theta = 180.0 / 3.141592653589 * acos(vx / norm);
    if (vx < 0) {theta = -theta;}
    glRotatef(theta, 0.0, -vz*vx, vy*vx);
    glScalef(norm, 1.0, 1.0);
    drawCylinder();
    glPopMatrix();
  }
}

void PolylineRenderer::drawCylinder()
{
  const int nv{ 4 * m_num_samples };

  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_NORMAL_ARRAY );

  glVertexPointer( 3, GL_FLOAT, 0, m_cylinder_verts.data() );
  glNormalPointer( GL_FLOAT, 0, m_cylinder_normals.data() );

  glDrawArrays( GL_QUADS, 0, nv );

  glDisableClientState( GL_NORMAL_ARRAY );
  glDisableClientState( GL_VERTEX_ARRAY );
}

int PolylineRenderer::computeNumSamples( const int num_subdivs ) const
{
  int num_samples{ 4 };
  for( int i = 0; i < num_subdivs; ++i )
  {
    num_samples *= 2;
  }
  return num_samples;
}

void PolylineRenderer::initializeCylinderMemory()
{
  m_cylinder_verts.resize( 3 * 4 * m_num_samples );
  m_cylinder_normals.resize( 3 * 4 * m_num_samples );

  const GLfloat dtheta{ static_cast<GLfloat>( 2.0 ) * PI<GLfloat> / GLfloat( m_num_samples ) };

  using std::cos;
  using std::sin;
  for( int quad_num = 0; quad_num < m_num_samples; ++ quad_num )
  {
    const GLfloat c0{ m_r * cos( GLfloat(quad_num) * dtheta ) };
    const GLfloat s0{ m_r * sin( GLfloat(quad_num) * dtheta ) };
    const GLfloat c1{ m_r * cos( GLfloat( ( quad_num + 1 ) % m_num_samples ) * dtheta ) };
    const GLfloat s1{ m_r * sin( GLfloat( ( quad_num + 1 ) % m_num_samples ) * dtheta ) };

    m_cylinder_verts.segment<3>( 12 * quad_num + 0 ) = Eigen::Matrix<GLfloat,3,1>{ -0.5, c0, s0 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 3 ) = Eigen::Matrix<GLfloat,3,1>{ -0.5, c1, s1 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 6 ) = Eigen::Matrix<GLfloat,3,1>{  0.5, c1, s1 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 9 ) = Eigen::Matrix<GLfloat,3,1>{  0.5, c0, s0 };

    const Eigen::Matrix<GLfloat,3,1> n0{ Eigen::Matrix<GLfloat,3,1>{ 0.0, c0, s0 }.normalized() };
    const Eigen::Matrix<GLfloat,3,1> n1{ Eigen::Matrix<GLfloat,3,1>{ 0.0, c1, s1 }.normalized() };

    m_cylinder_normals.segment<3>( 12 * quad_num + 0 ) = n0;
    m_cylinder_normals.segment<3>( 12 * quad_num + 3 ) = n1;
    m_cylinder_normals.segment<3>( 12 * quad_num + 6 ) = n1;
    m_cylinder_normals.segment<3>( 12 * quad_num + 9 ) = n0;
  }
}
