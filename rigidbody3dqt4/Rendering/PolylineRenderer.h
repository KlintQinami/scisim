#ifndef POLYLINE_RENDERER_H
#define POLYLINE_RENDERER_H

#include "BodyGeometryRenderer.h"

#include "OpenGL3DSphereRenderer.h"

#include "scisim/Math/MathDefines.h"

class PolylineRenderer final : public BodyGeometryRenderer
{

public:

  PolylineRenderer( const int num_subdivs, const std::vector<Vector3s>& points, const scalar& r );
  virtual ~PolylineRenderer() override;

  virtual void renderBody( const Eigen::Matrix<GLfloat,3,1>& color ) override;

private:

  void drawCylinder();

  int computeNumSamples( const int num_subdivs ) const;

  void initializeCylinderMemory();

  const int m_num_samples;
  const std::vector<Eigen::Matrix<GLfloat,3,1>> m_points;
  const GLfloat m_r;

  OpenGL3DSphereRenderer m_sphere_renderer;

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_cylinder_verts;
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_cylinder_normals;

};

#endif
