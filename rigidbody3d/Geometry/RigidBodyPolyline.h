// RigidBodyPolyline.h
//
// Klint Qinami
// Last updated: 03/22/2017

#ifndef RIGID_BODY_POLYLINE
#define RIGID_BODY_POLYLINE

#include "RigidBodyGeometry.h"

class RigidBodyPolyline final : public RigidBodyGeometry
{

public:

  explicit RigidBodyPolyline( std::istream& input_stream );

  explicit RigidBodyPolyline( const VectorXs& vertices, const scalar& r );

  explicit RigidBodyPolyline( const RigidBodyPolyline& other );

  virtual ~RigidBodyPolyline() override;

  virtual RigidBodyGeometryType getType() const override;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const override;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const override;

  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual scalar volume() const override;

  const std::vector<Vector3s>& points() const;

  const scalar& r() const;

private:
  std::vector<Vector3s> m_p;
  scalar m_r;

  void computeInertia( const scalar& M, Vector3s& I ) const;
 };

#endif
