#include <observation_tasks.h>

#include <Kin/frame.h>

#include <math_utility.h>

//===========================================================================

void HeadPoseMap::phi(arr& y, arr& J, const rai::KinematicWorld& G )
{
  rai::Frame *head = G.getFrameByName("manhead");
  arr posHead, JposHead;
  G.kinematicsPos(posHead, JposHead, head);    // get function to minimize and its jacobian in state G

  arr quatHead, JquatHead;
  G.kinematicsQuat(quatHead, JquatHead, head); // get function to minimize and its jacobian in state G

  // concatenate y and J from position and orientation (quaternion)
  arr tmp_y=zeros(dim_);
  arr tmp_J=zeros(dim_, JposHead.dim(1));
  tmp_y.setVectorBlock(posHead, 0);
  tmp_y.setVectorBlock(quatHead, posHead.dim(0) - 1);

  tmp_J.setMatrixBlock(JposHead, 0, 0);
  tmp_J.setMatrixBlock(JquatHead, JposHead.dim(0) - 1, 0);

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

arr HeadPoseMap::buildTarget( rai::Vector const& position, double yaw_deg )
{
  rai::Quaternion target_quat;
  target_quat.setDeg(yaw_deg, 0.0, 0.0, 1.0);

  arr target_arr(dim_);
  target_arr.setVectorBlock( position.getArr(), 0 );
  target_arr.setVectorBlock( conv_quat2arr(target_quat), position.getArr().dim(0) - 1 );

  return target_arr;
}

//===========================================================================

HeadGetSight::HeadGetSight( const arr& objectPosition, const arr& pivotPoint )
  : Feature()
  , objectPosition_( objectPosition )
  , pivotPoint_    ( pivotPoint )
  , headViewingDirection_( 0.0, -1.0, 0.0 )
  , moveAroundPivotDefined_( false )
{
  w1_ = objectPosition_ - pivotPoint_;

  if( norm2( w1_ ) > 0 ) // normalize if the vector is not null
  {
    w1_ = w1_ * 1. / norm2( w1_ );
    moveAroundPivotDefined_ = true;
  }

  // with current sensor position, determine which object occludes ( at least one triangle is traversed by the ray )

  // determine the pivot point
}

void HeadGetSight::phi(arr& y, arr& J, const rai::KinematicWorld& G )
{
  rai::Frame *head = G.getFrameByName("manhead");
  arr headPosition, headJPosition;
  G.kinematicsPos( headPosition, headJPosition, head );

  arr headViewingDirection, headJViewingDirection;
  G.kinematicsVec( headViewingDirection, headJViewingDirection, head, headViewingDirection_ ); // get function to minimize and its jacobian in state G

  // build u : vector between head and object point
  arr u = objectPosition_ - headPosition;
  arr Ju = - headJPosition;

  arr u1, Ju1;
  u1 = normalizedX( u, Ju, Ju1 );

  // build v : orientation vector of the head
  arr v1 = headViewingDirection;
  arr Jv1 = headJViewingDirection;

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // u - v
  tmp_y.setVectorBlock( 10.0 * ( u1  - v1 )     , 0 );    // cost
  tmp_J.setMatrixBlock( 10.0 * ( Ju1 - Jv1 ), 0 , 0 );    // jacobian

  // u - w
  if( moveAroundPivotDefined_ )
  {
    tmp_y.setVectorBlock( u1 -  w1_     , u.d0 - 1 );            // cost
    tmp_J.setMatrixBlock( Ju1, Ju.d0 - 1, 0 );                   // jacobian
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

HeadGetSightQuat::HeadGetSightQuat( const arr& objectPosition, const arr& pivotPoint )
  : Feature()
  , objectPosition_( objectPosition )
  , pivotPoint_    ( pivotPoint )
  , headViewingDirection_( 0.0, -1.0, 0.0 )
  , moveAroundPivotDefined_( false )
{
  w1_ = objectPosition_ - pivotPoint_;
  //std::cout << "w1:" << w1_ << std::endl;
  rai::Quaternion targetQuat;
  targetQuat.setDiff( rai::Vector( 0, -1.0, 0 ), w1_ / norm2( w1_ ) );

  targetQuat_ = conv_quat2arr( targetQuat );

  if( norm2( w1_ ) > 0 ) // normalize if the vector is not null
  {
    w1_ = w1_ * 1. / norm2( w1_ );
    moveAroundPivotDefined_ = true;
  }

  // with current sensor position, determine which object occludes ( at least one triangle is traversed by the ray )

  // determine the pivot point
}

void HeadGetSightQuat::phi(arr& y, arr& J, const rai::KinematicWorld& G )
{
  rai::Frame *head = G.getFrameByName("manhead");
  arr headPosition, headJPosition;
  G.kinematicsPos( headPosition, headJPosition, head );

  arr headQuat, JheadQuat;
  G.kinematicsQuat( headQuat, JheadQuat, head ); // get function to minimize and its jacobian in state G

  //arr headViewingDirection, headJViewingDirection;
  //G.kinematicsQuat();
  //G.kinematicsVec( headViewingDirection, headJViewingDirection, head->body, headViewingDirection_ ); // get function to minimize and its jacobian in state G

  // build u : vector between head and object point
  arr u = objectPosition_ - headPosition;
  double normU = norm2( u );
  arr Ju = - headJPosition;
  arr JnormU = Jnorm2( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // build v : orientation vector of the head
  //arr v1 = headViewingDirection;
  //arr Jv1 = headJViewingDirection;

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, headJPosition.dim(1) );

  // head orientation
  //tmp_y.setVectorBlock( 10.0*( u1  - v1)                       , 0 );    // cost
  //tmp_J.setMatrixBlock( 10.0 * ( Ju1 - Jv1), 0 , 0 ); // jacobian

  tmp_y.setVectorBlock( 1.0 * ( headQuat - targetQuat_ )       , 0 );    // cost
  tmp_J.setMatrixBlock( 1.0 * ( JheadQuat              )  , 0 , 0 ); // jacobian

  // head alignment
  if( moveAroundPivotDefined_ )
  {
    tmp_y.setVectorBlock( u1 -   w1_, headQuat.d0 - 1 );                    // cost
    tmp_J.setMatrixBlock( Ju1,   JheadQuat.d0 - 1, 0 );                    // jacobian
  }

  // head distance
  arr norm_a = zeros( 1 );
  norm_a( 0 ) = normU - 0.6;
  tmp_y.setVectorBlock( norm_a,         headQuat.d0 + u1.d0 - 1 );
  tmp_J.setMatrixBlock( JnormU * Ju,    JheadQuat.d0 + Ju1.d0 - 1, 0 );                    // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

ActiveGetSight::ActiveGetSight( rai::String const& sensorName,
                                        rai::String const& objectName,
                                        arr const& pivotPoint,
                                        arr const& aimingDir,
                                        double preferedDistance )
  : Feature()
  , sensorName_     ( sensorName )
  , objectName_( objectName )
  , pivotPoint_   ( pivotPoint )
  , aimingDir_    ( aimingDir / norm2( aimingDir ) )
  , preferedDistance_( preferedDistance )
{

}

void ActiveGetSight::phi( arr& y, arr& J, rai::KinematicWorld const& G )
{
  // get Object position and pivot position
  rai::Frame * object = G.getFrameByName( objectName_ );

  CHECK( object != nullptr, "body not found!" );

  arr aimPosition, aimJPosition;
  G.kinematicsPos( aimPosition, aimJPosition, object );

  arr pivotPosition, pivotJPosition;
  G.kinematicsPos( pivotPosition, pivotJPosition, object, pivotPoint_ );

  // get sensor position
  rai::Frame * sensor = G.getFrameByName( sensorName_ );
  arr sensorPosition, sensorJPosition;
  G.kinematicsPos( sensorPosition, sensorJPosition, sensor );

  //std::cout << "headPosition:" << headPosition << std::endl;

  // intermediary computations
  // build w1 : (normalized vector from pivot position to object center)
  arr w = aimPosition - pivotPosition;
  const double normW = norm2( w );
  //std::cout << "normW:" << normW << std::endl;
  arr w1 = w * 1. / normW;
  arr JnormW = Jnorm2( w );

  arr Jw = aimJPosition - pivotJPosition;
  arr Jw1 = ( Jw * normW - w * JnormW * Jw ) / ( normW * normW );

  // build u : vector between aiming point and head
  arr u = aimPosition - sensorPosition;
  const double normU = norm2( u );
  arr Ju = aimJPosition - sensorJPosition;
  arr JnormU = Jnorm2( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // build v : aiming direction of the sensor
  arr v, Jv;
  G.kinematicsVec( v, Jv, sensor, aimingDir_ ); // get function to minimize and its jacobian in state G

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, sensorJPosition.dim(1) );

  // sensor orientation - aiming directing is colinear to vector (head - center)
  tmp_y.setVectorBlock( ( u1  - v )     , 0 );    // cost
  tmp_J.setMatrixBlock( ( Ju1 - Jv ), 0 , 0 );    // jacobian

  // sensor alignment - object center, object pivot and sensor center are aligned
  tmp_y.setVectorBlock( u1 -  w1,   u1.d0  );                    // cost
  tmp_J.setMatrixBlock( Ju1 -  Jw1, Ju1.d0, 0 );                 // jacobian

  // sensor distance
  const double d = normU - preferedDistance_;
  tmp_y( 2*u1.d0 ) = d;
  tmp_J.setMatrixBlock( JnormU * Ju, 2 * Ju1.d0, 0 );            // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}


//===========================================================================

SensorAimAtObjectCenter::SensorAimAtObjectCenter( rai::String const& sensorName,
                                                  rai::String const& objectName,
                                                  arr const& aimingDir // sensor
                                                )
  : Feature()
  , sensorName_     ( sensorName )
  , objectName_     ( objectName )
  , aimingDir_    ( aimingDir / norm2( aimingDir ) )
{

}

void SensorAimAtObjectCenter::phi( arr& y, arr& J, rai::KinematicWorld const& G )
{
  // get Object position and pivot position
  rai::Frame * object = G.getFrameByName( objectName_ );

  CHECK( object != nullptr, "body not found!" );

  arr aimPosition, aimJPosition;
  G.kinematicsPos( aimPosition, aimJPosition, object );

  // get sensor position
  rai::Frame * sensor = G.getFrameByName( sensorName_ );
  arr sensorPosition, sensorJPosition;
  G.kinematicsPos( sensorPosition, sensorJPosition, sensor );

  //std::cout << "headPosition:" << headPosition << std::endl;

  // intermediary computations
  // build u : vector between aiming point and head
  arr u = aimPosition - sensorPosition;
  const double normU = norm2( u );
  arr Ju = aimJPosition - sensorJPosition;
  arr JnormU = Jnorm2( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // build v : aiming direction of the sensor
  arr v, Jv;
  G.kinematicsVec( v, Jv, sensor, aimingDir_ ); // get function to minimize and its jacobian in state G

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, sensorJPosition.dim(1) );

  // head orientation - aiming directing is colinear to vector (head - center)
  tmp_y.setVectorBlock( ( u1  - v )     , 0 );    // cost
  tmp_J.setMatrixBlock( ( Ju1 - Jv ), 0 , 0 );    // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

SensorAlignsWithPivot::SensorAlignsWithPivot( rai::String const& sensorName,
                                        rai::String const& objectName,
                                        arr const& pivotPoint,
                                        const double maxAngleRad )
  : Feature()
  , sensorName_     ( sensorName )
  , objectName_( objectName )
  , pivotPoint_   ( pivotPoint )
  , minDotProduct_( std::cos( maxAngleRad ) )
{

}

void SensorAlignsWithPivot::phi( arr& y, arr& J, rai::KinematicWorld const& G )
{
  // get Object position and pivot position
  rai::Frame * object = G.getFrameByName( objectName_ );

  CHECK( object != nullptr, "body not found!" );

  arr aimPosition, aimJPosition;
  G.kinematicsPos( aimPosition, aimJPosition, object );

  arr pivotPosition, pivotJPosition;
  G.kinematicsPos( pivotPosition, pivotJPosition, object, pivotPoint_ );

  // get sensor position
  rai::Frame * sensor = G.getFrameByName( sensorName_ );
  arr sensorPosition, sensorJPosition;
  G.kinematicsPos( sensorPosition, sensorJPosition, sensor );

  // intermediary computations
  // build w1 : normalized vector from pivot position to object center
  arr w = aimPosition - pivotPosition;
  const double normW = norm2( w );
  //std::cout << "normW:" << normW << std::endl;
  arr w1 = w * 1. / normW;
  arr JnormW = Jnorm2( w );

  arr Jw = aimJPosition - pivotJPosition;
  arr Jw1 = ( Jw * normW - w * JnormW * Jw ) / ( normW * normW );

  // build u : vector from sensor position to object center
  arr u = aimPosition - sensorPosition;
  const double normU = norm2( u );
  arr Ju = aimJPosition - sensorJPosition;
  arr JnormU = Jnorm2( u );  // get Jacobian of the norm operator
  arr u1 = u / normU;
  arr Ju1 = ( Ju * normU - u * JnormU * Ju ) / ( normU * normU ); // jacobian of u normalized

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  // sensor alignment - object center, object pivot and sensor center are aligned
  tmp_y(0) = minDotProduct_ - dot(u1, w1);                    // cost
  tmp_J.setMatrixBlock( - ~ ( ( ~ Ju1 ) * w1 + ( ~ Jw1 ) * u1 ), 0, 0 );                  // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//===========================================================================

SensorDistanceToObject::SensorDistanceToObject( rai::String const& sensorName,
                                        rai::String const& objectName,
                                        const double preferedDistance,
                                        const double tolerance )
  : Feature()
  , sensorName_     ( sensorName )
  , objectName_( objectName )
  , preferedDistance_( preferedDistance )
  , tolerance_( tolerance )
{

}

void SensorDistanceToObject::phi( arr& y, arr& J, rai::KinematicWorld const& G )
{
  // get Object position and pivot position
  rai::Frame * object = G.getFrameByName( objectName_ );

  CHECK( object != nullptr, "body not found!" );

  arr aimPosition, aimJPosition;
  G.kinematicsPos( aimPosition, aimJPosition, object );

  // get sensor position
  rai::Frame * sensor = G.getFrameByName( sensorName_ );
  arr sensorPosition, sensorJPosition;
  G.kinematicsPos( sensorPosition, sensorJPosition, sensor );

  //std::cout << "headPosition:" << headPosition << std::endl;

  // intermediary computations
  // build u : vector between aiming point and head
  arr u = aimPosition - sensorPosition;
  const double normU = norm2( u );
  arr Ju = aimJPosition - sensorJPosition;
  arr JnormU = Jnorm2( u );  // get Jacobian of the norm operator

  // instantiate a temporary vector for cost and its Jacobian
  arr tmp_y = zeros( dim_ );
  arr tmp_J = zeros( dim_, sensorJPosition.dim(1) );

  // sensor distance
  const double d = normU - preferedDistance_;
  tmp_y( 0 ) = d - tolerance_;
  tmp_J.setMatrixBlock( JnormU * Ju, 0, 0 );            // jacobian

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}
