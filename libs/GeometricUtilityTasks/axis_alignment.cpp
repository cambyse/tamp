/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <axis_alignment.h>
#include <Kin/taskMaps.h>
//-----AxisAlignment----------------//

using namespace std;

void AxisAlignment::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  const auto body = G.getFrameByName( bobyName_ );

  arr bodyAxisDirection, bodyJAxisDirection;
  G.kinematicsVec( bodyAxisDirection, bodyJAxisDirection, body, bodyAxis_ );

  const double dot_product = dot( bodyAxisDirection, worldAxis_ );

  const double cost = 1 - dot_product;

//  if(bodyAxis_(0) < -0.5)
//  {
//    std::cout << "bodyAxis:" << bodyAxis_ << std::endl;
//    std::cout << "bodyAxisDirection:" << bodyAxisDirection << std::endl;
//  }

  tmp_y( 0 ) = cost;
  tmp_J.setMatrixBlock( - ( ~ worldAxis_ ) * bodyJAxisDirection, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-----AxisOrthogonal----------------//

void AxisOrthogonal::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  const auto body = G.getFrameByName( bobyName_ );

  arr bodyAxisDirection, bodyJAxisDirection;
  G.kinematicsVec( bodyAxisDirection, bodyJAxisDirection, body, bodyAxis_ );

  const double dot_product = dot( bodyAxisDirection, worldAxis_ );

  tmp_y( 0 ) = dot_product;
  tmp_J.setMatrixBlock( ( ~ worldAxis_ ) * bodyJAxisDirection, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-----BoxAxisAligned----------------//

void BoxAxisAligned::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 1 );
  arr tmp_J = zeros( 1, G.q.N );

  // effector
  const auto effector = G.getFrameByName( effectorName_ );

  arr effectorAxisDirection, effectorJAxisDirection;
  G.kinematicsVec( effectorAxisDirection, effectorJAxisDirection, effector, effectorAxis_ );

  // box
  const auto box = G.getFrameByName( boxName_ );

  arr boxAxisDirection, boxJAxisDirection;
  G.kinematicsVec( boxAxisDirection, boxJAxisDirection, box, boxAxis_ );

  const double dot_product = dot( boxAxisDirection, effectorAxisDirection );

  tmp_y( 0 ) = 1.0 - dot_product;
  tmp_J.setMatrixBlock( - ( ~ boxAxisDirection ) * effectorJAxisDirection - ( ~effectorAxisDirection ) * boxJAxisDirection, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-----TargetPosition----------------//

void TargetPosition::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 3 );
  arr tmp_J = zeros( 3, G.q.N );

  // effector
  const auto effector = G.getFrameByName( effectorName_ );

  arr effectorPosition, effectorJPosition;
  G.kinematicsPos( effectorPosition, effectorJPosition, effector );

  // box
  const auto box = G.getFrameByName( boxName_ );

  arr targetPosition, jTargetPosition;
  G.kinematicsPos( targetPosition, jTargetPosition, box, targetPosition_ );

  tmp_y = targetPosition - effectorPosition;
  tmp_J.setMatrixBlock( jTargetPosition - effectorJPosition, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}


//-----ZeroVelocity----------------//

void ZeroVelocity::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 3 );
  arr tmp_J = zeros( 3, G.q.N );

  // effector
  const auto object = G.getFrameByName( objectName_ );

  arr objectVelocity, objectJVelocity;
  G.kinematicsPos( objectVelocity, objectJVelocity, object );

  tmp_y = objectVelocity;
  tmp_J.setMatrixBlock( objectJVelocity, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//-----ZeroRotation----------------//
void ZeroRotation::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 4 );
  arr tmp_J = zeros( 4, G.q.N );

  // effector
  const auto object = G.getFrameByName( objectName_ );

  arr objectQuat, objectJQuat;
  G.kinematicsQuat( objectQuat, objectJQuat, object );

  tmp_y = objectQuat;
  tmp_J.setMatrixBlock( objectJQuat, 0, 0 );

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//---ZeroMovement------------//

void ZeroMovement::phi( arr& y, arr& J, const WorldL& Gs )
{
  CHECK(order==1,"");
  CHECK(Gs.size() >= 1,"");

  const auto dim_Phi{ dim_phi(*Gs(-1))};

  y.resize(dim_Phi);
  if(!!J){
    uintA qidx(Gs.N);
    qidx(0)=0;
    for(uint i=1;i<Gs.N;i++) qidx(i) = qidx(i-1)+Gs(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Gs.last()->q.N);
  }

  auto frame = Gs(-1)->getFrameByName(objectName_);

  // get pose diff in order 1
  arr y_vel, J_vel;
  TM_Default vel(TMT_poseDiff, frame->ID); // to prevent both velocity and orientation changes
  vel.order = 1;
  vel.__phi(y_vel, J_vel, Gs);

  y.setVectorBlock( y_vel, 0);
  if(!!J)
  {
    J.setMatrixBlock( J_vel, 0, 0 );
  }
}

void ZeroMovement::phi(arr& y, arr& J, const rai::KinematicWorld& G)
{
  CHECK(false, "");
}


//-----ZeroRelativeRotationVel----------------//

void ZeroRelativeRotationVel::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 4 );
  arr tmp_J = zeros( 4, G.q.N );

  // object
  const auto object = G.getFrameByName( objectName_ );
  const auto parent = object->parent;

  if(parent != nullptr)
  {
    arr objectRotation, objectJRotation;
    TM_Default tm(TMT_quat, G, objectName_, NoVector, parent->name, NoVector);
    tm.phi(objectRotation, objectJRotation, G);

    tmp_y = objectRotation;
    tmp_J.setMatrixBlock( objectJRotation, 0, 0 );
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}


//-----ZeroRelativeVel----------------//

void ZeroRelativeVel::phi( arr& y, arr& J, const rai::KinematicWorld& G )
{
  arr tmp_y = zeros( 3 );
  arr tmp_J = zeros( 3, G.q.N );

  // object
  const auto object = G.getFrameByName( objectName_ );
  const auto parent = object->parent;

  if(parent != nullptr)
  {
    arr objectTranslation, objectJTranslation;
    TM_Default tm(TMT_pos, G, objectName_, NoVector, parent->name, NoVector);
    tm.phi(objectTranslation, objectJTranslation, G);

    tmp_y = objectTranslation;
    tmp_J.setMatrixBlock( objectJTranslation, 0, 0 );
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

//---ZeroVelocityOfAllXYPhiJoints------------//

void ZeroVelocityOfAllXYPhiJoints::phi( arr& y, arr& J, const WorldL& Gs )
{
  CHECK(order==1,"");
  CHECK(Gs.size() >= 1,"");

  const auto dim_Phi{ dim_phi(*Gs(-1))};

  y.resize(dim_Phi);
  if(!!J){
    uintA qidx(Gs.N);
    qidx(0)=0;
    for(uint i=1;i<Gs.N;i++) qidx(i) = qidx(i-1)+Gs(i-1)->q.N;
    J = zeros(y.N, qidx.last()+Gs.last()->q.N);
  }

  uint d{0};
  for(const auto frame: Gs(-1)->frames)
  {
    if(frame->joint && frame->joint->type == rai::JT_transXYPhi)
    {
      // get speed vector
      arr y_vel,Jvel;
      TM_Default vel(TMT_poseDiff, frame->ID); // to prevent both velocity and orientation changes
      vel.order = 1;
      vel.__phi(y_vel, Jvel, Gs);

      y.setVectorBlock( y_vel, d );
      if(!!J)
      {
        J.setMatrixBlock( Jvel, d, 0 );
      }

      d+=7;
    }
  }
}

void ZeroVelocityOfAllXYPhiJoints::phi(arr& y, arr& J, const rai::KinematicWorld& G)
{
  CHECK(false, "");
}


uint ZeroVelocityOfAllXYPhiJoints::dim_phi( const rai::KinematicWorld& G )
{
  uint d = 0;
  for(const auto frame: G.frames)
  {
    if(frame->joint && frame->joint->type == rai::JT_transXYPhi)
    {
      d+=7;
    }
  }

  return d;
}
