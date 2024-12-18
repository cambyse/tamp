/*  ------------------------------------------------------------------
    Copyright 2017 Camille Phiquepal
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


#pragma once

#include <Kin/feature.h>

namespace mp
{

struct AgentKinEquality:Feature{

  AgentKinEquality( uint id, const arr& q, const arr qmask )
    : id_( id )
    , q_  ( q )
    , qmask_( qmask )
    , dim_( q.N )
  {

  }

  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G )
  {
    y = zeros( dim_ );
    y.setVectorBlock( G.q - q_, 0 );

    auto tmp_J = eye( dim_, dim_ );

    // apply mask
    for( auto i = 0; i < qmask_.d0; ++i )
    {
      y( i ) = y( i ) * qmask_( i );
      tmp_J( i, i ) = tmp_J( i, i ) * qmask_( i );
    }

    if(&J) J = tmp_J;
  }

  virtual uint dim_phi( const rai::KinematicWorld& G )
  {
    return dim_;
  }

  virtual rai::String shortTag( const rai::KinematicWorld& G )
  {
    return rai::String( "AgentKinEquality" );
  }

private:
  // parameters
  const uint id_;
  const arr q_;
  const arr qmask_;
  const uint dim_;
};

struct AgentKinBounds:Feature{
  virtual void phi( arr& y, arr& J, const rai::KinematicWorld& G )
  {
    y.resize(dim_phi(G)).setZero();

    const auto& limits=G.getLimits(); //G might change joint ordering (kinematic switches), need to query limits every time

    if(!!J) J.resize(dim_phi(G), G.getJointStateDimension()).setZero();

    for( auto i = 0; i < G.q.N; ++i )
    {
      // overflow
      {
        const double d = G.q(i) - limits(i, 1);
        if( d > 0.0 )
        {
          y(i) = d;
          if(!!J) J( i, i ) = 1.0;
        }
      }

      // underflow
      {
        const double d = - G.q(i) + limits(i, 0);
        if( d > 0.0 )
        {
          y(i) = d;
          if(!!J) J( i, i ) = -1.0;
        }
      }
    }
  }

  virtual uint dim_phi( const rai::KinematicWorld& G )
  {
    return G.q.N;
  }

  virtual rai::String shortTag( const rai::KinematicWorld& G )
  {
    return rai::String( "AgentKinBounds" );
  }

private:
};

}
