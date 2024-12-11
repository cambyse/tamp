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

#include <trajectory_tree_visualizer.h>

namespace mp
{

TrajectoryTreeVisualizer::TrajectoryTreeVisualizer( const rai::Array< rai::Array< rai::Array< rai::KinematicWorld > > > & frames, const std::string & name, uint stepsPerSecs, bool saveVideo )
  : stepsPerSecs_( stepsPerSecs )
  , saveVideo_( saveVideo )
{
  // get number of views
  uint n = 0;
  for( auto leaf = frames.begin(); leaf != frames.end(); ++leaf )
  {
    for( const auto& x : *leaf )
    {
      if( x.N > 0 )
      {
        n++;
      }
    }
  }

  views_.resize( n );

  // build each view
  uint index = 0;
  for( auto leaf = frames.begin(); leaf != frames.end(); ++leaf )
  {
    for( uint w = 0; w < leaf->N; ++w )
    {
      rai::Array< rai::KinematicWorld > & traj = (*leaf)( w );
      if( traj.N > 0 )
      {
        const std::string windowName = name + std::string( "-world-" ) + std::to_string( w ) + " - " + std::to_string( index ) ;

        Var<WorldL> configs;
        configs.name() = windowName.c_str();
        views_[ index ] = std::make_shared< KinPathViewer >(configs ,  1.0 / stepsPerSecs_, -0 ); //0.05

        rai::Array< rai::KinematicWorld * > configurations( traj.N );

        for( uint s = 0; s < traj.N; ++s )
        {
          configurations( s ) = &traj( s ); //s
        }

        views_[ index ]->setConfigurations( configurations );
        views_[ index ]->writeToFiles = saveVideo_;
        views_[ index ]->text = STRING(index);
        // for franka
//        {
//        uint w{1100};
//        uint h{900};
//        auto gl = views_[ index ]->gl;
//        gl->resize(w,h);
//        gl->camera.setWHRatio((double)w/h);
//        gl->camera.focus(0.35, 0.0, 0.5);
//        gl->camera.setFocalLength(12.0);
//        }

//        {
//          auto & kin = views_[ index ];
//          const double zf = 1.47;
//          const double s = 0.55;
//          kin->copy.gl().camera.setPosition(s * 10., s * 3., zf + s * ( 2.28 - zf ));
//          kin->copy.gl().camera.focus(0, 0, zf);
//          kin->copy.gl().camera.upright();
//        }

        // for baxter
//        {
//          uint w{1100};
//          uint h{900};
//          auto gl = views_[ index ]->gl;

//          const double zf = 1.47;
//          const double s = 0.55;
//          gl->resize(w,h);
//          gl->camera.setWHRatio((double)w/h);
//          gl->camera.setPosition(s * 10., s * 3., zf + s * ( 2.28 - zf ));
//          gl->camera.focus(0, 0, zf);
//          gl->camera.upright();
//        }
        index++;
      }
    }
  }
}

}
