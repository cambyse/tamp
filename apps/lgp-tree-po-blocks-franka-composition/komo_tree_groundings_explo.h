#pragma once

#include <komo_factory.h>

namespace explo
{
void groundTreeInit( const mp::TreeBuilder& tb, KOMO_ext* komo, int verbose );
void groundTreePickUp( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeUnStack( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreePutDown( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreePutDownFlipped( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeCheck( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeStack( const mp::Interval& interval, const mp::TreeBuilder& tb, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
}
