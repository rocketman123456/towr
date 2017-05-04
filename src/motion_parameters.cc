/**
@file    motion_type.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#include <xpp/opt/motion_parameters.h>

namespace xpp {
namespace opt {

MotionParameters::~MotionParameters ()
{
}

MotionParameters::ContactSchedule
MotionParameters::GetContactSchedule () const
{
  ContactSchedule phases;
  for (int i=0; i<contact_sequence_.size(); ++i) {
    double duration = contact_timings_.at(i);
    if (duration < 1e-10)
      continue; // skip phases with zero duration
    phases.push_back(Phase(contact_sequence_.at(i), duration));
  }

  return phases;
}

MotionParameters::MaxDevXYZ
MotionParameters::GetMaximumDeviationFromNominal () const
{
  return max_dev_xy_;
}

MotionParameters::CostWeights
MotionParameters::GetCostWeights () const
{
  return cost_weights_;
}

MotionParameters::UsedConstraints
MotionParameters::GetUsedConstraints () const
{
  return constraints_;
}

double
MotionParameters::GetTotalTime () const
{
  double T = 0.0;
  for (auto t : contact_timings_)
    T += t;

  return T;

}

} // namespace opt
} // namespace xpp

