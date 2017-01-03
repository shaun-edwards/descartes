/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Shaun Edwards
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <console_bridge/console.h>
#include "descartes_trajectory/nominal_interpolator.h"

#include <ros/console.h>

namespace descartes_trajectory
{


bool NominalInterpolator::linear(const CartTrajectoryPt &p1, const CartTrajectoryPt &p2,
                                 const descartes_core::RobotModel &model, double fraction,
                                 CartTrajectoryPt &pi)
{
  if ( !isNominal(p1) )
  {
    ROS_ERROR("Point p1 is not nominal");
    return false;
  }
  if ( isNominal(p2) )
  {
    ROS_ERROR("Point p2 is not nominal");
    return false;
  }

  return linearNominal(p1, p2, model,time_fration, pi);


}




bool NominalInterpolator::linear(const JointTrajectoryPt &j1, const CartTrajectoryPt &p2,
                                 const descartes_core::RobotModel &model, double fraction,
                                 CartTrajectoryPt &pi)
{
  CartTrajectoryPt p1;

  if ( !isNominal(j1) )
  {
    ROS_ERROR("Point j1 is not nominal");
    return false;
  }

  if ( !isNominal(p2) )
  {
    ROS_ERROR("Point p2 is not nominal");
    return false;
  }

  if( !j1.getNominalCartPose(j1.nominal(), model, p1) )
  {

    ROS_ERROR("Failed to convert j1 to cartesian pose");
    return false;
  }

  return linearNominal(p1, p2, model,time_fration, pi);

}





bool NominalInterpolator::linear(const CartTrajectoryPt &p1, const JointTrajectoryPt &j2,
                                 const descartes_core::RobotModel &model, double fraction,
                                 CartTrajectoryPt &pi)
{
  CartTrajectoryPt p2;

  if ( !isNominal(p1) )
  {
    ROS_ERROR("Point p1 is not nominal");
    return false;
  }

  if ( !isNominal(j2) )
  {
    ROS_ERROR("Point j2 is not nominal");
    return false;
  }

  if( !j2.getNominalCartPose(j2.nominal(), model, p2) )
  {
    ROS_ERROR("Failed to convert j2 to cartesian pose");
    return false;
  }

  return linearNominal(p1, p2, model,time_fration, pi);;
}




bool NominalInterpolator::linear(const JointTrajectoryPt &j1, const JointTrajectoryPt &j2,
                                 double fraction, JointTrajectoryPt &ji)
{
  if( j1.size() != j2.size() )
  {
    ROS_ERROR("Joint point size mismatch, can't interpolate");
    return false;
  }

  if( j1.timing_.isSpecified() != j2.timing_.isSpecified() )
  {
    ROS_ERROR("Timing specification mismatch, can't interpolate");
    return false;
  }

  std::vector<double> joints(j1.size());
  descartes_core::TimingConstraint t;

  for (size_t ii = 0; ii < j1.size(); ++ii)
  {
    joints.at(ii) = j1.nominal().at(ii) +
        ( (j2.nominal().at(ii) - j1.nominal().at(ii)) * fraction );
  }

  if( j1.timing_.isSpecified() && j2.timing_.isSpecified() )
  {
    t.lower = j1.timing_.lower +
        ( (j2.timing_.lower - j1.timing_.lower) * fraction );
    t.upper = j1.timing_.upper +
        ( (j2.timing_.upper - j1.timing_.upper) * fraction );
    ji = JointTrajectoryPt(joints, t);
  }
  else
  {
    ji = JointTrajectoryPt(joints);
  }

  return true;

}




bool NominalInterpolator::linearNominal(const CartTrajectoryPt &p1, const CartTrajectoryPt &p2,
                                        const descartes_core::RobotModel &model, double fraction,
                                        CartTrajectoryPt &pi)
{

  if( p1.timing_.isSpecified() != p1.timing_.isSpecified() )
  {
    ROS_ERROR("Timing specification mismatch, can't interpolate");
    return false;
  }

  descartes_core::TimingConstraint t;


  if( p1.timing_.isSpecified() && p2.timing_.isSpecified() )
  {
    t.lower = p1.timing_.lower +
        ( (p2.timing_.lower - p1.timing_.lower) * fraction );
    t.upper = j1.timing_.upper +
        ( (p2.timing_.upper - p1.timing_.upper) * fraction );

    pi = CartTrajectoryPt(i);
  }
  else
  {
    pi = CartTrajectoryPt();
  }


  descartes_core::Frame base(interpolate(p1.getNominalCartPose(), p2.getNominalCartPose()));
  TolerancedFrame base_pt;
  descartes_core::Frame wobj;
  TolerancedFrame wobj_pt;


  return true;
}






bool isNominal(const CartTrajectoryPt p) const;
bool isNominal(const JointTrajectoryPt j) const:

}
