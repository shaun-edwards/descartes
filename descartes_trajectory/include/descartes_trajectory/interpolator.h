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


#pragma once

#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_core/robot_model.h"


namespace descartes_trajectory
{


/**@brief An Interpolator is...
 */
class Interpolator
{
public:

  virtual ~Interpolator()
  {
  }

  virtual bool linear(const CartTrajectoryPt &p1, const CartTrajectoryPt &p2,
                      const descartes_core::RobotModel &model, double time_fraction,
                      CartTrajectoryPt pi) const = 0;
  virtual bool linear(const JointTrajectoryPt &j1, const CartTrajectoryPt &p2,
                      const descartes_core::RobotModel &model, double time_fraction,
                      CartTrajectoryPt pi) const = 0;
  virtual bool linear(const CartTrajectoryPt &p1, const JointTrajectoryPt &j2,
                      const descartes_core::RobotModel &model, double time_fraction,
                      CartTrajectoryPt pi) const = 0;
  virtual bool linear(const JointTrajectoryPt &j1, const JointTrajectoryPt &j2,
                      double time_fraction, JointTrajectoryPt ji) const = 0;

};

}
