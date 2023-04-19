#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CYBERDOG_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CYBERDOG_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr
{

/**
 * @brief The Kinematics of the quadruped robot Cyberdog.
 */
	class CyberdogKinematicModel : public KinematicModel
	{
	public:
		CyberdogKinematicModel()
				: KinematicModel(4)
		{
			const double x_nominal_b = 0.235;
			const double y_nominal_b = 0.15715;
			const double z_nominal_b = -0.2800;

			nominal_stance_.at(LF) << x_nominal_b, y_nominal_b, z_nominal_b;
			nominal_stance_.at(RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
			nominal_stance_.at(LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
			nominal_stance_.at(RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

			max_dev_from_nominal_ << 0.1, 0.1, 0.1; // 最大偏差？
		}
	};

/**
 * @brief The Dynamics of the quadruped robot Cyberdog.
 */
	class CyberdogDynamicModel : public SingleRigidBodyDynamics
	{
	public:
		CyberdogDynamicModel()
				: SingleRigidBodyDynamics(12.328,
				                          0.13, 0.54, 0.63, 0, 0, 0,
				                          4) {}
	};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_CYBERDOG_MODEL_H_ */
