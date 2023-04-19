/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>


using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
int main()
{
	NlpFormulation formulation;

	// 地面
	formulation.terrain_ = std::make_shared<FlatGround>(0.0);

	// 机器人的运动学限制和动力学参数
	formulation.model_ = RobotModel(RobotModel::Cyberdog);

	// 设置机器人的初始位置
	formulation.initial_base_.lin.at(kPos).z() = 0.5;
	formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());
	formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());
	formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());
	formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());

	// 定义机器人的目标状态
	formulation.final_base_.lin.at(towr::kPos) << 1.0, 0.0, 0.5;

	// 定义运动的参数。有关默认值或可以修改的其他值，请参阅 c'tor。
	// 首先我们定义初始阶段持续时间，但是可以由优化器更改。 然而，摆动和站立阶段的数量是固定的。
	// 交替站姿和摆动：    ____-----_____-----_____-----_____
	formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
	formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
	formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
	formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
	formulation.params_.ee_in_contact_at_start_.push_back(true);
	formulation.params_.ee_in_contact_at_start_.push_back(true);
	formulation.params_.ee_in_contact_at_start_.push_back(true);
	formulation.params_.ee_in_contact_at_start_.push_back(true);

	// Initialize the nonlinear-programming problem with the variables,
	// constraints and costs.
	ifopt::Problem nlp;
	SplineHolder solution;
	for(auto c: formulation.GetVariableSets(solution))
		nlp.AddVariableSet(c);
	for(auto c: formulation.GetConstraints(solution))
		nlp.AddConstraintSet(c);
	for(auto c: formulation.GetCosts())
		nlp.AddCostSet(c);

	// You can add your own elements to the nlp as well, simply by calling:
	// nlp.AddVariablesSet(your_custom_variables);
	// nlp.AddConstraintSet(your_custom_constraints);

	// Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
	// solver->SetOption("derivative_test", "first-order");
	auto solver = std::make_shared<ifopt::IpoptSolver>();
	solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
	solver->SetOption("max_cpu_time", 20.0);
	solver->Solve(nlp);

	// Can directly view the optimization variables through:
	// Eigen::VectorXd x = nlp.GetVariableValues()
	// However, it's more convenient to access the splines constructed from these
	// variables and query their values at specific times:
	using namespace std;
	cout.precision(2);
	nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
	cout << fixed;
	cout << "\n====================\nMonoped trajectory:\n====================\n";

	double t = 0.0;
	while(t <= solution.base_linear_->GetTotalTime() + 1e-5)
	{
		cout << "t=" << t << "\n";
		cout << "Base linear position x,y,z:   \t";
		cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

		cout << "Base Euler roll, pitch, yaw:  \t";
		Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
		cout << (rad / M_PI * 180).transpose() << "\t[deg]" << endl;

		for(int i = 0; i < 4; ++i)
		{
			cout << "Foot[" << i << "] position x,y,z:          \t";
			cout << solution.ee_motion_.at(i)->GetPoint(t).p().transpose() << "\t[m]" << endl;
		}

		for(int i = 0; i < 4; ++i)
		{
			cout << "Contact[" << i << "] force x,y,z:          \t";
			cout << solution.ee_force_.at(i)->GetPoint(t).p().transpose() << "\t[N]" << endl;
		}

		for(int i = 0; i < 4; ++i)
		{
			bool contact = solution.phase_durations_.at(i)->IsContactPhase(t);
			std::string foot_in_contact = contact ? "yes" : "no";
			cout << "Foot[" << i << "] in contact:              \t" + foot_in_contact << endl;
		}

		cout << endl;

		t += 0.2;
	}
}
