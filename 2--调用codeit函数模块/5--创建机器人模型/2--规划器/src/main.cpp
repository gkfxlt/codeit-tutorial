#include <iostream>
#include <codeit.hpp>

using namespace std;

struct PlanParam
{
	vector<double> begin_pos_vec, end_pos_vec, vel_vec, acc_vec, dec_vec, jerk_vec;
	vector<double> distance, ForwardOrReverse;
	vector<double> vlim, alim;
	vector<int> init_flag;
	vector<int> running_flag;
	// pos表示的是相对于初始位置的增量 //
	vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint, t, Tt;//Tt为S规划出的时间特征参数//
	vector<double> dpos_joint_raw, dvel_joint_raw, dacc_joint_raw, djerk_joint_raw;
	std::int32_t algorithm_error_code{ 0 };
	char lgorithm_error_msg[1024]{};
};
void initPlanParam(PlanParam& param, Size num)
{
	param.begin_pos_vec.resize(num, 0);//初始位置
	param.end_pos_vec.resize(num, 1);//末位置
	param.vel_vec.resize(num, 1);//允许最大速度
	param.acc_vec.resize(num, 5);//允许最大加速度
	param.dec_vec.resize(num, -5);//允许最大减速度
	param.jerk_vec.resize(num, 20);//允许最大加加速度

	{
		param.ForwardOrReverse.resize(num, 0);
		param.distance.resize(num, 0);
		param.vlim.resize(num, 0);
		param.alim.resize(num, 0);
		param.init_flag.resize(num, 0);
		param.running_flag.resize(num, 0);
		param.t.resize(num, 0);
		param.dpos_joint.resize(num, 0);
		param.dvel_joint.resize(num, 0);
		param.dacc_joint.resize(num, 0);
		param.djerk_joint.resize(num, 0);
		param.dpos_joint_raw.resize(num, 0);
		param.dvel_joint_raw.resize(num, 0);
		param.dacc_joint_raw.resize(num, 0);
		param.djerk_joint_raw.resize(num, 0);
		param.Tt.resize(num * 4, 0);

		for (int i = 0; i < num; i++) {
			if (param.begin_pos_vec[i] < param.end_pos_vec[i])
				param.ForwardOrReverse[i] = 1;
			else
				param.ForwardOrReverse[i] = -1;
		}
		for (int i = 0; i < num; i++)
			param.distance[i] = abs(param.end_pos_vec[i] - param.begin_pos_vec[i]);
	}
}
int main()
{
	ofstream outfile;//创建文件
	outfile.open("data.txt");

	PlanParam param;
	Size num = 3;
	initPlanParam(param, num);
	codeit::model::JointPlanner* jointPlanner=new codeit::model::JointPlanner();
	jointPlanner->reset();
	double dt = 0.001;
	auto running_flag = true;
	Size count = 1;

	while (running_flag) {
		for (Size i = 0; i < num; i++)
			param.t[i] += dt;

		running_flag = jointPlanner->planning(0, num, param.distance, count - 1, \
			param.vel_vec, param.acc_vec, param.dec_vec, \
			param.jerk_vec, param.dpos_joint_raw, param.dvel_joint_raw, \
			param.dacc_joint_raw, param.djerk_joint_raw, param.t, param.Tt, \
			param.vlim, param.alim, param.init_flag, param.running_flag, param.algorithm_error_code);
		
		for (int i = 0; i < num; i++) {
			param.dpos_joint[i] = param.ForwardOrReverse[i] * param.dpos_joint_raw[i];
			param.dvel_joint[i] = param.ForwardOrReverse[i] * param.dvel_joint_raw[i];
			param.dacc_joint[i] = param.ForwardOrReverse[i] * param.dacc_joint_raw[i];
			param.djerk_joint[i] = param.ForwardOrReverse[i] * param.djerk_joint_raw[i];
		}

		for (int i = 0; i < num; i++)
		{
			outfile << param.begin_pos_vec[i] + param.dpos_joint[i] << "	"//规划的实时位置
				<< param.dvel_joint[i] << "	"//规划的实时速度
				<< param.dacc_joint[i] << "	"//规划的实时加速度
				<< param.djerk_joint[i] << "	";//规划的实时加加速度
		}
		outfile << endl;
		count++;
	}

	
	return 0;
}
