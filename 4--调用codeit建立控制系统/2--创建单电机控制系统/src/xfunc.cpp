#include "xfunc.hpp"
#include<codeit.hpp>
namespace codeit::function {

	//定义类，有几个类，就有几个控制算法

		//moveJS中的s是指sin函数
		//定义结构体，添加参数，其中有总时间total_time，步长step_size
	struct MoveSineParam
	{
		std::int32_t motion_id;
		double amp, freq;
		vector<double> axis_begin_pos_vec;
		double time{ 0 };
	};
	auto MoveJS::prepareNrt(BasisFunc&, int)->void
	{
		MoveSineParam param;
		for (auto cmd_param : cmdParams()) {
			if (cmd_param.first == "motion_id") {
				param.motion_id = int32Param(cmd_param.first);
			}if (cmd_param.first == "amp")
				param.amp = doubleParam(cmd_param.first);
			if (cmd_param.first == "freq")
				param.freq = doubleParam(cmd_param.first);
		}

		auto num = controller()->motionPool().size();
		param.axis_begin_pos_vec.resize(num, 0);

		this->param() = param;

		for (auto& option : motorOptions()) option |= NOT_CHECK_POS_CONTINUOUS
			| NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER
			| NOT_CHECK_VEL_CONTINUOUS
			| NOT_CHECK_ENABLE;
	}
	auto MoveJS::executeRT(BasisFunc&, int)->int
	{
		//////*********************** MoveSine 参数初始化 *************************//
		auto& param = std::any_cast<MoveSineParam&>(this->param());
		auto num = controller()->motionPool().size();

		if (count() == 1) {

			for (int i = 0; i < num; i++)
				param.axis_begin_pos_vec[i] = controller()->motionPool()[i].actualPos();

		}
		//////*********************** MoveSine 规划 *************************//
		double dt = controller()->samplePeriodNs() / 1.0e9;
		auto running_flag = true;
		param.time += dt;
		double pos = 0;
		for (Size i = 0; i < num; i++) {
			
			pos = param.axis_begin_pos_vec[i] + \
				param.amp * sin(2 * PI * param.freq * param.time);
			controller()->motionPool()[0].setTargetPos(pos);
		}

		// 打印 //
		auto& cout = controller()->mout();//cout区别于std::cout
		cout << "target_count:	" << count() << std::endl;
		// 保存 //
		auto& lout = controller()->lout();
		lout << pos << endl;//保存pos信息至txt文件

		if (count() == 300)
			return 0;//结束该指令
		return 1;//仍在运行该指令
	}
	auto MoveJS::collectNrt(BasisFunc&, int)->void {}
	MoveJS::~MoveJS() = default;
	MoveJS::MoveJS(const std::string& name) :BasisFunc(name)
	{
		command().loadXmlStr(
			"<Command name=\"sine\">"
			"	<GroupParam>"
			"		<Param name=\"amp\" default=\"0.2\"/>"
			"		<Param name=\"freq\" default=\"1\"/>"
			"		<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	CODEIT_DEFINE_BIG_FOUR_CPP(MoveJS);
}