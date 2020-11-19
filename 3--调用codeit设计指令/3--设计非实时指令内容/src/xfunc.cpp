#include "xfunc.hpp"
#include<codeit.hpp>
namespace codeit::function {
	struct MoveNrtParam
	{
		std::int32_t motion_id;
		double amp, freq;
		vector<double> axis_begin_pos_vec;
		double time{ 0 };
	};
	auto MoveNrt::prepareNrt(BasisFunc&, int)->void
	{
		MoveNrtParam param;
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
	auto MoveNrt::executeNRT(BasisFunc&, int)->int
	{
		auto& param = std::any_cast<MoveNrtParam&>(this->param());
		if (nrtCount() == 1) {

			;
		}
		//////*********************** MoveSine 规划 *************************//
		double dt = nrtControllerPool()->atNrt(0).samplePeriodNs() / 1.0e9;
		auto running_flag = true;
		param.time += dt;
		double pos = param.amp * sin(2 * PI * param.freq * param.time);
		
		
		codeit::core::cout() << "target_count:	" << nrtCount() <<"	"
			 << "pos:	" << pos
			 << std::endl;
		

		if (nrtCount() == 300)
			return 0;//结束该指令
		return 1;//仍在运行该指令
	}
	auto MoveNrt::collectNrt(BasisFunc&, int)->void {}
	MoveNrt::~MoveNrt() = default;
	MoveNrt::MoveNrt(const std::string& name) :BasisFunc(name)
	{
		setIsNeedRT(false);
		command().loadXmlStr(
			"<Command name=\"MoveNrt\">"
			"	<GroupParam>"
			"		<Param name=\"amp\" default=\"0.2\"/>"
			"		<Param name=\"freq\" default=\"1\"/>"
			"		<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	CODEIT_DEFINE_BIG_FOUR_CPP(MoveNrt);
}