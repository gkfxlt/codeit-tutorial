#include "xfunc.hpp"
#include<codeit.hpp>
namespace codeit::function {
	struct MoveCollectParam
	{
		std::int32_t motion_id;
		double amp, freq;
		vector<double> axis_begin_pos_vec;
		double time{ 0 };
	};
	auto MoveCollect::prepareNrt(BasisFunc&, int)->void
	{
		MoveCollectParam param;
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
	
	auto MoveCollect::collectNrt(BasisFunc&, int)->void 
	{
		//��collect�����ڣ����Դ����µ�ָ����������³�����
		//1����UR�������������˶�ָ�����collect��������˶�����ĳЩ������������stopָ�
		//2���Ӿ��ŷ�����̬���ϵ�Ӧ���У���collect�����ڴ����Ӿ���Ϣ�����б��ϣ��������˶�ָ�
		//��collect����������ָ���ͬʱ��֮ǰ���˶�ָ���ͬ����ִ�С�

		//collect����С���쳣�����ܵ�������throw������

		std::string cmd_str = "Disable";
		controlSystem()->executeCmdInCmdLine(cmd_str, [](codeit::function::BasisFunc& plan, int model_index)->void
				{
					//codeit::system::multiLingual(plan);
					COUT_PLAN((&plan)) << " model_index:" << model_index << "--return code :" << plan.retCode() << "\n";
					COUT_PLAN((&plan)) << " model_index:" << model_index << "--return msg  :" << plan.retMsg() << std::endl;
					LOG_INFO << "cmd " << plan.cmdId() << " model_index:" << model_index << " return code   :" << plan.retCode() << "\n" << std::setw(core::LOG_SPACE_WIDTH) << '|' << "return message:" << plan.retMsg() << std::endl;
				});
	}
	MoveCollect::~MoveCollect() = default;
	MoveCollect::MoveCollect(const std::string& name) :BasisFunc(name)
	{
		command().loadXmlStr(
			"<Command name=\"MoveCollect\">"
			"	<GroupParam>"
			"		<Param name=\"amp\" default=\"0.2\"/>"
			"		<Param name=\"freq\" default=\"1\"/>"
			"		<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	CODEIT_DEFINE_BIG_FOUR_CPP(MoveCollect);
}