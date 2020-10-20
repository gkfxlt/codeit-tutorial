#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>


using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto updateStateRt(codeit::core::Msg& msg)->void;
	class NumList
	{
	public:
		int di_num;	
		int do_num;	
		int ai_num;
		int ao_num;
		int motor_num;
		int subsys_num;
	};

	//*************************创建控制器***********************//////
	auto createVrepController()->std::unique_ptr<codeit::controller::Controller>;
	auto createEcatController()->std::unique_ptr<codeit::controller::Controller>;
    auto createZeroErrEcatController()->std::unique_ptr<codeit::controller::Controller>;

	auto createComController(const NumList* num = nullptr, const std::string& name = "com_controller", \
		const core::SerialPort::ComOptions& options = core::SerialPort::defaultComOptions, Size pack_size = 0, Size nrt_id = 0)->std::unique_ptr<codeit::controller::NrtController>;
	auto createSocketController(const NumList* num = nullptr, std::string name="socket_controller", std::string ip = "", std::string port="", controller::SocketMaster::TYPE type= controller::SocketMaster::TYPE::TCP, Size nrt_id=0)->std::unique_ptr<codeit::controller::NrtController>;
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>;
	//*********************************************************//////

	//*************************创建模型************************//////
	auto createIOModel(const NumList* num = nullptr, std::string name = "model")->std::unique_ptr<codeit::model::IOModel>;
	auto createModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::Model>>;
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>;

	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>;
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	auto createUserDataType(core::Calculator& cal)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
