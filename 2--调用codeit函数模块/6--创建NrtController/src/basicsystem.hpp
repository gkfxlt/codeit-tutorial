#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>
namespace codeit::system
{
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
	auto createController()->std::unique_ptr<codeit::controller::Controller>;
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>;
	auto createSocketController(const NumList* num = nullptr, std::string name = "socket_controller", std::string ip = "", std::string port = "", controller::SocketMaster::TYPE type = controller::SocketMaster::TYPE::TCP, Size pack_size = 0, Size nrt_id = 0)->std::unique_ptr<codeit::controller::NrtController>;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
