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
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>;
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>;
	auto createModelPool()->std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>>;
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	auto updateStateRt(codeit::core::Msg& msg)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
