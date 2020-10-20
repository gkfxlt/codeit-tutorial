#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>
namespace codeit::system
{
	auto createModelPool()->std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>>;
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	auto updateStateRt(codeit::core::Msg& msg)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
