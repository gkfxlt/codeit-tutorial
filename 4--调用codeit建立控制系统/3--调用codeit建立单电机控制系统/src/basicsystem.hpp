#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>
#include"xfunc.hpp"

using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto createModel(std::string name)->std::unique_ptr<codeit::model::Model>;
	auto createModelPool()->std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>>;
	auto createController()->std::unique_ptr<codeit::controller::Controller>;
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	//*************************创建异常信息池************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>;
	////*********************************************************//////

	auto createUserDataType(core::Calculator& cal)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
