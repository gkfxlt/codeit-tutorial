#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>

using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto createController()->std::unique_ptr<codeit::controller::Controller>;
	
	auto updateStateRt(codeit::core::Msg& msg)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
