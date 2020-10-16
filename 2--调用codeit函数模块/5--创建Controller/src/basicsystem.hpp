#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>
namespace codeit::system
{
	auto createController()->std::unique_ptr<codeit::controller::Controller>;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
