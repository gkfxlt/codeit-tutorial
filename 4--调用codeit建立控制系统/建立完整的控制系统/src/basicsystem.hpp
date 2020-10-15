#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>


using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	//*************************����������***********************//////
	auto createVrepController()->std::unique_ptr<codeit::controller::Controller>;
	auto createEcatController()->std::unique_ptr<codeit::controller::Controller>;
    //*********************************************************//////

	//*************************����ģ��************************//////
	auto createPumaModel(std::string name = "model")->std::unique_ptr<codeit::model::Model>;
	auto createScaraModel(std::string name = "model")->std::unique_ptr<codeit::model::Model>;
	auto createUrModel(std::string name="model")->std::unique_ptr<codeit::model::Model>;
	auto createModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::Model>>;
	////*********************************************************//////

	//*************************�����쳣��Ϣ��************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>;
	////*********************************************************//////

	//*************************����ָ�************************//////
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	////*********************************************************//////

	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
