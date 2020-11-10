#include "basicsystem.hpp"
namespace codeit::system
{
	auto createModel(std::string name)->std::unique_ptr<codeit::model::Model>
	{
		std::unique_ptr<codeit::model::Model> model = std::make_unique<codeit::model::Model>(name);
		return model;
	}
	auto createModelPool()->std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>>
	{
		std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>> modelPool(new codeit::core::ObjectPool<codeit::model::Model>);
		modelPool->add(createModel("motor").release());
		return modelPool;
	}

	/*�������ص���һ����ָ�룬ָ��ָ��controller������������ָ�� std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*����std::unique_ptrʵ��*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);
		return controller;
	}
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> func_root(new codeit::function::FuncRoot);
		func_root->funcPool().add<codeit::function::Mode>();
		return func_root;
	}
	auto updateStateRt(codeit::core::Msg& msg)->void {}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
