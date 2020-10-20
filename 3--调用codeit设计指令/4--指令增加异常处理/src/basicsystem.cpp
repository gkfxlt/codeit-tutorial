#include "basicsystem.hpp"
#include"xfunc.hpp"

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
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> func_root(new codeit::function::FuncRoot);
		func_root->funcPool().add<codeit::function::MoveJS>();
		func_root->funcPool().add<codeit::function::Clear>();
		return func_root;
	}
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>
	{
		std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<codeit::system::ErrorInfo>);

		/////慎用冒号，它要放在句尾，后面跟数字代表轴号,后面的内容都会在多语言中显示出来

		errorinfoPool->add<codeit::system::ErrorInfo>
			("an exception", -10, "WARNNING", "一个异常出现", "an exception exists");

		///****// executerRT(),运动算法错误，需加入errMsgMap //
		auto& cs = ControlSystem::instance();
		auto& errMap = cs.errorMap();
		errorinfoPool->add<codeit::system::ErrorInfo>
			("plan over time", -2001, "ERROR", "规划超时", "plan over time");
		errMap.insert(pair<std::int32_t, string>(-2001, "plan over time"));

		return errorinfoPool;


	}
	////*********************************************************//////
	auto updateStateRt(codeit::core::Msg& msg)->void {}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
