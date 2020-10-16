#include "basicsystem.hpp"
namespace codeit::system
{
	/*函数返回的是一个类指针，指针指向controller的类型是智能指针 std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*创建std::unique_ptr实例*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);
		return controller;
	}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
