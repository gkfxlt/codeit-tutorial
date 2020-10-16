#include "basicsystem.hpp"
namespace codeit::system
{
	/*�������ص���һ����ָ�룬ָ��ָ��controller������������ָ�� std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*����std::unique_ptrʵ��*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);
		return controller;
	}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
