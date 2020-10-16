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
	auto createSocketController(const NumList* numList, std::string name, std::string ip, std::string port, SocketMaster::TYPE type, Size pack_size, Size nrt_id)->std::unique_ptr<codeit::controller::NrtController>
	{
		static const NumList socket_num_list = { 0 };
		numList = numList ? numList : &socket_num_list;


		std::unique_ptr<codeit::controller::NrtController> controller(new codeit::controller::SocketController(name, ip, port, type, pack_size, nrt_id));

		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<SocketDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDI>";
			controller->slavePool().add<codeit::controller::SocketDI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<SocketDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDO>";
			controller->slavePool().add<codeit::controller::SocketDO>().loadXmlStr(xml_str);
			begin_index++;
		}


		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };

		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<SocketAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAI>";
			controller->slavePool().add<codeit::controller::SocketAI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<SocketAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAO>";
			controller->slavePool().add<codeit::controller::SocketAO>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<SocketSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketSubSystem>";
			controller->slavePool().add<codeit::controller::SocketSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;
	}

	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>
	{
		std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>> nrtControllerPool(new core::ObjectPool<codeit::controller::NrtController>);

		int i = 0;
		{
			NumList num{ 1,0,0,0,0,1 };
			auto sock = createSocketController(&num, "command", "", "6000", SocketMaster::TYPE::TCP, 0, i++).release();
			nrtControllerPool->add(sock);
		}

		return std::move(nrtControllerPool);
	}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
