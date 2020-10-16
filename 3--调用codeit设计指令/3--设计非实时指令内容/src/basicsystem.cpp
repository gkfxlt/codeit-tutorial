#include "basicsystem.hpp"
#include"xfunc.hpp"

namespace codeit::system
{
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
	auto createIOModel(const NumList* numList, std::string name)->std::unique_ptr<codeit::model::IOModel>
	{
		static const NumList model_num_list = { 0 };
		numList = numList ? numList : &model_num_list;
		std::unique_ptr<codeit::model::IOModel> iomodel(new codeit::model::IOModel(name));

		for (Size i = 0; i < numList->di_num; ++i)
			iomodel->ioPool().add<model::DI>(name + "_di" + std::to_string(i));

		for (Size i = 0; i < numList->do_num; ++i)
			iomodel->ioPool().add<model::DO>(name + "_do" + std::to_string(i));

		for (Size i = 0; i < numList->ao_num; ++i)
			iomodel->ioPool().add<model::AI>(name + "_ai" + std::to_string(i));

		for (Size i = 0; i < numList->ai_num; ++i)
			iomodel->ioPool().add<model::AO>(name + "_ao" + std::to_string(i));

		/*for (Size i = 0; i < numList->motor_num; ++i)
			iomodel->motorPool().add<model::Motion>(name + "motor" + std::to_string(i));
		*/

		for (Size i = 0; i < numList->subsys_num; ++i)
			iomodel->subsysPool().add<model::SubSysElement>(name + "_subsys" + std::to_string(i));

		return std::move(iomodel);
	}

	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>
	{
		std::unique_ptr<core::ObjectPool<codeit::model::IOModel>> iomodelPool(new core::ObjectPool<codeit::model::IOModel>);
		NumList num0{ 0,1,0,0,0,0 };
		auto iomodel = createIOModel(&num0, "sock0").release();
		iomodelPool->add(iomodel);
	
		return std::move(iomodelPool);
	}

	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> func_root(new codeit::function::FuncRoot);
		func_root->funcPool().add<codeit::function::MoveNrt>();
		func_root->funcPool().add<codeit::function::Disable>();
		return func_root;
	}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
