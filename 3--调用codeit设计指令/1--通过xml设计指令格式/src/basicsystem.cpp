#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	
	/*函数返回的是一个类指针，指针指向controller的类型是智能指针
	std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*创建std::unique_ptr实例*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);

		std::string xml_str =
			"<EthercatSlave phy_id=\"0\" product_code=\"0x00201\""
			" vendor_id=\"0x00000A09\" revision_num=\"0x64\" dc_assign_activate=\"0x00\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"Dout_0_7\" index=\"0x7001\" subindex=\"0x01\" size=\"8\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatSlave>";
		controller->slavePool().add<codeit::controller::EthercatSlave>().loadXmlStr(xml_str);

		return controller;
	};

	auto updateStateRt(codeit::core::Msg& msg)->void {}

	auto createDefaultData(codeit::model::Model& model)->void {}
}
