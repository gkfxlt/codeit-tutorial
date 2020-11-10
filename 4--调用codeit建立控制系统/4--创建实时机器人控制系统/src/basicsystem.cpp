#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto updateStateRt(codeit::core::Msg& msg)->void {}
	//*************************创建控制器***********************//////
	auto createVrepController()->std::unique_ptr<codeit::controller::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
	{
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::VrepController);

		for (Size i = 0; i < 6; ++i)
		{
			double zero_offset[6]
			{
				0,0 * PI / 2,0,0 * PI / 2,0,0
			};
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
			double pos_factor[6]
			{
				360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
			};
			double tor_const[6]
			{
				0.283 * 4808,0.283 * 4808,0.276 * 2546,0.226 * 1556,0.219 * 849,0.219 * 849
			};

			//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
			//+std::to_string(min_pos[i]) +
			std::string joint_handle = "UR5_joint" + std::to_string(i + 1);
			std::string xml_str =
				"<VrepMotor phy_id=\"" + std::to_string(i) + "\" handle=\"" + joint_handle + "\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" tor_const=\"" + std::to_string(tor_const[i]) + "\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</VrepMotor>";
			controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
#ifndef codeit_USE_ETHERCAT_SIMULATION
			//dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
		}

		for (Size i = 0; i < 6; ++i)
		{
			double zero_offset[6]
			{
				0,0 * PI / 2,0,0 * PI / 2,0,0
			};
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
			double pos_factor[6]
			{
				360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
			};
			double tor_const[6]
			{
				0.283 * 4808,0.283 * 4808,0.276 * 2546,0.226 * 1556,0.219 * 849,0.219 * 849
			};

			//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
			//+std::to_string(min_pos[i]) +
			std::string joint_handle = "UR5_joint" + std::to_string(i + 1) + "#0";
			std::string xml_str =
				"<VrepMotor phy_id=\"" + std::to_string(i + 6) + "\" handle=\"" + joint_handle + "\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" tor_const=\"" + std::to_string(tor_const[i]) + "\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</VrepMotor>";
			controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
#ifndef codeit_USE_ETHERCAT_SIMULATION
			//dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
		}

		// 外部轴 //
	//	for (Size i = 0; i < 2; ++i)
	//	{
	//		double zero_offset[6]
	//		{
	//			0,0 * PI / 2,0,0 * PI / 2,0,0
	//		};
	//		double pos_offset[6]
	//		{
	//			0,0,0,0,0,0
	//		};
	//		double pos_factor[6]
	//		{
	//			360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
	//		};
	//		double max_pos[6]
	//		{
	//			170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
	//		};
	//		double min_pos[6]
	//		{
	//			-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
	//		};
	//		double max_vel[6]
	//		{
	//			310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
	//		};
	//		double max_acc[6]
	//		{
	//			15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
	//		};
	//		double tor_const[6]
	//		{
	//			0.283 * 4808,0.283 * 4808,0.276 * 2546,0.226 * 1556,0.219 * 849,0.219 * 849
	//		};
	//
	//		//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
	//		//+std::to_string(min_pos[i]) +
	//		std::string joint_handle = "UR3_joint" + std::to_string(i + 1);
	//		std::string xml_str =
	//			"<VrepExternMotor phy_id=\"" + std::to_string(i+6) + "\" handle=\"" + joint_handle + "\""
	//			" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
	//			" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
	//			" tor_const=\"" + std::to_string(tor_const[i]) + "\""
	//			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
	//			"	<SyncManagerPoolObject>"
	//			"		<SyncManager is_tx=\"false\"/>"
	//			"		<SyncManager is_tx=\"true\"/>"
	//			"		<SyncManager is_tx=\"false\">"
	//			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
	//			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
	//			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
	//			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
	//			"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
	//			"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
	//			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
	//			"			</Pdo>"
	//			"		</SyncManager>"
	//			"		<SyncManager is_tx=\"true\">"
	//			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
	//			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
	//			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
	//			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
	//			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
	//			"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
	//			"			</Pdo>"
	//			"		</SyncManager>"
	//			"	</SyncManagerPoolObject>"
	//			"</VrepExternMotor>";
	//		controller->slavePool().add<codeit::controller::VrepExternMotor>().loadXmlStr(xml_str);
	//#ifndef codeit_USE_ETHERCAT_SIMULATION
	//		//dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
	//#endif
	//	}

		//for (Size i = 0; i < 6; ++i)
		//{
		//	double pos_offset[6]
		//	{
		//		0,0,0,0,0,0
		//	};
		//	double pos_factor[6]
		//	{
		//		360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
		//	};
		//	double max_pos[6]
		//	{
		//		170.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
		//	};
		//	double min_pos[6]
		//	{
		//		-170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
		//	};
		//	double max_vel[6]
		//	{
		//		310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
		//	};
		//	double max_acc[6]
		//	{
		//		1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
		//	};
		//	//+std::to_string(min_pos[i]) +
		//	std::string joint_handle = "joint_" + std::to_string(i + 1);
		//	std::string xml_str =
		//		"<VrepMotor phy_id=\"" + std::to_string(i) + "\" handle=\"" + joint_handle + "\""
		//		" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
		//		" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
		//		" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
		//		"	<SyncManagerPoolObject>"
		//		"		<SyncManager is_tx=\"false\"/>"
		//		"		<SyncManager is_tx=\"true\"/>"
		//		"		<SyncManager is_tx=\"false\">"
		//		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
		//		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		//		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		//		"			</Pdo>"
		//		"		</SyncManager>"
		//		"		<SyncManager is_tx=\"true\">"
		//		"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
		//		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		//		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
		//		"			</Pdo>"
		//		"		</SyncManager>"
		//		"	</SyncManagerPoolObject>"
		//		"</VrepMotor>";
		//	controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
		//	//#ifndef codeit_USE_ETHERCAT_SIMULATION
		//	//		dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
		//	//#endif
		//}


		//std::unique_ptr<codeit::control::Controller> controller(robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
		//std::cout << controller->xmlString() << endl;
		////ATI force sensor//
		//std::string xml_str =
		//	"<VrepSlave phy_id=\"6\">"
		//	"	<SyncManagerPoolObject>"
		//	"		<SyncManager is_tx=\"false\"/>"
		//	"		<SyncManager is_tx=\"true\"/>"
		//	"		<SyncManager is_tx=\"false\">"
		//	"			<Pdo index=\"0x1601\" is_tx=\"false\">"
		//	"				<PdoEntry name=\"Control_1\" index=\"0x7010\" subindex=\"0x01\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Control_2\" index=\"0x7010\" subindex=\"0x02\" size=\"32\"/>"
		//	"			</Pdo>"
		//	"		</SyncManager>"
		//	"		<SyncManager is_tx=\"true\">"
		//	"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
		//	"				<PdoEntry name=\"Int_Input_Fx\" index=\"0x6000\" subindex=\"0x01\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Int_Input_Fy\" index=\"0x6000\" subindex=\"0x02\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Int_Input_Fz\" index=\"0x6000\" subindex=\"0x03\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Int_Input_Mx\" index=\"0x6000\" subindex=\"0x04\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Int_Input_My\" index=\"0x6000\" subindex=\"0x05\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Int_Input_Mz\" index=\"0x6000\" subindex=\"0x06\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Status_Code\" index=\"0x6010\" subindex=\"0x00\" size=\"32\"/>"
		//	"				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"32\"/>"
		//	"			</Pdo>"
		//	"		</SyncManager>"
		//	"	</SyncManagerPoolObject>"
		//	"</VrepSlave>";
		//controller->slavePool().add<codeit::control::VrepSlave>().loadXmlStr(xml_str);

		//for (Size i = 0; i < 4; ++i)
		//{
		//	double zero_offset[6]
		//	{
		//		0,0 * PI / 2,0,0 * PI / 2,0,0
		//	};
		//	double pos_offset[6]
		//	{
		//		0,0,0,0,0,0
		//	};
		//	double pos_factor[6]
		//	{
		//		360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
		//	};
		//	double max_pos[6]
		//	{
		//		170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
		//	};
		//	double min_pos[6]
		//	{
		//		-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
		//	};
		//	double max_vel[6]
		//	{
		//		310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
		//	};
		//	double max_acc[6]
		//	{
		//		15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
		//	};
		//	//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
		//	//+std::to_string(min_pos[i]) +
		//	std::string joint_handle = "MTB_axis" + std::to_string(i + 1);
		//	std::string xml_str =
		//		"<VrepMotor phy_id=\"" + std::to_string(i) + "\" handle=\"" + joint_handle + "\""
		//		" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
		//		" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
		//		" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
		//		"	<SyncManagerPoolObject>"
		//		"		<SyncManager is_tx=\"false\"/>"
		//		"		<SyncManager is_tx=\"true\"/>"
		//		"		<SyncManager is_tx=\"false\">"
		//		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
		//		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
		//		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
		//		"			</Pdo>"
		//		"		</SyncManager>"
		//		"		<SyncManager is_tx=\"true\">"
		//		"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
		//		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
		//		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
		//		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
		//		"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
		//		"			</Pdo>"
		//		"		</SyncManager>"
		//		"	</SyncManagerPoolObject>"
		//		"</VrepMotor>";
		//	controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
		//	//#ifndef codeit_USE_ETHERCAT_SIMULATION
		//	//		dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
		//	//#endif
		//}

		return controller;
	}
	auto createEcatController()->std::unique_ptr<codeit::controller::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
{
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);/*创建std::unique_ptr实例*/
		controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
        for (Size i = 0; i < 1; ++i)
        {
#ifdef UNIX
            double pos_offset[6]
            {
               1.4327,   2.41069874505508,   0.762016940604939,   4.69692333813102,   0.0360485485152105,   1.43257027673246
            };
#endif
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
            double pos_factor[6]
            {
                1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, -1048576.0 / 2 / PI, -524288.0 / 2 / PI, 524288.0 / 2 / PI
            };
            double max_pos[6]
            {
                180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	60.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
            };
            double min_pos[6]
            {
                -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -240.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI
            };
            double max_vel[6]
            {
                0.4, 0.4, 0.4, 0.4, 0.6, 0.6
            };
            double max_acc[6]
            {
//                3.3, 3.3*2, 3.3, 3.3, 7.5, 7.5
//                10, 10, 10, 10, 25, 25
                  20,20,20,20,50,50
            };

            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x201\""
                " vendor_id=\"0x000022D2\" revision_num=\"0x0a000002\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
                " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
                "	<SyncManagerPoolObject>"
                "		<SyncManager is_tx=\"false\"/>"
                "		<SyncManager is_tx=\"true\"/>"
                "		<SyncManager is_tx=\"false\">"
                "			<Pdo index=\"0x1600\" is_tx=\"false\">"
                "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "		<SyncManager is_tx=\"true\">"
                "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
                "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "	</SyncManagerPoolObject>"
                "</EthercatMotor>";
            controller->slavePool().add<codeit::controller::EthercatMotor>().loadXmlStr(xml_str);
        }

#ifdef UNIX
        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();

        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();
#endif
        std::cout << controller->xmlString() << std::endl;



		return controller;
}
    auto createZeroErrEcatController()->std::unique_ptr<codeit::controller::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
{
        std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);/*创建std::unique_ptr实例*/
        controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
        for (Size i = 0; i < 1; ++i)
        {
#ifdef UNIX
            double pos_offset[6]
            {
               1.4327,   2.41069874505508,   0.762016940604939,   4.69692333813102,   0.0360485485152105,   1.43257027673246
            };
#endif
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
            double pos_factor[6]
            {
                1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, -1048576.0 / 2 / PI, -524288.0 / 2 / PI, 524288.0 / 2 / PI
            };
            double max_pos[6]
            {
                180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	60.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
            };
            double min_pos[6]
            {
                -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -240.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI
            };
            double max_vel[6]
            {
                0.4, 0.4, 0.4, 0.4, 0.6, 0.6
            };
            double max_acc[6]
            {
//                3.3, 3.3*2, 3.3, 3.3, 7.5, 7.5
//                10, 10, 10, 10, 25, 25
                  20,20,20,20,50,50
            };

            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x29252\""
                " vendor_id=\"0x5A65726F\" revision_num=\"0x00000001\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
                " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
                "	<SyncManagerPoolObject>"
                "		<SyncManager is_tx=\"false\"/>"
                "		<SyncManager is_tx=\"true\"/>"
                "		<SyncManager is_tx=\"false\">"
                "			<Pdo index=\"0x1600\" is_tx=\"false\">"
                "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
                "               <PdoEntry name=\"mode_of_operation\" index=\"0x60FE\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"control_tmp\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "		<SyncManager is_tx=\"true\">"
                "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
                "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"mode_of_display\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "	</SyncManagerPoolObject>"
                "</EthercatMotor>";
            controller->slavePool().add<codeit::controller::EthercatMotor>().loadXmlStr(xml_str);
        }

#ifdef UNIX
        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();

        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();
#endif
        std::cout << controller->xmlString() << std::endl;



        return controller;
}
	auto createSocketController(const NumList* numList,std::string name, std::string ip, std::string port, SocketMaster::TYPE type, Size nrt_id)->std::unique_ptr<codeit::controller::NrtController>
	{
		static const NumList socket_num_list= { 0 };
		numList = numList ? numList : &socket_num_list;


		std::unique_ptr<codeit::controller::NrtController> controller(new codeit::controller::SocketController(name,ip,port,type,nrt_id));

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

		for (Size i = 0; i < numList->motor_num; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<SocketMotor phy_id=\"" + std::to_string(begin_index) + "\" "
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"</SocketMotor>";

			controller->slavePool().add<codeit::controller::SocketMotor>().loadXmlStr(xml_str);
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
	auto createComController(const NumList* numList, const std::string& name, const core::SerialPort::ComOptions& options, Size pack_size, Size nrt_id)->std::unique_ptr<codeit::controller::NrtController>
	{
		static const NumList com_num_list = { 0 };
		numList = numList ? numList : &com_num_list;

		std::unique_ptr<codeit::controller::NrtController> controller(new codeit::controller::ComController(name, options, pack_size, nrt_id));

		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<ComDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDI>";
			controller->slavePool().add<codeit::controller::ComDI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<ComDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDO>";
			controller->slavePool().add<codeit::controller::ComDO>().loadXmlStr(xml_str);
			begin_index++;
		}

		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<ComAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAI>";
			controller->slavePool().add<codeit::controller::ComAI>().loadXmlStr(xml_str);
			begin_index++;
		}
		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<ComAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAO>";
			controller->slavePool().add<codeit::controller::ComAO>().loadXmlStr(xml_str);
			begin_index++;
		}


		for (Size i = 0; i < numList->motor_num; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<ComMotor phy_id=\"" + std::to_string(begin_index) + "\" "
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"</ComMotor>";

			controller->slavePool().add<codeit::controller::ComMotor>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<ComSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComSubSystem>";
			controller->slavePool().add<codeit::controller::ComSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;

	}
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>
	{
		std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>> nrtControllerPool(new core::ObjectPool<codeit::controller::NrtController>);
		
		int i = 0;
		NumList num0{ 6,6,6,6,6,1 };
		auto sock1 = createSocketController(&num0,"state", "", "6001", SocketMaster::TYPE::TCP,i++).release();
		NumList num1{ 0,0,0,0,0,1 };
		auto sock0 = createSocketController(&num1,"command", "", "6000", SocketMaster::TYPE::TCP,i++).release();
		//auto com0 = createComController(&num0,i++,"com", 4, 9600).release();


		//nrtControllerPool->add(com0);
		nrtControllerPool->add(sock1);
		nrtControllerPool->add(sock0);
		

		return std::move(nrtControllerPool);


	}
	//*************************创建模型************************//////
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
	auto createModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::Model>>
	{
		std::unique_ptr<core::ObjectPool<codeit::model::Model>> modelPool(new core::ObjectPool<codeit::model::Model>);
		{
			codeit::model::PumaParam param;
			param.d1 = 0.322;
			param.a1 = 0.088;
			param.a2 = 0.46;
			param.d3 = 0.0;
			param.a3 = 0.0418;
			param.d4 = 0.4342;
			param.tool0_pe[2] = 0.27544;
			auto model = createModelPuma(param, "MJ08").release();
			modelPool->add(model);
		}
		{
			codeit::model::UrParam param;
			param.L1 = 0.425;
			param.L2 = 0.39225;
			param.W1 = 0.13585 - 0.1197 + 0.093;
			param.W2 = 0.0823;
			param.H1 = 0.089159;
			param.H2 = 0.09465;
			auto model = createModelUr(param, "UR5").release();
			modelPool->add(model);
		}
		{
			codeit::model::ScaraParam param;
			param.a2 = 0.467;
			param.a3 = 0.4005;
			param.d3 = 0.08;
			auto model = createModelScara(param, "Scara").release();
			//modelPool->add(model);
		}

		for (Size i = 0; i < modelPool->size(); i++)
			createDefaultData(modelPool->at(i));
		return std::move(modelPool);
	}
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>
	{
		std::unique_ptr<core::ObjectPool<codeit::model::IOModel>> iomodelPool(new core::ObjectPool<codeit::model::IOModel>);
		NumList num0{ 6,6,6,6,6,1 };
		auto iomodel0 = createIOModel(&num0, "com0").release();
		NumList num1{ 0,0,0,0,0,1 };
		auto iomodel1 = createIOModel(&num1, "sock1").release();
		auto iomodel2 = createIOModel(&num0, "sock0").release();
		iomodelPool->add(iomodel0);
		iomodelPool->add(iomodel2);
		iomodelPool->add(iomodel1);

		NumList num2{ 0,0,0,0,0,0 };
		auto iomodel3 = createIOModel(&num2, "ecat_io").release();
		//iomodelPool->add(iomodel3);

		return std::move(iomodelPool);
	}
	

	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>
	{
		std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<codeit::system::ErrorInfo>);
		auto& cs = ControlSystem::instance();
		auto& errMap = cs.errorMap();
		defaultErrorInfoPool(*errorinfoPool, errMap);
		int code = -2;
		std::string errLevel = "ERROR";
		/////慎用冒号，它要放在句尾，后面跟数字代表轴号,后面的内容都会在多语言中显示出来

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master name not exist", code, errLevel, "主站不存在", "master name not exist");

		return errorinfoPool;
	}
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> cmd_root(new codeit::function::FuncRoot);
		defaultFuncRoot(*cmd_root);

		////////////////////////*************示例**************////////////
		//cmd_root->funcPool().add<codeit::function::MoveSine>();

		return cmd_root;
	}
	auto createUserDataType(core::Calculator& cal)->void
	{
		defaultUserDataType(cal);
		/*cal.addTypename("wobj");
		cal.addFunction("wobj", std::vector<std::string>{"Matrix"}, "wobj", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
		cal.addBinaryOperatorFunction("=", "wobj", "Matrix", "wobj", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});*/
	}
	auto createDefaultData(codeit::model::Model& model)->void
	{
		defaultModelData(model);
		//double load[10] = { 0 };
		//model.variablePool().add<codeit::model::MatrixVariable>("load0", core::Matrix(1, 10, load));
	}
}
