#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto createModel(std::string name)->std::unique_ptr<codeit::model::Model>
	{
		std::unique_ptr<codeit::model::Model> model = std::make_unique<codeit::model::Model>(name);

		// �������� //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// ��ӱ��� //
		model->calculator().addVariable("PI", "Matrix", codeit::core::Matrix(PI));
		// ����ؽڵ�λ�ã��Լ����ߣ�����1��ת������������Z��
		const double joint1_position[3]{ 0 , 0 , 0 };
		const double joint1_axis[3]{ 0 , 0 , 1 };
		// ����˼���λ����321ŷ���ǣ��Լ�10ά�Ĺ�������
		// inertia_vector�Ķ���Ϊ��[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]������x,y,zΪ����λ��
		const double link1_position_and_euler321[6]{ 0 , 0 , 0 , 0 , 0 , 0 };
		const double link1_inertia_vector[10]{ 0 , 0 , 0 , 0 , 0 , 0, 0 , 0, 0, 0 };
		// ����ĩ��λ����321ŷ���ǣ����λ��Ϊ������ʼʱ��λ��
		const double end_effector_position_and_euler321[6]{ 0 , 0, 0 , 0 , 0 , 0 };
		// ��Ӹ˼�������pe����˼Ϊposition and euler angle�������Ĳ���ָ����λ���Լ���������
		auto& link1 = model->addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
		// ��ӹؽڣ����ת���ؽڣ�ǰ��������Ϊ�ؽ����ӵĸ˼������������������˹ؽڵ�λ��������
		auto& joint1 = model->addRevoluteJoint(link1, model->ground(), joint1_position, joint1_axis);
		// �������������λ�ڹؽ���
		auto& motion1 = model->addMotion(joint1);

		return model;
	}
	auto createModelPool()->std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>>
	{
		std::unique_ptr<codeit::core::ObjectPool<codeit::model::Model>> modelPool(new codeit::core::ObjectPool<codeit::model::Model>);
		auto model = createModel("motor").release();
		modelPool->add(model);

		for (Size i = 0; i < modelPool->size(); i++)
			codeit::system::createDefaultData(modelPool->at(i));
		return modelPool;
	}
	
	/*�������ص���һ����ָ�룬ָ��ָ��controller������������ָ��
	std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*����std::unique_ptrʵ��*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);
		/*����Ethercat��������xmal�ļ�
			phy_idָ�������ţ�������������˴������0�ŵ����xml����
			product_code��vendor_id�������ɿ�������ȡ
			min_pos��max_pos���� ���г��йأ�ǰ��С�ڵ�׵���Сλ��0mm����
				 �ߴ��ڵ�׵�����γ�100mm
			max_vel���ݵ�׵Ķת�ٺ��г������㣬��
				 3000��ת�٣�*4�����̣�/60/1000=0.2����λm/s��
			min_vel��max_vel��С��ȣ������෴
			max_acc���վ��鶨��Ϊ�ٶȵ�10��
			max_pos_following_error��max_vel_following_error�ɾ�������ȷ��
			home_posָ��׵ĳ�ʼλ�ã�����Ϊ0
			pos_factorָ������ƽ�1�׵�����£��������ĵ��źŸ�����ͨ����ѯ���
				 Ϊ23bit������ת��һȦ������µ�����Ĵ�����2^23=8388608���������
				 ��һ����Ҫת��250Ȧ����pos_factor=8388608*250=2097152000
			pos_offset��ָ����ڶϵ����������ĳ�ʼλ�þ���0���ƫ��*/
		std::string xml_str =
			"<EthercatMotor phy_id=\"0\" product_code=\"0x60380005\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"-0.1\" max_pos=\"0.15\" max_vel=\"0.2\" min_vel=\"-0.2\""
			" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
			" home_pos=\"0\" pos_factor=\"2097152000\" pos_offset=\"0.00383346\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatMotor>";
		controller->slavePool().add<codeit::controller::EthercatMotor>().loadXmlStr(xml_str);

		return controller;
	};

	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> func_root(new codeit::function::FuncRoot);
		func_root->funcPool().add<codeit::function::MoveJS>();
		func_root->funcPool().add<codeit::function::Disable>();

		return func_root;
	}


	//*************************�����쳣��Ϣ��************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>
	{
		std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<codeit::system::ErrorInfo>);

		/////����ð�ţ���Ҫ���ھ�β����������ִ������,��������ݶ����ڶ���������ʾ����

		////****��errorinfo������쳣����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unknown errorinfo", -2, "ERROR", "���쳣��Ϣδע��", "unregistered errorinfo");
		///****��errorinfo������쳣����--------------------------------------




		///****�ؽڲ���
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input pos beyond range-joint", -2, "ERROR", "����λ�ó���", "input position beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input vel beyond range-joint", -2, "ERROR", "�����ٶȳ���", "input velocity beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input acc beyond range-joint", -2, "ERROR", "������ٶȳ���", "input acc beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input dec beyond range-joint", -2, "ERROR", "������ٶȳ���", "input dec beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-joint", -2, "ERROR", "���ٶ�ά����ƥ��", "acc dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec dimension mismatch-joint", -2, "ERROR", "���ٶ�ά����ƥ��", "dec dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk dimension mismatch-joint", -2, "ERROR", "�Ӽ��ٶ�ά����ƥ��", "jerk dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pos dimension mismatch-joint", -2, "ERROR", "λ��ά����ƥ��", "pos dimension mismatch-joint");
		///****�ؽڲ���-----------------------------------------------


		///****Ŀ��λ��
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-robottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-robottarget", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-robottarget", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-robottarget", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-robottarget", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-robottarget", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-robottarget", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-robottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-robottarget", -2, "ERROR", "����ĹؽڽǶ�ά����ƥ��", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-robottarget", -2, "ERROR", "�˶�ѧ����ʧ��", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-robottarget", -2, "ERROR", "robottarget����������", "robottarget param not exist");
		///****Ŀ��λ��----------------------------------------------

		///****�м�λ��
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-midrobottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-midrobottarget", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-midrobottarget", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-midrobottarget", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-midrobottarget", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-midrobottarget", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-midrobottarget", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-midrobottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-midrobottarget", -2, "ERROR", "����ĹؽڽǶ�ά����ƥ��", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-midrobottarget", -2, "ERROR", "�˶�ѧ����ʧ��", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-midrobottarget", -2, "ERROR", "robottarget����������", "robottarget param not exist");
		///****�м�λ��-----------------------------------------------

		///****��������ϵ
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-tool", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-tool", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-tool", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-tool", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-tool", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-tool", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-tool", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-tool", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-tool", -2, "ERROR", "tool����������", "tool param not exist");

		///****��������ϵ---------------------------------------------


		///****Wobj����ϵ
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-wobj", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-wobj", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-wobj", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-wobj", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-wobj", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-wobj", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-wobj", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-wobj", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-wobj", -2, "ERROR", "wobj����������", "wobj param not exist");

		///****Wobj����ϵ--------------------------------------------

		///****zone����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-zone", -2, "ERROR", "zone����������", "zone param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("out of range-zone", -2, "ERROR", "zone����Խ��", "zone out of range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-zone", -2, "ERROR", "zone����ά����ƥ��", "zone dimension mismatch");
		///****zone����-----------------------------------------------

		///****speed����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-speed", -2, "ERROR", "�ٶȲ���������", "speed param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-speed", -2, "ERROR", "�ٶȲ���ά����ƥ��", "speed dimension mismatch");
		///****speed����---------------------------------------------

		///****load����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-load", -2, "ERROR", "load����������", "load param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-load", -2, "ERROR", "load����ά����ƥ��", "load dimension mismatch");
		///****load����----------------------------------------------


		///****jointtarget����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-jointtarget", -2, "ERROR", "�ؽ�Ŀ�����������", "jointtarget param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-jointtarget", -2, "ERROR", "�ؽ�Ŀ�����ά����ƥ��", "jointtarget dimension mismatch");
		///****jointarget����--------------------------------------

		///****�ٶȡ����ٶȡ����ٶȡ��Ӽ��ٶȲ���
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc is negative", -2, "ERROR", "���ٶ�Ϊ����", "acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec is positive", -2, "ERROR", "���ٶ�Ϊ����", "dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk is negative", -2, "ERROR", "�Ӽ��ٶ�Ϊ����", "jerk is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmax is negative", -2, "ERROR", "�Ӽ��ٶ�Ϊ����", "jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmin is positive", -2, "ERROR", "�����ٶ�Ϊ����", "jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmax is negative", -2, "ERROR", "�ǼӼ��ٶ�Ϊ����", "angular_jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmin is positive", -2, "ERROR", "�Ǽ����ٶ�Ϊ����", "angular_jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_acc is negative", -2, "ERROR", "�Ǽ��ٶ�Ϊ����", "angular_acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_dec is positive", -2, "ERROR", "�Ǽ��ٶ�Ϊ����", "angular_dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jerk is negative", -2, "ERROR", "�ǼӼ��ٶ�Ϊ����", "angular_jerk is negative");
		///****�ٶȡ����ٶȡ����ٶȡ��Ӽ��ٶȲ���------------------------------------------


		///****ʾ�̵����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-teachpoint", -2, "ERROR", "��ʾ�̵������Ѵ���", "param already exist-teachpoint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-teachpoint", -2, "ERROR", "��ʾ�̵����Ʋ�����", "param not exist-teachpoint");
		///****ʾ�̵����------------------------------------------

		///****������������
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-definevar", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-definevar", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definespeed", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definezone", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-defineload", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-defineload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definejointtarget", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definejointtarget");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-definevar", -2, "ERROR", "���ʹ���", "type error-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		///****������������------------------------------------------

		///****������������
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-saveparam", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-saveparam", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savespeed", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-savespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savezone", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-savezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-saveload", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-saveload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-saveparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param already exist-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-saveparam", -2, "ERROR", "���ʹ���", "type error-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-deleteparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param not exist-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-deleteparam", -2, "ERROR", "���ʹ���", "type error-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-renameparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param not exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("new param already exist-renameparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "new param already exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-renameparam", -2, "ERROR", "���ʹ���", "type error-renameparam");
		///****������������------------------------------------------

		///****JogJ��JogC����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("last_count is negative", -2, "ERROR", "����ʱ��Ϊ����", "last_count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("error direction", -2, "ERROR", "jog�������ô���", "error direction");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint beyond range", -2, "ERROR", "�ؽ�ѡ����", "joint beyond range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("coordinate error", -2, "ERROR", "����ϵ�趨����", "coordinate error");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("motion type error", -2, "ERROR", "�˶������趨����", "motion type error");
		///****JogJ��JogC����--------------------------------------

		///****ServoJ����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "ʱ������Ϊ����", "time is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-servoJ", -2, "ERROR", "���ٶ�ά����ƥ��", "acc dimension mismatch-servoJ");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("look_ahead_time is negative", -2, "ERROR", "ǰհʱ����Ϊ����", "look_ahead_time is negative");
		///****ServoJ����--------------------------------------


		///****IO ָ�����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DO name not exist", -2, "ERROR", "do ������", "DO name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DI name not exist", -2, "ERROR", "di ������", "DI name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("model name not exist", -2, "ERROR", "ģ�Ͳ�����", "model name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("do_value dimension mismatch do_name", -2, "ERROR", "���ݳ��Ȳ�ƥ��", "do_value dimension mismatch do_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cmd dimension mismatch model_name", -2, "ERROR", "cmd���Ȳ�ƥ��", "cmd dimension mismatch model_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no subsys", -2, "ERROR", "û��subsys", "no subsys");

		///****IO ָ�����--------------------------------------


		///****MoveS,Calibrator����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq set dimension mismatch", -2, "ERROR", "pq������ά����ƥ��", "pq set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm set dimension mismatch", -2, "ERROR", "pm������ά����ƥ��", "pm set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa set dimension mismatch", -2, "ERROR", "pa������ά����ƥ��", "pa set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe set dimension mismatch", -2, "ERROR", "pe������ά����ƥ��", "pe set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint set dimension mismatch", -2, "ERROR", "�ؽڽǶȲ�����ά����ƥ��", "joint set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic pm failed", -2, "ERROR", "�����˶�ѧ��ȡpmʧ��", "forward kinematic pm failed");

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("regressor matrix not full rank", -2, "ERROR", "�ع��������", "regressor matrix not full rank");

		///****MoveS,Calibrator����--------------------------------------


		///****����ָ�����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("count is negative", -2, "ERROR", "����ʱ��Ϊ����", "count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("program_rate", -2, "ERROR", "��������Ϊ����", "program_rate");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cs is running", -2, "ERROR", "ʵʱ�߳���������", "cs is running, please stop the cs using cs_stop!");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to stop server", -2, "ERROR", "ֹͣ������ʧ��", "failed to stop server, because it is not running");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to start server", -2, "ERROR", "����������ʧ��", "failed to start server, because it is already started");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no model", -2, "ERROR", "��ģ�Ͳ�����", "no model");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "�ȴ�ʱ��Ϊ����", "wait time is negative");

		///****����ָ�����--------------------------------------

		///****ָ������쳣��Դ��command.cpp
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("brace not pair", -2, "ERROR", "û�гɶԵ�����", "brace not pair");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("empty command string", -2, "ERROR", "�ַ�������Ϊ��", "invalid command string: please at least contain a word");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("invalid command name", -2, "ERROR", "��Чָ��", "server does not have this command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param should not start with '='", -2, "ERROR", "����������=��ʼ", "param should not start with '='");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '-' must be followed by an abbreviation of param", -2, "ERROR", "����'-'���������д", "symbol '-' must be followed by an abbreviation of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '--' must be followed by a full name of param", -2, "ERROR", "����'--'���������������", "symbol '--' must be followed by a full name of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param start with single '-' must be an abbreviation", -2, "ERROR", "��'-'��ͷ�Ĳ�����������д", "param start with single '-' must be an abbreviation");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a abbreviation of any valid param", -2, "ERROR", "��������������д", "not a abbreviation of any valid param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a valid param", -2, "ERROR", "����һ����Ч��ָ�����", "not a valid param of command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to find default param in command", -2, "ERROR", "ָ�����Ҳ���Ĭ�ϲ���", "failed to find default param in command");


		///****ָ������쳣��Դ��command.cpp--------------------------------------

		///****// step 3.  execute //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server in error", -2, "ERROR", "���������ڴ���״̬", "server in error, use cl to clear");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server not started", -2, "ERROR", "������δ����", "server not started, use cs_start to start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("command pool is full", -2, "ERROR", "ָ�������", "command pool is full");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to get current TARGET", -2, "ERROR", "��ȡ��ǰĿ��ʧ��", "failed to get current TARGET, because ControlServer is not running");
		///****// step 3.  execute //--------------------------------------

		///****// Motion Check //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion is not in OPERATION_ENABLE mode", -2, "ERROR", "��û��ʹ��", "Motion is not in OPERATION_ENABLE mode");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MAX", -2, "ERROR", "���λ��ָ������ֵ", "Motion target position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MIN", -2, "ERROR", "���λ��ָ�����Сֵ", "Motion target position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT CONTINUOUS", -2, "ERROR", "���λ��ָ��һ�ײ�����", "Motion target position NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT SECOND CONTINUOUS", -2, "ERROR", "���λ��ָ����ײ�����", "Motion target position NOT SECOND CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position has FOLLOW ERROR", -2, "ERROR", "�����λ�ø������", "Motion target position has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MAX", -2, "ERROR", "����ٶ�ָ������ֵ", "Motion target velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MIN", -2, "ERROR", "����ٶ�ָ�����Сֵ", "Motion target velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity NOT CONTINUOUS", -2, "ERROR", "����ٶ�ָ��һ�ײ�����", "Motion target velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity has FOLLOW ERROR", -2, "ERROR", "����ٶ�ָ����ڸ������", "Motion target velocity has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MAX", -2, "ERROR", "���ʵ��λ�ó������ֵ", "Motion actual position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MIN", -2, "ERROR", "���ʵ��λ�ó�����Сֵ", "Motion actual position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MAX", -2, "ERROR", "���ʵ���ٶȳ������ֵ", "Motion actual velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MIN", -2, "ERROR", "���ʵ���ٶȳ�����Сֵ", "Motion actual velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity NOT CONTINUOUS", -2, "ERROR", "���ʵ���ٶ�һ�ײ�����", "Motion actual velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion MODE INVALID", -2, "ERROR", "���ģʽ��Ч", "Motion MODE INVALID");
		///****// Motion Check //--------------------------------------


		///****// Masterͨ�� //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master Lost Connection with", -2, "ERROR", "��վʧȥ����", "Master Lost Connection with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master failed start with", -2, "ERROR", "��վ����ʧ��", "Master failed start with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master already running", -2, "ERROR", "��վ�Ѿ�����", "master already running, so cannot start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master is not running", -2, "ERROR", "��վ�Ѿ�����", "master is not running, so can't stop");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "��վ�Ѿ�����", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "��վ�Ѿ�����", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("phy id already exists", -2, "ERROR", "��վ�Ѿ�����", "phy id already exists");


		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_create failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "rt_task_create failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_rt_task_join failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "codeit_rt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_nrt_task_join failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "codeit_nrt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_start failed", -2, "ERROR", "ʵʱ��������ʧ��", "rt_task_start failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com controller initport fail", -2, "ERROR", "���ڿ������˿ڳ�ʼ��ʧ��", "Com controller initport fail");
		///****// Masterͨ�Ŵ��� //--------------------------------------


		///****// Interface���� //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't WSAstartup", -2, "ERROR", "Socket������Ϊ��վ����", "Socket can't Start as server, because it can't WSAstartup");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket has empty port", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket has empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't bind", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't Start as server, because it can't bind");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't listen", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't Start as server, because it can't listen");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty ip address", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it empty ip address");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty port", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket is busy now", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it is busy now, please close it");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't connect", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because can't connect");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt TCP_USER_TIMEOUT FAILED", -2, "ERROR", "��վ�Ѿ�����", "socket setsockopt TCP_USER_TIMEOUT FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt SO_KEEPALIVE FAILED", -2, "ERROR", "��վ�Ѿ�����", "socket setsockopt SO_KEEPALIVE FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("setsockopt failed", -2, "ERROR", "��վ�Ѿ�����", "setsockopt failed: SO_REUSEADDR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface open listen thread fail", -2, "ERROR", "��վ�Ѿ�����", "Com interface open listen thread fail");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface initPort fail", -2, "ERROR", "��վ�Ѿ�����", "Com interface initPort fail");

		///****// Interface���� //--------------------------------------


		///****// executerRT(),�˶��㷨���������errMsgMap //
		auto& cs = ControlSystem::instance();
		auto& errMap = cs.errorMap();
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unregistered key", -1000, "ERROR", "δע��ļ�ֵ", "unregistered key");
		errMap.insert(pair<std::int32_t, string>(-1000, "unregistered key"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("plan over time", codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "ERROR", "�滮��ʱ", "plan over time");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "plan over time"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed", codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "ERROR", "����ʧ��", "forward kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "forward kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("inverse kinematic failed", codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "ERROR", "���ʧ��", "inverse kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "inverse kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("wrist singularity", codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "ERROR", "�������", "wrist singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "wrist singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("shoulder singularity", codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "ERROR", "�粿�����", "shoulder singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "shoulder singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("elbow singularity", codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "ERROR", "�ⲿ�����", "elbow singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "elbow singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("three points collinear", codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "ERROR", "���㹲��", "three points collinear");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "three points collinear"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveJ planner", codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "ERROR", "û��moveJ�滮��", "no moveJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "no moveJ planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveL planner", codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "ERROR", "û��moveL�滮��", "no moveL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "no moveL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveC planner", codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "ERROR", "û��moveC�滮��", "no moveC planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "no moveC planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveS planner", codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "ERROR", "û��moveS�滮��", "no moveS planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "no moveS planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveLL planner", codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "ERROR", "û��moveLL�滮��", "no moveLL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "no moveLL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no servoJ planner", codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "ERROR", "û��servoJ�滮��", "no servoJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "no servoJ planner"));

		///****// executerRT(),�˶��㷨���� //--------------------------------------

		return errorinfoPool;


	}
	////*********************************************************//////

	auto createUserDataType(core::Calculator& cal)->void
{
	std::cout << "create user data!" << std::endl;
	cal.addTypename("array");
	cal.addFunction("array", std::vector<std::string>{"Matrix"}, "array", [](std::vector<std::any>& params)->std::any
		{
			return params[0];
		});
	
	cal.addTypename("load");
	cal.addFunction("load", std::vector<std::string>{"Matrix"}, "load", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 10)
			{
				THROW_FILE_LINE("input data error");
			}
			Load a;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp+10, &a.inertia[0]);
			/*std::copy(temp + 1, temp + 4, &a.cog[0]);
			std::copy(temp + 4, temp + 8, &a.pq[0]);
			std::copy(temp + 8, temp + 11, &a.iner[0]);*/

			return a;
		});
	cal.addBinaryOperatorFunction("=", "load", "Matrix", "load", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 10)
			{
				THROW_FILE_LINE("input data error");
			}
			Load load;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 10, &load.inertia[0]);
			
			left = load;
			return left;
		});

	
	cal.addTypename("pose");
	cal.addFunction("pose", std::vector<std::string>{"Matrix"}, "pose", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 7)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addTypename("jointtarget");
	cal.addFunction("jointtarget", std::vector<std::string>{"Matrix"}, "jointtarget", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	
	
	cal.addTypename("robottarget");
	cal.addFunction("robottarget", std::vector<std::string>{"Matrix"}, "robottarget", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	
	cal.addTypename("TeachTarget");
	

	cal.addTypename("zone");
	cal.addFunction("zone", std::vector<std::string>{"Matrix"}, "zone", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 2)
			{
				THROW_FILE_LINE("input data error");
			}
			Zone z;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.dis);
			std::copy(temp + 1, temp + 2, &z.per);

			return z;
		});
	cal.addBinaryOperatorFunction("=", "zone", "Matrix", "zone", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 2)
			{
				THROW_FILE_LINE("input data error");
			}
			Zone z;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 1, &z.dis);
			std::copy(temp + 1, temp + 2, &z.per);

			left = z;
			return left;
		});
	// add zone variables
	
	cal.addVariable("fine", "zone", Zone({ 0.0, 0.0 }));
	cal.addVariable("z1", "zone", Zone({ 0.001, 0.01 }));
	cal.addVariable("z5", "zone", Zone({ 0.005, 0.03 }));
	cal.addVariable("z10", "zone", Zone({ 0.01, 0.05 }));
	cal.addVariable("z15", "zone", Zone({ 0.015, 0.08 }));
	cal.addVariable("z20", "zone", Zone({ 0.02, 0.1 }));
	cal.addVariable("z30", "zone", Zone({ 0.03, 0.15 }));
	cal.addVariable("z40", "zone", Zone({ 0.04, 0.2 }));
	cal.addVariable("z50", "zone", Zone({ 0.05, 0.25 }));
	cal.addVariable("z60", "zone", Zone({ 0.06, 0.3 }));
	cal.addVariable("z80", "zone", Zone({ 0.08, 0.4 }));
	cal.addVariable("z100", "zone", Zone({ 0.1, 0.45 }));
	cal.addVariable("z150", "zone", Zone({ 0.15, 0.45 }));
	cal.addVariable("z200", "zone", Zone({ 0.2, 0.45 }));
	

	cal.addTypename("speed");
	cal.addFunction("speed", std::vector<std::string>{"Matrix"}, "speed", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 5)
			{
				THROW_FILE_LINE("input data error");
			}
			Speed z;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.w_per);
			std::copy(temp + 1, temp + 2, &z.v_tcp);
			std::copy(temp + 2, temp + 3, &z.w_tcp);
			std::copy(temp + 3, temp + 4, &z.w_ext);
			std::copy(temp + 4, temp + 5, &z.v_ext);

			return z;

		});
	cal.addBinaryOperatorFunction("=", "speed", "Matrix", "speed", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 5)
			{
				THROW_FILE_LINE("input data error");
			}
			Speed z;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 1, &z.w_per);
			std::copy(temp + 1, temp + 2, &z.v_tcp);
			std::copy(temp + 2, temp + 3, &z.w_tcp);
			std::copy(temp + 3, temp + 4, &z.w_ext);
			std::copy(temp + 4, temp + 5, &z.v_ext);

			left = z;
			return left;
		});
	// add velocity variables
	
	cal.addVariable("v5", "speed", Speed({ 0.005, 0.005, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v10", "speed", Speed({ 0.01, 0.01, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v25", "speed", Speed({ 0.025, 0.025, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v30", "speed", Speed({ 0.03, 0.03, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v40", "speed", Speed({ 0.04, 0.04, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v50", "speed", Speed({ 0.05, 0.05, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v60", "speed", Speed({ 0.06, 0.06, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v80", "speed", Speed({ 0.08, 0.08, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v100", "speed", Speed({ 0.1, 0.1, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v150", "speed", Speed({ 0.15, 0.15, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v200", "speed", Speed({ 0.2, 0.2, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v300", "speed", Speed({ 0.3, 0.3, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v400", "speed", Speed({ 0.4, 0.4, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v500", "speed", Speed({ 0.5, 0.5, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v600", "speed", Speed({ 0.6, 0.6, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v800", "speed", Speed({ 0.8, 0.8, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v1000", "speed", Speed({ 1.0, 1.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v1500", "speed", Speed({ 1.0, 1.5, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v2000", "speed", Speed({ 1.0, 2.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v3000", "speed", Speed({ 1.0, 3.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v4000", "speed", Speed({ 1.0, 4.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v5000", "speed", Speed({ 1.0, 5.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v6000", "speed", Speed({ 1.0, 6.0, 200 * PI / 180, 0.0, 0.0 }));
	

	cal.addTypename("tool");
	cal.addFunction("tool", std::vector<std::string>{"Matrix"}, "tool", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addBinaryOperatorFunction("=", "tool", "Matrix", "tool", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});
	// add tool offset, inertia variables
	//cal.addVariable("tool0_axis_home", "tool", core::Matrix(1, 6, 0.0));
	//for (int i = 1; i < 17; ++i)
	//{
	//	cal.addVariable("tool" + std::to_string(i) + "_axis_offset", "tool", core::Matrix(1, 16, 0.0));
	//	//cal.addVariable("tool" + std::to_string(i) + "_inertia", "tool", core::Matrix(1, 10, 0.0));
	//}


	cal.addTypename("wobj");
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
		});


	cal.addTypename("zero_comp");
	cal.addFunction("zero_comp", std::vector<std::string>{"Matrix"}, "wobj", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addBinaryOperatorFunction("=", "zero_comp", "Matrix", "zero_comp", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});

}
	auto updateStateRt(codeit::core::Msg& msg)->void {}
	auto createDefaultData(codeit::model::Model& model)->void {}
}
