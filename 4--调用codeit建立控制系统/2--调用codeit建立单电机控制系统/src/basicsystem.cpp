#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	auto createModel(std::string name)->std::unique_ptr<codeit::model::Model>
	{
		std::unique_ptr<codeit::model::Model> model = std::make_unique<codeit::model::Model>(name);

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", "Matrix", codeit::core::Matrix(PI));
		// 定义关节的位置，以及轴线，包含1个转动副，轴线是Z轴
		const double joint1_position[3]{ 0 , 0 , 0 };
		const double joint1_axis[3]{ 0 , 0 , 1 };
		// 定义杆件的位置与321欧拉角，以及10维的惯量向量
		// inertia_vector的定义为：[m, m*x, m*y, m*z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]，其中x,y,z为质心位置
		const double link1_position_and_euler321[6]{ 0 , 0 , 0 , 0 , 0 , 0 };
		const double link1_inertia_vector[10]{ 0 , 0 , 0 , 0 , 0 , 0, 0 , 0, 0, 0 };
		// 定义末端位置与321欧拉角，这个位置为机构起始时的位置
		const double end_effector_position_and_euler321[6]{ 0 , 0, 0 , 0 , 0 , 0 };
		// 添加杆件，这里pe的意思为position and euler angle，函数的参数指定了位姿以及惯性向量
		auto& link1 = model->addPartByPe(link1_position_and_euler321, "321", link1_inertia_vector);
		// 添加关节，添加转动关节，前两个参数为关节连接的杆件，后两个参数定义了关节的位置与轴线
		auto& joint1 = model->addRevoluteJoint(link1, model->ground(), joint1_position, joint1_axis);
		// 添加驱动，驱动位于关节上
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
	
	/*函数返回的是一个类指针，指针指向controller的类型是智能指针
	std::unique_ptr*/
	auto createController()->std::unique_ptr<codeit::controller::Controller>
	{
		/*创建std::unique_ptr实例*/
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);
		/*定义Ethercat控制器的xmal文件
			phy_id指电机的序号，由两个电机，此处先完成0号电机的xml配置
			product_code、vendor_id等数据由控制器读取
			min_pos、max_pos与电缸 的行程有关，前者小于电缸的最小位置0mm，后
				 者大于电缸的最大形成100mm
			max_vel根据电缸的额定转速和行程来计算，即
				 3000（转速）*4（导程）/60/1000=0.2（单位m/s）
			min_vel与max_vel大小相等，方向相反
			max_acc按照经验定义为速度的10倍
			max_pos_following_error、max_vel_following_error由经验数据确定
			home_pos指电缸的初始位置，定义为0
			pos_factor指电缸在推进1米的情况下，控制器的电信号个数，通过查询电机
				 为23bit，则电机转动一圈的情况下电脉冲的次数是2^23=8388608个，电缸推
				 进一米需要转动250圈，则pos_factor=8388608*250=2097152000
			pos_offset是指电机在断电重启后电机的初始位置距离0点的偏差*/
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


	//*************************创建异常信息池************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>
	{
		std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<codeit::system::ErrorInfo>);

		/////慎用冒号，它要放在句尾，后面跟数字代表轴号,后面的内容都会在多语言中显示出来

		////****对errorinfo本身的异常报错
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unknown errorinfo", -2, "ERROR", "该异常信息未注册", "unregistered errorinfo");
		///****对errorinfo本身的异常报错--------------------------------------




		///****关节参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input pos beyond range-joint", -2, "ERROR", "输入位置超限", "input position beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input vel beyond range-joint", -2, "ERROR", "输入速度超限", "input velocity beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input acc beyond range-joint", -2, "ERROR", "输入加速度超限", "input acc beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input dec beyond range-joint", -2, "ERROR", "输入减速度超限", "input dec beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-joint", -2, "ERROR", "加速度维数不匹配", "acc dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec dimension mismatch-joint", -2, "ERROR", "减速度维数不匹配", "dec dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk dimension mismatch-joint", -2, "ERROR", "加加速度维数不匹配", "jerk dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pos dimension mismatch-joint", -2, "ERROR", "位置维数不匹配", "pos dimension mismatch-joint");
		///****关节参数-----------------------------------------------


		///****目标位置
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-robottarget", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-robottarget", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-robottarget", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-robottarget", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-robottarget", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-robottarget", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-robottarget", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-robottarget", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-robottarget", -2, "ERROR", "输入的关节角度维数不匹配", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-robottarget", -2, "ERROR", "运动学正解失败", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-robottarget", -2, "ERROR", "robottarget参数不存在", "robottarget param not exist");
		///****目标位置----------------------------------------------

		///****中间位置
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-midrobottarget", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-midrobottarget", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-midrobottarget", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-midrobottarget", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-midrobottarget", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-midrobottarget", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-midrobottarget", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-midrobottarget", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-midrobottarget", -2, "ERROR", "输入的关节角度维数不匹配", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-midrobottarget", -2, "ERROR", "运动学正解失败", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-midrobottarget", -2, "ERROR", "robottarget参数不存在", "robottarget param not exist");
		///****中间位置-----------------------------------------------

		///****工具坐标系
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-tool", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-tool", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-tool", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-tool", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-tool", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-tool", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-tool", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-tool", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-tool", -2, "ERROR", "tool参数不存在", "tool param not exist");

		///****工具坐标系---------------------------------------------


		///****Wobj坐标系
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-wobj", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-wobj", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-wobj", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-wobj", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-wobj", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-wobj", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-wobj", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-wobj", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-wobj", -2, "ERROR", "wobj参数不存在", "wobj param not exist");

		///****Wobj坐标系--------------------------------------------

		///****zone参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-zone", -2, "ERROR", "zone参数不存在", "zone param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("out of range-zone", -2, "ERROR", "zone参数越界", "zone out of range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-zone", -2, "ERROR", "zone参数维数不匹配", "zone dimension mismatch");
		///****zone参数-----------------------------------------------

		///****speed参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-speed", -2, "ERROR", "速度参数不存在", "speed param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-speed", -2, "ERROR", "速度参数维数不匹配", "speed dimension mismatch");
		///****speed参数---------------------------------------------

		///****load参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-load", -2, "ERROR", "load参数不存在", "load param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-load", -2, "ERROR", "load参数维数不匹配", "load dimension mismatch");
		///****load参数----------------------------------------------


		///****jointtarget参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-jointtarget", -2, "ERROR", "关节目标参数不存在", "jointtarget param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-jointtarget", -2, "ERROR", "关节目标参数维数不匹配", "jointtarget dimension mismatch");
		///****jointarget参数--------------------------------------

		///****速度、加速度、减速度、加加速度参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc is negative", -2, "ERROR", "加速度为负数", "acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec is positive", -2, "ERROR", "减速度为正数", "dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk is negative", -2, "ERROR", "加加速度为负数", "jerk is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmax is negative", -2, "ERROR", "加加速度为负数", "jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmin is positive", -2, "ERROR", "减减速度为正数", "jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmax is negative", -2, "ERROR", "角加加速度为负数", "angular_jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmin is positive", -2, "ERROR", "角减减速度为正数", "angular_jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_acc is negative", -2, "ERROR", "角加速度为负数", "angular_acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_dec is positive", -2, "ERROR", "角减速度为正数", "angular_dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jerk is negative", -2, "ERROR", "角加加速度为负数", "angular_jerk is negative");
		///****速度、加速度、减速度、加加速度参数------------------------------------------


		///****示教点参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-teachpoint", -2, "ERROR", "该示教点名称已存在", "param already exist-teachpoint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-teachpoint", -2, "ERROR", "该示教点名称不存在", "param not exist-teachpoint");
		///****示教点参数------------------------------------------

		///****变量操作参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-definevar", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-definevar", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definespeed", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definezone", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-defineload", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-defineload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definejointtarget", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definejointtarget");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-definevar", -2, "ERROR", "类型错误", "type error-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
		///****变量操作参数------------------------------------------

		///****参数操作参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-saveparam", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-saveparam", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savespeed", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-savespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savezone", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-savezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-saveload", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-saveload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-saveparam", -2, "ERROR", "该示教点名称已存在", "param already exist-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-saveparam", -2, "ERROR", "类型错误", "type error-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-deleteparam", -2, "ERROR", "该示教点名称已存在", "param not exist-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-deleteparam", -2, "ERROR", "类型错误", "type error-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-renameparam", -2, "ERROR", "该示教点名称已存在", "param not exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("new param already exist-renameparam", -2, "ERROR", "该示教点名称已存在", "new param already exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-renameparam", -2, "ERROR", "类型错误", "type error-renameparam");
		///****参数操作参数------------------------------------------

		///****JogJ、JogC参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("last_count is negative", -2, "ERROR", "持续时间为负数", "last_count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("error direction", -2, "ERROR", "jog方向设置错误", "error direction");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint beyond range", -2, "ERROR", "关节选择超限", "joint beyond range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("coordinate error", -2, "ERROR", "坐标系设定错误", "coordinate error");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("motion type error", -2, "ERROR", "运动类型设定错误", "motion type error");
		///****JogJ、JogC参数--------------------------------------

		///****ServoJ参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "时间间隔设为负数", "time is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-servoJ", -2, "ERROR", "加速度维数不匹配", "acc dimension mismatch-servoJ");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("look_ahead_time is negative", -2, "ERROR", "前瞻时间设为负数", "look_ahead_time is negative");
		///****ServoJ参数--------------------------------------


		///****IO 指令参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DO name not exist", -2, "ERROR", "do 不存在", "DO name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DI name not exist", -2, "ERROR", "di 不存在", "DI name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("model name not exist", -2, "ERROR", "模型不存在", "model name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("do_value dimension mismatch do_name", -2, "ERROR", "数据长度不匹配", "do_value dimension mismatch do_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cmd dimension mismatch model_name", -2, "ERROR", "cmd长度不匹配", "cmd dimension mismatch model_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no subsys", -2, "ERROR", "没用subsys", "no subsys");

		///****IO 指令参数--------------------------------------


		///****MoveS,Calibrator参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq set dimension mismatch", -2, "ERROR", "pq参数集维数不匹配", "pq set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm set dimension mismatch", -2, "ERROR", "pm参数集维数不匹配", "pm set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa set dimension mismatch", -2, "ERROR", "pa参数集维数不匹配", "pa set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe set dimension mismatch", -2, "ERROR", "pe参数集维数不匹配", "pe set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint set dimension mismatch", -2, "ERROR", "关节角度参数集维数不匹配", "joint set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic pm failed", -2, "ERROR", "正向运动学获取pm失败", "forward kinematic pm failed");

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("regressor matrix not full rank", -2, "ERROR", "回归矩阵不满秩", "regressor matrix not full rank");

		///****MoveS,Calibrator参数--------------------------------------


		///****其他指令参数
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("count is negative", -2, "ERROR", "持续时间为负数", "count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("program_rate", -2, "ERROR", "程序速率为负数", "program_rate");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cs is running", -2, "ERROR", "实时线程在运行中", "cs is running, please stop the cs using cs_stop!");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to stop server", -2, "ERROR", "停止服务器失败", "failed to stop server, because it is not running");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to start server", -2, "ERROR", "启动服务器失败", "failed to start server, because it is already started");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no model", -2, "ERROR", "该模型不存在", "no model");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "等待时间为负数", "wait time is negative");

		///****其他指令参数--------------------------------------

		///****指令解析异常，源于command.cpp
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("brace not pair", -2, "ERROR", "没有成对的括号", "brace not pair");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("empty command string", -2, "ERROR", "字符串命令为空", "invalid command string: please at least contain a word");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("invalid command name", -2, "ERROR", "无效指令", "server does not have this command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param should not start with '='", -2, "ERROR", "参数不能以=开始", "param should not start with '='");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '-' must be followed by an abbreviation of param", -2, "ERROR", "符号'-'后必须用缩写", "symbol '-' must be followed by an abbreviation of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '--' must be followed by a full name of param", -2, "ERROR", "符号'--'后必须用完整名称", "symbol '--' must be followed by a full name of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param start with single '-' must be an abbreviation", -2, "ERROR", "以'-'开头的参数必须是缩写", "param start with single '-' must be an abbreviation");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a abbreviation of any valid param", -2, "ERROR", "不存在这样的缩写", "not a abbreviation of any valid param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a valid param", -2, "ERROR", "不是一个有效的指令参数", "not a valid param of command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to find default param in command", -2, "ERROR", "指令中找不到默认参数", "failed to find default param in command");


		///****指令解析异常，源于command.cpp--------------------------------------

		///****// step 3.  execute //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server in error", -2, "ERROR", "服务器处于错误状态", "server in error, use cl to clear");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server not started", -2, "ERROR", "服务器未启动", "server not started, use cs_start to start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("command pool is full", -2, "ERROR", "指令缓冲满了", "command pool is full");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to get current TARGET", -2, "ERROR", "获取当前目标失败", "failed to get current TARGET, because ControlServer is not running");
		///****// step 3.  execute //--------------------------------------

		///****// Motion Check //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion is not in OPERATION_ENABLE mode", -2, "ERROR", "轴没有使能", "Motion is not in OPERATION_ENABLE mode");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MAX", -2, "ERROR", "轴的位置指令超过最大值", "Motion target position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MIN", -2, "ERROR", "轴的位置指令超过最小值", "Motion target position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT CONTINUOUS", -2, "ERROR", "轴的位置指令一阶不连续", "Motion target position NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT SECOND CONTINUOUS", -2, "ERROR", "轴的位置指令二阶不连续", "Motion target position NOT SECOND CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position has FOLLOW ERROR", -2, "ERROR", "轴存在位置跟踪误差", "Motion target position has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MAX", -2, "ERROR", "轴的速度指令超过最大值", "Motion target velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MIN", -2, "ERROR", "轴的速度指令超过最小值", "Motion target velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity NOT CONTINUOUS", -2, "ERROR", "轴的速度指令一阶不连续", "Motion target velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity has FOLLOW ERROR", -2, "ERROR", "轴的速度指令存在跟踪误差", "Motion target velocity has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MAX", -2, "ERROR", "轴的实际位置超过最大值", "Motion actual position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MIN", -2, "ERROR", "轴的实际位置超过最小值", "Motion actual position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MAX", -2, "ERROR", "轴的实际速度超过最大值", "Motion actual velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MIN", -2, "ERROR", "轴的实际速度超过最小值", "Motion actual velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity NOT CONTINUOUS", -2, "ERROR", "轴的实际速度一阶不连续", "Motion actual velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion MODE INVALID", -2, "ERROR", "轴的模式无效", "Motion MODE INVALID");
		///****// Motion Check //--------------------------------------


		///****// Master通信 //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master Lost Connection with", -2, "ERROR", "主站失去连接", "Master Lost Connection with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master failed start with", -2, "ERROR", "主站启动失败", "Master failed start with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master already running", -2, "ERROR", "主站已经启动", "master already running, so cannot start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master is not running", -2, "ERROR", "主站已经启动", "master is not running, so can't stop");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "主站已经启动", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "主站已经启动", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("phy id already exists", -2, "ERROR", "主站已经启动", "phy id already exists");


		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_create failed", -2, "ERROR", "实时任务创建失败", "rt_task_create failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_rt_task_join failed", -2, "ERROR", "实时任务创建失败", "codeit_rt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_nrt_task_join failed", -2, "ERROR", "实时任务创建失败", "codeit_nrt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_start failed", -2, "ERROR", "实时任务启动失败", "rt_task_start failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com controller initport fail", -2, "ERROR", "串口控制器端口初始化失败", "Com controller initport fail");
		///****// Master通信错误 //--------------------------------------


		///****// Interface错误 //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't WSAstartup", -2, "ERROR", "Socket不能作为主站启动", "Socket can't Start as server, because it can't WSAstartup");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket has empty port", -2, "ERROR", "Socket端口号为空", "Socket has empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't bind", -2, "ERROR", "Socket端口号为空", "Socket can't Start as server, because it can't bind");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't listen", -2, "ERROR", "Socket端口号为空", "Socket can't Start as server, because it can't listen");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty ip address", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it empty ip address");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty port", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket is busy now", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it is busy now, please close it");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't connect", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because can't connect");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt TCP_USER_TIMEOUT FAILED", -2, "ERROR", "主站已经启动", "socket setsockopt TCP_USER_TIMEOUT FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt SO_KEEPALIVE FAILED", -2, "ERROR", "主站已经启动", "socket setsockopt SO_KEEPALIVE FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("setsockopt failed", -2, "ERROR", "主站已经启动", "setsockopt failed: SO_REUSEADDR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface open listen thread fail", -2, "ERROR", "主站已经启动", "Com interface open listen thread fail");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface initPort fail", -2, "ERROR", "主站已经启动", "Com interface initPort fail");

		///****// Interface错误 //--------------------------------------


		///****// executerRT(),运动算法错误，需加入errMsgMap //
		auto& cs = ControlSystem::instance();
		auto& errMap = cs.errorMap();
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unregistered key", -1000, "ERROR", "未注册的键值", "unregistered key");
		errMap.insert(pair<std::int32_t, string>(-1000, "unregistered key"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("plan over time", codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "ERROR", "规划超时", "plan over time");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "plan over time"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed", codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "ERROR", "正解失败", "forward kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "forward kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("inverse kinematic failed", codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "ERROR", "逆解失败", "inverse kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "inverse kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("wrist singularity", codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "ERROR", "腕部奇异点", "wrist singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "wrist singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("shoulder singularity", codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "ERROR", "肩部奇异点", "shoulder singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "shoulder singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("elbow singularity", codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "ERROR", "肘部奇异点", "elbow singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "elbow singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("three points collinear", codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "ERROR", "三点共线", "three points collinear");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "three points collinear"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveJ planner", codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "ERROR", "没有moveJ规划器", "no moveJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "no moveJ planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveL planner", codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "ERROR", "没有moveL规划器", "no moveL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "no moveL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveC planner", codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "ERROR", "没有moveC规划器", "no moveC planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "no moveC planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveS planner", codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "ERROR", "没有moveS规划器", "no moveS planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "no moveS planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveLL planner", codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "ERROR", "没有moveLL规划器", "no moveLL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "no moveLL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no servoJ planner", codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "ERROR", "没有servoJ规划器", "no servoJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "no servoJ planner"));

		///****// executerRT(),运动算法错误 //--------------------------------------

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
