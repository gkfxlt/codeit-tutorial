#ifndef X_FUNC_H_
#define X_FUNC_H_
#include<codeit\function\basefunc.hpp>
namespace codeit::function {
	class MoveNrt:public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index = 0)->int { return 1; }
		auto virtual executeNRT(BasisFunc&, int index = 0)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MoveNrt();
		explicit MoveNrt(const std::string& name = "myplan");
		CODEIT_REGISTER_TYPE(MoveNrt);
		CODEIT_DECLARE_BIG_FOUR(MoveNrt);
	};
	
}

#endif
