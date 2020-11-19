#ifndef X_FUNC_H_
#define X_FUNC_H_
#include<codeit\function\basefunc.hpp>
namespace codeit::function {
	class MoveCollect:public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index = 0)->int { return 0; }
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MoveCollect();
		explicit MoveCollect(const std::string& name = "myplan");
		CODEIT_REGISTER_TYPE(MoveCollect);
		CODEIT_DECLARE_BIG_FOUR(MoveCollect);
	};
	
}

#endif
