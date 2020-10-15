#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	

	auto createDefaultData(codeit::model::Model& model)->void
{
	double zone[2] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("fine", core::Matrix(1, 2, zone));
	zone[0] = 0.001; zone[1] = 0.01;
	model.variablePool().add<codeit::model::MatrixVariable>("z1", core::Matrix(1, 2, zone));
	zone[0] = 0.005; zone[1] = 0.03;
	model.variablePool().add<codeit::model::MatrixVariable>("z5", core::Matrix(1, 2, zone));
	zone[0] = 0.01; zone[1] = 0.05;
	model.variablePool().add<codeit::model::MatrixVariable>("z10", core::Matrix(1, 2, zone));
	zone[0] = 0.015; zone[1] = 0.08;
	model.variablePool().add<codeit::model::MatrixVariable>("z15", core::Matrix(1, 2, zone));
	zone[0] = 0.02; zone[1] = 0.1;
	model.variablePool().add<codeit::model::MatrixVariable>("z20", core::Matrix(1, 2, zone));
	zone[0] = 0.03; zone[1] = 0.15;
	model.variablePool().add<codeit::model::MatrixVariable>("z30", core::Matrix(1, 2, zone));
	zone[0] = 0.04; zone[1] = 0.2;
	model.variablePool().add<codeit::model::MatrixVariable>("z40", core::Matrix(1, 2, zone));
	zone[0] = 0.05; zone[1] = 0.25;
	model.variablePool().add<codeit::model::MatrixVariable>("z50", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<codeit::model::MatrixVariable>("z60", core::Matrix(1, 2, zone));
	zone[0] = 0.08; zone[1] = 0.4;
	model.variablePool().add<codeit::model::MatrixVariable>("z80", core::Matrix(1, 2, zone));
	zone[0] = 0.1; zone[1] = 0.45;
	model.variablePool().add<codeit::model::MatrixVariable>("z100", core::Matrix(1, 2, zone));
	zone[0] = 0.2; zone[1] = 0.45;
	model.variablePool().add<codeit::model::MatrixVariable>("z150", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<codeit::model::MatrixVariable>("z200", core::Matrix(1, 2, zone));

	double v[5] = { 0 };
	v[0] = 0.005; v[1] = 0.005; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v5", core::Matrix(1, 5, v));
	v[0] = 0.01; v[1] = 0.01; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v10", core::Matrix(1, 5, v));
	v[0] = 0.025; v[1] = 0.025; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v25", core::Matrix(1, 5, v));
	v[0] = 0.03; v[1] = 0.03; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v30", core::Matrix(1, 5, v));
	v[0] = 0.04; v[1] = 0.04; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v40", core::Matrix(1, 5, v));
	v[0] = 0.05; v[1] = 0.05; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v50", core::Matrix(1, 5, v));
	v[0] = 0.06; v[1] = 0.06; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v60", core::Matrix(1, 5, v));
	v[0] = 0.08; v[1] = 0.08; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v80", core::Matrix(1, 5, v));
	v[0] = 0.1; v[1] = 0.1; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v100", core::Matrix(1, 5, v));
	v[0] = 0.15; v[1] = 0.15; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v150", core::Matrix(1, 5, v));
	v[0] = 0.2; v[1] = 0.2; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v200", core::Matrix(1, 5, v));
	v[0] = 0.3; v[1] = 0.3; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v300", core::Matrix(1, 5, v));
	v[0] = 0.4; v[1] = 0.4; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v400", core::Matrix(1, 5, v));
	v[0] = 0.5; v[1] = 0.5; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v500", core::Matrix(1, 5, v));
	v[0] = 0.6; v[1] = 0.6; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v600", core::Matrix(1, 5, v));
	v[0] = 0.8; v[1] = 0.8; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v800", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v1000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.5; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v1500", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 2.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v2000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 3.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v3000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 4.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v4000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 5.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v5000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 6.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v6000", core::Matrix(1, 5, v));


	double load[10] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("load0", core::Matrix(1, 10, load));

	double zero[MAX_DOFS] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("zero_comp0", core::Matrix(1, MAX_DOFS, zero));

}
}
