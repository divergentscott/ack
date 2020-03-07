#include <boost/filesystem.hpp>
#include "iostream"
#include "tetra.h"
#include "bap.h"

class LieutenantGeneral : public General {
private:
	battleplan secret_plan;
public:
	void setPlan(battleplan plan) {
		std::cout << "These plans are secret!\n";
		this->secret_plan = plan;
	};
	battleplan getPlan() {
		std::cout << "Fetch the plans!\n";
		return this->secret_plan;
	};
};

std::unique_ptr<General> appointGeneral(battleplan plan) {
	LieutenantGeneral lt_gen_gump;
	std::unique_ptr<General> gen_dan = std::make_unique<LieutenantGeneral>(lt_gen_gump);
	gen_dan->setPlan(plan);
	return gen_dan;
};
