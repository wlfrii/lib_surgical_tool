#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <vector>
#include <array>
#include "./configspcs/configspc.h"
#include "./define/rt.h"
#include "./define/math_extension.h"


class Instrument;
class Kinematics
{	
public:
	Kinematics(Instrument *ins);
	/**
	 * Actually, add a base circle first.
	 */
	void addFirstStem(const RT& origin, ConfigSpc& q);

	/**
	 * Add a stem of a instrument.
	 */
	void addMoreStem(ConfigSpc& q);

	/**
	 * Add the gripper.
	 * 
	 * Revised. 2019.11.13. Revised the return type 'void' by RT, which represent the end frame of a arm
	 */
	RT addGripper();

private:
	// forward kinematics accumulation
	void accRT();   

	// calulate each stem's pose with respect to world frame
	void execute(ConfigSpc& q);

	// get a single transformation of a stem.
	void getRT(ConfigSpc &q, RT& rt);
	void getRotatedRT(ConfigSpc &q, RT& rt);

	// Addition. 2019.10.8, Wanglf. Revise. 2021.1.7
	RT addCamera();
	RT addGripper(std::string &body0, std::string &body1);

private:
	Instrument*			instrument;
	ConfigSpc*			prevQ;
	RT*					prevRt;
	RT					self;	
};


#endif // KINEMATICS_H