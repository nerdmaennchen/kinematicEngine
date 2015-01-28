#include "debug.h"
#include "services.h"

#include "management/commandLine.h"

// module framework
#include "modules/motion/motion.h"

// representations
#include "representations/motion/motorPositionRequest.h"
#include "representations/hardware/motorAngles.h"
#include "representations/motion/kinematicTree.h"

#include "hardware/robot/robotDescription.h"
#include "hardware/robot/robotModel.h"

#include <sstream>
#include <iostream>



/*------------------------------------------------------------------------------------------------*/

BEGIN_DECLARE_MODULE(QuadcopterTest)
	REQUIRE(KinematicTree)
	REQUIRE(MotorAngles)
	PROVIDE(MotorPositionRequest)
END_DECLARE_MODULE(QuadcopterTest)

class QuadcopterTest : public QuadcopterTestBase {
private:

public:
	QuadcopterTest() {
	}

	virtual void init() {
	}

	virtual void execute() {
		KinematicMass robotMass = getKinematicTree().getCOM(getKinematicTree().getRootNode()->getID());

		RobotDescription const& robotDescription = *services.getRobotModel().getRobotDescription();
		MotorID id0 = robotDescription.getEffectorID("propeller0");
		MotorID id1 = robotDescription.getEffectorID("propeller1");
		MotorID id2 = robotDescription.getEffectorID("propeller2");
		MotorID id3 = robotDescription.getEffectorID("propeller3");

		getMotorPositionRequest().setSpeed(id0, robotMass.m_massGrams * rounds_per_minute * 9.1 / 4);
		getMotorPositionRequest().setSpeed(id1, robotMass.m_massGrams * rounds_per_minute * 11 / 4);
		getMotorPositionRequest().setSpeed(id2, robotMass.m_massGrams * rounds_per_minute * 9 / 4);
		getMotorPositionRequest().setSpeed(id3, robotMass.m_massGrams * rounds_per_minute * 11 / 4);
	}
};



class QuadcopterTestCmdLineCallback : public CommandLineInterface {
public:
	virtual bool commandLineCallback(const CommandLine &cmdLine) {
		// enable the test modules
		services.getModuleManagers().get<Motion>()->setModuleEnabled("QuadcopterTest", true);

		// run managers and wait for termination - comment this line if you want to quit right away
		services.runManagers();

		return true;
	}
};

/*------------------------------------------------------------------------------------------------*/

REGISTER_MODULE(Motion,    QuadcopterTest,    false, "Test module to show a quadcopter (see demo_quadcopter command)")

namespace {
	auto cmdTest = CommandLine::getInstance().registerCommand<QuadcopterTestCmdLineCallback>(
			"demo_quadcopter",
			"KinematicEngine Demo: Quadcopter Test (set robot.description=quadcopter.xml and robotmodel=ode)",
			ModuleManagers::none()->enable<Motion>() );
}
