#include "debug.h"
#include "services.h"

#include "management/commandLine.h"

// module framework
#include "modules/motion/motion.h"

// representations
#include "representations/motion/motorPositionRequest.h"
#include "representations/hardware/motorAngles.h"

#include "hardware/robot/robotDescription.h"
#include "hardware/robot/robotModel.h"


/*------------------------------------------------------------------------------------------------*/

BEGIN_DECLARE_MODULE(CarMotionTest)
	REQUIRE(MotorAngles)
	PROVIDE(MotorPositionRequest)
END_DECLARE_MODULE(CarMotionTest)

class CarMotionTest : public CarMotionTestBase {
private:

public:
	CarMotionTest() {
	}

	virtual void init() {
	}

	virtual void execute() {
		RobotDescription const& robotDescription = *services.getRobotModel().getRobotDescription();
		MotorID id0 = robotDescription.getEffectorID("wheel0");
		MotorID id1 = robotDescription.getEffectorID("wheel1");
		MotorID id2 = robotDescription.getEffectorID("wheel2");
		MotorID id3 = robotDescription.getEffectorID("wheel3");

		MotorID steerL = robotDescription.getEffectorID("steeringL");
		MotorID steerR = robotDescription.getEffectorID("steeringR");

		getMotorPositionRequest().setSpeed(id0, 20. * rounds_per_minute);
		getMotorPositionRequest().setSpeed(id1, 20. * rounds_per_minute);
		getMotorPositionRequest().setSpeed(id2, 20. * rounds_per_minute);
		getMotorPositionRequest().setSpeed(id3, 20. * rounds_per_minute);

		getMotorPositionRequest().setPositionAndSpeed(steerL, 10 * sin(getCurrentTime().value() * degrees / 10) * degrees, 10 * rounds_per_minute);
		getMotorPositionRequest().setPositionAndSpeed(steerR, 10 * sin(getCurrentTime().value() * degrees / 10) * degrees, 10 * rounds_per_minute);
	}
};



class CarTestCmdLineCallback : public CommandLineInterface {
public:
	virtual bool commandLineCallback(const CommandLine &cmdLine) {
		// enable the test modules
		services.getModuleManagers().get<Motion>()->setModuleEnabled("CarMotionTest", true);

		// run managers and wait for termination - comment this line if you want to quit right away
		services.runManagers();

		return true;
	}
};

/*------------------------------------------------------------------------------------------------*/

REGISTER_MODULE(Motion,    CarMotionTest,    false, "Test module to show a car (see demo_car command)")

namespace {
	auto cmdTest = CommandLine::getInstance().registerCommand<CarTestCmdLineCallback>(
			"demo_car",
			"KinematicEngine Demo: Robot Car Test(set robot.description=car.xml and robot.model=ode)",
			ModuleManagers::none()->enable<Motion>() );
}
