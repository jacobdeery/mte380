#include "logging.h"

using namespace mte;

int main() {
    InitializeLogging("main");

    LOG_INFO("Initializing autonomy stack...");

    while (true) {
        /* Fetch sensor data
        const auto new_sensor_frame = sensing::GetSensorData();
        */

        /* Localize
        const auto pose = localizer.Localize(new_sensor_frame);
        */

        /* Plan
        const auto new_plan = planning::PlanTrajectory(pose);
        */

        /* Update controls
        const auto actuator_states = control::GetActuatorStates();
        controller.StepControlLoop(actuator_states, pose);
        const auto updated_commands = controller.GetUpdatedActuatorCommands(new_plan);
        */
    }
}
