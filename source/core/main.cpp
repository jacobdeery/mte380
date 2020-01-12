#include <iostream>

#include "logging.h"

using namespace mte;

int main() {
    InitializeLogging("main");

    LOG_INFO("Initializing autonomy stack...");

    while (true) {
        // Fetch sensor data

        // Localize

        // Plan

        // Update controls
    }
}
