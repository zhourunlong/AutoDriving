// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/agents/agents.h"

#include "pnc/agents/vz/vz_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::vz::vzVehicleAgent> registrar("vz_agent");
