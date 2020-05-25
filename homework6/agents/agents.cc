// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework6/agents/agents.h"

//#include "homework6/agents/sample/sample_agent.h"

#include "homework6/agents/vz/vz_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
//static simulation::Registrar<::sample::SampleVehicleAgent> registrar("sample_agent");
static simulation::Registrar<::vz::vzVehicleAgent> registrar("vz_agent");
