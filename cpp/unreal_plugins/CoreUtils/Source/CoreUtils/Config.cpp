//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/Config.h"

#include <Misc/CommandLine.h>
#include <Misc/Parse.h>
#include <Misc/Paths.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Unreal.h"
#include "CoreUtils/YamlCpp.h"

bool Config::s_initialized_;
YAML::Node Config::s_config_;

void Config::initialize()
{
    FString config_file;

    // If a config file is provided via the command-line, then load it
    if (FParse::Value(FCommandLine::Get(), TEXT("config_file="), config_file)) {
        s_config_ = YAML::LoadFile(Unreal::toStdString(config_file));
        s_initialized_ = true;

    // Otherwise assert
    } else {
        s_initialized_ = false;
    }
}

void Config::terminate()
{
    s_config_.reset();
}
