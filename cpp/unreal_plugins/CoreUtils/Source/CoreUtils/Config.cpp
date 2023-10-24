//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/Config.h"

#include <Containers/UnrealString.h> // FString
#include <HAL/Platform.h>            // TEXT
#include <Misc/CommandLine.h>
#include <Misc/Parse.h>

#include "CoreUtils/Unreal.h"
#include "CoreUtils/YamlCpp.h"

void Config::initialize()
{
    FString config_file;

    // if a config file is provided via the command-line, then load it
    if (FParse::Value(FCommandLine::Get(), *Unreal::toFString("config_file="), config_file)) {
        SP_LOG("Found config file via the -config_file command-line argument: ", Unreal::toStdString(config_file));
        s_config_ = YAML::LoadFile(Unreal::toStdString(config_file));
        s_initialized_ = true;
    } else {
        s_initialized_ = false;
    }
}

void Config::terminate()
{
    s_config_.reset();
    s_initialized_ = false;
}
