// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class RobotProjectTarget : TargetRules
{
    public RobotProjectTarget( TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.AddRange( new string[] { "RobotProject" } );

        if (Target.Platform == UnrealTargetPlatform.Win64) {
            bBuildAdditionalConsoleApp = true;
        }
    }
}
