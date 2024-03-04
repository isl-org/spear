//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

using UnrealBuildTool;

public class SpEngine : SpModuleRules
{
    public SpEngine(ReadOnlyTargetRules target) : base(target)
    {
        SP_LOG_CURRENT_FUNCTION();

        PublicDependencyModuleNames.AddRange(new string[] {"SpCore"});
        PrivateDependencyModuleNames.AddRange(new string[] {});
    }
}
