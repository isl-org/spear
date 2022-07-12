#pragma once

#include <vector>

#include <CoreMinimal.h>

class SCENEMANAGER_API CustomRenderingManager
{
public:
	/**
     * @brief achieve lambertian rendering by disable metallic, roughness and specular parts in asset material
     *
     * @param actors - actors in the world that like to change rendering mode
     * @param enable - enable lambert rendering
     */
	static void setLambertianRendering(std::vector<AActor*>& actors, bool enable);
};
