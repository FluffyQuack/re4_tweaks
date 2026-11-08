#pragma once
#include <vector>
#include "SDK/basic_types.h"

class cModel;

// Ragdolls (see RAGDOLL_PLAN.md). Everything here runs on the game thread.
namespace ragdoll
{
	// A model with the humanoid skeleton this mod can ragdoll (the Ganado family or the player)
	bool Supported(cModel* model);

	// Turns a model into a ragdoll at its next pose, starting from that pose and its current motion, plus `push` (world,
	// m/s). False if it isn't supported or Jolt isn't available.
	bool Start(cModel* model, const Vec& push);

	void Stop(cModel* model);
	void StopAll();

	// Ragdolling, or waiting for its next pose to start
	bool IsActive(cModel* model);
	int ActiveCount();

	// Static and moving collision pieces taken from the room (SatMgr)
	int WorldBodyCount();

	// Every ragdoll body's capsule, in game units, for drawing
	struct DebugCapsule
	{
		Vec a, b;
		float radius;
		bool awake;
	};
	void GetDebugCapsules(std::vector<DebugCapsule>& out);
}
