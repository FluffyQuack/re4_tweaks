// Jolt first: it must be the first Jolt header, and before Windows' headers define their macros
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/ConfigurationString.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include "dllmain.h"

// Ragdolls on death (humanoid enemies, and the player on generic deaths), simulated with Jolt Physics. See RAGDOLL_PLAN.md.
//
// Phase 0: Jolt is set up (allocator, factory, types) and checked with a smoke test: a sphere dropped onto a box in a
// throwaway physics system must come to rest on it.

// ---------------------------------------------------------------------------------------------------------------------
// Jolt's reporting

static void Ragdoll_JoltTrace(const char* fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	spd::log()->info("Jolt: {}", buf);
}

#ifdef JPH_ENABLE_ASSERTS
// Logs and carries on: returning true would hit a breakpoint, which crashes the game without a debugger attached
static bool Ragdoll_JoltAssertFailed(const char* expr, const char* msg, const char* file, JPH::uint line)
{
	spd::log()->error("Jolt assert: {}:{}: ({}) {}", file, line, expr, msg ? msg : "");
	return false;
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Smoke test

namespace RagdollLayers
{
	static constexpr JPH::ObjectLayer World = 0;
	static constexpr JPH::ObjectLayer Ragdoll = 1;
	static constexpr JPH::uint Count = 2;
};

namespace RagdollBPLayers
{
	static const JPH::BroadPhaseLayer World(0);
	static const JPH::BroadPhaseLayer Moving(1);
	static constexpr JPH::uint Count = 2;
};

static bool Ragdoll_SmokeTest()
{
	using namespace JPH;

	ObjectLayerPairFilterTable layerFilter(RagdollLayers::Count);
	layerFilter.EnableCollision(RagdollLayers::World, RagdollLayers::Ragdoll);
	layerFilter.EnableCollision(RagdollLayers::Ragdoll, RagdollLayers::Ragdoll);

	BroadPhaseLayerInterfaceTable bpLayers(RagdollLayers::Count, RagdollBPLayers::Count);
	bpLayers.MapObjectToBroadPhaseLayer(RagdollLayers::World, RagdollBPLayers::World);
	bpLayers.MapObjectToBroadPhaseLayer(RagdollLayers::Ragdoll, RagdollBPLayers::Moving);

	ObjectVsBroadPhaseLayerFilterTable bpFilter(bpLayers, RagdollBPLayers::Count, layerFilter, RagdollLayers::Count);

	TempAllocatorImpl tempAllocator(1024 * 1024);
	JobSystemSingleThreaded jobSystem(cMaxPhysicsJobs);

	PhysicsSystem system;
	system.Init(64, 0, 64, 64, bpLayers, bpFilter, layerFilter);

	BodyInterface& bi = system.GetBodyInterface();

	// 10 x 1 x 10 m floor, top at y = 0
	BodyID floor = bi.CreateAndAddBody(BodyCreationSettings(new BoxShape(Vec3(5.0f, 0.5f, 5.0f)), RVec3(0.0f, -0.5f, 0.0f),
		Quat::sIdentity(), EMotionType::Static, RagdollLayers::World), EActivation::DontActivate);

	// 0.25 m sphere, dropped from 2 m
	const float radius = 0.25f;
	BodyID ball = bi.CreateAndAddBody(BodyCreationSettings(new SphereShape(radius), RVec3(0.0f, 2.0f, 0.0f),
		Quat::sIdentity(), EMotionType::Dynamic, RagdollLayers::Ragdoll), EActivation::Activate);

	// 3 s at 60 Hz
	for (int i = 0; i < 180; i++)
		system.Update(1.0f / 60.0f, 1, &tempAllocator, &jobSystem);

	RVec3 pos = bi.GetCenterOfMassPosition(ball);
	bool asleep = !bi.IsActive(ball);

	bi.RemoveBody(ball);
	bi.DestroyBody(ball);
	bi.RemoveBody(floor);
	bi.DestroyBody(floor);

	// Resting contacts are allowed to sink in by up to the penetration slop (2 cm by default)
	bool ok = fabsf(float(pos.GetY()) - radius) <= system.GetPhysicsSettings().mPenetrationSlop + 0.005f;
	spd::log()->info("{} -> sphere settled at y = {:.4f} (expected {:.4f}), {}: {}", __FUNCTION__, float(pos.GetY()), radius,
		asleep ? "asleep" : "still awake", ok ? "OK" : "FAILED");
	return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

static bool s_joltReady = false;

void re4t::init::Ragdoll()
{
	JPH::RegisterDefaultAllocator();
	JPH::Trace = Ragdoll_JoltTrace;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = Ragdoll_JoltAssertFailed;)

	// Same JPH_* defines here as in the library (Jolt.props)
	if (!JPH::VerifyJoltVersionID())
	{
		spd::log()->error("{} -> Jolt was built with different settings than re4_tweaks, ragdolls disabled", __FUNCTION__);
		return;
	}

	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();

	spd::log()->info("{} -> Jolt Physics {}.{}.{} ({})", __FUNCTION__, JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH,
		JPH::GetConfigurationString());

	s_joltReady = Ragdoll_SmokeTest();
}
