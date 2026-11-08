#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include "dllmain.h"
#include "Game.h"
#include "Utils.h"
#include "Ragdoll.h"
#include "imgui.h"

// The "Parts (ragdoll)" section in the Em Manager.
//
// For the selected model it lists every part (index, parent, length to the parent, local position, world position), can
// draw the skeleton and the part indices over the game, highlight one part, and copy or log the whole list.
//
// It can also turn the selected model into a ragdoll on the spot (with a push away from the player, or backwards for the
// player), and draw every ragdoll's bodies.

namespace
{
	bool s_drawSkeleton = true;
	bool s_drawIndices = true;
	int s_highlight = -1;
	bool s_drawBodies = true;
	float s_push = 3.0f; // m/s

	struct PartRow
	{
		cParts* part;
		int parent; // index, -1 = the model itself (a root part), -2 = something else
		float length;
	};

	std::vector<PartRow> CollectParts(cModel* model)
	{
		std::vector<PartRow> rows;
		for (cParts* p = model->childParts_F4; p; p = p->nextParts_F4)
			rows.push_back({ p, -2, 0.0f });

		for (auto& r : rows)
		{
			cCoord* parent = r.part->pParent_6C;
			if (parent == model)
				r.parent = -1;
			for (size_t i = 0; i < rows.size(); i++)
			{
				if (rows[i].part == parent)
				{
					r.parent = int(i);
					break;
				}
			}

			const Vec& a = r.part->world_70;
			const Vec& b = parent ? parent->world_70 : a;
			r.length = sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
		}
		return rows;
	}

	std::string ParentStr(int parent)
	{
		if (parent == -1)
			return "root";
		if (parent == -2)
			return "?";
		return std::to_string(parent);
	}

	std::string BuildText(cModel* model, const std::vector<PartRow>& rows, float minY, float maxY)
	{
		std::string out;
		char line[256];
		sprintf(line, "%s (id 0x%02X, type 0x%02X), %d parts, root pos (%.1f, %.1f, %.1f), parts y %.1f..%.1f (height %.1f)\n",
			cEmMgr::EmIdToName(model->id_100).c_str(), int(model->id_100), int(model->type_101), int(rows.size()),
			model->pos_94.x, model->pos_94.y, model->pos_94.z, minY, maxY, maxY - minY);
		out += line;
		out += "idx parent  length   local (x, y, z)              world (x, y, z)\n";
		for (size_t i = 0; i < rows.size(); i++)
		{
			const cParts* p = rows[i].part;
			sprintf(line, "%3d %6s %7.1f   (%7.1f, %7.1f, %7.1f)   (%8.1f, %8.1f, %8.1f)\n", int(i), ParentStr(rows[i].parent).c_str(),
				rows[i].length, p->pos_94.x, p->pos_94.y, p->pos_94.z, p->world_70.x, p->world_70.y, p->world_70.z);
			out += line;
		}
		return out;
	}

	bool ToScreen(const Vec& w, ImVec2& out)
	{
		GLOBAL_WK* pG = GlobalPtr();
		ImGuiIO& io = ImGui::GetIO();
		Vec s;
		if (!WorldToScreen(w, s, pG->Camera_74.v_mat_30, pG->Camera_74.CamPoint_A4.Fovy_1C, io.DisplaySize.x, io.DisplaySize.y))
			return false;
		out = ImVec2(s.x, s.y);
		return true;
	}

	// Each ragdoll body: its capsule's axis, with the radius as a circle at each end (green awake, grey asleep)
	void DrawBodies()
	{
		std::vector<ragdoll::DebugCapsule> caps;
		ragdoll::GetDebugCapsules(caps);

		GLOBAL_WK* pG = GlobalPtr();
		ImDrawList* dl = ImGui::GetBackgroundDrawList();
		const Vec camPos = pG->Camera_74.CamPoint_A4.Campos_0;
		const float screenH = ImGui::GetIO().DisplaySize.y;
		const float fovRad = pG->Camera_74.CamPoint_A4.Fovy_1C / 1.3f * 3.14159265f / 180.0f;

		for (const auto& c : caps)
		{
			ImVec2 a, b;
			if (!ToScreen(c.a, a) || !ToScreen(c.b, b))
				continue;

			const ImU32 col = c.awake ? IM_COL32(80, 255, 120, 220) : IM_COL32(170, 170, 170, 200);
			dl->AddLine(a, b, col, 2.0f);

			// Rough on-screen radius at each end
			for (const Vec* w : { &c.a, &c.b })
			{
				Vec d = *w - camPos;
				float dist = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
				if (dist < 1.0f || fovRad <= 0.0f)
					continue;
				float r = c.radius / (dist * tanf(0.5f * fovRad)) * 0.5f * screenH;
				dl->AddCircle(w == &c.a ? a : b, r, col, 16, 1.0f);
			}
		}
	}

	// Away from the player, a little upwards; backwards for the player
	Vec PushFor(cModel* model, float speed)
	{
		Vec dir;
		cModel* player = (cModel*)PlayerPtr();
		if (model == player || !player)
			dir = Vec(-model->mat_C[0][2], 0.0f, -model->mat_C[2][2]);
		else
			dir = Vec(model->pos_94.x - player->pos_94.x, 0.0f, model->pos_94.z - player->pos_94.z);

		float len = sqrtf(dir.x * dir.x + dir.z * dir.z);
		if (len < 1e-3f)
			return Vec(0.0f, 0.2f * speed, 0.0f);
		return Vec(dir.x / len * speed, 0.2f * speed, dir.z / len * speed);
	}

	void DrawOverlay(cModel* model, const std::vector<PartRow>& rows)
	{
		ImDrawList* dl = ImGui::GetBackgroundDrawList();
		auto toScreen = ToScreen;

		const ImU32 boneCol = IM_COL32(80, 220, 255, 200);
		const ImU32 highCol = IM_COL32(255, 220, 0, 255);
		const ImU32 textCol = IM_COL32(255, 255, 255, 230);

		for (size_t i = 0; i < rows.size(); i++)
		{
			const cParts* p = rows[i].part;
			const bool high = int(i) == s_highlight;

			ImVec2 a;
			if (!toScreen(p->world_70, a))
				continue;

			if (s_drawSkeleton && rows[i].parent >= 0)
			{
				ImVec2 b;
				if (toScreen(rows[rows[i].parent].part->world_70, b))
					dl->AddLine(a, b, high ? highCol : boneCol, high ? 3.0f : 1.5f);
			}

			if (high)
				dl->AddCircleFilled(a, 5.0f, highCol);
			else if (s_drawSkeleton)
				dl->AddCircleFilled(a, 2.5f, boneCol);

			if (s_drawIndices && (s_highlight < 0 || high))
			{
				char buf[8];
				sprintf(buf, "%d", int(i));
				dl->AddText(ImVec2(a.x + 4.0f, a.y - 7.0f), high ? highCol : textCol, buf);
			}
		}
	}
}

void RagdollDebug_PartsUI(cModel* model)
{
	if (!model || !model->childParts_F4)
		return;

	if (!ImGui::CollapsingHeader("Parts (ragdoll)"))
		return;

	std::vector<PartRow> rows = CollectParts(model);

	float minY = FLT_MAX, maxY = -FLT_MAX;
	for (auto& r : rows)
	{
		minY = fminf(minY, r.part->world_70.y);
		maxY = fmaxf(maxY, r.part->world_70.y);
	}

	ImGui::Checkbox("Draw skeleton", &s_drawSkeleton);
	ImGui::SameLine();
	ImGui::Checkbox("Draw indices", &s_drawIndices);
	ImGui::SetNextItemWidth(120.0f * esHook._cur_monitor_dpi);
	ImGui::SliderInt("Highlight part (-1: none)", &s_highlight, -1, int(rows.size()) - 1);

	ImGui::Text("%d parts, parts y %.1f..%.1f (height %.1f), root y %.1f", int(rows.size()), minY, maxY, maxY - minY,
		model->pos_94.y);

	// Ragdoll test
	ImGui::Separator();
	if (ragdoll::IsActive(model))
	{
		if (ImGui::Button("Stop ragdoll"))
			ragdoll::Stop(model);
	}
	else if (ragdoll::Supported(model))
	{
		if (ImGui::Button("Ragdoll"))
			ragdoll::Start(model, PushFor(model, s_push));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(120.0f * esHook._cur_monitor_dpi);
		ImGui::SliderFloat("Push (m/s)", &s_push, 0.0f, 15.0f, "%.1f");
	}
	else
		ImGui::TextDisabled("Ragdoll: not supported for this model");

	ImGui::Text("Ragdolls: %d (%d simulated, max %d), room collision pieces: %d", ragdoll::ActiveCount(), ragdoll::SimulatedCount(),
		ragdoll::c_maxSimulated, ragdoll::WorldBodyCount());
	ImGui::SameLine();
	if (ImGui::Button("Stop all"))
		ragdoll::StopAll();
	ImGui::Checkbox("Draw ragdoll bodies", &s_drawBodies);
	ImGui::SameLine();
	ImGui::Checkbox("Enemy death ragdolls", &ragdoll::g_enemyDeaths);
	ImGui::SameLine();
	ImGui::Checkbox("Root follows body", &ragdoll::g_followRoot);
	ImGui::SetNextItemWidth(120.0f * esHook._cur_monitor_dpi);
	ImGui::SliderFloat("Death push scale", &ragdoll::g_deathPushScale, 0.0f, 3.0f, "%.2f");
	ImGui::Separator();

	if (ImGui::Button("Copy list to clipboard"))
		ImGui::SetClipboardText(BuildText(model, rows, minY, maxY).c_str());
	ImGui::SameLine();
	if (ImGui::Button("Write list to log"))
		spd::log()->info("Parts of {}:\n{}", (void*)model, BuildText(model, rows, minY, maxY));

	if (ImGui::BeginTable("##ragdollparts", 5, ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV,
		ImVec2(0.0f, 260.0f * esHook._cur_monitor_dpi)))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn("#");
		ImGui::TableSetupColumn("Parent");
		ImGui::TableSetupColumn("Length");
		ImGui::TableSetupColumn("Local");
		ImGui::TableSetupColumn("World");
		ImGui::TableHeadersRow();

		for (size_t i = 0; i < rows.size(); i++)
		{
			const cParts* p = rows[i].part;
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			char label[16];
			sprintf(label, "%d", int(i));
			if (ImGui::Selectable(label, s_highlight == int(i), ImGuiSelectableFlags_SpanAllColumns))
				s_highlight = s_highlight == int(i) ? -1 : int(i);
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(ParentStr(rows[i].parent).c_str());
			ImGui::TableNextColumn();
			ImGui::Text("%.1f", rows[i].length);
			ImGui::TableNextColumn();
			ImGui::Text("%.1f, %.1f, %.1f", p->pos_94.x, p->pos_94.y, p->pos_94.z);
			ImGui::TableNextColumn();
			ImGui::Text("%.1f, %.1f, %.1f", p->world_70.x, p->world_70.y, p->world_70.z);
		}
		ImGui::EndTable();
	}

	if (s_drawSkeleton || s_drawIndices)
		DrawOverlay(model, rows);
	if (s_drawBodies)
		DrawBodies();
}
