#ifndef UI_CYL_COLORS_H
#define UI_CYL_COLORS_H

#include "imgui.h"
#include "physics/engine_model.h"

/* One colour per cylinder, kept apart from the amber / red used for limits.
 * Explicit so a cylinder has the same colour in every plot */
static const ImVec4 CYL_COLORS[ENGINE_MAX_CYLINDERS] = {
    ImVec4(0.35f, 0.65f, 0.95f, 1.f), ImVec4(0.35f, 0.80f, 0.45f, 1.f),
    ImVec4(0.85f, 0.45f, 0.75f, 1.f), ImVec4(0.55f, 0.85f, 0.90f, 1.f),
    ImVec4(0.72f, 0.62f, 0.95f, 1.f), ImVec4(0.85f, 0.85f, 0.85f, 1.f),
};

#endif /* UI_CYL_COLORS_H */
