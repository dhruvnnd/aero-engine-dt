#include "ui/about_panel.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "imgui.h"
#include "implot.h"
#include "build_info.h" /* generated each build: AERO_GIT_* */
#include "sqlite3.h"
#include "ui/panel_names.h"

/* Set from CMakeLists.txt; fall back so the file also builds standalone. */
#ifndef AERO_VERSION
#define AERO_VERSION "dev"
#endif
#ifndef AERO_BUILD_TYPE
#define AERO_BUILD_TYPE "unknown"
#endif

/* Commit shown with a marker when tracked files differ from HEAD. */
#if AERO_GIT_DIRTY
#define AERO_REVISION AERO_GIT_HASH "-dirty"
#else
#define AERO_REVISION AERO_GIT_HASH
#endif

#define PROJECT_NAME "Aero Engine DT"
#define PROJECT_TAGLINE "Aero engine digital twin - real-time physics dashboard"
#define PROJECT_URL "https://github.com/dhruvnnd/aero-engine-dt"
#define DEVELOPER_NAME "Dhruv Anand"
#define DEVELOPER_URL "https://github.com/dhruvnnd"
#define DEVELOPER_EMAIL "ananddhruv29@gmail.com"

static const ImVec4 kAccent(0.35f, 0.72f, 1.00f, 1.0f);
static const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
static const ImVec4 kWarn(1.00f, 0.75f, 0.20f, 1.0f);

/* ---- small helpers ------------------------------------------------------ */

static void compiler_string(char *out, size_t cap) {
#if defined(__clang__)
  snprintf(out, cap, "Clang %s", __clang_version__);
#elif defined(__GNUC__)
  snprintf(out, cap, "GCC %s%s", __VERSION__,
#ifdef __MINGW64__
           " (MinGW-w64)"
#elif defined(__MINGW32__)
           " (MinGW)"
#else
           ""
#endif
  );
#elif defined(_MSC_VER)
  snprintf(out, cap, "MSVC %d", _MSC_VER);
#else
  snprintf(out, cap, "unknown");
#endif
}

static const char *arch_string() {
#if defined(__x86_64__) || defined(_M_X64)
  return "x86-64";
#elif defined(__aarch64__) || defined(_M_ARM64)
  return "ARM64";
#elif defined(__i386__) || defined(_M_IX86)
  return "x86 (32-bit)";
#else
  return "unknown";
#endif
}

/* Hyperlink-styled text: click opens the URL, right-click copies it. */
static void link_text(const char *label, const char *url) {
  ImGui::PushStyleColor(ImGuiCol_Text, kAccent);
  ImGui::TextUnformatted(label);
  ImGui::PopStyleColor();
  if (ImGui::IsItemHovered()) {
    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    const ImVec2 mn = ImGui::GetItemRectMin();
    const ImVec2 mx = ImGui::GetItemRectMax();
    ImGui::GetWindowDrawList()->AddLine(ImVec2(mn.x, mx.y), mx,
                                        ImGui::GetColorU32(kAccent));
    ImGui::SetTooltip("%s\nclick: open   right-click: copy", url);
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      SDL_OpenURL(url);
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
      ImGui::SetClipboardText(url);
    }
  }
}

static void kv_row(const char *key, const char *fmt, ...) IM_FMTARGS(2);
static void kv_row(const char *key, const char *fmt, ...) {
  ImGui::TableNextRow();
  ImGui::TableNextColumn();
  ImGui::TextDisabled("%s", key);
  ImGui::TableNextColumn();
  va_list args;
  va_start(args, fmt);
  ImGui::TextWrappedV(fmt, args);
  va_end(args);
}

static bool kv_begin(const char *id) {
  if (!ImGui::BeginTable(id, 2,
                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                             ImGuiTableFlags_SizingStretchProp)) {
    return false;
  }
  ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthFixed,
                          ImGui::GetFontSize() * 11.0f);
  ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch);
  return true;
}

static void section(const char *title) {
  ImGui::Spacing();
  ImGui::TextColored(kAccent, "%s", title);
  ImGui::Separator();
}

/* ---- library table ------------------------------------------------------ */

struct LibRow {
  const char *name;
  const char *role;
  char compiled[48]; /* header version the code was built against */
  char linked[48];   /* version reported by the library at runtime */
  const char *license;
  const char *url;
};

static void fill_libs(LibRow *rows, int *count) {
  int n = 0;

  {
    LibRow &r = rows[n++];
    r.name = "SDL3";
    r.role = "windowing, input, timing, renderer";
    snprintf(r.compiled, sizeof r.compiled, "%d.%d.%d", SDL_MAJOR_VERSION,
             SDL_MINOR_VERSION, SDL_MICRO_VERSION);
    const int v = SDL_GetVersion();
    snprintf(r.linked, sizeof r.linked, "%d.%d.%d", SDL_VERSIONNUM_MAJOR(v),
             SDL_VERSIONNUM_MINOR(v), SDL_VERSIONNUM_MICRO(v));
    r.license = "zlib";
    r.url = "https://libsdl.org";
  }
  {
    LibRow &r = rows[n++];
    r.name = "Dear ImGui";
    r.role = "immediate-mode UI, docking";
    snprintf(r.compiled, sizeof r.compiled, "%s (%d)", IMGUI_VERSION,
             IMGUI_VERSION_NUM);
    snprintf(r.linked, sizeof r.linked, "%s", ImGui::GetVersion());
    r.license = "MIT";
    r.url = "https://github.com/ocornut/imgui";
  }
  {
    LibRow &r = rows[n++];
    r.name = "ImGui SDL3 backend";
    r.role = "platform + SDL_Renderer bridge";
    snprintf(r.compiled, sizeof r.compiled, "imgui_impl_sdl3 / sdlrenderer3");
    snprintf(r.linked, sizeof r.linked, "bundled with ImGui");
    r.license = "MIT";
    r.url = "https://github.com/ocornut/imgui/tree/master/backends";
  }
  {
    LibRow &r = rows[n++];
    r.name = "ImPlot";
    r.role = "interactive 2D plotting";
    snprintf(r.compiled, sizeof r.compiled, "%s (%d)", IMPLOT_VERSION,
             IMPLOT_VERSION_NUM);
    snprintf(r.linked, sizeof r.linked, "%s", IMPLOT_VERSION);
    r.license = "MIT";
    r.url = "https://github.com/epezent/implot";
  }
  {
    LibRow &r = rows[n++];
    r.name = "SQLite";
    r.role = "run recording / run log store";
    snprintf(r.compiled, sizeof r.compiled, "%s", SQLITE_VERSION);
    snprintf(r.linked, sizeof r.linked, "%s", sqlite3_libversion());
    r.license = "Public domain";
    r.url = "https://sqlite.org";
  }
  {
    LibRow &r = rows[n++];
    r.name = "C runtime";
    r.role = "libc / libstdc++ (MinGW-w64)";
#ifdef __MINGW64_VERSION_STR
    snprintf(r.compiled, sizeof r.compiled, "mingw-w64 %s",
             __MINGW64_VERSION_STR);
#else
    snprintf(r.compiled, sizeof r.compiled, "system");
#endif
    snprintf(r.linked, sizeof r.linked, "C++%ld / C%ld",
             (long)(__cplusplus / 100L % 100L),
#ifdef __STDC_VERSION__
             (long)(__STDC_VERSION__ / 100L % 100L)
#else
             0L
#endif
    );
    r.license = "ZPL / GPL+exc.";
    r.url = "https://www.mingw-w64.org";
  }
  *count = n;
}

static bool version_mismatch(const LibRow &r) {
  /* Only meaningful for libs whose runtime string is a plain version. */
  if (!strcmp(r.name, "SDL3") || !strcmp(r.name, "SQLite")) {
    return strcmp(r.compiled, r.linked) != 0;
  }
  return false;
}

/* ---- tabs --------------------------------------------------------------- */

static void tab_overview() {
  ImGui::Spacing();
  ImGui::PushFont(NULL, ImGui::GetStyle().FontSizeBase * 1.6f);
  ImGui::TextColored(kAccent, "%s", PROJECT_NAME);
  ImGui::PopFont();
  ImGui::SameLine();
  ImGui::TextDisabled("v%s", AERO_VERSION);
  ImGui::TextDisabled("%s", PROJECT_TAGLINE);

  section("PROJECT");
  if (kv_begin("##ov_project")) {
    kv_row("Version", "%s  (%s build)", AERO_VERSION, AERO_BUILD_TYPE);
    kv_row("Revision", "%s  on  %s", AERO_REVISION, AERO_GIT_BRANCH);
    kv_row("Compiled", "%s  %s", __DATE__, __TIME__);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("Repository");
    ImGui::TableNextColumn();
    link_text(PROJECT_URL, PROJECT_URL);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("Issues");
    ImGui::TableNextColumn();
    link_text(PROJECT_URL "/issues", PROJECT_URL "/issues");
    ImGui::EndTable();
  }

  section("DEVELOPER");
  if (kv_begin("##ov_dev")) {
    kv_row("Author", "%s", DEVELOPER_NAME);
    kv_row("Email", "%s", DEVELOPER_EMAIL);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("GitHub");
    ImGui::TableNextColumn();
    link_text(DEVELOPER_URL, DEVELOPER_URL);
    ImGui::EndTable();
  }

  section("ARCHITECTURE");
  if (kv_begin("##ov_arch")) {
    kv_row("engine_core", "C11 static library: physics, telemetry, model, "
                          "data. No SDL dependency; unit-tested headless.");
    kv_row("aero_engine_dt", "C++17 app shell: SDL3 platform layer + "
                             "ImGui/ImPlot dockable panels.");
    kv_row("tools", "twin_sim (headless CSV runner), twin_config (engine "
                    "spec author/validator).");
    kv_row("Persistence", "SQLite run log; engine specs as .cfg files; "
                          "layouts in ./layouts.");
    ImGui::EndTable();
  }

  section("LICENSES");
  ImGui::TextWrapped(
      "Third-party components are used under their own licenses "
      "(see the Libraries tab). Physics models are simplified and intended "
      "for simulation and education only - not for real flight or "
      "maintenance decisions.");
}

static void tab_libraries() {
  LibRow rows[8];
  int n = 0;
  fill_libs(rows, &n);

  ImGui::Spacing();
  ImGui::TextDisabled("Compiled = header version at build time.  "
                      "Linked = version reported by the loaded library.");
  ImGui::Spacing();

  const ImGuiTableFlags flags =
      ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
      ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp |
      ImGuiTableFlags_ScrollX;
  if (ImGui::BeginTable("##libs", 6, flags)) {
    ImGui::TableSetupColumn("Library");
    ImGui::TableSetupColumn("Role");
    ImGui::TableSetupColumn("Compiled");
    ImGui::TableSetupColumn("Linked");
    ImGui::TableSetupColumn("License");
    ImGui::TableSetupColumn("Source");
    ImGui::TableHeadersRow();
    for (int i = 0; i < n; i++) {
      const LibRow &r = rows[i];
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextColored(kAccent, "%s", r.name);
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.role);
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.compiled);
      ImGui::TableNextColumn();
      if (version_mismatch(r)) {
        ImGui::TextColored(kWarn, "%s", r.linked);
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("Runtime differs from the headers this binary "
                            "was compiled against.");
        }
      } else {
        ImGui::TextColored(kGood, "%s", r.linked);
      }
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(r.license);
      ImGui::TableNextColumn();
      link_text(r.url + (strncmp(r.url, "https://", 8) == 0 ? 8 : 0), r.url);
    }
    ImGui::EndTable();
  }

  section("SDL3 DETAIL");
  if (kv_begin("##sdl")) {
    kv_row("Revision", "%s", SDL_GetRevision());
    kv_row("Platform", "%s", SDL_GetPlatform());
    kv_row("Video driver", "%s", SDL_GetCurrentVideoDriver()
                                     ? SDL_GetCurrentVideoDriver()
                                     : "n/a");
    ImGui::EndTable();
  }

  section("SQLITE DETAIL");
  if (kv_begin("##sqlite")) {
    kv_row("Source id", "%s", sqlite3_sourceid());
    const char *ts[] = {"single-thread", "serialized", "multi-thread"};
    const int t = sqlite3_threadsafe();
    kv_row("Thread safety", "%s (%d)", (t >= 0 && t < 3) ? ts[t] : "?", t);
    char opts[2048];
    size_t len = 0;
    opts[0] = '\0';
    for (int i = 0;; i++) {
      const char *o = sqlite3_compileoption_get(i);
      if (!o) {
        break;
      }
      len += (size_t)snprintf(opts + len, sizeof opts - len, "%s%s",
                              i ? ", " : "", o);
      if (len >= sizeof opts - 1) {
        break;
      }
    }
    kv_row("Compile options", "%s", opts[0] ? opts : "(none)");
    ImGui::EndTable();
  }
}

static void tab_runtime(SDL_Window *window, SDL_Renderer *renderer) {
  const ImGuiIO &io = ImGui::GetIO();

  ImGui::Spacing();
  section("SYSTEM");
  if (kv_begin("##rt_sys")) {
    kv_row("OS / platform", "%s", SDL_GetPlatform());
    kv_row("CPU", "%d logical cores, %d-byte cache line",
           SDL_GetNumLogicalCPUCores(), SDL_GetCPUCacheLineSize());
    kv_row("SIMD", "%s%s%s%s%s%s", SDL_HasSSE2() ? "SSE2 " : "",
           SDL_HasSSE42() ? "SSE4.2 " : "", SDL_HasAVX() ? "AVX " : "",
           SDL_HasAVX2() ? "AVX2 " : "", SDL_HasAVX512F() ? "AVX512F " : "",
           SDL_HasNEON() ? "NEON " : "");
    kv_row("System RAM", "%.1f GiB", SDL_GetSystemRAM() / 1024.0);
    kv_row("Uptime", "%.1f s", SDL_GetTicks() / 1000.0);
    ImGui::EndTable();
  }

  section("DISPLAY");
  if (kv_begin("##rt_disp")) {
    const SDL_DisplayID did = SDL_GetDisplayForWindow(window);
    const char *dname = did ? SDL_GetDisplayName(did) : NULL;
    kv_row("Display", "%s", dname ? dname : "n/a");
    const SDL_DisplayMode *dm = did ? SDL_GetCurrentDisplayMode(did) : NULL;
    if (dm) {
      kv_row("Mode", "%d x %d @ %.2f Hz", dm->w, dm->h, dm->refresh_rate);
    }
    int pw = 0, ph = 0, lw = 0, lh = 0;
    SDL_GetWindowSizeInPixels(window, &pw, &ph);
    SDL_GetWindowSize(window, &lw, &lh);
    kv_row("Window", "%d x %d px  (%d x %d logical)", pw, ph, lw, lh);
    kv_row("Content scale", "%.2fx  (pixel density %.2f)",
           SDL_GetWindowDisplayScale(window), SDL_GetWindowPixelDensity(window));
    const char *rname = SDL_GetRendererName(renderer);
    kv_row("Renderer", "%s", rname ? rname : "n/a");
    int vsync = 0;
    SDL_GetRenderVSync(renderer, &vsync);
    kv_row("VSync", "%s", vsync ? "on" : "off");
    ImGui::EndTable();
  }

  section("UI");
  if (kv_begin("##rt_ui")) {
    kv_row("Frame rate", "%.1f FPS  (%.2f ms/frame)", io.Framerate,
           io.Framerate > 0.0f ? 1000.0f / io.Framerate : 0.0f);
    kv_row("Draw data", "%d vertices, %d indices", io.MetricsRenderVertices,
           io.MetricsRenderIndices);
    kv_row("Windows", "%d visible / %d active", io.MetricsRenderWindows,
           io.MetricsActiveWindows);
    kv_row("Docking", "%s",
           (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) ? "enabled"
                                                             : "disabled");
    int ngp = 0;
    SDL_JoystickID *gps = SDL_GetGamepads(&ngp);
    SDL_free(gps);
    kv_row("Gamepads", "%d connected", ngp);
    ImGui::EndTable();
  }
}

static void tab_build() {
  char cc[160];
  compiler_string(cc, sizeof cc);

  ImGui::Spacing();
  section("TOOLCHAIN");
  if (kv_begin("##bd_tool")) {
    kv_row("Compiler", "%s", cc);
    kv_row("Language", "C++%ld  /  C%ld", (long)(__cplusplus / 100L % 100L),
#ifdef __STDC_VERSION__
           (long)(__STDC_VERSION__ / 100L % 100L)
#else
           0L
#endif
    );
    kv_row("Target", "%s, %u-bit", arch_string(),
           (unsigned)(sizeof(void *) * 8));
    kv_row("Build type", "%s", AERO_BUILD_TYPE);
#ifdef NDEBUG
    kv_row("Assertions", "disabled (NDEBUG)");
#else
    kv_row("Assertions", "enabled");
#endif
    kv_row("Compiled", "%s %s", __DATE__, __TIME__);
    ImGui::EndTable();
  }

  section("SOURCE");
  if (kv_begin("##bd_src")) {
    kv_row("Version", "%s", AERO_VERSION);
    kv_row("Commit", "%s", AERO_REVISION);
    kv_row("Branch", "%s", AERO_GIT_BRANCH);
    ImGui::EndTable();
  }
}

static void build_report(char *out, size_t cap, SDL_Window *window,
                         SDL_Renderer *renderer) {
  LibRow rows[8];
  int n = 0;
  fill_libs(rows, &n);
  char cc[160];
  compiler_string(cc, sizeof cc);
  const char *rname = SDL_GetRendererName(renderer);
  (void)window;

  size_t len = (size_t)snprintf(
      out, cap,
      "%s v%s (%s) rev %s [%s]\nbuilt %s %s with %s, %s %u-bit\n"
      "platform: %s, renderer: %s\nlibraries:\n",
      PROJECT_NAME, AERO_VERSION, AERO_BUILD_TYPE, AERO_REVISION,
      AERO_GIT_BRANCH, __DATE__, __TIME__, cc, arch_string(),
      (unsigned)(sizeof(void *) * 8), SDL_GetPlatform(),
      rname ? rname : "n/a");
  for (int i = 0; i < n && len < cap; i++) {
    len += (size_t)snprintf(out + len, cap - len, "  %s: compiled %s, linked %s\n",
                            rows[i].name, rows[i].compiled, rows[i].linked);
  }
}

/* ---- window ------------------------------------------------------------- */

void about_panel_draw(bool *open, SDL_Window *window, SDL_Renderer *renderer) {
  ImGui::SetNextWindowSize(ImVec2(680.0f, 620.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(PANEL_ABOUT, open)) {
    ImGui::End();
    return;
  }

  if (ImGui::BeginTabBar("##about_tabs")) {
    if (ImGui::BeginTabItem("Overview")) {
      tab_overview();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Libraries")) {
      tab_libraries();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Runtime")) {
      tab_runtime(window, renderer);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Build")) {
      tab_build();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::Spacing();
  ImGui::Separator();
  if (ImGui::Button("Copy build report")) {
    char buf[2048];
    build_report(buf, sizeof buf, window, renderer);
    ImGui::SetClipboardText(buf);
  }
  ImGui::SameLine();
  if (ImGui::Button("Open GitHub")) {
    SDL_OpenURL(PROJECT_URL);
  }
  ImGui::SameLine();
  ImGui::TextDisabled("(c) %s", DEVELOPER_NAME);

  ImGui::End();
}
