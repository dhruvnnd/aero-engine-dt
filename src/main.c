
/*
 * This example code creates an SDL window and renderer, and then clears the
 * window to a different color every frame, so you'll effectively get a window
 * that's smoothly fading between colors.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

#include "SDL3/SDL_render.h"
#include "SDL3/SDL_stdinc.h"
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

/* We will use this renderer to draw into this window every frame. */
static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  SDL_SetAppMetadata("Example Renderer Clear", "1.0",
                     "com.example.renderer-clear");

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }

  if (!SDL_CreateWindowAndRenderer("aero engine dt | simulator", 640, 480,
                                   SDL_WINDOW_RESIZABLE, &window, &renderer)) {
    SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
    return SDL_APP_FAILURE;
  }
  SDL_SetRenderLogicalPresentation(renderer, 640, 480,
                                   SDL_LOGICAL_PRESENTATION_LETTERBOX);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS; /* end the program, reporting success to the OS. */
  }
  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void *appstate) {
  /* 1. Clear the screen to a background color (e.g., dark blue) */
  SDL_SetRenderDrawColor(renderer, 20, 40, 80, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(renderer);

  /* 2. Set color for the debug text (e.g., solid white) */
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

  /* 3. Render a simple static string */
  SDL_RenderDebugText(renderer, 10, 10, "Aero Engine DT | Simulator Running");

  /* 4. Render a formatted string (great for variables/frame times) */
  const Uint64 ticks = SDL_GetTicks();
  SDL_RenderDebugTextFormat(renderer, 10, 30, "Time: %.2f seconds",
                            (double)ticks / 1000.0);

  /* FPS counter */
  static Uint64 last_ticks = 0;
  static float fps = 0.0f;
  const Uint64 curr_ticks = SDL_GetTicks();
  Uint64 frame_time = curr_ticks - last_ticks;
  if (frame_time > 0) {
    fps = 1000.0f / (float)frame_time;
  }
  last_ticks = curr_ticks;

  SDL_RenderDebugTextFormat(renderer, 10, 40, "FPS: %.2f", (double)fps);

  /* 5. Put everything on the screen */
  SDL_RenderPresent(renderer);

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  /* SDL will clean up the window/renderer for us. */
}
