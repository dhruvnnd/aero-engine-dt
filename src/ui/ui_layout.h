#ifndef UI_UI_LAYOUT_H
#define UI_UI_LAYOUT_H

typedef struct {
  float x, y, w, h;
} UiRect;

/* Shrink `r` by `dx` on the left and right and `dy` on top and bottom. */
UiRect ui_rect_inset(UiRect r, float dx, float dy);

/* Shrink `r` by an independent margin on each edge. */
UiRect ui_rect_pad(UiRect r, float left, float top, float right, float bottom);

/* True if the rect has strictly positive area (worth drawing into). */
int ui_rect_valid(UiRect r);

/* Carve a strip of the given pixel height/width off one edge of `r`.
 * Returns the strip; writes the remainder to *rest (NULL ok). */
UiRect ui_split_top(UiRect r, float amount, float gap, UiRect *rest);
UiRect ui_split_bottom(UiRect r, float amount, float gap, UiRect *rest);
UiRect ui_split_left(UiRect r, float amount, float gap, UiRect *rest);
UiRect ui_split_right(UiRect r, float amount, float gap, UiRect *rest);

/* As above but the strip size is a fraction (0..1) of the relevant span,
 * measured before the gap is removed. `frac` is clamped to [0, 1]. */
UiRect ui_split_top_frac(UiRect r, float frac, float gap, UiRect *rest);
UiRect ui_split_left_frac(UiRect r, float frac, float gap, UiRect *rest);

/* Uniform grid over an area: `cols` x `rows` cells with `gap` between them. */
typedef struct {
  UiRect area;
  int cols, rows;
  float gap;
} UiGrid;

UiGrid ui_grid(UiRect area, int cols, int rows, float gap);

/* Cell at (col, row), 0-based. Out-of-range indices return a zero rect. */
UiRect ui_grid_cell(UiGrid g, int col, int row);

/* Cell by row-major linear index (0 .. cols*rows-1). */
UiRect ui_grid_at(UiGrid g, int index);

/* Top-to-bottom cursor that hands out rows of a requested height, each
 * followed by `row_gap`. Runs off the bottom silently (rows become zero). */
typedef struct {
  UiRect area;
  float cursor_y;
  float row_gap;
} UiStack;

UiStack ui_stack(UiRect area, float row_gap);
UiRect ui_stack_row(UiStack *s, float height);

/* Remaining unspent area below the cursor. */
UiRect ui_stack_rest(const UiStack *s);

#endif /* UI_UI_LAYOUT_H */
