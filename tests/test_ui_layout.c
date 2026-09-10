#include "ui/ui_layout.h"

#include "test_util.h"

static void test_pad_and_inset(void) {
  UiRect r = {10, 20, 100, 80};

  UiRect i = ui_rect_inset(r, 5, 4);
  CHECK_NEAR(i.x, 15, 1e-6);
  CHECK_NEAR(i.y, 24, 1e-6);
  CHECK_NEAR(i.w, 90, 1e-6);
  CHECK_NEAR(i.h, 72, 1e-6);

  UiRect p = ui_rect_pad(r, 1, 2, 3, 4);
  CHECK_NEAR(p.x, 11, 1e-6);
  CHECK_NEAR(p.y, 22, 1e-6);
  CHECK_NEAR(p.w, 96, 1e-6);
  CHECK_NEAR(p.h, 74, 1e-6);

  /* over-inset clamps to zero, never negative */
  UiRect z = ui_rect_inset(r, 200, 200);
  CHECK_NEAR(z.w, 0, 1e-6);
  CHECK_NEAR(z.h, 0, 1e-6);
  CHECK(!ui_rect_valid(z));
  CHECK(ui_rect_valid(r));
}

static void test_split_edges(void) {
  UiRect r = {0, 0, 200, 100};
  UiRect rest;

  UiRect top = ui_split_top(r, 30, 10, &rest);
  CHECK_NEAR(top.h, 30, 1e-6);
  CHECK_NEAR(top.y, 0, 1e-6);
  CHECK_NEAR(rest.y, 40, 1e-6);
  CHECK_NEAR(rest.h, 60, 1e-6);
  CHECK_NEAR(rest.w, 200, 1e-6);

  UiRect bot = ui_split_bottom(r, 25, 0, &rest);
  CHECK_NEAR(bot.y, 75, 1e-6);
  CHECK_NEAR(bot.h, 25, 1e-6);
  CHECK_NEAR(rest.h, 75, 1e-6);

  UiRect lft = ui_split_left(r, 50, 10, &rest);
  CHECK_NEAR(lft.w, 50, 1e-6);
  CHECK_NEAR(rest.x, 60, 1e-6);
  CHECK_NEAR(rest.w, 140, 1e-6);

  UiRect rgt = ui_split_right(r, 40, 0, &rest);
  CHECK_NEAR(rgt.x, 160, 1e-6);
  CHECK_NEAR(rgt.w, 40, 1e-6);
  CHECK_NEAR(rest.w, 160, 1e-6);
}

static void test_split_clamps_and_null_rest(void) {
  UiRect r = {0, 0, 100, 100};

  /* asking for more than exists yields the whole span and an empty remainder */
  UiRect rest;
  UiRect big = ui_split_top(r, 999, 10, &rest);
  CHECK_NEAR(big.h, 100, 1e-6);
  CHECK_NEAR(rest.h, 0, 1e-6);

  /* NULL rest must not crash */
  UiRect s = ui_split_left(r, 20, 5, NULL);
  CHECK_NEAR(s.w, 20, 1e-6);
}

static void test_split_frac(void) {
  UiRect r = {0, 0, 200, 100};
  UiRect rest;

  UiRect half = ui_split_left_frac(r, 0.25f, 0, &rest);
  CHECK_NEAR(half.w, 50, 1e-6);
  CHECK_NEAR(rest.w, 150, 1e-6);

  UiRect t = ui_split_top_frac(r, 0.5f, 0, &rest);
  CHECK_NEAR(t.h, 50, 1e-6);

  /* frac clamps to [0,1] */
  UiRect over = ui_split_top_frac(r, 3.0f, 0, &rest);
  CHECK_NEAR(over.h, 100, 1e-6);
}

static void test_grid(void) {
  UiRect area = {0, 0, 320, 200};
  UiGrid g = ui_grid(area, 3, 2, 10);

  /* (330 - 2*10)/3 = 100 ; (200 - 1*10)/2 = 95 */
  UiRect c00 = ui_grid_cell(g, 0, 0);
  CHECK_NEAR(c00.w, 100, 1e-6);
  CHECK_NEAR(c00.h, 95, 1e-6);
  CHECK_NEAR(c00.x, 0, 1e-6);

  UiRect c21 = ui_grid_cell(g, 2, 1);
  CHECK_NEAR(c21.x, 220, 1e-6); /* 2*(100+10) */
  CHECK_NEAR(c21.y, 105, 1e-6); /* 1*(95+10) */

  /* linear index is row-major */
  UiRect at4 = ui_grid_at(g, 4); /* col 1, row 1 */
  UiRect c11 = ui_grid_cell(g, 1, 1);
  CHECK_NEAR(at4.x, c11.x, 1e-6);
  CHECK_NEAR(at4.y, c11.y, 1e-6);

  /* out of range -> zero rect */
  UiRect oob = ui_grid_cell(g, 3, 0);
  CHECK_NEAR(oob.w, 0, 1e-6);
  CHECK(!ui_rect_valid(ui_grid_at(g, 99)));

  /* degenerate cols/rows are coerced to at least 1 */
  UiGrid g1 = ui_grid(area, 0, -4, 5);
  CHECK(g1.cols == 1);
  CHECK(g1.rows == 1);
}

static void test_stack(void) {
  UiRect area = {5, 5, 100, 100};
  UiStack s = ui_stack(area, 4);

  UiRect r0 = ui_stack_row(&s, 20);
  CHECK_NEAR(r0.y, 5, 1e-6);
  CHECK_NEAR(r0.h, 20, 1e-6);

  UiRect r1 = ui_stack_row(&s, 20);
  CHECK_NEAR(r1.y, 29, 1e-6); /* 5 + 20 + 4 */

  UiRect rest = ui_stack_rest(&s);
  CHECK_NEAR(rest.y, 53, 1e-6); /* 29 + 20 + 4 */
  CHECK_NEAR(rest.h, 52, 1e-6); /* 105 - 53 */

  /* running past the bottom yields zero-height rows, not negative */
  ui_stack_row(&s, 200);
  UiRect after = ui_stack_row(&s, 20);
  CHECK_NEAR(after.h, 0, 1e-6);
  CHECK(!ui_rect_valid(after));
}

static const TestCase kCases[] = {
    {"pad_and_inset", test_pad_and_inset},
    {"split_edges", test_split_edges},
    {"split_clamps_and_null_rest", test_split_clamps_and_null_rest},
    {"split_frac", test_split_frac},
    {"grid", test_grid},
    {"stack", test_stack},
};

RUN_TESTS(kCases)
