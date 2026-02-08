/**
 * robodash_selector.h
 *
 * Optional RoboDash hooks for autonomous selector display.
 * Default implementation is a no-op; wire in RoboDash as needed.
 */
#ifndef ROBODASH_SELECTOR_H
#define ROBODASH_SELECTOR_H

#include "robodash/api.h"

void sbot_robodash_init();
void sbot_robodash_update_selector(int idx, const char* name, bool confirmed);
bool sbot_robodash_get_selection(int* idx, bool* confirmed);
void sbot_robodash_set_selection(int idx, bool confirmed);
extern rd::Selector selector;

#endif // ROBODASH_SELECTOR_H
