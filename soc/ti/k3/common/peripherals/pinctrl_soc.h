/*
 * Copyright (c) 2023 Enphase Energy
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM64_TI_K3_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM64_TI_K3_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * One entry per devicetree cell: a group node's pinmux property is a list of
 * <offset value> pairs (K3_PINMUX), and every pair of a group is applied, not
 * only the first one. Entry 2n carries the pad offset, entry 2n + 1 its value.
 */
struct pinctrl_soc_pin {
	const struct device *dev;
	uint32_t cell;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#define TI_K3_DT_PIN_CELL(node_id, prop, idx, _)                                                   \
	{.dev = DEVICE_DT_GET(DT_PARENT(node_id)), .cell = DT_PROP_BY_IDX(node_id, prop, idx)},

/* the _VARGS form: a plain DT_FOREACH_PROP_ELEM cannot nest in the one below */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	DT_FOREACH_PROP_ELEM_VARGS(DT_PROP_BY_IDX(node_id, prop, idx), pinmux, TI_K3_DT_PIN_CELL, 0)

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM64_TI_K3_PINCTRL_SOC_H_ */
