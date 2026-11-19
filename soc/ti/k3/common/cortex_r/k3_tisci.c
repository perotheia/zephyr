/*
 * Copyright (c) 2026 The Perotheia authors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * A minimal TI-SCI client for a K3 R5F, polled over the secure proxy, and what this SoC code uses it for.
 *
 * Route mailbox interrupts through the NAVSS interrupt router with TI-SCI, the way TI's PDK sciclient does it
 * (Sciclient_rmIrqProgramRoute, one IR hop): RM_IRQ_SET with src = dst = the interrupt router, src_index = the
 * router input, dst_host_irq = the router output, secondary host = this host. The router output is hard-wired to
 * a VIM line of this core (the mailbox node's interrupt).
 *
 * The TI-SCI messages go over the secure proxy and are polled: this core's secure proxy threads are above the
 * range the generic secure proxy driver handles, and its response interrupt would itself need routing.
 *
 * A route set by a previous run of this core is still in place after a restart with only a local reset (the
 * router is outside the core): sysfw then NAKs the SET and the existing route is used as it is (releasing and
 * setting it again breaks delivery).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(k3_tisci, CONFIG_SOC_LOG_LEVEL);

#define SP_THREAD(base, t)  ((base) + 0x1000U * (uint32_t)(t))
#define SP_DATA_START       0x4U
#define SP_DATA_END         0x3cU
#define SP_RT_ERR           BIT(31)
#define SP_RT_CNT_MASK      0xffU
#define SP_TIMEOUT_US       100000U

#define TISCI_MSG_RM_IRQ_SET          0x1000U
#define TISCI_FLAG_AOP                BIT(1)
#define TISCI_FLAG_ACK                BIT(1)
#define TISCI_RM_DST_ID_VALID         BIT(0)
#define TISCI_RM_DST_HOST_IRQ_VALID   BIT(1)
#define TISCI_RM_SECONDARY_HOST_VALID BIT(31)

struct tisci_hdr {
	uint16_t type;
	uint8_t host;
	uint8_t seq;
	uint32_t flags;
} __packed;

struct tisci_rm_irq_set_req {
	struct tisci_hdr hdr;
	uint32_t valid_params;
	uint16_t src_id, src_index, dst_id, dst_host_irq, ia_id, vint, global_event;
	uint8_t vint_status_bit_index, secondary_host;
} __packed;

static uint8_t k3_tisci_seq;

static int k3_sp_send(const void *msg, size_t len)
{
	mem_addr_t t = SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_DATA, CONFIG_SOC_TI_K3_TISCI_TX_THREAD);
	uint32_t st = sys_read32(SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_RT, CONFIG_SOC_TI_K3_TISCI_TX_THREAD));
	mem_addr_t d = t + SP_DATA_START;
	const uint8_t *b = msg;
	uint32_t w;

	if ((st & SP_RT_ERR) != 0U) {
		return -EIO;
	}
	if ((st & SP_RT_CNT_MASK) == 0U) {
		return -EBUSY;
	}
	for (size_t i = 0; i < len; i += 4U, d += 4U) {
		w = 0U;
		memcpy(&w, b + i, MIN(4U, len - i));
		sys_write32(w, d);
	}
	if (d <= t + SP_DATA_END) {
		sys_write32(0U, t + SP_DATA_END); /* writing the last register sends */
	}
	return 0;
}

static int k3_sp_recv(void *msg, size_t len)
{
	mem_addr_t t = SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_DATA, CONFIG_SOC_TI_K3_TISCI_RX_THREAD);
	mem_addr_t rt = SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_RT, CONFIG_SOC_TI_K3_TISCI_RX_THREAD);
	mem_addr_t d = t + SP_DATA_START;
	uint8_t *b = msg;
	uint32_t st, w;

	for (uint32_t us = 0U;; us += 10U) {
		st = sys_read32(rt);
		if ((st & SP_RT_ERR) != 0U) {
			return -EIO;
		}
		if ((st & SP_RT_CNT_MASK) != 0U) {
			break;
		}
		if (us >= SP_TIMEOUT_US) {
			return -ETIMEDOUT;
		}
		k_busy_wait(10U);
	}
	for (size_t i = 0; i < len; i += 4U, d += 4U) {
		w = sys_read32(d);
		memcpy(b + i, &w, MIN(4U, len - i));
	}
	if (d <= t + SP_DATA_END) {
		(void)sys_read32(t + SP_DATA_END); /* reading the last register pops */
	}
	return 0;
}

/* 0 = ACK, 1 = NAK, < 0 = transport error */
static int k3_tisci_xfer(void *req, size_t req_len, void *resp, size_t resp_len)
{
	struct tisci_hdr *q = req, *r = resp;
	int rc;

	q->host = CONFIG_SOC_TI_K3_TISCI_HOST_ID;
	q->seq = ++k3_tisci_seq;
	q->flags = TISCI_FLAG_AOP;
	rc = k3_sp_send(req, req_len);
	if (rc != 0) {
		return rc;
	}
	for (int tries = 0; tries < 4; tries++) {
		rc = k3_sp_recv(resp, resp_len);
		if (rc != 0) {
			return rc;
		}
		if (r->seq == q->seq) {
			return ((r->flags & TISCI_FLAG_ACK) != 0U) ? 0 : 1;
		}
	}
	return -EPROTO;
}

int k3_ir_route(uint16_t ir_dev, uint16_t ir_input, uint16_t ir_output)
{
	struct tisci_rm_irq_set_req req = {
		.hdr.type = TISCI_MSG_RM_IRQ_SET,
		.valid_params = TISCI_RM_DST_ID_VALID | TISCI_RM_DST_HOST_IRQ_VALID |
				TISCI_RM_SECONDARY_HOST_VALID,
		.src_id = ir_dev,
		.src_index = ir_input,
		.dst_id = ir_dev,
		.dst_host_irq = ir_output,
		.secondary_host = CONFIG_SOC_TI_K3_TISCI_HOST_ID,
	};
	struct tisci_hdr resp;
	int rc = k3_tisci_xfer(&req, sizeof(req), &resp, sizeof(resp));

	if (rc == 0) {
		LOG_INF("IR %u: input %u -> output %u routed", ir_dev, ir_input, ir_output);
	} else if (rc == 1) {
		LOG_INF("IR %u: input %u -> output %u refused: kept from a previous run, used as is", ir_dev,
			ir_input, ir_output);
	} else {
		LOG_ERR("IR %u: input %u -> output %u: TI-SCI transport error %d", ir_dev, ir_input,
			ir_output, rc);
	}
	return rc;
}

#ifdef CONFIG_SOC_TI_K3_TISCI_IR_ROUTE
#define K3_MBOX_IR_ROUTE(node)                                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(node, ti_sci_ir_route),                                       \
		    ((void)k3_ir_route(DT_PROP_BY_IDX(node, ti_sci_ir_route, 0),                   \
				       DT_PROP_BY_IDX(node, ti_sci_ir_route, 1),                   \
				       DT_PROP_BY_IDX(node, ti_sci_ir_route, 2));),                \
		    ())

static int k3_ir_routes_init(void)
{
	DT_FOREACH_STATUS_OKAY(ti_omap_mailbox, K3_MBOX_IR_ROUTE)
	return 0;
}

/* before the mailbox driver */
SYS_INIT(k3_ir_routes_init, POST_KERNEL, 0);
#endif /* CONFIG_SOC_TI_K3_TISCI_IR_ROUTE */
