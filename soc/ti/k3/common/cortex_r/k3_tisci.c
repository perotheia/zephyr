/*
 * Copyright (c) 2026 The Perotheia authors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * A minimal TI-SCI client for a K3 R5F, polled over the secure proxy, and what this SoC code uses it for.
 *
 * Route device interrupts (mailbox, UART, ...) through an interrupt router with TI-SCI, the way TI's PDK sciclient does it
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
/*
 * Reply timeout as a poll count, not a time: this runs before the system
 * clock (PRE_KERNEL_1), so k_busy_wait() is not available. One poll is one
 * device register read, some hundred ns; the system firmware answers in well
 * under a millisecond.
 */
#define SP_TIMEOUT_POLLS 4000000U

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

	for (uint32_t polls = 0U;; polls++) {
		st = sys_read32(rt);
		if ((st & SP_RT_ERR) != 0U) {
			return -EIO;
		}
		if ((st & SP_RT_CNT_MASK) != 0U) {
			break;
		}
		if (polls >= SP_TIMEOUT_POLLS) {
			return -ETIMEDOUT;
		}
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

/* Drop responses left in the RX thread by a previous image of this core */
static void k3_sp_flush(void)
{
	mem_addr_t t = SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_DATA, CONFIG_SOC_TI_K3_TISCI_RX_THREAD);
	mem_addr_t rt = SP_THREAD(CONFIG_SOC_TI_K3_TISCI_SECPROXY_RT, CONFIG_SOC_TI_K3_TISCI_RX_THREAD);

	for (int i = 0; i < 16; i++) {
		uint32_t st = sys_read32(rt);

		if ((st & SP_RT_ERR) != 0U || (st & SP_RT_CNT_MASK) == 0U) {
			break;
		}
		(void)sys_read32(t + SP_DATA_END); /* reading the last register pops */
	}
}

/* 0 = ACK, 1 = NAK, < 0 = transport error */
static int k3_tisci_xfer(void *req, size_t req_len, void *resp, size_t resp_len)
{
	static bool flushed;
	struct tisci_hdr *q = req, *r = resp;
	int rc;

	if (!flushed) {
		k3_sp_flush();
		flushed = true;
	}

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
		if (r->seq == q->seq && r->type == q->type) {
			return ((r->flags & TISCI_FLAG_ACK) != 0U) ? 0 : 1;
		}
	}
	return -EPROTO;
}

static __maybe_unused int k3_ir_route(uint16_t ir_dev, uint16_t ir_input, uint16_t ir_output)
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
		/*
		 * The firmware refuses a route that is already programmed, which
		 * is what a restart of this core finds; it also refuses a wrong
		 * or foreign one, and the two cannot be told apart from here.
		 */
		LOG_WRN("IR %u: input %u -> output %u refused: assumed kept from a previous run", ir_dev,
			ir_input, ir_output);
	} else {
		LOG_ERR("IR %u: input %u -> output %u: TI-SCI transport error %d", ir_dev, ir_input,
			ir_output, rc);
	}
	return rc;
}

#define TISCI_MSG_SET_DEVICE       0x0200U
#define TISCI_DEVICE_SW_STATE_ON   2U

struct tisci_set_device_req {
	struct tisci_hdr hdr;
	uint32_t id;
	uint32_t reserved;
	uint8_t state;
} __packed;

/* Request a device (power + module clock) on, as TI's PDK Sciclient_pmSetModuleState(ON) does */
static __maybe_unused int k3_tisci_set_device_on(uint32_t dev_id)
{
	struct tisci_set_device_req req = {
		.hdr.type = TISCI_MSG_SET_DEVICE,
		.id = dev_id,
		.state = TISCI_DEVICE_SW_STATE_ON,
	};
	struct tisci_hdr resp;

	return k3_tisci_xfer(&req, sizeof(req), &resp, sizeof(resp));
}

#ifdef CONFIG_SOC_TI_K3_TISCI_DEVICES
/*
 * Devices are powered on before every driver, the PRE_KERNEL_1 ones (UART)
 * included, which is before the console: keep the results and report them at
 * POST_KERNEL. A device id shared by several nodes (GPIO banks) is requested
 * once.
 */
#define K3_DEVICES_MAX 24U

static struct {
	uint16_t id;
	int8_t rc;
} k3_devices[K3_DEVICES_MAX];
static uint8_t k3_devices_n;

static void k3_device_on_early(uint32_t dev_id)
{
	for (uint8_t i = 0U; i < k3_devices_n; i++) {
		if (k3_devices[i].id == dev_id) {
			return;
		}
	}
	int rc = k3_tisci_set_device_on(dev_id);

	if (k3_devices_n < K3_DEVICES_MAX) {
		k3_devices[k3_devices_n].id = (uint16_t)dev_id;
		k3_devices[k3_devices_n].rc = (int8_t)CLAMP(rc, INT8_MIN, INT8_MAX);
		k3_devices_n++;
	}
}

#define K3_DEVICE_ON(node)                                                                         \
	COND_CODE_1(DT_NODE_HAS_PROP(node, ti_sci_dev_id),                                         \
		    (k3_device_on_early(DT_PROP(node, ti_sci_dev_id));), ())

static int k3_tisci_devices_init(void)
{
	DT_FOREACH_STATUS_OKAY_NODE(K3_DEVICE_ON)
	return 0;
}

static int k3_tisci_devices_report(void)
{
	for (uint8_t i = 0U; i < k3_devices_n; i++) {
		if (k3_devices[i].rc == 0) {
			LOG_INF("device %u on", k3_devices[i].id);
		} else {
			LOG_ERR("device %u: SET_DEVICE(ON) %s %d", k3_devices[i].id,
				k3_devices[i].rc == 1 ? "NAK" : "error", k3_devices[i].rc);
		}
	}
	return 0;
}

SYS_INIT(k3_tisci_devices_init, PRE_KERNEL_1, 1);
SYS_INIT(k3_tisci_devices_report, POST_KERNEL, 0);
#endif /* CONFIG_SOC_TI_K3_TISCI_DEVICES */

#ifdef CONFIG_SOC_TI_K3_TISCI_IR_ROUTE
#define K3_IR_ROUTE(node)                                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(node, ti_sci_ir_route),                                       \
		    ((void)k3_ir_route(DT_PROP_BY_IDX(node, ti_sci_ir_route, 0),                   \
				       DT_PROP_BY_IDX(node, ti_sci_ir_route, 1),                   \
				       DT_PROP_BY_IDX(node, ti_sci_ir_route, 2));),                \
		    ())

static int k3_ir_routes_init(void)
{
	DT_FOREACH_STATUS_OKAY_NODE(K3_IR_ROUTE)
	return 0;
}

/* before the mailbox driver */
SYS_INIT(k3_ir_routes_init, POST_KERNEL, 0);
#endif /* CONFIG_SOC_TI_K3_TISCI_IR_ROUTE */
