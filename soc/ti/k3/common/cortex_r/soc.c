/* Copyright (C) 2023 BeagleBoard.org Foundation
 * Copyright (C) 2023 S Prashanth
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/fatal.h>

#include "soc.h"
#include "ctrl_partitions.h"
#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>

unsigned int z_soc_irq_get_active(void)
{
	return z_vim_irq_get_active();
}

void z_soc_irq_eoi(unsigned int irq)
{
	z_vim_irq_eoi(irq);
}

void z_soc_irq_init(void)
{
	z_vim_irq_init();
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	/* Configure interrupt type and priority */
	z_vim_irq_priority_set(irq, prio, flags);
}

void z_soc_irq_enable(unsigned int irq)
{
	/* Enable interrupt */
	z_vim_irq_enable(irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	/* Disable interrupt */
	z_vim_irq_disable(irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	/* Check if interrupt is enabled */
	return z_vim_irq_is_enabled(irq);
}

#ifdef CONFIG_SOC_TI_K3_R5F_EARLY_MPU
/*
 * TI's reference table, region number = index. DRSR = (log2(size) - 1) << 1 | EN,
 * DRACR = XN << 12 | AP << 8 | TEX << 3 | S << 2 | C << 1 | B, encoded as TI's CSL
 * does: cacheable write-back write-allocate = TEX 0b101 CB 0b01, cacheable
 * non-cacheable = TEX 0b100, strongly-ordered = all zero.
 */
#define MPU_DRSR(log2_size) ((((log2_size) - 1U) << 1) | 1U)
#define MPU_XN              BIT(12)
#define MPU_AP_RW           (3U << 8)
#define MPU_SO              0U
#define MPU_WB_WA           ((5U << 3) | 1U)
#define MPU_TCM_NC          (4U << 3)
#define MPU_NORMAL_NC       (1U << 3)

#if DT_HAS_CHOSEN(zephyr_ipc_shm)
#define IPC_WINDOW_BASE (DT_REG_ADDR(DT_CHOSEN(zephyr_ipc_shm)) & \
			 ~(CONFIG_SOC_TI_K3_R5F_IPC_WINDOW_SIZE - 1U))
#define IPC_WINDOW_DRSR MPU_DRSR(LOG2(CONFIG_SOC_TI_K3_R5F_IPC_WINDOW_SIZE))
#else
#define IPC_WINDOW_BASE 0U
#define IPC_WINDOW_DRSR 0U
#endif

static const struct {
	uint32_t base;
	uint32_t drsr;
	uint32_t dracr;
} k3_r5f_mpu[] = {
	{0x00000000U, MPU_DRSR(32), MPU_XN | MPU_AP_RW | MPU_SO},        /* background */
	{0x00000000U, MPU_DRSR(15), MPU_AP_RW | MPU_WB_WA},              /* ATCM */
	{0x41c00000U, MPU_DRSR(19), MPU_AP_RW | MPU_WB_WA},              /* OCSRAM */
	{0x70000000U, MPU_DRSR(23), MPU_AP_RW | MPU_WB_WA},              /* MSMC */
	{0x80000000U, MPU_DRSR(31), MPU_AP_RW | MPU_WB_WA},              /* DDR */
	{0x41010000U, MPU_DRSR(15), MPU_AP_RW | MPU_TCM_NC},             /* BTCM */
	{0x00000000U, 0U, 0U},                                           /* unused */
	{IPC_WINDOW_BASE, IPC_WINDOW_DRSR, MPU_XN | MPU_AP_RW | MPU_NORMAL_NC}, /* IPC */
	{0xaa000000U, MPU_DRSR(25), MPU_XN | MPU_AP_RW | MPU_SO},        /* shared vrings */
};

static inline void k3_r5f_mpu_region(uint32_t idx, uint32_t base, uint32_t drsr, uint32_t dracr)
{
	__asm__ volatile("mcr p15, 0, %0, c6, c2, 0" ::"r"(idx));
	barrier_isync_fence_full();
	/* disable the region while it is rewritten */
	__asm__ volatile("mcr p15, 0, %0, c6, c1, 2" ::"r"(0U));
	__asm__ volatile("mcr p15, 0, %0, c6, c1, 0" ::"r"(base));
	__asm__ volatile("mcr p15, 0, %0, c6, c1, 4" ::"r"(dracr));
	__asm__ volatile("mcr p15, 0, %0, c6, c1, 2" ::"r"(drsr));
	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}

void soc_prep_hook(void)
{
	uint32_t mpuir, num, sctlr;

	__asm__ volatile("mrc p15, 0, %0, c0, c0, 4" : "=r"(mpuir));
	num = (mpuir >> 8) & 0xffU;

	/*
	 * The MPU is enabled (springboard) and the code runs from DDR: write the
	 * regions from the highest number down so that the DDR region (4) is in
	 * place before region 0 turns into the execute-never background.
	 */
	for (uint32_t i = num; i-- > ARRAY_SIZE(k3_r5f_mpu);) {
		k3_r5f_mpu_region(i, 0U, 0U, 0U);
	}
	for (uint32_t i = ARRAY_SIZE(k3_r5f_mpu); i-- > 0U;) {
		k3_r5f_mpu_region(i, k3_r5f_mpu[i].base, k3_r5f_mpu[i].drsr, k3_r5f_mpu[i].dracr);
	}

	/* region 0 is the background now */
	sctlr = __get_SCTLR();
	sctlr &= ~SCTLR_BR_Msk;
	barrier_dsync_fence_full();
	__set_SCTLR(sctlr);
	barrier_isync_fence_full();
}
#endif /* CONFIG_SOC_TI_K3_R5F_EARLY_MPU */

void soc_early_init_hook(void)
{
	sys_cache_data_enable();
	sys_cache_instr_enable();

	k3_unlock_all_ctrl_partitions();
}
