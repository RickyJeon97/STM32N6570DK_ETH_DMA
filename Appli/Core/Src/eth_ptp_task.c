/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    eth_ptp_task.c
  * @brief   Ethernet PTP timestamp validation task.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "eth_ptp_task.h"
#include "main.h"
#include "stm32n6xx_hal_eth_ex.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

/* --- Configuration ------------------------------------------------------- */
#define ETH_PTP_TASK_STACK_SIZE     (1024U)
#define ETH_PTP_TASK_PRIORITY       (osPriorityAboveNormal)
#define ETH_PTP_RX_BUF_SIZE         (1536U)
#define ETH_PTP_RX_BUF_COUNT        (ETH_RX_DESC_CNT * ETH_DMA_RX_CH_CNT)
#define ETH_PTP_RX_SEM_MAX_COUNT    (8U)

/* Test/bring-up toggles */
#define ETH_PTP_TESTS_ENABLE        (1U)
#define ETH_PTP_DESC_LOG_ONLY       (0U) /* 1: keep descriptor logs only */
#define ETH_PTP_TX_TEST_ENABLE      (1U)
#define ETH_PTP_TX_TEST_INTERVAL_MS (1000U)
#define ETH_PTP_TX_USE_IT           (1U) /* 0: polling TX, 1: interrupt TX */
#define ETH_PTP_TX_USE_PTP_TS       (0U) /* 0: disable PTP TX timestamp insert */
#define ETH_PTP_TX_LEARN_ENABLE     (0U) /* 1: send broadcast frame for MAC learn */
#define ETH_PTP_TX_LEARN_ONCE       (1U) /* 1: send only once at start */
#define ETH_PTP_TX_LEARN_INTERVAL_MS (2000U)
#define ETH_PTP_TX_REG_DUMP         (0U) /* 1: dump minimal TX regs on error */
#define ETH_PTP_TX_DESC_DUMP        (0U) /* 1: dump TX descriptor fields */
#define ETH_PTP_TX_PROBE_LOG        (0U) /* 1: log per-TX probe deltas */
#define ETH_PTP_TX_PATH_LOG         (0U) /* 1: one-shot submit path log (catxdr/catxbr/csr/qdr) */
#define ETH_PTP_TX_BAD_DLAR_PROBE   (0U) /* 1: one-shot invalid-DLAR probe to check if AXI read is attempted */
#define ETH_PTP_TX_CPLT_LOG         (0U) /* 1: log cplt only when it increments */
#define ETH_PTP_TX_KICK_HACKS       (0U) /* 1: enable TX doorbell/recover hacks (debug only) */
#define ETH_PTP_TX_TEST_ON_LINKUP_ONLY (1U) /* 1: send TX test once on link-up only */
#define ETH_PTP_TX_STABLE_LINK_MS   (500U)  /* require link-up stable before one-shot TX */
#define ETH_PTP_TX_SUBMIT_LOG       (0U) /* 1: log a single line on TX submit result */
#define ETH_PTP_TX_RECOVER_TBU      (0U) /* 1: clear TBU/TPS and re-kick tail ptr after submit */
#define ETH_PTP_RX_FRAME_DUMP       (0U) /* 1: dump RX frame headers */
#define ETH_PTP_PROGRESS_LOG        (0U) /* 1: periodic rx/cplt progress logs */
#define ETH_PTP_DIAG_INTERNAL_LB    (0U) /* 1: MAC internal loopback diagnostic */
/* Keep RX enabled during TX bring-up; on this IP, stopping RX can leave the DMA
 * channel in a stopped state (RPS) and prevent TX descriptor fetch. */
#define ETH_PTP_RX_DISABLE_FOR_TX_TEST (0U)
#define ETH_PTP_LINK_POLL_ENABLE    (1U)
#define ETH_PTP_LINK_POLL_MS        (1000U)
#define ETH_PTP_LINK_POLL_ALWAYS_LOG (0U)
#define ETH_PTP_PREKICK_ON_LINKUP    (0U)
/* During bring-up, automatic Stop/Start on link transitions adds noise and can
 * hide init-order issues (and would require re-applying DMA/descriptor knobs).
 * Keep it off until basic TX/RX is proven. */
#define ETH_PTP_RESTART_ETH_ON_LINK_CHANGE (0U)
/* Force MTL TXQ0 to normal queue mode (TXQEN=0b01), not AV mode. */
#define ETH_PTP_FORCE_TXQ0_NORMAL_MODE (1U)

/* CubeMX enables 2 DMA channels/queues by default. For bring-up, force only CH0/Q0
 * to remove ambiguity about which channel is active. */
#define ETH_PTP_BRINGUP_SINGLE_CH0 (1U)

/* When TX DMA stalls with TBU, try programming the Tx descriptor base/tail
 * using the Non-secure alias of the same SRAM (0x34xxxxxx -> 0x24xxxxxx).
 * This is a diagnostic to detect S/NS master/address mismatches. */
#define ETH_PTP_TX_DIAG_TRY_NS_ALIAS (0U)
/* Decisive test: keep TX descriptor ring/buffer on NS alias
 * (0x34xxxxxx -> 0x24xxxxxx) and do NOT restore. */
#define ETH_PTP_TX_FORCE_NS_ALIAS_ONCE (0U)

/* On TX stall, try a couple of tail-pointer "kick" variants to determine what
 * DMACTXDTPR semantics this ETH instance expects (points-to-last vs points-to-next). */
#define ETH_PTP_TX_DIAG_TAILPTR_KICKS (1U)

/* AXI4 descriptor/data cache controls (ETH DMAA4* registers). If descriptor cache
 * is enabled without coherency, DMA can keep seeing stale OWN=0 and suspend with no AXI reads. */
#define ETH_PTP_DIAG_DUMP_AXI_A4 (0U)
/* Try disabling/bypassing descriptor/data caching in the DMA AXI4 ACE wrapper.
 * If DMAA4 caches stale descriptor content (e.g., OWN=0) it can suspend with TBU
 * while issuing no AXI reads (CATXBR stays 0). */
#define ETH_PTP_DIAG_FORCE_A4DACR_BYPASS (0U)
#define ETH_PTP_DIAG_DUMP_RIFSC_ATTR_ON_STALL (0U)
#define ETH_PTP_MDIO_SCAN_LOG       (0U)
#define ETH_PTP_MDIO_SCAN_VERBOSE   (0U)
#define ETH_PTP_DISABLE_DCACHE      (0U) /* 1: disable D-Cache for DMA debug */
#define ETH_PTP_CONFIGURE_MPU_ETHRAM (0U) /* 1: set ETHRAM non-cacheable */
#define ETH_PTP_FORCE_EDSE          (0U) /* 1: enable enhanced TX descriptors */
#define ETH_PTP_CLEAN_TX_DESC_AFTER_TX (1U)
#define ETH_PTP_FORCE_DSL_64        (0U) /* 1: force descriptor skip length to 64-bit */
#define ETH_DEBUG_OPEN_GATE         (0U) /* 1: open RISAF gates for DMA debug */
#define ETH_PTP_DMA_DIAG_VERBOSE    (0U) /* 1: print per-copy CSR/CBR1/CSAR/CDAR */
#define ETH_PTP_DMA_DIAG_ENABLE     (0U) /* 1: run GPDMA descriptor diagnostic */
#define ETH_PTP_START_DESC_DUMP     (0U) /* 1: dump CH0 descriptors at ETH start */
#define ETH_PTP_USE_LOCAL_DESC      (0U) /* 1: rebind ETH descriptors to local arrays */

/* Bring-up: keep descriptors (and the small TX test frame) inside ETHRAM (0x341F8000..0x341F8FFF)
 * so we don't depend on broader SRAM security/firewall settings while TX DMA is blocked. */
#define ETH_PTP_USE_ETHRAM_DESC     (0U)
/* ETH DMA still shows CATXBR=0 (no buffer address loaded). To eliminate ETHRAM as a variable,
 * keep the TX test buffer in normal SRAM while debugging descriptor fetch. */
#define ETH_PTP_USE_ETHRAM_TXFRAME  (0U)

/* Secure project: program RISAF2 to allow Secure/Privileged accesses to ETHRAM for CID0..7. */
#define ETH_PTP_SEC_CONFIG_ETHRAM   (0U)

/* Observed at TX stall: RIMC_ATTR2/3 = 0x00000210 (MCID=1, MPRIV=1, MSEC=0).
 * If ETH DMA master is non-secure, it cannot read SEC=1 ETHRAM and will suspend with TBU.
 * Fix: mark the likely ETH DMA master indices (2/3) as Secure. */
#define ETH_PTP_SEC_FIX_ETH_DMA_MASTER (0U)
/* Force CubeMX RIF setting in code when UI checkbox is not editable:
 * ETH1 is RIMU ID 6 on your project. */
#define ETH_PTP_SEC_FORCE_ETH1_RIMU6  (0U)
#define ETH_PTP_SEC_FORCE_ETH1_CID    (0U)
/* Diagnostic: allow a small non-secure AXI SRAM window for ETH DMA descriptor/buffer fetch. */
#define ETH_PTP_DIAG_NS_DMA_WINDOW    (0U)

/* One-shot discriminator: if ETH DMA is still non-secure and blocked from SEC ETHRAM, making ETHRAM non-secure
 * should immediately allow CATXBR to advance. Use only for diagnosis, then turn back off. */
#define ETH_PTP_DIAG_ETHRAM_NONSECURE (0U)

/* Diagnostic: if TX DMA still stalls after forcing a couple of RIMC_ATTR entries secure,
 * sweep MSEC across all RIMC_ATTRx entries one-by-one and see which one unblocks CATXBR.
 * This avoids guesswork about the master-index mapping and converges in a single run. */
#define ETH_PTP_DIAG_SWEEP_RIMC_MSEC (0U)
/* 1: store sweep context in RAM and halt in-place for debugger inspection. */
#define ETH_PTP_DIAG_SWEEP_TRAP_BEGIN (0U)
/* One-shot discriminator: force all RIFSC master attributes secure+privileged,
 * run manual TX probe, then restore originals. */
#define ETH_PTP_DIAG_OPEN_ALL_RIMC_ONCE (0U)
/* Safer discriminator: try only likely ETH/AXI master indices one-by-one. */
#define ETH_PTP_DIAG_TRY_RIMC_SUBSET_ONCE (0U)

/* STM32N6 HAL writes DMACTXDTPR = "next free" descriptor.
 * Some ETH DMA variants expect DMACTXDTPR = "last prepared" descriptor (inclusive).
 * This knob overrides DTPR once after a successful submit to validate tail-pointer semantics. */
#define ETH_PTP_TX_DTPR_POINTS_TO_LAST_PREPARED (1U)
/* One-shot direct descriptor submit (bypass HAL descriptor path) to isolate
 * ETH DMA fetch vs HAL queue management. */
#define ETH_PTP_TX_MANUAL_PROBE      (0U)
/* 1: in manual probe, also test NS alias addresses (0x34xx -> 0x24xx). */
#define ETH_PTP_TX_MANUAL_PROBE_NS_ALIAS (0U)
/* Requested test: force descriptor skip length to 0. */
#define ETH_PTP_FORCE_DSL_0          (1U)
#define ETH_PTP_FOCUS_LOG            (1U) /* 1: verbose focus logs on stall */
#define ETH_PTP_DIAG_DMA_SWR_ONCE    (0U) /* 1: one-shot DMAMR.SWR before Start_IT */
/* 1: print compact stall line only (own/csr/catxdr/catxbr/dlar/dtpr). */
#define ETH_PTP_TX_STALL_MIN_LOG     (1U)

#if (ETH_PTP_DESC_LOG_ONLY != 0U)
#define ETH_PTP_LOG_NON_DESC(...) do { } while (0)
#else
#define ETH_PTP_LOG_NON_DESC(...) printf(__VA_ARGS__)
#endif

/* PTP clock source/divider selection for ETH1 PTP kernel clock. */
#define ETH_PTP_CLK_SOURCE          (RCC_ETH1PTPCLKSOURCE_HCLK)
#define ETH_PTP_CLK_DIVIDER         (1U)
/* Runtime ETH1PTP PeriphCLK reconfiguration hard-faults on this target.
 * Use the clock tree configured during boot/MSP init. */
#define ETH_PTP_SKIP_RUNTIME_PTP_CLKCFG (1U)
#define ETH_PTP_CONFIGURE_CLOCK     (0U) /* 0: use CubeMX default, avoids RCC hardfault */

/* Timestamp configuration */
#define ETH_PTP_TS_ALL_ENABLE       (1U) /* 1: timestamp all frames for bring-up */

/* PHY configuration */
#define ETH_PHY_ADDR                (0U)
#define ETH_PHY_ADDR_AUTO_SCAN      (1U)
#define ETH_PHY_SCAN_TIMEOUT_MS     (2000U)
#define ETH_PHY_RESET_TIMEOUT_MS    (500U)
#define ETH_PHY_AN_TIMEOUT_MS       (5000U)

/* --- External ETH handle ------------------------------------------------- */
extern ETH_HandleTypeDef heth1;

/* --- RX buffer pool ------------------------------------------------------ */
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
static uint8_t s_rx_buf_pool[ETH_PTP_RX_BUF_COUNT][ETH_PTP_RX_BUF_SIZE];
static uint8_t s_rx_buf_in_use[ETH_PTP_RX_BUF_COUNT];
static ETH_BufferTypeDef s_rx_nodes[ETH_PTP_RX_BUF_COUNT];

/* --- TX test frame ------------------------------------------------------- */
/* Keep TX test frame in the same NS window used for ETH DMA bring-up. */
__attribute__((section(".eth_dma_ns_buf"))) __attribute__((aligned(32)))
static uint8_t s_tx_frame[64];

#if (ETH_PTP_USE_LOCAL_DESC != 0U)
#if (ETH_PTP_USE_ETHRAM_DESC != 0U)
__attribute__((section(".ethram")))
#endif
static ETH_DMADescTypeDef s_local_tx_desc[ETH_DMA_TX_CH_CNT][ETH_TX_DESC_CNT] __attribute__((aligned(32)));
#if (ETH_PTP_USE_ETHRAM_DESC != 0U)
__attribute__((section(".ethram")))
#endif
static ETH_DMADescTypeDef s_local_rx_desc[ETH_DMA_RX_CH_CNT][ETH_RX_DESC_CNT] __attribute__((aligned(32)));
#endif

/* --- RTOS sync ----------------------------------------------------------- */
static osSemaphoreId_t s_rx_sem;
static osThreadId_t s_eth_task_handle;

/* --- Timestamp capture --------------------------------------------------- */
static volatile uint8_t s_tx_ts_ready;
static ETH_TimeStampTypeDef s_tx_ts_last;

/* --- Deferred error logging (avoid printf in ISR) ------------------------- */
static volatile uint32_t s_eth_err_code;
static volatile uint32_t s_eth_dma_err;
static volatile uint8_t s_eth_err_pending;

/* Throttle one-time dumps to keep logs readable during bring-up. */
static uint8_t s_tx_fail_dumped;
static uint8_t s_dma_probe_done;
static uint8_t s_tbu_recovered;
static uint8_t s_tdt_diag_done;
static uint8_t s_axi_a4_dumped;
static uint8_t s_manual_probe_done;
static uint8_t s_dma_swr_done;
static uint8_t s_tx_path_log_done;
static uint8_t s_bad_dlar_probe_done;
static uint8_t s_ch_diag_once;
static uint8_t s_ns_alias_force_done;
static uint8_t s_ns_alias_check_done;

typedef struct
{
  uint32_t magic;
  uint32_t call_count;
  uint32_t stage;
  uint32_t dlar;
  uint32_t dtpr;
  uint32_t idx;
  uint32_t attr;
  uint32_t catxbr_pre;
  uint32_t catxbr_post;
  uint32_t csr_pre;
  uint32_t csr_post;
  uint32_t dmadsr_pre;
  uint32_t dmadsr_post;
  uint32_t risaf2_iasr;
  uint32_t risaf2_iaesr;
  uint32_t risaf2_iaddr;
  uint32_t risaf6_iasr;
  uint32_t risaf6_iaesr;
  uint32_t risaf6_iaddr;
} ETH_SweepTrapTypeDef;

__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
static volatile ETH_SweepTrapTypeDef s_sweep_trap;

/* --- RX debug ------------------------------------------------------------- */
static volatile uint32_t s_rx_frame_count;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_rx_alloc_fail;
static volatile uint32_t s_tx_cplt_count;
/* --- PHY address --------------------------------------------------------- */
static uint32_t s_phy_addr = ETH_PHY_ADDR;
static DMA_HandleTypeDef s_dma_diag;

#if (ETH_PTP_TX_MANUAL_PROBE != 0U)
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
static ETH_DMADescTypeDef s_probe_desc_nc;
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
static uint8_t s_probe_frame_nc[64];
__attribute__((section(".ethram"))) __attribute__((aligned(32)))
static ETH_DMADescTypeDef s_probe_desc_er;
__attribute__((section(".ethram"))) __attribute__((aligned(32)))
static uint8_t s_probe_frame_er[64];
#endif

/* --- Link status --------------------------------------------------------- */
static uint32_t s_link_up = 0U;
static uint8_t s_sec_probe_done = 0U;

/* --- Helpers ------------------------------------------------------------- */
static void eth_sec_config_ethram(void);
static void eth_dma_diag_clear_risaf_flags(void);
static HAL_StatusTypeDef eth_dma_diag_copy(const char *tag, uint32_t src, uint32_t dst, uint32_t len, uint32_t *err);
static void eth_tx_manual_probe_once(void);
static void eth_dma_soft_reset_once(void);
static uint32_t eth_to_ns_alias(uint32_t addr);
static void eth_force_ch0_ns_alias(void);
static void eth_force_txq0_normal_mode(void);
static void eth_open_ns_dma_window_once(void);
static void eth_pre_tx_submit_recover_once(void);

/* CubeMX ETH descriptor tables live in main.c. Declare them here for one-time layout sanity checks. */
extern ETH_DMADescTypeDef DMARxDscrTab[ETH_DMA_RX_CH_CNT][ETH_RX_DESC_CNT];
extern ETH_DMADescTypeDef DMATxDscrTab[ETH_DMA_TX_CH_CNT][ETH_TX_DESC_CNT];

static void eth_sec_config_ethram(void)
{
#if (ETH_PTP_SEC_CONFIG_ETHRAM != 0U)
#if defined(RISAF2)
  /* ETHRAM is defined in the linker script as 0x341F8000..0x341F8FFF.
   * Configure RISAF2 REG0 to allow Secure privileged CID0..7 masters to read/write it. */
  RISAF2->IACR = RISAF2->IASR; /* clear latched illegal access flags */

  /* RISAF2 region registers use local-offset addressing inside SRAM2,
   * not full system addresses. ETHRAM is SRAM2 offset 0x000F8000..0x000F8FFF. */
  RISAF2->REG[0].STARTR  = 0x000F8000U;
  RISAF2->REG[0].ENDR    = 0x000F8FFFU;
  /* Allow all master CIDs for this region during bring-up.
   * If ETH DMA is tagged with an unexpected CID, limiting to 0..7 can cause
   * silent read-as-zero behavior and persistent TBU with OWN never consumed. */
  RISAF2->REG[0].CIDCFGR = 0xFFFFFFFFU;
  /* Do not require privileged access here: some bus masters can be treated as unprivileged.
   * Default is Secure-only. For diagnosis, ETH_PTP_DIAG_ETHRAM_NONSECURE can drop SEC to test if ETH DMA is NS. */
#if (ETH_PTP_DIAG_ETHRAM_NONSECURE != 0U)
  RISAF2->REG[0].CFGR = (RISAF_REGx_CFGR_BREN);
#else
  RISAF2->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | RISAF_REGx_CFGR_SEC);
#endif
#endif

#if defined(RISAF6)
  /* TX/RX descriptors and test buffers are currently placed in AXI SRAM (0x34xx...).
   * Keep a permissive bring-up region so ETH DMA CID/SEC tagging does not silently
   * return zeros when reading descriptor words. */
  RISAF6->IACR = RISAF6->IASR;
  RISAF6->REG[0].STARTR  = 0x34000000U;
  RISAF6->REG[0].ENDR    = 0x341FFFFFU;
  RISAF6->REG[0].CIDCFGR = 0xFFFFFFFFU;
#if (ETH_PTP_DIAG_ETHRAM_NONSECURE != 0U)
  RISAF6->REG[0].CFGR = (RISAF_REGx_CFGR_BREN);
#else
  RISAF6->REG[0].CFGR = (RISAF_REGx_CFGR_BREN | RISAF_REGx_CFGR_SEC);
#endif
#endif

#if (ETH_PTP_SEC_FIX_ETH_DMA_MASTER != 0U)
#if defined(RIFSC)
  {
    uint32_t i;
    /* Security-only project policy:
     * force all RIF masters to issue Secure + Privileged transactions.
     * This removes master-index guesswork for ETH DMA path. */
    for (i = 0U; i < 13U; i++)
    {
      SET_BIT(RIFSC->RIMC_ATTRx[i], (RIFSC_RIMC_ATTRx_MSEC | RIFSC_RIMC_ATTRx_MPRIV));
    }
    __DSB();
    ETH_PTP_LOG_NON_DESC("[sec][rifsc] force MSEC|MPRIV on RIMC_ATTR0..12\r\n");
  }
#endif
#endif

#if (ETH_PTP_SEC_FORCE_ETH1_RIMU6 != 0U)
#if defined(RIFSC)
  /* CubeMX shows ETH1 RIMU ID = 6. Force SEC|PRIV and keep CID explicit. */
  MODIFY_REG(RIFSC->RIMC_ATTRx[6],
             (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MSEC | RIFSC_RIMC_ATTRx_MPRIV),
             ((ETH_PTP_SEC_FORCE_ETH1_CID << RIFSC_RIMC_ATTRx_MCID_Pos) |
              RIFSC_RIMC_ATTRx_MSEC |
              RIFSC_RIMC_ATTRx_MPRIV));
  __DSB();
  ETH_PTP_LOG_NON_DESC("[sec][rifsc] force ETH1 RIMU6 attr=0x%08lX (cid=%lu sec=1 priv=1)\r\n",
                       (unsigned long)RIFSC->RIMC_ATTRx[6],
                       (unsigned long)ETH_PTP_SEC_FORCE_ETH1_CID);
#endif
#endif
#endif
}

static uint32_t eth_to_ns_alias(uint32_t addr)
{
  if ((addr & 0xFF000000U) == 0x34000000U)
  {
    return (addr - 0x10000000U);
  }
  return addr;
}

static void eth_force_ch0_ns_alias(void)
{
#if (ETH_PTP_TX_FORCE_NS_ALIAS_ONCE != 0U)
  uint32_t dlar_old = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
  uint32_t dtpr_old = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  uint32_t dlar_ns = eth_to_ns_alias(dlar_old);
  uint32_t dtpr_ns = eth_to_ns_alias(dtpr_old);

  if ((dlar_ns != dlar_old) || (dtpr_ns != dtpr_old))
  {
    CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
    __DSB();
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = dlar_ns;
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_ns;
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
    __DSB();
    ETH_PTP_LOG_NON_DESC("[tx][ns-force] dlar 0x%08lX->0x%08lX dtpr 0x%08lX->0x%08lX\r\n",
                         (unsigned long)dlar_old,
                         (unsigned long)dlar_ns,
                         (unsigned long)dtpr_old,
                         (unsigned long)dtpr_ns);
  }
#endif
}

static void eth_force_txq0_normal_mode(void)
{
#if (ETH_PTP_FORCE_TXQ0_NORMAL_MODE != 0U)
  uint32_t qomr = heth1.Instance->MTL_QUEUE[ETH_DMA_CH0_IDX].MTLTXQOMR;
  uint32_t txqen_normal = (1UL << ETH_MTLTXQxOMR_TXQEN_Pos);
  MODIFY_REG(heth1.Instance->MTL_QUEUE[ETH_DMA_CH0_IDX].MTLTXQOMR, ETH_MTLTXQxOMR_TXQEN, txqen_normal);
  __DSB();
  ETH_PTP_LOG_NON_DESC("[eth][txq0] qomr 0x%08lX->0x%08lX\r\n",
                       (unsigned long)qomr,
                       (unsigned long)heth1.Instance->MTL_QUEUE[ETH_DMA_CH0_IDX].MTLTXQOMR);
#endif
}

static void eth_open_ns_dma_window_once(void)
{
#if (ETH_PTP_DIAG_NS_DMA_WINDOW != 0U)
#if defined(RISAF6)
  static uint8_t done = 0U;
  if (done != 0U)
  {
    return;
  }
  done = 1U;
  {
    uint32_t a = (uint32_t)heth1.Init.TxDesc[ETH_DMA_CH0_IDX];
    uint32_t b = (uint32_t)&s_tx_frame[0];
    uint32_t start = (a < b) ? a : b;
    uint32_t end = (a > b) ? a : b;
    start &= 0xFFFFF000U;
    end = (end + 0xFFFU) & 0xFFFFF000U;
    end += 0xFFFU;

    RISAF6->IACR = RISAF6->IASR;
    RISAF6->REG[1].STARTR = start;
    RISAF6->REG[1].ENDR = end;
    RISAF6->REG[1].CIDCFGR = 0xFFFFFFFFU;
    RISAF6->REG[1].CFGR = RISAF_REGx_CFGR_BREN; /* non-secure allow */
    __DSB();
    ETH_PTP_LOG_NON_DESC("[sec][nswin] RISAF6 REG1 start=0x%08lX end=0x%08lX cfgr=0x%08lX\r\n",
                         (unsigned long)RISAF6->REG[1].STARTR,
                         (unsigned long)RISAF6->REG[1].ENDR,
                         (unsigned long)RISAF6->REG[1].CFGR);
  }
#endif
#endif
}

static void eth_pre_tx_submit_recover_once(void)
{
  static uint8_t done = 0U;
  uint32_t dlar;
  uint32_t dtpr;
  if (done != 0U)
  {
    return;
  }

  dlar = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
  dtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  if ((dlar == 0U) || (dtpr == 0U))
  {
    ETH_PTP_LOG_NON_DESC("[tx][pre-kick] skip (dlar=0x%08lX dtpr=0x%08lX)\r\n",
                         (unsigned long)dlar,
                         (unsigned long)dtpr);
    return;
  }
  done = 1U;

  /* Requested sequence: one SWR, then clear TBU/TPS and rewrite DTPR before first TX submit. */
  eth_dma_soft_reset_once();

  {
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr;
    SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
    __DSB();
    ETH_PTP_LOG_NON_DESC("[tx][pre-kick] csr=0x%08lX dtpr=0x%08lX\r\n",
                         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR,
                         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR);
  }
}

static void eth_dma_soft_reset_once(void)
{
#if (ETH_PTP_DIAG_DMA_SWR_ONCE != 0U)
  if (s_dma_swr_done != 0U)
  {
    return;
  }

  s_dma_swr_done = 1U;

  {
    uint32_t pre = heth1.Instance->DMAMR;
    uint32_t wait_ms = 0U;

    SET_BIT(heth1.Instance->DMAMR, ETH_DMAMR_SWR);
    __DSB();

    while (((heth1.Instance->DMAMR & ETH_DMAMR_SWR) != 0U) && (wait_ms < 10U))
    {
      HAL_Delay(1U);
      wait_ms++;
    }

    ETH_PTP_LOG_NON_DESC("[eth][swr] pre=0x%08lX post=0x%08lX wait_ms=%lu\r\n",
                         (unsigned long)pre,
                         (unsigned long)heth1.Instance->DMAMR,
                         (unsigned long)wait_ms);
  }
#endif
}

static void eth_dump_rifsc_attrs_once(const char *tag)
{
#if (ETH_PTP_DIAG_DUMP_RIFSC_ATTR_ON_STALL != 0U)
#if defined(RIFSC)
  static uint8_t dumped = 0U;
  uint32_t i;
  if (dumped != 0U)
  {
    return;
  }
  dumped = 1U;

  ETH_PTP_LOG_NON_DESC("[sec][%s] RIFSC_RISC_CR=0x%08lX\r\n",
                       tag, (unsigned long)RIFSC->RISC_CR);
  for (i = 0U; i < 13U; i++)
  {
    ETH_PTP_LOG_NON_DESC("[sec][%s] RIMC_ATTR%lu=0x%08lX\r\n",
                         tag,
                         (unsigned long)i,
                         (unsigned long)RIFSC->RIMC_ATTRx[i]);
  }
#else
  (void)tag;
#endif
#else
  (void)tag;
#endif
}

static void eth_diag_sweep_rimc_msec_once(uint32_t dlar, uint32_t dtpr)
{
#if (ETH_PTP_DIAG_SWEEP_RIMC_MSEC != 0U)
#if defined(RIFSC)
  static uint8_t init = 0U;
  static uint8_t done = 0U;
  static uint32_t cur = 0U;
  static uint32_t orig[13];
  uint32_t i;

  if (done != 0U)
  {
    return;
  }

  if (init == 0U)
  {
    for (i = 0U; i < 13U; i++)
    {
      orig[i] = RIFSC->RIMC_ATTRx[i];
    }
    init = 1U;
  }

  s_sweep_trap.magic = 0x53575031U; /* "SWP1" */
  s_sweep_trap.call_count++;
  s_sweep_trap.stage = 1U;           /* entered */
  s_sweep_trap.dlar = dlar;
  s_sweep_trap.dtpr = dtpr;
  s_sweep_trap.idx = cur;
  s_sweep_trap.attr = 0U;
  s_sweep_trap.catxbr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  s_sweep_trap.catxbr_post = s_sweep_trap.catxbr_pre;
  s_sweep_trap.csr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  s_sweep_trap.csr_post = s_sweep_trap.csr_pre;
  s_sweep_trap.dmadsr_pre = heth1.Instance->DMADSR;
  s_sweep_trap.dmadsr_post = s_sweep_trap.dmadsr_pre;
#if defined(RISAF2)
  s_sweep_trap.risaf2_iasr = RISAF2->IASR;
  s_sweep_trap.risaf2_iaesr = RISAF2->IAR[0].IAESR;
  s_sweep_trap.risaf2_iaddr = RISAF2->IAR[0].IADDR;
#else
  s_sweep_trap.risaf2_iasr = 0U;
  s_sweep_trap.risaf2_iaesr = 0U;
  s_sweep_trap.risaf2_iaddr = 0U;
#endif
#if defined(RISAF6)
  s_sweep_trap.risaf6_iasr = RISAF6->IASR;
  s_sweep_trap.risaf6_iaesr = RISAF6->IAR[0].IAESR;
  s_sweep_trap.risaf6_iaddr = RISAF6->IAR[0].IADDR;
#else
  s_sweep_trap.risaf6_iasr = 0U;
  s_sweep_trap.risaf6_iaesr = 0U;
  s_sweep_trap.risaf6_iaddr = 0U;
#endif
  __DSB();

#if (ETH_PTP_DIAG_SWEEP_TRAP_BEGIN != 0U)
  for (;;)
  {
    __NOP();
  }
#endif

  if (cur >= 13U)
  {
    done = 1U;
    s_sweep_trap.stage = 0xEEU; /* no hit in 0..12 */
    return;
  }

  i = cur++;
  s_sweep_trap.stage = 0x10U; /* programming attr */
  s_sweep_trap.idx = i;
  __DSB();

  /* Candidate i only: clear SEC/PRIV then set SEC|PRIV on this index. */
  RIFSC->RIMC_ATTRx[i] = orig[i];
  CLEAR_BIT(RIFSC->RIMC_ATTRx[i], (RIFSC_RIMC_ATTRx_MSEC | RIFSC_RIMC_ATTRx_MPRIV));
  SET_BIT(RIFSC->RIMC_ATTRx[i], (RIFSC_RIMC_ATTRx_MSEC | RIFSC_RIMC_ATTRx_MPRIV));
  __DSB();

  s_sweep_trap.attr = RIFSC->RIMC_ATTRx[i];
  s_sweep_trap.catxbr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  s_sweep_trap.csr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  s_sweep_trap.dmadsr_pre = heth1.Instance->DMADSR;
#if defined(RISAF2)
  s_sweep_trap.risaf2_iasr = RISAF2->IASR;
  s_sweep_trap.risaf2_iaesr = RISAF2->IAR[0].IAESR;
  s_sweep_trap.risaf2_iaddr = RISAF2->IAR[0].IADDR;
#endif
#if defined(RISAF6)
  s_sweep_trap.risaf6_iasr = RISAF6->IASR;
  s_sweep_trap.risaf6_iaesr = RISAF6->IAR[0].IAESR;
  s_sweep_trap.risaf6_iaddr = RISAF6->IAR[0].IADDR;
#endif
  s_sweep_trap.stage = 0x20U; /* kick dma */
  __DSB();

  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr;
  SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
  __DSB();
  HAL_Delay(1U);

  s_sweep_trap.catxbr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  s_sweep_trap.csr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  s_sweep_trap.dmadsr_post = heth1.Instance->DMADSR;
#if defined(RISAF2)
  s_sweep_trap.risaf2_iasr = RISAF2->IASR;
  s_sweep_trap.risaf2_iaesr = RISAF2->IAR[0].IAESR;
  s_sweep_trap.risaf2_iaddr = RISAF2->IAR[0].IADDR;
#endif
#if defined(RISAF6)
  s_sweep_trap.risaf6_iasr = RISAF6->IASR;
  s_sweep_trap.risaf6_iaesr = RISAF6->IAR[0].IAESR;
  s_sweep_trap.risaf6_iaddr = RISAF6->IAR[0].IADDR;
#endif
  s_sweep_trap.stage = 0x30U; /* sample done */
  __DSB();

  if ((s_sweep_trap.catxbr_pre == 0U) && (s_sweep_trap.catxbr_post != 0U))
  {
    done = 1U;
    s_sweep_trap.stage = 0x41U; /* hit */
  }
#else
  (void)dlar;
  (void)dtpr;
#endif
#else
  (void)dlar;
  (void)dtpr;
#endif
}

static void eth_diag_try_rimc_subset_once(uint32_t dlar)
{
#if (ETH_PTP_DIAG_TRY_RIMC_SUBSET_ONCE != 0U)
#if defined(RIFSC)
  static uint8_t done = 0U;
  static const uint8_t idx_list[] = {8U, 9U, 10U, 11U, 12U};
  uint32_t i;

  if (done != 0U)
  {
    return;
  }
  done = 1U;
  ETH_PTP_LOG_NON_DESC("[sec][subset] start idx=8..12\r\n");

  for (i = 0U; i < (uint32_t)(sizeof(idx_list) / sizeof(idx_list[0])); i++)
  {
    uint32_t idx = idx_list[i];
    uint32_t attr_pre = RIFSC->RIMC_ATTRx[idx];
    uint32_t catxbr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;

    /* Apply only on one candidate index (MSEC + MPRIV), keep others unchanged. */
    SET_BIT(RIFSC->RIMC_ATTRx[idx], (RIFSC_RIMC_ATTRx_MSEC | RIFSC_RIMC_ATTRx_MPRIV));
    __DSB();

    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dlar;
    SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
    __DSB();
    HAL_Delay(1U);

    ETH_PTP_LOG_NON_DESC("[sec][subset] idx=%lu attr 0x%08lX->0x%08lX catxbr 0x%08lX->0x%08lX\r\n",
                         (unsigned long)idx,
                         (unsigned long)attr_pre,
                         (unsigned long)RIFSC->RIMC_ATTRx[idx],
                         (unsigned long)catxbr_pre,
                         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR);

    /* Restore candidate before next try. */
    RIFSC->RIMC_ATTRx[idx] = attr_pre;
    __DSB();
  }
#else
  (void)dlar;
#endif
#else
  (void)dlar;
#endif
}

static void eth_dump_ethram_layout_once(void)
{
  static uint8_t done = 0U;
  if (done != 0U)
  {
    return;
  }
  done = 1U;

  ETH_PTP_LOG_NON_DESC("[eth][layout] tx_frame=0x%08lX rx_desc=0x%08lX tx_desc=0x%08lX (heth tx0=0x%08lX rx0=0x%08lX)\r\n",
                       (unsigned long)(uint32_t)&s_tx_frame[0],
                       (unsigned long)(uint32_t)&DMARxDscrTab[0][0],
                       (unsigned long)(uint32_t)&DMATxDscrTab[0][0],
                       (unsigned long)(uint32_t)heth1.Init.TxDesc[ETH_DMA_CH0_IDX],
                       (unsigned long)(uint32_t)heth1.Init.RxDesc[ETH_DMA_CH0_IDX]);
#if defined(RISAF2)
  ETH_PTP_LOG_NON_DESC("[eth][layout] RISAF2 REG0 CFGR=0x%08lX STARTR=0x%08lX ENDR=0x%08lX CIDCFGR=0x%08lX\r\n",
                       (unsigned long)RISAF2->REG[0].CFGR,
                       (unsigned long)RISAF2->REG[0].STARTR,
                       (unsigned long)RISAF2->REG[0].ENDR,
                       (unsigned long)RISAF2->REG[0].CIDCFGR);
#endif
}

static void eth_fixup_desc_format(void)
{
  /* Descriptor skip-length test point:
   * HAL's ETH_DMADescTypeDef is 24 bytes, but for current root-cause narrowing
   * user requested forcing DSL=0. */
  ETH_DMAConfigTypeDef dmacfg = {0};
  uint32_t ch;

  /* Program via HAL so we touch the correct register (DMACCR), not TX/RXCR. */
  if (HAL_ETH_GetDMAConfig(&heth1, &dmacfg) == HAL_OK)
  {
    for (ch = 0U; ch < ETH_DMA_CH_CNT; ch++)
    {
#if (ETH_PTP_FORCE_DSL_0 != 0U)
      dmacfg.DMACh[ch].DescriptorSkipLength = ETH_DMA_DESC_SKIP_LENGTH_0;
#else
      dmacfg.DMACh[ch].DescriptorSkipLength = ETH_DMA_DESC_SKIP_LENGTH_64;
#endif
    }
    if (HAL_ETH_SetDMAConfig(&heth1, &dmacfg) != HAL_OK)
    {
      /* If gState != READY, HAL will refuse to program DMACCR. */
      ETH_PTP_LOG_NON_DESC("[eth][dsl] HAL_ETH_SetDMAConfig refused (gState=%lu)\r\n",
                           (unsigned long)heth1.gState);
    }
    else
    {
#if (ETH_PTP_FORCE_DSL_0 != 0U)
      ETH_PTP_LOG_NON_DESC("[eth][dsl] forced DSL=0\r\n");
#else
      ETH_PTP_LOG_NON_DESC("[eth][dsl] forced DSL=64\r\n");
#endif
    }
  }

  /* Current bring-up path uses extended TX descriptor fields (D0/D2/D3 and doorbell +0x18),
   * so keep EDSE aligned with that format. */
#if (ETH_PTP_FORCE_EDSE != 0U)
  SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_EDSE);
  SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXCR, ETH_DMACxTXCR_EDSE);
#else
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_EDSE);
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXCR, ETH_DMACxTXCR_EDSE);
#endif
}

static void eth_reinit_with_local_desc(void)
{
#if (ETH_PTP_USE_LOCAL_DESC != 0U)
  uint32_t ch;

  memset(s_local_tx_desc, 0, sizeof(s_local_tx_desc));
  memset(s_local_rx_desc, 0, sizeof(s_local_rx_desc));

  for (ch = 0U; ch < ETH_DMA_TX_CH_CNT; ch++)
  {
    heth1.Init.TxDesc[ch] = &s_local_tx_desc[ch][0];
  }
  for (ch = 0U; ch < ETH_DMA_RX_CH_CNT; ch++)
  {
    heth1.Init.RxDesc[ch] = &s_local_rx_desc[ch][0];
  }

  (void)HAL_ETH_DeInit(&heth1);
  if (HAL_ETH_Init(&heth1) != HAL_OK)
  {
    Error_Handler();
  }

  ETH_PTP_LOG_NON_DESC("[eth][desc] local tx0=0x%08lX rx0=0x%08lX\r\n",
                       (unsigned long)heth1.Init.TxDesc[ETH_DMA_CH0_IDX],
                       (unsigned long)heth1.Init.RxDesc[ETH_DMA_CH0_IDX]);
#endif
  /* Disabled path: ETH re-init from task context can hardfault on RCC clock config. */
}

static void eth_ptp_mpu_config_ethram(void)
{
#if defined(MPU_REGION_NUMBER7)
  MPU_Attributes_InitTypeDef attr = {0};
  MPU_Region_InitTypeDef mpu = {0};

  HAL_MPU_Disable();

  attr.Number = MPU_ATTRIBUTES_NUMBER0;
  attr.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);
  HAL_MPU_ConfigMemoryAttributes(&attr);

  mpu.Enable = MPU_REGION_ENABLE;
  mpu.Number = MPU_REGION_NUMBER7;
  mpu.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  mpu.BaseAddress = 0x341F8000U;
  mpu.LimitAddress = 0x341F8FFFU;
  mpu.AccessPermission = MPU_REGION_ALL_RW;
  mpu.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  mpu.DisablePrivExec = MPU_PRIV_INSTRUCTION_ACCESS_DISABLE;
  mpu.IsShareable = MPU_ACCESS_INNER_SHAREABLE;
  HAL_MPU_ConfigRegion(&mpu);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif
}

static void eth_cache_clean(const void *addr, uint32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (len == 0U)
  {
    return;
  }
  uintptr_t start = (uintptr_t)addr & ~(uintptr_t)0x1FU;
  uintptr_t end = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)0x1FU;
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

static void eth_cache_invalidate(const void *addr, uint32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (len == 0U)
  {
    return;
  }
  uintptr_t start = (uintptr_t)addr & ~(uintptr_t)0x1FU;
  uintptr_t end = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)0x1FU;
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

static void eth_dump_tx_min_regs(void)
{
  uint32_t dmacsr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  uint32_t txcr   = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR;
  uint32_t mtlomr = heth1.Instance->MTL_QUEUE[0].MTLTXQOMR;
  uint32_t maccr  = heth1.Instance->MACCR;
  uint32_t txdlar = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
  uint32_t txdtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  uint32_t txrlr  = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR;
  uint32_t catxdr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR;
  uint32_t catxbr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  uint32_t ahb5enr = RCC->AHB5ENR;

  ETH_PTP_LOG_NON_DESC("[tx][reg] DMACSR0=0x%08lX DMACTXCR=0x%08lX MTLTXQOMR=0x%08lX MACCR=0x%08lX\r\n",
                       (unsigned long)dmacsr,
                       (unsigned long)txcr,
                       (unsigned long)mtlomr,
                       (unsigned long)maccr);
  ETH_PTP_LOG_NON_DESC("[tx][reg] TXDLAR=0x%08lX TXDTPR=0x%08lX TXRLR=0x%08lX\r\n",
                       (unsigned long)txdlar,
                       (unsigned long)txdtpr,
                       (unsigned long)txrlr);
  ETH_PTP_LOG_NON_DESC("[tx][reg] CATXDR=0x%08lX CATXBR=0x%08lX AHB5ENR=0x%08lX\r\n",
                       (unsigned long)catxdr,
                       (unsigned long)catxbr,
                       (unsigned long)ahb5enr);
  ETH_PTP_LOG_NON_DESC("[tx][reg1] DMACSR1=0x%08lX DMACTXCR1=0x%08lX MTLTXQOMR1=0x%08lX\r\n",
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACSR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXCR,
                       (unsigned long)heth1.Instance->MTL_QUEUE[1].MTLTXQOMR);
  ETH_PTP_LOG_NON_DESC("[tx][reg1] CATXDR1=0x%08lX TXDTPR1=0x%08lX\r\n",
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACCATXDR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXDTPR);
}

static void eth_phy_dump_an_once(void)
{
  uint32_t bmcr = 0U;
  uint32_t bmsr = 0U;
  uint32_t anar = 0U;
  uint32_t anlpar = 0U;
  uint32_t gctl = 0U;
  uint32_t gstat = 0U;

  /* Read standard MII/GMII regs. Keep it raw (hex) to avoid wrong decode. */
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x00U, &bmcr);
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x01U, &bmsr);
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x04U, &anar);
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x05U, &anlpar);
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x09U, &gctl);
  (void)HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x0AU, &gstat);

  ETH_PTP_LOG_NON_DESC("[phy][an] bmcr=0x%04lX bmsr=0x%04lX anar=0x%04lX anlpar=0x%04lX 1000ctl=0x%04lX 1000stat=0x%04lX\r\n",
                       (unsigned long)bmcr,
                       (unsigned long)bmsr,
                       (unsigned long)anar,
                       (unsigned long)anlpar,
                       (unsigned long)gctl,
                       (unsigned long)gstat);
}

static void eth_dump_eth1_clk_once(void)
{
  uint32_t ccipr2 = RCC->CCIPR2;
  uint32_t ahb5enr = RCC->AHB5ENR;
  uint32_t ahb5ensr = RCC->AHB5ENSR;
  uint32_t ahb5rstsr = RCC->AHB5RSTSR;
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  uint32_t eth1_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ETH1);

  ETH_PTP_LOG_NON_DESC("[eth][clk] CCIPR2=0x%08lX ETH1CLKSEL=%lu GTXSEL=%lu REFSEL=%lu ETH1SEL=%lu PTPSEL=%lu PTPDIV=%lu AHB5ENR=0x%08lX AHB5ENSR=0x%08lX AHB5RSTSR=0x%08lX HCLK=%lu ETH1=%lu\r\n",
                       (unsigned long)ccipr2,
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1CLKSEL_Msk) >> RCC_CCIPR2_ETH1CLKSEL_Pos),
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1GTXCLKSEL_Msk) ? 1U : 0U),
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1REFCLKSEL_Msk) ? 1U : 0U),
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1SEL_Msk) >> RCC_CCIPR2_ETH1SEL_Pos),
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1PTPSEL_Msk) >> RCC_CCIPR2_ETH1PTPSEL_Pos),
                       (unsigned long)((ccipr2 & RCC_CCIPR2_ETH1PTPDIV_Msk) >> RCC_CCIPR2_ETH1PTPDIV_Pos),
                       (unsigned long)ahb5enr,
                       (unsigned long)ahb5ensr,
                       (unsigned long)ahb5rstsr,
                       (unsigned long)hclk_hz,
                       (unsigned long)eth1_hz);
}

static void eth_dma_probe_desc_access_once(uint32_t desc_addr)
{
  /* Use GPDMA SW request as a proxy to test if bus masters can read the descriptor memory.
   * If secure read works but non-secure read returns zeros, ETH DMA might be non-secure
   * (or the SRAM MPC requires secure), explaining why OWN never gets consumed.
   */
  HAL_StatusTypeDef st;
  DMA_InitTypeDef init = {0};
  uint32_t attrs = 0U;
  uint32_t err = 0U;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef tmp;

  if (s_dma_probe_done != 0U)
  {
    return;
  }
  s_dma_probe_done = 1U;

  memset(&s_dma_diag, 0, sizeof(s_dma_diag));
  s_dma_diag.Instance = GPDMA1_Channel5;
  init.Request = DMA_REQUEST_SW;
  init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  init.Direction = DMA_MEMORY_TO_MEMORY;
  init.SrcInc = DMA_SINC_INCREMENTED;
  init.DestInc = DMA_DINC_INCREMENTED;
  init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  init.Priority = DMA_HIGH_PRIORITY;
  init.SrcBurstLength = 1U;
  init.DestBurstLength = 1U;
  init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  init.Mode = DMA_NORMAL;
  s_dma_diag.Init = init;

  (void)HAL_DMA_DeInit(&s_dma_diag);
  st = HAL_DMA_Init(&s_dma_diag);
  if (st != HAL_OK)
  {
    ETH_PTP_LOG_NON_DESC("[dma-probe] init fail st=%d err=0x%08lX\r\n", (int)st, (unsigned long)HAL_DMA_GetError(&s_dma_diag));
    return;
  }

  /* Secure read */
  (void)HAL_DMA_ConfigChannelAttributes(&s_dma_diag,
                                       DMA_CHANNEL_PRIV | DMA_CHANNEL_SEC |
                                       DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC);
  (void)HAL_DMA_GetConfigChannelAttributes(&s_dma_diag, &attrs);
  memset(&tmp, 0, sizeof(tmp));
  st = eth_dma_diag_copy("probe-s", desc_addr, (uint32_t)&tmp, sizeof(tmp), &err);
  ETH_PTP_LOG_NON_DESC("[dma-probe][S] st=%d attr=0x%08lX err=0x%08lX d3=0x%08lX\r\n",
                       (int)st, (unsigned long)attrs, (unsigned long)err, (unsigned long)tmp.DESC3);

  /* Non-secure read */
  (void)HAL_DMA_ConfigChannelAttributes(&s_dma_diag, 0U);
  (void)HAL_DMA_GetConfigChannelAttributes(&s_dma_diag, &attrs);
  memset(&tmp, 0, sizeof(tmp));
  st = eth_dma_diag_copy("probe-ns", desc_addr, (uint32_t)&tmp, sizeof(tmp), &err);
  ETH_PTP_LOG_NON_DESC("[dma-probe][NS] st=%d attr=0x%08lX err=0x%08lX d3=0x%08lX\r\n",
                       (int)st, (unsigned long)attrs, (unsigned long)err, (unsigned long)tmp.DESC3);
}

static void eth_dump_risaf_fault_snapshot(void)
{
  uint32_t iaesr2 = RISAF2->IAR[0].IAESR;
  uint32_t iaesr6 = RISAF6->IAR[0].IAESR;

  ETH_PTP_LOG_NON_DESC("[tx][sec] RISAF2 IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX CID=%lu SEC=%lu PRIV=%lu NRW=%lu\r\n",
                       (unsigned long)RISAF2->IASR,
                       (unsigned long)iaesr2,
                       (unsigned long)RISAF2->IAR[0].IADDR,
                       (unsigned long)((iaesr2 & RISAF_IAESR_IACID_Msk) >> RISAF_IAESR_IACID_Pos),
                       (unsigned long)((iaesr2 & RISAF_IAESR_IASEC_Msk) ? 1U : 0U),
                       (unsigned long)((iaesr2 & RISAF_IAESR_IAPRIV_Msk) ? 1U : 0U),
                       (unsigned long)((iaesr2 & RISAF_IAESR_IANRW_Msk) ? 1U : 0U));
  ETH_PTP_LOG_NON_DESC("[tx][sec] RISAF6 IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX CID=%lu SEC=%lu PRIV=%lu NRW=%lu\r\n",
                       (unsigned long)RISAF6->IASR,
                       (unsigned long)iaesr6,
                       (unsigned long)RISAF6->IAR[0].IADDR,
                       (unsigned long)((iaesr6 & RISAF_IAESR_IACID_Msk) >> RISAF_IAESR_IACID_Pos),
                       (unsigned long)((iaesr6 & RISAF_IAESR_IASEC_Msk) ? 1U : 0U),
                       (unsigned long)((iaesr6 & RISAF_IAESR_IAPRIV_Msk) ? 1U : 0U),
                       (unsigned long)((iaesr6 & RISAF_IAESR_IANRW_Msk) ? 1U : 0U));
}

static void eth_force_recover_tx_ch0(void)
{
  uint32_t i;
  ETH_DMADescTypeDef *txd;

  if (heth1.Init.TxDesc[ETH_DMA_CH0_IDX] == NULL)
  {
    return;
  }

  for (i = 0U; i < ETH_TX_DESC_CNT; i++)
  {
    txd = &heth1.Init.TxDesc[ETH_DMA_CH0_IDX][i];
    txd->DESC3 = (ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD);
    heth1.TxDescList[ETH_DMA_CH0_IDX].PacketAddress[i] = NULL;
  }

  heth1.TxDescList[ETH_DMA_CH0_IDX].CurTxDesc = 0U;
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR =
    (uint32_t)heth1.TxDescList[ETH_DMA_CH0_IDX].TxDesc[0];
  SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
}

static void eth_kick_tx_ch0_pre(void)
{
  uint32_t clr;

  /* Clear sticky TX status bits before a new kick. */
  clr = ETH_DMACxSR_TBU | ETH_DMACxSR_TPS | ETH_DMACxSR_TI | ETH_DMACxSR_ETI |
        ETH_DMACxSR_FBE | ETH_DMACxSR_AIS | ETH_DMACxSR_NIS;
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = clr;

  /* Ensure TX state machine is enabled. */
  SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);

  /* Re-issue doorbell to current descriptor pointer once before HAL submit. */
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR =
    (uint32_t)heth1.TxDescList[ETH_DMA_CH0_IDX].TxDesc[heth1.TxDescList[ETH_DMA_CH0_IDX].CurTxDesc];
}

static void eth_dump_desc_ch0_once(const char *tag)
{
  ETH_DMADescTypeDef *tx0;
  ETH_DMADescTypeDef *rx0;
  uint32_t dmacsr0;
  uint32_t txdtpr;
  uint32_t rxdtpr;

  if ((heth1.Init.TxDesc[ETH_DMA_CH0_IDX] == NULL) || (heth1.Init.RxDesc[ETH_DMA_CH0_IDX] == NULL))
  {
    return;
  }

  tx0 = &heth1.Init.TxDesc[ETH_DMA_CH0_IDX][0];
  rx0 = &heth1.Init.RxDesc[ETH_DMA_CH0_IDX][0];
  dmacsr0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  txdtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  rxdtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACRXDTPR;

  printf("[eth][%s] DMACSR0=0x%08lX TXDTPR=0x%08lX RXDTPR=0x%08lX\r\n",
         tag,
         (unsigned long)dmacsr0,
         (unsigned long)txdtpr,
         (unsigned long)rxdtpr);
  printf("[eth][%s] TXD0=0x%08lX TXD1=0x%08lX TXD2=0x%08lX TXD3=0x%08lX\r\n",
         tag,
         (unsigned long)tx0->DESC0,
         (unsigned long)tx0->DESC1,
         (unsigned long)tx0->DESC2,
         (unsigned long)tx0->DESC3);
  printf("[eth][%s] RXD0=0x%08lX RXD1=0x%08lX RXD2=0x%08lX RXD3=0x%08lX\r\n",
         tag,
         (unsigned long)rx0->DESC0,
         (unsigned long)rx0->DESC1,
         (unsigned long)rx0->DESC2,
         (unsigned long)rx0->DESC3);
}

static void eth_probe_sec_fault_once(void)
{
  if (s_sec_probe_done != 0U)
  {
    return;
  }
  s_sec_probe_done = 1U;

#if defined(RIFSC)
  ETH_PTP_LOG_NON_DESC("[sec][ppsr] 0=0x%08lX 1=0x%08lX 2=0x%08lX 3=0x%08lX 4=0x%08lX 5=0x%08lX\r\n",
                       (unsigned long)RIFSC->PPSRx[0],
                       (unsigned long)RIFSC->PPSRx[1],
                       (unsigned long)RIFSC->PPSRx[2],
                       (unsigned long)RIFSC->PPSRx[3],
                       (unsigned long)RIFSC->PPSRx[4],
                       (unsigned long)RIFSC->PPSRx[5]);
#endif

#if defined(RISAF2)
  if (RISAF2->IASR != 0U)
  {
    ETH_PTP_LOG_NON_DESC("[sec][risaf2] IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX\r\n",
                         (unsigned long)RISAF2->IASR,
                         (unsigned long)RISAF2->IAR[0].IAESR,
                         (unsigned long)RISAF2->IAR[0].IADDR);
    RISAF2->IACR = RISAF2->IASR;
  }
#endif

#if defined(RISAF6)
  if (RISAF6->IASR != 0U)
  {
    ETH_PTP_LOG_NON_DESC("[sec][risaf6] IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX\r\n",
                         (unsigned long)RISAF6->IASR,
                         (unsigned long)RISAF6->IAR[0].IAESR,
                         (unsigned long)RISAF6->IAR[0].IADDR);
    RISAF6->IACR = RISAF6->IASR;
  }
#endif
}

static void eth_debug_open_gate(void)
{
#if (ETH_DEBUG_OPEN_GATE != 0U)
#if defined(RISAF2)
  /* Debug only: open full region to remove RISAF block effects. */
  RISAF2->REG[0].CFGR = 0x00FF0101U;
  RISAF2->REG[0].STARTR = 0x341F8000U;
  RISAF2->REG[0].ENDR = 0x341F8FFFU;
  RISAF2->REG[0].CIDCFGR = 0x00FF00FFU;
  RISAF2->IACR = RISAF2->IASR;
  printf("[sec][gate] RISAF2 open CFGR=0x%08lX STARTR=0x%08lX ENDR=0x%08lX\r\n",
         (unsigned long)RISAF2->REG[0].CFGR,
         (unsigned long)RISAF2->REG[0].STARTR,
         (unsigned long)RISAF2->REG[0].ENDR);
#endif
#if defined(RISAF6)
  RISAF6->REG[0].CFGR = 0x00FF0101U;
  RISAF6->REG[0].STARTR = 0x34000000U;
  RISAF6->REG[0].ENDR = 0x341FFFFFU;
  RISAF6->REG[0].CIDCFGR = 0x00FF00FFU;
  RISAF6->IACR = RISAF6->IASR;
  printf("[sec][gate] RISAF6 open CFGR=0x%08lX STARTR=0x%08lX ENDR=0x%08lX\r\n",
         (unsigned long)RISAF6->REG[0].CFGR,
         (unsigned long)RISAF6->REG[0].STARTR,
         (unsigned long)RISAF6->REG[0].ENDR);
#endif
#endif
}

static void eth_dma_diag_dump_ppsr_delta(const char *tag, const uint32_t pre[6], const uint32_t post[6])
{
#if defined(RIFSC)
  uint32_t i;
  uint8_t any = 0U;
  for (i = 0U; i < 6U; i++)
  {
    uint32_t delta = pre[i] ^ post[i];
    if (delta != 0U)
    {
      any = 1U;
      printf("[dma-diag][%s] ppsr%lu pre=0x%08lX post=0x%08lX delta=0x%08lX\r\n",
             tag,
             (unsigned long)i,
             (unsigned long)pre[i],
             (unsigned long)post[i],
             (unsigned long)delta);
    }
  }
  if (any == 0U)
  {
    printf("[dma-diag][%s] ppsr delta=none\r\n", tag);
  }
#else
  (void)tag;
  (void)pre;
  (void)post;
#endif
}

static HAL_StatusTypeDef eth_dma_diag_copy(const char *tag, uint32_t src, uint32_t dst, uint32_t len, uint32_t *err)
{
  HAL_StatusTypeDef st;
  uint32_t err_flags = (DMA_FLAG_DTE | DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_TO);
  uint32_t cbr1_pre = 0U;
  uint32_t csar_pre = 0U;
  uint32_t cdar_pre = 0U;
  uint32_t cbr1_post = 0U;
  uint32_t csar_post = 0U;
  uint32_t cdar_post = 0U;
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  uint32_t ppsr_pre[6] = {0U};
  uint32_t ppsr_post[6] = {0U};
#endif
  uint32_t iaesr;
  uint32_t iasr;
  uint32_t iaddr;

  cbr1_pre = s_dma_diag.Instance->CBR1;
  csar_pre = s_dma_diag.Instance->CSAR;
  cdar_pre = s_dma_diag.Instance->CDAR;
#if defined(RIFSC)
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  ppsr_pre[0] = RIFSC->PPSRx[0];
  ppsr_pre[1] = RIFSC->PPSRx[1];
  ppsr_pre[2] = RIFSC->PPSRx[2];
  ppsr_pre[3] = RIFSC->PPSRx[3];
  ppsr_pre[4] = RIFSC->PPSRx[4];
  ppsr_pre[5] = RIFSC->PPSRx[5];
#endif
#endif
  eth_dma_diag_clear_risaf_flags();
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  printf("[dma-diag][copy-pre] CSR=0x%08lX CBR1=0x%08lX CSAR=0x%08lX CDAR=0x%08lX len=%lu\r\n",
         (unsigned long)s_dma_diag.Instance->CSR,
         (unsigned long)cbr1_pre,
         (unsigned long)csar_pre,
         (unsigned long)cdar_pre,
         (unsigned long)len);
#endif

  __HAL_DMA_CLEAR_FLAG(&s_dma_diag, DMA_FLAG_IDLE | DMA_FLAG_TC | DMA_FLAG_HT | err_flags);
  st = HAL_DMA_Start(&s_dma_diag, src, dst, len);
  if (st != HAL_OK)
  {
    if (err != NULL)
    {
      *err = HAL_DMA_GetError(&s_dma_diag);
    }
    return st;
  }
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  printf("[dma-diag][bndt] req_len=%lu CBR1=0x%08lX\r\n",
         (unsigned long)len,
         (unsigned long)s_dma_diag.Instance->CBR1);
#endif
  st = HAL_DMA_PollForTransfer(&s_dma_diag, HAL_DMA_FULL_TRANSFER, 100U);
  if (err != NULL)
  {
    *err = ((uint32_t)HAL_DMA_GetError(&s_dma_diag) << 16) | (s_dma_diag.Instance->CSR & 0xFFFFU);
  }
#if defined(RISAF2)
  iasr = RISAF2->IASR;
  if (iasr != 0U)
  {
    iaesr = RISAF2->IAR[0].IAESR;
    iaddr = RISAF2->IAR[0].IADDR;
    printf("[dma-diag][%s] risaf2 IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX CID=%lu SEC=%lu PRIV=%lu NRW=%lu\r\n",
           tag,
           (unsigned long)iasr,
           (unsigned long)iaesr,
           (unsigned long)iaddr,
           (unsigned long)((iaesr & RISAF_IAESR_IACID_Msk) >> RISAF_IAESR_IACID_Pos),
           (unsigned long)((iaesr & RISAF_IAESR_IASEC) ? 1U : 0U),
           (unsigned long)((iaesr & RISAF_IAESR_IAPRIV) ? 1U : 0U),
           (unsigned long)((iaesr & RISAF_IAESR_IANRW) ? 1U : 0U));
    /* Clear latched flags so the next copy can be attributed cleanly. */
    RISAF2->IACR = (RISAF_IACR_CAEF | RISAF_IACR_IAEF);
  }
#endif
#if defined(RISAF6)
  iasr = RISAF6->IASR;
  if (iasr != 0U)
  {
    iaesr = RISAF6->IAR[0].IAESR;
    iaddr = RISAF6->IAR[0].IADDR;
    printf("[dma-diag][%s] risaf6 IASR=0x%08lX IAESR=0x%08lX IADDR=0x%08lX CID=%lu SEC=%lu PRIV=%lu NRW=%lu\r\n",
           tag,
           (unsigned long)iasr,
           (unsigned long)iaesr,
           (unsigned long)iaddr,
           (unsigned long)((iaesr & RISAF_IAESR_IACID_Msk) >> RISAF_IAESR_IACID_Pos),
           (unsigned long)((iaesr & RISAF_IAESR_IASEC) ? 1U : 0U),
           (unsigned long)((iaesr & RISAF_IAESR_IAPRIV) ? 1U : 0U),
           (unsigned long)((iaesr & RISAF_IAESR_IANRW) ? 1U : 0U));
    RISAF6->IACR = (RISAF_IACR_CAEF | RISAF_IACR_IAEF);
  }
#endif
#if defined(RIFSC)
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  ppsr_post[0] = RIFSC->PPSRx[0];
  ppsr_post[1] = RIFSC->PPSRx[1];
  ppsr_post[2] = RIFSC->PPSRx[2];
  ppsr_post[3] = RIFSC->PPSRx[3];
  ppsr_post[4] = RIFSC->PPSRx[4];
  ppsr_post[5] = RIFSC->PPSRx[5];
  eth_dma_diag_dump_ppsr_delta(tag, ppsr_pre, ppsr_post);
#endif
#endif

  cbr1_post = s_dma_diag.Instance->CBR1;
  csar_post = s_dma_diag.Instance->CSAR;
  cdar_post = s_dma_diag.Instance->CDAR;
#if (ETH_PTP_DMA_DIAG_VERBOSE != 0U)
  printf("[dma-diag][copy-post] CSR=0x%08lX CBR1=0x%08lX CSAR=0x%08lX CDAR=0x%08lX\r\n",
         (unsigned long)s_dma_diag.Instance->CSR,
         (unsigned long)cbr1_post,
         (unsigned long)csar_post,
         (unsigned long)cdar_post);
#endif
  return st;
}

static void eth_dma_diag_clear_risaf_flags(void)
{
#if defined(RISAF2)
  RISAF2->IACR = (RISAF_IACR_CAEF | RISAF_IACR_IAEF);
#endif
#if defined(RISAF6)
  RISAF6->IACR = (RISAF_IACR_CAEF | RISAF_IACR_IAEF);
#endif
}

static HAL_StatusTypeDef eth_dma_diag_dma_init(void)
{
  HAL_StatusTypeDef st;
  uint32_t attrs = 0U;

  (void)HAL_DMA_DeInit(&s_dma_diag);
  st = HAL_DMA_Init(&s_dma_diag);
  if (st != HAL_OK)
  {
    return st;
  }

  /* Configure DMA channel secure/privilege attributes for secure-only domain. */
  st = HAL_DMA_ConfigChannelAttributes(&s_dma_diag,
                                       DMA_CHANNEL_PRIV | DMA_CHANNEL_SEC |
                                       DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC);
  if (st != HAL_OK)
  {
    return st;
  }

  st = HAL_DMA_GetConfigChannelAttributes(&s_dma_diag, &attrs);
  if (st == HAL_OK)
  {
    printf("[dma-diag][attr] 0x%08lX\r\n", (unsigned long)attrs);
  }
  return st;
}

static void eth_dma_diag_run(void)
{
  HAL_StatusTypeDef st;
  DMA_InitTypeDef init = {0};
  uint32_t err = 0U;
  int cmp = 0;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static uint32_t diag_src[16];
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static uint32_t diag_dst[16];
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static uint32_t diag_src_zero[16];
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static uint32_t diag_dst_mark[16];
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef txd_pat;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef rxd_pat;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef txd_tmp;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef rxd_tmp;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef txd_backup;
  __attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32))) static ETH_DMADescTypeDef rxd_backup;
  ETH_DMADescTypeDef *txd0;
  ETH_DMADescTypeDef *rxd0;

  txd0 = (heth1.Init.TxDesc[ETH_DMA_CH0_IDX] != NULL) ? &heth1.Init.TxDesc[ETH_DMA_CH0_IDX][0] : NULL;
  if (txd0 == NULL)
  {
    printf("[dma-diag] skip: tx descriptor not initialized\r\n");
    return;
  }
  rxd0 = (heth1.Init.RxDesc[ETH_DMA_CH0_IDX] != NULL) ? &heth1.Init.RxDesc[ETH_DMA_CH0_IDX][0] : NULL;
  if (rxd0 == NULL)
  {
    printf("[dma-diag] skip: rx descriptor not initialized\r\n");
    return;
  }
  printf("[dma-diag][addr] txd0=0x%08lX rxd0=0x%08lX\r\n",
         (unsigned long)txd0, (unsigned long)rxd0);
  printf("[dma-diag][addr] TXDLAR=0x%08lX RXDLAR=0x%08lX TXDTPR=0x%08lX RXDTPR=0x%08lX\r\n",
         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR,
         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACRXDLAR,
         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR,
         (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACRXDTPR);

  memset(&s_dma_diag, 0, sizeof(s_dma_diag));
  s_dma_diag.Instance = GPDMA1_Channel5;
  init.Request = DMA_REQUEST_SW;
  init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  init.Direction = DMA_MEMORY_TO_MEMORY;
  init.SrcInc = DMA_SINC_INCREMENTED;
  init.DestInc = DMA_DINC_INCREMENTED;
  init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  init.Priority = DMA_HIGH_PRIORITY;
  init.SrcBurstLength = 1U;
  init.DestBurstLength = 1U;
  init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  init.Mode = DMA_NORMAL;
  s_dma_diag.Init = init;

  st = eth_dma_diag_dma_init();
  if (st != HAL_OK)
  {
    printf("[dma-diag] init fail st=%d err=0x%08lX\r\n", (int)st, (unsigned long)HAL_DMA_GetError(&s_dma_diag));
    return;
  }

  /* Baseline: noncacheable buffer to noncacheable buffer copy. */
  for (uint32_t i = 0U; i < 16U; i++)
  {
    diag_src[i] = 0xA5000000U + i;
    diag_dst[i] = 0U;
  }
  eth_cache_clean(diag_src, sizeof(diag_src));
  eth_cache_clean(diag_dst, sizeof(diag_dst));
  eth_cache_invalidate(diag_dst, sizeof(diag_dst));
  st = eth_dma_diag_copy("buf", (uint32_t)diag_src, (uint32_t)diag_dst, sizeof(diag_src), &err);
  eth_cache_invalidate(diag_dst, sizeof(diag_dst));
  cmp = memcmp(diag_src, diag_dst, sizeof(diag_src));
  printf("[dma-diag][buf] st=%d err=0x%08lX cmp=%d src0=0x%08lX dst0=0x%08lX\r\n",
         (int)st, (unsigned long)err, cmp,
         (unsigned long)diag_src[0], (unsigned long)diag_dst[0]);

  /* Read/Write path split test:
   * src is forced zero, dst is forced marker.
   * If dst stays marker -> DMA write path blocked.
   * If dst becomes zero   -> DMA write path works, source read path may be issue in other cases.
   */
  for (uint32_t i = 0U; i < 16U; i++)
  {
    diag_src_zero[i] = 0x00000000U;
    diag_dst_mark[i] = 0xDEADBEEFU;
  }
  eth_cache_clean(diag_src_zero, sizeof(diag_src_zero));
  eth_cache_clean(diag_dst_mark, sizeof(diag_dst_mark));
  eth_cache_invalidate(diag_dst_mark, sizeof(diag_dst_mark));
  st = eth_dma_diag_copy("rw-split", (uint32_t)diag_src_zero, (uint32_t)diag_dst_mark, sizeof(diag_src_zero), &err);
  eth_cache_invalidate(diag_dst_mark, sizeof(diag_dst_mark));
  printf("[dma-diag][rw-split] st=%d err=0x%08lX src0=0x%08lX dst0=0x%08lX dst1=0x%08lX\r\n",
         (int)st, (unsigned long)err,
         (unsigned long)diag_src_zero[0],
         (unsigned long)diag_dst_mark[0],
         (unsigned long)diag_dst_mark[1]);

  /* Test A: DMA reads ETH TX descriptor area into local tmp buffer. */
  memcpy(&txd_backup, txd0, sizeof(txd_backup));
  memcpy(&rxd_backup, rxd0, sizeof(rxd_backup));
  /* Inject known non-zero CPU patterns before DMA read tests. */
  txd0->DESC0 = 0x11223344U;
  txd0->DESC1 = 0x55667788U;
  txd0->DESC2 = 0x99AABBCCU;
  txd0->DESC3 = 0xDDEEFF00U;
  rxd0->DESC0 = 0x0A0B0C0DU;
  rxd0->DESC1 = 0x10203040U;
  rxd0->DESC2 = 0x50607080U;
  rxd0->DESC3 = 0x90A0B0C0U;
  eth_cache_clean(txd0, sizeof(*txd0));
  eth_cache_clean(rxd0, sizeof(*rxd0));
  eth_cache_invalidate(txd0, sizeof(*txd0));
  eth_cache_invalidate(rxd0, sizeof(*rxd0));
  memset(&txd_tmp, 0, sizeof(txd_tmp));
  eth_cache_invalidate(&txd_tmp, sizeof(txd_tmp));
  st = eth_dma_diag_copy("read", (uint32_t)txd0, (uint32_t)&txd_tmp, sizeof(txd_tmp), &err);
  eth_cache_invalidate(&txd_tmp, sizeof(txd_tmp));
  printf("[dma-diag][read] st=%d err=0x%08lX d0=0x%08lX d3=0x%08lX\r\n",
         (int)st, (unsigned long)err,
         (unsigned long)txd_tmp.DESC0, (unsigned long)txd_tmp.DESC3);

  /* Read RX descriptor through DMA as a second point (usually non-zero). */
  (void)HAL_DMA_Abort(&s_dma_diag);
  st = eth_dma_diag_dma_init();
  if (st != HAL_OK)
  {
    printf("[dma-diag] reinit(read-rx) fail st=%d err=0x%08lX\r\n",
           (int)st, (unsigned long)HAL_DMA_GetError(&s_dma_diag));
    goto diag_restore;
  }
  memset(&rxd_tmp, 0, sizeof(rxd_tmp));
  st = eth_dma_diag_copy("read-rx", (uint32_t)rxd0, (uint32_t)&rxd_tmp, sizeof(rxd_tmp), &err);
  eth_cache_invalidate(&rxd_tmp, sizeof(rxd_tmp));
  printf("[dma-diag][read-rx] st=%d err=0x%08lX d0=0x%08lX d3=0x%08lX\r\n",
         (int)st, (unsigned long)err,
         (unsigned long)rxd_tmp.DESC0, (unsigned long)rxd_tmp.DESC3);

  /* Test B: DMA writes a pattern into ETH TX descriptor area, then restore. */
  txd_pat.DESC0 = 0xA5A5A5A5U;
  txd_pat.DESC1 = 0x5A5A5A5AU;
  txd_pat.DESC2 = 0x12345678U;
  txd_pat.DESC3 = 0x87654321U;
  eth_cache_clean(&txd_pat, sizeof(txd_pat));
  (void)HAL_DMA_Abort(&s_dma_diag);
  (void)HAL_DMA_DeInit(&s_dma_diag);
  st = HAL_DMA_Init(&s_dma_diag);
  if (st != HAL_OK)
  {
    printf("[dma-diag] reinit(write) fail st=%d err=0x%08lX\r\n",
           (int)st, (unsigned long)HAL_DMA_GetError(&s_dma_diag));
    return;
  }
  st = eth_dma_diag_copy("write", (uint32_t)&txd_pat, (uint32_t)txd0, sizeof(txd_pat), &err);
  eth_cache_invalidate(txd0, sizeof(*txd0));
  printf("[dma-diag][write] st=%d err=0x%08lX rb_d0=0x%08lX rb_d3=0x%08lX\r\n",
         (int)st, (unsigned long)err,
         (unsigned long)txd0->DESC0, (unsigned long)txd0->DESC3);

  /* Write RX descriptor too and verify readback. */
  rxd_pat.DESC0 = 0xCAFEBABEU;
  rxd_pat.DESC1 = 0x0F0F0F0FU;
  rxd_pat.DESC2 = 0x55AA55AAU;
  rxd_pat.DESC3 = 0xAA55AA55U;
  eth_cache_clean(&rxd_pat, sizeof(rxd_pat));
  (void)HAL_DMA_Abort(&s_dma_diag);
  st = eth_dma_diag_dma_init();
  if (st == HAL_OK)
  {
    st = eth_dma_diag_copy("write-rx", (uint32_t)&rxd_pat, (uint32_t)rxd0, sizeof(rxd_pat), &err);
    eth_cache_invalidate(rxd0, sizeof(*rxd0));
    printf("[dma-diag][write-rx] st=%d err=0x%08lX rb_d0=0x%08lX rb_d3=0x%08lX\r\n",
           (int)st, (unsigned long)err,
           (unsigned long)rxd0->DESC0, (unsigned long)rxd0->DESC3);
  }

  /* Restore descriptor content for normal ETH flow. */
diag_restore:
  memcpy(txd0, &txd_backup, sizeof(txd_backup));
  memcpy(rxd0, &rxd_backup, sizeof(rxd_backup));
  eth_cache_clean(txd0, sizeof(*txd0));
  eth_cache_clean(rxd0, sizeof(*rxd0));
  (void)HAL_DMA_Abort(&s_dma_diag);
  (void)HAL_DMA_DeInit(&s_dma_diag);
  eth_cache_invalidate(txd0, sizeof(*txd0));
  eth_cache_invalidate(rxd0, sizeof(*rxd0));

  (void)HAL_DMA_DeInit(&s_dma_diag);
}

static void eth_cache_clean_tx_desc(void)
{
  if (heth1.Init.TxDesc[ETH_DMA_CH0_IDX] == NULL)
  {
    return;
  }
  eth_cache_clean(heth1.Init.TxDesc[ETH_DMA_CH0_IDX],
                  (uint32_t)(ETH_TX_DESC_CNT * sizeof(ETH_DMADescTypeDef)));
}

static int32_t eth_rx_buf_index(const uint8_t *buff)
{
  uintptr_t base = (uintptr_t)&s_rx_buf_pool[0][0];
  uintptr_t end = base + sizeof(s_rx_buf_pool);
  uintptr_t addr = (uintptr_t)buff;

  if ((addr < base) || (addr >= end))
  {
    return -1;
  }
  return (int32_t)((addr - base) / ETH_PTP_RX_BUF_SIZE);
}

static void eth_rx_buf_free(uint8_t *buff)
{
  int32_t idx = eth_rx_buf_index(buff);
  if (idx >= 0)
  {
    s_rx_buf_in_use[idx] = 0U;
  }
}

static void eth_rx_chain_free(ETH_BufferTypeDef *chain)
{
  ETH_BufferTypeDef *node = chain;
  while (node != NULL)
  {
    ETH_BufferTypeDef *next = node->next;
    if (node->buffer != NULL)
    {
      eth_rx_buf_free(node->buffer);
    }
    node->buffer = NULL;
    node->len = 0U;
    node->next = NULL;
    node = next;
  }
}

static void eth_ptp_clock_config(void)
{
#if (ETH_PTP_SKIP_RUNTIME_PTP_CLKCFG != 0U)
  ETH_PTP_LOG_NON_DESC("[eth][ptpclk] skip runtime periph clk config\r\n");
  return;
#else
  RCC_PeriphCLKInitTypeDef periph = {0};

  periph.PeriphClockSelection = RCC_PERIPHCLK_ETH1PTP;
  periph.Eth1PtpClockSelection = ETH_PTP_CLK_SOURCE;
  periph.Eth1PtpDivider = ETH_PTP_CLK_DIVIDER;

  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

static uint32_t eth_ptp_get_clk_hz(void)
{
  uint32_t clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ETH1PTP);
  if (clk == 0U)
  {
    clk = HAL_RCC_GetHCLKFreq();
  }
  return clk;
}

static void eth_ptp_calc_addend(uint32_t ptp_clk_hz, uint32_t *subsec_inc, uint32_t *addend)
{
  if (ptp_clk_hz == 0U)
  {
    *subsec_inc = 20U;
    *addend = 0x0U;
    return;
  }

  /* Subsecond increment in ns (rounded). */
  *subsec_inc = (1000000000U + (ptp_clk_hz / 2U)) / ptp_clk_hz;

  /* Addend = (2^32 * ptp_clk) / 1e9. */
  *addend = (uint32_t)((((uint64_t)ptp_clk_hz) << 32) / 1000000000ULL);
}

static void eth_ptp_configure(void)
{
  /* This HAL package doesn't expose HAL_ETH_PTP_SetConfig()/ETH_PTP_ConfigTypeDef.
   * Keep baseline MAC/DMA bring-up path and skip PTP register programming here. */
  (void)eth_ptp_get_clk_hz;
  (void)eth_ptp_calc_addend;
}

static void eth_ptp_set_mac_filter(void)
{
  ETH_MACFilterConfigTypeDef filter = {0};

  if (HAL_ETH_GetMACFilterConfig(&heth1, &filter) != HAL_OK)
  {
    return;
  }

  /* Open RX filter for bring-up (gPTP uses link-local multicast). */
  filter.PromiscuousMode = ENABLE;
  filter.ReceiveAllMode = ENABLE;
  filter.HashMulticast = DISABLE;
  filter.PassAllMulticast = ENABLE;
  filter.BroadcastFilter = DISABLE;
  filter.ControlPacketsFilter = ETH_CTRLPACKETS_FORWARD_ALL;

  (void)HAL_ETH_SetMACFilterConfig(&heth1, &filter);
}

static void eth_enable_internal_loopback(void)
{
#if (ETH_PTP_DIAG_INTERNAL_LB != 0U)
  SET_BIT(heth1.Instance->MACCR, ETH_MACCR_LM);
  __DSB();
  ETH_PTP_LOG_NON_DESC("[diag] internal loopback enabled, MACCR=0x%08lX\r\n",
                       (unsigned long)heth1.Instance->MACCR);
#endif
}

static int32_t eth_phy_bringup(void)
{
  uint32_t reg = 0U;
  uint32_t tick = HAL_GetTick();

  /* Basic registers */
  const uint32_t PHY_BMCR = 0x00U;
  const uint32_t PHY_BMSR = 0x01U;
  const uint32_t BMCR_RESET = 0x8000U;
  const uint32_t BMCR_AN_ENABLE = 0x1000U;
  const uint32_t BMCR_AN_RESTART = 0x0200U;
  const uint32_t BMSR_LINK_STATUS = 0x0004U;

#if (ETH_PHY_ADDR_AUTO_SCAN != 0U)
  /* Auto-detect PHY address by reading PHY ID */
  {
    uint32_t id1 = 0U;
    uint32_t id2 = 0U;
    int32_t found = -1;
    uint32_t found_id1 = 0U;
    uint32_t found_id2 = 0U;
    uint32_t addr;
    uint32_t scan_start = HAL_GetTick();

    for (addr = 0; addr < 32U; addr++)
    {
      if (HAL_ETH_ReadPHYRegister(&heth1, addr, 0x02U, &id1) == HAL_OK &&
          HAL_ETH_ReadPHYRegister(&heth1, addr, 0x03U, &id2) == HAL_OK)
      {
        if (ETH_PTP_MDIO_SCAN_LOG != 0U)
        {
#if (ETH_PTP_MDIO_SCAN_VERBOSE != 0U)
          ETH_PTP_LOG_NON_DESC("[phy][scan] addr=%lu id1=0x%04lX id2=0x%04lX\r\n",
                               (unsigned long)addr,
                               (unsigned long)id1,
                               (unsigned long)id2);
#endif
        }

        if (id1 != 0x0000U && id1 != 0xFFFFU)
        {
          found = (int32_t)addr;
          found_id1 = id1;
          found_id2 = id2;
          break;
        }
      }

      if ((HAL_GetTick() - scan_start) > ETH_PHY_SCAN_TIMEOUT_MS)
      {
        break;
      }
    }

    if (found >= 0)
    {
      s_phy_addr = (uint32_t)found;
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
      ETH_PTP_LOG_NON_DESC("[phy] addr=%lu id1=0x%04lX id2=0x%04lX\r\n",
                           (unsigned long)s_phy_addr,
                           (unsigned long)found_id1,
                           (unsigned long)found_id2);
#endif
    }
    else
    {
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
      ETH_PTP_LOG_NON_DESC("[phy] auto-scan failed, using addr=%lu\r\n",
                           (unsigned long)s_phy_addr);
#endif
    }
  }
#endif

  /* PHY reset */
  if (HAL_ETH_WritePHYRegister(&heth1, s_phy_addr, PHY_BMCR, BMCR_RESET) != HAL_OK)
  {
    return -1;
  }

  /* Wait for reset complete */
  do
  {
    if (HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, PHY_BMCR, &reg) != HAL_OK)
    {
      return -2;
    }
    if ((reg & BMCR_RESET) == 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - tick) < ETH_PHY_RESET_TIMEOUT_MS);

  if ((reg & BMCR_RESET) != 0U)
  {
    return -3;
  }

  /* Enable and restart auto-negotiation */
  reg = BMCR_AN_ENABLE | BMCR_AN_RESTART;
  if (HAL_ETH_WritePHYRegister(&heth1, s_phy_addr, PHY_BMCR, reg) != HAL_OK)
  {
    return -4;
  }

  /* Wait for link up */
  tick = HAL_GetTick();
  do
  {
    if (HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, PHY_BMSR, &reg) != HAL_OK)
    {
      return -5;
    }
    if ((reg & BMSR_LINK_STATUS) != 0U)
    {
      s_link_up = 1U;
      return 0;
    }
    HAL_Delay(50);
  } while ((HAL_GetTick() - tick) < ETH_PHY_AN_TIMEOUT_MS);

  s_link_up = 0U;
  return 1; /* link still down */
}

static void eth_ptp_build_test_frame(uint8_t *frame, uint32_t *len)
{
  const uint8_t dst[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E}; /* gPTP multicast */
  const uint8_t *src = heth1.Init.MACAddr;
  uint16_t ethertype = 0x88F7; /* PTP over Ethernet */

  memcpy(&frame[0], dst, 6);
  memcpy(&frame[6], src, 6);
  frame[12] = (uint8_t)(ethertype >> 8);
  frame[13] = (uint8_t)(ethertype & 0xFF);
  memset(&frame[14], 0, 46);
  *len = 60U;
}

static void eth_ptp_build_learn_frame(uint8_t *frame, uint32_t *len)
{
  const uint8_t dst[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; /* broadcast */
  const uint8_t *src = heth1.Init.MACAddr;
  uint16_t ethertype = 0x0800; /* IPv4 ethertype (payload can be dummy) */

  memcpy(&frame[0], dst, 6);
  memcpy(&frame[6], src, 6);
  frame[12] = (uint8_t)(ethertype >> 8);
  frame[13] = (uint8_t)(ethertype & 0xFF);
  memset(&frame[14], 0xA5, 46);
  *len = 60U;
}

static void eth_tx_manual_probe_once(void)
{
#if (ETH_PTP_TX_MANUAL_PROBE != 0U)
  uint32_t saved_dlar0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
  uint32_t saved_dtpr0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  uint32_t saved_rlr0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR;
  uint32_t saved_txcr0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR;
  uint32_t dmacc0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCR;
  uint32_t stride = 16U;
  uint32_t frame_len = 0U;

  switch (dmacc0 & ETH_DMACxCR_DSL)
  {
    case ETH_DMACxCR_DSL_0BIT:   stride = 16U; break;
    case ETH_DMACxCR_DSL_32BIT:  stride = 20U; break;
    case ETH_DMACxCR_DSL_64BIT:  stride = 24U; break;
    case ETH_DMACxCR_DSL_128BIT: stride = 32U; break;
    default:                     stride = 16U; break;
  }
  ETH_PTP_LOG_NON_DESC("[tx][manual] dmacc0=0x%08lX dsl0=0x%08lX stride=0x%lX\r\n",
                       (unsigned long)dmacc0,
                       (unsigned long)(dmacc0 & ETH_DMACxCR_DSL),
                       (unsigned long)stride);

  eth_ptp_build_test_frame(s_probe_frame_nc, &frame_len);
  memset(&s_probe_desc_nc, 0, sizeof(s_probe_desc_nc));
  s_probe_desc_nc.DESC0 = (uint32_t)s_probe_frame_nc;
  s_probe_desc_nc.DESC2 = frame_len;
  s_probe_desc_nc.DESC3 = (ETH_DMATXNDESCWBF_OWN | ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD | frame_len);
  eth_cache_clean(s_probe_frame_nc, frame_len);
  eth_cache_clean(&s_probe_desc_nc, sizeof(s_probe_desc_nc));

  {
    uint32_t dtpr_a = (uint32_t)&s_probe_desc_nc;
    uint32_t dtpr_b = (uint32_t)&s_probe_desc_nc + stride;
#if (ETH_PTP_TX_MANUAL_PROBE_NS_ALIAS != 0U)
    uint32_t ns_off = 0x10000000U;
    uint32_t dlar_ns = (uint32_t)&s_probe_desc_nc - ns_off;
    uint32_t dtpr_ns_a = dlar_ns;
    uint32_t dtpr_ns_b = dlar_ns + stride;
#endif
    volatile uint32_t *txcr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR;
    volatile uint32_t *csr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
    volatile uint32_t *dlar = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
    volatile uint32_t *rlr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR;
    volatile uint32_t *tpr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
    volatile uint32_t *catxdr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR;
    volatile uint32_t *catxbr = &heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;

    MODIFY_REG(heth1.Instance->MTL_QUEUE[ETH_DMA_CH0_IDX].MTLTXQOMR, ETH_MTLTXQxOMR_TXQEN, ETH_MTLTXQxOMR_TXQEN_EN);

    /* A: DTPR = DLAR */
    s_probe_desc_nc.DESC0 = (uint32_t)s_probe_frame_nc;
    s_probe_desc_nc.DESC3 = (ETH_DMATXNDESCWBF_OWN | ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD | frame_len);
    eth_cache_clean(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    CLEAR_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    *csr = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    *dlar = (uint32_t)&s_probe_desc_nc;
    *rlr = 0U;
    *tpr = dtpr_a;
    SET_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    HAL_Delay(2U);
    eth_cache_invalidate(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    ETH_PTP_LOG_NON_DESC("[tx][manual-ch0-A] dtpr=0x%08lX own=%lu d3=0x%08lX catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX\r\n",
                         (unsigned long)dtpr_a,
                         (unsigned long)((s_probe_desc_nc.DESC3 & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)s_probe_desc_nc.DESC3,
                         (unsigned long)(*catxdr),
                         (unsigned long)(*catxbr),
                         (unsigned long)(*csr));

    /* B: DTPR = DLAR + stride */
    s_probe_desc_nc.DESC0 = (uint32_t)s_probe_frame_nc;
    s_probe_desc_nc.DESC3 = (ETH_DMATXNDESCWBF_OWN | ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD | frame_len);
    eth_cache_clean(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    CLEAR_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    *csr = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    *dlar = (uint32_t)&s_probe_desc_nc;
    *rlr = 0U;
    *tpr = dtpr_b;
    SET_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    HAL_Delay(2U);
    eth_cache_invalidate(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    ETH_PTP_LOG_NON_DESC("[tx][manual-ch0-B] dtpr=0x%08lX own=%lu d3=0x%08lX catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX\r\n",
                         (unsigned long)dtpr_b,
                         (unsigned long)((s_probe_desc_nc.DESC3 & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)s_probe_desc_nc.DESC3,
                         (unsigned long)(*catxdr),
                         (unsigned long)(*catxbr),
                         (unsigned long)(*csr));

#if (ETH_PTP_TX_MANUAL_PROBE_NS_ALIAS != 0U)
    /* NS alias probe: same physical memory, NS aliased addresses. */
    s_probe_desc_nc.DESC0 = (uint32_t)s_probe_frame_nc - ns_off;
    s_probe_desc_nc.DESC3 = (ETH_DMATXNDESCWBF_OWN | ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD | frame_len);
    eth_cache_clean(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    CLEAR_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    *csr = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    *dlar = dlar_ns;
    *rlr = 0U;
    *tpr = dtpr_ns_a;
    SET_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    HAL_Delay(2U);
    eth_cache_invalidate(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    ETH_PTP_LOG_NON_DESC("[tx][manual-ch0-NS-A] dtpr=0x%08lX own=%lu d3=0x%08lX catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX\r\n",
                         (unsigned long)dtpr_ns_a,
                         (unsigned long)((s_probe_desc_nc.DESC3 & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)s_probe_desc_nc.DESC3,
                         (unsigned long)(*catxdr),
                         (unsigned long)(*catxbr),
                         (unsigned long)(*csr));

    s_probe_desc_nc.DESC0 = (uint32_t)s_probe_frame_nc - ns_off;
    s_probe_desc_nc.DESC3 = (ETH_DMATXNDESCWBF_OWN | ETH_DMATXNDESCWBF_FD | ETH_DMATXNDESCWBF_LD | frame_len);
    eth_cache_clean(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    CLEAR_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    *csr = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
    *dlar = dlar_ns;
    *rlr = 0U;
    *tpr = dtpr_ns_b;
    SET_BIT(*txcr, ETH_DMACxTXCR_ST);
    __DSB();
    HAL_Delay(2U);
    eth_cache_invalidate(&s_probe_desc_nc, sizeof(s_probe_desc_nc));
    ETH_PTP_LOG_NON_DESC("[tx][manual-ch0-NS-B] dtpr=0x%08lX own=%lu d3=0x%08lX catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX\r\n",
                         (unsigned long)dtpr_ns_b,
                         (unsigned long)((s_probe_desc_nc.DESC3 & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)s_probe_desc_nc.DESC3,
                         (unsigned long)(*catxdr),
                         (unsigned long)(*catxbr),
                         (unsigned long)(*csr));
#endif
  }

  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
  __DSB();
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = saved_dlar0;
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR = saved_rlr0;
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = saved_dtpr0;
  heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR = saved_txcr0;
  __DSB();
#else
  (void)0;
#endif
}

static void eth_ptp_send_test_frame(void)
{
  ETH_BufferTypeDef txbuf = {0};
  ETH_TxPacketConfigTypeDef txcfg = {0};
  uint32_t frame_len = 0U;
  HAL_StatusTypeDef st;
  uint32_t tx_desc_pre_idx;
  uint32_t tx_desc_idx;
  ETH_DMADescTypeDef *d_pre = NULL;
  uint32_t d3_pre = 0U;
  uint32_t d3_post = 0U;
  uint32_t own_pre = 0U;
  uint32_t own_post = 0U;
  uint32_t catxdr_pre = 0U;
  uint32_t catxdr_post = 0U;
  uint32_t catxbr_pre = 0U;
  uint32_t catxbr_post = 0U;
  uint32_t tpr_pre = 0U;
  uint32_t tpr_post = 0U;
  uint32_t txg_pre = 0U;
  uint32_t txg_post = 0U;
  uint32_t qdr_pre = 0U;
  uint32_t qdr_post = 0U;
  uint32_t csr_pre = 0U;
  uint32_t csr_post = 0U;

  eth_ptp_build_test_frame(s_tx_frame, &frame_len);
  eth_cache_clean(s_tx_frame, frame_len);
  eth_cache_clean_tx_desc();

  txbuf.buffer = s_tx_frame;
  txbuf.len = frame_len;
  txbuf.next = NULL;

  txcfg.TxDMACh = ETH_DMA_CH0_IDX;
  txcfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  txcfg.Length = frame_len;
  txcfg.TxBuffer = &txbuf;
  txcfg.SrcAddrCtrl = ETH_SRC_ADDR_CONTROL_DISABLE;
  txcfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
  txcfg.MaxSegmentSize = 0U;
  txcfg.PayloadLen = 0U;
  txcfg.TCPHeaderLen = 0U;
  txcfg.VlanTag = 0U;
  txcfg.VlanCtrl = 0U;
  txcfg.InnerVlanTag = 0U;
  txcfg.InnerVlanCtrl = 0U;
  txcfg.pData = s_tx_frame;

  heth1.TxOpCH = ETH_DMA_CH0_IDX;
#if (ETH_PTP_TX_KICK_HACKS != 0U)
  eth_kick_tx_ch0_pre();
#endif
  /* Clear latched RISAF faults so any new fault can be attributed to this TX attempt. */
  eth_dma_diag_clear_risaf_flags();
  catxdr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR;
  catxbr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  tpr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  txg_pre = heth1.Instance->MMCTPCGR;
  qdr_pre = heth1.Instance->MTL_QUEUE[0].MTLTXQDR;
  csr_pre = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
  tx_desc_pre_idx = heth1.TxDescList[ETH_DMA_CH0_IDX].CurTxDesc;
  if (heth1.Init.TxDesc[ETH_DMA_CH0_IDX] != NULL)
  {
    d_pre = &heth1.Init.TxDesc[ETH_DMA_CH0_IDX][tx_desc_pre_idx];
    d3_pre = d_pre->DESC3;
    own_pre = ((d3_pre & ETH_DMATXNDESCRF_OWN) != 0U) ? 1U : 0U;
#if (ETH_PTP_TX_DESC_DUMP != 0U)
    printf("[tx][desc-pre] idx=%lu D3=0x%08lX OWN=%lu\r\n",
           (unsigned long)tx_desc_pre_idx,
           (unsigned long)d3_pre,
           (unsigned long)((d3_pre & ETH_DMATXNDESCRF_OWN) ? 1U : 0U));
#endif
  }
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_TX_USE_PTP_TS != 0U)
  (void)HAL_ETH_PTP_InsertTxTimestamp(&heth1);
#endif
  /* Isolate per-attempt error cause: clear sticky HAL error fields before submit. */
  heth1.ErrorCode = HAL_ETH_ERROR_NONE;
  heth1.DMAErrorCode = HAL_ETH_ERROR_NONE;
#if (ETH_PTP_DESC_LOG_ONLY == 0U) && (ETH_PTP_TX_PROBE_LOG != 0U)
  ETH_PTP_LOG_NON_DESC("[tx][pre] gState=%d txch=%lu\r\n",
                       (int)heth1.gState,
                       (unsigned long)heth1.TxOpCH);
#endif
#if (ETH_PTP_TX_USE_IT != 0U)
  st = HAL_ETH_Transmit_IT(&heth1, &txcfg);
#else
  st = HAL_ETH_Transmit(&heth1, &txcfg, 10U);
#endif
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_CLEAN_TX_DESC_AFTER_TX != 0U)
  eth_cache_clean_tx_desc();
#endif
  if (d_pre != NULL)
  {
    d3_post = d_pre->DESC3;
    own_post = ((d3_post & ETH_DMATXNDESCRF_OWN) != 0U) ? 1U : 0U;
#if (ETH_PTP_TX_DESC_DUMP != 0U)
    printf("[tx][desc-post] idx=%lu D3=0x%08lX OWN=%lu st=%d err=0x%08lX\r\n",
           (unsigned long)tx_desc_pre_idx,
           (unsigned long)d3_post,
           (unsigned long)((d3_post & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
           (int)st,
           (unsigned long)heth1.ErrorCode);
#endif
  }
  catxdr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR;
  catxbr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
  tpr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
  txg_post = heth1.Instance->MMCTPCGR;
  qdr_post = heth1.Instance->MTL_QUEUE[0].MTLTXQDR;
  csr_post = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
#if (ETH_PTP_TX_PATH_LOG != 0U)
  if (s_tx_path_log_done == 0U)
  {
    s_tx_path_log_done = 1U;
    ETH_PTP_LOG_NON_DESC("[tx][path] st=%d own=%lu->%lu catxdr=0x%08lX->0x%08lX catxbr=0x%08lX->0x%08lX csr=0x%08lX->0x%08lX qdr=0x%08lX->0x%08lX\r\n",
                         (int)st,
                         (unsigned long)own_pre,
                         (unsigned long)own_post,
                         (unsigned long)catxdr_pre,
                         (unsigned long)catxdr_post,
                         (unsigned long)catxbr_pre,
                         (unsigned long)catxbr_post,
                         (unsigned long)csr_pre,
                         (unsigned long)csr_post,
                         (unsigned long)qdr_pre,
                         (unsigned long)qdr_post);
  }
#endif
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
#if (ETH_PTP_TX_PROBE_LOG != 0U)
  ETH_PTP_LOG_NON_DESC("[tx][probe] st=%d own=%lu->%lu catxdr=0x%08lX->0x%08lX tpr=0x%08lX->0x%08lX txg=%lu->%lu qdr=0x%08lX->0x%08lX\r\n",
                       (int)st,
                       (unsigned long)own_pre,
                       (unsigned long)own_post,
                       (unsigned long)catxdr_pre,
                       (unsigned long)catxdr_post,
                       (unsigned long)tpr_pre,
                       (unsigned long)tpr_post,
                       (unsigned long)txg_pre,
                       (unsigned long)txg_post,
                       (unsigned long)qdr_pre,
                       (unsigned long)qdr_post);
#endif
#endif
  if (heth1.Init.TxDesc[ETH_DMA_CH0_IDX] != NULL)
  {
    tx_desc_idx = heth1.TxDescList[ETH_DMA_CH0_IDX].CurTxDesc;
    if (tx_desc_idx == 0U)
    {
      tx_desc_idx = ETH_TX_DESC_CNT - 1U;
    }
    else
    {
      tx_desc_idx -= 1U;
    }

    ETH_DMADescTypeDef *d = &heth1.Init.TxDesc[ETH_DMA_CH0_IDX][tx_desc_idx];
    (void)d;
#if (ETH_PTP_TX_DESC_DUMP != 0U)
    printf("[tx][desc] idx=%lu D0=0x%08lX D2=0x%08lX D3=0x%08lX\r\n",
           (unsigned long)tx_desc_idx,
           (unsigned long)d->DESC0,
           (unsigned long)d->DESC2,
           (unsigned long)d->DESC3);
#endif
  }

  /* Clear Tx buffer unavailable and restart DMA if it latched TBU. */
#if (ETH_PTP_TX_KICK_HACKS != 0U)
  if ((heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR & ETH_DMACxSR_TBU) != 0U)
  {
    heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = ETH_DMACxSR_TBU;
    SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
  }
#endif
  if (st != HAL_OK)
  {
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
    ETH_PTP_LOG_NON_DESC("[tx] HAL_ETH_Transmit failed: st=%d err=0x%08lX state=%d own=%lu d3=0x%08lX mtl=0x%08lX\r\n",
                         (int)st, (unsigned long)heth1.ErrorCode, (int)heth1.gState,
                         (unsigned long)((d3_post & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)d3_post,
                         (unsigned long)heth1.Instance->MTL_QUEUE[0].MTLTXQOMR);
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_TX_REG_DUMP != 0U)
    if (s_tx_fail_dumped == 0U)
    {
      s_tx_fail_dumped = 1U;
      eth_dump_tx_min_regs();
      eth_dump_risaf_fault_snapshot();
    }
#endif
#endif
#if (ETH_PTP_TX_KICK_HACKS != 0U)
    eth_force_recover_tx_ch0();
#endif
  }
  else
  {
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
    s_tx_fail_dumped = 0U;
#endif
#if (ETH_PTP_DESC_LOG_ONLY == 0U) && (ETH_PTP_TX_SUBMIT_LOG != 0U)
    ETH_PTP_LOG_NON_DESC("[tx] submit ok own=%lu d3=0x%08lX\r\n",
                         (unsigned long)((d3_post & ETH_DMATXNDESCRF_OWN) ? 1U : 0U),
                         (unsigned long)d3_post);
#endif
#if (ETH_PTP_TX_FORCE_NS_ALIAS_ONCE != 0U)
    if (d_pre != NULL)
    {
      uint32_t d0_old = d_pre->DESC0;
      uint32_t d1_old = d_pre->DESC1;
      d_pre->DESC0 = eth_to_ns_alias(d0_old);
      d_pre->DESC1 = eth_to_ns_alias(d1_old);
      __DMB();
      eth_cache_clean_tx_desc();
      eth_force_ch0_ns_alias();
      if (s_ns_alias_force_done == 0U)
      {
        s_ns_alias_force_done = 1U;
        ETH_PTP_LOG_NON_DESC("[tx][ns-force] d0 0x%08lX->0x%08lX d1 0x%08lX->0x%08lX\r\n",
                             (unsigned long)d0_old,
                             (unsigned long)d_pre->DESC0,
                             (unsigned long)d1_old,
                             (unsigned long)d_pre->DESC1);
      }
      if (s_ns_alias_check_done == 0U)
      {
        uint32_t d0_ns = 0U;
        uint32_t d3_ns = 0U;
        uint32_t buf_ns = 0U;
        volatile uint32_t *p_desc_ns = (volatile uint32_t *)eth_to_ns_alias((uint32_t)d_pre);
        volatile uint32_t *p_buf_ns = (volatile uint32_t *)eth_to_ns_alias(d_pre->DESC0);
        d0_ns = p_desc_ns[0];
        d3_ns = p_desc_ns[3];
        buf_ns = p_buf_ns[0];
        s_ns_alias_check_done = 1U;
        ETH_PTP_LOG_NON_DESC("[tx][ns-check] desc_ns d0=0x%08lX d3=0x%08lX buf_ns_w0=0x%08lX\r\n",
                             (unsigned long)d0_ns,
                             (unsigned long)d3_ns,
                             (unsigned long)buf_ns);
      }
    }
#endif

#if (ETH_PTP_TX_DTPR_POINTS_TO_LAST_PREPARED != 0U)
    /* Tail-pointer semantics probe: set DTPR to the descriptor we just prepared (inclusive tail),
     * instead of HAL's default "next free" descriptor. */
    if (d_pre != NULL)
    {
      uint32_t dtpr_last = (uint32_t)d_pre;
#if (ETH_PTP_TX_FORCE_NS_ALIAS_ONCE != 0U)
      dtpr_last = eth_to_ns_alias(dtpr_last);
#endif
      uint32_t catxbr0 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
      heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
      heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_last;
      SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
      __DSB();
      HAL_Delay(1U);
      uint32_t catxbr1 = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
      if ((catxbr0 == 0U) && (catxbr1 != 0U))
      {
        ETH_PTP_LOG_NON_DESC("[tx][dtpr] HIT last-prepared DTPR=0x%08lX CATXBR 0x%08lX->0x%08lX\r\n",
                             (unsigned long)dtpr_last,
                             (unsigned long)catxbr0,
                             (unsigned long)catxbr1);
      }
    }
#endif

#if (ETH_PTP_TX_RECOVER_TBU != 0U)
    /* On this platform, first submit often lands in TBU when ring starts empty.
     * Do an explicit W1C + ST re-arm + tail rewrite to force a fetch retry.
     */
    {
      uint32_t csr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
      if (((csr & (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS)) != 0U) && (s_tbu_recovered == 0U))
      {
        uint32_t tdt = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
        CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
        __DSB();
        heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS); /* W1C */
        heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = tdt; /* doorbell */
        SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
        __DSB();
        s_tbu_recovered = 1U;
        ETH_PTP_LOG_NON_DESC("[tx][tbu] cleared csr=0x%08lX tdt=0x%08lX\r\n",
                             (unsigned long)csr,
                             (unsigned long)tdt);
      }
    }
#endif
#if (ETH_PTP_TX_USE_IT == 0U)
    (void)HAL_ETH_ReleaseTxPacket(&heth1);
    s_tx_cplt_count++;
#endif

    /* One-shot stall check: if DMA never consumes the descriptor, OWN will remain set. */
    osDelay(10U);
    if (d_pre != NULL)
    {
      uint32_t d3_10ms = d_pre->DESC3;
      if ((d3_10ms & ETH_DMATXNDESCRF_OWN) != 0U)
      {
#if (ETH_PTP_TX_MANUAL_PROBE != 0U)
        if (s_manual_probe_done == 0U)
        {
          s_manual_probe_done = 1U;
          eth_tx_manual_probe_once();
        }
#endif
        uint32_t dmacsr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR;
        uint32_t txcr   = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR;
        uint32_t dmacc  = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCR;
        uint32_t maccr  = heth1.Instance->MACCR;
        uint32_t txqomr = heth1.Instance->MTL_QUEUE[0].MTLTXQOMR;
        uint32_t dmad = heth1.Instance->DMADSR;
        uint32_t dlar = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
        uint32_t dtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;

#if (ETH_PTP_DIAG_DUMP_AXI_A4 != 0U)
        if (s_axi_a4_dumped == 0U)
        {
          s_axi_a4_dumped = 1U;
          ETH_PTP_LOG_NON_DESC("[eth][a4] DACR=0x%08lX TXACR=0x%08lX RXACR=0x%08lX SBMR=0x%08lX MR=0x%08lX\r\n",
                               (unsigned long)heth1.Instance->DMAA4DACR,
                               (unsigned long)heth1.Instance->DMAA4TXACR,
                               (unsigned long)heth1.Instance->DMAA4RXACR,
                               (unsigned long)heth1.Instance->DMASBMR,
                               (unsigned long)heth1.Instance->DMAMR);
#if (ETH_PTP_DIAG_FORCE_A4DACR_BYPASS != 0U)
          /* Try to effectively "disable" descriptor caching by setting the cache-control
           * fields to their most permissive value (all ones). This is a one-shot diagnostic.
           * If this unblocks CATXBR/OWN, we know the default DMAA4* cache settings were the issue. */
          heth1.Instance->DMAA4DACR =
              (0xFU << ETH_DMAA4DACR_TDWC_Pos) |
              (0xFU << ETH_DMAA4DACR_TDWD_Pos) |
              (0xFU << ETH_DMAA4DACR_RDRC_Pos);
          __DSB();
          ETH_PTP_LOG_NON_DESC("[eth][a4] DACR forced=0x%08lX\r\n", (unsigned long)heth1.Instance->DMAA4DACR);
#endif
        }
#endif

        ETH_PTP_LOG_NON_DESC("[tx][stall] own=1 csr=0x%08lX catxdr=0x%08lX catxbr=0x%08lX dlar=0x%08lX dtpr=0x%08lX d0=0x%08lX d3=0x%08lX dmad=0x%08lX axr=%lu axw=%lu\r\n",
                             (unsigned long)dmacsr,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                             (unsigned long)dlar,
                             (unsigned long)dtpr,
                             (unsigned long)d_pre->DESC0,
                             (unsigned long)d3_10ms,
                             (unsigned long)dmad,
                             (unsigned long)((dmad & ETH_DMADSR_AXRHSTS) ? 1U : 0U),
                             (unsigned long)((dmad & ETH_DMADSR_AXWHSTS) ? 1U : 0U));
#if (ETH_PTP_TX_STALL_MIN_LOG == 0U)
        ETH_PTP_LOG_NON_DESC("[tx][stall-ext] d2=0x%08lX txcr=0x%08lX dmacc=0x%08lX dsl=0x%08lX maccr=0x%08lX txqomr=0x%08lX dmad=0x%08lX tps0=0x%08lX axr=%lu axw=%lu\r\n",
                             (unsigned long)d_pre->DESC2,
                             (unsigned long)txcr,
                             (unsigned long)dmacc,
                             (unsigned long)(dmacc & ETH_DMACxCR_DSL),
                             (unsigned long)maccr,
                             (unsigned long)txqomr,
                             (unsigned long)dmad,
                             (unsigned long)(dmad & ETH_DMADSR_TPS0),
                             (unsigned long)((dmad & ETH_DMADSR_AXRHSTS) ? 1U : 0U),
                             (unsigned long)((dmad & ETH_DMADSR_AXWHSTS) ? 1U : 0U));
#endif
        if ((s_ch_diag_once == 0U) && (ETH_PTP_TX_STALL_MIN_LOG == 0U))
        {
          s_ch_diag_once = 1U;
          ETH_PTP_LOG_NON_DESC("[tx][ch] c0:csr=0x%08lX dlar=0x%08lX dtpr=0x%08lX catxdr=0x%08lX catxbr=0x%08lX txcr=0x%08lX | c1:csr=0x%08lX dlar=0x%08lX dtpr=0x%08lX catxdr=0x%08lX catxbr=0x%08lX txcr=0x%08lX rxmap=0x%08lX\r\n",
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACSR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXDLAR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXDTPR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACCATXDR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACCATXBR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXCR,
                               (unsigned long)heth1.Instance->MTLRXQDMAMR);
        }
#if (ETH_PTP_TX_FORCE_NS_ALIAS_ONCE != 0U)
        if (((dmacsr & ETH_DMACxSR_TBU) != 0U) && (s_ns_alias_force_done == 0U))
        {
          uint32_t ns_off = 0x10000000U;
          uint32_t d0_old = d_pre->DESC0;
          uint32_t d1_old = d_pre->DESC1;
          uint32_t dlar_old = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
          uint32_t dtpr_old = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;

          if (((dlar_old & 0xFF000000U) == 0x34000000U) && ((dtpr_old & 0xFF000000U) == 0x34000000U))
          {
            s_ns_alias_force_done = 1U;
            if ((d0_old & 0xFF000000U) == 0x34000000U) { d_pre->DESC0 = d0_old - ns_off; }
            if ((d1_old & 0xFF000000U) == 0x34000000U) { d_pre->DESC1 = d1_old - ns_off; }
            __DMB();

            CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = dlar_old - ns_off;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_old - ns_off;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
            SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
            __DSB();

            ETH_PTP_LOG_NON_DESC("[tx][ns-force] dlar 0x%08lX->0x%08lX dtpr 0x%08lX->0x%08lX d0 0x%08lX->0x%08lX\r\n",
                                 (unsigned long)dlar_old,
                                 (unsigned long)(dlar_old - ns_off),
                                 (unsigned long)dtpr_old,
                                 (unsigned long)(dtpr_old - ns_off),
                                 (unsigned long)d0_old,
                                 (unsigned long)d_pre->DESC0);
          }
        }
#endif
#if (ETH_PTP_TX_BAD_DLAR_PROBE != 0U)
        if (s_bad_dlar_probe_done == 0U)
        {
          uint32_t save_dlar = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
          uint32_t save_dtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
          uint32_t save_rlr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR;
          uint32_t save_txcr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR;
          uint32_t bad_dlar = 0x3FFFFFF0U; /* intentionally invalid/unmapped */
          uint32_t bad_dtpr = bad_dlar + 0x18U;

          s_bad_dlar_probe_done = 1U;
          CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
          __DSB();
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = bad_dlar;
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR = 0U;
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = bad_dtpr;
          SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
          __DSB();
          osDelay(1U);

          ETH_PTP_LOG_NON_DESC("[tx][bad-dlar] csr=0x%08lX dmad=0x%08lX catxdr=0x%08lX catxbr=0x%08lX axr=%lu axw=%lu\r\n",
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR,
                               (unsigned long)heth1.Instance->DMADSR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                               (unsigned long)((heth1.Instance->DMADSR & ETH_DMADSR_AXRHSTS) ? 1U : 0U),
                               (unsigned long)((heth1.Instance->DMADSR & ETH_DMADSR_AXWHSTS) ? 1U : 0U));

          CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
          __DSB();
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = save_dlar;
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR = save_rlr;
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = save_dtpr;
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR = save_txcr;
          __DSB();
        }
#endif
#if (ETH_PTP_FOCUS_LOG != 0U)
        ETH_PTP_LOG_NON_DESC("[focus][begin]\r\n");
        ETH_PTP_LOG_NON_DESC("[focus] txdesc_s=0x%08lX txdesc_ns=0x%08lX txbuf_s=0x%08lX txbuf_ns=0x%08lX\r\n",
                             (unsigned long)(uint32_t)d_pre,
                             (unsigned long)((uint32_t)d_pre - 0x10000000U),
                             (unsigned long)d_pre->DESC0,
                             (unsigned long)((d_pre->DESC0 >= 0x34000000U) ? (d_pre->DESC0 - 0x10000000U) : d_pre->DESC0));
        ETH_PTP_LOG_NON_DESC("[focus] ch0 dlar=0x%08lX dtpr=0x%08lX catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX\r\n",
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                             (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR);
        ETH_PTP_LOG_NON_DESC("[focus][end]\r\n");
#endif
        if ((RISAF2->IASR != 0U) || (RISAF6->IASR != 0U))
        {
          eth_dump_risaf_fault_snapshot();
          eth_dma_diag_clear_risaf_flags();
        }
        eth_dump_rifsc_attrs_once("stall");
#if (ETH_PTP_DIAG_SWEEP_RIMC_MSEC != 0U)
        eth_diag_sweep_rimc_msec_once(dlar, dtpr);
#endif
#if (ETH_PTP_DIAG_TRY_RIMC_SUBSET_ONCE != 0U)
        eth_diag_try_rimc_subset_once(dlar);
#endif

#if (ETH_PTP_TX_DIAG_TAILPTR_KICKS != 0U)
        if ((s_tdt_diag_done == 0U) && ((dmacsr & ETH_DMACxSR_TBU) != 0U))
        {
          /* Variant A: DTPR points to current descriptor (some IPs treat this as "last").
           * Variant B: DTPR points to last descriptor in the ring (DLAR + stride*RLR). */
          uint32_t rlr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR;
          uint32_t stride = (dtpr > dlar) ? (dtpr - dlar) : 0U;
          if ((stride == 0U) || (stride > 0x100U))
          {
            stride = 0x18U; /* expected for HAL ETH_DMADescTypeDef + DSL=64 */
          }
          uint32_t dtpr_a = dlar;
          uint32_t dtpr_b = dlar + (stride * (rlr & 0x3FFU));

          s_tdt_diag_done = 1U;

          /* A */
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_a;
          osDelay(1U);
          ETH_PTP_LOG_NON_DESC("[tx][tdtA] rlr=0x%08lX stride=0x%lX dtpr=0x%08lX catxbr=0x%08lX dmad=0x%08lX\r\n",
                               (unsigned long)rlr,
                               (unsigned long)stride,
                               (unsigned long)dtpr_a,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                               (unsigned long)heth1.Instance->DMADSR);

          /* B */
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_b;
          osDelay(1U);
          ETH_PTP_LOG_NON_DESC("[tx][tdtB] rlr=0x%08lX stride=0x%lX dtpr=0x%08lX catxbr=0x%08lX dmad=0x%08lX\r\n",
                               (unsigned long)rlr,
                               (unsigned long)stride,
                               (unsigned long)dtpr_b,
                               (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                               (unsigned long)heth1.Instance->DMADSR);

          /* Restore original tail pointer to avoid confusing subsequent tests. */
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
          heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr;
        }
#endif

#if (ETH_PTP_TX_DIAG_TRY_NS_ALIAS != 0U)
        /* If descriptor fetches are blocked due to a S/NS alias mismatch, ETH DMA may see
         * zeros even though CPU (secure) can read the secure alias. Try the NS alias once. */
        if ((dmacsr & ETH_DMACxSR_TBU) != 0U)
        {
          uint32_t catxdr_s = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR;
          uint32_t catxbr_s = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR;
          if (((catxdr_s & 0xFF000000U) == 0x34000000U) && (catxbr_s == 0U))
          {
            uint32_t ns_off = 0x10000000U;
            uint32_t dlar_ns = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR - ns_off;
            uint32_t dtpr_ns = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR - ns_off;
            /* Also translate buffer pointers inside the descriptor to the NS alias, otherwise
             * a non-secure ETH DMA master could still fail when fetching the packet buffer. */
            uint32_t d0_old = d_pre->DESC0;
            uint32_t d1_old = d_pre->DESC1;
            if ((d0_old & 0xFF000000U) == 0x34000000U)
            {
              d_pre->DESC0 = d0_old - ns_off;
            }
            if ((d1_old & 0xFF000000U) == 0x34000000U)
            {
              d_pre->DESC1 = d1_old - ns_off;
            }
            __DMB();

            CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = dlar_ns;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_ns;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
            SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);

            osDelay(1U);
            ETH_PTP_LOG_NON_DESC("[tx][ns-alias] catxdr=0x%08lX catxbr=0x%08lX csr=0x%08lX d0=0x%08lX\r\n",
                                 (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXDR,
                                 (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCATXBR,
                                 (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR,
                                 (unsigned long)d_pre->DESC0);

            /* Restore secure alias to avoid confusing the rest of the stack. */
            CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR = dlar_ns + ns_off;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR = dtpr_ns + ns_off;
            heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACSR = (ETH_DMACxSR_TBU | ETH_DMACxSR_TPS);
            SET_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR, ETH_DMACxTXCR_ST);

            /* Restore descriptor buffer pointers. */
            d_pre->DESC0 = d0_old;
            d_pre->DESC1 = d1_old;
            __DMB();
          }
        }
#endif
      }
    }
  }
}

static void eth_ptp_send_learn_frame(void)
{
  ETH_BufferTypeDef txbuf = {0};
  ETH_TxPacketConfigTypeDef txcfg = {0};
  uint32_t frame_len = 0U;
  HAL_StatusTypeDef st;

  eth_ptp_build_learn_frame(s_tx_frame, &frame_len);
  eth_cache_clean(s_tx_frame, frame_len);
  eth_cache_clean_tx_desc();

  txbuf.buffer = s_tx_frame;
  txbuf.len = frame_len;
  txbuf.next = NULL;

  txcfg.TxDMACh = ETH_DMA_CH0_IDX;
  txcfg.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  txcfg.Length = frame_len;
  txcfg.TxBuffer = &txbuf;
  txcfg.SrcAddrCtrl = ETH_SRC_ADDR_CONTROL_DISABLE;
  txcfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
  txcfg.MaxSegmentSize = 0U;
  txcfg.PayloadLen = 0U;
  txcfg.TCPHeaderLen = 0U;
  txcfg.VlanTag = 0U;
  txcfg.VlanCtrl = 0U;
  txcfg.InnerVlanTag = 0U;
  txcfg.InnerVlanCtrl = 0U;
  txcfg.pData = s_tx_frame;

  heth1.TxOpCH = ETH_DMA_CH0_IDX;
  eth_kick_tx_ch0_pre();
#if (ETH_PTP_TX_USE_IT != 0U)
  st = HAL_ETH_Transmit_IT(&heth1, &txcfg);
#else
  st = HAL_ETH_Transmit(&heth1, &txcfg, 10U);
#endif
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_CLEAN_TX_DESC_AFTER_TX != 0U)
  eth_cache_clean_tx_desc();
#endif
  if (st != HAL_OK)
  {
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
    ETH_PTP_LOG_NON_DESC("[tx][learn] HAL_ETH_Transmit failed: st=%d err=0x%08lX state=%d\r\n",
                         (int)st, (unsigned long)heth1.ErrorCode, (int)heth1.gState);
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_TX_REG_DUMP != 0U)
    eth_dump_tx_min_regs();
    eth_dump_risaf_fault_snapshot();
#endif
#endif
    eth_force_recover_tx_ch0();
  }
  else
  {
#if (ETH_PTP_TX_USE_IT == 0U)
    (void)HAL_ETH_ReleaseTxPacket(&heth1);
    s_tx_cplt_count++;
#endif
  }
}

/* --- HAL callbacks -------------------------------------------------------- */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  uint32_t i;
  if (buff == NULL)
  {
    return;
  }

  for (i = 0; i < ETH_PTP_RX_BUF_COUNT; i++)
  {
    if (s_rx_buf_in_use[i] == 0U)
    {
      s_rx_buf_in_use[i] = 1U;
      *buff = &s_rx_buf_pool[i][0];
      return;
    }
  }
  *buff = NULL;
  s_rx_alloc_fail++;
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  int32_t idx = eth_rx_buf_index(buff);
  if (idx < 0)
  {
    return;
  }

  s_rx_nodes[idx].buffer = buff;
  s_rx_nodes[idx].len = Length;
  s_rx_nodes[idx].next = NULL;

  if (*pStart == NULL)
  {
    *pStart = &s_rx_nodes[idx];
  }
  else
  {
    ((ETH_BufferTypeDef *)(*pEnd))->next = &s_rx_nodes[idx];
  }
  *pEnd = &s_rx_nodes[idx];
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  (void)heth;
  if (s_rx_sem != NULL)
  {
    (void)osSemaphoreRelease(s_rx_sem);
  }
}

void HAL_ETH_TxPtpCallback(uint32_t *buff, ETH_TimeStampTypeDef *timestamp)
{
  (void)buff;
  if (timestamp != NULL)
  {
    s_tx_ts_last = *timestamp;
    s_tx_ts_ready = 1U;
  }
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
  (void)heth;
  s_tx_cplt_count++;
  (void)HAL_ETH_ReleaseTxPacket(&heth1);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
  if (heth == NULL)
  {
    return;
  }
  s_eth_err_code = heth->ErrorCode;
  s_eth_dma_err = heth->DMAErrorCode;
  s_eth_err_pending = 1U;
}

/* --- Task entry ----------------------------------------------------------- */
static void ETH_PTP_Task(void *argument)
{
  uint32_t last_tx_ms = 0U;
  uint32_t last_learn_ms = 0U;
  uint32_t last_link_ms = 0U;
  uint32_t last_sweep_ms = 0U;
  uint32_t link_up_since_ms = 0U;
  uint32_t last_rx_log_ms = 0U;
  uint32_t last_rx_frame_count = 0U;
#if (ETH_PTP_PROGRESS_LOG != 0U)
  uint32_t last_tx_cplt_reported = 0xFFFFFFFFU;
#endif
  uint32_t last_tx_cplt_once = 0xFFFFFFFFU;
  uint8_t learn_sent_once = 0U;
  uint8_t tx_test_sent_once = 0U;

  (void)argument;

  s_rx_sem = osSemaphoreNew(ETH_PTP_RX_SEM_MAX_COUNT, 0U, NULL);
  if (s_rx_sem == NULL)
  {
    Error_Handler();
  }

  if (ETH_PTP_CONFIGURE_CLOCK != 0U)
  {
    eth_ptp_clock_config();
  }
  eth_sec_config_ethram();
  eth_debug_open_gate();
  eth_reinit_with_local_desc();
  eth_open_ns_dma_window_once();
  (void)eth_phy_bringup();
  eth_ptp_set_mac_filter();
  eth_ptp_configure();
#if (ETH_PTP_DMA_DIAG_ENABLE != 0U)
  eth_dma_diag_run();
#endif

  /* One-shot DMA software reset to rule out a stale DMA state machine before Start_IT. */
  eth_dma_soft_reset_once();

#if (ETH_PTP_FORCE_EDSE != 0U)
  /* (legacy) */
#endif

#if (ETH_PTP_FORCE_DSL_64 != 0U)
  /* (legacy) */
#endif

  /* Define/patch descriptor format and DMA channel knobs *before* enabling ETH DMA. */
  eth_fixup_desc_format();

  /* Keep queues in normal mode (not AV). HAL_ETH_Start_IT may reprogram queue mode. */
  eth_force_txq0_normal_mode();
  MODIFY_REG(heth1.Instance->MTL_QUEUE[1].MTLTXQOMR, ETH_MTLTXQxOMR_TXQEN, ETH_MTLTXQxOMR_TXQEN_NOT);

  if (HAL_ETH_Start_IT(&heth1) != HAL_OK)
  {
    Error_Handler();
  }
  ETH_PTP_LOG_NON_DESC("[eth][post-start] dlar=0x%08lX dtpr=0x%08lX rlr=0x%08lX txcr=0x%08lX dmacc=0x%08lX txqomr=0x%08lX\r\n",
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXRLR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXCR,
                       (unsigned long)heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACCR,
                       (unsigned long)heth1.Instance->MTL_QUEUE[0].MTLTXQOMR);
#if (ETH_PTP_FORCE_TXQ0_NORMAL_MODE != 0U)
  eth_force_txq0_normal_mode();
#endif
#if (ETH_PTP_TX_FORCE_NS_ALIAS_ONCE != 0U)
  eth_force_ch0_ns_alias();
#endif

#if (ETH_PTP_BRINGUP_SINGLE_CH0 != 0U)
  /* Disable CH1/Q1 to keep TX bring-up focused on a single DMA channel. */
  MODIFY_REG(heth1.Instance->MTL_QUEUE[1].MTLTXQOMR, ETH_MTLTXQxOMR_TXQEN, ETH_MTLTXQxOMR_TXQEN_NOT);
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACTXCR, ETH_DMACxTXCR_ST);
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACRXCR, ETH_DMACxRXCR_SR);
  __HAL_ETH_DMA_CH_DISABLE_IT(&heth1, 0xFFFFFFFFU, ETH_DMA_CH1_IDX);
#endif

  eth_enable_internal_loopback();
#if (ETH_PTP_START_DESC_DUMP != 0U)
  eth_dump_desc_ch0_once("start");
  HAL_Delay(1U);
  eth_dump_desc_ch0_once("start+1ms");
#endif
#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_RX_DISABLE_FOR_TX_TEST != 0U)
  /* Disable RX path to isolate TX bring-up. */
  CLEAR_BIT(heth1.Instance->MACCR, ETH_MACCR_RE);
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACRXCR, ETH_DMACxRXCR_SR);
  CLEAR_BIT(heth1.Instance->DMA_CH[ETH_DMA_CH1_IDX].DMACRXCR, ETH_DMACxRXCR_SR);
  __HAL_ETH_DMA_CH_DISABLE_IT(&heth1, ETH_DMACxIER_RIE | ETH_DMACxIER_RBUE, ETH_DMA_CH0_IDX);
  __HAL_ETH_DMA_CH_DISABLE_IT(&heth1, ETH_DMACxIER_RIE | ETH_DMACxIER_RBUE, ETH_DMA_CH1_IDX);
#endif

  for (;;)
  {
    void *pAppBuff = NULL;
    ETH_TimeStampTypeDef rx_ts = {0};

    (void)osSemaphoreAcquire(s_rx_sem, 50U);

    while (HAL_ETH_ReadData(&heth1, &pAppBuff) == HAL_OK)
    {
      ETH_BufferTypeDef *buf = (ETH_BufferTypeDef *)pAppBuff;
      const uint8_t *frame = (buf != NULL) ? buf->buffer : NULL;
      uint16_t len = (buf != NULL) ? buf->len : 0U;

      if (pAppBuff == NULL)
      {
        break;
      }

      if (frame != NULL && len != 0U)
      {
        eth_cache_invalidate(frame, len);
      }

      s_rx_frame_count++;
      s_rx_byte_count += len;

#if (ETH_PTP_RX_FRAME_DUMP != 0U)
        if (frame != NULL && len >= 14U)
        {
          uint16_t ethertype = (uint16_t)((frame[12] << 8) | frame[13]);
          ETH_PTP_LOG_NON_DESC("[rx] len=%u dst=%02X:%02X:%02X:%02X:%02X:%02X src=%02X:%02X:%02X:%02X:%02X:%02X type=0x%04X\r\n",
                               (unsigned int)len,
                               frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
                               frame[6], frame[7], frame[8], frame[9], frame[10], frame[11],
                               ethertype);
        }
#endif

      /* Some HAL variants don't expose PTP RX timestamp API. */
#if defined(HAL_ETH_PTP_GetRxTimestamp)
      if (HAL_ETH_PTP_GetRxTimestamp(&heth1, &rx_ts) == HAL_OK)
      {
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
        ETH_PTP_LOG_NON_DESC("[ptp][rx] ts=%lu.%09lu\r\n",
                             (unsigned long)rx_ts.TimeStampHigh,
                             (unsigned long)rx_ts.TimeStampLow);
#endif
      }
#endif

      eth_rx_chain_free((ETH_BufferTypeDef *)pAppBuff);
      pAppBuff = NULL;
    }

    if ((HAL_GetTick() - last_rx_log_ms) >= 1000U)
    {
      last_rx_log_ms = HAL_GetTick();
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
#if (ETH_PTP_PROGRESS_LOG != 0U)
      if (s_rx_frame_count == last_rx_frame_count)
      {
        ETH_PTP_LOG_NON_DESC("[rx] no frames (total=%lu)\r\n", (unsigned long)s_rx_frame_count);
      }
      if (s_rx_alloc_fail != 0U)
      {
        ETH_PTP_LOG_NON_DESC("[rx] alloc fail=%lu\r\n", (unsigned long)s_rx_alloc_fail);
      }
      if (s_tx_cplt_count != last_tx_cplt_reported)
      {
        ETH_PTP_LOG_NON_DESC("[tx] cplt=%lu\r\n", (unsigned long)s_tx_cplt_count);
        last_tx_cplt_reported = s_tx_cplt_count;
      }
#endif
#endif
      last_rx_frame_count = s_rx_frame_count;
    }

#if (ETH_PTP_DESC_LOG_ONLY == 0U) && (ETH_PTP_TX_CPLT_LOG != 0U)
    if (s_tx_cplt_count != last_tx_cplt_once)
    {
      ETH_PTP_LOG_NON_DESC("[tx] cplt=%lu\r\n", (unsigned long)s_tx_cplt_count);
      last_tx_cplt_once = s_tx_cplt_count;
    }
#endif

    if (s_tx_ts_ready != 0U)
    {
      s_tx_ts_ready = 0U;
#if (ETH_PTP_DESC_LOG_ONLY == 0U)
      ETH_PTP_LOG_NON_DESC("[ptp][tx] ts=%lu.%09lu\r\n",
                           (unsigned long)s_tx_ts_last.TimeStampHigh,
                           (unsigned long)s_tx_ts_last.TimeStampLow);
#endif
    }

    if (s_eth_err_pending != 0U)
    {
      s_eth_err_pending = 0U;
    }

#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_LINK_POLL_ENABLE != 0U)
    if ((HAL_GetTick() - last_link_ms) >= ETH_PTP_LINK_POLL_MS)
    {
      uint32_t bmsr = 0U;
      last_link_ms = HAL_GetTick();
      if (HAL_ETH_ReadPHYRegister(&heth1, s_phy_addr, 0x01U, &bmsr) == HAL_OK)
      {
        uint32_t up = ((bmsr & 0x0004U) != 0U) ? 1U : 0U;
          if (up != s_link_up)
          {
            s_link_up = up;
            ETH_PTP_LOG_NON_DESC("[phy] link %s (addr=%lu bmsr=0x%04lX)\r\n",
                                 up ? "up" : "down",
                                 (unsigned long)s_phy_addr,
                                 (unsigned long)bmsr);
#if (ETH_PTP_TX_TEST_ON_LINKUP_ONLY != 0U)
            /* Track stable link-up window for one-shot TX. */
            if (up != 0U)
            {
              link_up_since_ms = HAL_GetTick();
              s_tbu_recovered = 0U;
            }
            else
            {
              link_up_since_ms = 0U;
              s_tbu_recovered = 0U;
            }
#endif
#if (ETH_PTP_RESTART_ETH_ON_LINK_CHANGE != 0U)
            if (up == 0U)
            {
              if (heth1.gState == HAL_ETH_STATE_STARTED)
              {
                (void)HAL_ETH_Stop_IT(&heth1);
              }
              tx_test_sent_once = 0U; /* re-arm one-shot TX after link returns */
            }
            else
            {
              if (heth1.gState == HAL_ETH_STATE_READY)
              {
                (void)HAL_ETH_Start_IT(&heth1);
              }
            }
#endif
          }
          else if (ETH_PTP_LINK_POLL_ALWAYS_LOG != 0U)
          {
            ETH_PTP_LOG_NON_DESC("[phy] link %s (addr=%lu bmsr=0x%04lX)\r\n",
                                 up ? "up" : "down",
                                 (unsigned long)s_phy_addr,
                                 (unsigned long)bmsr);
          }
      }
      else
      {
        ETH_PTP_LOG_NON_DESC("[phy] bmsr read failed (addr=%lu)\r\n", (unsigned long)s_phy_addr);
      }
    }
#endif

#if (ETH_PTP_DIAG_SWEEP_RIMC_MSEC != 0U)
    if ((HAL_GetTick() - last_sweep_ms) >= 100U)
    {
      uint32_t dlar = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDLAR;
      uint32_t dtpr = heth1.Instance->DMA_CH[ETH_DMA_CH0_IDX].DMACTXDTPR;
      last_sweep_ms = HAL_GetTick();
      eth_diag_sweep_rimc_msec_once(dlar, dtpr);
    }
#endif

#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_TX_TEST_ENABLE != 0U)
#if (ETH_PTP_TX_TEST_ON_LINKUP_ONLY != 0U)
    if ((s_link_up != 0U) && (tx_test_sent_once == 0U) &&
        (link_up_since_ms != 0U) &&
        ((HAL_GetTick() - link_up_since_ms) >= ETH_PTP_TX_STABLE_LINK_MS))
    {
#if (ETH_PTP_DESC_LOG_ONLY == 0U) && (ETH_PTP_TX_SUBMIT_LOG != 0U)
      ETH_PTP_LOG_NON_DESC("[tx] test send (link up)\r\n");
#endif
#if (ETH_PTP_PREKICK_ON_LINKUP != 0U)
      eth_pre_tx_submit_recover_once();
#endif
      eth_ptp_send_test_frame();
      tx_test_sent_once = 1U;
    }
#else
    if ((HAL_GetTick() - last_tx_ms) >= ETH_PTP_TX_TEST_INTERVAL_MS)
    {
      last_tx_ms = HAL_GetTick();
      if (s_link_up != 0U)
      {
        eth_ptp_send_test_frame();
      }
    }
#endif
#endif

#if (ETH_PTP_TESTS_ENABLE != 0U) && (ETH_PTP_TX_LEARN_ENABLE != 0U)
#if (ETH_PTP_TX_LEARN_ONCE != 0U)
    if (learn_sent_once == 0U)
    {
      eth_ptp_send_learn_frame();
      learn_sent_once = 1U;
    }
#else
    if ((HAL_GetTick() - last_learn_ms) >= ETH_PTP_TX_LEARN_INTERVAL_MS)
    {
      last_learn_ms = HAL_GetTick();
      eth_ptp_send_learn_frame();
    }
#endif
#endif
  }
}

void ETH_PTP_TaskStart(void)
{
  const osThreadAttr_t attr = {
    .name = "EthPtpTask",
    .priority = (osPriority_t)ETH_PTP_TASK_PRIORITY,
    .stack_size = ETH_PTP_TASK_STACK_SIZE
  };

  if (ETH_PTP_CONFIGURE_MPU_ETHRAM != 0U)
  {
    eth_ptp_mpu_config_ethram();
  }

  if (ETH_PTP_DISABLE_DCACHE != 0U)
  {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_DisableDCache();
    __DSB();
    __ISB();
#endif
  }

  if (s_eth_task_handle == NULL)
  {
    s_eth_task_handle = osThreadNew(ETH_PTP_Task, NULL, &attr);
  }
}










