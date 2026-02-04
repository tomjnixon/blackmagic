#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "lpc_common.h"


#define LPC18xx_SRAM_SIZE_MIN 32768U
#define LPC18xx_SRAM_IAP_SIZE 32U   // IAP routines use 32 bytes at top of ram

#define LPC18xx_IAP_ENTRYPOINT_LOCATION 0x10400100U
#define LPC18xx_IAP_RAM_BASE            0x10000000U

#define LPC18xx_IAP_PGM_CHUNKSIZE 4096U

#define LPC18xx_FLASH_NUM_SECTOR 30U

#define LPC18xx_MEMMAP   UINT32_C(0x400fc040)
#define LPC18xx_MPU_BASE UINT32_C(0xe000ed90)
#define LPC18xx_MPU_CTRL (LPC18xx_MPU_BASE + 0x04U)

typedef struct lpc18xx_priv {
	lpc_priv_s base;
	uint32_t mpu_ctrl_state;
	uint32_t memmap_state;
} lpc18xx_priv_s;

static void lpc18xx_extended_reset(target_s *target);
static bool lpc18xx_enter_flash_mode(target_s *target);
static bool lpc18xx_exit_flash_mode(target_s *target);
static bool lpc18xx_mass_erase(target_s *target, platform_timeout_s *print_progess);

static size_t lpc18xx_iap_params(iap_cmd_e cmd);

static void lpc18xx_add_flash(target_s *const target, const uint8_t bank, const uint8_t base_sector,
	const uint32_t addr, const size_t len, const size_t erasesize)
{
	lpc_flash_s *const flash = lpc_add_flash(target, addr, len, LPC18xx_IAP_PGM_CHUNKSIZE);
	flash->target_flash.blocksize = erasesize;
	// XXX
	// flash->target_flash.erase = lpc18xx_iap_flash_erase;
	flash->bank = bank;
	flash->base_sector = base_sector;
}

typedef struct {
	uint32_t id[2];
	uint8_t flash_chunks[2];
	uint8_t local_sram_1_chunks;
	uint8_t ahb_sram_chunks[2];
} part_t;

part_t parts[] = {
	{ .id = {0xF00B5B3F, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 2, .ahb_sram_chunks = {1, 0}}, // LPC1810
	{ .id = {0xF00BDB3F, 0x80}, .flash_chunks = {3, 0}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {1, 0}}, // LPC1812
	{ .id = {0xF00BDB3F, 0x44}, .flash_chunks = {1, 1}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {1, 0}}, // LPC1813
	{ .id = {0xF001DB3F, 0x22}, .flash_chunks = {2, 2}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1815
	{ .id = {0xF001DB3F, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1817
	{ .id = {0xF00ADB3C, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {1, 0}}, // LPC1820
	{ .id = {0xF00BDB3C, 0x80}, .flash_chunks = {3, 0}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {1, 0}}, // LPC1822
	{ .id = {0xF00BDB3C, 0x44}, .flash_chunks = {1, 1}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {1, 0}}, // LPC1823
	{ .id = {0xF001DB3C, 0x22}, .flash_chunks = {2, 2}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1825
	{ .id = {0xF001DB3C, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1827
	{ .id = {0xF000DA30, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {2, 1}}, // LPC1830
	{ .id = {0xF001DA30, 0x44}, .flash_chunks = {1, 1}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1833
	{ .id = {0xF001DA30, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1837
	{ .id = {0xF000D830, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {2, 1}}, // LPC1850
	{ .id = {0xF001D830, 0x44}, .flash_chunks = {1, 1}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1853
	{ .id = {0xF001D830, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC1857
	{ .id = {0xF00B5B6F, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {1, 0}}, // LPC18S10
	{ .id = {0xF00ADB6C, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {1, 0}}, // LPC18S20
	{ .id = {0xF000DA60, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {2, 1}}, // LPC18S30
	{ .id = {0xF001DA60, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC18S37
	{ .id = {0xF000D860, 0x00}, .flash_chunks = {0, 0}, .local_sram_1_chunks = 3, .ahb_sram_chunks = {2, 1}}, // LPC18S50
	{ .id = {0xF001D860, 0x00}, .flash_chunks = {3, 3}, .local_sram_1_chunks = 1, .ahb_sram_chunks = {2, 1}}, // LPC18S57
};

bool lpc18xx_probe(target_s *const target)
{
  DEBUG_ERROR("lpc18xx A\n");
	if ((target->cpuid & CORTEX_CPUID_PARTNO_MASK) != CORTEX_M3)
		return false;

	/*
	 * Now that we're sure it's a Cortex-M3, we need to halt the
	 * target and make an IAP call to get the part number.
	 * There appears to have no other method of reading the part number.
	 */
	target_halt_request(target);

	/* Allocate private storage so the flash mode entry/exit routines can save state */
	lpc18xx_priv_s *const priv = calloc(1, sizeof(*priv));
	if (!priv) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return false;
	}
	target->target_storage = priv;

	/* Set the structure up for this target */
	priv->base.iap_params = lpc18xx_iap_params;
	priv->base.iap_entry = target_mem32_read32(target, LPC18xx_IAP_ENTRYPOINT_LOCATION);
	priv->base.iap_ram = LPC18xx_IAP_RAM_BASE;
	priv->base.iap_msp = LPC18xx_IAP_RAM_BASE + LPC18xx_SRAM_SIZE_MIN - LPC18xx_SRAM_IAP_SIZE;

	/* Prepare Flash mode */
	lpc18xx_enter_flash_mode(target);
	/* Read the Part ID */
	iap_result_s result;
	lpc_iap_call(target, &result, IAP_CMD_PARTID);
	/* Transition back to normal mode and resume the target */
	lpc18xx_exit_flash_mode(target);
	target_halt_resume(target, false);

	/*
	 * If we got an error response, it cannot be a LPC18xx as the only response
	 * a real device gives is IAP_STATUS_CMD_SUCCESS.
	 */
	if (result.return_code) {
		DEBUG_ERROR("lpc18xx fail\n");
		free(priv);
		target->target_storage = NULL;
		return false;
	}

  DEBUG_ERROR("lpc18xx a, %x\n", result.values[0]);

	part_t *part = NULL;
	for (size_t i = 0; i < ARRAY_LENGTH(parts); i++) {
		if (result.values[0] == parts[i].id[0] && (0xff & result.values[1]) == parts[i].id[1]) {
			part = parts + i;
			break;
		}
	}

	if (part == NULL)
		return false;

	target->driver = "LPC18xx";
	target->extended_reset = lpc18xx_extended_reset;
	target->mass_erase = lpc18xx_mass_erase;
	target->enter_flash_mode = lpc18xx_enter_flash_mode;
	target->exit_flash_mode = lpc18xx_exit_flash_mode;

	// local SRAM
	target_add_ram32(target, 0x10000000U, part->local_sram_1_chunks * 0x8000U);
	target_add_ram32(target, 0x10080000U, 0xa000U);

	// AHB SRAM
	target_add_ram32(target, 0x20000000U, part->ahb_sram_chunks[0] * 0x4000U);
	if (part->ahb_sram_chunks[1])
		target_add_ram32(target, 0x20008000U, part->ahb_sram_chunks[1] * 0x4000U);

	// AHB/ETB SRAM
	target_add_ram32(target, 0x2000C000U, 0x4000U);

	uint32_t bank_starts[2] = {0x1A000000U, 0x1B000000U};

	for (uint8_t bank = 0; bank < 2; bank++) {
		switch (part->flash_chunks[bank]) {
			case 0:
				break;
			case 1:
				lpc18xx_add_flash(target, bank, 0U, bank_starts[bank],            0x10000U, 0x2000U);
				lpc18xx_add_flash(target, bank, 8U, bank_starts[bank] + 0x10000U, 0x30000U, 0x10000U);
				break;
			case 2:
				lpc18xx_add_flash(target, bank, 0U, bank_starts[bank],            0x10000U, 0x2000U);
				lpc18xx_add_flash(target, bank, 8U, bank_starts[bank] + 0x10000U, 0x50000U, 0x10000U);
				break;
			case 3:
				lpc18xx_add_flash(target, bank, 0U, bank_starts[bank],            0x10000U, 0x2000U);
				lpc18xx_add_flash(target, bank, 8U, bank_starts[bank] + 0x10000U, 0x70000U, 0x10000U);
				break;
		}
	}

	lpc_add_commands(target);
	return true;
}

static bool lpc18xx_enter_flash_mode(target_s *const target)
{
	lpc18xx_priv_s *const priv = (lpc18xx_priv_s *)target->target_storage;
	/* Disable the MPU, if enabled */
	priv->mpu_ctrl_state = target_mem32_read32(target, LPC18xx_MPU_CTRL);
	target_mem32_write32(target, LPC18xx_MPU_CTRL, 0);
	/* And store the memory mapping state */
	priv->memmap_state = target_mem32_read32(target, LPC18xx_MEMMAP);
	return true;
}

static bool lpc18xx_exit_flash_mode(target_s *const target)
{
	const lpc18xx_priv_s *const priv = (lpc18xx_priv_s *)target->target_storage;
	/* Restore the memory mapping and MPU state (in that order!) */
	target_mem32_write32(target, LPC18xx_MEMMAP, priv->memmap_state);
	target_mem32_write32(target, LPC18xx_MPU_CTRL, priv->mpu_ctrl_state);
	return true;
}

static bool lpc18xx_mass_erase(target_s *const target, platform_timeout_s *const print_progess)
{
	(void)print_progess;
	iap_result_s result;

	if (lpc_iap_call(target, &result, IAP_CMD_PREPARE, 0, LPC18xx_FLASH_NUM_SECTOR - 1U)) {
		DEBUG_ERROR("%s: prepare failed %" PRIu32 "\n", __func__, result.return_code);
		return false;
	}

	if (lpc_iap_call(target, &result, IAP_CMD_ERASE, 0, LPC18xx_FLASH_NUM_SECTOR - 1U, CPU_CLK_KHZ)) {
		DEBUG_ERROR("%s: erase failed %" PRIu32 "\n", __func__, result.return_code);
		return false;
	}

	if (lpc_iap_call(target, &result, IAP_CMD_BLANKCHECK, 0, LPC18xx_FLASH_NUM_SECTOR - 1U)) {
		DEBUG_ERROR("%s: blankcheck failed %" PRIu32 "\n", __func__, result.return_code);
		return false;
	}

	return true;
}

/*
 * Target has been reset, make sure to remap the boot ROM
 * from 0x00000000 leaving the user flash visible
 */
static void lpc18xx_extended_reset(target_s *const target)
{
	/*
	 * Transition the memory map to user mode (if it wasn't already) to ensure
	 * the correct environment is seen by the user
	 * See §33.6 Debug memory re-mapping, pg655 of UM10360 for more details.
	 */
	target_mem32_write32(target, LPC18xx_MEMMAP, 1);
}

static size_t lpc18xx_iap_params(const iap_cmd_e cmd)
{
	switch (cmd) {
	case IAP_CMD_PREPARE:
	case IAP_CMD_BLANKCHECK:
	case IAP_CMD_ERASE_PAGE:
		return 3U;
	case IAP_CMD_ERASE:
	case IAP_CMD_PROGRAM:
		return 4U;
	case IAP_CMD_SET_ACTIVE_BANK:
		return 2U;
	default:
		return 0U;
	}
}
