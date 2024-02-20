// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/bug.h>

#include "cam_cdm_intf_api.h"
#include "cam_cdm_util.h"
#include "cam_cdm.h"
#include "cam_io_util.h"

#define CAM_CDM_DWORD 4

#define CAM_CDM_SW_CMD_COUNT    2
#define CAM_CMD_LENGTH_MASK     0xFFFF
#define CAM_CDM_COMMAND_OFFSET  24
#define CAM_CDM_REG_OFFSET_MASK 0x00FFFFFF

#define CAM_CDM_DMI_DATA_HI_OFFSET   8
#define CAM_CDM_DMI_DATA_OFFSET      8
#define CAM_CDM_DMI_DATA_LO_OFFSET   12

static unsigned int CDMCmdHeaderSizes[
	CAM_CDM_CMD_PRIVATE_BASE + CAM_CDM_SW_CMD_COUNT] = {
	0, /* UNUSED*/
	3, /* DMI*/
	0, /* UNUSED*/
	2, /* RegContinuous*/
	1, /* RegRandom*/
	2, /* BUFFER_INDIREC*/
	2, /* GenerateIRQ*/
	3, /* WaitForEvent*/
	1, /* ChangeBase*/
	1, /* PERF_CONTROL*/
	3, /* DMI32*/
	3, /* DMI64*/
	3, /* WaitCompEvent*/
	3, /* ClearCompEvent*/
	3, /* WaitPrefetchDisable*/
};

/**
 * struct cdm_regrandom_cmd - Definition for CDM random register command.
 * @count: Number of register writes
 * @reserved: reserved bits
 * @cmd: Command ID (CDMCmd)
 */
struct cdm_regrandom_cmd {
	unsigned int count    : 16;
	unsigned int reserved : 8;
	unsigned int cmd      : 8;
} __attribute__((__packed__));

/**
 * struct cdm_regcontinuous_cmd - Definition for a CDM register range command.
 * @count: Number of register writes
 * @reserved0: reserved bits
 * @cmd: Command ID (CDMCmd)
 * @offset: Start address of the range of registers
 * @reserved1: reserved bits
 */
struct cdm_regcontinuous_cmd {
	unsigned int count     : 16;
	unsigned int reserved0 : 8;
	unsigned int cmd       : 8;
	unsigned int offset    : 24;
	unsigned int reserved1 : 8;
} __attribute__((__packed__));

/**
 * struct cdm_dmi_cmd - Definition for a CDM DMI command.
 * @length: Number of bytes in LUT - 1
 * @reserved: reserved bits
 * @cmd: Command ID (CDMCmd)
 * @addr: Address of the LUT in memory
 * @DMIAddr: Address of the target DMI config register
 * @DMISel: DMI identifier
 */
struct cdm_dmi_cmd {
	unsigned int length   : 16;
	unsigned int reserved : 8;
	unsigned int cmd      : 8;
	unsigned int addr;
	unsigned int DMIAddr  : 24;
	unsigned int DMISel   : 8;
} __attribute__((__packed__));

/**
 * struct cdm_indirect_cmd - Definition for a CDM indirect buffer command.
 * @length: Number of bytes in buffer - 1
 * @reserved: reserved bits
 * @cmd: Command ID (CDMCmd)
 * @addr:  Device address of the indirect buffer
 */
struct cdm_indirect_cmd {
	unsigned int length     : 16;
	unsigned int reserved   : 8;
	unsigned int cmd        : 8;
	unsigned int addr;
} __attribute__((__packed__));

/**
 * struct cdm_changebase_cmd - Definition for CDM base address change command.
 * @base: Base address to be changed to
 * @cmd:Command ID (CDMCmd)
 */
struct cdm_changebase_cmd {
	unsigned int base   : 24;
	unsigned int cmd    : 8;
} __attribute__((__packed__));

/**
 * struct cdm_wait_event_cmd - Definition for a CDM Gen IRQ command.
 * @mask: Mask for the events
 * @id: ID to read back for debug
 * @iw_reserved: reserved bits
 * @iw: iw AHB write bit
 * @cmd:Command ID (CDMCmd)
 * @offset: Offset to where data is written
 * @offset_reserved: reserved bits
 * @data: data returned in IRQ_USR_DATA
 */
struct cdm_wait_event_cmd {
	unsigned int mask             : 8;
	unsigned int id               : 8;
	unsigned int iw_reserved      : 7;
	unsigned int iw               : 1;
	unsigned int cmd              : 8;
	unsigned int offset           : 24;
	unsigned int offset_reserved  : 8;
	unsigned int data;
} __attribute__((__packed__));

/**
 * struct cdm_genirq_cmd - Definition for a CDM Wait event command.
 * @reserved: reserved bits
 * @cmd:Command ID (CDMCmd)
 * @userdata: userdata returned in IRQ_USR_DATA
 */
struct cdm_genirq_cmd {
	unsigned int reserved   : 24;
	unsigned int cmd        : 8;
	unsigned int userdata;
} __attribute__((__packed__));

/**
 * struct cdm_perf_ctrl_cmd_t - Definition for CDM perf control command.
 * @perf: perf command
 * @reserved: reserved bits
 * @cmd:Command ID (CDMCmd)
 */
struct cdm_perf_ctrl_cmd {
	unsigned int perf     : 2;
	unsigned int reserved : 22;
	unsigned int cmd      : 8;
} __attribute__((__packed__));

struct cdm_wait_comp_event_cmd {
	unsigned int reserved   : 8;
	unsigned int id         : 8;
	unsigned int id_reserved: 8;
	unsigned int cmd        : 8;
	unsigned int mask1;
	unsigned int mask2;
} __attribute__((__packed__));

struct cdm_clear_comp_event_cmd {
	unsigned int reserved   : 8;
	unsigned int id         : 8;
	unsigned int id_reserved: 8;
	unsigned int cmd        : 8;
	unsigned int mask1;
	unsigned int mask2;
} __attribute__((__packed__));

struct cdm_prefetch_disable_event_cmd {
	unsigned int reserved   : 8;
	unsigned int id         : 8;
	unsigned int id_reserved: 8;
	unsigned int cmd        : 8;
	unsigned int mask1;
	unsigned int mask2;
} __attribute__((__packed__));

uint32_t cdm_get_cmd_header_size(unsigned int command)
{
	return CDMCmdHeaderSizes[command];
}

uint32_t cdm_required_size_dmi(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_DMI);
}

uint32_t cdm_required_size_reg_continuous(uint32_t  numVals)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT) + numVals;
}

uint32_t cdm_required_size_reg_random(uint32_t numRegVals)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM) +
		(2 * numRegVals);
}

uint32_t cdm_required_size_indirect(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_BUFF_INDIRECT);
}

uint32_t cdm_required_size_genirq(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_GEN_IRQ);
}

uint32_t cdm_required_size_wait_event(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_WAIT_EVENT);
}

uint32_t cdm_required_size_changebase(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_CHANGE_BASE);
}

uint32_t cdm_required_size_comp_wait(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CMD_COMP_WAIT);
}

uint32_t cdm_required_size_clear_comp_event(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_CLEAR_COMP_WAIT);
}

uint32_t cdm_required_size_prefetch_disable(void)
{
	return cdm_get_cmd_header_size(CAM_CDM_WAIT_PREFETCH_DISABLE);
}

uint32_t cdm_offsetof_dmi_addr(void)
{
	return offsetof(struct cdm_dmi_cmd, addr);
}

uint32_t cdm_offsetof_indirect_addr(void)
{
	return offsetof(struct cdm_indirect_cmd, addr);
}

uint32_t *cdm_write_dmi(uint32_t *pCmdBuffer, uint8_t dmiCmd,
	uint32_t DMIAddr, uint8_t DMISel, uint32_t dmiBufferAddr,
	uint32_t length)
{
	struct cdm_dmi_cmd *pHeader = (struct cdm_dmi_cmd *)pCmdBuffer;

	pHeader->cmd        = CAM_CDM_CMD_DMI;
	pHeader->addr = dmiBufferAddr;
	pHeader->length = length;
	pHeader->DMIAddr = DMIAddr;
	pHeader->DMISel = DMISel;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_DMI);

	return pCmdBuffer;
}

uint32_t *cdm_write_regcontinuous(uint32_t *pCmdBuffer, uint32_t reg,
	uint32_t numVals, uint32_t *pVals)
{
	uint32_t i;
	struct cdm_regcontinuous_cmd *pHeader =
		(struct cdm_regcontinuous_cmd *)pCmdBuffer;

	pHeader->count = numVals;
	pHeader->cmd = CAM_CDM_CMD_REG_CONT;
	pHeader->reserved0 = 0;
	pHeader->reserved1 = 0;
	pHeader->offset = reg;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT);

	for (i = 0; i < numVals; i++)
		(((uint32_t *)pCmdBuffer)[i]) = (((uint32_t *)pVals)[i]);

	pCmdBuffer += numVals;

	return pCmdBuffer;
}

uint32_t *cdm_write_regrandom(uint32_t *pCmdBuffer, uint32_t numRegVals,
	uint32_t *pRegVals)
{
	uint32_t i;
	uint32_t *dst, *src;
	struct cdm_regrandom_cmd *pHeader =
		(struct cdm_regrandom_cmd *)pCmdBuffer;

	pHeader->count = numRegVals;
	pHeader->cmd = CAM_CDM_CMD_REG_RANDOM;
	pHeader->reserved = 0;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM);
	dst = pCmdBuffer;
	src = pRegVals;
	for (i = 0; i < numRegVals; i++) {
		*dst++ = *src++;
		*dst++ = *src++;
	}

	return dst;
}

uint32_t *cdm_write_indirect(uint32_t *pCmdBuffer, uint32_t indirectBufAddr,
	uint32_t length)
{
	struct cdm_indirect_cmd *pHeader =
		(struct cdm_indirect_cmd *)pCmdBuffer;

	pHeader->cmd = CAM_CDM_CMD_BUFF_INDIRECT;
	pHeader->addr = indirectBufAddr;
	pHeader->length = length - 1;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_BUFF_INDIRECT);

	return pCmdBuffer;
}

void cdm_write_genirq(uint32_t *pCmdBuffer, uint32_t userdata,
		bool bit_wr_enable, uint32_t fifo_idx)
{
	struct cdm_genirq_cmd *pHeader = (struct cdm_genirq_cmd *)pCmdBuffer;

	CAM_DBG(CAM_CDM, "userdata 0x%x, fifo_idx %d",
		userdata, fifo_idx);

	if (bit_wr_enable)
		pHeader->reserved = (unsigned int)((fifo_idx << 1)
			| (unsigned int)(bit_wr_enable));

	pHeader->cmd = CAM_CDM_CMD_GEN_IRQ;
	pHeader->userdata = (userdata << (8 * fifo_idx));
}

uint32_t *cdm_write_wait_event(uint32_t *pcmdbuffer, uint32_t iw,
	uint32_t id, uint32_t mask,
	uint32_t offset, uint32_t data)
{
	struct cdm_wait_event_cmd *pheader =
		(struct cdm_wait_event_cmd *)pcmdbuffer;

	pheader->cmd = CAM_CDM_CMD_WAIT_EVENT;
	pheader->mask = mask;
	pheader->data = data;
	pheader->id = id;
	pheader->iw = iw;
	pheader->offset = offset;
	pheader->iw_reserved = 0;
	pheader->offset_reserved = 0;

	pcmdbuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_WAIT_EVENT);

	return pcmdbuffer;
}

uint32_t *cdm_write_changebase(uint32_t *pCmdBuffer, uint32_t base)
{
	struct cdm_changebase_cmd *pHeader =
		(struct cdm_changebase_cmd *)pCmdBuffer;

	CAM_DBG(CAM_CDM, "Change to base 0x%x", base);

	pHeader->cmd = CAM_CDM_CMD_CHANGE_BASE;
	pHeader->base = base;
	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_CHANGE_BASE);

	return pCmdBuffer;
}

uint32_t *cdm_write_wait_comp_event(
	uint32_t *pCmdBuffer, uint32_t mask1, uint32_t mask2)
{
	struct cdm_wait_comp_event_cmd *pHeader =
		(struct cdm_wait_comp_event_cmd *)pCmdBuffer;

	pHeader->cmd = CAM_CDM_CMD_COMP_WAIT;
	pHeader->mask1 = mask1;
	pHeader->mask2 = mask2;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CMD_COMP_WAIT);

	return pCmdBuffer;
}

uint32_t *cdm_write_clear_comp_event(
	uint32_t *pCmdBuffer, uint32_t mask1, uint32_t mask2)
{
	struct cdm_clear_comp_event_cmd *pHeader =
		(struct cdm_clear_comp_event_cmd *)pCmdBuffer;

	pHeader->cmd = CAM_CDM_CLEAR_COMP_WAIT;
	pHeader->mask1 = mask1;
	pHeader->mask2 = mask2;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_CLEAR_COMP_WAIT);

	return pCmdBuffer;
}

uint32_t *cdm_write_wait_prefetch_disable(
	uint32_t                   *pCmdBuffer,
	uint32_t                    id,
	uint32_t                    mask1,
	uint32_t                    mask2)
{
	struct cdm_prefetch_disable_event_cmd *pHeader =
		(struct cdm_prefetch_disable_event_cmd *)pCmdBuffer;

	pHeader->cmd = CAM_CDM_WAIT_PREFETCH_DISABLE;
	pHeader->id  = id;
	pHeader->mask1 = mask1;
	pHeader->mask2 = mask2;

	pCmdBuffer += cdm_get_cmd_header_size(CAM_CDM_WAIT_PREFETCH_DISABLE);

	return pCmdBuffer;
}


struct cam_cdm_utils_ops CDM170_ops = {
	cdm_get_cmd_header_size,
	cdm_required_size_dmi,
	cdm_required_size_reg_continuous,
	cdm_required_size_reg_random,
	cdm_required_size_indirect,
	cdm_required_size_genirq,
	cdm_required_size_wait_event,
	cdm_required_size_changebase,
	cdm_required_size_comp_wait,
	cdm_required_size_clear_comp_event,
	cdm_required_size_prefetch_disable,
	cdm_offsetof_dmi_addr,
	cdm_offsetof_indirect_addr,
	cdm_write_dmi,
	cdm_write_regcontinuous,
	cdm_write_regrandom,
	cdm_write_indirect,
	cdm_write_genirq,
	cdm_write_wait_event,
	cdm_write_changebase,
	cdm_write_wait_comp_event,
	cdm_write_clear_comp_event,
	cdm_write_wait_prefetch_disable,
};

int cam_cdm_get_ioremap_from_base(uint32_t hw_base,
	uint32_t base_array_size,
	struct cam_soc_reg_map *base_table[CAM_SOC_MAX_BLOCK],
	void __iomem **device_base)
{
	int ret = -EINVAL, i;

	for (i = 0; i < base_array_size; i++) {
		if (base_table[i])
			CAM_DBG(CAM_CDM, "In loop %d ioremap for %x addr=%x",
			i, (base_table[i])->mem_cam_base, hw_base);
		if ((base_table[i]) &&
			((base_table[i])->mem_cam_base == hw_base)) {
			*device_base = (base_table[i])->mem_base;
			ret = 0;
			break;
		}
	}

	return ret;
}

static int cam_cdm_util_reg_cont_write(void __iomem *base_addr,
	uint32_t *cmd_buf, uint32_t cmd_buf_size, uint32_t *used_bytes)
{
	int ret = 0;
	uint32_t *data;
	struct cdm_regcontinuous_cmd *reg_cont;

	if ((cmd_buf_size < cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT)) ||
		(!base_addr)) {
		CAM_ERR(CAM_CDM, "invalid base addr and data length  %d %pK",
			cmd_buf_size, base_addr);
		return -EINVAL;
	}

	reg_cont = (struct cdm_regcontinuous_cmd *)cmd_buf;
	if ((!reg_cont->count) || (reg_cont->count > 0x10000) ||
		(((reg_cont->count * sizeof(uint32_t)) +
			cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT)) >
			cmd_buf_size)) {
		CAM_ERR(CAM_CDM, "buffer size %d is not sufficient for count%d",
			cmd_buf_size, reg_cont->count);
		return -EINVAL;
	}
	data = cmd_buf + cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT);
	cam_io_memcpy(base_addr + reg_cont->offset,	data,
		reg_cont->count * sizeof(uint32_t));

	*used_bytes = (reg_cont->count * sizeof(uint32_t)) +
		(4 * cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT));

	return ret;
}

static int cam_cdm_util_reg_random_write(void __iomem *base_addr,
	uint32_t *cmd_buf, uint32_t cmd_buf_size, uint32_t *used_bytes)
{
	uint32_t i;
	struct cdm_regrandom_cmd *reg_random;
	uint32_t *data;

	if (!base_addr) {
		CAM_ERR(CAM_CDM, "invalid base address");
		return -EINVAL;
	}

	reg_random = (struct cdm_regrandom_cmd *) cmd_buf;
	if ((!reg_random->count) || (reg_random->count > 0x10000) ||
		(((reg_random->count * (sizeof(uint32_t) * 2)) +
		cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM)) >
			cmd_buf_size)) {
		CAM_ERR(CAM_CDM, "invalid reg_count  %d cmd_buf_size %d",
			reg_random->count, cmd_buf_size);
		return -EINVAL;
	}
	data = cmd_buf + cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM);

	for (i = 0; i < reg_random->count; i++) {
		CAM_DBG(CAM_CDM, "reg random: offset %pK, value 0x%x",
			((void __iomem *)(base_addr + data[0])),
			data[1]);
		cam_io_w(data[1], base_addr + data[0]);
		data += 2;
	}

	*used_bytes = ((reg_random->count * (sizeof(uint32_t) * 2)) +
		(4 * cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM)));

	return 0;
}

static int cam_cdm_util_swd_dmi_write(uint32_t cdm_cmd_type,
	void __iomem *base_addr, uint32_t *cmd_buf, uint32_t cmd_buf_size,
	uint32_t *used_bytes)
{
	uint32_t i;
	struct cdm_dmi_cmd *swd_dmi;
	uint32_t *data;

	swd_dmi = (struct cdm_dmi_cmd *)cmd_buf;

	if (cmd_buf_size < (cdm_required_size_dmi() + swd_dmi->length + 1)) {
		CAM_ERR(CAM_CDM, "invalid CDM_SWD_DMI length %d",
			swd_dmi->length + 1);
		return -EINVAL;
	}
	data = cmd_buf + cdm_required_size_dmi();

	if (cdm_cmd_type == CAM_CDM_CMD_SWD_DMI_64) {
		for (i = 0; i < (swd_dmi->length + 1)/8; i++) {
			cam_io_w_mb(data[0], base_addr +
				swd_dmi->DMIAddr + CAM_CDM_DMI_DATA_LO_OFFSET);
			cam_io_w_mb(data[1], base_addr +
				swd_dmi->DMIAddr + CAM_CDM_DMI_DATA_HI_OFFSET);
			data += 2;
		}
	} else if (cdm_cmd_type == CAM_CDM_CMD_DMI) {
		for (i = 0; i < (swd_dmi->length + 1)/4; i++) {
			cam_io_w_mb(data[0], base_addr +
				swd_dmi->DMIAddr + CAM_CDM_DMI_DATA_OFFSET);
			data += 1;
		}
	} else {
		for (i = 0; i < (swd_dmi->length + 1)/4; i++) {
			cam_io_w_mb(data[0], base_addr +
				swd_dmi->DMIAddr + CAM_CDM_DMI_DATA_LO_OFFSET);
			data += 1;
		}
	}
	*used_bytes = (4 * cdm_required_size_dmi()) + swd_dmi->length + 1;

	return 0;
}

int cam_cdm_util_cmd_buf_write(void __iomem **current_device_base,
	uint32_t *cmd_buf, uint32_t cmd_buf_size,
	struct cam_soc_reg_map *base_table[CAM_SOC_MAX_BLOCK],
	uint32_t base_array_size, uint8_t bl_tag)
{
	int ret = 0;
	uint32_t cdm_cmd_type = 0, total_cmd_buf_size = 0;
	uint32_t used_bytes = 0;

	total_cmd_buf_size = cmd_buf_size;

	while (cmd_buf_size > 0) {
		CAM_DBG(CAM_CDM, "cmd data=%x", *cmd_buf);
		cdm_cmd_type = (*cmd_buf >> CAM_CDM_COMMAND_OFFSET);
		switch (cdm_cmd_type) {
		case CAM_CDM_CMD_REG_CONT: {
			ret = cam_cdm_util_reg_cont_write(*current_device_base,
				cmd_buf, cmd_buf_size, &used_bytes);
			if (ret)
				break;

			if (used_bytes > 0) {
				cmd_buf_size -= used_bytes;
				cmd_buf += used_bytes/4;
			}
			}
			break;
		case CAM_CDM_CMD_REG_RANDOM: {
			ret = cam_cdm_util_reg_random_write(
				*current_device_base, cmd_buf, cmd_buf_size,
				&used_bytes);
			if (ret)
				break;

			if (used_bytes > 0) {
				cmd_buf_size -= used_bytes;
				cmd_buf += used_bytes / 4;
			}
			}
			break;
		case CAM_CDM_CMD_DMI:
		case CAM_CDM_CMD_SWD_DMI_32:
		case CAM_CDM_CMD_SWD_DMI_64: {
			if (*current_device_base == 0) {
				CAM_ERR(CAM_CDM,
					"Got SWI DMI cmd =%d for invalid hw",
					cdm_cmd_type);
				ret = -EINVAL;
				break;
			}
			ret = cam_cdm_util_swd_dmi_write(cdm_cmd_type,
				*current_device_base, cmd_buf, cmd_buf_size,
				&used_bytes);
			if (ret)
				break;

			if (used_bytes > 0) {
				cmd_buf_size -= used_bytes;
				cmd_buf += used_bytes / 4;
			}
			}
			break;
		case CAM_CDM_CMD_CHANGE_BASE: {
			struct cdm_changebase_cmd *change_base_cmd =
				(struct cdm_changebase_cmd *)cmd_buf;

			ret = cam_cdm_get_ioremap_from_base(
				change_base_cmd->base, base_array_size,
				base_table, current_device_base);
			if (ret != 0) {
				CAM_ERR(CAM_CDM,
					"Get ioremap change base failed %x",
					change_base_cmd->base);
				break;
			}
			CAM_DBG(CAM_CDM, "Got ioremap for %x addr=%pK",
				change_base_cmd->base,
				current_device_base);
			cmd_buf_size -= (4 *
				cdm_required_size_changebase());
			cmd_buf += cdm_required_size_changebase();
			}
			break;
		default:
			CAM_ERR(CAM_CDM, "unsupported cdm_cmd_type type 0%x",
			cdm_cmd_type);
			ret = -EINVAL;
			break;
		}

		if (ret < 0)
			break;
	}

	return ret;
}

static long cam_cdm_util_dump_dmi_cmd(uint32_t *cmd_buf_addr,
	uint32_t *cmd_buf_addr_end)
{
	long ret = 0;
	struct cdm_dmi_cmd *p_dmi_cmd;
	uint32_t *temp_ptr = cmd_buf_addr;

	p_dmi_cmd = (struct cdm_dmi_cmd *)cmd_buf_addr;
	temp_ptr += CDMCmdHeaderSizes[CAM_CDM_CMD_DMI];
	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_DMI];

	if (temp_ptr > cmd_buf_addr_end)
		CAM_ERR(CAM_CDM,
			"Invalid cmd start addr:%pK end addr:%pK",
			temp_ptr, cmd_buf_addr_end);

	CAM_INFO(CAM_CDM,
		"DMI: LEN: %u DMIAddr: 0x%X DMISel: 0x%X LUT_addr: 0x%X",
		p_dmi_cmd->length, p_dmi_cmd->DMIAddr,
		p_dmi_cmd->DMISel, p_dmi_cmd->addr);
	return ret;
}

static long cam_cdm_util_dump_buff_indirect(uint32_t *cmd_buf_addr,
	uint32_t *cmd_buf_addr_end)
{
	long ret = 0;
	struct cdm_indirect_cmd *p_indirect_cmd;
	uint32_t *temp_ptr = cmd_buf_addr;

	p_indirect_cmd = (struct cdm_indirect_cmd *)cmd_buf_addr;
	temp_ptr += CDMCmdHeaderSizes[CAM_CDM_CMD_BUFF_INDIRECT];
	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_BUFF_INDIRECT];

	if (temp_ptr > cmd_buf_addr_end)
		CAM_ERR(CAM_CDM,
			"Invalid cmd start addr:%pK end addr:%pK",
			temp_ptr, cmd_buf_addr_end);

	CAM_INFO(CAM_CDM,
		"Buff Indirect: LEN: %u addr: 0x%X",
		p_indirect_cmd->length, p_indirect_cmd->addr);
	return ret;
}

static long cam_cdm_util_dump_reg_cont_cmd(uint32_t *cmd_buf_addr,
	uint32_t *cmd_buf_addr_end)
{
	long ret = 0;
	struct cdm_regcontinuous_cmd *p_regcont_cmd;
	uint32_t *temp_ptr = cmd_buf_addr;
	int i = 0;

	p_regcont_cmd = (struct cdm_regcontinuous_cmd *)temp_ptr;
	temp_ptr += CDMCmdHeaderSizes[CAM_CDM_CMD_REG_CONT];
	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_REG_CONT];

	CAM_INFO(CAM_CDM, "REG_CONT: COUNT: %u OFFSET: 0x%X",
		p_regcont_cmd->count, p_regcont_cmd->offset);

	for (i = 0; i < p_regcont_cmd->count; i++) {
		if (temp_ptr > cmd_buf_addr_end) {
			CAM_ERR(CAM_CDM,
				"Invalid cmd(%d) start addr:%pK end addr:%pK",
				i, temp_ptr, cmd_buf_addr_end);
			break;
		}
		CAM_INFO(CAM_CDM, "DATA_%d: 0x%X", i,
			*temp_ptr);
		temp_ptr++;
		ret++;
	}

	return ret;
}

static long cam_cdm_util_dump_reg_random_cmd(uint32_t *cmd_buf_addr,
	uint32_t *cmd_buf_addr_end)
{
	struct cdm_regrandom_cmd *p_regrand_cmd;
	uint32_t *temp_ptr = cmd_buf_addr;
	long ret = 0;
	int i = 0;

	p_regrand_cmd = (struct cdm_regrandom_cmd *)temp_ptr;
	temp_ptr += CDMCmdHeaderSizes[CAM_CDM_CMD_REG_RANDOM];
	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_REG_RANDOM];

	CAM_INFO(CAM_CDM, "REG_RAND: COUNT: %u",
		p_regrand_cmd->count);

	for (i = 0; i < p_regrand_cmd->count; i++) {
		if (temp_ptr > cmd_buf_addr_end) {
			CAM_ERR(CAM_CDM,
				"Invalid cmd(%d) start addr:%pK end addr:%pK",
				i, temp_ptr, cmd_buf_addr_end);
			break;
		}
		CAM_INFO(CAM_CDM, "OFFSET_%d: 0x%X DATA_%d: 0x%X",
			i, *temp_ptr & CAM_CDM_REG_OFFSET_MASK, i,
			*(temp_ptr + 1));
		temp_ptr += 2;
		ret += 2;
	}

	return ret;
}

static long cam_cdm_util_dump_gen_irq_cmd(uint32_t *cmd_buf_addr)
{
	long ret = 0;

	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_GEN_IRQ];

	CAM_INFO(CAM_CDM, "GEN_IRQ");

	return ret;
}

static long cam_cdm_util_dump_wait_event_cmd(uint32_t *cmd_buf_addr)
{
	long ret = 0;

	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_WAIT_EVENT];

	CAM_INFO(CAM_CDM, "WAIT_EVENT");

	return ret;
}

static long cam_cdm_util_dump_change_base_cmd(uint32_t *cmd_buf_addr,
	uint32_t *cmd_buf_addr_end)
{
	long ret = 0;
	struct cdm_changebase_cmd *p_cbase_cmd;
	uint32_t *temp_ptr = cmd_buf_addr;

	p_cbase_cmd = (struct cdm_changebase_cmd *)temp_ptr;
	temp_ptr += CDMCmdHeaderSizes[CAM_CDM_CMD_CHANGE_BASE];
	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_CHANGE_BASE];

	if (temp_ptr > cmd_buf_addr_end)
		CAM_ERR(CAM_CDM,
			"Invalid cmd start addr:%pK end addr:%pK",
			temp_ptr, cmd_buf_addr_end);

	CAM_INFO(CAM_CDM, "CHANGE_BASE: 0x%X",
		p_cbase_cmd->base);

	return ret;
}

static long cam_cdm_util_dump_comp_wait_event_cmd(uint32_t *cmd_buf_addr)
{
	long ret = 0;

	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_COMP_WAIT];

	CAM_INFO(CAM_CDM, "WAIT_EVENT");

	return ret;
}

static long cam_cdm_util_dump_perf_ctrl_cmd(uint32_t *cmd_buf_addr)
{
	long ret = 0;

	ret += CDMCmdHeaderSizes[CAM_CDM_CMD_PERF_CTRL];

	CAM_INFO(CAM_CDM, "PERF_CTRL");

	return ret;
}

void cam_cdm_util_dump_cmd_buf(
	uint32_t *cmd_buf_start, uint32_t *cmd_buf_end)
{
	uint32_t *buf_now = cmd_buf_start;
	uint32_t *buf_end = cmd_buf_end;
	uint32_t cmd = 0;

	if (!cmd_buf_start || !cmd_buf_end) {
		CAM_ERR(CAM_CDM, "Invalid args");
		return;
	}

	do {
		cmd = *buf_now;
		cmd = cmd >> CAM_CDM_COMMAND_OFFSET;

		switch (cmd) {
		case CAM_CDM_CMD_DMI:
		case CAM_CDM_CMD_DMI_32:
		case CAM_CDM_CMD_DMI_64:
			buf_now += cam_cdm_util_dump_dmi_cmd(buf_now,
				buf_end);
			break;
		case CAM_CDM_CMD_REG_CONT:
			buf_now += cam_cdm_util_dump_reg_cont_cmd(buf_now,
				buf_end);
			break;
		case CAM_CDM_CMD_REG_RANDOM:
			buf_now += cam_cdm_util_dump_reg_random_cmd(buf_now,
				buf_end);
			break;
		case CAM_CDM_CMD_BUFF_INDIRECT:
			buf_now += cam_cdm_util_dump_buff_indirect(buf_now,
				buf_end);
			break;
		case CAM_CDM_CMD_GEN_IRQ:
			buf_now += cam_cdm_util_dump_gen_irq_cmd(buf_now);
			break;
		case CAM_CDM_CMD_WAIT_EVENT:
			buf_now += cam_cdm_util_dump_wait_event_cmd(buf_now);
			break;
		case CAM_CDM_CMD_CHANGE_BASE:
			buf_now += cam_cdm_util_dump_change_base_cmd(buf_now,
				buf_end);
			break;
		case CAM_CDM_CMD_PERF_CTRL:
			buf_now += cam_cdm_util_dump_perf_ctrl_cmd(buf_now);
			break;
		case CAM_CDM_CMD_COMP_WAIT:
			buf_now +=
				cam_cdm_util_dump_comp_wait_event_cmd(buf_now);
			break;
		default:
			CAM_ERR(CAM_CDM, "Invalid CMD: 0x%x buf 0x%x",
				cmd, *buf_now);
			buf_now++;
			break;
		}
	} while (buf_now <= cmd_buf_end);
}

static uint32_t cam_cdm_util_dump_reg_cont_cmd_v2(
	uint32_t                         *cmd_buf_addr,
	struct cam_cdm_cmd_buf_dump_info *dump_info)
{
	int                             i;
	long                            ret;
	uint8_t                        *dst;
	size_t                          remain_len;
	uint32_t                       *temp_ptr = cmd_buf_addr;
	uint32_t                       *addr, *start;
	uint32_t                        min_len;
	struct cdm_regcontinuous_cmd   *p_regcont_cmd;
	struct cam_cdm_cmd_dump_header *hdr;

	p_regcont_cmd = (struct cdm_regcontinuous_cmd *)temp_ptr;
	temp_ptr += cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT);
	ret = cdm_get_cmd_header_size(CAM_CDM_CMD_REG_CONT);

	min_len = (sizeof(uint32_t) * p_regcont_cmd->count) +
		sizeof(struct cam_cdm_cmd_dump_header) +
		(2 * sizeof(uint32_t));
	remain_len = dump_info->dst_max_size - dump_info->dst_offset;

	if (remain_len < min_len) {
		CAM_WARN_RATE_LIMIT(CAM_CDM,
			"Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		return ret;
	}

	dst = (char *)dump_info->dst_start + dump_info->dst_offset;
	hdr = (struct cam_cdm_cmd_dump_header *)dst;
	scnprintf(hdr->tag, CAM_CDM_CMD_TAG_MAX_LEN, "CDM_REG_CONT:");
	hdr->word_size = sizeof(uint32_t);
	addr = (uint32_t *)(dst + sizeof(struct cam_cdm_cmd_dump_header));
	start = addr;
	*addr++ = p_regcont_cmd->offset;
	*addr++ = p_regcont_cmd->count;
	for (i = 0; i < p_regcont_cmd->count; i++) {
		*addr = *temp_ptr;
		temp_ptr++;
		addr++;
		ret++;
	}
	hdr->size = hdr->word_size * (addr - start);
	dump_info->dst_offset += hdr->size +
		sizeof(struct cam_cdm_cmd_dump_header);

	return ret;
}

static uint32_t cam_cdm_util_dump_reg_random_cmd_v2(
	uint32_t                         *cmd_buf_addr,
	struct cam_cdm_cmd_buf_dump_info *dump_info)
{
	int                             i;
	long                            ret;
	uint8_t                        *dst;
	uint32_t                       *temp_ptr = cmd_buf_addr;
	uint32_t                       *addr, *start;
	size_t                          remain_len;
	uint32_t                        min_len;
	struct cdm_regrandom_cmd       *p_regrand_cmd;
	struct cam_cdm_cmd_dump_header *hdr;

	p_regrand_cmd = (struct cdm_regrandom_cmd *)temp_ptr;
	temp_ptr += cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM);
	ret = cdm_get_cmd_header_size(CAM_CDM_CMD_REG_RANDOM);

	min_len = (2 * sizeof(uint32_t) * p_regrand_cmd->count) +
		sizeof(struct cam_cdm_cmd_dump_header) + sizeof(uint32_t);
	remain_len = dump_info->dst_max_size - dump_info->dst_offset;

	if (remain_len < min_len) {
		CAM_WARN_RATE_LIMIT(CAM_CDM,
			"Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		return ret;
	}

	dst = (char *)dump_info->dst_start + dump_info->dst_offset;
	hdr = (struct cam_cdm_cmd_dump_header *)dst;
	scnprintf(hdr->tag, CAM_CDM_CMD_TAG_MAX_LEN, "CDM_REG_RANDOM:");
	hdr->word_size = sizeof(uint32_t);
	addr = (uint32_t *)(dst + sizeof(struct cam_cdm_cmd_dump_header));
	start = addr;
	*addr++ = p_regrand_cmd->count;
	for (i = 0; i < p_regrand_cmd->count; i++) {
		addr[0] = temp_ptr[0] & CAM_CDM_REG_OFFSET_MASK;
		addr[1] = temp_ptr[1];
		temp_ptr += 2;
		addr += 2;
		ret += 2;
	}
	hdr->size = hdr->word_size * (addr - start);
	dump_info->dst_offset += hdr->size +
		sizeof(struct cam_cdm_cmd_dump_header);
	return ret;
}

int cam_cdm_util_dump_cmd_bufs_v2(
	struct cam_cdm_cmd_buf_dump_info *dump_info)
{
	uint32_t  cmd;
	uint32_t *buf_now;
	int rc = 0;

	if (!dump_info || !dump_info->src_start || !dump_info->src_end ||
		!dump_info->dst_start) {
		CAM_INFO(CAM_CDM, "Invalid args");
		return -EINVAL;
	}

	buf_now = dump_info->src_start;
	do {
		if (dump_info->dst_offset >= dump_info->dst_max_size) {
			CAM_WARN(CAM_CDM,
				"Dump overshoot offset %zu size %zu",
				dump_info->dst_offset,
				dump_info->dst_max_size);
			return -ENOSPC;
		}
		cmd = *buf_now;
		cmd = cmd >> CAM_CDM_COMMAND_OFFSET;

		switch (cmd) {
		case CAM_CDM_CMD_DMI:
		case CAM_CDM_CMD_DMI_32:
		case CAM_CDM_CMD_DMI_64:
			buf_now += cdm_get_cmd_header_size(CAM_CDM_CMD_DMI);
			break;
		case CAM_CDM_CMD_REG_CONT:
			buf_now += cam_cdm_util_dump_reg_cont_cmd_v2(buf_now,
				dump_info);
			break;
		case CAM_CDM_CMD_REG_RANDOM:
			buf_now += cam_cdm_util_dump_reg_random_cmd_v2(buf_now,
				dump_info);
			break;
		case CAM_CDM_CMD_BUFF_INDIRECT:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_BUFF_INDIRECT);
			break;
		case CAM_CDM_CMD_GEN_IRQ:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_GEN_IRQ);
			break;
		case CAM_CDM_CMD_WAIT_EVENT:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_WAIT_EVENT);
			break;
		case CAM_CDM_CMD_CHANGE_BASE:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_CHANGE_BASE);
			break;
		case CAM_CDM_CMD_PERF_CTRL:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_PERF_CTRL);
			break;
		case CAM_CDM_CMD_COMP_WAIT:
			buf_now += cdm_get_cmd_header_size(
				CAM_CDM_CMD_COMP_WAIT);
			break;
		default:
			CAM_ERR(CAM_CDM, "Invalid CMD: 0x%x", cmd);
			buf_now++;
			break;
		}
	} while (buf_now <= dump_info->src_end);
	return rc;
}
