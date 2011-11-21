/*
 * ALSA SoC OMAP ABE driver
 *
 * Author:	Laurent Le Faucheur <l-le-faucheur@ti.com>
 *		Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#include "abe_main.h"
#include "abe_ref.h"
#include "abe_dm_addr.h"

/*
 * initialize the default values for call-backs to subroutines
 * - FIFO IRQ call-backs for sequenced tasks
 * - FIFO IRQ call-backs for audio player/recorders (ping-pong protocols)
 * - Remote debugger interface
 * - Error monitoring
 * - Activity Tracing
 */

/**
 * abe_build_scheduler_table
 *
 */
void abe_build_scheduler_table()
{
	u16 i, n;
	u16 aUplinkMuxing[NBROUTE_UL];

#define ABE_TASK_ID(ID) (D_tasksList_ADDR + sizeof(ABE_STask)*(ID))
	/* LOAD OF THE TASKS' MULTIFRAME */
	/* WARNING ON THE LOCATION OF IO_MM_DL WHICH IS PATCHED
	   IN "abe_init_io_tasks" */
	memset(abe->MultiFrame, 0, sizeof(abe->MultiFrame));

	abe->MultiFrame[0][2] = ABE_TASK_ID(C_ABE_FW_TASK_IO_VX_DL);
#define TASK_ASRC_VX_DL_SLT 1
#define TASK_ASRC_VX_DL_IDX 2
	abe->MultiFrame[1][2] = ABE_TASK_ID(C_ABE_FW_TASK_ASRC_VX_DL_8);
#define TASK_VX_DL_SLT 1
#define TASK_VX_DL_IDX 3
	abe->MultiFrame[1][3] = ABE_TASK_ID(C_ABE_FW_TASK_VX_DL_8_48);
	abe->MultiFrame[1][6] = ABE_TASK_ID(C_ABE_FW_TASK_DL2Mixer);
	abe->MultiFrame[1][7] = ABE_TASK_ID(C_ABE_FW_TASK_IO_VIB_DL);
	abe->MultiFrame[2][0] = ABE_TASK_ID(C_ABE_FW_TASK_DL1Mixer);
	abe->MultiFrame[2][1] = ABE_TASK_ID(C_ABE_FW_TASK_SDTMixer);
	abe->MultiFrame[2][5] = ABE_TASK_ID(C_ABE_FW_TASK_IO_DMIC);
	abe->MultiFrame[3][0] = ABE_TASK_ID(C_ABE_FW_TASK_DL1_GAIN);
	abe->MultiFrame[3][6] = ABE_TASK_ID(C_ABE_FW_TASK_DL2_GAIN);
	abe->MultiFrame[3][7] = ABE_TASK_ID(C_ABE_FW_TASK_DL2_EQ);
	abe->MultiFrame[4][0] = ABE_TASK_ID(C_ABE_FW_TASK_DL1_EQ);
	abe->MultiFrame[4][2] = ABE_TASK_ID(C_ABE_FW_TASK_VXRECMixer);
	abe->MultiFrame[4][3] = ABE_TASK_ID(C_ABE_FW_TASK_VXREC_SPLIT);
	abe->MultiFrame[4][6] = ABE_TASK_ID(C_ABE_FW_TASK_VIBRA1);
	abe->MultiFrame[4][7] = ABE_TASK_ID(C_ABE_FW_TASK_VIBRA2);
	abe->MultiFrame[5][1] = ABE_TASK_ID(C_ABE_FW_TASK_EARP_48_96_LP);
	abe->MultiFrame[5][2] = ABE_TASK_ID(C_ABE_FW_TASK_IO_PDM_UL);
	abe->MultiFrame[5][7] = ABE_TASK_ID(C_ABE_FW_TASK_VIBRA_SPLIT);
	abe->MultiFrame[6][0] = ABE_TASK_ID(C_ABE_FW_TASK_EARP_48_96_LP);
	abe->MultiFrame[6][5] = ABE_TASK_ID(C_ABE_FW_TASK_EchoMixer);
	abe->MultiFrame[7][0] = ABE_TASK_ID(C_ABE_FW_TASK_IO_PDM_DL);
	abe->MultiFrame[7][2] = ABE_TASK_ID(C_ABE_FW_TASK_BT_UL_SPLIT);
	abe->MultiFrame[7][3] = ABE_TASK_ID(C_ABE_FW_TASK_DBG_SYNC);
	abe->MultiFrame[7][5] = ABE_TASK_ID(C_ABE_FW_TASK_ECHO_REF_SPLIT);
	abe->MultiFrame[8][2] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC1_96_48_LP);
	abe->MultiFrame[8][4] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC1_SPLIT);
	abe->MultiFrame[9][2] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC2_96_48_LP);
	abe->MultiFrame[9][4] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC2_SPLIT);
	abe->MultiFrame[9][6] = 0;
	abe->MultiFrame[9][7] = ABE_TASK_ID(C_ABE_FW_TASK_IHF_48_96_LP);
	abe->MultiFrame[10][2] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC3_96_48_LP);
	abe->MultiFrame[10][4] = ABE_TASK_ID(C_ABE_FW_TASK_DMIC3_SPLIT);
	abe->MultiFrame[10][7] = ABE_TASK_ID(C_ABE_FW_TASK_IHF_48_96_LP);
	abe->MultiFrame[11][2] = ABE_TASK_ID(C_ABE_FW_TASK_AMIC_96_48_LP);
	abe->MultiFrame[11][4] = ABE_TASK_ID(C_ABE_FW_TASK_AMIC_SPLIT);
	abe->MultiFrame[11][7] = ABE_TASK_ID(C_ABE_FW_TASK_VIBRA_PACK);
	abe->MultiFrame[12][3] = ABE_TASK_ID(C_ABE_FW_TASK_VX_UL_ROUTING);
	abe->MultiFrame[12][4] = ABE_TASK_ID(C_ABE_FW_TASK_ULMixer);
#define TASK_VX_UL_SLT 12
#define TASK_VX_UL_IDX 5
	abe->MultiFrame[12][5] = ABE_TASK_ID(C_ABE_FW_TASK_VX_UL_48_8);
	abe->MultiFrame[13][2] = ABE_TASK_ID(C_ABE_FW_TASK_MM_UL2_ROUTING);
	abe->MultiFrame[13][3] = ABE_TASK_ID(C_ABE_FW_TASK_SideTone);
	abe->MultiFrame[13][5] = ABE_TASK_ID(C_ABE_FW_TASK_IO_BT_VX_DL);
	abe->MultiFrame[14][3] = ABE_TASK_ID(C_ABE_FW_TASK_IO_DMIC);
#define TASK_BT_DL_48_8_SLT 14
#define TASK_BT_DL_48_8_IDX 4
	abe->MultiFrame[14][4] = ABE_TASK_ID(C_ABE_FW_TASK_BT_DL_48_8);
#define TASK_ASRC_BT_UL_SLT 15
#define TASK_ASRC_BT_UL_IDX 6
	abe->MultiFrame[15][0] = ABE_TASK_ID(C_ABE_FW_TASK_IO_MM_EXT_OUT);
	abe->MultiFrame[15][3] = ABE_TASK_ID(C_ABE_FW_TASK_IO_BT_VX_UL);
	abe->MultiFrame[15][6] = ABE_TASK_ID(C_ABE_FW_TASK_ASRC_BT_UL_8);
#define TASK_ASRC_VX_UL_SLT 16
#define TASK_ASRC_VX_UL_IDX 2
	abe->MultiFrame[16][2] = ABE_TASK_ID(C_ABE_FW_TASK_ASRC_VX_UL_8);
	abe->MultiFrame[16][3] = ABE_TASK_ID(C_ABE_FW_TASK_IO_VX_UL);
#define TASK_BT_UL_8_48_SLT 17
#define TASK_BT_UL_8_48_IDX 2
	abe->MultiFrame[17][2] = ABE_TASK_ID(C_ABE_FW_TASK_BT_UL_8_48);
#define TASK_IO_MM_UL2_SLT 17
#define TASK_IO_MM_UL2_IDX 3
	abe->MultiFrame[17][3] = 0;
#define TASK_IO_MM_DL_SLT 18
#define TASK_IO_MM_DL_IDX 0
	abe->MultiFrame[18][0] = 0;
#define TASK_ASRC_BT_DL_SLT 18
#define TASK_ASRC_BT_DL_IDX 6
	abe->MultiFrame[18][6] = ABE_TASK_ID(C_ABE_FW_TASK_ASRC_BT_DL_8);
	abe->MultiFrame[19][0] = ABE_TASK_ID(C_ABE_FW_TASK_IO_PDM_DL);
	/* MM_UL is moved to OPP 100% */
#define TASK_IO_MM_UL_SLT 19
#define TASK_IO_MM_UL_IDX 6
	abe->MultiFrame[19][6] = 0;
	abe->MultiFrame[20][0] = ABE_TASK_ID(C_ABE_FW_TASK_IO_TONES_DL);
	abe->MultiFrame[20][6] = ABE_TASK_ID(C_ABE_FW_TASK_ASRC_MM_EXT_IN);
	abe->MultiFrame[21][1] = ABE_TASK_ID(C_ABE_FW_TASK_DEBUGTRACE_VX_ASRCs);
	abe->MultiFrame[21][3] = ABE_TASK_ID(C_ABE_FW_TASK_IO_MM_EXT_IN);
	/* MUST STAY ON SLOT 22 */
	abe->MultiFrame[22][0] = ABE_TASK_ID(C_ABE_FW_TASK_DEBUG_IRQFIFO);
	abe->MultiFrame[22][1] = ABE_TASK_ID(C_ABE_FW_TASK_INIT_FW_MEMORY);
	abe->MultiFrame[22][2] = 0;
	/*
	 * MM_EXT_IN_SPLIT task must be after IO_MM_EXT_IN and before
	 * ASRC_MM_EXT_IN in order to manage OPP50 <-> transitions
	 */
	abe->MultiFrame[22][4] = ABE_TASK_ID(C_ABE_FW_TASK_MM_EXT_IN_SPLIT);
	abe->MultiFrame[23][0] = ABE_TASK_ID(C_ABE_FW_TASK_GAIN_UPDATE);

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_multiFrame_ADDR,
		       (u32 *) abe->MultiFrame, sizeof(abe->MultiFrame));

	/* reset the uplink router */
	n = (D_aUplinkRouting_sizeof) >> 1;
	for (i = 0; i < n; i++)
		aUplinkMuxing[i] = ZERO_labelID;

	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM, D_aUplinkRouting_ADDR,
		       (u32 *) aUplinkMuxing, sizeof(aUplinkMuxing));
}

/**
 * abe_init_atc
 * @id: ABE port ID
 *
 * load the DMEM ATC/AESS descriptors
 */
void abe_init_atc(u32 id)
{
	u8 iter;
	s32 datasize;
	abe_satcdescriptor_aess atc_desc;

#define JITTER_MARGIN 4
	/* load default values of the descriptor */
	memset(&atc_desc, 0, sizeof(atc_desc));
	datasize = abe_dma_port_iter_factor(&((abe_port[id]).format));
	iter = (u8) abe_dma_port_iteration(&((abe_port[id]).format));
	/* if the ATC FIFO is too small there will be two ABE firmware
	   utasks to do the copy this happems on DMIC and MCPDMDL */
	/* VXDL_8kMono = 4 = 2 + 2x1 */
	/* VXDL_16kstereo = 12 = 8 + 2x2 */
	/* MM_DL_1616 = 14 = 12 + 2x1 */
	/* DMIC = 84 = 72 + 2x6 */
	/* VXUL_8kMono = 2 */
	/* VXUL_16kstereo = 4 */
	/* MM_UL2_Stereo = 4 */
	/* PDMDL = 12 */
	/* IN from AESS point of view */
	if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN)
		if (iter + 2 * datasize > 126)
			atc_desc.wrpt = (iter >> 1) +
				((JITTER_MARGIN-1) * datasize);
		else
			atc_desc.wrpt = iter + ((JITTER_MARGIN-1) * datasize);
	else
		atc_desc.wrpt = 0 + ((JITTER_MARGIN+1) * datasize);

	switch ((abe_port[id]).protocol.protocol_switch) {
	case SLIMBUS_PORT_PROT:
		atc_desc.cbdir = (abe_port[id]).protocol.direction;
		atc_desc.cbsize =
			(abe_port[id]).protocol.p.prot_slimbus.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_slimbus.buf_addr1) >> 4;
		atc_desc.iter = (abe_port[id]).protocol.p.prot_slimbus.iter;
		atc_desc.srcid =
			abe_atc_srcid[(abe_port[id]).protocol.p.prot_slimbus.
				      desc_addr1 >> 3];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (abe_port[id]).protocol.p.prot_slimbus.
			       desc_addr1, (u32 *) &atc_desc, sizeof(atc_desc));
		atc_desc.badd =
			(abe_port[id]).protocol.p.prot_slimbus.buf_addr2;
		atc_desc.srcid =
			abe_atc_srcid[(abe_port[id]).protocol.p.prot_slimbus.
				      desc_addr2 >> 3];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (abe_port[id]).protocol.p.prot_slimbus.
			       desc_addr2, (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	case SERIAL_PORT_PROT:
		atc_desc.cbdir = (abe_port[id]).protocol.direction;
		atc_desc.cbsize =
			(abe_port[id]).protocol.p.prot_serial.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_serial.buf_addr) >> 4;
		atc_desc.iter = (abe_port[id]).protocol.p.prot_serial.iter;
		atc_desc.srcid =
			abe_atc_srcid[(abe_port[id]).protocol.p.prot_serial.
				      desc_addr >> 3];
		atc_desc.destid =
			abe_atc_dstid[(abe_port[id]).protocol.p.prot_serial.
				      desc_addr >> 3];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (abe_port[id]).protocol.p.prot_serial.desc_addr,
			       (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	case DMIC_PORT_PROT:
		atc_desc.cbdir = ABE_ATC_DIRECTION_IN;
		atc_desc.cbsize = (abe_port[id]).protocol.p.prot_dmic.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_dmic.buf_addr) >> 4;
		atc_desc.iter = DMIC_ITER;
		atc_desc.srcid = abe_atc_srcid[ABE_ATC_DMIC_DMA_REQ];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (ABE_ATC_DMIC_DMA_REQ*ATC_SIZE),
			       (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	case MCPDMDL_PORT_PROT:
		atc_desc.cbdir = ABE_ATC_DIRECTION_OUT;
		atc_desc.cbsize =
			(abe_port[id]).protocol.p.prot_mcpdmdl.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_mcpdmdl.buf_addr) >> 4;
		atc_desc.iter = MCPDM_DL_ITER;
		atc_desc.destid = abe_atc_dstid[ABE_ATC_MCPDMDL_DMA_REQ];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (ABE_ATC_MCPDMDL_DMA_REQ*ATC_SIZE),
			       (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	case MCPDMUL_PORT_PROT:
		atc_desc.cbdir = ABE_ATC_DIRECTION_IN;
		atc_desc.cbsize =
			(abe_port[id]).protocol.p.prot_mcpdmul.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_mcpdmul.buf_addr) >> 4;
		atc_desc.iter = MCPDM_UL_ITER;
		atc_desc.srcid = abe_atc_srcid[ABE_ATC_MCPDMUL_DMA_REQ];
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (ABE_ATC_MCPDMUL_DMA_REQ*ATC_SIZE),
			       (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	case PINGPONG_PORT_PROT:
		/* software protocol, nothing to do on ATC */
		break;
	case DMAREQ_PORT_PROT:
		atc_desc.cbdir = (abe_port[id]).protocol.direction;
		atc_desc.cbsize =
			(abe_port[id]).protocol.p.prot_dmareq.buf_size;
		atc_desc.badd =
			((abe_port[id]).protocol.p.prot_dmareq.buf_addr) >> 4;
		/* CBPr needs ITER=1.
		It is the job of eDMA to do the iterations */
		atc_desc.iter = 1;
		/* input from ABE point of view */
		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN) {
			/* atc_atc_desc.rdpt = 127; */
			/* atc_atc_desc.wrpt = 0; */
			atc_desc.srcid = abe_atc_srcid
				[(abe_port[id]).protocol.p.prot_dmareq.
				 desc_addr >> 3];
		} else {
			/* atc_atc_desc.rdpt = 0; */
			/* atc_atc_desc.wrpt = 127; */
			atc_desc.destid = abe_atc_dstid
				[(abe_port[id]).protocol.p.prot_dmareq.
				 desc_addr >> 3];
		}
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       (abe_port[id]).protocol.p.prot_dmareq.desc_addr,
			       (u32 *) &atc_desc, sizeof(atc_desc));
		break;
	}
}

/**
 * abe_init_dma_t
 * @ id: ABE port ID
 * @ prot: protocol being used
 *
 * load the dma_t with physical information from AE memory mapping
 */
void abe_init_dma_t(u32 id, abe_port_protocol_t *prot)
{
	abe_dma_t_offset dma;
	u32 idx;

	/* default dma_t points to address 0000... */
	dma.data = 0;
	dma.iter = 0;
	switch (prot->protocol_switch) {
	case PINGPONG_PORT_PROT:
		for (idx = 0; idx < 32; idx++) {
			if (((prot->p).prot_pingpong.irq_data) ==
			    (u32) (1 << idx))
				break;
		}
		(prot->p).prot_dmareq.desc_addr =
			((CBPr_DMA_RTX0 + idx)*ATC_SIZE);
		/* translate byte address/size in DMEM words */
		dma.data = (prot->p).prot_pingpong.buf_addr >> 2;
		dma.iter = (prot->p).prot_pingpong.buf_size >> 2;
		break;
	case DMAREQ_PORT_PROT:
		for (idx = 0; idx < 32; idx++) {
			if (((prot->p).prot_dmareq.dma_data) ==
			    (u32) (1 << idx))
				break;
		}
		dma.data = (CIRCULAR_BUFFER_PERIPHERAL_R__0 + (idx << 2));
		dma.iter = (prot->p).prot_dmareq.iter;
		(prot->p).prot_dmareq.desc_addr =
			((CBPr_DMA_RTX0 + idx)*ATC_SIZE);
		break;
	case SLIMBUS_PORT_PROT:
	case SERIAL_PORT_PROT:
	case DMIC_PORT_PROT:
	case MCPDMDL_PORT_PROT:
	case MCPDMUL_PORT_PROT:
	default:
		break;
	}
	/* upload the dma type */
	abe_port[id].dma = dma;
}

/**
 * abe_disenable_dma_request
 * Parameter:
 * Operations:
 * Return value:
 *	none
 */
void abe_disable_enable_dma_request(u32 id, u32 on_off)
{
	u8 desc_third_word[4], irq_dmareq_field;
	u32 sio_desc_address;
	u32 struct_offset;
	ABE_SIODescriptor sio_desc;
	ABE_SPingPongDescriptor desc_pp;

	if (abe_port[id].protocol.protocol_switch == PINGPONG_PORT_PROT) {
		irq_dmareq_field =
			(u8) (on_off *
			      abe_port[id].protocol.p.prot_pingpong.irq_data);
		sio_desc_address = D_PingPongDesc_ADDR;
		struct_offset = (u32) &(desc_pp.data_size) - (u32) &(desc_pp);
		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
			       sio_desc_address + struct_offset,
			       (u32 *) desc_third_word, 4);
		desc_third_word[2] = irq_dmareq_field;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       sio_desc_address + struct_offset,
			       (u32 *) desc_third_word, 4);
	} else {
		/* serial interface: sync ATC with Firmware activity */
		sio_desc_address =
			dmem_port_descriptors +
			(id * sizeof(ABE_SIODescriptor));
		abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
			sio_desc_address, (u32 *) &sio_desc,
			sizeof(sio_desc));
		if (on_off) {
			sio_desc.atc_irq_data =
				(u8) abe_port[id].protocol.p.prot_dmareq.
				dma_data;
			sio_desc.on_off = 0x80;
		} else {
			sio_desc.atc_irq_data = 0;
			sio_desc.on_off = 0;
		}
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			sio_desc_address, (u32 *) &sio_desc,
			sizeof(sio_desc));
	}

}

void abe_enable_dma_request(u32 id)
{
	abe_disable_enable_dma_request(id, 1);
}

/**
 * abe_disable_dma_request
 *
 * Parameter:
 * Operations:
 * Return value:
 *	none
 */
void abe_disable_dma_request(u32 id)
{
	abe_disable_enable_dma_request(id, 0);
}

/**
 * abe_enable_atc
 * Parameter:
 * Operations:
 * Return value:
 *	none
 */
void abe_enable_atc(u32 id)
{
	abe_satcdescriptor_aess atc_desc;

	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		       (abe_port[id]).protocol.p.prot_dmareq.desc_addr,
		       (u32 *) &atc_desc, sizeof(atc_desc));
	atc_desc.desen = 1;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		       (abe_port[id]).protocol.p.prot_dmareq.desc_addr,
		       (u32 *) &atc_desc, sizeof(atc_desc));

}

/**
 * abe_disable_atc
 * Parameter:
 * Operations:
 * Return value:
 *	none
 */
void abe_disable_atc(u32 id)
{
	abe_satcdescriptor_aess atc_desc;

	abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
		       (abe_port[id]).protocol.p.prot_dmareq.desc_addr,
		       (u32 *) &atc_desc, sizeof(atc_desc));
	atc_desc.desen = 0;
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
		       (abe_port[id]).protocol.p.prot_dmareq.desc_addr,
		       (u32 *) &atc_desc, sizeof(atc_desc));

}

/**
 * abe_init_io_tasks
 * @prot : protocol being used
 *
 * load the micro-task parameters doing to DMEM <==> SMEM data moves
 *
 * I/O descriptors input parameters :
 * For Read from DMEM usually THR1/THR2 = X+1/X-1
 * For Write to DMEM usually THR1/THR2 = 2/0
 * UP_1/2 =X+1/X-1
 */
void abe_init_io_tasks(u32 id, abe_data_format_t *format,
		       abe_port_protocol_t *prot)
{
	u32 x_io, direction, iter_samples, smem1, smem2, smem3, io_sub_id,
		io_flag;
	u32 copy_func_index, before_func_index, after_func_index;
	u32 dmareq_addr, dmareq_field;
	u32 sio_desc_address, datasize, iter, nsamp, datasize2, dOppMode32;
	u32 atc_ptr_saved, atc_ptr_saved2, copy_func_index1;
	u32 copy_func_index2, atc_desc_address1, atc_desc_address2;
	ABE_SPingPongDescriptor desc_pp;
	ABE_SIODescriptor sio_desc;

	if (prot->protocol_switch == PINGPONG_PORT_PROT) {
		/* ping_pong is only supported on MM_DL */
		if (MM_DL_PORT != id) {
			abe->dbg_param |= ERR_API;
			abe_dbg_error_log(ABE_PARAMETER_ERROR);
			return;
		}
		smem1 = smem_mm_dl;
		copy_func_index = (u8) abe_dma_port_copy_subroutine_id(id);
		dmareq_addr = abe_port[id].protocol.p.prot_pingpong.irq_addr;
		dmareq_field = abe_port[id].protocol.p.prot_pingpong.irq_data;
		datasize = abe_dma_port_iter_factor(format);
		/* number of "samples" either mono or stereo */
		iter = abe_dma_port_iteration(format);
		iter_samples = (iter / datasize);
		/* load the IO descriptor */
		/* no drift */
		desc_pp.drift_ASRC = 0;
		/* no drift */
		desc_pp.drift_io = 0;
		desc_pp.hw_ctrl_addr = (u16) dmareq_addr;
		desc_pp.copy_func_index = (u8) copy_func_index;
		desc_pp.smem_addr = (u8) smem1;
		/* DMA req 0 is used for CBPr0 */
		desc_pp.atc_irq_data = (u8) dmareq_field;
		/* size of block transfer */
		desc_pp.x_io = (u8) iter_samples;
		desc_pp.data_size = (u8) datasize;
		/* address comunicated in Bytes */
		desc_pp.workbuff_BaseAddr =
			(u16) (abe_base_address_pingpong[1]);
		/* size comunicated in XIO sample */
		desc_pp.workbuff_Samples = 0;
		desc_pp.nextbuff0_BaseAddr =
			(u16) (abe_base_address_pingpong[0]);
		desc_pp.nextbuff1_BaseAddr =
			(u16) (abe_base_address_pingpong[1]);
		if (dmareq_addr == ABE_DMASTATUS_RAW) {
			desc_pp.nextbuff0_Samples =
				(u16) ((abe_size_pingpong >> 2) / datasize);
			desc_pp.nextbuff1_Samples =
				(u16) ((abe_size_pingpong >> 2) / datasize);
		} else {
			desc_pp.nextbuff0_Samples = 0;
			desc_pp.nextbuff1_Samples = 0;
		}
		/* next buffer to send is B1, first IRQ fills B0 */
		desc_pp.counter = 0;
		/* send a DMA req to fill B0 with N samples
		   abe_block_copy (COPY_FROM_HOST_TO_ABE,
			ABE_ATC,
			ABE_DMASTATUS_RAW,
			&(abe_port[id].protocol.p.prot_pingpong.irq_data),
			4); */
		sio_desc_address = D_PingPongDesc_ADDR;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       sio_desc_address, (u32 *) &desc_pp,
			       sizeof(desc_pp));
	} else {
		io_sub_id = dmareq_addr = ABE_DMASTATUS_RAW;
		dmareq_field = 0;
		atc_desc_address1 = atc_desc_address2 = 0;
		/* default: repeat of the last downlink samples in case of
		   DMA errors, (disable=0x00) */
		io_flag = 0xFF;
		datasize2 = datasize = abe_dma_port_iter_factor(format);
		x_io = (u8) abe_dma_port_iteration(format);
		nsamp = (x_io / datasize);
		atc_ptr_saved2 = atc_ptr_saved = DMIC_ATC_PTR_labelID + id;
		smem1 = abe_port[id].smem_buffer1;
		smem3 = smem2 = abe_port[id].smem_buffer2;
		copy_func_index1 = (u8) abe_dma_port_copy_subroutine_id(id);
		before_func_index = after_func_index =
			copy_func_index2 = NULL_COPY_CFPID;
		switch (prot->protocol_switch) {
		case DMIC_PORT_PROT:
			/* DMIC port is read in two steps */
			x_io = x_io >> 1;
			nsamp = nsamp >> 1;
			atc_desc_address1 = (ABE_ATC_DMIC_DMA_REQ*ATC_SIZE);
			io_sub_id = IO_IP_CFPID;
			break;
		case MCPDMDL_PORT_PROT:
			/* PDMDL port is written to in two steps */
			x_io = x_io >> 1;
			atc_desc_address1 =
				(ABE_ATC_MCPDMDL_DMA_REQ*ATC_SIZE);
			io_sub_id = IO_IP_CFPID;
			break;
		case MCPDMUL_PORT_PROT:
			atc_desc_address1 =
				(ABE_ATC_MCPDMUL_DMA_REQ*ATC_SIZE);
			io_sub_id = IO_IP_CFPID;
			break;
		case SLIMBUS_PORT_PROT:
			atc_desc_address1 =
				abe_port[id].protocol.p.prot_slimbus.desc_addr1;
			atc_desc_address2 =
				abe_port[id].protocol.p.prot_slimbus.desc_addr2;
			copy_func_index2 = NULL_COPY_CFPID;
			/* @@@@@@
			   #define SPLIT_SMEM_CFPID 9
			   #define MERGE_SMEM_CFPID 10
			   #define SPLIT_TDM_12_CFPID 11
			   #define MERGE_TDM_12_CFPID 12
			 */
			io_sub_id = IO_IP_CFPID;
			break;
		case SERIAL_PORT_PROT:	/* McBSP/McASP */
			atc_desc_address1 =
				(s16) abe_port[id].protocol.p.prot_serial.
				desc_addr;
			io_sub_id = IO_IP_CFPID;
			break;
		case DMAREQ_PORT_PROT:	/* DMA w/wo CBPr */
			dmareq_addr =
				abe_port[id].protocol.p.prot_dmareq.dma_addr;
			dmareq_field = 0;
			atc_desc_address1 =
				abe_port[id].protocol.p.prot_dmareq.desc_addr;
			io_sub_id = IO_IP_CFPID;
			break;
		}
		/* special situation of the PING_PONG protocol which
		has its own SIO descriptor format */
		/*
		   Sequence of operations on ping-pong buffers B0/B1
		   -------------- time ---------------------------->>>>
		   Host Application is ready to send data from DDR to B0
		   SDMA is initialized from "abe_connect_irq_ping_pong_port" to B0
		   FIRMWARE starts with #12 B1 data,
		   sends IRQ/DMAreq, sends #pong B1 data,
		   sends IRQ/DMAreq, sends #ping B0,
		   sends B1 samples
		   ARM / SDMA | fills B0 | fills B1 ... | fills B0 ...
		   Counter 0 1 2 3
		 */
		if (MM_UL_PORT == id) {
			copy_func_index1 = COPY_MM_UL_CFPID;
			before_func_index = ROUTE_MM_UL_CFPID;
			abe->MultiFrame[TASK_IO_MM_UL_SLT]
				[TASK_IO_MM_UL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_IO_MM_UL);
		}
		if (MM_UL2_PORT == id) {
			abe->MultiFrame[TASK_IO_MM_UL2_SLT]
				[TASK_IO_MM_UL2_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_IO_MM_UL2);
		}

		/* check for 8kHz/16kHz */
		if (VX_DL_PORT == id) {
			if (abe_port[id].format.f == 8000) {
				abe->MultiFrame[TASK_ASRC_VX_DL_SLT]
					[TASK_ASRC_VX_DL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_ASRC_VX_DL_8);
				abe->MultiFrame[TASK_VX_DL_SLT][TASK_VX_DL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_VX_DL_8_48);
				/*Voice_8k_DL_labelID */
				smem1 = IO_VX_DL_ASRC_labelID;
			} else {
				abe->MultiFrame[TASK_ASRC_VX_DL_SLT]
					[TASK_ASRC_VX_DL_IDX] =
					ABE_TASK_ID
					(C_ABE_FW_TASK_ASRC_VX_DL_16);
				abe->MultiFrame[TASK_VX_DL_SLT][TASK_VX_DL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_VX_DL_16_48);
				/* Voice_16k_DL_labelID */
				smem1 = IO_VX_DL_ASRC_labelID;
			}
		}
		/* check for 8kHz/16kHz */
		if (VX_UL_PORT == id) {
			if (abe_port[id].format.f == 8000) {
				abe->MultiFrame[TASK_ASRC_VX_UL_SLT]
					[TASK_ASRC_VX_UL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_ASRC_VX_UL_8);
				abe->MultiFrame[TASK_VX_UL_SLT][TASK_VX_UL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_VX_UL_48_8);
				/* MultiFrame[TASK_ECHO_SLT][TASK_ECHO_IDX] =
				   ABE_TASK_ID(C_ABE_FW_TASK_ECHO_REF_48_8); */
				smem1 = Voice_8k_UL_labelID;
			} else {
				abe->MultiFrame[TASK_ASRC_VX_UL_SLT]
					[TASK_ASRC_VX_UL_IDX] =
					ABE_TASK_ID
					(C_ABE_FW_TASK_ASRC_VX_UL_16);
				abe->MultiFrame[TASK_VX_UL_SLT][TASK_VX_UL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_VX_UL_48_16);
				/* MultiFrame[TASK_ECHO_SLT][TASK_ECHO_IDX] =
				   ABE_TASK_ID(C_ABE_FW_TASK_ECHO_REF_48_16); */
				smem1 = Voice_16k_UL_labelID;
			}
		}
		/* check for 8kHz/16kHz */
		if (BT_VX_DL_PORT == id) {
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
				       D_maxTaskBytesInSlot_ADDR, &dOppMode32,
				       sizeof(u32));
			if (abe_port[id].format.f == 8000) {
				abe->MultiFrame[TASK_ASRC_BT_DL_SLT]
					[TASK_ASRC_BT_DL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_ASRC_BT_DL_8);
				if (dOppMode32 == DOPPMODE32_OPP100) {
					abe->MultiFrame[TASK_BT_DL_48_8_SLT]
						[TASK_BT_DL_48_8_IDX] =
						ABE_TASK_ID
						(C_ABE_FW_TASK_BT_DL_48_8_OPP100);
					smem1 = BT_DL_8k_opp100_labelID;
				} else {
					abe->MultiFrame[TASK_BT_DL_48_8_SLT]
						[TASK_BT_DL_48_8_IDX] =
						ABE_TASK_ID
						(C_ABE_FW_TASK_BT_DL_48_8);
					smem1 = BT_DL_8k_labelID;
				}
			} else {
				abe->MultiFrame[TASK_ASRC_BT_DL_SLT]
					[TASK_ASRC_BT_DL_IDX] =
					ABE_TASK_ID
					(C_ABE_FW_TASK_ASRC_BT_DL_16);
				if (dOppMode32 == DOPPMODE32_OPP100) {
					abe->MultiFrame[TASK_BT_DL_48_8_SLT]
						[TASK_BT_DL_48_8_IDX] =
						ABE_TASK_ID
						(C_ABE_FW_TASK_BT_DL_48_16_OPP100);
					smem1 = BT_DL_16k_opp100_labelID;
				} else {
					abe->MultiFrame[TASK_BT_DL_48_8_SLT]
						[TASK_BT_DL_48_8_IDX] =
						ABE_TASK_ID
						(C_ABE_FW_TASK_BT_DL_48_16);
					smem1 = BT_DL_16k_labelID;
				}
			}
		}
		/* check for 8kHz/16kHz */
		if (BT_VX_UL_PORT == id) {
			/* set the SMEM buffer -- programming sequence */
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
				       D_maxTaskBytesInSlot_ADDR, &dOppMode32,
				       sizeof(u32));
			if (abe_port[id].format.f == 8000) {
				abe->MultiFrame[TASK_ASRC_BT_UL_SLT]
					[TASK_ASRC_BT_UL_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_ASRC_BT_UL_8);
				abe->MultiFrame[TASK_BT_UL_8_48_SLT]
					[TASK_BT_UL_8_48_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_BT_UL_8_48);
				if (dOppMode32 == DOPPMODE32_OPP100)
					/* ASRC input buffer, size 40 */
					smem1 = smem_bt_vx_ul_opp100;
				else
					/* at OPP 50 without ASRC */
					smem1 = BT_UL_8k_labelID;
			} else {
				abe->MultiFrame[TASK_ASRC_BT_UL_SLT]
					[TASK_ASRC_BT_UL_IDX] =
					ABE_TASK_ID
					(C_ABE_FW_TASK_ASRC_BT_UL_16);
				abe->MultiFrame[TASK_BT_UL_8_48_SLT]
					[TASK_BT_UL_8_48_IDX] =
					ABE_TASK_ID(C_ABE_FW_TASK_BT_UL_16_48);
				if (dOppMode32 == DOPPMODE32_OPP100)
					/* ASRC input buffer, size 40 */
					smem1 = smem_bt_vx_ul_opp100;
				else
					/* at OPP 50 without ASRC */
					smem1 = BT_UL_16k_labelID;
			}
		}
		if (MM_DL_PORT == id) {
			/* check for CBPr / serial_port / Ping-pong access */
			abe->MultiFrame[TASK_IO_MM_DL_SLT][TASK_IO_MM_DL_IDX] =
				ABE_TASK_ID(C_ABE_FW_TASK_IO_MM_DL);
			smem1 = smem_mm_dl;
		}
		if (MM_EXT_IN_PORT == id) {
			/* set the SMEM buffer -- programming sequence */
			abe_block_copy(COPY_FROM_ABE_TO_HOST, ABE_DMEM,
				       D_maxTaskBytesInSlot_ADDR, &dOppMode32,
				       sizeof(u32));
			if (dOppMode32 == DOPPMODE32_OPP100)
				/* ASRC input buffer, size 40 */
				smem1 = smem_mm_ext_in_opp100;
			else
				/* at OPP 50 without ASRC */
				smem1 = smem_mm_ext_in_opp50;
		}
		if (abe_port[id].protocol.direction == ABE_ATC_DIRECTION_IN)
			direction = 0;
		else
			/* offset of the write pointer in the ATC descriptor */
			direction = 3;
		sio_desc.drift_ASRC = 0;
		sio_desc.drift_io = 0;
		sio_desc.io_type_idx = (u8) io_sub_id;
		sio_desc.samp_size = (u8) datasize;
		sio_desc.hw_ctrl_addr = (u16) (dmareq_addr << 2);
		sio_desc.atc_irq_data = (u8) dmareq_field;
		sio_desc.flow_counter = (u16) 0;
		sio_desc.direction_rw = (u8) direction;
		sio_desc.repeat_last_samp = (u8) io_flag;
		sio_desc.nsamp = (u8) nsamp;
		sio_desc.x_io = (u8) x_io;
		/* set ATC ON */
		sio_desc.on_off = 0x80;
		sio_desc.split_addr1 = (u16) smem1;
		sio_desc.split_addr2 = (u16) smem2;
		sio_desc.split_addr3 = (u16) smem3;
		sio_desc.before_f_index = (u8) before_func_index;
		sio_desc.after_f_index = (u8) after_func_index;
		sio_desc.smem_addr1 = (u16) smem1;
		sio_desc.atc_address1 = (u16) atc_desc_address1;
		sio_desc.atc_pointer_saved1 = (u16) atc_ptr_saved;
		sio_desc.data_size1 = (u8) datasize;
		sio_desc.copy_f_index1 = (u8) copy_func_index1;
		sio_desc.smem_addr2 = (u16) smem2;
		sio_desc.atc_address2 = (u16) atc_desc_address2;
		sio_desc.atc_pointer_saved2 = (u16) atc_ptr_saved2;
		sio_desc.data_size2 = (u8) datasize2;
		sio_desc.copy_f_index2 = (u8) copy_func_index2;
		sio_desc_address = dmem_port_descriptors + (id *
				sizeof(ABE_SIODescriptor));
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
				sio_desc_address, (u32 *) &sio_desc,
				sizeof(sio_desc));

		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       D_multiFrame_ADDR,
			       (u32 *) abe->MultiFrame,
			       sizeof(abe->MultiFrame));
	}

}

/**
 * abe_enable_pp_io_task
 * @id: port_id
 *
 *
 */
void abe_enable_pp_io_task(u32 id)
{
	if (MM_DL_PORT == id) {
		/* MM_DL managed in ping-pong */
		abe->MultiFrame[TASK_IO_MM_DL_SLT][TASK_IO_MM_DL_IDX] =
			ABE_TASK_ID(C_ABE_FW_TASK_IO_PING_PONG);
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       D_multiFrame_ADDR, (u32 *) abe->MultiFrame,
			       sizeof(abe->MultiFrame));
	} else {
		/* ping_pong is only supported on MM_DL */
		abe->dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}
}

/**
 * abe_disable_pp_io_task
 * @id: port_id
 *
 *
 */
void abe_disable_pp_io_task(u32 id)
{
	if (MM_DL_PORT == id) {
		/* MM_DL managed in ping-pong */
		abe->MultiFrame[TASK_IO_MM_DL_SLT][TASK_IO_MM_DL_IDX] = 0;
		abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_DMEM,
			       D_multiFrame_ADDR, (u32 *) abe->MultiFrame,
			       sizeof(abe->MultiFrame));
	} else {
		/* ping_pong is only supported on MM_DL */
		abe->dbg_param |= ERR_API;
		abe_dbg_error_log(ABE_PARAMETER_ERROR);
	}
}

/**
 * abe_init_dmic
 * @x: d
 *
 *
 */
void abe_init_dmic(u32 x)
{
}

/**
 * abe_init_mcpdm
 * @x: d
 *
 */
void abe_init_mcpdm(u32 x)
{
}

/**
 * abe_reset_feature
 * @x: index of the feature to be initialized
 *
 * reload the configuration
 */
void abe_reset_one_feature(u32 x)
{
	all_feature[x] = all_feature_init[x];	/* load default fields */
	/* abe_call_subroutine ((all_feature[x]).disable_feature, NOPARAMETER,
	   NOPARAMETER, NOPARAMETER, NOPARAMETER); */
}

/**
 * abe_reset_all_feature
 *
 * load default configuration for all features
 * struct {
 *		uint16 load_default_data;
 *		uint16 read_parameter;
 *		uint16 write_parameter;
 *		uint16 running_status;
 *		uint16 fw_input_buffer_address;
 *		uint16 fw_output_buffer_address;
 *		uint16 fw_scheduler_slot_position;
 *		uint16 fw_scheduler_subslot_position;
 *		uint16 min_opp;
 *		char name[NBCHARFEATURENAME];
 * } abe_feature_t;
 */
void abe_reset_all_features(void)
{
	u16 i;
	for (i = 0; i < MAXNBFEATURE; i++)
		abe_reset_one_feature(i);
}

/**
 * abe_reset_all_ports
 *
 * load default configuration for all features
 */
void abe_reset_all_ports(void)
{
	u16 i;

	for (i = 0; i < LAST_PORT_ID; i++)
		abe_reset_port(i);
	/* mixers' configuration */
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_100MS, MIX_DL1_INPUT_MM_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_100MS, MIX_DL1_INPUT_MM_UL2);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_100MS, MIX_DL1_INPUT_VX_DL);
	abe_write_mixer(MIXDL1, MUTE_GAIN, RAMP_100MS, MIX_DL1_INPUT_TONES);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_TONES);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_VX_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_MM_DL);
	abe_write_mixer(MIXDL2, MUTE_GAIN, RAMP_100MS, MIX_DL2_INPUT_MM_UL2);
	abe_write_mixer(MIXSDT, MUTE_GAIN, RAMP_100MS, MIX_SDT_INPUT_UP_MIXER);
	abe_write_mixer(MIXSDT, GAIN_0dB, RAMP_100MS, MIX_SDT_INPUT_DL1_MIXER);
	abe_write_mixer(MIXECHO, MUTE_GAIN, RAMP_100MS, MIX_ECHO_DL1);
	abe_write_mixer(MIXECHO, MUTE_GAIN, RAMP_100MS, MIX_ECHO_DL2);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_MM_DL);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_TONES);
	abe_write_mixer(MIXAUDUL, GAIN_0dB, RAMP_100MS, MIX_AUDUL_INPUT_UPLINK);
	abe_write_mixer(MIXAUDUL, MUTE_GAIN, RAMP_100MS, MIX_AUDUL_INPUT_VX_DL);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_100MS, MIX_VXREC_INPUT_TONES);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_100MS, MIX_VXREC_INPUT_VX_DL);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_100MS, MIX_VXREC_INPUT_MM_DL);
	abe_write_mixer(MIXVXREC, MUTE_GAIN, RAMP_100MS, MIX_VXREC_INPUT_VX_UL);
	abe_write_gain(GAINS_DMIC1, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC1, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DMIC2, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC2, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DMIC3, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DMIC3, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_AMIC, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_AMIC, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_SPLIT, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL1, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_DL2, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
	abe_write_gain(GAINS_BTUL, GAIN_0dB, RAMP_100MS, GAIN_LEFT_OFFSET);
	abe_write_gain(GAINS_BTUL, GAIN_0dB, RAMP_100MS, GAIN_RIGHT_OFFSET);
}

/**
 * abe_clean_temporay buffers
 *
 * clear temporary buffers
 */
void abe_clean_temporary_buffers(u32 id)
{
	switch (id) {
	case DMIC_PORT:
		abe_reset_mem(ABE_DMEM, D_DMIC_UL_FIFO_ADDR,
			D_DMIC_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_DMIC0_96_48_data_ADDR << 3,
			S_DMIC0_96_48_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DMIC1_96_48_data_ADDR << 3,
			S_DMIC1_96_48_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DMIC2_96_48_data_ADDR << 3,
			S_DMIC2_96_48_data_sizeof << 3);
		/* reset working values of the gain, target gain is preserved */
		abe_reset_gain_mixer(GAINS_DMIC1, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
		abe_reset_gain_mixer(GAINS_DMIC2, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
		abe_reset_gain_mixer(GAINS_DMIC3, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_DMIC3, GAIN_RIGHT_OFFSET);
		break;
	case PDM_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_McPDM_UL_FIFO_ADDR,
			D_McPDM_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_AMIC_96_48_data_ADDR << 3,
			S_AMIC_96_48_data_sizeof << 3);
		/* reset working values of the gain, target gain is preserved */
		abe_reset_gain_mixer(GAINS_AMIC, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_AMIC, GAIN_RIGHT_OFFSET);
		break;
	case BT_VX_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_BT_UL_FIFO_ADDR, D_BT_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_BT_UL_ADDR << 3, S_BT_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_UL_8_48_HP_data_ADDR << 3,
			      S_BT_UL_8_48_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_UL_8_48_LP_data_ADDR << 3,
			      S_BT_UL_8_48_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_UL_16_48_HP_data_ADDR << 3,
			      S_BT_UL_16_48_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_UL_16_48_LP_data_ADDR << 3,
			      S_BT_UL_16_48_LP_data_sizeof << 3);
		/* reset working values of the gain, target gain is preserved */
		abe_reset_gain_mixer(GAINS_BTUL, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_BTUL, GAIN_RIGHT_OFFSET);
		break;
	case MM_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_UL_FIFO_ADDR, D_MM_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_UL_ADDR << 3, S_MM_UL_sizeof << 3);
		break;
	case MM_UL2_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_UL2_FIFO_ADDR,
			      D_MM_UL2_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_UL2_ADDR << 3,
			      S_MM_UL2_sizeof << 3);
		break;
	case VX_UL_PORT:
		abe_reset_mem(ABE_DMEM, D_VX_UL_FIFO_ADDR, D_VX_UL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VX_UL_ADDR << 3, S_VX_UL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_8_HP_data_ADDR << 3,
			      S_VX_UL_48_8_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_8_LP_data_ADDR << 3,
			      S_VX_UL_48_8_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_16_HP_data_ADDR << 3,
			      S_VX_UL_48_16_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_UL_48_16_LP_data_ADDR << 3,
			      S_VX_UL_48_16_LP_data_sizeof << 3);
		abe_reset_gain_mixer(MIXAUDUL, MIX_AUDUL_INPUT_UPLINK);
		break;
	case MM_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_DL_FIFO_ADDR, D_MM_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_MM_DL_ADDR << 3, S_MM_DL_sizeof << 3);
		abe_reset_gain_mixer(MIXDL1, MIX_DL1_INPUT_MM_DL);
		abe_reset_gain_mixer(MIXDL2, MIX_DL2_INPUT_MM_DL);
		break;
	case VX_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_VX_DL_FIFO_ADDR, D_VX_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VX_DL_ADDR << 3, S_VX_DL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_8_48_HP_data_ADDR << 3,
			      S_VX_DL_8_48_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_8_48_LP_data_ADDR << 3,
			      S_VX_DL_8_48_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_16_48_HP_data_ADDR << 3,
			      S_VX_DL_16_48_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_VX_DL_16_48_LP_data_ADDR << 3,
			      S_VX_DL_16_48_LP_data_sizeof << 3);
		abe_reset_gain_mixer(MIXDL1, MIX_DL1_INPUT_VX_DL);
		abe_reset_gain_mixer(MIXDL2, MIX_DL2_INPUT_VX_DL);
		break;
	case TONES_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_TONES_DL_FIFO_ADDR,
			      D_TONES_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_Tones_ADDR << 3, S_Tones_sizeof << 3);
		abe_reset_gain_mixer(MIXDL1, MIX_DL1_INPUT_TONES);
		abe_reset_gain_mixer(MIXDL2, MIX_DL2_INPUT_TONES);
		break;
	case VIB_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_VIB_DL_FIFO_ADDR,
			      D_VIB_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_VIBRA_ADDR << 3, S_VIBRA_sizeof << 3);
		break;
	case BT_VX_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_BT_DL_FIFO_ADDR, D_BT_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_BT_DL_ADDR << 3, S_BT_DL_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_DL_48_8_HP_data_ADDR << 3,
			      S_BT_DL_48_8_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_DL_48_8_LP_data_ADDR << 3,
			      S_BT_DL_48_8_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_DL_48_16_HP_data_ADDR << 3,
			      S_BT_DL_48_16_HP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_BT_DL_48_16_LP_data_ADDR << 3,
			      S_BT_DL_48_16_LP_data_sizeof << 3);
		break;
	case PDM_DL_PORT:
		abe_reset_mem(ABE_DMEM, D_McPDM_DL_FIFO_ADDR,
			      D_McPDM_DL_FIFO_sizeof);
		abe_reset_mem(ABE_SMEM, S_DL2_M_LR_EQ_data_ADDR << 3,
			      S_DL2_M_LR_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_DL1_M_EQ_data_ADDR << 3,
			      S_DL1_M_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_EARP_48_96_LP_data_ADDR << 3,
			      S_EARP_48_96_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_IHF_48_96_LP_data_ADDR << 3,
			      S_IHF_48_96_LP_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL1_EQ_data_ADDR << 3,
			      S_APS_DL1_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL2_EQ_data_ADDR << 3,
			      S_APS_DL2_EQ_data_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL2_L_IIRmem1_ADDR << 3,
			      S_APS_DL2_L_IIRmem1_sizeof << 3);
		abe_reset_mem(ABE_SMEM, S_APS_DL2_R_IIRmem1_ADDR << 3,
			      S_APS_DL2_R_IIRmem1_sizeof << 3);
		abe_reset_gain_mixer(GAINS_DL1, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_DL1, GAIN_RIGHT_OFFSET);
		abe_reset_gain_mixer(GAINS_DL2, GAIN_LEFT_OFFSET);
		abe_reset_gain_mixer(GAINS_DL2, GAIN_RIGHT_OFFSET);
		abe_reset_gain_mixer(MIXSDT, MIX_SDT_INPUT_UP_MIXER);
		abe_reset_gain_mixer(MIXSDT, MIX_SDT_INPUT_DL1_MIXER);
		break;
	case MM_EXT_OUT_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_EXT_OUT_FIFO_ADDR,
			      D_MM_EXT_OUT_FIFO_sizeof);
		break;
	case MM_EXT_IN_PORT:
		abe_reset_mem(ABE_DMEM, D_MM_EXT_IN_FIFO_ADDR,
			      D_MM_EXT_IN_FIFO_sizeof);
		break;
	}
}

/**
 * abe_reset_gain_mixer
 * @id: name of the mixer
 * @p: list of port corresponding to the above gains
 *
 * restart the working gain value of the mixers when a port is enabled
 */
void abe_reset_gain_mixer(u32 id, u32 p)
{
	u32 lin_g, mixer_target, mixer_offset;
	switch (id) {
	default:
	case GAINS_DMIC1:
		mixer_offset = dmic1_gains_offset;
		break;
	case GAINS_DMIC2:
		mixer_offset = dmic2_gains_offset;
		break;
	case GAINS_DMIC3:
		mixer_offset = dmic3_gains_offset;
		break;
	case GAINS_AMIC:
		mixer_offset = amic_gains_offset;
		break;
	case GAINS_DL1:
		mixer_offset = dl1_gains_offset;
		break;
	case GAINS_DL2:
		mixer_offset = dl2_gains_offset;
		break;
	case GAINS_SPLIT:
		mixer_offset = splitters_gains_offset;
		break;
	case MIXDL1:
		mixer_offset = mixer_dl1_offset;
		break;
	case MIXDL2:
		mixer_offset = mixer_dl2_offset;
		break;
	case MIXECHO:
		mixer_offset = mixer_echo_offset;
		break;
	case MIXSDT:
		mixer_offset = mixer_sdt_offset;
		break;
	case MIXVXREC:
		mixer_offset = mixer_vxrec_offset;
		break;
	case MIXAUDUL:
		mixer_offset = mixer_audul_offset;
		break;
	case GAINS_BTUL:
		mixer_offset = btul_gains_offset;
		break;
	}
	/* SMEM word32 address for the CURRENT gain values */
	mixer_target = (S_GCurrent_ADDR << 1);
	mixer_target += mixer_offset;
	mixer_target += p;
	/* translate coef address in Bytes */
	mixer_target <<= 2;
	lin_g = 0;
	/* load the S_G_Target SMEM table */
	abe_block_copy(COPY_FROM_HOST_TO_ABE, ABE_SMEM, mixer_target,
		       (u32 *) &lin_g, sizeof(lin_g));
}

/**
 * abe_init_asrc_vx_dl
 *
 * Initialize the following ASRC VX_DL parameters :
 * 1. DriftSign = D_AsrcVars[1] = 1 or -1
 * 2. Subblock = D_AsrcVars[2] = 0
 * 3. DeltaAlpha = D_AsrcVars[3] =
 *	(round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 4. MinusDeltaAlpha = D_AsrcVars[4] =
 *	(-round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 5. OneMinusEpsilon = D_AsrcVars[5] = 1 - DeltaAlpha/2
 * 6. AlphaCurrent = 0x000020 (CMEM), initial value of Alpha parameter
 * 7. BetaCurrent = 0x3fffe0 (CMEM), initial value of Beta parameter
 * AlphaCurrent + BetaCurrent = 1 (=0x400000 in CMEM = 2^20 << 2)
 * 8. drift_ASRC = 0 & drift_io = 0
 * 9. SMEM for ASRC_DL_VX_Coefs pointer
 * 10. CMEM for ASRC_DL_VX_Coefs pointer
 * ASRC_DL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
 * C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1
 * 11. SMEM for XinASRC_DL_VX pointer
 * 12. CMEM for XinASRC_DL_VX pointer
 * XinASRC_DL_VX = S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/0/1/0/0/0/0
 * 13. SMEM for IO_VX_DL_ASRC pointer
 * 14. CMEM for IO_VX_DL_ASRC pointer
 * IO_VX_DL_ASRC =
 *	S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/
 *	ASRC_DL_VX_FIR_L+ASRC_margin/1/0/0/0/0
 */
void abe_init_asrc_vx_dl(s32 dppm)
{
	s32 el[45];
	s32 temp0, temp1, adppm, dtemp, mem_tag, mem_addr;
	u32 i = 0;
	u32 n_fifo_el = 42;
	temp0 = 0;
	temp1 = 1;
	/* 1. DriftSign = D_AsrcVars[1] = 1 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_DL_VX_ADDR + (1 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm >= 0) {
		el[i + 1] = 1;
		adppm = dppm;
	} else {
		el[i + 1] = -1;
		adppm = (-1 * dppm);
	}
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	dtemp = (adppm << 4) + adppm - ((adppm * 3481L) / 15625L);
	/* 2. Subblock = D_AsrcVars[2] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_DL_VX_ADDR + (2 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 3. DeltaAlpha = D_AsrcVars[3] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_DL_VX_ADDR + (3 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = dtemp << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 4. MinusDeltaAlpha = D_AsrcVars[4] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_DL_VX_ADDR + (4 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = (-dtemp) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/*5. OneMinusEpsilon = D_AsrcVars[5] = 0x00400000 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_DL_VX_ADDR + (5 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0x00400000;
	else
		el[i + 1] = (0x00100000 - (dtemp / 2)) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 6. AlphaCurrent = 0x000020 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_AlphaCurrent_DL_VX_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x00000020;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 7. BetaCurrent = 0x3fffe0 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_BetaCurrent_DL_VX_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x003fffe0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 8. drift_ASRC = 0 & drift_io = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_IOdescr_ADDR + (VX_DL_PORT * sizeof(ABE_SIODescriptor))
		+ drift_asrc_;
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 9. SMEM for ASRC_DL_VX_Coefs pointer */
	/* ASRC_DL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_SMEM;
	mem_addr = ASRC_DL_VX_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = C_CoefASRC16_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + C_CoefASRC16_VX_sizeof;
	el[i + 2] = C_CoefASRC15_VX_ADDR;
	el[i + 2] = (el[i + 2] << 8) + C_CoefASRC15_VX_sizeof;
	i = i + 3;
	/* 10. CMEM for ASRC_DL_VX_Coefs pointer */
	/* ASRC_DL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_CMEM;
	mem_addr = ASRC_DL_VX_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp1;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 11. SMEM for XinASRC_DL_VX pointer */
	/* XinASRC_DL_VX =
		S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = XinASRC_DL_VX_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_DL_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_DL_VX_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 12. CMEM for XinASRC_DL_VX pointer */
	/* XinASRC_DL_VX =
		S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = XinASRC_DL_VX_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 13. SMEM for IO_VX_DL_ASRC pointer */
	/* IO_VX_DL_ASRC = S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/
	   ASRC_DL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = IO_VX_DL_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_DL_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_DL_VX_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 14. CMEM for IO_VX_DL_ASRC pointer */
	/* IO_VX_DL_ASRC = S_XinASRC_DL_VX_ADDR/S_XinASRC_DL_VX_sizeof/
	   ASRC_DL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = IO_VX_DL_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_DL_VX_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	abe_write_fifo(ABE_DMEM, D_FwMemInitDescr_ADDR, (u32 *) &el[0],
		       n_fifo_el);
}

/**
 * abe_init_asrc_vx_ul
 *
 * Initialize the following ASRC VX_UL parameters :
 * 1. DriftSign = D_AsrcVars[1] = 1 or -1
 * 2. Subblock = D_AsrcVars[2] = 0
 * 3. DeltaAlpha = D_AsrcVars[3] =
 *	(round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 4. MinusDeltaAlpha = D_AsrcVars[4] =
 *	(-round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 5. OneMinusEpsilon = D_AsrcVars[5] = 1 - DeltaAlpha/2
 * 6. AlphaCurrent = 0x000020 (CMEM), initial value of Alpha parameter
 * 7. BetaCurrent = 0x3fffe0 (CMEM), initial value of Beta parameter
 * AlphaCurrent + BetaCurrent = 1 (=0x400000 in CMEM = 2^20 << 2)
 * 8. drift_ASRC = 0 & drift_io = 0
 * 9. SMEM for ASRC_UL_VX_Coefs pointer
 * 10. CMEM for ASRC_UL_VX_Coefs pointer
 * ASRC_UL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
 *	C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1
 * 11. SMEM for XinASRC_UL_VX pointer
 * 12. CMEM for XinASRC_UL_VX pointer
 * XinASRC_UL_VX = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/0/1/0/0/0/0
 * 13. SMEM for UL_48_8_DEC pointer
 * 14. CMEM for UL_48_8_DEC pointer
 * UL_48_8_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
 *	ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0
 * 15. SMEM for UL_48_16_DEC pointer
 * 16. CMEM for UL_48_16_DEC pointer
 * UL_48_16_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
 *	ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0
 */
void abe_init_asrc_vx_ul(s32 dppm)
{
	s32 el[51];
	s32 temp0, temp1, adppm, dtemp, mem_tag, mem_addr;
	u32 i = 0;
	u32 n_fifo_el = 48;
	temp0 = 0;
	temp1 = 1;
	/* 1. DriftSign = D_AsrcVars[1] = 1 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_UL_VX_ADDR + (1 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm >= 0) {
		el[i + 1] = 1;
		adppm = dppm;
	} else {
		el[i + 1] = -1;
		adppm = (-1 * dppm);
	}
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	dtemp = (adppm << 4) + adppm - ((adppm * 3481L) / 15625L);
	/* 2. Subblock = D_AsrcVars[2] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_UL_VX_ADDR + (2 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 3. DeltaAlpha = D_AsrcVars[3] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_UL_VX_ADDR + (3 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = dtemp << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 4. MinusDeltaAlpha = D_AsrcVars[4] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_UL_VX_ADDR + (4 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = (-dtemp) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 5. OneMinusEpsilon = D_AsrcVars[5] = 0x00400000 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_UL_VX_ADDR + (5 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0x00400000;
	else
		el[i + 1] = (0x00100000 - (dtemp / 2)) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 6. AlphaCurrent = 0x000020 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_AlphaCurrent_UL_VX_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x00000020;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 7. BetaCurrent = 0x3fffe0 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_BetaCurrent_UL_VX_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x003fffe0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 8. drift_ASRC = 0 & drift_io = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_IOdescr_ADDR + (VX_UL_PORT * sizeof(ABE_SIODescriptor))
		+ drift_asrc_;
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 9. SMEM for ASRC_UL_VX_Coefs pointer */
	/* ASRC_UL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_SMEM;
	mem_addr = ASRC_UL_VX_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = C_CoefASRC16_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + C_CoefASRC16_VX_sizeof;
	el[i + 2] = C_CoefASRC15_VX_ADDR;
	el[i + 2] = (el[i + 2] << 8) + C_CoefASRC15_VX_sizeof;
	i = i + 3;
	/* 10. CMEM for ASRC_UL_VX_Coefs pointer */
	/* ASRC_UL_VX_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_CMEM;
	mem_addr = ASRC_UL_VX_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp1;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 11. SMEM for XinASRC_UL_VX pointer */
	/* XinASRC_UL_VX = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/0/1/
		0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = XinASRC_UL_VX_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_UL_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_UL_VX_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 12. CMEM for XinASRC_UL_VX pointer */
	/* XinASRC_UL_VX = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/0/1/
		0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = XinASRC_UL_VX_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 13. SMEM for UL_48_8_DEC pointer */
	/* UL_48_8_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
	   ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = UL_48_8_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_UL_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_UL_VX_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 14. CMEM for UL_48_8_DEC pointer */
	/* UL_48_8_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
	   ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = UL_48_8_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_UL_VX_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 15. SMEM for UL_48_16_DEC pointer */
	/* UL_48_16_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
	   ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = UL_48_16_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_UL_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_UL_VX_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 16. CMEM for UL_48_16_DEC pointer */
	/* UL_48_16_DEC = S_XinASRC_UL_VX_ADDR/S_XinASRC_UL_VX_sizeof/
	   ASRC_UL_VX_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = UL_48_16_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_UL_VX_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	abe_write_fifo(ABE_DMEM, D_FwMemInitDescr_ADDR, (u32 *) &el[0],
		       n_fifo_el);
}

/**
 * abe_init_asrc_mm_ext_in
 *
 * Initialize the following ASRC MM_EXT_IN parameters :
 * 1. DriftSign = D_AsrcVars[1] = 1 or -1
 * 2. Subblock = D_AsrcVars[2] = 0
 * 3. DeltaAlpha = D_AsrcVars[3] =
 *	(round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 4. MinusDeltaAlpha = D_AsrcVars[4] =
 *	(-round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 5. OneMinusEpsilon = D_AsrcVars[5] = 1 - DeltaAlpha/2
 * 6. AlphaCurrent = 0x000020 (CMEM), initial value of Alpha parameter
 * 7. BetaCurrent = 0x3fffe0 (CMEM), initial value of Beta parameter
 * AlphaCurrent + BetaCurrent = 1 (=0x400000 in CMEM = 2^20 << 2)
 * 8. drift_ASRC = 0 & drift_io = 0
 * 9. SMEM for ASRC_MM_EXT_IN_Coefs pointer
 * 10. CMEM for ASRC_MM_EXT_IN_Coefs pointer
 * ASRC_MM_EXT_IN_Coefs = C_CoefASRC16_MM_ADDR/C_CoefASRC16_MM_sizeof/0/1/
 *	C_CoefASRC15_MM_ADDR/C_CoefASRC15_MM_sizeof/0/1
 * 11. SMEM for XinASRC_MM_EXT_IN pointer
 * 12. CMEM for XinASRC_MM_EXT_IN pointer
 * XinASRC_MM_EXT_IN = S_XinASRC_MM_EXT_IN_ADDR/S_XinASRC_MM_EXT_IN_sizeof/0/1/
 *	0/0/0/0
 * 13. SMEM for IO_MM_EXT_IN_ASRC pointer
 * 14. CMEM for IO_MM_EXT_IN_ASRC pointer
 * IO_MM_EXT_IN_ASRC = S_XinASRC_MM_EXT_IN_ADDR/S_XinASRC_MM_EXT_IN_sizeof/
 *	ASRC_MM_EXT_IN_FIR_L+ASRC_margin+ASRC_N_48k/1/0/0/0/0
 */
void abe_init_asrc_mm_ext_in(s32 dppm)
{
	s32 el[45];
	s32 temp0, temp1, adppm, dtemp, mem_tag, mem_addr;
	u32 i = 0;
	u32 n_fifo_el = 42;
	temp0 = 0;
	temp1 = 1;
	/* 1. DriftSign = D_AsrcVars[1] = 1 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_MM_EXT_IN_ADDR + (1 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm >= 0) {
		el[i + 1] = 1;
		adppm = dppm;
	} else {
		el[i + 1] = -1;
		adppm = (-1 * dppm);
	}
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	dtemp = (adppm << 4) + adppm - ((adppm * 3481L) / 15625L);
	/* 2. Subblock = D_AsrcVars[2] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_MM_EXT_IN_ADDR + (2 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 3. DeltaAlpha = D_AsrcVars[3] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_MM_EXT_IN_ADDR + (3 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = dtemp << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 4. MinusDeltaAlpha = D_AsrcVars[4] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_MM_EXT_IN_ADDR + (4 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = (-dtemp) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 5. OneMinusEpsilon = D_AsrcVars[5] = 0x00400000 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_MM_EXT_IN_ADDR + (5 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0x00400000;
	else
		el[i + 1] = (0x00100000 - (dtemp / 2)) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 6. AlphaCurrent = 0x000020 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_AlphaCurrent_MM_EXT_IN_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x00000020;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 7. BetaCurrent = 0x3fffe0 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_BetaCurrent_MM_EXT_IN_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x003fffe0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 8. drift_ASRC = 0 & drift_io = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_IOdescr_ADDR + (MM_EXT_IN_PORT * sizeof(ABE_SIODescriptor))
		+ drift_asrc_;
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 9. SMEM for ASRC_MM_EXT_IN_Coefs pointer */
	/* ASRC_MM_EXT_IN_Coefs = C_CoefASRC16_MM_ADDR/C_CoefASRC16_MM_sizeof/
		0/1/C_CoefASRC15_MM_ADDR/C_CoefASRC15_MM_sizeof/0/1 */
	mem_tag = ABE_SMEM;
	mem_addr = ASRC_MM_EXT_IN_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = C_CoefASRC16_MM_ADDR;
	el[i + 1] = (el[i + 1] << 8) + C_CoefASRC16_MM_sizeof;
	el[i + 2] = C_CoefASRC15_MM_ADDR;
	el[i + 2] = (el[i + 2] << 8) + C_CoefASRC15_MM_sizeof;
	i = i + 3;
	/*10. CMEM for ASRC_MM_EXT_IN_Coefs pointer */
	/* ASRC_MM_EXT_IN_Coefs = C_CoefASRC16_MM_ADDR/C_CoefASRC16_MM_sizeof/
		0/1/C_CoefASRC15_MM_ADDR/C_CoefASRC15_MM_sizeof/0/1 */
	mem_tag = ABE_CMEM;
	mem_addr = ASRC_MM_EXT_IN_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp1;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 11. SMEM for XinASRC_MM_EXT_IN pointer */
	/* XinASRC_MM_EXT_IN = S_XinASRC_MM_EXT_IN_ADDR/
		S_XinASRC_MM_EXT_IN_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = XinASRC_MM_EXT_IN_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_MM_EXT_IN_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_MM_EXT_IN_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 12. CMEM for XinASRC_MM_EXT_IN pointer */
	/* XinASRC_MM_EXT_IN = S_XinASRC_MM_EXT_IN_ADDR/
		S_XinASRC_MM_EXT_IN_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = XinASRC_MM_EXT_IN_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 13. SMEM for IO_MM_EXT_IN_ASRC pointer */
	/* IO_MM_EXT_IN_ASRC =
		S_XinASRC_MM_EXT_IN_ADDR/S_XinASRC_MM_EXT_IN_sizeof/
		ASRC_MM_EXT_IN_FIR_L+ASRC_margin+ASRC_N_48k/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = IO_MM_EXT_IN_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_MM_EXT_IN_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_MM_EXT_IN_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 14. CMEM for IO_MM_EXT_IN_ASRC pointer */
	/* IO_MM_EXT_IN_ASRC =
		S_XinASRC_MM_EXT_IN_ADDR/S_XinASRC_MM_EXT_IN_sizeof/
		ASRC_MM_EXT_IN_FIR_L+ASRC_margin+ASRC_N_48k/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = IO_MM_EXT_IN_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_MM_EXT_IN_FIR_L + ASRC_margin + ASRC_N_48k) << 16) +
		(temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	abe_write_fifo(ABE_DMEM, D_FwMemInitDescr_ADDR, (u32 *) &el[0],
		       n_fifo_el);
}

/**
 * abe_init_asrc_bt_ul
 *
 * Initialize the following ASRC BT_UL parameters :
 * 1. DriftSign = D_AsrcVars[1] = 1 or -1
 * 2. Subblock = D_AsrcVars[2] = 0
 * 3. DeltaAlpha = D_AsrcVars[3] =
 *	(round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 4. MinusDeltaAlpha = D_AsrcVars[4] =
 *	(-round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 5. OneMinusEpsilon = D_AsrcVars[5] = 1 - DeltaAlpha/2
 * 6. AlphaCurrent = 0x000020 (CMEM), initial value of Alpha parameter
 * 7. BetaCurrent = 0x3fffe0 (CMEM), initial value of Beta parameter
 * AlphaCurrent + BetaCurrent = 1 (=0x400000 in CMEM = 2^20 << 2)
 * 8. drift_ASRC = 0 & drift_io = 0
 * 9. SMEM for ASRC_BT_UL_Coefs pointer
 * 10. CMEM for ASRC_BT_UL_Coefs pointer
 * ASRC_BT_UL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
 * C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1
 * 11. SMEM for XinASRC_BT_UL pointer
 * 12. CMEM for XinASRC_BT_UL pointer
 * XinASRC_BT_UL = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/0/1/0/0/0/0
 * 13. SMEM for IO_BT_UL_ASRC pointer
 * 14. CMEM for IO_BT_UL_ASRC pointer
 * IO_BT_UL_ASRC = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/
 *	ASRC_BT_UL_FIR_L+ASRC_margin/1/0/0/0/0
 */
void abe_init_asrc_bt_ul(s32 dppm)
{
	s32 el[45];
	s32 temp0, temp1, adppm, dtemp, mem_tag, mem_addr;
	u32 i = 0;
	u32 n_fifo_el = 42;
	temp0 = 0;
	temp1 = 1;
	/* 1. DriftSign = D_AsrcVars[1] = 1 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_UL_ADDR + (1 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm >= 0) {
		el[i + 1] = 1;
		adppm = dppm;
	} else {
		el[i + 1] = -1;
		adppm = (-1 * dppm);
	}
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	dtemp = (adppm << 4) + adppm - ((adppm * 3481L) / 15625L);
	/* 2. Subblock = D_AsrcVars[2] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_UL_ADDR + (2 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 3. DeltaAlpha = D_AsrcVars[3] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_UL_ADDR + (3 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = dtemp << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 4. MinusDeltaAlpha = D_AsrcVars[4] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_UL_ADDR + (4 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = (-dtemp) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/*5. OneMinusEpsilon = D_AsrcVars[5] = 0x00400000 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_UL_ADDR + (5 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0x00400000;
	else
		el[i + 1] = (0x00100000 - (dtemp / 2)) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 6. AlphaCurrent = 0x000020 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_AlphaCurrent_BT_UL_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x00000020;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 7. BetaCurrent = 0x3fffe0 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_BetaCurrent_BT_UL_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x003fffe0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 8. drift_ASRC = 0 & drift_io = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_IOdescr_ADDR + (BT_VX_UL_PORT * sizeof(ABE_SIODescriptor))
		+ drift_asrc_;
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 9. SMEM for ASRC_BT_UL_Coefs pointer */
	/* ASRC_BT_UL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_SMEM;
	mem_addr = ASRC_BT_UL_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = C_CoefASRC16_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + C_CoefASRC16_VX_sizeof;
	el[i + 2] = C_CoefASRC15_VX_ADDR;
	el[i + 2] = (el[i + 2] << 8) + C_CoefASRC15_VX_sizeof;
	i = i + 3;
	/* 10. CMEM for ASRC_BT_UL_Coefs pointer */
	/* ASRC_BT_UL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_CMEM;
	mem_addr = ASRC_BT_UL_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp1;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 11. SMEM for XinASRC_BT_UL pointer */
	/* XinASRC_BT_UL = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/0/1/
		0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = XinASRC_BT_UL_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_BT_UL_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_BT_UL_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 12. CMEM for XinASRC_BT_UL pointer */
	/* XinASRC_BT_UL = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/0/1/
		0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = XinASRC_BT_UL_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 13. SMEM for IO_BT_UL_ASRC pointer */
	/* IO_BT_UL_ASRC = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/
		ASRC_BT_UL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = IO_BT_UL_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_BT_UL_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_BT_UL_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 14. CMEM for IO_BT_UL_ASRC pointer */
	/* IO_BT_UL_ASRC = S_XinASRC_BT_UL_ADDR/S_XinASRC_BT_UL_sizeof/
		ASRC_BT_UL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = IO_BT_UL_ASRC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_BT_UL_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	abe_write_fifo(ABE_DMEM, D_FwMemInitDescr_ADDR, (u32 *) &el[0],
		       n_fifo_el);
}

/**
 * abe_init_asrc_bt_dl
 *
 * Initialize the following ASRC BT_DL parameters :
 * 1. DriftSign = D_AsrcVars[1] = 1 or -1
 * 2. Subblock = D_AsrcVars[2] = 0
 * 3. DeltaAlpha = D_AsrcVars[3] =
 *	(round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 4. MinusDeltaAlpha = D_AsrcVars[4] =
 *	(-round(nb_phases * drift[ppm] * 10^-6 * 2^20)) << 2
 * 5. OneMinusEpsilon = D_AsrcVars[5] = 1 - DeltaAlpha/2
 * 6. AlphaCurrent = 0x000020 (CMEM), initial value of Alpha parameter
 * 7. BetaCurrent = 0x3fffe0 (CMEM), initial value of Beta parameter
 * AlphaCurrent + BetaCurrent = 1 (=0x400000 in CMEM = 2^20 << 2)
 * 8. drift_ASRC = 0 & drift_io = 0
 * 9. SMEM for ASRC_BT_DL_Coefs pointer
 * 10. CMEM for ASRC_BT_DL_Coefs pointer
 * ASRC_BT_DL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
 *	C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1
 * 11. SMEM for XinASRC_BT_DL pointer
 * 12. CMEM for XinASRC_BT_DL pointer
 * XinASRC_BT_DL = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/0/1/0/0/0/0
 * 13. SMEM for DL_48_8_DEC pointer
 * 14. CMEM for DL_48_8_DEC pointer
 * DL_48_8_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
 *	ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0
 * 15. SMEM for DL_48_16_DEC pointer
 * 16. CMEM for DL_48_16_DEC pointer
 * DL_48_16_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
 *	ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0
 */
void abe_init_asrc_bt_dl(s32 dppm)
{
	s32 el[51];
	s32 temp0, temp1, adppm, dtemp, mem_tag, mem_addr;
	u32 i = 0;
	u32 n_fifo_el = 48;
	temp0 = 0;
	temp1 = 1;
	/* 1. DriftSign = D_AsrcVars[1] = 1 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_DL_ADDR + (1 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm >= 0) {
		el[i + 1] = 1;
		adppm = dppm;
	} else {
		el[i + 1] = -1;
		adppm = (-1 * dppm);
	}
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	dtemp = (adppm << 4) + adppm - ((adppm * 3481L) / 15625L);
	/* 2. Subblock = D_AsrcVars[2] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_DL_ADDR + (2 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 3. DeltaAlpha = D_AsrcVars[3] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_DL_ADDR + (3 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = dtemp << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 4. MinusDeltaAlpha = D_AsrcVars[4] = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_DL_ADDR + (4 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0;
	else
		el[i + 1] = (-dtemp) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 5. OneMinusEpsilon = D_AsrcVars[5] = 0x00400000 */
	mem_tag = ABE_DMEM;
	mem_addr = D_AsrcVars_BT_DL_ADDR + (5 * sizeof(s32));
	el[i] = (mem_tag << 16) + mem_addr;
	if (dppm == 0)
		el[i + 1] = 0x00400000;
	else
		el[i + 1] = (0x00100000 - (dtemp / 2)) << 2;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 6. AlphaCurrent = 0x000020 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_AlphaCurrent_BT_DL_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x00000020;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 7. BetaCurrent = 0x3fffe0 (CMEM) */
	mem_tag = ABE_CMEM;
	mem_addr = C_BetaCurrent_BT_DL_ADDR;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = 0x003fffe0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 8. drift_ASRC = 0 & drift_io = 0 */
	mem_tag = ABE_DMEM;
	mem_addr = D_IOdescr_ADDR + (BT_VX_DL_PORT * sizeof(ABE_SIODescriptor))
		+ drift_asrc_;
	el[i] = (mem_tag << 16) + mem_addr;
	el[i + 1] = temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 9. SMEM for ASRC_BT_DL_Coefs pointer */
	/* ASRC_BT_DL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_SMEM;
	mem_addr = ASRC_BT_DL_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = C_CoefASRC16_VX_ADDR;
	el[i + 1] = (el[i + 1] << 8) + C_CoefASRC16_VX_sizeof;
	el[i + 2] = C_CoefASRC15_VX_ADDR;
	el[i + 2] = (el[i + 2] << 8) + C_CoefASRC15_VX_sizeof;
	i = i + 3;
	/* 10. CMEM for ASRC_BT_DL_Coefs pointer */
	/* ASRC_BT_DL_Coefs = C_CoefASRC16_VX_ADDR/C_CoefASRC16_VX_sizeof/0/1/
		C_CoefASRC15_VX_ADDR/C_CoefASRC15_VX_sizeof/0/1 */
	mem_tag = ABE_CMEM;
	mem_addr = ASRC_BT_DL_Coefs_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp1;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 11. SMEM for XinASRC_BT_DL pointer */
	/* XinASRC_BT_DL =
		S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = XinASRC_BT_DL_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_BT_DL_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_BT_DL_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 12. CMEM for XinASRC_BT_DL pointer */
	/* XinASRC_BT_DL =
		S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/0/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = XinASRC_BT_DL_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = (temp0 << 16) + (temp1 << 12) + (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 13. SMEM for DL_48_8_DEC pointer */
	/* DL_48_8_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
		ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = DL_48_8_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_BT_DL_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_BT_DL_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 14. CMEM for DL_48_8_DEC pointer */
	/* DL_48_8_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
		ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = DL_48_8_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_BT_DL_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	i = i + 3;
	/* 15. SMEM for DL_48_16_DEC pointer */
	/* DL_48_16_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
		ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_SMEM;
	mem_addr = DL_48_16_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	el[i + 1] = S_XinASRC_BT_DL_ADDR;
	el[i + 1] = (el[i + 1] << 8) + S_XinASRC_BT_DL_sizeof;
	el[i + 2] = temp0;
	i = i + 3;
	/* 16. CMEM for DL_48_16_DEC pointer */
	/* DL_48_16_DEC = S_XinASRC_BT_DL_ADDR/S_XinASRC_BT_DL_sizeof/
		ASRC_BT_DL_FIR_L+ASRC_margin/1/0/0/0/0 */
	mem_tag = ABE_CMEM;
	mem_addr = DL_48_16_DEC_labelID;
	el[i] = (mem_tag << 16) + (mem_addr << 2);
	/* el[i+1] = iam1<<16 + inc1<<12 + iam2<<4 + inc2 */
	el[i + 1] = ((ASRC_BT_DL_FIR_L + ASRC_margin) << 16) + (temp1 << 12)
		+ (temp0 << 4) + temp0;
	/* dummy field */
	el[i + 2] = temp0;
	abe_write_fifo(ABE_DMEM, D_FwMemInitDescr_ADDR, (u32 *) &el[0],
		       n_fifo_el);
}
