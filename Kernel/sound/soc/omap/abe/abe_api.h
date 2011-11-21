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
#ifndef _ABE_API_H_
#define _ABE_API_H_

#include <linux/string.h>
#include <linux/mutex.h>

struct omap_abe {
	void __iomem *io_base;
	u32 firmware_version_number;
	u16 MultiFrame[PROCESSING_SLOTS][TASKS_IN_SLOT];
	u32 compensated_mixer_gain;
	u8  muted_gains_indicator[MAX_NBGAIN_CMEM];
	u32 desired_gains_decibel[MAX_NBGAIN_CMEM];
	u32 muted_gains_decibel[MAX_NBGAIN_CMEM];
	u32 desired_gains_linear[MAX_NBGAIN_CMEM];
	u32 desired_ramp_delay_ms[MAX_NBGAIN_CMEM];
	struct mutex mutex;

	/* Debug Data */
	u32 irq_dbg_read_ptr;
	u32 dbg_activity_log[D_DEBUG_HAL_TASK_sizeof];
	u32 dbg_activity_log_write_pointer;
	u32 dbg_mask;
	u32 dbg_param;
	u32 dbg_output;
};

/**
 * abe_reset_hal - reset the ABE/HAL
 * @rdev: regulator source
 * @constraints: constraints to apply
 *
 * Operations : reset the HAL by reloading the static variables and
 * default AESS registers.
 * Called after a PRCM cold-start reset of ABE
 */
int abe_reset_hal(void);

/**
 * abe_load_fw_param - Load ABE Firmware memories
 * @PMEM: Pointer of Program memory data
 * @PMEM_SIZE: Size of PMEM data
 * @CMEM: Pointer of Coeffients memory data
 * @CMEM_SIZE: Size of CMEM data
 * @SMEM: Pointer of Sample memory data
 * @SMEM_SIZE: Size of SMEM data
 * @DMEM: Pointer of Data memory data
 * @DMEM_SIZE: Size of DMEM data
 *
 */
int abe_load_fw_param(u32 *FW);

/**
 * abe_reload_fw - Reload ABE Firmware after OFF mode
 */
int abe_reload_fw(void);

/**
 * abe_load_fw - Load ABE Firmware and initialize memories
 *
 */
int abe_load_fw(void);

/**
 * abe_read_hardware_configuration - Returns default
 *	HW periferals configuration
 * @u: use-case description list (pointer)
 * @o: opp mode (pointer)
 * @hw: pointer to the output HW structure
 *
 * Parameter :
 * U : use-case description list (pointer)
 * H : pointer to the output structure
 *
 * Operations :
 * Returns a structure with the HW thresholds compatible with
 *	the HAL/FW/AESS_ATC. Will be upgraded in FW06.
 */
int abe_read_hardware_configuration(u32 *u, u32 *o,
				abe_hw_config_init_t *hw);

/**
 * abe_irq_processing - Process ABE interrupt
 *
 * This subroutine is call upon reception of "MA_IRQ_99 ABE_MPU_IRQ" Audio
 * back-end interrupt. This subroutine will check the ATC Hrdware, the
 * IRQ_FIFO from the AE and act accordingly. Some IRQ source are originated
 * for the delivery of "end of time sequenced tasks" notifications, some are
 * originated from the Ping-Pong protocols, some are generated from
 * the embedded debugger when the firmware stops on programmable break-points,
 * etc ...
 */
int abe_irq_processing(void);

/**
 * abe_irq_clear - clear ABE interrupt
 *
 * This subroutine is call to clear MCU Irq
 */
int abe_clear_irq(void);

/**
 * abe_disable_irq - disable MCU/DSP ABE interrupt
 *
 * This subroutine is disabling ABE MCU/DSP Irq
 */
int abe_disable_irq(void);

/*
 * abe_check_activity - check all ports are closed
 */
int abe_check_activity(void);

/**
 * abe_wakeup - Wakeup ABE
 *
 * Wakeup ABE in case of retention
 */
int abe_wakeup(void);

/**
 * abe_start_event_generator - Stops event generator source
 *
 * Start the event genrator of AESS. No more event will be send to AESS engine.
 * Upper layer must wait 1/96kHz to be sure that engine reaches
 * the IDLE instruction.
 */
int abe_start_event_generator(void);

/**
 * abe_stop_event_generator - Stops event generator source
 *
 * Stop the event genrator of AESS. No more event will be send to AESS engine.
 * Upper layer must wait 1/96kHz to be sure that engine reaches
 * the IDLE instruction.
 */
int abe_stop_event_generator(void);

/**
 * abe_select_main_port - Select stynchronization port for Event generator.
 * @id: audio port name
 *
 * tells the FW which is the reference stream for adjusting
 * the processing on 23/24/25 slots
 */
int abe_select_main_port(u32 id);

/**
 * abe_write_event_generator - Selects event generator source
 * @e: Event Generation Counter, McPDM, DMIC or default.
 *
 * Loads the AESS event generator hardware source.
 * Loads the firmware parameters accordingly.
 * Indicates to the FW which data stream is the most important to preserve
 * in case all the streams are asynchronous.
 * If the parameter is "default", then HAL decides which Event source
 * is the best appropriate based on the opened ports.
 *
 * When neither the DMIC and the McPDM are activated, the AE will have
 * its EVENT generator programmed with the EVENT_COUNTER.
 * The event counter will be tuned in order to deliver a pulse frequency higher
 * than 96 kHz.
 * The DPLL output at 100% OPP is MCLK = (32768kHz x6000) = 196.608kHz
 * The ratio is (MCLK/96000)+(1<<1) = 2050
 * (1<<1) in order to have the same speed at 50% and 100% OPP
 * (only 15 MSB bits are used at OPP50%)
 */
int abe_write_event_generator(u32 e);

/**
 * abe_read_use_case_opp() - description for void abe_read_use_case_opp().
 *
 * returns the expected min OPP for a given use_case list
 */
int abe_read_use_case_opp(u32 *u, u32 *o);

/**
 * abe_set_opp_processing - Set OPP mode for ABE Firmware
 * @opp: OOPP mode
 *
 * New processing network and OPP:
 * 0: Ultra Lowest power consumption audio player (no post-processing, no mixer)
 * 1: OPP 25% (simple multimedia features, including low-power player)
 * 2: OPP 50% (multimedia and voice calls)
 * 3: OPP100% (EANC, multimedia complex use-cases)
 *
 * Rearranges the FW task network to the corresponding OPP list of features.
 * The corresponding AE ports are supposed to be set/reset accordingly before
 * this switch.
 *
 */
int abe_set_opp_processing(u32 opp);

/**
 * abe_set_ping_pong_buffer
 * @port: ABE port ID
 * @n_bytes: Size of Ping/Pong buffer
 *
 * Updates the next ping-pong buffer with "size" bytes copied from the
 * host processor. This API notifies the FW that the data transfer is done.
 */
int abe_set_ping_pong_buffer(u32 port, u32 n_bytes);

/**
 * abe_read_next_ping_pong_buffer
 * @port: ABE portID
 * @p: Next buffer address (pointer)
 * @n: Next buffer size (pointer)
 *
 * Tell the next base address of the next ping_pong Buffer and its size
 */
int abe_read_next_ping_pong_buffer(u32 port, u32 *p, u32 *n);

/**
 * abe_init_ping_pong_buffer
 * @id: ABE port ID
 * @size_bytes:size of the ping pong
 * @n_buffers:number of buffers (2 = ping/pong)
 * @p:returned address of the ping-pong list of base address (byte offset
	from DMEM start)
 *
 * Computes the base address of the ping_pong buffers
 */
int abe_init_ping_pong_buffer(u32 id, u32 size_bytes, u32 n_buffers,
				u32 *p);

/**
 * abe_read_offset_from_ping_buffer
 * @id: ABE port ID
 * @n:  returned address of the offset
 *	from the ping buffer start address (in samples)
 *
 * Computes the current firmware ping pong read pointer location,
 * expressed in samples, as the offset from the start address of ping buffer.
 */
int abe_read_offset_from_ping_buffer(u32 id, u32 *n);

/**
 * abe_plug_subroutine
 * @id: returned sequence index after plugging a new subroutine
 * @f: subroutine address to be inserted
 * @n: number of parameters of this subroutine
 * @params: pointer on parameters
 *
 * register a list of subroutines for call-back purpose
 */
int abe_plug_subroutine(u32 *id, abe_subroutine2 f, u32 n,
			  u32 *params);

/**
 * abe_set_sequence_time_accuracy
 * @fast: fast counter
 * @slow: slow counter
 *
 */
int abe_set_sequence_time_accuracy(u32 fast, u32 slow);

/**
 * abe_reset_port
 * @id: ABE port ID
 *
 * stop the port activity and reload default parameters on the associated
 * processing features.
 * Clears the internal AE buffers.
 */
int abe_reset_port(u32 id);

/**
 * abe_read_remaining_data
 * @id:	ABE port_ID
 * @n: size pointer to the remaining number of 32bits words
 *
 * computes the remaining amount of data in the buffer.
 */
int abe_read_remaining_data(u32 port, u32 *n);

/**
 * abe_disable_data_transfer
 * @id: ABE port id
 *
 * disables the ATC descriptor and stop IO/port activities
 * disable the IO task (@f = 0)
 * clear ATC DMEM buffer, ATC enabled
 */
int abe_disable_data_transfer(u32 id);

/**
 * abe_enable_data_transfer
 * @ip: ABE port id
 *
 * enables the ATC descriptor
 * reset ATC pointers
 * enable the IO task (@f <> 0)
 */
int abe_enable_data_transfer(u32 id);

/**
 * abe_set_dmic_filter
 * @d: DMIC decimation ratio : 16/25/32/40
 *
 * Loads in CMEM a specific list of coefficients depending on the DMIC sampling
 * frequency (2.4MHz or 3.84MHz). This table compensates the DMIC decimator
 * roll-off at 20kHz.
 * The default table is loaded with the DMIC 2.4MHz recommended configuration.
 */
int abe_set_dmic_filter(u32 d);

/**
 * abe_connect_cbpr_dmareq_port
 * @id: port name
 * @f: desired data format
 * @d: desired dma_request line (0..7)
 * @a: returned pointer to the base address of the CBPr register and number of
 *	samples to exchange during a DMA_request.
 *
 * enables the data echange between a DMA and the ABE through the
 *	CBPr registers of AESS.
 */
int abe_connect_cbpr_dmareq_port(u32 id, abe_data_format_t *f, u32 d,
				   abe_dma_t *returned_dma_t);

/**
 * abe_connect_dmareq_ping_pong_port
 * @id: port name
 * @f: desired data format
 * @d: desired dma_request line (0..7)
 * @s: half-buffer (ping) size
 * @a: returned pointer to the base address of the ping-pong buffer and number
 * of samples to exchange during a DMA_request.
 *
 * enables the data echanges between a DMA and a direct access to
 * the DMEM memory of ABE. On each dma_request activation the DMA will exchange
 * "s" bytes and switch to the "pong" buffer for a new buffer exchange.
 */
int abe_connect_dmareq_ping_pong_port(u32 id, abe_data_format_t *f,
					u32 d, u32 s,
					abe_dma_t *returned_dma_t);

/**
 * abe_connect_irq_ping_pong_port
 * @id: port name
 * @f: desired data format
 * @I: index of the call-back subroutine to call
 * @s: half-buffer (ping) size
 * @p: returned base address of the first (ping) buffer)
 *
 * enables the data echanges between a direct access to the DMEM
 * memory of ABE using cache flush. On each IRQ activation a subroutine
 * registered with "abe_plug_subroutine" will be called. This subroutine
 * will generate an amount of samples, send them to DMEM memory and call
 * "abe_set_ping_pong_buffer" to notify the new amount of samples in the
 * pong buffer.
 */
int abe_connect_irq_ping_pong_port(u32 id, abe_data_format_t *f,
				     u32 subroutine_id, u32 size,
				     u32 *sink, u32 dsp_mcu_flag);

/**
 * abe_connect_serial_port()
 * @id: port name
 * @f: data format
 * @i: peripheral ID (McBSP #1, #2, #3)
 *
 * Operations : enables the data echanges between a McBSP and an ATC buffer in
 * DMEM. This API is used connect 48kHz McBSP streams to MM_DL and 8/16kHz
 * voice streams to VX_UL, VX_DL, BT_VX_UL, BT_VX_DL. It abstracts the
 * abe_write_port API.
 */
int abe_connect_serial_port(u32 id, abe_data_format_t *f,
			      u32 mcbsp_id);

/**
 * abe_connect_slimbus_port
 * @id: port name
 * @f: data format
 * @i: peripheral ID (McBSP #1, #2, #3)
 * @j: peripheral ID (McBSP #1, #2, #3)
 *
 * enables the data echanges between 1/2 SB and an ATC buffers in
 * DMEM.
 */
int abe_connect_slimbus_port(u32 id, abe_data_format_t *f,
			       u32 sb_port1, u32 sb_port2);

/**
 * abe_connect_tdm_port
 * @id: port name
 * @f: data format
 * @i: peripheral ID (McBSP #1, #2, #3)
 * @j: peripheral ID (McBSP #1, #2, #3)
 *
 * enables the data echanges between TDM McBSP ATC buffers in
 * DMEM and 1/2 SMEM buffers
 */
int abe_connect_tdm_port(u32 id, abe_data_format_t *f, u32 mcbsp_id);

/**
 * abe_read_port_address
 * @dma: output pointer to the DMA iteration and data destination pointer
 *
 * This API returns the address of the DMA register used on this audio port.
 * Depending on the protocol being used, adds the base address offset L3
 * (DMA) or MPU (ARM)
 */
int abe_read_port_address(u32 port, abe_dma_t *dma2);

/**
 * abe_write_equalizer
 * @id: name of the equalizer
 * @param : equalizer coefficients
 *
 * Load the coefficients in CMEM.
 */
int abe_write_equalizer(u32 id, abe_equ_t *param);

/**
 * abe_write_asrc
 * @id: name of the port
 * @param: drift value to compensate [ppm]
 *
 * Load the drift variables to the FW memory. This API can be called only
 * when the corresponding port has been already opened and the ASRC has
 * been correctly initialized with API abe_init_asrc_... If this API is
 * used such that the drift has been changed from positive to negative drift
 * or vice versa, there will be click in the output signal. Loading the drift
 * value with zero disables the feature.
 */
int abe_write_asrc(u32 port, s32 dppm);

/**
 * abe_write_aps
 * @id: name of the aps filter
 * @param: table of filter coefficients
 *
 * Load the filters and thresholds coefficients in FW memory. This AP
 * can be called when the corresponding APS is not activated. After
 * reloading the firmware the default coefficients corresponds to "no APS
 * activated".
 * Loading all the coefficients value with zero disables the feature.
 */
int abe_write_aps(u32 id, abe_aps_t *param);

/**
 * abe_write_mixer
 * @id: name of the mixer
 * @param: list of input gains of the mixer
 * @p: list of port corresponding to the above gains
 *
 * Load the gain coefficients in FW memory. This API can be called when
 * the corresponding MIXER is not activated. After reloading the firmware
 * the default coefficients corresponds to "all input and output mixer's gain
 * in mute state". A mixer is disabled with a network reconfiguration
 * corresponding to an OPP value.
 */
int abe_write_gain(u32 id, s32 f_g, u32 ramp, u32 p);

int abe_use_compensated_gain(u32 on_off);

int abe_enable_gain(u32 id, u32 p);

int abe_disable_gain(u32 id, u32 p);

int abe_mute_gain(u32 id, u32 p);

int abe_unmute_gain(u32 id, u32 p);

/**
 * abe_write_mixer
 * @id: name of the mixer
 * @param: input gains and delay ramp of the mixer
 * @p: port corresponding to the above gains
 *
 * Load the gain coefficients in FW memory. This API can be called when
 * the corresponding MIXER is not activated. After reloading the firmware
 * the default coefficients corresponds to "all input and output mixer's
 * gain in mute state". A mixer is disabled with a network reconfiguration
 * corresponding to an OPP value.
 */
int abe_write_mixer(u32 id, s32 f_g, u32 f_ramp, u32 p);

/**
 * abe_read_gain
 * @id: name of the mixer
 * @param: list of input gains of the mixer
 * @p: list of port corresponding to the above gains
 *
 */
int abe_read_gain(u32 id, u32 *f_g, u32 p);

/**
 * abe_read_mixer
 * @id: name of the mixer
 * @param: gains of the mixer
 * @p: port corresponding to the above gains
 *
 * Load the gain coefficients in FW memory. This API can be called when
 * the corresponding MIXER is not activated. After reloading the firmware
 * the default coefficients corresponds to "all input and output mixer's
 * gain in mute state". A mixer is disabled with a network reconfiguration
 * corresponding to an OPP value.
 */
int abe_read_mixer(u32 id, u32 *f_g, u32 p);

/**
 * abe_set_router_configuration
 * @Id: name of the router
 * @Conf: id of the configuration
 * @param: list of output index of the route
 *
 * The uplink router takes its input from DMIC (6 samples), AMIC (2 samples)
 * and PORT1/2 (2 stereo ports). Each sample will be individually stored in
 * an intermediate table of 10 elements. The intermediate table is used to
 * route the samples to three directions : REC1 mixer, 2 EANC DMIC source of
 * filtering and MM recording audio path.
 */
int abe_set_router_configuration(u32 id, u32 k, u32 *param);

/**
 * abe_select_data_source
 * @@@
 */
int abe_select_data_source(u32 port_id, u32 smem_source);

/**
 * ABE_READ_DEBUG_TRACE
 *
 * Parameters :
 * @data: data destination pointer
 * @n	: max number of read data
 *
 * Operations :
 * Reads the AE circular data pointer that holds pairs of debug data +
 * timestamps, and stores the pairs, via linear addressing, to the parameter
 * pointer.
 * Stops the copy when the max parameter is reached or when the FIFO is empty.
 *
 * Return value :
 *	status
 */
int abe_read_debug_trace(u32 *data, u32 *n);

/**
 * abe_connect_debug_trace
 * @dma2:pointer to the DMEM trace buffer
 *
 * returns the address and size of the real-time debug trace buffer,
 * the content of which will vary from one firmware release to an other
 */
int abe_connect_debug_trace(abe_dma_t *dma2);

/**
 * abe_set_debug_trace
 * @debug: debug ID from a list to be defined
 *
 * load a mask which filters the debug trace to dedicated types of data
 */
int abe_set_debug_trace(abe_dbg_t debug);

/**
 * abe_remote_debugger_interface
 *
 * interpretation of the UART stream from the remote debugger commands.
 * The commands consist in setting break points, loading parameter
 */
int abe_remote_debugger_interface(u32 n, u8 *p);

/**
 * abe_enable_test_pattern
 *
 */
int abe_enable_test_pattern(u32 smem_id, u32 on_off);

/**
 * abe_init_mem - Allocate Kernel space memory map for ABE
 *
 * Memory map of ABE memory space for PMEM/DMEM/SMEM/DMEM
 */
int abe_init_mem(void __iomem *_io_base);

/*
 * abe_add_subroutine - Add a subroutine to be called upon interrupt
 */
void abe_add_subroutine(u32 *id, abe_subroutine2 f, u32 nparam, u32 *params);

/*
 * abe_read_next_ping_pong_buffer - Read next address in ping-pong buffer
 * processing
 */
int abe_read_next_ping_pong_buffer(u32 port, u32 *p, u32 *n);

extern u32 abe_irq_pingpong_player_id;

#endif/* _ABE_API_H_ */
