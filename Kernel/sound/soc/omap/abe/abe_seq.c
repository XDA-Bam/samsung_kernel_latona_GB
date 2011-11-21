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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#include "abe_main.h"
#include "abe_ref.h"

/**
 * abe_null_subroutine
 *
 */
void abe_null_subroutine_0(void)
{
}

void abe_null_subroutine_2(u32 a, u32 b)
{
}

void abe_null_subroutine_4(u32 a, u32 b, u32 c, u32 d)
{
}

/**
 * abe_init_subroutine_table - initializes the default table of pointers
 * to subroutines
 *
 * initializes the default table of pointers to subroutines
 *
 */
void abe_init_subroutine_table(void)
{
	u32 id;

	/* reset the table's pointers */
	abe_subroutine_write_pointer = 0;
	/* the first index is the NULL task */
	abe_add_subroutine(&id, (abe_subroutine2) abe_null_subroutine_2,
			   SUB_0_PARAM, (u32 *) 0);
	/* write mixer has 4 parameters */
	abe_add_subroutine(&(abe_subroutine_id[SUB_WRITE_MIXER]),
			   (abe_subroutine2) abe_write_mixer, SUB_4_PARAM,
			   (u32 *) 0);
	/* ping-pong player IRQ */
	abe_add_subroutine(&abe_irq_pingpong_player_id,
			   (abe_subroutine2) abe_null_subroutine_0, SUB_0_PARAM,
			   (u32 *) 0);
	abe_add_subroutine(&abe_irq_aps_adaptation_id,
			   (abe_subroutine2) abe_default_irq_aps_adaptation,
			   SUB_0_PARAM, (u32 *) 0);
}

/**
 * abe_add_subroutine
 * @id: ABE port id
 * @f: pointer to the subroutines
 * @nparam: number of parameters
 * @params: pointer to the psrameters
 *
 * add one function pointer more and returns the index to it
 */
void abe_add_subroutine(u32 *id, abe_subroutine2 f, u32 nparam, u32 *params)
{
	u32 i, i_found;

	if ((abe_subroutine_write_pointer >= MAXNBSUBROUTINE) ||
			((u32) f == 0)) {
		abe->dbg_param |= ERR_SEQ;
		abe_dbg_error_log(ABE_PARAMETER_OVERFLOW);
	} else {
		/* search if this subroutine address was not already
		 * declared, then return the previous index
		 */
		for (i_found = abe_subroutine_write_pointer, i = 0;
		     i < abe_subroutine_write_pointer; i++) {
			if (f == abe_all_subsubroutine[i])
				i_found = i;
		}
		if (i_found == abe_subroutine_write_pointer) {
			*id = abe_subroutine_write_pointer;
			abe_all_subsubroutine
				[abe_subroutine_write_pointer] = (f);
			abe_all_subroutine_params
				[abe_subroutine_write_pointer] = params;
			abe_all_subsubroutine_nparam
				[abe_subroutine_write_pointer] = nparam;
			abe_subroutine_write_pointer++;
		} else {
			abe_all_subroutine_params[i_found] = params;
			*id = i_found;
		}
	}
}

/**
 * abe_add_sequence
 * @id: returned sequence index after pluging a new sequence
 * (index in the tables)
 * @s: sequence to be inserted
 *
 * Load a time-sequenced operations.
 */
void abe_add_sequence(u32 *id, abe_sequence_t *s)
{
	abe_seq_t *seq_src, *seq_dst;
	u32 i, no_end_of_sequence_found;

	seq_src = &(s->seq1);
	seq_dst = &((abe_all_sequence[abe_sequence_write_pointer]).seq1);
	if ((abe_sequence_write_pointer >= MAXNBSEQUENCE) || ((u32) s == 0)) {
		abe->dbg_param |= ERR_SEQ;
		abe_dbg_error_log(ABE_PARAMETER_OVERFLOW);
	} else {
		*id = abe_subroutine_write_pointer;
		/* copy the mask */
		(abe_all_sequence[abe_sequence_write_pointer]).mask = s->mask;
		for (no_end_of_sequence_found = 1, i = 0; i < MAXSEQUENCESTEPS;
		     i++, seq_src++, seq_dst++) {
			/* sequence copied line by line */
			(*seq_dst) = (*seq_src);
			/* stop when the line start with time=(-1) */
			if ((*(s32 *) seq_src) == (-1)) {
				/* stop when the line start with time=(-1) */
				no_end_of_sequence_found = 0;
				break;
			}
		}
		abe_subroutine_write_pointer++;
		if (no_end_of_sequence_found)
			abe_dbg_error_log(ABE_SEQTOOLONG);
	}
}
/**
 * abe_reset_one_sequence
 * @id: sequence ID
 *
 * load default configuration for that sequence
 * kill running activities
 */
void abe_reset_one_sequence(u32 id)
{
}
/**
 * abe_reset_all_sequence
 *
 * load default configuration for all sequences
 * kill any running activities
 */
void abe_reset_all_sequence(void)
{
	u32 i;

	abe_init_subroutine_table();
	/* arrange to have the first sequence index=0 to the NULL operation
	   sequence */
	abe_add_sequence(&i, (abe_sequence_t *) &seq_null);
	/* reset the the collision protection mask */
	abe_global_sequence_mask = 0;
	/* reset the pending sequences list */
	for (abe_nb_pending_sequences = i = 0; i < MAXNBSEQUENCE; i++)
		abe_pending_sequences[i] = 0;
}
/**
 * abe_call_subroutine
 * @idx: index to the table of all registered Call-backs and subroutines
 *
 * run and log a subroutine
 */
void abe_call_subroutine(u32 idx, u32 p1, u32 p2, u32 p3, u32 p4)
{
	abe_subroutine0 f0;
	abe_subroutine1 f1;
	abe_subroutine2 f2;
	abe_subroutine3 f3;
	abe_subroutine4 f4;
	u32 *params;

	if (idx > MAXNBSUBROUTINE)
		return;
	switch (idx) {
		/* call the subroutines defined at compilation time
		   (const .. sequences) */
#if 0
	case SUB_WRITE_MIXER_DL1:
		abe_write_mixer_dl1(p1, p2, p3)
			abe_fprintf("write_mixer");
		break;
#endif
		/* call the subroutines defined at execution time
		   (dynamic sequences) */
	default:
		switch (abe_all_subsubroutine_nparam[idx]) {
		case SUB_0_PARAM:
			f0 = (abe_subroutine0) abe_all_subsubroutine[idx];
			(*f0) ();
			break;
		case SUB_1_PARAM:
			f1 = (abe_subroutine1) abe_all_subsubroutine[idx];
			params = abe_all_subroutine_params
				[abe_irq_pingpong_player_id];
			if (params != (u32 *) 0)
				p1 = params[0];
			(*f1) (p1);
			break;
		case SUB_2_PARAM:
			f2 = abe_all_subsubroutine[idx];
			params = abe_all_subroutine_params
				[abe_irq_pingpong_player_id];
			if (params != (u32 *) 0) {
				p1 = params[0];
				p2 = params[1];
			}
			(*f2) (p1, p2);
			break;
		case SUB_3_PARAM:
			f3 = (abe_subroutine3) abe_all_subsubroutine[idx];
			params = abe_all_subroutine_params
				[abe_irq_pingpong_player_id];
			if (params != (u32 *) 0) {
				p1 = params[0];
				p2 = params[1];
				p3 = params[2];
			}
			(*f3) (p1, p2, p3);
			break;
		case SUB_4_PARAM:
			f4 = (abe_subroutine4) abe_all_subsubroutine[idx];
			params = abe_all_subroutine_params
				[abe_irq_pingpong_player_id];
			if (params != (u32 *) 0) {
				p1 = params[0];
				p2 = params[1];
				p3 = params[2];
				p4 = params[3];
			}
			(*f4) (p1, p2, p3, p4);
			break;
		default:
			break;
		}
	}
}
