/*
 *	subsystem2.h
 *	Subsystem 2 – Motion Control: task configuration and VC request interface.  Student ID: 2503272.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include "../app.h"

/* Task configuration */
#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PRI_MOTION    	15  /* S2 motion priority */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */
#define S2_MOTION_MS     	20  /* Motion update period */

/* VC command numbers */
#define S2_MOTION_STOP          0u
#define S2_MOTION_SET_SPEED     1u
#define S2_MOTION_MOVE_DISTANCE 2u
#define S2_MOTION_TURN_LEFT     3u
#define S2_MOTION_TURN_RIGHT    4u

/* Agreed speed settings (in cm/s). The 40 cm/s max speed is just a sample, not measured. */
#define S2_MOTION_MAX_SPEED     40u
#define S2_MOTION_DEFAULT_SPEED 10u

/* Request format: command in bits 0–2, argument in bits 3–15. */
#define S2_MOTION_REQUEST_BYTES 2u
#define S2_MOTION_COMMAND_MASK  0x0007u
#define S2_MOTION_ARGUMENT_MAX  0x1FFFu

/* Objects created by Motion Control during initialisation. */
IMPORT ID mbf_motion_req;
IMPORT ID flg_motion;

/* Motion sets this bit when a distance/turn completes. The VC clears it after reading. */
#define S2_MOTION_DONE (1u << 0)

/* Caller must validate command and argument before packing. Reserved command numbers: 5–7. */
static inline uint16_t motion_packRequest(uint8_t command,
                                          uint16_t argument)
{
    return (uint16_t)(
        ((uint16_t)command & S2_MOTION_COMMAND_MASK)
        | ((uint32_t)(argument & S2_MOTION_ARGUMENT_MAX) << 3)
    );
}

static inline uint8_t motion_unpackCommand(uint16_t request)
{
    return (uint8_t)(request & S2_MOTION_COMMAND_MASK);
}

static inline uint16_t motion_unpackArgument(uint16_t request)
{
    return (uint16_t)(request >> 3);
}

static inline void motion_encodeRequest(uint8_t bytes[2],
                                        uint8_t command,
                                        uint16_t argument)
{
    uint16_t request = motion_packRequest(command, argument);

    bytes[0] = (uint8_t)(request & 0xFFu);
    bytes[1] = (uint8_t)(request >> 8);
}

static inline uint16_t motion_decodeRequest(const uint8_t bytes[2])
{
    return (uint16_t)(
        (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8)
    );
}

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* Private to subsystem 2 */
IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);

#endif /* SUBSYSTEM2_H */
