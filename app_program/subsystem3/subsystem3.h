/*
 *	subsystem3.h
 *	Subsystem 3 -- IR line following and barcode decoding.
 *	Student ID: <student ID>.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdint.h>
#include "../app.h"
#include "ir_sense.h"

#define S3_PRI_IR		16	/* ir_sensor_task, band 16-17 (TEAM.md) */
#define S3_PRI_CONSUMER		17	/* legacy example, not started */
#define S3_BATCH_SIZE		5	/* legacy example */

/* ---------------------------------------------------------------------
 *  IF-15: S3 -> VC.  Message buffer owned by S3.  One 1-byte message per
 *  barcode read, sent with TMO_POL (dropped if the VC has not read the
 *  previous ones).  Read with tk_rcv_mbf(mbf_s3_nav, &b, TMO_POL).
 *  Byte layout = BARCODE telemetry body: letter in bits 0-4 (0-25 =
 *  'A'-'Z', 31 = unreadable), command in bits 5-7 (IR_NAV_xxx, which
 *  equal PROTO_BC_CMD_xxx).
 * --------------------------------------------------------------------- */
IMPORT ID	mbf_s3_nav;
#define S3_NAV_MSG_BYTES	1u
#define S3_NAV_BUF_BYTES	32u

static inline uint8_t
s3_nav_pack (uint8_t letter_code, uint8_t cmd)
{
	return (uint8_t)((letter_code & 0x1Fu) | ((uint32_t)(cmd & 0x07u) << 5));
}

static inline uint8_t
s3_nav_cmd_of (uint8_t packed)
{
	return (uint8_t)(packed >> 5);
}

static inline uint8_t
s3_nav_letter_of (uint8_t packed)
{
	return (uint8_t)(packed & 0x1Fu);
}

/* ---------------------------------------------------------------------
 *  Legacy example objects (IF-01, IF-04).  Still created because
 *  subsystem 4's monitor_task and the example tests read them; the
 *  consumer task itself is no longer started (it would only block on
 *  an empty mailbox and cost a 2 KB stack).
 * --------------------------------------------------------------------- */
IMPORT ID mbx_samples;
IMPORT ID sem_slots;
IMPORT ID mpf_samples;

typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID mbf_summary;    /* Owned by Subsystem 4 */
IMPORT ID flg_events;
IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */