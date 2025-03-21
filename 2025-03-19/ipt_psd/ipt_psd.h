#ifndef _IPT_PSD_H
#define _IPT_PSD_H

#include <linux/param.h>
#include <linux/types.h>

/*
 * High port numbers have a lower weight to reduce the frequency of false
 * positives, such as from passive mode FTP transfers.
 */
#define PORT_WEIGHT_PRIV		3
#define PORT_WEIGHT_HIGH		1

/*
 * Port scan detection thresholds: at least COUNT ports need to be scanned
 * from the same source, with no longer than DELAY ticks between ports.
 */
#define SCAN_MIN_COUNT			7
#define SCAN_MAX_COUNT			(SCAN_MIN_COUNT * PORT_WEIGHT_PRIV)
#define SCAN_WEIGHT_THRESHOLD		SCAN_MAX_COUNT
#define SCAN_DELAY_THRESHOLD		(300) /* old usage of HZ here was erroneously and broke under uml */

/*
 * Keep track of up to PSD_LIST_SIZE source addresses, using a hash table of
 * PSD_HASH_SIZE entries for faster lookups, but limiting hash collisions to
 * PSD_HASH_MAX source addresses per the same hash value.
 */
#define PSD_LIST_SIZE			0x100
#define PSD_HASH_LOG			9
#define PSD_HASH_SIZE			(1 << PSD_HASH_LOG)
#define PSD_HASH_MAX			0x10

struct ipt_psd_info {
	unsigned int weight_threshold;
	unsigned int delay_threshold;
	unsigned short lo_ports_weight;
	unsigned short hi_ports_weight;
};

#endif /*_IPT_PSD_H*/
