#include <linux/of_device.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/switch.h>
#include <linux/module.h>

#include "gfar.h"
#include "perf_core.h"
#include "perf_cpu.h"
#include "alt.h"

static int wait_more_duration = 10;
static int test_duration = 100;
static int finish_duration = 20;
module_param_named(duration, test_duration, int, 0444);

static int alt_tx_descriptor_count[MAX_ALT_DEVS] = { DEFAULT_DESC_COUNT };
module_param_array_named(tx_descs, alt_tx_descriptor_count, int, NULL, 0444);
static int alt_rx_descriptor_count[MAX_ALT_DEVS] = { DEFAULT_DESC_COUNT };
module_param_array_named(rx_descs, alt_rx_descriptor_count, int, NULL, 0444);

static int stream_tx_dev[MAX_STREAMS] = { 0, 1, -1, -1 };
module_param_array_named(tx_dev, stream_tx_dev, int, NULL, 0444);
static int packet_size[MAX_STREAMS] = { 60 };
module_param_array_named(size, packet_size, int, NULL, 0444);
static int packets_per_second[MAX_STREAMS] = { -1 };
module_param_array_named(pps, packets_per_second, int, NULL, 0444);
static int mbits_per_second[MAX_STREAMS] = { -1 };
module_param_array_named(mbps, mbits_per_second, int, NULL, 0444);

static int remotely_managed = 0;
module_param_named(managed, remotely_managed, int, 0444);

static int manag_dev_num = 0;
static struct test_dev_parameters manag_dev_params = {
    .tx_desc_count = 4,
    .rx_desc_count = 4,
};



static inline unsigned long long alt_get_time(void) {
    struct timespec ts;
    ktime_get_ts(&ts);
    return (unsigned long long)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

static inline void alt_delay(unsigned delay) {
    unsigned long long start_time = alt_get_time();
    unsigned long long end_time =
	start_time + (unsigned long long)delay * (NSEC_PER_HZ);

    while (1) {
	unsigned long long t = alt_get_time();
	if (t >= end_time) {
	    break;
	}
    }
}

static inline unsigned alt_get_cycles(void) {
    return perf_core_counter_get(0);
}

static inline unsigned long long alt_cycles_to_nsecs(
    unsigned long long cycles) {
    unsigned long long x = cycles * NSEC_PER_HZ;
    do_div(x, CYCLES_PER_HZ);
    return x;
}

static inline unsigned long long alt_cycles_to_nsecs_fast(
    unsigned long long cycles) {
    return (cycles * 87380) >> 16;
}

static inline unsigned long long alt_nsecs_to_cycles_fast(
    unsigned long long nsecs) {
    return (nsecs * 49152) >> 16;
}

// it happens so that 1 bit on gigabit link takes 1 nanosec
static inline unsigned long long alt_bytes_to_cycles_fast(
    unsigned long long bytes) {
    return alt_nsecs_to_cycles_fast(bytes * 8);
}


static inline void alt_tx_pack(struct alt_dev *alt_dev,
	unsigned dma_buf, unsigned len) {
    struct desc *bd;
    unsigned status;
    unsigned idx = alt_dev->tx_dirty;
    unsigned desc_count = alt_dev->params.tx_desc_count - 1;

    bd = alt_dev->tx_desc + idx;
    bd->buf = dma_buf;

    status = TXBD_READY | TXBD_LAST | TXBD_CRC;
    if (bd == alt_dev->tx_desc + desc_count) {
	status |= TXBD_WRAP;
	alt_dev->tx_dirty = 0;
    }
    else {
	++alt_dev->tx_dirty;
    }

    eieio();
    bd->length = len;
    bd->status = status;
    out_be32(&alt_dev->regs->tstat, TSTAT_CLEAR_THALT);
}

#ifdef CONFIG_REPORT
static inline void alt_send_report(struct alt_dev *target_dev,
	struct alt_dev *report_dev) {
    unsigned dma_addr;

    if (report_dev->report == &report_dev->report1) {
	dma_addr = report_dev->report1_dma_addr;
	report_dev->report = &report_dev->report2;
    }
    else {
	dma_addr = report_dev->report2_dma_addr;
	report_dev->report = &report_dev->report1;
    }
    report_dev->report->size = 0;

    alt_tx_pack(target_dev, dma_addr, sizeof(struct alt_report));
}

static inline void alt_report(struct alt_dev *alt_dev, unsigned char byte) {
    struct alt_report *r = alt_dev->report;
    r->report[r->size] = byte;
    ++r->size;
}

static inline void alt_check_send_report(struct alt_dev *alt_dev) {
    struct alt_report *r = alt_dev->report;
    if (r->size >= MAX_REPORT_FRAME) {
	if (alt_dev->alt->manag_dev) {
	    alt_send_report(alt_dev->alt->manag_dev, alt_dev);
	}
    }
}
#endif

static inline unsigned alt_prepare_tx(struct alt_tx_stream *str,
	struct alt_stream_stats *stats, unsigned char *data) {
    struct alt_header *header = (struct alt_header *)(data + ALT_HEADER_OFFSET);
    unsigned len = str->params.packet_size;

    header->magic = ALT_MAGIC;
    header->stream = str->num;
    header->seq = str->tx_seq;
    ++str->tx_seq;
    stats->tx_packet++;
    stats->tx_byte += len;
#ifdef CONFIG_MEASURE_LATENCY
    // adjust for theoretical mininum latency
    header->tstamp = alt_get_cycles() + alt_bytes_to_cycles_fast(len);
#endif

    return len;
}

static inline void alt_rx_process(struct alt_dev *alt_dev,
	unsigned char *data, unsigned len) {
    struct alt_header *header = (struct alt_header *)(data + ALT_HEADER_OFFSET);
    struct alt_rx_stream *str;
    struct alt_stream_stats *stats;
#ifdef CONFIG_MEASURE_LATENCY
    unsigned latency = alt_get_cycles() - header->tstamp;
#endif

    if (header->magic != ALT_MAGIC || header->stream >= MAX_STREAMS) {
	alt_dev->stats->rx_unknown_packet++;
	alt_dev->stats->rx_unknown_byte += len;
	return;
    }

    str = &alt_dev->rx_stream[header->stream];
    stats = &alt_dev->stats->stream[header->stream];
#ifdef CONFIG_REPORT
    if (str->rx_seq + 1 == header->seq) {
	// normal sequence
	alt_report(alt_dev, header->stream);
    }
    else {
	alt_report(alt_dev, header->stream | 0x80);
	alt_report(alt_dev, header->seq >> 24);
	alt_report(alt_dev, header->seq >> 16);
	alt_report(alt_dev, header->seq >> 8);
	alt_report(alt_dev, header->seq);
    }
#endif

    stats->rx_packet++;
    stats->rx_byte += len;
    if (str->rx_seq >= header->seq) {
	++stats->rx_out_of_order_packet;
    }
    else {
	str->rx_seq = header->seq;
    }

#ifdef CONFIG_MEASURE_LATENCY
    if (latency < stats->latency_min) {
	stats->latency_min = latency;
    }
    if (latency > stats->latency_max) {
	stats->latency_max = latency;
    }
    stats->latency_total += latency;
    latency >>= LATENCY_SHIFT;
    if (latency < LATENCY_ITEMS) {
	++stats->latency_distribution[latency];
    }
#endif

#ifdef CONFIG_REPORT
#ifdef CONFIG_MEASURE_LATENCY
    alt_report(alt_dev, latency);
#endif
    alt_check_send_report(alt_dev);
#endif
}

static inline void alt_account_tx_error(struct alt_stats *stats, unsigned s) {
    stats->tx_error++;
    if (s & TXBD_LATECOLLISION) {
	stats->tx_late_collision++;
    }
    if (s & TXBD_RETRYLIMIT) {
	stats->tx_retry_limit++;
    }
    if (s & TXBD_UNDERRUN) {
	stats->tx_underrun++;
    }
}

static inline unsigned alt_tx(struct alt_dev *alt_dev,
	struct alt_tx_stream *str, struct alt_stream_stats *stats) {
    struct desc *bd;
    unsigned status;
    unsigned char *data;
    unsigned idx = alt_dev->tx_dirty;

    bd = alt_dev->tx_desc + idx;
    status = bd->status;
    if (status & TXBD_READY) {
	return 0;
    }
    if (status & (TXBD_LATECOLLISION | TXBD_RETRYLIMIT | TXBD_UNDERRUN)) {
	alt_account_tx_error(alt_dev->stats, status);
    }

    data = buffer_store_get_buffer(&str->bufs, idx);
    bd->buf = buffer_store_get_dma_buffer(&str->bufs, idx);
    bd->length = alt_prepare_tx(str, stats, data);

    status = TXBD_READY | TXBD_LAST | TXBD_CRC;
    if (bd == alt_dev->tx_desc + alt_dev->params.tx_desc_count - 1) {
	status |= TXBD_WRAP;
    }
    eieio();
    bd->status = status;
    out_be32(&alt_dev->regs->tstat, TSTAT_CLEAR_THALT);

    alt_dev->tx_dirty = (idx + 1) & (alt_dev->params.tx_desc_count - 1);

    str->next_tx_time += str->cycles_per_packet;
    return 1;
}

static inline void alt_try_tx(struct alt_dev *alt_dev, unsigned long t) {
    unsigned i;
    for (i = 0; i < alt_dev->valid_tx_streams; ++i) {
	struct alt_tx_stream *str = &alt_dev->tx_stream[i];
	if (str->cycles_per_packet != -1 && (str->cycles_per_packet == 0 ||
			time_after_eq(t, str->next_tx_time))) {
	    alt_tx(alt_dev, str, &alt_dev->stats->stream[str->num]);
	}
    }
}

static inline void alt_account_rx_error(struct alt_stats *stats, unsigned s) {
    stats->rx_error++;
    if (s & RXBD_LARGE) {
	stats->rx_large++;
    }
    if (s & RXBD_NONOCTET) {
	stats->rx_non_octet++;
    }
    if (s & RXBD_SHORT) {
	stats->rx_short++;
    }
    if (s & RXBD_CRCERR) {
	stats->rx_crc_error++;
    }
    if (s & RXBD_OVERRUN) {
	stats->rx_overrun++;
    }
    if (s & RXBD_TRUNCATED) {
	stats->rx_truncated++;
    }
}

static inline void alt_rx_insert(struct alt_dev *alt_dev, unsigned idx,
	struct desc *bd) {
    unsigned status = RXBD_EMPTY;
    bd->buf = buffer_store_get_dma_buffer(alt_dev->rx_back, idx);

    if (bd == alt_dev->rx_desc + alt_dev->params.rx_desc_count - 1) {
	struct buffer_store *o = alt_dev->rx_front;
	alt_dev->rx_front = alt_dev->rx_back;
	alt_dev->rx_back = o;
	status |= RXBD_WRAP;
    }
    eieio();
    bd->status = status;
    bd->length = 0;

    out_be32(&alt_dev->regs->rstat, RSTAT_CLEAR_RHALT);

    alt_dev->rx_cur = (idx + 1) & (alt_dev->params.rx_desc_count - 1);
}

static inline unsigned alt_rx(struct alt_dev *alt_dev) {
    unsigned idx = alt_dev->rx_cur;
    struct desc *bd = alt_dev->rx_desc + idx;
    unsigned status;
    unsigned len;
    unsigned char *data;

    status = bd->status;
    if (status & RXBD_EMPTY) {
	return 0;
    }

    if (status & RXBD_ERR) {
	alt_account_rx_error(alt_dev->stats, status);
    } else {
	len = bd->length - ETH_FCS_LEN;

	data = buffer_store_get_buffer(alt_dev->rx_front, idx);
	alt_rx_process(alt_dev, data, len);
    }

    alt_rx_insert(alt_dev, idx, bd);
    return 1;
}

static inline unsigned alt_rx_manag(struct alt_dev *alt_dev,
	void **data, unsigned *len) {
    unsigned idx = alt_dev->rx_cur;
    struct desc *bd = alt_dev->rx_desc + idx;
    unsigned status = bd->status;
    if (status & RXBD_EMPTY) {
	return 0;
    }

    if (len) {
	*len = bd->length - ETH_FCS_LEN;
    }
    if (data) {
	*data = buffer_store_get_buffer(alt_dev->rx_front, idx);
    }

    alt_rx_insert(alt_dev, idx, bd);
    return 1;
}

static inline unsigned alt_has_rx(struct alt_dev *alt_dev) {
    unsigned idx = alt_dev->rx_cur;
    struct desc *bd = alt_dev->rx_desc + idx;
    unsigned status;

    status = bd->status;
    if (status & RXBD_EMPTY) {
	return 0;
    }
    return 1;
}


static inline void alt_reset_stats(struct alt_stats *s) {
    unsigned buf = s->buf;
    unsigned num = s->num;
    memset(s, 0, sizeof(struct alt_stats));
    s->packet_id = PACKET_ID_STATS;
    s->buf = buf;
    s->num = num;
#ifdef CONFIG_MEASURE_LATENCY
    {
	unsigned i;
	for (i = 0; i < MAX_STREAMS; ++i) {
	    s->stream[i].latency_min = -1u;
	    s->stream[i].latency_max = 0;
	}
    }
#endif
}

static inline void alt_dev_send_stats(struct alt_dev *target_dev,
	struct alt_dev *stats_dev) {
    unsigned dma_addr;

    stats_dev->stats->seq++;
    if (stats_dev->stats == &stats_dev->stats1) {
	dma_addr = stats_dev->stats1_dma_addr;
	stats_dev->stats = &stats_dev->stats2;
    }
    else {
	dma_addr = stats_dev->stats2_dma_addr;
	stats_dev->stats = &stats_dev->stats1;
    }
//    memcpy(packet->stats, stats_dev->stats, sizeof(struct alt_stats));
//    memcp(&packet->stats, &stats_dev->stats, sizeof(struct alt_stats));

    alt_tx_pack(target_dev, dma_addr, sizeof(struct alt_stats));
}

static inline void alt_send_stats(struct alt *alt) {
    struct alt_dev *alt_dev;

    if (!alt->manag_dev) {
	return;
    }

    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	alt_dev_send_stats(alt->manag_dev, alt_dev);
    }
}

static void alt_test_loop(struct alt *alt) {
    struct alt_dev *alt_dev;
    unsigned long t, p;
    unsigned long long fd = (unsigned long long)finish_duration * CYCLES_PER_HZ;
    unsigned long long start_time;
    unsigned long long end_time;
    unsigned long long break_time;
    unsigned long long stats_time;
    unsigned long long cur_time;
    unsigned i;

    p = alt_get_cycles();
    start_time = p + fd;
    end_time = start_time + (unsigned long long)alt->duration * CYCLES_PER_HZ;
    break_time = end_time + fd;
    stats_time = start_time + CYCLES_PER_SEC;
    cur_time = p;

    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	for (i = 0; i < MAX_STREAMS; ++i) {
	    alt_dev->tx_stream[i].next_tx_time = start_time;
	}
    }

    while (1) {
	t = alt_get_cycles();
	cur_time += t - p;
	p = t;

	if (cur_time >= stats_time) {
	    alt_send_stats(alt);
	    stats_time += CYCLES_PER_SEC;
	}
	if (start_time <= cur_time && cur_time < end_time) {
	    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
		alt_rx(alt_dev);
		alt_try_tx(alt_dev, t);
	    }
	}
	else {
	    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
		alt_rx(alt_dev);
	    }
	    if (cur_time >= break_time) {
		break;
	    }
	}
	if (alt->manag_dev) {
	    if (alt_has_rx(alt->manag_dev)) {
		unsigned len;
		unsigned *data;
		alt_rx_manag(alt->manag_dev, (void **)&data, &len);
		if (*data == ALT_MAGIC) {
		    if (cur_time < end_time) {
			end_time = cur_time;
			break_time = end_time + fd;
		    }
		    break;
		}
	    }
	}
//	++i;
    }
    alt_send_stats(alt);
    udelay(100);

//    printk("loop iterations: %u\n", i);
}

static inline void alt_perf_core_loop(struct alt *alt) {
    perf_core_cycles();

//    perf_cpu_eth(PERF_ETSEC_DMA_WRITE_DATA_BEATS);
//    perf_cpu_eth(PERF_ETSEC_DMA_READ_DATA_BEATS);
    perf_cpu_gen(PERF_INSN_L2_MISS);
    perf_cpu_gen(PERF_DATA_L2_MISS);
    perf_cpu_gen(PERF_INSN_L2_HIT);
    perf_cpu_gen(PERF_DATA_L2_HIT);
//    perf_cpu_gen(PERF_DRAM_READ);
//    perf_cpu_gen(PERF_DRAM_WRITE);

    perf_core_enable();
    perf_cpu_enable();

    alt_test_loop(alt);
//    alt_delay(100);

    perf_core_disable();
    perf_cpu_disable();

    perf_core_show();
    perf_cpu_show();
}

static inline void alt_dev_dump_descs(struct alt_dev *alt_dev) {
    unsigned i;

    printk("alt%u tx descs:\n", alt_dev->num);
    for (i = 0; i < alt_dev->params.tx_desc_count; ++i) {
	if (i && !(i % 8)) printk("\n");
	printk("%08x ", alt_dev->tx_desc[i].buf);
    }
    printk("\n");
    printk("alt%u rx descs:\n", alt_dev->num);
    for (i = 0; i < alt_dev->params.rx_desc_count; ++i) {
	if (i && !(i % 4)) printk("\n");
	printk("%02x: s:%04x l:%04x %08x ", i,
		alt_dev->rx_desc[i].status,
		alt_dev->rx_desc[i].length,
		alt_dev->rx_desc[i].buf);
    }
    printk("\n");
}

static inline void alt_dump_descs(struct alt *alt) {
    struct alt_dev *alt_dev;
    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	alt_dev_dump_descs(alt_dev);
    }
}

static inline void alt_heat_mem(const void *data, unsigned len) {
    unsigned char *x = (unsigned char *)data;
    unsigned i;
    unsigned sum = 0;
    for (i = 0; i < len; ++i) {
	sum += x[i];
    }
}

static void alt_heat_caches(struct alt *alt) {
    struct alt_dev *alt_dev;
    unsigned i;
    unsigned x = 0;

    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	for (i = 0; i < alt_dev->params.tx_desc_count; ++i) {
	    x += alt_dev->tx_desc[i].buf;
	}
	for (i = 0; i < alt_dev->params.rx_desc_count; ++i) {
	    x += alt_dev->rx_desc[i].buf;
	    x += *buffer_store_get_buffer(&alt_dev->rx_bufs1, i);
	    x += *buffer_store_get_buffer(&alt_dev->rx_bufs2, i);
	}
	alt_heat_mem(&alt_dev->stats1, sizeof(struct alt_stats));
	alt_heat_mem(&alt_dev->stats2, sizeof(struct alt_stats));
    }
}

static struct alt_dev *alt_get_dev(struct alt *alt, unsigned num) {
    struct alt_dev *alt_dev;
    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	if (alt_dev->num == num) {
	    return alt_dev;
	}
    }
    return NULL;
}

static void alt_setup_buf(unsigned char *buf,
	struct stream_parameters *params) {
    unsigned i = 0;
    unsigned char byte = params->fill_byte;

    memcpy(&buf[i], params->header, params->header_len);
    i += params->header_len;
    for (; i < RX_BUF_SIZE; ++i) {
	buf[i] = byte;
	byte += params->fill_inc;
    }
}

static void alt_setup_stream(struct alt *alt,
	struct stream_parameters *params, unsigned num) {
    struct alt_dev *alt_dev;
    struct alt_tx_stream *str;
    unsigned long long x;
    unsigned i;

    if (params->tx_dev_num == -1) {
	return;
    }
    alt_dev = alt_get_dev(alt, params->tx_dev_num);
    if (!alt_dev) {
	printk("alt: could not find dev #%u\n", params->tx_dev_num);
	return;
    }
    if (alt_dev == alt->manag_dev) {
	printk("alt: cannot setup stream on managment dev #%u\n",
		params->tx_dev_num);
	return;
    }

    if (alt_dev->valid_tx_streams >= MAX_STREAMS) {
	printk("alt%u: too many streams\n", alt_dev->num);
	return;
    }
    
    str = &alt_dev->tx_stream[alt_dev->valid_tx_streams];

    buffer_store_free(&str->bufs);
    if (buffer_store_init(
		&str->bufs, alt_dev->params.tx_desc_count, RX_BUF_SIZE, 6,
		DMA_TO_DEVICE)) {
	printk("alt%u: cannot allocat stream buffers\n", alt_dev->num);
	return;
    }

    ++alt_dev->valid_tx_streams;
    memcpy(&str->params, params, sizeof(*params));
    str->num = num;
    str->tx_seq = 1;

    str->target_bitrate = -1;
    if (params->packets_per_second != -1) {
	str->target_bitrate =
	    params->packets_per_second *
	    (params->packet_size + ETH_FRAME_OVERHEAD) * 8;
    }
    else if (params->mbits_per_second != -1) {
	str->target_bitrate = params->mbits_per_second * 1000000;
    }
    if (str->target_bitrate != -1) {
	if (str->target_bitrate) {
	    x = (unsigned long long)CYCLES_PER_SEC *
		(params->packet_size + ETH_FRAME_OVERHEAD) * 8;
	    do_div(x, str->target_bitrate);
	    str->cycles_per_packet = x;
	}
	else {
	    str->cycles_per_packet = -1;
	}
	printk("alt%u str%u: target_bitrate %u, cycles_per_packet: %u\n",
		alt_dev->num, str->num,
		str->target_bitrate, str->cycles_per_packet);
    }
    else {
	str->cycles_per_packet = 0;
    }
    str->next_tx_time = 0;

    for (i = 0; i < str->bufs.count; ++i) {
	alt_setup_buf(buffer_store_get_buffer(&str->bufs, i), params);
    }
}

static void alt_dev_cleanup_rx_buffers(struct alt_dev *alt_dev) {
    buffer_store_free(&alt_dev->rx_bufs1);
    buffer_store_free(&alt_dev->rx_bufs2);
}

static int alt_dev_init_rx_buffers(struct alt_dev *alt_dev) {
    int ret;
    struct test_dev_parameters *p = &alt_dev->params;

    alt_dev_cleanup_rx_buffers(alt_dev);
    if (p->tx_desc_count > alt_dev->mac->tx_desc_per_queue_count ||
	    p->rx_desc_count > alt_dev->mac->rx_desc_per_queue_count) {
	printk("alt%u: too many descs %u > %u || %u > %u\n", alt_dev->num,
		p->tx_desc_count, alt_dev->mac->tx_desc_per_queue_count,
		p->rx_desc_count, alt_dev->mac->rx_desc_per_queue_count);
	return -EINVAL;
    }
    ret = buffer_store_init(
	&alt_dev->rx_bufs1, p->rx_desc_count, RX_BUF_SIZE, 6,
	DMA_FROM_DEVICE);
    if (ret) {
	goto err;
    }
    ret = buffer_store_init(
	&alt_dev->rx_bufs2, p->rx_desc_count, RX_BUF_SIZE, 6,
	DMA_FROM_DEVICE);
    if (ret) {
	goto err;
    }
    alt_dev->rx_front = &alt_dev->rx_bufs1;
    alt_dev->rx_back = &alt_dev->rx_bufs2;
    return 0;
err:
    alt_dev_cleanup_rx_buffers(alt_dev);
    return ret;
}

static void alt_dev_stop_dma(struct alt_dev *alt_dev) {
    struct gfar *regs = alt_dev->regs;
    unsigned x;

    x = in_be32(&regs->dmactrl);
    x |= (DMACTRL_GRS | DMACTRL_GTS);
    out_be32(&regs->dmactrl, x);
    while (!(in_be32(&regs->ievent) & (IEVENT_GRSC | IEVENT_GTSC))) {
	cpu_relax();
    }
}

static void alt_dev_start_dma(struct alt_dev *alt_dev) {
    struct gfar *regs = alt_dev->regs;
    unsigned x;

    x = in_be32(&regs->tx[0].tbase);
    out_be32(&regs->tx[0].tbase, x);
    x = in_be32(&regs->rx[0].rbase);
    out_be32(&regs->rx[0].rbase, x);

    x = in_be32(&regs->dmactrl);
    x = DMACTRL_INIT_SETTINGS;
    out_be32(&regs->dmactrl, x);
}

static void alt_setup_rings(struct alt_dev *alt_dev) {
    unsigned i;
    struct desc *bd = NULL;

    alt_dev_stop_dma(alt_dev);

    if (alt_dev_init_rx_buffers(alt_dev)) {
	return;
    }

//    printk("setup rings alt%u rx/tx descs: %u/%u\n", alt_dev->num,
//	    alt_dev->params.rx_desc_count,
//	    alt_dev->params.tx_desc_count);
    for (i = 0; i < alt_dev->params.tx_desc_count; ++i) {
	bd = alt_dev->tx_desc + i;
	bd->status = 0;
    }

    for (i = 0; i < alt_dev->params.rx_desc_count; ++i) {
	bd = alt_dev->rx_desc + i;
	bd->length = RX_BUF_SIZE;
	bd->buf = buffer_store_get_dma_buffer(alt_dev->rx_front, i);
	bd->status = RXBD_EMPTY;
    }
    bd->status |= RXBD_WRAP;

    alt_dev->tx_dirty = 0;
    alt_dev->rx_cur = 0;

    alt_dev->rx_front = &alt_dev->rx_bufs1;
    alt_dev->rx_back = &alt_dev->rx_bufs2;


    alt_dev_start_dma(alt_dev);
}

static void alt_reset_streams(struct alt *alt) {
    struct alt_dev *alt_dev;
    unsigned i;

    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	alt_dev->valid_tx_streams = 0;
	for (i = 0; i < MAX_STREAMS; ++i) {
	    struct alt_tx_stream *str = &alt_dev->tx_stream[i];
	    memset(&str->params, 0, sizeof(str->params));
	    str->tx_seq = 1;
	    str->target_bitrate = -1;
	    str->cycles_per_packet = 0;
	    str->next_tx_time = 0;
	}
	memset(alt_dev->rx_stream, 0, sizeof(alt_dev->rx_stream));
    }
}

static void alt_setup_test_from_module_params(struct alt *alt) {
    unsigned i;
    struct stream_parameters params;

    alt->duration = test_duration;

    alt_reset_streams(alt);
    for (i = 0; i < MAX_STREAMS; ++i) {
	if (stream_tx_dev[i] == -1) {
	    break;
	}
	if (!packet_size[i]) {
	    packet_size[i] = packet_size[i - 1];
	}
	if (!packets_per_second[i]) {
	    packets_per_second[i] = packets_per_second[i - 1];
	}
	if (!mbits_per_second[i]) {
	    mbits_per_second[i] = mbits_per_second[i - 1];
	}

	memset(&params, 0, sizeof(params));
	params.tx_dev_num = stream_tx_dev[i];
	params.packet_size = packet_size[i];
	params.packets_per_second = packets_per_second[i];
	params.mbits_per_second = mbits_per_second[i];

	alt_setup_stream(alt, &params, i);
    }

}

static void alt_start_test(struct alt *alt) {
    unsigned long flags;
    struct alt_dev *alt_dev;

    printk("alt start @%lu sizeof stats: %u, stream stats: %u\n",
	    jiffies, sizeof(struct alt_stats), sizeof(struct alt_stream_stats));
    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	alt_setup_rings(alt_dev);
	alt_reset_stats(&alt_dev->stats1);
	alt_reset_stats(&alt_dev->stats2);
    }

    perf_core_counter_setup(0, PERF_CYCLES);
    perf_core_enable();
    local_irq_save(flags);
    alt_heat_caches(alt);
    alt_test_loop(alt);

    perf_core_disable();
//    perf_core_show();

    local_irq_restore(flags);
}

static void alt_change_status(struct alt *alt, unsigned new_status) {
//    printk("%s: alt status %u->%u @%lu\n",
//	    alt->dev->name, alt->status, new_status, jiffies);
    alt->last_status_change = jiffies;
    alt->status = new_status;
}

static void alt_setup_test_managed(struct alt *alt,
	struct test_parameters *params) {
    unsigned i;
    struct alt_dev *alt_dev;

    alt->duration = params->duration * HZ;

    for (i = 0; i < MAX_ALT_DEVS; ++i) {
	alt_dev = alt_get_dev(alt, i);
	if (!alt_dev) {
	    continue;
	}
	if (alt_dev == alt->manag_dev) {
	    continue;
	}
	memcpy(&alt_dev->params, &params->devs[i],
		sizeof(struct test_dev_parameters));
    }

    alt_reset_streams(alt);
    for (i = 0; i < MAX_STREAMS; ++i) {
	alt_setup_stream(alt, &params->streams[i], i);
    }
//    alt_change_status(alt, TEST_WAIT_FOR_LINK);
    alt_change_status(alt, TEST_WAIT_MORE);
}

static void alt_setup_packet(struct alt *alt, struct setup_packet_frame *frame) {
//    unsigned i;
    struct alt_dev *alt_dev = alt_get_dev(alt, frame->tx_dev_num);
    if (!alt_dev) {
	return;
    }

//    printk("send setup packet %u %u\n", frame->tx_dev_num, frame->len);
    memcpy(alt_dev->setup_packet, frame->data, frame->len);
//    for (i = 0; i < alt_dev->params.tx_desc_count; ++i) {
	alt_tx_pack(alt_dev, alt_dev->setup_packet_dma_addr, frame->len);
//    }
}

static void alt_manage(struct alt *alt) {
    struct alt_dev *alt_dev;
    unsigned len;
    union {
	struct start_test_frame *start;
	struct setup_packet_frame *setup;
    } x;

    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	while (alt_has_rx(alt_dev)) {
	    alt_rx_manag(alt_dev, NULL, NULL);
	}
    }

    if (!alt->manag_dev) {
	return;
    }
//    alt_dev_dump_descs(alt->manag_dev);
    if (!alt_has_rx(alt->manag_dev)) {
	return;
    }
    alt_rx_manag(alt->manag_dev, (void **)&x, &len);

    if (len < 8) {
	printk("got strange management frame len: %u\n", len);
	return;
    }
    if (x.start->magic != ALT_MAGIC) {
//	printk("bad magic %x != %x\n", x.start->magic, ALT_MAGIC);
	return;
    }

    switch (x.start->cmd) {
    case ALT_CMD_START:
	if (len != sizeof(struct start_test_frame)) {
	    printk("got strange start frame len: %u != %u\n",
		    len, sizeof(struct start_test_frame));
	    break;
	}
	alt_setup_test_managed(alt, &x.start->params);
	break;
    case ALT_CMD_STOP:
	break;
    case ALT_CMD_SETUP_PACKET:
	if (len < sizeof(struct setup_packet_frame)) {
	    printk("got strange setup packet frame len: %u < %u\n",
		    len, sizeof(struct setup_packet_frame));
	    break;
	}
	alt_setup_packet(alt, x.setup);
	break;
    default:
	printk("alt%u: got strange management frame cmd: %u, len: %u\n",
		alt->manag_dev->num, x.start->cmd, len);
	break;
    }
}

static void alt_setup_managed_mode(struct alt *alt) {
    struct alt_dev *alt_dev = alt->manag_dev;
    if (!alt_dev) {
	return;
    }
    alt_setup_rings(alt_dev);
    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	alt_setup_rings(alt_dev);
    }

    alt_change_status(alt, TEST_MANAGED);
}

static void alt_sum_stream_stats(struct alt_stream_stats *s1,
	struct alt_stream_stats *s2) {
    s1->tx_packet += s2->tx_packet;
    s1->tx_byte += s2->tx_byte;
    s1->rx_packet += s2->rx_packet;
    s1->rx_byte += s2->rx_byte;
    s1->rx_out_of_order_packet += s2->rx_out_of_order_packet;
#ifdef CONFIG_MEASURE_LATENCY
    if (s1->latency_min > s2->latency_min) {
	s1->latency_min = s2->latency_min;
    }
    if (s1->latency_max < s2->latency_max) {
	s1->latency_max = s2->latency_max;
    }
    s1->latency_total += s2->latency_total;
    {
	unsigned i;
	for (i = 0; i < LATENCY_ITEMS; ++i) {
	    s1->latency_distribution[i] += s2->latency_distribution[i];
	}
    }
#endif
}

static void alt_sum_stats(struct alt_dev *alt_dev) {
    struct alt_stats *s1 = &alt_dev->stats1;
    struct alt_stats *s2 = &alt_dev->stats2;
    unsigned i;
    for (i = 0; i < MAX_STREAMS; ++i) {
	alt_sum_stream_stats(&s1->stream[i], &s2->stream[i]);
    }
    s1->tx_error += s2->tx_error;
    s1->tx_late_collision += s2->tx_late_collision;
    s1->tx_retry_limit += s2->tx_retry_limit;
    s1->tx_underrun += s2->tx_underrun;

    s1->rx_unknown_packet += s2->rx_unknown_packet;
    s1->rx_unknown_byte += s2->rx_unknown_byte;
    s1->rx_error += s2->rx_error;
    s1->rx_large += s2->rx_large;
    s1->rx_non_octet += s2->rx_non_octet;
    s1->rx_short += s2->rx_short;
    s1->rx_crc_error += s2->rx_crc_error;
    s1->rx_overrun += s2->rx_overrun;
    s1->rx_truncated += s2->rx_truncated;

    alt_reset_stats(s2);
    alt_dev->stats = &alt_dev->stats1;
}

static void alt_dump_stats(struct alt_dev *alt_dev) {
    unsigned i;
    struct alt_stats *stats;
    unsigned tx_packet = 0;
    unsigned rx_packet = 0;
    unsigned long long t = 0;
    unsigned long long r = 0;

    alt_sum_stats(alt_dev);
    stats = alt_dev->stats;

    for (i = 0; i < MAX_STREAMS; ++i) {
	tx_packet += stats->stream[i].tx_packet;
	rx_packet += stats->stream[i].rx_packet;
	t += stats->stream[i].tx_byte;
	r += stats->stream[i].rx_byte;
    }
    rx_packet += stats->rx_unknown_packet;
    r += stats->rx_unknown_byte;
    t += tx_packet * ETH_FRAME_OVERHEAD;
    r += rx_packet * ETH_FRAME_OVERHEAD;
    t *= 8;
    r *= 8;
    do_div(t, 10000);
    do_div(r, 10000);
    do_div(t, alt_dev->alt->duration / HZ);
    do_div(r, alt_dev->alt->duration / HZ);

    printk("alt%u: tx/rx packets %u/%u", alt_dev->num, tx_packet, rx_packet);
    printk(" pipe %u.%03u%%/%u.%03u%%",
	    (unsigned)t / 1000, (unsigned)t % 1000,
	    (unsigned)r / 1000, (unsigned)r % 1000);
    if (stats->tx_error || stats->rx_error) {
	printk(", errors %u/%u", stats->tx_error, stats->rx_error);
    }
    printk("\n");

    for (i = 0; i < MAX_STREAMS; ++i) {
	struct alt_stream_stats *s = &stats->stream[i];
	if (s->tx_packet || s->rx_packet) {
	    printk("alt%u str%u: tx/rx packets %u/%u", alt_dev->num, i,
		    stats->stream[i].tx_packet,
		    stats->stream[i].rx_packet
		);
	    if (s->rx_out_of_order_packet) {
		printk(", rx out of order %u", s->rx_out_of_order_packet);
	    }
	    printk("\n");
	}
    }
}


static void alt_dump_rmon_stats(struct alt_dev *alt_dev) {
    u32 __iomem *rmon = (u32 __iomem *)&alt_dev->regs->rmon;
    struct rmon_mib __iomem *r = &alt_dev->regs->rmon;
//    unsigned rxb = in_be32(&r->rbyt);
//    unsigned txb = in_be32(&r->tbyt);
    int i;
    static char *d[] = {
	"tx/rx 64",
	"tx/rx 65-127",
	"tx/rx 128-255",
	"tx/rx 256-511",
	"tx/rx 512-1023",
	"tx/rx 1024-1518",
	"tx/rx 1519-1522",
	"rx byte",
	"rx packet",
	"rx fcs err",
	"rx multi",
	"rx broad",
	"rx control",
	"rx pause",
	"rx unknown op",
	"rx alignment err",
	"rx length err",
	"rx code err",
	"rx carrier err",
	"rx undersize",
	"rx oversize",
	"rx fragment",
	"rx jabber",
	"rx drop",
	"tx byte",
	"tx packet",
	"tx multi",
	"tx broad",
	"tx pause",
	"tx deferral",
	"tx exc deferral",
	"tx single col",
	"tx multi col",
	"tx late col",
	"tx exc col",
	"tx total col",
	"xxx",
	"tx drop",
	"tx jabber",
	"tx fcs err",
	"tx control",
	"tx oversize",
	"tx undersize",
	"tx fragment",
	"carry one",
	"carry two",
	"carry mask one",
	"carry mask two",
    };
    unsigned interesting = 0;

    for (i = 0; i < GFAR_RMON_LEN; i++) {
	if (&rmon[i] == &r->tr64) continue;
	if (&rmon[i] == &r->tr127) continue;
	if (&rmon[i] == &r->tr255) continue;
	if (&rmon[i] == &r->tr511) continue;
	if (&rmon[i] == &r->tr1k) continue;
	if (&rmon[i] == &r->trmax) continue;
	if (&rmon[i] == &r->trmgv) continue;
	if (&rmon[i] == &r->rbyt) continue;
	if (&rmon[i] == &r->rpkt) continue;
	if (&rmon[i] == &r->tbyt) continue;
	if (&rmon[i] == &r->tpkt) continue;
	if (in_be32(&rmon[i])) {
	    interesting = 1;
	}
    }
    if (!interesting) {
	return;
    }

    printk("alt%u rmon: ", alt_dev->num);
    for (i = 0; i < GFAR_RMON_LEN; i++) {
	unsigned x = in_be32(&rmon[i]);
	if (x) {
	    printk("%s: %u, ", d[i], x);
	}
    }
//    printk("tx/rx packet adj: %u/%u",
//	    txb / (alt_dev->params.packet_size + ETH_FCS_LEN),
//	    rxb / (alt_dev->params.packet_size + ETH_FCS_LEN));
    printk("\n");
}

#ifdef CONFIG_MEASURE_LATENCY
static void alt_dump_latency_stats(struct alt_dev *alt_dev,
	struct alt_rx_stream *str,
	struct alt_stream_stats *stats, unsigned num) {
    unsigned i;
    unsigned ns;
    unsigned ns_min;
    unsigned ns_avg;
    unsigned ns_max;
    unsigned first = LATENCY_ITEMS;
    unsigned last = 0;

    if (!stats->rx_packet) {
	return;
    }

    do_div(stats->latency_total, stats->rx_packet);
    ns_min = alt_cycles_to_nsecs(stats->latency_min);
    ns_avg = alt_cycles_to_nsecs(stats->latency_total);
    ns_max = alt_cycles_to_nsecs(stats->latency_max);
    printk("alt%u str%u: latency min/avg/max: %u.%03uus/%u.%03uus/%u.%03uus\n",
	    alt_dev->num, num,
	    ns_min / 1000, ns_min % 1000,
	    ns_avg / 1000, ns_avg % 1000,
	    ns_max / 1000, ns_max % 1000
	);
    for (i = LATENCY_ITEMS - 1; i > 0; --i) {
	if (stats->latency_distribution[i]) {
	    last = i;
	    break;
	}
    }

    for (i = 0; i < last; ++i) {
	if (stats->latency_distribution[i]) {
	    first = i;
	    break;
	}
    }
    for (i = first; i <= last; ++i) {
	ns = alt_cycles_to_nsecs(i << LATENCY_SHIFT);
	printk("%u.%uus: %u\n", ns / 1000, (ns / 100) % 10,
		stats->latency_distribution[i]);
    }
}
#endif

static void alt_dump(struct alt *alt) {
    struct alt_dev *alt_dev;
    unsigned i;
    unsigned tx_packet = 0;
    unsigned rx_packet = 0;
    unsigned tx_error = 0;
    unsigned rx_error = 0;

    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	alt_dump_stats(alt_dev);
    }

    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	for (i = 0; i < MAX_STREAMS; ++i) {
	    tx_packet += alt_dev->stats->stream[i].tx_packet;
	    rx_packet += alt_dev->stats->stream[i].rx_packet;
	}
	tx_error += alt_dev->stats->tx_error;
	rx_error += alt_dev->stats->rx_error;
    }
    printk("total: tx/rx packets %u/%u (lost: %u)\n",
	    tx_packet, rx_packet,
	    tx_packet - rx_packet);


#ifdef CONFIG_MEASURE_LATENCY
    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	for (i = 0; i < MAX_STREAMS; ++i) {
	    alt_dump_latency_stats(alt_dev, &alt_dev->rx_stream[i],
		    &alt_dev->stats->stream[i], i);
	}
    }
#endif
    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	alt_dump_rmon_stats(alt_dev);
    }
}

static void alt_disable_regular(struct alt *alt) {
    unsigned i;
    for (i = 0; i < alt->regular_dev_count; ++i) {
	struct net_device *dev = alt->regular_devs[i].dev;
	int open = netif_running(dev);
	if (open) {
	    dev_close(dev);
	}
	alt->regular_devs[i].was_open = open;
    }
}

static void alt_enable_regular(struct alt *alt) {
    unsigned i;
    for (i = 0; i < MAX_REGULAR_DEVS; ++i) {
	struct net_device *dev = alt->regular_devs[i].dev;
	if (!dev) {
	    break;
	}
	if (alt->regular_devs[i].was_open) {
	    dev_open(dev);
	}
    }
}

static int alt_check_link(struct alt *alt) {
    struct alt_dev *alt_dev;
    list_for_each_entry(alt_dev, &alt->test_dev_head, test_list) {
	if (!alt_dev->mac->link) {
	    return 0;
	}
    }
    return 1;
}

static void alt_open(struct alt_dev *alt_dev) {
    unsigned rctrl;
//    printk("alt%u: tx fifo thr: %08x, starve: %08x, starve shutoff: %08x\n",
//	    alt_dev->num,
//	    in_be32(&alt_dev->regs->fifo_tx_thr),
//	    in_be32(&alt_dev->regs->fifo_tx_starve),
//	    in_be32(&alt_dev->regs->fifo_tx_starve_shutoff));

//    out_be32(&alt_dev->regs->fifo_tx_thr, 0x4);
//    out_be32(&alt_dev->regs->fifo_tx_starve, 0x08);
//    out_be32(&alt_dev->regs->fifo_tx_starve_shutoff, 0x10);

    dev_open(alt_dev->mac->dev);

    // disable padding if any
    rctrl = in_be32(&alt_dev->regs->rctrl);
    rctrl &= ~RCTRL_PAL_MASK;
    out_be32(&alt_dev->regs->rctrl, rctrl);

//    alt_dev->tx_desc = alt_dev->mac->tx_desc;
//    alt_dev->rx_desc = alt_dev->mac->rx_desc;

//    printk("alt%u tx_desc: %p %u, rx_desc: %p %u\n", alt_dev->num,
//	    alt_dev->tx_desc, alt_dev->params.tx_desc_count,
//	    alt_dev->rx_desc, alt_dev->params.rx_desc_count);
}

static void alt_open_task(struct work_struct *work) {
    struct alt *alt = container_of(work, struct alt, open_task);
    struct alt_dev *alt_dev;

//    printk("%s: open_task\n", alt->dev->name);
    rtnl_lock();
    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	if (!netif_running(alt_dev->mac->dev)) {
	    alt_open(alt_dev);
	}
    }
    rtnl_unlock();
    alt_change_status(alt, TEST_OPENED);
}

static void alt_close_task(struct work_struct *work) {
    struct alt *alt = container_of(work, struct alt, close_task);
    struct alt_dev *alt_dev;

//    printk("%s: close_task\n", alt->dev->name);
    rtnl_lock();
    list_for_each_entry(alt_dev, &alt->dev_head, list) {
	struct net_device *dev = alt_dev->mac->dev;
	if (netif_running(dev)) {
	    dev_close(dev);
	}
    }
    rtnl_unlock();
}

static void alt_open_regular_task(struct work_struct *work) {
    struct alt *alt = container_of(work, struct alt, open_regular_task);
//    printk("open_regular_task\n");
    rtnl_lock();
    alt_enable_regular(alt);
    rtnl_unlock();
}

static void alt_close_regular_task(struct work_struct *work) {
    struct alt *alt = container_of(work, struct alt, close_regular_task);
//    printk("close_regular_task\n");
    rtnl_lock();
    alt_disable_regular(alt);
    rtnl_unlock();
}

static void alt_timer(unsigned long data) {
    struct alt *alt = (struct alt *)data;
    unsigned long t;
//    printk("alt timer\n");
    mod_timer(&alt->timer, jiffies + HZ / 10);

    switch (alt->status) {
    case TEST_OPEN:
	schedule_work(&alt->open_task);
	break;
    case TEST_OPENED:
	if (remotely_managed) {
	    alt_setup_managed_mode(alt);
	}
	else {
//	    alt_change_status(alt, TEST_WAIT_FOR_LINK);
	    alt_change_status(alt, TEST_WAIT_MORE);
	    alt_setup_test_from_module_params(alt);
	}
	break;
    case TEST_MANAGED:
	alt_manage(alt);
	break;
    case TEST_WAIT_FOR_LINK:
	if (alt_check_link(alt)) {
	    schedule_work(&alt->close_regular_task);
	    alt_change_status(alt, TEST_WAIT_MORE);
	}
	break;
    case TEST_WAIT_MORE:
	t = alt->last_status_change + wait_more_duration;
	if (time_after_eq(jiffies, t)) {
	    alt_change_status(alt, TEST_IN_PROGRESS);
	    alt_start_test(alt);
	    alt_change_status(alt, TEST_FINISH);
	    alt_dump(alt);
	    schedule_work(&alt->open_regular_task);
	}
	break;
    case TEST_FINISH:
	if (remotely_managed) {
	    alt_change_status(alt, TEST_OPEN);
	    schedule_work(&alt->close_task);
//	    alt_setup_managed_mode(alt);
	}
	else {
	    alt_change_status(alt, TEST_DONE);
	    schedule_work(&alt->close_task);
	}
	break;
    case TEST_DONE:
	printk("done\n");
	break;
    default:
	break;
    }
}

static int alt_init(struct alt *alt) {
    int ret;
    unsigned i;

    if (alt->dev_head.prev) {
	// already initialized
	return 0;
    }

    ret = perf_cpu_init();
    if (ret) {
	return ret;
    }

    for (i = 1; i < MAX_ALT_DEVS; ++i) {
	if (!alt_tx_descriptor_count[i]) {
	    alt_tx_descriptor_count[i] = alt_tx_descriptor_count[i - 1];
	}
	if (!alt_rx_descriptor_count[i]) {
	    alt_rx_descriptor_count[i] = alt_rx_descriptor_count[i - 1];
	}
    }

    INIT_LIST_HEAD(&alt->dev_head);
    INIT_LIST_HEAD(&alt->test_dev_head);
    alt->manag_dev = NULL;
    init_timer(&alt->timer);
    alt->timer.data = (unsigned long)alt;
    alt->timer.function = alt_timer;
    mod_timer(&alt->timer, jiffies);
    INIT_WORK(&alt->open_task, alt_open_task);
    INIT_WORK(&alt->close_task, alt_close_task);
    INIT_WORK(&alt->open_regular_task, alt_open_regular_task);
    INIT_WORK(&alt->close_regular_task, alt_close_regular_task);

    return 0;
}

static void alt_cleanup(struct alt *alt) {
    perf_cpu_uninit();

    del_timer_sync(&alt->timer);
    cancel_work_sync(&alt->open_task);
    cancel_work_sync(&alt->close_task);
    memset(alt, 0, sizeof(*alt));
//    printk("cleanup test status %u\n", alt->status);
}

static void alt_dev_cleanup(struct alt_dev *alt_dev) {
    if (alt_dev->list.prev) {
	list_del(&alt_dev->list);
    }
    if (alt_dev->test_list.prev) {
	list_del(&alt_dev->test_list);
    }
    if (alt_dev->alt->manag_dev == alt_dev) {
	alt_dev->alt->manag_dev = NULL;
    }

    alt_dev_cleanup_rx_buffers(alt_dev);
    kfree(alt_dev);
}

static void alt_dev_init(struct alt_dev *alt_dev) {
    struct test_dev_parameters *p = &alt_dev->params;
//    printk("alt dev init %u\n", alt_dev->num);

    if (remotely_managed && alt_dev->num == manag_dev_num) {
	memcpy(p, &manag_dev_params, sizeof(*p));
    }
    else {
	p->tx_desc_count = alt_tx_descriptor_count[alt_dev->num];
	p->rx_desc_count = alt_rx_descriptor_count[alt_dev->num];
    }

    alt_dev->stats1.packet_id = PACKET_ID_STATS;
    alt_dev->stats1.buf = 0;
    alt_dev->stats1.num = alt_dev->num;
    alt_dev->stats1_dma_addr = dma_map_single(
	NULL, &alt_dev->stats1, sizeof(alt_dev->stats1), DMA_TO_DEVICE);
    alt_dev->stats2.packet_id = PACKET_ID_STATS;
    alt_dev->stats2.buf = 1;
    alt_dev->stats2.num = alt_dev->num;
    alt_dev->stats2_dma_addr = dma_map_single(
	NULL, &alt_dev->stats2, sizeof(alt_dev->stats2), DMA_TO_DEVICE);
    alt_dev->stats = &alt_dev->stats1;

#ifdef CONFIG_REPORT
    alt_dev->report1.packet_id = PACKET_ID_REPORT;
    alt_dev->report1.num = alt_dev->num;
    alt_dev->report1_dma_addr = dma_map_single(
	NULL, &alt_dev->report1, sizeof(alt_dev->report1), DMA_TO_DEVICE);
    alt_dev->report2.packet_id = PACKET_ID_REPORT;
    alt_dev->report2.num = alt_dev->num;
    alt_dev->report2_dma_addr = dma_map_single(
	NULL, &alt_dev->report2, sizeof(alt_dev->report2), DMA_TO_DEVICE);
    alt_dev->report = &alt_dev->report1;
#endif
    alt_dev->setup_packet_dma_addr = dma_map_single(
	NULL, &alt_dev->setup_packet,
	sizeof(alt_dev->setup_packet), DMA_TO_DEVICE);

    if (remotely_managed && alt_dev->num == manag_dev_num) {
//	printk("set manag dev\n");
	alt_dev->alt->manag_dev = alt_dev;
    }
    else {
	list_add_tail(&alt_dev->test_list, &alt.test_dev_head);
    }
}

void gfar_init_alternative(struct gfar_mac *mac) {
    struct alt_dev *alt_dev;
    static unsigned num = 0;
//    printk("%s: init alternative function\n", dev->name);

    if (alt_init(&alt)) {
	return;
    }

    memcpy(mac->dev->name, "alt", 3);
#ifdef DUAL_NAPI
    netif_napi_del(&mac->napi_rx);
    netif_napi_del(&mac->napi_tx);
#else
//    netif_napi_del(&mac->napi);
#endif

    alt_dev = kzalloc(sizeof(struct alt_dev), GFP_ATOMIC);
    if (!alt_dev) {
	return;
    }

    alt_dev->num = num++;
    alt_dev->mac = mac;
    alt_dev->alt = &alt;
    alt_dev->regs = mac->regs;
    alt_dev_init(alt_dev);

    list_add_tail(&alt_dev->list, &alt.dev_head);
}

void gfar_init_regular(struct net_device *dev) {
    if (alt.regular_dev_count >= MAX_REGULAR_DEVS) {
	return;
    }
//    printk("init regular %s %u\n", dev->name, alt.regular_dev_count);
    alt.regular_devs[alt.regular_dev_count].dev = dev;
    alt.regular_dev_count++;
}

void gfar_uninit_alternative(struct gfar_mac *mac) {
    struct alt_dev *alt_dev;
    if (alt.dev_head.prev == NULL) {
	return;
    }
    if (list_empty(&alt.dev_head)) {
	return;
    }
    list_for_each_entry(alt_dev, &alt.dev_head, list) {
	if (alt_dev->mac == mac) {
	    alt_dev_cleanup(alt_dev);
	    break;
	}
    }

    if (list_empty(&alt.dev_head)) {
	alt_cleanup(&alt);
    }
}

