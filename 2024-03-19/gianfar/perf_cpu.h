#ifndef PERF_CPU_H
#define PERF_CPU_H

#define PERF_BASE 0xe00e1000
#define PERF_SIZE 0x100
#define PERF_REG_A(counter) (4 + (4 * (counter)) + 0)
#define PERF_REG_B(counter) (4 + (4 * (counter)) + 1)
#define PERF_REG_CNT(counter) (4 + (4 * (counter)) + 2)
#define PERF_REG_CNT_UPPER(counter) (4 + (4 * (counter)) + 3)
static unsigned *perf_regs;

struct perf_cpu_counter_event {
    unsigned counter;
    unsigned event;
};
struct perf_cpu_counter {
    struct perf_cpu_counter_event c;
    const char *name;
};
struct perf_cpu_eth_counter {
    struct perf_cpu_counter_event c[4];
    const char *name;
};

/*
// MPC8548 counters
static struct perf_cpu_counter perf_cpu_counters[] = {
    { {-1, 19}, "dram read" },
    { {-1, 11}, "draw write" },
    { {-1, 13}, "reads or writes from core" },
    { {3, 1}, "reads or writes from eTSEC 1-4" },
    { {3, 2}, "reads or writes from PCI" },
    { {4, 3}, "reads or writes from PCI Express / Serial RapidIO" },
    { {5, 2}, "reads or writes from DMA" },
    { {6, 5}, "reads or writes from Security" },
    { {-1, 22}, "core insn access to L2 that hit" },
    { {2, 59}, "core insn access to L2 that miss" },
    { {-1, 23}, "core data access to L2 that hit" },
    { {4, 57}, "core data access to L2 that miss" },
    { {5, 51}, "non-core burst write to L2" },
    { {6, 56}, "non-core non-burst write to L2" },
    { {7, 52}, "non-core write misses cache external write window and SRAM" },
    { {-1, 24}, "non-core read hit L2" },
    { {1, 54}, "non-core read miss L2" },
    { {-1, 25}, "L2 allocates, from any source" },
    { {2, 60}, "L2 retires due to full write queue" },
    { {3, 58}, "L2 retries due to address collision" },
    { {4, 48}, "L2 failed lock attempts due to full set" },
    { {5, 52}, "L2 victimizations of valid lines" },
    { {6, 57}, "L2 invalidations of lines" },
    { {7, 53}, "L2 clearing of locks" },
};

#define PERF_DRAM_READ				0
#define PERF_DRAM_WRITE				1
#define PERF_RW_CORE				2
#define PERF_RW_ETSEC				3
#define PERF_RW_PCI				4
#define PERF_RW_PCIX				5
#define PERF_RW_DMA				6
#define PERF_RW_SECURITY			7
#define PERF_INSN_L2_HIT			8
#define PERF_INSN_L2_MISS			9
#define PERF_DATA_L2_HIT			10
#define PERF_DATA_L2_MISS			11
#define PERF_NON_CORE_BURST_WRITE_L2		12
#define PERF_NON_CORE_NON_BURST_WRITE_L2	13
#define PERF_NON_CORE_WRITE_MISS		14
#define PERF_NON_CORE_READ_HIT_L2		15
#define PERF_NON_CORE_READ_MISS_L2		16
#define PERF_L2_ALLOCATES			17
#define PERF_L2_RETIRES_FULL_WRITE_QUEUE	18
#define PERF_L2_RETIRES_ADDRESS_COLLISION	19
#define PERF_L2_FAILED_LOCK_FULL_SET		20
#define PERF_L2_VICTIMIZATIONS			21
#define PERF_L2_INVALIDATIONS			22
#define PERF_CLEARING_OF_LOCKS			23

#define PERF_ETSEC_COUNT 4
*/



#define Q(x) (x - 64)
static struct perf_cpu_counter perf_cpu_counters[] = {
    // DDR memory controller events
    { {-1, 13}, "reads or writes from core 0-1" },
    { {3, Q(65)}, "reads or writes from eTSEC 1-3" },
    { {4, Q(67)}, "reads or writes from PCI Express 1-3 / Serial RapidIO 1-2" },
    { {5, Q(66)}, "reads or writes from DMA 1-2" },
    { {6, Q(69)}, "reads or writes from Security" },
    { {2, Q(65)}, "row open table misses" },
    { {3, Q(64)}, "row open table hits" },

    // e500 Coherency Module (ECM) Events
    { {8, Q(77)}, "cycles core 0 request occurs" },
    { {11, Q(98)}, "cycles core 1 request occurs" },
    { {7, Q(77)}, "cycles I2C/Security/Test Port request occurs" },
    { {5, Q(80)}, "cycles PEX1-3/DMA1-2/SRIO1-2 request occurs" },
    { {6, Q(80)}, "cycles eTSEC1-3 request occurs" },
    { {4, Q(84)}, "cycles SAP/USB/eSDHC request occurs" },
    { {-1, 15}, "ECM dispatch" },
    { {1, Q(80)}, "ECM dispatch from core 0" },
    { {10, Q(95)}, "ECM dispatch from core 1" },
    { {4, Q(85)}, "ECM dispatch from eTSEC 1" },
    { {8, Q(80)}, "ECM dispatch from eTSEC 2" },
    { {9, Q(80)}, "ECM dispatch from eTSEC 3" },

    // L2 Cache/SRAM Events
    { {2, Q(123)}, "core insn access to L2 that hit" },
    { {-1, 23}, "core insn access to L2 that miss" },
    { {4, Q(121)}, "core data access to L2 that hit" },
    { {5, Q(115)}, "core data access to L2 that miss" },
    { {6, Q(120)}, "non-core burst write to L2" },
    { {7, Q(116)}, "non-core non-burst write to L2" },
    { {-1, 24}, "non-core write misses cache external write window and SRAM" },
    { {1, Q(118)}, "non-core read hit L2" },
    { {-1, 25}, "non-core read miss L2" },
    { {2, Q(124)}, "L2 allocates, from any source" },
    { {3, Q(122)}, "L2 retires due to full write queue" },
    { {4, Q(122)}, "L2 retries due to address collision" },
    { {5, Q(116)}, "L2 failed lock attempts due to full set" },
    { {6, Q(121)}, "L2 victimizations of valid lines" },
    { {7, Q(117)}, "L2 invalidations of lines" },
    { {-1, 22}, "L2 clearing of locks" },
};
#undef Q

#define PERF_RW_CORES				0
#define PERF_RW_ETSEC				1
#define PERF_RW_PCIX				2
#define PERF_RW_DMA				3
#define PERF_RW_SECURITY			4
#define PERF_DDR_ROW_OPEN_MISS			5
#define PERF_DDR_ROW_OPEN_HIT			6

#define PERF_ECM_CORE0_REQUEST			7
#define PERF_ECM_CORE1_REQUEST			8
#define PERF_ECM_I2C_REQUEST			9
#define PERF_ECM_PEX_REQUEST			10
#define PERF_ECM_ETSEC_REQUEST			11
#define PERF_ECM_SAP_REQUEST			12
#define PERF_ECM_DISPATCH			13
#define PERF_ECM_DISPATCH_CORE0			14
#define PERF_ECM_DISPATCH_CORE1			15
#define PERF_ECM_DISPATCH_ETSEC1		16
#define PERF_ECM_DISPATCH_ETSEC2		17
#define PERF_ECM_DISPATCH_ETSEC3		18

#define PERF_INSN_L2_HIT			19
#define PERF_INSN_L2_MISS			20
#define PERF_DATA_L2_HIT			21
#define PERF_DATA_L2_MISS			22
#define PERF_NON_CORE_BURST_WRITE_L2		23
#define PERF_NON_CORE_NON_BURST_WRITE_L2	24
#define PERF_NON_CORE_WRITE_MISS		25
#define PERF_NON_CORE_READ_HIT_L2		26
#define PERF_NON_CORE_READ_MISS_L2		27
#define PERF_L2_ALLOCATES			28
#define PERF_L2_RETIRES_FULL_WRITE_QUEUE	29
#define PERF_L2_RETIRES_ADDRESS_COLLISION	30
#define PERF_L2_FAILED_LOCK_FULL_SET		31
#define PERF_L2_VICTIMIZATIONS			32
#define PERF_L2_INVALIDATIONS			33
#define PERF_CLEARING_OF_LOCKS			34

#define PERF_ETSEC_COUNT 3


static struct perf_cpu_eth_counter perf_cpu_eth_counters[] = {
    { { {3, 45}, {5, 43}, {7, 44}, {9, 27} }, "dma write data beats" },
    { { {4, 46}, {6, 46}, {8, 44}, {1, 40} }, "dma read data beats" },
    { { {5, 42}, {7, 43}, {9, 26}, {2, 48} }, "dma write request" },
    { { {6, 45}, {8, 43}, {1, 39}, {3, 48} }, "dma read request" },
    { { {9, 24}, {2, 46}, {4, 48}, {6, 40} }, "dropped frames" },
    { { {-1, 34}, {-1, 35}, {-1, 36}, {-1, 37} }, "txbd read lifetime" },
    { { {-1, 38}, {-1, 39}, {-1, 40}, {-1, 41} }, "rxbd read lifetime" },
    { { {-1, 42}, {-1, 43}, {-1, 44}, {-1, 45} }, "txbd write lifetime" },
    { { {-1, 46}, {-1, 47}, {-1, 48}, {-1, 49} }, "rxbd write lifetime" },
    { { {-1, 50}, {-1, 51}, {-1, 52}, {-1, 53} }, "read data lifetime" },
    { { {9, 28}, {3, 51}, {6, 52}, {9, 31} }, "rx ip packets csum" },
    { { {1, 41}, {4, 52}, {7, 48}, {1, 43} }, "tx ip packets csum" },
    { { {2, 49}, {5, 37}, {8, 48}, {2, 51} }, "tx tcp/udp packets csum" },
    { { {3, 50}, {6, 51}, {9, 30}, {3, 53} }, "rx tcp/udp packets csum" },
    { { {4, 51}, {7, 47}, {1, 42}, {4, 53} }, "rx ip/tcp/udp csum error" },
    { { {5, 47}, {8, 47}, {2, 50}, {5, 38} }, "rejected by filter" },
    { { {6, 50}, {9, 29}, {3, 52}, {6, 42} }, "rejected by filter error" },
    { { {5, 46}, {7, 49}, {9, 33}, {8, 40} }, "cycles rx fifo 1/4 full" },
    { { {6, 49}, {8, 49}, {1, 45}, {9, 34} }, "cycles rx fifo 2/4 full" },
    { { {7, 46}, {9, 32}, {2, 52}, {1, 46} }, "cycles rx fifo 3/4 full" },
    { { {8, 46}, {1, 44}, {3, 54}, {2, 53} }, "cycles rx fifo 4/4 full" },

    // P2020 only
#define Q(x) (x - 64)
    { { {9, Q(110)}, {9, Q(113)}, {9, Q(116)} }, "accepted match" },
    { { {10, Q(80)}, {10, Q(82)}, {10, Q(84)} }, "accepted station address" },
    { { {11, Q(78)}, {11, Q(80)}, {11, Q(82)} }, "accepted unicast via hash" },
    { { {6, Q(96)}, {6, Q(97)}, {6, Q(98)} }, "accepted group via hash" },
    { { {7, Q(100)}, {7, Q(101)}, {7, Q(102)} }, "accepted via exact match" },
    { { {9, Q(111)}, {9, Q(114)}, {9, Q(117)} }, "rejected at layer 2" },
    { { {8, Q(96)}, {10, Q(83)}, {8, Q(97)} }, "rx interrupts" },
    { { {9, Q(112)}, {11, Q(81)}, {9, Q(118)} }, "tx interrupts" },
    { { {10, Q(81)}, {8, Q(96)}, {10, Q(85)} }, "rx data write lifetime" },
    { { {11, Q(79)}, {9, Q(115)}, {11, Q(83)} }, "rx packets while fifo full" },
#undef Q
};

#define PERF_ETSEC_DMA_WRITE_DATA_BEATS		0
#define PERF_ETSEC_DMA_READ_DATA_BEATS		1
#define PERF_ETSEC_DMA_WRITE_REQUEST		2
#define PERF_ETSEC_DMA_READ_REQUEST		3
#define PERF_ETSEC_DROPPED_FRAMES		4
#define PERF_ETSEC_TXBD_READ_LIFETIME		5
#define PERF_ETSEC_RXBD_READ_LIFETIME		6
#define PERF_ETSEC_TXBD_WRITE_LIFETIME		7
#define PERF_ETSEC_RXBD_WRITE_LIFETIME		8
#define PERF_ETSEC_READ_DATA_LIFETIME		9
#define PERF_ETSEC_RX_IP_PACKETS_CKSUM		10
#define PERF_ETSEC_TX_IP_PACKETS_CKSUM		11
#define PERF_ETSEC_TX_TCP_UDP_PACKETS_CKSUM	12
#define PERF_ETSEC_RX_TCP_UDP_PACKETS_CKSUM	13
#define PERF_ETSEC_RX_IP_TCP_UDP_CKSUM_ERR	14
#define PERF_ETSEC_REJECTED_BY_FILTER		15
#define PERF_ETSEC_REJECTED_BY_FILTER_ERROR	16
#define PERF_ETSEC_CYCLES_RX_FIFO_1_4_FULL	17
#define PERF_ETSEC_CYCLES_RX_FIFO_2_4_FULL	18
#define PERF_ETSEC_CYCLES_RX_FIFO_3_4_FULL	19
#define PERF_ETSEC_CYCLES_RX_FIFO_4_4_FULL	20

#define PERF_ETSEC_ACCEPTED_MATCH		21
#define PERF_ETSEC_ACCEPTED_STATION_ADDRESS	22
#define PERF_ETSEC_ACCEPTED_UNICAST_VIA_HASH	23
#define PERF_ETSEC_ACCEPTED_GROUP_VIA_HASH	24
#define PERF_ETSEC_ACCEPTED_VIA_EXACT_MATCH	25
#define PERF_ETSEC_REJECTED_AT_LAYER2		26
#define PERF_ETSEC_RX_INTERRUPTS		27
#define PERF_ETSEC_TX_INTERUPTS			28
#define PERF_ETSEC_RX_DATA_WRITE_LIFETIME	29
#define PERF_ETSEC_RX_PACKETS_WHILE_FIFO_FULL	30



static inline void perf_cpu_reset(void) {
    unsigned i;
    if (!perf_regs) {
	return;
    }
    out_be32(perf_regs, 0x80000000);
    out_be32(&perf_regs[PERF_REG_A(0)], 0);
    out_be32(&perf_regs[PERF_REG_CNT(0)], 0);
    out_be32(&perf_regs[PERF_REG_CNT_UPPER(0)], 0);
    for (i = 1; i < 10; ++i) {
	out_be32(&perf_regs[PERF_REG_A(i)], 0x80000000);
	out_be32(&perf_regs[PERF_REG_CNT(i)], 0);
    }
}

static inline int perf_cpu_init(void) {
    perf_regs = ioremap(PERF_BASE, PERF_SIZE);
    if (!perf_regs) {
	printk("perf_regs: %p\n", perf_regs);
	return -EBUSY;
    }
    out_be32(perf_regs, 0x80000000);
    perf_cpu_reset();
    return 0;
}

static inline void perf_cpu_uninit(void) {
    if (perf_regs) {
	iounmap(perf_regs);
	perf_regs = NULL;
    }
}

static inline void perf_cpu_enable(void) {
    out_be32(perf_regs, 0);
}

static inline void perf_cpu_disable(void) {
    out_be32(perf_regs, 0x80000000);
}

static inline void perf_cpu_counter_setup(unsigned counter, unsigned event) {
    out_be32(&perf_regs[PERF_REG_A(counter)], 0x80000000);
    out_be32(&perf_regs[PERF_REG_CNT(counter)], 0);
    if (counter == 0) {
	out_be32(&perf_regs[PERF_REG_CNT_UPPER(counter)], 0);
    }
    out_be32(&perf_regs[PERF_REG_A(counter)], event << 16);
}

static inline unsigned perf_cpu_counter_get(unsigned counter) {
    unsigned ret = in_be32(&perf_regs[PERF_REG_CNT(counter)]);
    out_be32(&perf_regs[PERF_REG_CNT(counter)], 0);
    return ret;
}

static inline unsigned long long perf_cpu_counter_get(unsigned counter) {
    unsigned long long x;
    x = in_be32(&perf_regs[PERF_REG_CNT(counter)]);
    x |= (unsigned long long)in_be32(&perf_regs[PERF_REG_CNT_UPPER(counter)])
	<< 32;
    return x;
}

static inline unsigned perf_cpu_counter_event_get(unsigned counter) {
    return (in_be32(&perf_regs[PERF_REG_A(counter)]) >> 16) & 0xff;
}


static int perf_cpu_get_free_counter(void) {
    unsigned i;
    for (i = 1; i < 10; ++i) {
	if (!perf_cpu_counter_event_get(i)) return i;
    }
    return -1;
}

static inline void perf_cpu_setup(struct perf_cpu_counter_event *x) {
    unsigned counter;
    if (x->counter == -1) {
	counter = perf_cpu_get_free_counter();
	if (counter == -1) {
	    return;
	}
	perf_cpu_counter_setup(counter, x->event);
    }
    else {
	counter = x->counter;
	if (perf_cpu_counter_event_get(counter)) {
	    return;
	}
	perf_cpu_counter_setup(counter, x->event + 64);
    }
}

static inline void perf_cpu_gen(unsigned event) {
    perf_cpu_setup(&perf_cpu_counters[event].c);
}

static inline void perf_cpu_eth(unsigned eth_event) {
    unsigned i;
    for (i = 0; i < PERF_ETSEC_COUNT; ++i) {
	perf_cpu_setup(&perf_cpu_eth_counters[eth_event].c[i]);
    }
}

static void perf_cpu_format_event_name(char *buf,
	unsigned counter, unsigned event) {
    unsigned i;
    unsigned j;
    for (i = 0; i < ARRAY_SIZE(perf_cpu_counters); ++i) {
	unsigned e = perf_cpu_counters[i].c.event;
	if (perf_cpu_counters[i].c.counter != -1) {
	    e += 64;
	}
	if (e != event) {
	    continue;
	}
	if (perf_cpu_counters[i].c.counter == -1 ||
		perf_cpu_counters[i].c.counter == counter) {
	    strcpy(buf, perf_cpu_counters[i].name);
	    return;
	}
    }
    for (i = 0; i < ARRAY_SIZE(perf_cpu_eth_counters); ++i) {
	for (j = 0; j < PERF_ETSEC_COUNT; ++j) {
	    unsigned e = perf_cpu_eth_counters[i].c[j].event;
	    if (perf_cpu_eth_counters[i].c[j].counter != -1) {
		e += 64;
	    }
	    if (e != event) {
		continue;
	    }
	    if (perf_cpu_eth_counters[i].c[j].counter == -1 ||
		    perf_cpu_eth_counters[i].c[j].counter == counter) {
		sprintf(buf, "etsec%u %s", j + 1, perf_cpu_eth_counters[i].name);
		return;
	    }
	}
    }
    sprintf(buf, "%u.%u", counter, event);
}


static inline const char *perf_cpu_format_number(unsigned num) {
    static char buf[80];
    if (num < 1000) {
	sprintf(buf, "%u", num);
	return buf;
    }
    if (num < 1000000) {
	sprintf(buf, "%u,%03u", num / 1000, num % 1000);
	return buf;
    }
    sprintf(buf, "%u,%03u,%03u",
	    num / 1000000, (num / 1000) % 1000, num % 1000);
    return buf;
}

static inline void perf_cpu_show(void) {
    unsigned i;
    char buf[80];

    printk("0: system cycles = %s\n",
	    perf_cpu_format_number(perf_cpu_counter_get(0)));
    for (i = 1; i < 10; ++i) {
	if (perf_cpu_counter_event_get(i)) {
	    perf_cpu_format_event_name(buf, i, perf_cpu_counter_event_get(i));
	    printk("%u: %s = %s\n", i, buf,
		    perf_cpu_format_number(perf_cpu_counter_get(i)));
	}
    }
}

#endif
