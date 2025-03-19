#ifndef PERF_CORE_H
#define PERF_CORE_H

#include "asm/reg_fsl_emb.h"


static inline unsigned get_pmlca(unsigned counter) {
    unsigned pmlca;
    switch (counter) {
    case 0:
	pmlca = mfpmr(PMRN_PMLCA0);
	break;
    case 1:
	pmlca = mfpmr(PMRN_PMLCA1);
	break;
    case 2:
	pmlca = mfpmr(PMRN_PMLCA2);
	break;
    case 3:
	pmlca = mfpmr(PMRN_PMLCA3);
	break;
    default:
	panic("Bad counter\n");
    }
    return pmlca;
}

static inline void set_pmlca(unsigned counter, unsigned pmlca) {
    switch (counter) {
    case 0:
	mtpmr(PMRN_PMLCA0, pmlca);
	break;
    case 1:
	mtpmr(PMRN_PMLCA1, pmlca);
	break;
    case 2:
	mtpmr(PMRN_PMLCA2, pmlca);
	break;
    case 3:
	mtpmr(PMRN_PMLCA3, pmlca);
	break;
    default:
	panic("Bad counter\n");
    }
}

static inline unsigned get_pmlcb(unsigned counter) {
    unsigned pmlcb;
    switch (counter) {
    case 0:
	pmlcb = mfpmr(PMRN_PMLCB0);
	break;
    case 1:
	pmlcb = mfpmr(PMRN_PMLCB1);
	break;
    case 2:
	pmlcb = mfpmr(PMRN_PMLCB2);
	break;
    case 3:
	pmlcb = mfpmr(PMRN_PMLCB3);
	break;
    default:
	panic("Bad counter\n");
    }
    return pmlcb;
}

static inline void set_pmlcb(unsigned counter, unsigned pmlcb) {
    switch (counter) {
    case 0:
	mtpmr(PMRN_PMLCB0, pmlcb);
	break;
    case 1:
	mtpmr(PMRN_PMLCB1, pmlcb);
	break;
    case 2:
	mtpmr(PMRN_PMLCB2, pmlcb);
	break;
    case 3:
	mtpmr(PMRN_PMLCB3, pmlcb);
	break;
    default:
	panic("Bad counter\n");
    }
}

static inline unsigned perf_core_counter_get(unsigned counter) {
    switch (counter) {
    case 0:
	return mfpmr(PMRN_PMC0);
    case 1:
	return mfpmr(PMRN_PMC1);
    case 2:
	return mfpmr(PMRN_PMC2);
    case 3:
	return mfpmr(PMRN_PMC3);
    default:
	return 0;
    }
}

static inline void perf_core_counter_clear(unsigned counter) {
    switch (counter) {
    case 0:
	mtpmr(PMRN_PMC0, 0);
	break;
    case 1:
	mtpmr(PMRN_PMC1, 0);
	break;
    case 2:
	mtpmr(PMRN_PMC2, 0);
	break;
    case 3:
	mtpmr(PMRN_PMC3, 0);
	break;
    default:
	break;
    }
}

#define PERF_CYCLES			1
#define PERF_INSNS_COMPLETED		2
#define PERF_MICRO_OPS_COMPLETED	3
#define PERF_INSNS_FETCHED		4
#define PERF_MICRO_OPS_DECODED		5
#define PERF_PM_EVENT_TRANSITIONS	6
#define PERF_PM_EVENT_CYCLES		7
#define PERF_BRANCH_INSNS_COMPLETED	8
#define PERF_LOAD_MICRO_OPS_COMPLETED	9
#define PERF_STORE_MICRO_OPS_COMPLETED	10
#define PERF_CQ_REDIRECTS		11
#define PERF_BRANCHES_FINISHED		12
#define PERF_TAKEN_BRANCHES_FINISHED	13
#define PERF_BRANCHES_MISPREDICTED	15

// pipline stalls
#define PERF_DECODE_STALLED_CYCLES	18
#define PERF_ISSUE_STALLED_CYCLES	19
#define PERF_BRANCH_STALLED_CYCLES	20
#define PERF_SU1_SCHED_STALLED_CYCLES	21
#define PERF_SU2_SCHED_STALLED_CYCLES	22
#define PERF_MU_SCHED_STALLED_CYCLES	23
#define PERF_LSU_SCHED_STALLED_CYCLES	24
#define PERF_BU_SCHED_STALLED_CYCLES	25

// load/store/dlfb events
#define PERF_TOTAL_TRANSLATED		26
#define PERF_LOADS_TRANSLATED		27
#define PERF_STORES_TRANSLATED		28
#define PERF_TOUCHES_TRANSLATED		29
#define PERF_TOTAL_ALLOC		35
#define PERF_LOADS_TRANS_AND_ALLOC	36
#define PERF_STORES_TRANS_AND_ALLOC	37
#define PERF_TOUCHES_TRANS_AND_ALLOC	38
#define PERF_STORES_COMPLETED		39
#define PERF_DATA_L1_CACHE_LOCKS	40
#define PERF_DATA_L1_CACHE_RELOADS	41

// data side replay conditions
#define PERF_LOAD_MISS_DLBF_FULL	43
#define PERF_LOAD_MISS_QUEUE_FULL	44
#define PERF_LOAD_GUARDED_MISS		45
#define PERF_TRANS_STORE_QUEUE_FULL	46
#define PERF_ADDRESS_COLLISION		47
#define PERF_DATA_MMU_MISS		48
#define PERF_DATA_MMU_BUSY		49
#define PERF_LOAD_MISS_DLBF_FULL_CYCLES	51
#define PERF_LOAD_MISS_QUEUE_FULL_CYCLES	52
#define PERF_LOAD_GUARDED_MISS_CYCLES	53
#define PERF_TRANS_STORE_QUEUE_FULL_CYCLES	54
#define PERF_ADDRESS_COLLISION_CYCLES	55
#define PERF_DATA_MMU_MISS_CYCLES	56
#define PERF_DATA_MMU_BUSY_CYCLES	57
#define PERF_INSN_L1_CACHE_LOCKS	59
#define PERF_INSN_L1_CACHE_RELOADS	60

#define PERF_INSN_MMU_TLB4K_RELOADS	62
#define PERF_INSN_MMU_VSP_RELOADS	63
#define PERF_DATA_MMU_TLB4K_RELOADS	64
#define PERF_DATA_MMU_VSP_RELOADS	65

#define PERF_L2_MMU_MISS		66

#define PERF_SNOOP_REQUESTS		72
#define PERF_SNOOP_HITS			73
#define PERF_SNOOP_PUSHES		74
#define PERF_SNOOP_RETRIES		75

#define PERF_INTERRUPTS			86
#define PERF_EXTERNAL_INTERRUPTS	87
#define PERF_CRITICAL_INTERRUPTS	88
#define PERF_SYSTEM_INTERRUPTS		89

static const char *perf_core_get_event_name(unsigned event) {
    switch (event) {
    case PERF_CYCLES: return "processor cycles";
    case PERF_INSNS_COMPLETED: return "instructions completed";
    case PERF_MICRO_OPS_COMPLETED: return "micro-ops completed";
    case PERF_INSNS_FETCHED: return "instructions fetched";
    case PERF_MICRO_OPS_DECODED: return "micro-ops decoded";
    case PERF_PM_EVENT_TRANSITIONS: return "PM_EVENT transitions";
    case PERF_PM_EVENT_CYCLES: return "PM_EVENT cycles";
    case PERF_BRANCH_INSNS_COMPLETED: return "branch instructions completed";
    case PERF_LOAD_MICRO_OPS_COMPLETED: return "load micro-ops completed";
    case PERF_STORE_MICRO_OPS_COMPLETED: return " store micro-ops completed";
    case PERF_CQ_REDIRECTS: return "CQ redirects";
    case PERF_BRANCHES_FINISHED: return "branches finished";
    case PERF_TAKEN_BRANCHES_FINISHED: return "taken branches finished";
    case PERF_BRANCHES_MISPREDICTED: return "branches mispredicted";

    case PERF_DECODE_STALLED_CYCLES: return "decode stalled cycles";
    case PERF_ISSUE_STALLED_CYCLES: return "issue stalled cycles";
    case PERF_BRANCH_STALLED_CYCLES: return "branch issue stalled cycles";
    case PERF_SU1_SCHED_STALLED_CYCLES: return "SU1 stalled cycles";
    case PERF_SU2_SCHED_STALLED_CYCLES: return "SU2 stalled cycles";
    case PERF_MU_SCHED_STALLED_CYCLES: return "MU stalled cycles";
    case PERF_LSU_SCHED_STALLED_CYCLES: return "LSU stalled cycles";
    case PERF_BU_SCHED_STALLED_CYCLES: return "BU stalled cycles";

    case PERF_TOTAL_TRANSLATED: return "total translated";
    case PERF_LOADS_TRANSLATED: return "loads translated";
    case PERF_STORES_TRANSLATED: return "stores translated";
    case PERF_TOUCHES_TRANSLATED: return "touches translated";
    case PERF_TOTAL_ALLOC: return "total allocated";
    case PERF_LOADS_TRANS_AND_ALLOC: return "loads translated and allocated to DLFB";
    case PERF_STORES_TRANS_AND_ALLOC: return "stores completed and allocated to DLFB";
    case PERF_TOUCHES_TRANS_AND_ALLOC: return "touches translated and allocated to DLFB";
    case PERF_STORES_COMPLETED: return "stores completed";
    case PERF_DATA_L1_CACHE_LOCKS: return "data L1 cache locks";
    case PERF_DATA_L1_CACHE_RELOADS: return "data L1 cache reloads";

    case PERF_LOAD_MISS_DLBF_FULL: return "load miss with DLBF full";
    case PERF_LOAD_MISS_QUEUE_FULL: return "load miss with queue full";
    case PERF_LOAD_GUARDED_MISS: return "load guarded miss when load is not yet at bootom of the CQ";
    case PERF_TRANS_STORE_QUEUE_FULL: return "translate store when queue is full";
    case PERF_ADDRESS_COLLISION: return "address collision";
    case PERF_DATA_MMU_MISS: return "data MMU miss";
    case PERF_DATA_MMU_BUSY: return "data MMU busy";
    case PERF_LOAD_MISS_DLBF_FULL_CYCLES: return "load miss with DLBF full cycles";
    case PERF_LOAD_MISS_QUEUE_FULL_CYCLES: return "load miss with queue full cycles";
    case PERF_LOAD_GUARDED_MISS_CYCLES: return "load guarded miss when load is not yet at bootom of the CQ cycles cycles";
    case PERF_TRANS_STORE_QUEUE_FULL_CYCLES: return "translate store when queue is full cycles";
    case PERF_ADDRESS_COLLISION_CYCLES: return "address collision cycles";
    case PERF_DATA_MMU_MISS_CYCLES: return "data MMU miss cycles";
    case PERF_DATA_MMU_BUSY_CYCLES: return "data MMU busy cycles";
    case PERF_INSN_L1_CACHE_LOCKS: return "instruction L1 cache locks";
    case PERF_INSN_L1_CACHE_RELOADS: return "instruction L1 cache reloads";

    case PERF_INSN_MMU_TLB4K_RELOADS: return "instruction MMU TLB4K reloads";
    case PERF_INSN_MMU_VSP_RELOADS: return "instruction MMU VSP reloads";
    case PERF_DATA_MMU_TLB4K_RELOADS: return "data MMU TLB4K reloads";
    case PERF_DATA_MMU_VSP_RELOADS: return "data MMU VSP reloads";

    case PERF_L2_MMU_MISS: return "L2 MMU miss";

    case PERF_SNOOP_REQUESTS: return "snoop requests";
    case PERF_SNOOP_HITS: return "snoop hits";
    case PERF_SNOOP_PUSHES: return "snoop pushes";
    case PERF_SNOOP_RETRIES: return "snoop retries";

    case PERF_INTERRUPTS: return "interrupts";
    case PERF_EXTERNAL_INTERRUPTS: return "external input interrupts";
    case PERF_CRITICAL_INTERRUPTS: return "critical input interrupts";
    case PERF_SYSTEM_INTERRUPTS: return "system call and trap interrupts";
    }
    return NULL;
}

static void perf_core_format_event_name(char *buf, unsigned event) {
    const char *x = perf_core_get_event_name(event);
    if (x) {
	strcpy(buf, x);
	return;
    }
    sprintf(buf, "event #%u", event);
}

static inline void perf_core_counter_setup(unsigned counter, unsigned event) {
    unsigned pmlca = get_pmlca(counter);
    pmlca &= ~PMLCA_FC;
    pmlca &= ~PMLCA_FCU;
    pmlca &= ~PMLCA_FCS;
    pmlca &= ~PMLCA_FCM1;
    pmlca &= ~PMLCA_FCM0;
    pmlca &= ~PMLCA_CE;
    pmlca = (pmlca & ~PMLCA_EVENT_MASK) |
	((event << PMLCA_EVENT_SHIFT) &
		PMLCA_EVENT_MASK);
    set_pmlca(counter, pmlca);
    perf_core_counter_clear(counter);
//    printk("pmlca: %08x\n", get_pmlca(counter));
}

static inline void perf_core_enable(void) {
    unsigned pmgc0 = mfpmr(PMRN_PMGC0);
    pmgc0 &= ~PMGC0_FAC;
    pmgc0 &= ~PMGC0_FCECE;
    pmgc0 &= ~PMGC0_PMIE;
    mtpmr(PMRN_PMGC0, pmgc0);
//    printk("pmgc0: %08x\n", mfpmr(PMRN_PMGC0));
}

static inline void perf_core_disable(void) {
    mtpmr(PMRN_PMGC0, PMGC0_FAC);
}


static inline void perf_core_start(unsigned e1, unsigned e2, unsigned e3,
	unsigned e4) {
    perf_core_counter_setup(0, e1);
    perf_core_counter_setup(1, e2);
    perf_core_counter_setup(2, e3);
    perf_core_counter_setup(3, e4);
}

static inline void perf_core_cycles(void) {
    perf_core_start(
	PERF_CYCLES,
	PERF_INSNS_COMPLETED,
	PERF_MICRO_OPS_COMPLETED,
	PERF_INSNS_FETCHED);
}

static inline void perf_core_interrupts(void) {
    perf_core_start(
	PERF_INTERRUPTS,
	PERF_EXTERNAL_INTERRUPTS,
	PERF_CRITICAL_INTERRUPTS,
	PERF_SYSTEM_INTERRUPTS);
}

static inline void perf_core_tlb_reloads(void) {
    perf_core_start(
	PERF_INSN_MMU_TLB4K_RELOADS,
	PERF_INSN_MMU_VSP_RELOADS,
	PERF_DATA_MMU_TLB4K_RELOADS,
	PERF_DATA_MMU_VSP_RELOADS);
}

static inline void perf_core_branches(void) {
    perf_core_start(
	PERF_BRANCH_INSNS_COMPLETED,
	PERF_BRANCHES_FINISHED,
	PERF_TAKEN_BRANCHES_FINISHED,
	PERF_BRANCHES_MISPREDICTED);
}

static inline void perf_core_l1_cache(void) {
    perf_core_start(
	PERF_DATA_L1_CACHE_LOCKS,
	PERF_DATA_L1_CACHE_RELOADS,
	PERF_INSN_L1_CACHE_LOCKS,
	PERF_INSN_L1_CACHE_RELOADS);
}


static inline const char *perf_core_format_number(unsigned num) {
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

static inline void perf_core_show(void) {
    unsigned i;
    for (i = 0; i < 4; ++i) {
	unsigned x = (get_pmlca(i) & PMLCA_EVENT_MASK) >> PMLCA_EVENT_SHIFT;
	char event_name[80];
	if (!x) {
	    continue;
	}
	perf_core_format_event_name(event_name, x);
	printk("@%u counter %d: %s = %s\n",
		smp_processor_id(), i, event_name,
		perf_core_format_number(perf_core_counter_get(i)));
    }
}


#endif
