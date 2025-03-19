#ifndef ALT_H
#define ALT_H

#include "buffer_store.h"
#include "stack.h"


#define ETH_FRAME_OVERHEAD 24
#define NSEC_PER_HZ (NSEC_PER_SEC / HZ)
#define RX_BUF_SIZE 1600
#define DEFAULT_DESC_COUNT 16
#define DEFAULT_SIZE 60

#define MAX_ALT_DEVS 4
#define MAX_STREAMS 4

#define ALT_MAGIC 0x12345678
#define ALT_HEADER_OFFSET 40

#define BITS_PER_SEC 1000000000
#define CYCLES_PER_SEC 1333333333
#define CYCLES_PER_HZ (CYCLES_PER_SEC / HZ)

#define LATENCY_ITEMS 64
#define LATENCY_SHIFT 10

#define MAX_HEADER_LEN 40
#define MAX_REPORT_FRAME 1500
#define MAX_SETUP_FRAME 500

#define ALT_CMD_START 1
#define ALT_CMD_STOP 2
#define ALT_CMD_SETUP_PACKET 3

//#define CONFIG_REPORT
#define CONFIG_MEASURE_LATENCY

#define PACKET_ID_STATS 1
#define PACKET_ID_REPORT 2


struct test_dev_parameters {
    unsigned tx_desc_count;
    unsigned rx_desc_count;
};

struct stream_parameters {
    int tx_dev_num;

    unsigned packet_size;
    unsigned packet_size_min;
    unsigned packet_size_max;

    int packets_per_second;
    int mbits_per_second;

    unsigned fill_byte;
    unsigned fill_inc;
    unsigned header_len;
    unsigned char header[MAX_HEADER_LEN];
};

struct test_parameters {
    unsigned duration;
    struct test_dev_parameters devs[MAX_ALT_DEVS];
    struct stream_parameters streams[MAX_STREAMS];
};

struct start_test_frame {
    unsigned magic;
    unsigned cmd;
    struct test_parameters params;
};

struct setup_packet_frame {
    unsigned magic;
    unsigned cmd;
    unsigned tx_dev_num;
    unsigned len;
    unsigned char data[0];
};

struct alt_header {
    unsigned magic;
    unsigned stream;
    unsigned long long seq;
    unsigned tstamp;
};

enum TestStatus {
    TEST_OPEN = 0,
    TEST_OPENED,
    TEST_WAIT_FOR_LINK,
    TEST_WAIT_MORE,
    TEST_MANAGED,
    TEST_IN_PROGRESS,
    TEST_FINISH,
    TEST_DONE,
};

struct alt_tx_stream {
    struct stream_parameters params;
    unsigned num;
    unsigned long long tx_seq;
    int target_bitrate;
    int cycles_per_packet;
    unsigned long next_tx_time;

    struct buffer_store bufs;
};

struct alt_rx_stream {
    unsigned long long rx_seq;
};

struct alt_stream_stats {
    unsigned tx_packet;
    unsigned tx_byte;

    unsigned rx_packet;
    unsigned rx_byte;
    unsigned rx_out_of_order_packet;

#ifdef CONFIG_MEASURE_LATENCY
    unsigned latency_min;
    unsigned latency_max;
    unsigned pad;
    unsigned long long latency_total;
    unsigned latency_distribution[LATENCY_ITEMS];
#endif
};

struct alt_stats {
    unsigned packet_id;
    unsigned buf;
    unsigned num;
    unsigned seq;

    struct alt_stream_stats stream[MAX_STREAMS];
    unsigned tx_error;
    unsigned tx_late_collision;
    unsigned tx_retry_limit;
    unsigned tx_underrun;

    unsigned rx_unknown_packet;
    unsigned rx_unknown_byte;
    unsigned rx_error;
    unsigned rx_large;
    unsigned rx_non_octet;
    unsigned rx_short;
    unsigned rx_crc_error;
    unsigned rx_overrun;
    unsigned rx_truncated;
};

struct alt_report {
    unsigned packet_id;
    unsigned num;
    unsigned size;
    unsigned char report[MAX_REPORT_FRAME];
};

struct alt;
struct alt_dev {
    struct list_head list;
    struct list_head test_list;
    unsigned num;
    struct gfar_mac *mac;
    struct alt *alt;

    struct gfar __iomem *regs;
    struct desc *tx_desc;
    struct desc *rx_desc;
    struct test_dev_parameters params;
    unsigned tx_dirty;
    unsigned rx_cur;

    struct buffer_store rx_bufs1;
    struct buffer_store rx_bufs2;
    struct buffer_store *rx_front;
    struct buffer_store *rx_back;

    unsigned valid_tx_streams;
    struct alt_tx_stream tx_stream[MAX_STREAMS];
    struct alt_rx_stream rx_stream[MAX_STREAMS];

    unsigned stats1_dma_addr;
    unsigned stats2_dma_addr;
    struct alt_stats stats1;
    struct alt_stats stats2;
    struct alt_stats *stats;

#ifdef CONFIG_REPORT
    unsigned report1_dma_addr;
    unsigned report2_dma_addr;
    struct alt_report report1;
    struct alt_report report2;
    struct alt_report *report;
#endif
    unsigned setup_packet_dma_addr;
    unsigned char setup_packet[MAX_SETUP_FRAME];
};

#define MAX_REGULAR_DEVS 16
struct alt_regular_dev {
    struct net_device *dev;
    unsigned was_open;
};

struct alt {
    struct list_head dev_head;
    struct list_head test_dev_head;
    struct alt_dev *manag_dev;
    struct alt_regular_dev regular_devs[MAX_REGULAR_DEVS];
    unsigned regular_dev_count;

    struct timer_list timer;
    struct work_struct open_task;
    struct work_struct close_task;
    struct work_struct open_regular_task;
    struct work_struct close_regular_task;
    unsigned long last_status_change;
    unsigned status;
    unsigned duration;
};
struct alt alt;


#endif
