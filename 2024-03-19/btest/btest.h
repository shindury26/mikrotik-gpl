#ifndef _LINUX_BTEST_H_
#define _LINUX_BTEST_H_

#include <linux/ioctl.h>


#define BTEST_SEND 0x01
#define BTEST_RECEIVE 0x02

struct btest_params {
    int fd;
    unsigned flags;
    unsigned send_size_from;
    unsigned send_size_to;
    unsigned long long send_rate;
    unsigned send_random;
    unsigned padding;
};

struct btest_info {
    unsigned long long received;
    unsigned lost;
    unsigned outoforder;
    unsigned duplicate;
    unsigned padding;
};

#define BTEST_IOCTL_START _IOW('x', 1, struct btest_params)
#define BTEST_IOCTL_GET _IOR('x', 2, struct btest_info)
#define BTEST_IOCTL_ADJUST _IOW('x', 3, unsigned long long)

#endif
