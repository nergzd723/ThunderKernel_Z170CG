#ifndef _CADIZ_IOCTL_H_
#define _CADIZ_IOCTL_H_
#define MAX_CADIZ_REGS 100

struct cadiz_regs {
	int len;
	int regs[MAX_CADIZ_REGS][2];
};

#define CADIZ_IOCTL_SET_REGISTERS \
	_IOW('s', 1, struct cadiz_regs)

#define CADIZ_IOCTL_GET_REGISTERS \
	_IOWR('s', 2, struct cadiz_regs)

#endif
