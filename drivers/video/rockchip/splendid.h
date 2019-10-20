typedef struct splendid_gamma {
	int lut[256];
} splendid_gamma_settings;

typedef struct splendid_bcsh {
	char cmd[20];
} splendid_bcsh_settings;

#define SPLENDID_IOCTL_SET_GAMMA _IOW('s', 1, splendid_gamma_settings)
#define SPLENDID_IOCTL_SET_BCSH _IOW('s', 2, splendid_bcsh_settings)