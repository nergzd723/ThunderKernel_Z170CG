/*

Cadiz register settings for BOE TV070WXM-TU1 panel panel

address: 0x025C [1] ->1 to set output discontinuous mode

Cadiz register settings tool Ver. 1.10:
HSYNC   11
HBP     49
HACT    800
HFP     46
VSYNC   4
VBP     18
VACT    1280
VFP     18

LAN     4
HS bitrate      430 MHz
Rx sync mode    pulses
Tx sync mode     pulses
Tx EOT          1
TLPX            2(55.96ns)
Ths-Prepare for Data    0(99.75ns)
Ths-Prepare for Clock   0(56.48ns)
Ths-Zero for Data       25(440ns)
Ths-Zero for Clock      25(440ns)

SYSCLK          26 MHz
Modesel0        0

IPC             ON
COM Unsed Mode		Enable
ICE             ON
COM             OFF
SPC             ON
GRC             ON
PQ              STD_2
Effect Area     1
IBC:
ON/OFF		ON
PWM frequency setting1		50
PWM frequency		1364

*/

static struct cadiz_i2c_reg_val boot1_boe[] = {
{0x0830,0x00},
{0x0200,0x00},
{0x0201,0x00},
{0x0202,0x00},
{0x0203,0x00},
{0x0274,0x00},
{0x0275,0x00},
{0x0276,0x00},
{0x0277,0x00},
{0x0600,0x00},
{0x0601,0x00},
{0x0602,0x00},
{0x0603,0x00},
{0x0820,0x32},
{0x0821,0x04},
{0x0822,0x33},
{0x0823,0x33},
{0x0824,0x33},
{0x0825,0x03},
{0x0826,0x00},
{0x0827,0x00},
{0x0840,0x40},
{0x0841,0x7B},
{0x0842,0x14},
{0x0843,0x00},
{0x0900,0x06},
{0x0901,0x01},
{0x0902,0x20},
{0x0903,0x03},
{0x0904,0x00},
{0x0905,0x05},
{0x0906,0x3C},
{0x0907,0x00},
{0x0908,0x00},
{0x0909,0x00},
{0x090A,0xCE},
{0x090B,0x19},
{0x090C,0x00},
{0x090D,0x00},
{0x090E,0x00},
{0x090F,0x00},
{0x0914,0x01},
{0x0915,0x10},
{0x0916,0x14},
{0x0917,0x07},
{0x0940,0x05},
{0x0941,0x00},
{0x0942,0x06},
{0x0943,0x00},
{0x020C,0x00},
{0x020D,0x62},
{0x020E,0x00},
{0x020F,0x00},
{0x0220,0x20},
{0x0221,0x03},
{0x0222,0x00},
{0x0223,0x05},
{0x0228,0x08},
{0x0229,0x00},
{0x022A,0x00},
{0x022B,0x00},
{0x022C,0x24},
{0x022D,0x00},
{0x022E,0x00},
{0x022F,0x00},
{0x0230,0x22},
{0x0231,0x00},
{0x0232,0x00},
{0x0233,0x00},
{0x0234,0x58},
{0x0235,0x02},
{0x0236,0x00},
{0x0237,0x00},
{0x0238,0x04},
{0x0239,0x00},
{0x023A,0x00},
{0x023B,0x00},
{0x023C,0x0E},
{0x023D,0x00},
{0x023E,0x00},
{0x023F,0x00},
{0x0240,0x0E},
{0x0241,0x00},
{0x0242,0x00},
{0x0243,0x00},
{0x0244,0x25},
{0x0245,0x00},
{0x0246,0x00},
{0x0247,0x00},
{0x0258,0x01},
{0x0259,0x00},
{0x025A,0x00},
{0x025B,0x00},
{0x025C,0x07},
{0x025D,0x00},
{0x025E,0x00},
{0x025F,0x00},
{0x0260,0x0C},
{0x0261,0x00},
{0x0262,0x00},
{0x0263,0x00},
{0x0264,0x0F},
{0x0265,0x00},
{0x0266,0x26},
{0x0267,0x00},
{0x0268,0x02},
{0x0269,0x00},
{0x026A,0x00},
{0x026B,0x00},
{0x026C,0x00},
{0x026D,0x19},
{0x026E,0x04},
{0x026F,0x08},
{0x0270,0x00},
{0x0271,0x19},
{0x0272,0x04},
{0x0273,0x08},
{0x0278,0x1A},
{0x0279,0x04},
{0x027A,0x84},
{0x027B,0x3B},
{0x027C,0x07},
{0x027D,0x38},
{0x027E,0x32},
{0x027F,0xB5},
{0x0280,0x00},
{0x0281,0x00},
{0x0282,0x40},
{0x0283,0x10},
{0x0284,0x00},
{0x0285,0x40},
{0x0286,0x44},
{0x0287,0x02},
{0x02A8,0x01},
{0x02A9,0x00},
{0x02AA,0x00},
{0x02AB,0x00},
{0x000C,0x60},
{0x000D,0x04},
{0x000E,0x00},
{0x000F,0x00},
{0x0024,0x01},
{0x0025,0x00},
{0x0026,0x00},
{0x0027,0x00},
{0x002C,0x05},
{0x002D,0xFF},
{0x002E,0x00},
{0x002F,0x00},
{0x0034,0x1A},
{0x0035,0x84},
{0x0036,0x04},
{0x0037,0x3A},
{0x0038,0x07},
{0x0039,0x38},
{0x003A,0x32},
{0x003B,0xB5},
{0x003C,0x1C},
{0x003D,0x04},
{0x003E,0x00},
{0x003F,0x00},
{0x0040,0x00},
{0x0041,0x00},
{0x0042,0x44},
{0x0043,0x02},
{0x0058,0x0B},
{0x0059,0x00},
{0x005A,0x04},
{0x005B,0x00},
{0x090C,0x00},
{0x090D,0x00},
{0x090E,0x34},
{0x090F,0x07}
};

static struct cadiz_i2c_reg_val boot2_boe[] = {
{0x001C,0x05},
{0x001D,0x00},
{0x001E,0x00},
{0x001F,0x00},
{0x0000,0x01},
{0x0001,0x00},
{0x0002,0x00},
{0x0003,0x00},
{0x0940,0x05},
{0x0941,0x06},
{0x0942,0x06},
{0x0943,0x00}
};

static struct cadiz_i2c_reg_val boot3_boe[] = {
{0x0274,0x01},
{0x0200,0x01},
{0x0006,0xFF}
};

static struct cadiz_i2c_reg_val boot4_boe[] = {
{0x0000,0x01},
{0x0941,0x07},
{0x0941,0x01},
{0x0005,0xFF}
};

static struct cadiz_i2c_reg_val ipc_ibc_boe[] = {
{0x0C14,0x00},
{0x0C15,0x00},
{0x0C16,0x00},
{0x0C17,0x02},
{0x0C20,0x00},
{0x0C21,0x02},
{0x0C22,0x00},
{0x0C23,0x01},
{0x0C34,0x10},
{0x0C35,0x82},
{0x0C36,0x05},
{0x0C37,0x51},
{0x0C28,0x80},
{0x0C29,0x00},
{0x0C2A,0x32},
{0x0C2B,0x00},
{0x0C2C,0x54},
{0x0C2D,0x05},
{0x0C2E,0x00},
{0x0C2F,0x00},
{0x0C30,0x00},
{0x0C31,0x00},
{0x0C32,0x50},
{0x0C33,0x15},
{0x0A00,0x80},
{0x0A01,0x00},
{0x0A02,0x04},
{0x0A03,0x00},
{0x0A04,0x20},
{0x0A05,0x03},
{0x0A06,0x00},
{0x0A07,0x00},
{0x0A08,0x00},
{0x0A09,0x05},
{0x0A0A,0x00},
{0x0A0B,0x00},
{0x0A0C,0x00},
{0x0A0D,0x00},
{0x0A0E,0x00},
{0x0A0F,0x00},
{0x0A10,0x00},
{0x0A11,0x00},
{0x0A12,0x00},
{0x0A13,0x88},
{0x0A14,0x01},
{0x0A15,0x00},
{0x0A16,0x00},
{0x0A17,0x00},
{0x0A18,0x80},
{0x0A19,0x80},
{0x0A1A,0x80},
{0x0A1B,0x80},
{0x0A1C,0x00},
{0x0A1D,0x00},
{0x0A1E,0x02},
{0x0A1F,0x08},
{0x0A20,0x0A},
{0x0A21,0x06},
{0x0A22,0x1F},
{0x0A23,0x3F},
{0x0A24,0x16},
{0x0A25,0x16},
{0x0A26,0x04},
{0x0A27,0x04},
{0x0A28,0x0A},
{0x0A29,0x00},
{0x0A2A,0x04},
{0x0A2B,0x04},
{0x0A2C,0x04},
{0x0A2D,0x04},
{0x0A2E,0x02},
{0x0A2F,0x04},
{0x0A30,0x04},
{0x0A31,0x01},
{0x0A32,0x30},
{0x0A33,0xFF},
{0x0A34,0x80},
{0x0A35,0x30},
{0x0A36,0x37},
{0x0A37,0xFF},
{0x0A38,0x00},
{0x0A39,0x00},
{0x0A3A,0x68},
{0x0A3B,0x00},
{0x0A3C,0x58},
{0x0A3D,0x02},
{0x0A3E,0xA0},
{0x0A3F,0x00},
{0x0A40,0xC0},
{0x0A41,0x03},
{0x0A42,0x06},
{0x0A43,0x00},
{0x0A44,0x85},
{0x0A45,0x00},
{0x0A46,0x02},
{0x0A47,0x00},
{0x0A48,0xD5},
{0x0A49,0x00},
{0x0A4A,0x3A},
{0x0A4B,0x00},
{0x0A4C,0x93},
{0x0A4D,0x04},
{0x0A4E,0xEC},
{0x0A4F,0x01},
{0x0A50,0x33},
{0x0A51,0x01},
{0x0A52,0x78},
{0x0A53,0x68},
{0x0A54,0x2C},
{0x0A55,0x71},
{0x0A56,0x84},
{0x0A57,0x03},
{0x0A58,0x24},
{0x0A59,0x00},
{0x0A5A,0x00},
{0x0A5B,0x67},
{0x0A5C,0xC0},
{0x0A5D,0x88},
{0x0A5E,0xB6},
{0x0A5F,0xAF},
{0x0A60,0x00},
{0x0A61,0xFF},
{0x0A62,0xF0},
{0x0A63,0x88},
{0x0A64,0x22},
{0x0A65,0x3A},
{0x0A66,0xD6},
{0x0A67,0xA7},
{0x0A68,0x60},
{0x0A69,0xC5},
{0x0A6A,0xB0},
{0x0A6B,0x20},
{0x0A6C,0xCC},
{0x0A6D,0x8A},
{0x0A6E,0x8A},
{0x0A6F,0x83},
{0x0A70,0x84},
{0x0A71,0x86},
{0x0A72,0x89},
{0x0A73,0x83},
{0x0A74,0x91},
{0x0A75,0x8B},
{0x0A76,0x01},
{0x0A77,0x00},
{0x0A80,0x00},
{0x0A81,0x00},
{0x0A82,0x04},
{0x0A83,0x00},
{0x0A84,0x20},
{0x0A85,0x03},
{0x0A86,0x04},
{0x0A87,0x00},
{0x0A88,0x00},
{0x0A89,0x05},
{0x0A8A,0x60},
{0x0A8B,0x00},
{0x0A8C,0x65},
{0x0A8D,0x04},
{0x0A8E,0x42},
{0x0A8F,0x00},
{0x0A90,0x09},
{0x0A91,0x3D},
{0x0A92,0x00},
{0x0A93,0x00},
{0x0A94,0x20},
{0x0A95,0x00},
{0x0A96,0x64},
{0x0A97,0x00},
{0x0A98,0xA0},
{0x0A99,0x00},
{0x0A9A,0x00},
{0x0A9B,0x10},
{0x0A9C,0x00},
{0x0A9D,0x00},
{0x0A9E,0x01},
{0x0A9F,0x00},
{0x0AA0,0xF5},
{0x0AA1,0x04},
{0x0AA2,0x08},
{0x0AA3,0x00},
{0x0AA8,0x00},
{0x0AA9,0x00},
{0x0AAA,0x78},
{0x0AAB,0x00},
{0x0AAC,0x30},
{0x0AAD,0x02},
{0x0AAE,0x26},
{0x0AAF,0x00},
{0x0AB0,0x80},
{0x0AB1,0x04},
{0x0AB2,0x00},
{0x0AB3,0x00},
{0x0AB4,0x00},
{0x0AB5,0x2A},
{0x0AB6,0x00},
{0x0AB7,0x00},
{0x0AB8,0x21},
{0x0AB9,0x3F},
{0x0ABA,0x00},
{0x0ABB,0x01},
{0x0ABC,0x0F},
{0x0ABD,0x80},
{0x0ABE,0xB8},
{0x0ABF,0x1F},
{0x0AC0,0xC0},
{0x0AC1,0x64},
{0x0AC2,0x02},
{0x0AC3,0x80},
{0x0AC4,0x26},
{0x0AC5,0x00},
{0x0AC6,0x88},
{0x0AC7,0x20},
{0x0AC8,0x00},
{0x0AC9,0x40},
{0x0ACA,0x00},
{0x0ACB,0x3F},
{0x0ACC,0x00},
{0x0ACD,0x43},
{0x0ACE,0x00},
{0x0ACF,0x4A},
{0x0AD0,0x20},
{0x0AD1,0x33},
{0x0AD2,0x7F},
{0x0AD3,0x96},
{0x0AD4,0x06},
{0x0AD5,0x0F},
{0x0AD6,0x21},
{0x0AD7,0x28},
{0x0AD8,0x2C},
{0x0AD9,0x7F},
{0x0ADA,0xC8},
{0x0ADB,0x96},
{0x0ADC,0x0F},
{0x0ADD,0x19},
{0x0ADE,0x28},
{0x0ADF,0x2C},
{0x0AE0,0x00},
{0x0AE1,0x00},
{0x0AE2,0x00},
{0x0AE3,0x00},
{0x0AFC,0x00},
{0x0AFD,0x00},
{0x0AFE,0x00},
{0x0AFF,0x08}
};