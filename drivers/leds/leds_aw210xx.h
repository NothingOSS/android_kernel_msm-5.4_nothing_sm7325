#ifndef __LEDS_AW210XX_H__
#define __LEDS_AW210XX_H__

#define AW210XX_I2C_NAME "aw210xx_led"

#define AW_DEBUG 							(0)

#if AW_DEBUG
#define AW_LOG(fmt, args...)	pr_debug("[%s] %s %d: " fmt, AW210XX_I2C_NAME, \
		__func__, __LINE__, ##args)
#else
#define AW_LOG(fmt, args...)
#endif

#define AW_INFO(fmt, args...)	pr_info("[%s] %s %d: " fmt, AW210XX_I2C_NAME, \
		__func__, __LINE__, ##args)
#define AW_ERR(fmt, args...)	pr_err("[%s] %s %d: " fmt, AW210XX_I2C_NAME, \
		__func__, __LINE__, ##args)


/*****************************************************
* register about led mode
*****************************************************/
#define AW210XX_REG_GCR					(0x20)
#define AW210XX_REG_BR00L					(0x21)
#define AW210XX_REG_BR00H					(0x22)
#define AW210XX_REG_BR01L					(0x23)
#define AW210XX_REG_BR01H					(0x24)
#define AW210XX_REG_BR02L					(0x25)
#define AW210XX_REG_BR02H					(0x26)
#define AW210XX_REG_BR03L					(0x27)
#define AW210XX_REG_BR03H					(0x28)
#define AW210XX_REG_BR04L					(0x29)
#define AW210XX_REG_BR04H					(0x2A)
#define AW210XX_REG_BR05L					(0x2B)
#define AW210XX_REG_BR05H					(0x2C)
#define AW210XX_REG_BR06L					(0x2D)
#define AW210XX_REG_BR06H					(0x2E)
#define AW210XX_REG_BR07L					(0x2F)
#define AW210XX_REG_BR07H					(0x30)
#define AW210XX_REG_BR08L					(0x31)
#define AW210XX_REG_BR08H					(0x32)
#define AW210XX_REG_BR09L					(0x33)
#define AW210XX_REG_BR09H					(0x34)
#define AW210XX_REG_BR10L					(0x35)
#define AW210XX_REG_BR10H					(0x36)
#define AW210XX_REG_BR11L					(0x37)
#define AW210XX_REG_BR11H					(0x38)
#define AW210XX_REG_BR12L					(0x39)
#define AW210XX_REG_BR12H					(0x3A)
#define AW210XX_REG_BR13L					(0x3B)
#define AW210XX_REG_BR13H					(0x3C)
#define AW210XX_REG_BR14L					(0x3D)
#define AW210XX_REG_BR14H					(0x3E)
#define AW210XX_REG_BR15L					(0x3F)
#define AW210XX_REG_BR15H					(0x40)
#define AW210XX_REG_BR16L					(0x41)
#define AW210XX_REG_BR16H					(0x42)
#define AW210XX_REG_BR17L					(0x43)
#define AW210XX_REG_BR17H					(0x44)
#define AW210XX_REG_UPDATE					(0x45)
#define AW210XX_REG_SL00					(0x46)
#define AW210XX_REG_SL01					(0x47)
#define AW210XX_REG_SL02					(0x48)
#define AW210XX_REG_SL03					(0x49)
#define AW210XX_REG_SL04					(0x4A)
#define AW210XX_REG_SL05					(0x4B)
#define AW210XX_REG_SL06					(0x4C)
#define AW210XX_REG_SL07					(0x4D)
#define AW210XX_REG_SL08					(0x4E)
#define AW210XX_REG_SL09					(0x4F)
#define AW210XX_REG_SL10					(0x50)
#define AW210XX_REG_SL11					(0x51)
#define AW210XX_REG_SL12					(0x52)
#define AW210XX_REG_SL13					(0x53)
#define AW210XX_REG_SL14					(0x54)
#define AW210XX_REG_SL15					(0x55)
#define AW210XX_REG_SL16					(0x56)
#define AW210XX_REG_SL17					(0x57)
#define AW210XX_REG_GCCR					(0x58)
#define AW210XX_REG_PHCR					(0x59)
#define AW210XX_REG_OSDCR					(0x5A)
#define AW210XX_REG_OSST0					(0x5B)
#define AW210XX_REG_OSST1					(0x5C)
#define AW210XX_REG_OSST2					(0x5D)
#define AW210XX_REG_OTCR					(0x5E)
#define AW210XX_REG_SSCR					(0x5F)
#define AW210XX_REG_UVCR					(0x60)
#define AW210XX_REG_GCR2					(0x61)
#define AW210XX_REG_GCR3					(0x62)
#define AW210XX_REG_RESET					(0x70)
#define AW210XX_REG_ABMCFG					(0x80)
#define AW210XX_REG_ABMGO					(0x81)
#define AW210XX_REG_ABMT0					(0x82)
#define AW210XX_REG_ABMT1					(0x83)
#define AW210XX_REG_ABMT2					(0x84)
#define AW210XX_REG_ABMT3					(0x85)
#define AW210XX_REG_GBRH					(0x86)
#define AW210XX_REG_GBRL					(0x87)
#define AW210XX_REG_GSLR					(0x88)
#define AW210XX_REG_GSLG					(0x89)
#define AW210XX_REG_GSLB					(0x8A)
#define AW210XX_REG_GCFG					(0x8B)

/*****************************************************
 * define register Detail
*****************************************************/
#define AW210XX_BIT_APSE_MASK				(~(1 << 7))
#define AW210XX_BIT_APSE_ENABLE				(1 << 7)
#define AW210XX_BIT_APSE_DISENA				(0 << 7)
#define AW210XX_BIT_CHIPEN_MASK				(~(1 << 0))
#define AW210XX_BIT_CHIPEN_ENABLE			(1 << 0)
#define AW210XX_BIT_CHIPEN_DISENA			(0 << 0)
#define AW210XX_BIT_UVPD_MASK				(~(1 << 1))
#define AW210XX_BIT_UVPD_ENABLE				(0 << 1)
#define AW210XX_BIT_UVPD_DISENA				(1 << 1)
#define AW210XX_BIT_UVDIS_MASK				(~(1 << 0))
#define AW210XX_BIT_UVDIS_ENABLE			(0 << 0)
#define AW210XX_BIT_UVDIS_DISENA			(1 << 0)
#define AW210XX_BIT_ABME_MASK				(~(1 << 0))
#define AW210XX_BIT_ABME_ENABLE				(1 << 0)
#define AW210XX_BIT_ABME_DISENA				(0 << 0)
#define AW210XX_BIT_GSLDIS_MASK				(~(1 << 6))
#define AW210XX_BIT_GSLDIS_ENABLE			(0 << 6)
#define AW210XX_BIT_GSLDIS_DISENA			(1 << 6)
#define AW210XX_BIT_RGBMD_MASK				(~(1 << 0))
#define AW210XX_BIT_RGBMD_ENABLE			(1 << 0)
#define AW210XX_BIT_RGBMD_DISENA			(0 << 0)
#define AW210XX_BIT_SBMD_MASK				(~(1 << 1))
#define AW210XX_BIT_SBMD_ENABLE				(1 << 1)
#define AW210XX_BIT_SBMD_DISENA				(0 << 1)
#define AW210XX_BIT_CLKFRQ_MASK				(~(7 << 4))
#define AW210XX_BIT_CLKFRQ_16MHz			(0 << 4)
#define AW210XX_BIT_CLKFRQ_8MHz				(1 << 4)
#define AW210XX_BIT_CLKFRQ_1MHz				(2 << 4)
#define AW210XX_BIT_CLKFRQ_512kHz			(3 << 4)
#define AW210XX_BIT_CLKFRQ_256kHz			(4 << 4)
#define AW210XX_BIT_CLKFRQ_125kHz			(5 << 4)
#define AW210XX_BIT_CLKFRQ_62_5kHz			(6 << 4)
#define AW210XX_BIT_CLKFRQ_31_25kHz			(7 << 4)

#define AW210XX_BIT_PWMRES_MASK				(~(3 << 1))
#define AW210XX_BIT_PWMRES_8BIT				(0 << 1)
#define AW210XX_BIT_PWMRES_9BIT				(1 << 1)
#define AW210XX_BIT_PWMRES_12BIT			(2 << 1)
#define AW210XX_BIT_PWMRES_9_AND_3_BIT		(3 << 1)

#define AW210XX_DCPWM_SET					(7 << 5)
#define AW210XX_DCPWM_CLEAN					(0x00)
#define AW210XX_DCPWM_SET_MASK				~(7 << 5)
#define AW210XX_OPEN_THRESHOLD_SET			(1 << 3)
#define AW210XX_OPEN_THRESHOLD_SET_MASK		~(1 << 3)
#define AW210XX_SHORT_THRESHOLD_SET			(1 << 2)
#define AW210XX_SHORT_THRESHOLD_SET_MASK	~(1 << 2)

/*****************************************************
 * define register data
*****************************************************/
#define AW210XX_RESET_CHIP					(0x00)
#define AW210XX_UPDATE_BR_SL				(0x00)
#define AW21018_GROUP_ENABLE				(0x3F)
#define AW21018_GROUP_DISABLE				(0x40)
#define AW21012_GROUP_ENABLE				(0x3C)
#define AW21012_GROUP_DISABLE				(0x40)
#define AW21009_GROUP_ENABLE				(0x38)
#define AW21009_GROUP_DISABLE				(0x40)
#define AW210XX_GLOBAL_DEFAULT_SET			(0x66)
#define AW210XX_GBRH_DEFAULT_SET			(0x60)
#define AW210XX_GBRL_DEFAULT_SET			(0x00)
#define AW210XX_ABMT0_SET					(0x99)
#define AW210XX_ABMT1_SET					(0x99)
#define AW210XX_ABMT2_SET					(0x00)
#define AW210XX_ABMT3_SET					(0x00)
#define AW210XX_ABMCFG_SET					(0x03)
#define AW210XX_ABMGO_SET					(0x01)

/*****************************************************
* define chipid info
*****************************************************/
#define AW95016_CHIPID						(0x00)
#define AW95016A_CHIPID						(0x80)
#define AW21018_CHIPID						(0x02)
#define AW21012_CHIPID						(0x22)
#define AW21009_CHIPID						(0x12)
#define AW21018_LED_NUM						(0x12)
#define AW21012_LED_NUM						(0x0b)
#define AW21009_LED_NUM						(0x08)
#define AW210XX_OPEN_DETECT_EN				(0x03)
#define AW210XX_SHORT_DETECT_EN				(0x02)
#define AW210XX_OPEN_SHORT_DIS				(0x00)

/*****************************************************
* define return value
*****************************************************/
#define AW210XX_RETURN_OK					(0)
#define AW210XX_CHIPID_FAILD				(1)
#define AW210XX_DATA_ERROR					(2)
#define AW210XX_INPUT_ERROR					(3)
#define AW210XX_NO_DECTECT					(4)
#define AW210XX_NO_GPIO_INFO				(5)
#define AW210XX_NO_ACCESS					(6)
#define AW210XX_WRITE_FAIL					(7)
#define AW210XX_EFFECT_MODE_UNSUPPORT		(8)
#define AW210XX_CLK_MODE_UNSUPPORT			(9)
#define AW210XX_MALLOC_FAILEDT				(10)
#define AW210XX_OPEN_DETECT					(1)
#define AW210XX_SHORT_DETECT				(0)

/***********************************************
 * define register Detail
 ***********************************************/
#define REG_NONE_ACCESS						(0)
#define REG_RD_ACCESS						(1<<0)
#define REG_WR_ACCESS						(1<<1)
#define AW210XX_REG_MAX						(0x8C)

static uint8_t aw210xx_reg_access[AW210XX_REG_MAX] = {
	[AW210XX_REG_GCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR00L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR00H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR01L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR01H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR02L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR02H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR03L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR03H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR04L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR04H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR05L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR05H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR06L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR06H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR07L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR07H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR08L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR08H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR09L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR09H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR10L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR10H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR11L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR11H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR12L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR12H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR13L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR13H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR14L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR14H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR15L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR15H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR16L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR16H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR17L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_BR17H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_UPDATE] = REG_WR_ACCESS,
	[AW210XX_REG_SL00] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL01] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL02] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL03] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL04] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL05] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL06] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL07] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL08] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL09] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL10] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL11] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL12] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL13] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL14] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL15] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL16] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SL17] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_PHCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_OSDCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_OSST0] = REG_RD_ACCESS,
	[AW210XX_REG_OSST1] = REG_RD_ACCESS,
	[AW210XX_REG_OSST2] = REG_RD_ACCESS,
	[AW210XX_REG_OTCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_SSCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_UVCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_RESET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMGO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_ABMT3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GBRH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GBRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GSLB] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW210XX_REG_GCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
};

typedef enum {
	CLK_FRQ_16M = 1,
	CLK_FRQ_8M,
	CLK_FRQ_1M,
	CLK_FRQ_512k,
	CLK_FRQ_256k,
	CLK_FRQ_125K,
	CLK_FRQ_62_5K,
	CLK_FRQ_31_25K,
} clk_pwm_t;

typedef enum {
	BR_RESOLUTION_8BIT = 1,
	BR_RESOLUTION_9BIT,
	BR_RESOLUTION_12BIT,
	BR_RESOLUTION_9_AND_3_BIT,
} br_pwm_t;

typedef struct aw210xx_cfg {
	uint8_t *p;
	uint32_t count;
} aw210xx_cfg_t;

typedef struct aw210xx_effect_cfg {
	uint32_t *p;
	uint32_t count;
} aw210xx_effect_cfg_t;

typedef struct {
	uint32_t time[5];
	uint32_t repeat_nums;
	uint16_t  fadeh;
	uint16_t  fadel;
} AW_MULTI_BREATH_DATA_STRUCT;

typedef struct{
	uint16_t total_frames;
	uint16_t cur_frame;
	uint16_t data_start;
	uint16_t data_end;
} ALGO_DATA_STRUCT;

typedef struct aw210xx_music_state {
	int f_cam_led_br;
	int r_cam_led_br;
	int round_leds_br;
	int vline_leds_br;
	int dot_led_br;
} aw210xx_music_state_t;


typedef struct aw210xx_r_cam_state {
	int state;
	int r_cam_led_br;
	int all_white_leds_br;
} aw210xx_r_cam_state_t;

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct aw210xx {
	struct i2c_client *i2c;
	struct device *dev;
	struct led_classdev cdev;
	struct workqueue_struct *leds_workqueue;
	struct workqueue_struct *video_workqueue;
	struct work_struct brightness_work;
	struct work_struct wired_charging_work;
	struct work_struct wlr_charging_work;
	struct work_struct random_effect_work;
	struct work_struct video_effect_work;
	struct work_struct ga_effect_work;
	struct work_struct nf_effect_work;
	struct work_struct bootan_effect_work;
	struct work_struct ringtone_effect_work;
	struct work_struct flip_effect_work;
	int wired_charging;
	int wlr_charging;
	int random_effect;
	int random_delay_t;
	int video_effect;
	uint32_t ga_effect;
	int nf_effect;
	int bootan_effect;
	int ringtone_effect;
	int flip_effect;
	int exclamation_effect;
	aw210xx_music_state_t music_effect;
	aw210xx_r_cam_state_t r_cam_effect;
	uint8_t sdmd_flag;
	uint8_t rgbmd_flag;
	br_pwm_t br_res;
	clk_pwm_t osc_clk;
	uint8_t chipid;
	uint8_t all_leds_effect;
	int enable_gpio;
	uint32_t rgbcolor;
	uint32_t glo_current;
	unsigned int imax;
	int horse_race_leds_br;
	int round_leds_br;
	int vline_leds_br;
	int all_white_leds_br;
	int setting_br;
	struct mutex led_mutex;
};

#endif
