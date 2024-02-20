/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _LINUX_SOUNDWIRE_H
#define _LINUX_SOUNDWIRE_H
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/irqdomain.h>
#include <linux/regmap.h>
#include "audio_mod_devicetable.h"

#define SWR_CLK_RATE_0P3MHZ       300000
#define SWR_CLK_RATE_0P6MHZ       600000
#define SWR_CLK_RATE_1P2MHZ      1200000
#define SWR_CLK_RATE_2P4MHZ      2400000
#define SWR_CLK_RATE_4P8MHZ      4800000
#define SWR_CLK_RATE_9P6MHZ      9600000
#define SWR_CLK_RATE_11P2896MHZ  11289600

extern struct bus_type soundwire_type;
struct swr_device;

/* Soundwire supports max. of 8 channels per port */
#define SWR_MAX_CHANNEL_NUM	8
/* Soundwire supports max. of 14 ports on each device */
#define SWR_MAX_DEV_PORT_NUM	14
/* Maximum number of slave devices that a master can control */
#define SWR_MAX_DEV_NUM		11
/* Maximum number of ports on master so that it can accommodate all the port
 * configurations of all devices
 */
#define SWR_MAX_MSTR_PORT_NUM	(SWR_MAX_DEV_NUM * SWR_MAX_DEV_PORT_NUM)

/* Regmap support for soundwire interface */
struct regmap *__devm_regmap_init_swr(struct swr_device *dev,
				      const struct regmap_config *config,
				      struct lock_class_key *lock_key,
				      const char *lock_name);

/**
 * regmap_init_swr(): Initialise register map
 *
 * @swr: Device that will be interacted with
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer to
 * a struct regmap.
 */
#define regmap_init_swr(swr, config)					\
	__regmap_lockdep_wrapper(__regmap_init_swr, #config,		\
				swr, config)

/**
 * devm_regmap_init_swr(): Initialise managed register map
 *
 * @swr: Device that will be interacted with
 * @config: Configuration for register map
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct regmap.  The regmap will be automatically freed by the
 * device management code.
 */
#define devm_regmap_init_swr(swr, config)                              \
	__regmap_lockdep_wrapper(__devm_regmap_init_swr, #config,       \
				swr, config)

/* Indicates soundwire devices group information */
enum {
	SWR_GROUP_NONE = 0,
	SWR_GROUP_12 = 12,
	SWR_GROUP_13 = 13,
	SWR_BROADCAST = 15,
};

/*
 * struct swr_port_info - represents new soundwire frame shape
 *							with full data ports
 * @list: link with other soundwire port info nodes
 * @dev_num: logical device number of the soundwire slave device
 * @port_en: flag indicates whether the port is enabled
 * @slave_port_id: logical port number of the soundwire slave device
 * @offset1: sample offset indicating the offset of the channel
 * from the start of the frame
 * @offset2: channel offset indicating offset between to channels
 * @hstart: start offset for subframe window.
 * @hstop: start offset for subframe window.
 * @master_port_id: logical port number of corresponding soundwire master device
 * @blk_grp_count: grouping count for n.o of channels.
 * @blk_pack_mode: packing mode for channels in each port.
 * @sinterval: sample interval indicates spacing from one sample
 * event to the next
 * @ch_en: channels enabled in a port.
 * @req_ch: channels requested to be enabled in a port.
 * @num_ch: number of channels enabled in a port
 * @ch_rate: sampling rate of the channel with which data will be
 * transferred
 *
 * Soundwire frame shape is created based on swr_port_info struct
 * parameters.
 */
struct swr_port_info {
	u8 dev_num;
	u8 port_en;
	u8 slave_port_id;
	u8 offset1;
	u8 offset2;
	u16 sinterval;
	struct list_head list;
	u8 master_port_id;
	u8 hstart;
	u8 hstop;
	u8 blk_grp_count;
	u8 blk_pack_mode;
	u8 word_length;
	u8 lane_ctrl;
	u8 ch_en;
	u8 req_ch;
	u8 num_ch;
	u32 ch_rate;
};

/*
 * struct swr_params - represent transfer of data from soundwire slave
 * to soundwire master
 * @tid: transaction ID to track each transaction
 * @dev_num: logical device number of the soundwire slave device
 * @num_port: number of ports that needs to be configured
 * @port_id: array of logical port numbers of the soundwire slave device
 * @num_ch: array of number of channels enabled
 * @ch_rate: array of sampling rate of different channels that need to
 * be configured
 * @ch_en: array of channels mask for all the ports
 * @port_type: the required master port type
 */
struct swr_params {
	u8 tid;
	u8 dev_num;
	u8 num_port;
	u8 port_id[SWR_MAX_DEV_PORT_NUM];
	u8 num_ch[SWR_MAX_DEV_PORT_NUM];
	u32 ch_rate[SWR_MAX_DEV_PORT_NUM];
	u8 ch_en[SWR_MAX_DEV_PORT_NUM];
	u8 port_type[SWR_MAX_DEV_PORT_NUM];
};

/*
 * struct swr_reg - struct to handle soundwire slave register read/writes
 * @tid: transaction id for reg read/writes
 * @dev_id: logical device number of the soundwire slave device
 * @regaddr: 16 bit regaddr of soundwire slave
 * @buf: value to be written/read to/from regaddr
 * @len: length of the buffer buf
 */
struct swr_reg {
	u8  tid;
	u8  dev_id;
	u32 regaddr;
	u32 *buf;
	u32 len;
};

/*
 * struct swr_master - Interface to the soundwire master controller
 * @dev: device interface to this driver
 * @list: link with other soundwire master controllers
 * @bus_num: board/SoC specific identifier for a soundwire master
 * @mlock: mutex protecting master data structures
 * @devices: list of devices on this master
 * @port: logical port numbers of the soundwire master. This array
 * can hold maximum master ports which is equal to number of slave
 * devices multiplied by number of ports in each slave device
 * @port_txn: table of port config transactions with transaction id
 * @reg_txn: table of register transactions with transaction id
 * @last_tid: size of table port_txn (can't grow beyond 256 since
 * tid is 8 bits)
 * @num_port: number of active ports on soundwire master
 * @gr_sid: slave id used by the group for write operations
 * @connect_port: callback for configuration of soundwire port(s)
 * @disconnect_port: callback for disable of soundwire port(s)
 * @read: callback for soundwire slave register read
 * @write: callback for soundwire slave register write
 * @get_logical_dev_num: callback to get soundwire slave logical
 * device number
 * @port_en_mask: bit mask of active ports on soundwire master
 */
struct swr_master {
	struct device dev;
	struct list_head list;
	unsigned int bus_num;
	struct mutex mlock;
	struct list_head devices;
	struct swr_port_info port[SWR_MAX_MSTR_PORT_NUM];
	struct swr_params **port_txn;
	struct swr_reg **reg_txn;
	u8 last_tid;
	u8 num_port;
	u8 num_dev;
	u8 gr_sid;
	int (*connect_port)(struct swr_master *mstr, struct swr_params *txn);
	int (*disconnect_port)(struct swr_master *mstr, struct swr_params *txn);
	int (*read)(struct swr_master *mstr, u8 dev_num, u16 reg_addr,
			void *buf, u32 len);
	int (*write)(struct swr_master *mstr, u8 dev_num, u16 reg_addr,
			const void *buf);
	int (*bulk_write)(struct swr_master *master, u8 dev_num, void *reg,
			  const void *buf, size_t len);
	int (*get_logical_dev_num)(struct swr_master *mstr, u64 dev_id,
				u8 *dev_num);
	int (*slvdev_datapath_control)(struct swr_master *mstr, bool enable);
	bool (*remove_from_group)(struct swr_master *mstr);
	void (*device_wakeup_vote)(struct swr_master *mstr);
	void (*device_wakeup_unvote)(struct swr_master *mstr);
	u16 port_en_mask;

};

static inline struct swr_master *to_swr_master(struct device *dev)
{
	return dev ? container_of(dev, struct swr_master, dev) : NULL;
}

/*
 * struct swr_device - represent a soundwire slave device
 * @name: indicates the name of the device, defined in devicetree
 * binding under soundwire slave device node as a compatible field.
 * @master: soundwire master managing the bus hosting this device
 * @driver: Device's driver. Pointer to access routines
 * @dev_list: list of devices on a controller
 * @dev_num: logical device number of the soundwire slave device
 * @dev: driver model representation of the device
 * @addr: represents "ea-addr" which is unique-id of soundwire slave
 * device
 * @group_id: group id supported by the slave device
 * @slave_irq: irq handle of slave to be invoked by master
 * during slave interrupt
 */
struct swr_device {
	char name[SOUNDWIRE_NAME_SIZE];
	struct swr_master *master;
	struct swr_driver *driver;
	struct list_head dev_list;
	u8               dev_num;
	struct device    dev;
	u64 addr;
	u8 group_id;
	struct irq_domain *slave_irq;
	bool slave_irq_pending;
};

static inline struct swr_device *to_swr_device(struct device *dev)
{
	return dev ? container_of(dev, struct swr_device, dev) : NULL;
}

/*
 * struct swr_driver - Manage soundwire slave device driver
 * @probe: binds this driver to soundwire device
 * @remove: unbinds this driver from soundwire device
 * @shutdown: standard shutdown callback used during power down/halt
 * @suspend: standard suspend callback used during system suspend
 * @resume: standard resume callback used during system resume
 * @driver: soundwire device drivers should initialize name and
 * owner field of this structure
 * @id_table: list of soundwire devices supported by this driver
 */
struct swr_driver {
	int	(*probe)(struct swr_device *swr);
	int	(*remove)(struct swr_device *swr);
	void	(*shutdown)(struct swr_device *swr);
	int	(*suspend)(struct swr_device *swr, pm_message_t pmesg);
	int	(*resume)(struct swr_device *swr);
	int	(*device_up)(struct swr_device *swr);
	int	(*device_down)(struct swr_device *swr);
	int	(*reset_device)(struct swr_device *swr);
	struct device_driver		driver;
	const struct swr_device_id	*id_table;
};

static inline struct swr_driver *to_swr_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct swr_driver, driver) : NULL;
}

/*
 * struct swr_boardinfo - Declare board info for soundwire device bringup
 * @name: name to initialize swr_device.name
 * @bus_num: identifies which soundwire master parents the soundwire
 * slave_device
 * @addr: represents "ea-addr" of soundwire slave device
 * @of_node: pointer to OpenFirmware device node
 * @swr_slave: device to be registered with soundwire
 */
struct swr_boardinfo {
	char               name[SOUNDWIRE_NAME_SIZE];
	int                bus_num;
	u64		   addr;
	struct device_node *of_node;
	struct swr_device  *swr_slave;
};

static inline void *swr_get_ctrl_data(const struct swr_master *master)
{
	return master ? dev_get_drvdata(&master->dev) : NULL;
}

static inline void swr_set_ctrl_data(struct swr_master *master, void *data)
{
	dev_set_drvdata(&master->dev, data);
}

static inline void *swr_get_dev_data(const struct swr_device *dev)
{
	return dev ? dev_get_drvdata(&dev->dev) : NULL;
}

static inline void swr_set_dev_data(struct swr_device *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

extern int swr_startup_devices(struct swr_device *swr_dev);

extern struct swr_device *swr_new_device(struct swr_master *master,
				struct swr_boardinfo const *info);

extern int of_register_swr_devices(struct swr_master *master);

extern void swr_port_response(struct swr_master *mstr, u8 tid);

extern int swr_get_logical_dev_num(struct swr_device *dev, u64 dev_id,
			u8 *dev_num);

extern int swr_read(struct swr_device *dev, u8 dev_num, u16 reg_addr,
			void *buf, u32 len);

extern int swr_write(struct swr_device *dev, u8 dev_num, u16 reg_addr,
			const void *buf);

extern int swr_bulk_write(struct swr_device *dev, u8 dev_num, void *reg_addr,
			  const void *buf, size_t len);

extern int swr_connect_port(struct swr_device *dev, u8 *port_id, u8 num_port,
				u8 *ch_mask, u32 *ch_rate, u8 *num_ch,
				u8 *port_type);

extern int swr_disconnect_port(struct swr_device *dev,
				u8 *port_id, u8 num_port, u8 *ch_mask,
				u8 *port_type);

extern int swr_set_device_group(struct swr_device *swr_dev, u8 id);

extern int swr_driver_register(struct swr_driver *drv);

extern void swr_driver_unregister(struct swr_driver *drv);

extern int swr_add_device(struct swr_master *master,
				struct swr_device *swrdev);
extern void swr_remove_device(struct swr_device *swr);

extern void swr_master_add_boarddevices(struct swr_master *master);

extern void swr_unregister_master(struct swr_master *master);

extern int swr_register_master(struct swr_master *master);

extern int swr_device_up(struct swr_device *swr_dev);

extern int swr_device_down(struct swr_device *swr_dev);

extern int swr_reset_device(struct swr_device *swr_dev);

extern int swr_slvdev_datapath_control(struct swr_device *swr_dev, u8 dev_num,
				       bool enable);
extern int swr_remove_from_group(struct swr_device *dev, u8 dev_num);

extern void swr_remove_device(struct swr_device *swr_dev);

extern struct swr_device *get_matching_swr_slave_device(struct device_node *np);

extern int swr_device_wakeup_vote(struct swr_device *dev);

extern int swr_device_wakeup_unvote(struct swr_device *dev);

#endif /* _LINUX_SOUNDWIRE_H */
