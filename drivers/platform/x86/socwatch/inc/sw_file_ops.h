#ifndef __SW_FILE_OPS_H__
#define __SW_FILE_OPS_H__

enum sw_driver_collection_cmd;
struct sw_file_ops {
	long (*ioctl_handler) (unsigned int ioctl_num, void *local_args);
	int (*stop_handler) (void);
	enum sw_driver_collection_cmd (*get_current_cmd) (void);
	 bool(*should_flush) (void);
};

int sw_register_dev(struct sw_file_ops *ops);
void sw_unregister_dev(void);

#endif // __SW_FILE_OPS_H__
