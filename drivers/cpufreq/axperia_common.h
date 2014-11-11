#ifndef __AXPERIA_COMMON__
#define __AXPERIA_COMMON__

#include <linux/kallsyms.h>

#define DEVICE_NAME "Xperia NovaThor U8500"
// for get proc address
typedef unsigned long (*kallsyms_lookup_name_type)(const char *name);
static kallsyms_lookup_name_type kallsyms_lookup_name_ax = kallsyms_lookup_name;


#endif
