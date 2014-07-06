/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010-2011
 *
 * License Terms: GNU General Public License v2
 * Author: Sundar Iyer
 * Author: Martin Persson
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com>
 */

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <mach/id.h>

#include "../mfd/dbx500-prcmu-regs.h"

static struct cpufreq_frequency_table db8500_freq_table[] = {
	[0] = {
		.index = 0,
		.frequency = 200000,
	},
	[1] = {
		.index = 1,
		.frequency = 400000,
	},
	[2] = {
		.index = 2,
		.frequency = 800000,
	},
	[3] = {
		/* Used for MAX_OPP, if available */
		.index = 3,
		.frequency = CPUFREQ_TABLE_END,
	},
	[4] = {
		.index = 4,
		.frequency = CPUFREQ_TABLE_END,
	},
#ifdef CONFIG_DB8500_LIVEOPP
	[5] = {
		.index = 5,
		.frequency = CPUFREQ_TABLE_END,
	},
	[6] = {
		.index = 6,
		.frequency = CPUFREQ_TABLE_END,
	},
	[7] = {
		.index = 7,
		.frequency = CPUFREQ_TABLE_END,
	},
	[8] = {
		.index = 8,
		.frequency = CPUFREQ_TABLE_END,
	},
	[9] = {
		.index = 9,
		.frequency = CPUFREQ_TABLE_END,
	},
	[10] = {
		.index = 10,
		.frequency = CPUFREQ_TABLE_END,
	},
	[11] = {
		.index = 11,
		.frequency = CPUFREQ_TABLE_END,
	},
	[12] = {
		.index = 12,
		.frequency = CPUFREQ_TABLE_END,
	},
	[13] = {
		.index = 13,
		.frequency = CPUFREQ_TABLE_END,
	},
	[14] = {
		.index = 14,
		.frequency = CPUFREQ_TABLE_END,
	},
	[15] = {
		.index = 15,
		.frequency = CPUFREQ_TABLE_END,
	},
	[16] = {
		.index = 16,
		.frequency = CPUFREQ_TABLE_END,
	},
	[17] = {
		.index = 17,
		.frequency = CPUFREQ_TABLE_END,
	},
	[18] = {
		.index = 18,
		.frequency = CPUFREQ_TABLE_END,
	},
	[19] = {
		.index = 19,
		.frequency = CPUFREQ_TABLE_END,
	},
	[20] = {
		.index = 20,
		.frequency = CPUFREQ_TABLE_END,
	},
#endif
};

static struct cpufreq_frequency_table db5500_freq_table[] = {
	[0] = {
		.index = 0,
		.frequency = 200000,
	},
	[1] = {
		.index = 1,
		.frequency = 396500,
	},
	[2] = {
		.index = 2,
		.frequency = 793000,
	},
	[3] = {
		.index = 3,
		.frequency = CPUFREQ_TABLE_END,
	},
};

static struct cpufreq_frequency_table *freq_table;
static int freq_table_len;

#ifdef CONFIG_DB8500_LIVEOPP
#include <linux/kobject.h>
#include <linux/mfd/db8500-liveopp.h>

#define LiveOPP_VER		"1.0.1"

#define NOCHG			0
#define SET_PLL			1
#define SET_EXT			1
#define SET_VOLT		1

static unsigned int last_arm_idx = 0;

/**
 * Hard-coded Custom ARM Frequency and Voltage Table
 */
struct liveopp_arm_table liveopp_arm[] = {
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	{100000,    99840, ARM_EXTCLK,  SET_EXT, 0x582, NOCHG,   0x00050168, SET_VOLT, 0x0C, 0x18, 0xDB},
#endif
	{200000,   199680, ARM_EXTCLK,  NOCHG,   0x581, NOCHG,   0x00050168, SET_VOLT, 0x0C, 0x18, 0xDB},
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	{300000,   299520, ARM_50_OPP,  NOCHG,   0x741, SET_PLL, 0x00050127, SET_VOLT, 0x0C, 0x19, 0xDB},
#endif
	{400000,   399360, ARM_50_OPP,  NOCHG,   0x741, SET_PLL, 0x01050168, SET_VOLT, 0x0C, 0x1A, 0xDB},
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	{500000,   499200, ARM_50_OPP,  NOCHG,   0x741, SET_PLL, 0x0001010D, SET_VOLT, 0x0C, 0x1E, 0xDB},
#endif
	{600000,   599040, ARM_50_OPP,  NOCHG,   0x741, SET_PLL, 0x0005014E, SET_VOLT, 0x0C, 0x20, 0xDB},
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	{700000,   698880, ARM_50_OPP,  NOCHG,   0x741, SET_PLL, 0x0005015B, SET_VOLT, 0x0C, 0x22, 0xDB},
#endif
	{800000,   798720, ARM_100_OPP, NOCHG,   0x741, SET_PLL, 0x00050168, NOCHG,    0x0B, 0x24, 0xDB},
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	{900000,   898560, ARM_100_OPP, NOCHG,   0x741, SET_PLL, 0x00050175, SET_VOLT, 0x0B, 0x26, 0xDB},
#endif
	{1000000,  998400, ARM_MAX_OPP, NOCHG,   0x741, NOCHG,   0x0001011A, NOCHG,    0x0B, 0x2F, 0xDB},
	{1050000, 1049600, ARM_MAX_OPP, NOCHG,   0x741, SET_PLL, 0x00030152, SET_VOLT, 0x0B, 0x32, 0xDB},
	{1100000, 1100800, ARM_MAX_OPP, NOCHG,   0x741, SET_PLL, 0x00030156, SET_VOLT, 0x0B, 0x3F, 0x8F},
	{1150000, 1152000, ARM_MAX_OPP, NOCHG,   0x741, SET_PLL, 0x0001011E, SET_VOLT, 0x0B, 0x3F, 0x8F},
	{1200000, 1200000, ARM_MAX_OPP, NOCHG,   0x741, SET_PLL, 0x0004017D, SET_VOLT, 0x0B, 0x3F, 0x8F},
	{1250000, 1228800, ARM_MAX_OPP, NOCHG,   0x741, SET_PLL, 0x00010120, SET_VOLT, 0x0B, 0x3F, 0x8F},
};

static void liveopp_set_armvolt(struct liveopp_arm_table table)
{
	/* Varm */
	prcmu_abb_write(AB8500_REGU_CTRL2, table.varm_sel, &table.varm_raw, 1);

	/* VBBp/VBBn */
	prcmu_abb_write(AB8500_REGU_CTRL2, AB8500_VBBX_REG, &table.vbbx_raw, 1);
}

static void liveopp_set_armpll(struct liveopp_arm_table table)
{
	/* ARM PLL */
	db8500_prcmu_writel(PRCMU_PLLARM_REG, table.pllarm_raw);
}

static void liveopp_set_armext(struct liveopp_arm_table table)
{
	/* ArmFixClk */
	db8500_prcmu_writel(PRCMU_ARMFIX_REG, table.extarm_raw);
}

void liveopp_update_arm(struct liveopp_arm_table table)
{
	if (table.set_volt)
		liveopp_set_armvolt(table);

	if (table.set_pllarm)
		liveopp_set_armpll(table);

	if (table.set_extarm)
		liveopp_set_armext(table);
}

#define ATTR_RO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0444, _name##_show, NULL);

#define ATTR_WO(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0220, NULL, _name##_store);

#define ATTR_RW(_name)	\
	static struct kobj_attribute _name##_interface = __ATTR(_name, 0644, _name##_show, _name##_store);


static const char *armopp_name[] = 
{
	"ARM_OPP_INIT",		/* 0x00 */
	"ARM_NO_CHANGE",	/* 0x01 */
	"ARM_100_OPP", 		/* 0x02 */
	"ARM_50_OPP", 		/* 0x03 */
	"ARM_MAX_OPP", 		/* 0x04 */
	"ARM_MAX_FREQ100OPP", 	/* 0x05 */
	"(null)",		/* 0x06 */
	"ARM_EXTCLK",		/* 0x07 */
};

static int varm_voltage(u8 raw)
{
	if (raw <= 0x35) {
		return (AB8500_VARM_MIN_UV + (raw * AB8500_VARM_STEP_UV));
	} else {
		return AB8500_VARM_MAX_UV;
	}
}

static int pllarm_freq(u32 raw)
{
	int multiple = raw & 0x000000FF;
	int divider = (raw & 0x00FF0000) >> 16;
	int half = (raw & 0x01000000) >> 24;
	int pll;

	pll = (multiple * PLLARM_FREQ_STEPS);
	pll /= divider;

	if (half) {
		pll /= 2;
	}

	return pll;
}

/* 
 * LiveOPP sysfs interfaces
 */

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	sprintf(buf, "LiveOPP (%s), cocafe\n", LiveOPP_VER);

	return strlen(buf);
}

ATTR_RO(version);

static ssize_t arm_extclk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	u32 r;
	unsigned long rate;

	r = readl(PRCM_ARM_CHGCLKREQ);

	/* External ARM clock uses PLLDDR */
	/* We use PLLARM function here, they are common */
	rate = pllarm_freq(db8500_prcmu_readl(PRCMU_PLLDDR_REG));

	/* Check PRCM_ARM_CHGCLKREQ divider */
	if (!(r & PRCM_ARM_CHGCLKREQ_PRCM_ARM_DIVSEL))
		rate /= 2;

	/* Check PRCM_ARMCLKFIX_MGT divider */
	r = readl(PRCM_ARMCLKFIX_MGT);
	r &= PRCM_CLK_MGT_CLKPLLDIV_MASK;
	rate /= r;

	/* PLLDDR belongs to PLL_FIX branch */
	return sprintf(buf, "%lu kHz\n", rate / 2);
}
ATTR_RO(arm_extclk);

static ssize_t arm_pllclk_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d kHz\n", pllarm_freq(db8500_prcmu_readl(PRCMU_PLLARM_REG)));
}
ATTR_RO(arm_pllclk);

#define ARM_STEP(_name, _index)	\
static ssize_t _name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{	\
	sprintf(buf, "[LiveOPP ARM Step %d]\n\n", _index);	\
	sprintf(buf, "%sSet EXTARM:\t\t%s\n", buf, liveopp_arm[_index].set_extarm ? "Enabled" : "Disabled");	\
	sprintf(buf, "%sSet PLLARM:\t\t%s\n", buf, liveopp_arm[_index].set_pllarm ? "Enabled" : "Disabled");	\
	sprintf(buf, "%sSet Voltage:\t\t%s\n", buf, liveopp_arm[_index].set_volt ? "Enabled" : "Disabled");	\
	sprintf(buf, "%sFrequency show:\t\t%d kHz\n", buf, liveopp_arm[_index].freq_show);	\
	sprintf(buf, "%sFrequency real:\t\t%d kHz\n", buf, liveopp_arm[_index].set_pllarm ? 	\
			pllarm_freq(liveopp_arm[_index].pllarm_raw) : liveopp_arm[_index].freq_raw);	\
	sprintf(buf, "%sArmFix:\t\t\t%#010x\n", buf, liveopp_arm[_index].extarm_raw);	\
	sprintf(buf, "%sArmPLL:\t\t\t%#010x\n", buf, liveopp_arm[_index].pllarm_raw);	\
	sprintf(buf, "%sArmOPP:\t\t\t%s (%#04x)\n", buf, armopp_name[(int)liveopp_arm[_index].arm_opp],	\
								     (int)liveopp_arm[_index].arm_opp);	\
	sprintf(buf, "%sVarm:\t\t\t%d uV (%#04x)\n", buf, varm_voltage(liveopp_arm[_index].varm_raw), 	\
								     (int)liveopp_arm[_index].varm_raw);	\
	sprintf(buf, "%sVbbx:\t\t\t%#04x\n", buf, (int)liveopp_arm[_index].vbbx_raw);	\
	\
	return sprintf(buf, "%s\n", buf);	\
}	\
	\
static ssize_t _name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)	\
{	\
	int ret;	\
	int val;	\
	\
	if (!strncmp(buf, "set_ext=", 8)) {	\
		ret = sscanf(&buf[8], "%d", &val);	\
		if ((!ret) || (val != 0 && val != 1)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].set_extarm = val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "set_pll=", 8)) {	\
		ret = sscanf(&buf[8], "%d", &val);	\
		if ((!ret) || (val != 0 && val != 1)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].set_pllarm = val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "set_volt=", 9)) {	\
		ret = sscanf(&buf[9], "%d", &val);	\
		if ((!ret) || (val != 0 && val != 1)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].set_volt = val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "opp=", 4)) {	\
		ret = sscanf(&buf[4], "%d", &val);	\
		if ((!ret) || (val < 0x00 || val > 0x07)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].arm_opp = (unsigned char)val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "pll=", 4)) {	\
		ret = sscanf(&buf[4], "%x", &val);	\
		if ((!ret)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].pllarm_raw = val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "ext=", 4)) {	\
		ret = sscanf(&buf[4], "%x", &val);	\
		if ((!ret)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].extarm_raw = val;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "varm+", 5)) {	\
		liveopp_arm[_index].varm_raw ++;	\
	\
		return count;	\
	}	\
	\
	if (!strncmp(buf, "varm-", 5)) {	\
		liveopp_arm[_index].varm_raw --;	\
	\
		return count;	\
	}	\
	if (!strncmp(buf, "varm=", 5)) {	\
		ret = sscanf(&buf[5], "%x", &val);	\
		if ((!ret)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].varm_raw = val;	\
	\
		return count;	\
	}	\
	if (!strncmp(buf, "vbbx=", 5)) {	\
		ret = sscanf(&buf[5], "%x", &val);	\
		if ((!ret)) {	\
			pr_err("[LiveOPP] Invalid value\n");	\
			return -EINVAL;	\
		}	\
	\
		liveopp_arm[_index].vbbx_raw = val;	\
	\
		return count;	\
	}	\
	\
	return count;	\
}	\
ATTR_RW(_name);

ARM_STEP(arm_step00, 0);
ARM_STEP(arm_step01, 1);
ARM_STEP(arm_step02, 2);
ARM_STEP(arm_step03, 3);
ARM_STEP(arm_step04, 4);
ARM_STEP(arm_step05, 5);
ARM_STEP(arm_step06, 6);
ARM_STEP(arm_step07, 7);
ARM_STEP(arm_step08, 8);
ARM_STEP(arm_step09, 9);
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
ARM_STEP(arm_step10, 10);
ARM_STEP(arm_step11, 11);
ARM_STEP(arm_step12, 12);
ARM_STEP(arm_step13, 13);
ARM_STEP(arm_step14, 14);
#endif

static struct attribute *liveopp_attrs[] = {
	&version_interface.attr, 
	&arm_extclk_interface.attr, 
	&arm_pllclk_interface.attr, 
	&arm_step00_interface.attr, 
	&arm_step01_interface.attr, 
	&arm_step02_interface.attr, 
	&arm_step03_interface.attr, 
	&arm_step04_interface.attr, 
	&arm_step05_interface.attr, 
	&arm_step06_interface.attr, 
	&arm_step07_interface.attr, 
	&arm_step08_interface.attr, 
	&arm_step09_interface.attr, 
#ifdef CONFIG_LIVEOPP_EXTENDED_FREQ
	&arm_step10_interface.attr, 
	&arm_step11_interface.attr, 
	&arm_step12_interface.attr, 
	&arm_step13_interface.attr, 
	&arm_step14_interface.attr, 
#endif
	NULL,
};

static struct attribute_group liveopp_interface_group = {
	.attrs = liveopp_attrs,
};

static struct kobject *liveopp_kobject;

#endif /* CONFIG_DB8500_LIVEOPP */

#ifndef CONFIG_DB8500_LIVEOPP
static enum arm_opp db8500_idx2opp[] = {
	ARM_EXTCLK,
	ARM_50_OPP,
	ARM_100_OPP,
	ARM_MAX_OPP
};
#endif /* CONFIG_DB8500_LIVEOPP */

static enum arm_opp db5500_idx2opp[] = {
	ARM_EXTCLK,
	ARM_50_OPP,
	ARM_100_OPP,
};

static enum arm_opp *idx2opp;

static struct freq_attr *dbx500_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static int dbx500_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static int dbx500_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	struct cpufreq_freqs freqs;
	unsigned int idx;

	/* scale the target frequency to one of the extremes supported */
	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;
	if (target_freq > policy->cpuinfo.max_freq)
		target_freq = policy->cpuinfo.max_freq;

	/* Lookup the next frequency */
	if (cpufreq_frequency_table_target
	    (policy, freq_table, target_freq, relation, &idx)) {
		return -EINVAL;
	}

	freqs.old = policy->cur;
	freqs.new = freq_table[idx].frequency;

	if (freqs.old == freqs.new)
		return 0;

	/* pre-change notification */
	for_each_cpu(freqs.cpu, policy->cpus)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	BUG_ON(idx >= freq_table_len);

	/* request the PRCM unit for opp change */
#ifdef CONFIG_DB8500_LIVEOPP
	int i;
#ifdef LIVEOPP_DEBUG
	pr_info("kHz: %lu\n", freq_table[last_arm_idx].frequency);
#endif
	for (i = 0; i < ARRAY_SIZE(liveopp_arm); i++) {
		if (freq_table[last_arm_idx].frequency == freq_table[i].frequency) {
			if (prcmu_get_arm_opp() == ARM_MAX_OPP) {
				db8500_prcmu_writel(PRCMU_PLLARM_REG, PLLARM_MAXOPP);
			}
			#if defined CONFIG_LIVEOPP_EXTENDED_FREQ
			 else if (prcmu_get_arm_opp() == ARM_100_OPP) {
				db8500_prcmu_writel(PRCMU_PLLARM_REG, PLLARM_FREQ100OPP);
			}
			#endif

			prcmu_set_arm_lopp(liveopp_arm[i].arm_opp, i);
			last_arm_idx = i;

			break;
		}
	}
#else
	if (prcmu_set_arm_opp(idx2opp[idx])) {
		pr_err("ux500-cpufreq:  Failed to set OPP level\n");
		return -EINVAL;
	}
#endif /* CONFIG_DB8500_LIVEOPP */

	/* post change notification */
	for_each_cpu(freqs.cpu, policy->cpus)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static unsigned int dbx500_cpufreq_getspeed(unsigned int cpu)
{
	int i;
	enum arm_opp current_opp;

	current_opp = prcmu_get_arm_opp();

	/* request the prcm to get the current ARM opp */
	for (i = 0;  i < freq_table_len; i++) {
#ifdef CONFIG_DB8500_LIVEOPP
		return freq_table[last_arm_idx].frequency;
#else
		if (current_opp == idx2opp[i])
			return freq_table[i].frequency;
#endif /* CONFIG_DB8500_LIVEOPP */
	}

	pr_err("cpufreq: ERROR: unknown opp %d given from prcmufw!\n",
	       current_opp);
	BUG_ON(1);

	/*
	 * Better to return something that might be correct than
	 * errno or zero, since clk_get_rate() won't do well with an errno.
	 */
	return freq_table[0].frequency;
}

static void __init dbx500_cpufreq_init_maxopp_freq(void)
{
	#ifdef CONFIG_DB8500_LIVEOPP
	int i;

	pr_info("[LiveOPP] Available freqs: %d\n", ARRAY_SIZE(liveopp_arm));
	for (i = 0; i < ARRAY_SIZE(liveopp_arm); i++) {
		freq_table[i].frequency = liveopp_arm[i].freq_show;
	}

	#else /* CONFIG_DB8500_LIVEOPP */
	struct prcmu_fw_version *fw_version = prcmu_get_fw_version();

	if ((fw_version == NULL) || !prcmu_has_arm_maxopp())
		return;

	switch (fw_version->project) {
	case PRCMU_FW_PROJECT_U8500:
	case PRCMU_FW_PROJECT_U9500:
	case PRCMU_FW_PROJECT_U8420:
		freq_table[3].frequency = 1000000;
		break;
	case PRCMU_FW_PROJECT_U8500_C2:
	case PRCMU_FW_PROJECT_U9500_C2:
	case PRCMU_FW_PROJECT_U8520:
		freq_table[3].frequency = 1150000;
		break;
	default:
		break;
	}
	#endif /* CONFIG_DB8500_LIVEOPP */
}

static bool initialized;

static void __init dbx500_cpufreq_early_init(void)
{
	#ifdef CONFIG_DB8500_LIVEOPP
	int i;
	#endif /* CONFIG_DB8500_LIVEOPP */
	if (cpu_is_u5500()) {
		freq_table = db5500_freq_table;
		idx2opp = db5500_idx2opp;
		freq_table_len = ARRAY_SIZE(db5500_freq_table);
	} else if (cpu_is_u8500()) {
	#ifdef CONFIG_DB8500_LIVEOPP
		freq_table = db8500_freq_table;
		dbx500_cpufreq_init_maxopp_freq();
		freq_table_len = ARRAY_SIZE(db8500_freq_table);
		for (i = 0; i < ARRAY_SIZE(liveopp_arm); i++)
			freq_table_len--;
	#else /* CONFIG_DB8500_LIVEOPP */
		freq_table = db8500_freq_table;
		idx2opp = db8500_idx2opp;
		dbx500_cpufreq_init_maxopp_freq();
		freq_table_len = ARRAY_SIZE(db8500_freq_table);
		if (!prcmu_has_arm_maxopp())
			freq_table_len--;
	#endif /* CONFIG_DB8500_LIVEOPP */
	} else {
		ux500_unknown_soc();
	}
	initialized = true;
}

/*
 * This is called from localtimer initialization, via the clk_get_rate() for
 * the smp_twd clock.  This is way before cpufreq is initialized.
 */
unsigned long dbx500_cpufreq_getfreq(void)
{
	if (!initialized)
		dbx500_cpufreq_early_init();

	return dbx500_cpufreq_getspeed(0) * 1000;
}

static int __cpuinit dbx500_cpufreq_init(struct cpufreq_policy *policy)
{
	int res;
	int i;

	/* get policy fields based on the table */
	res = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (!res)
		cpufreq_frequency_table_get_attr(freq_table, policy->cpu);
	else {
		pr_err("dbx500-cpufreq : Failed to read policy table\n");
		return res;
	}

	#ifdef CONFIG_DB8500_LIVEOPP
	policy->min = 200  * 1000;
	policy->max = 1000 * 1000;
	policy->cur = dbx500_cpufreq_getspeed(policy->cpu);
	#else
	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = dbx500_cpufreq_getspeed(policy->cpu);
	#endif

	for (i = 0; freq_table[i].frequency != policy->cur; i++)
		;

	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

	/*
	 * FIXME : Need to take time measurement across the target()
	 *	   function with no/some/all drivers in the notification
	 *	   list.
	 */
	#ifdef CONFIG_DB8500_LIVEOPP
	policy->cpuinfo.transition_latency = 30 * 1000; /* in ns */
	#else
	policy->cpuinfo.transition_latency = 20 * 1000; /* in ns */
	#endif

	/* policy sharing between dual CPUs */
	cpumask_copy(policy->cpus, &cpu_present_map);

	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;

	return 0;
}

static struct cpufreq_driver dbx500_cpufreq_driver = {
	.flags  = CPUFREQ_STICKY,
	.verify = dbx500_cpufreq_verify_speed,
	.target = dbx500_cpufreq_target,
	.get    = dbx500_cpufreq_getspeed,
	.init   = dbx500_cpufreq_init,
	.name   = "DBX500",
	.attr   = dbx500_cpufreq_attr,
};

#ifdef CONFIG_DB8500_LIVEOPP
static int __init late(void)
{
	int ret;
	liveopp_kobject = kobject_create_and_add("liveopp", kernel_kobj);
	if (!liveopp_kobject) {
		pr_err("[LiveOPP] Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(liveopp_kobject, &liveopp_interface_group);
	if (ret) {
		kobject_put(liveopp_kobject);
	}
	pr_info("[LiveOPP] Initialized: v%s\n", LiveOPP_VER);

	return 0;
}
late_initcall(late);
#endif /* CONFIG_DB8500_LIVEOPP */

static int __init dbx500_cpufreq_register(void)
{
	int i;

	if (!initialized)
		dbx500_cpufreq_early_init();

	pr_info("dbx500-cpufreq : Available frequencies:\n");

	for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
		pr_info("  %d Mhz\n", freq_table[i].frequency / 1000);

	return cpufreq_register_driver(&dbx500_cpufreq_driver);
}
device_initcall(dbx500_cpufreq_register);
