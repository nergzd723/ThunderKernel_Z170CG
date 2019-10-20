/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;

u8 empty_slot = 0xff;           /* bit[n] = 0, means there is no task
                                 * which adj is great or equal to the binded
                                 * lowmem_adj[x] */
unsigned long jiffies_empty_slot = INITIAL_JIFFIES;



static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

DEFINE_MUTEX(profile_mutex);

struct profile
{
    unsigned long timeout;
    unsigned long nr_to_scan;   /* summation of sc->nr_to_scan in time period */
    int fire;                   /* numbers of call for LMK shrinker */
    int select;                 /* numbers of killing */
    int reason_adj_max;         /* escape reason: min_score_adj == OOM_SCORE_ADJ_MAX + 1 */
    int reason_tif_memdie;      /* escape reason: selecting a teriminating task */
    int reason_nop;             /* escape reason: no operation + selected */
    int ofree;
    int ofile;
    unsigned long pressure;     /* pressure for previous profile */
};

static struct profile g_lmk_profile = {
        .timeout = INITIAL_JIFFIES,
};

#define LMK_PROFILE_INTERVAL (0.25)
//Lower bound for cache = 56MB
#define LMK_LOW_BOUND_CACHE 14336
#define LMK_LOW_BOUND_NR 20
#define LMK_LOW_BOUND_FIRE ((unsigned long)(8000*LMK_PROFILE_INTERVAL))

/* standardize memory pressure: 0~10 */
#define standardize_lmk_pressure(p)                         \
    do {                                                    \
        p = (p/(LMK_LOW_BOUND_NR*LMK_LOW_BOUND_FIRE/10));   \
        if (p > 10)                                         \
            p = 10;                                         \
    } while(0)

#define profile_reset(p)                               \
    do {                                               \
        memset(p, 0, sizeof(struct profile));          \
        p->timeout = jiffies + (unsigned long)(LMK_PROFILE_INTERVAL*HZ); \
    } while(0)

static int profile_show(struct profile *p, u8 task_bitmap_mask)
{
    int retv = 0;
    int pressure;
    unsigned long sp;

    if (p->timeout == INITIAL_JIFFIES) {
        lowmem_print(2, "totalreserve_pages=%lu, LMK_LOW_BOUND_FIRE=%lu\n", totalreserve_pages, LMK_LOW_BOUND_FIRE);
        profile_reset(p);
        return 0;
    }

    task_bitmap_mask >>= 1;     /* excludes the most important processes. */
    mutex_lock(&profile_mutex);
    if (p->fire > 0 && time_after(jiffies, p->timeout)) {
        pressure = p->nr_to_scan;
        lowmem_print(2, "efficicy: kill=%d, shrink=%d, adj_max=%d, memdie=%d, nop=%d, ofree=%d, ofile=%d, pressure=%d\n",
                     p->select, p->fire, p->reason_adj_max, p->reason_tif_memdie,
                     p->reason_nop - p->select,
                     p->ofree/p->fire, p->ofile/p->fire, pressure);

        if (p->select == 0) {
                /* No killed process in previous profiling */
                sp = pressure;
                standardize_lmk_pressure(sp);
                switch (sp) {
                case 10:
                        /* Highest pressure score.
                         * Basically, we want to selection some one to be killed. */
                        retv = 1;

                        if ((empty_slot & task_bitmap_mask) == 0 &&
                            (p->ofile/p->fire > LMK_LOW_BOUND_CACHE))
                                /* All other than foreground app has been killed
                                 * Force selection when cache pages is low enough.
                                 * This prevent killing the FG app soon, but it may
                                 * cause system lags because low memory. */
                                retv = 0;
                        break;
                case 9:
                case 8:
                case 7:
                case 6:
                        /* Higher pressure score.
                         * Kill some one that is not a FG process */
                        if (empty_slot & task_bitmap_mask)
                                retv = 1;
                        break;
                default:;
                }
                if (retv)
                        lowmem_print(1, "force selecting(p:%u, ofile: %u)\n", sp, p->ofile/p->fire);
        }
        profile_reset(p);
        p->pressure = pressure;
    }
    mutex_unlock(&profile_mutex);
    return retv;
}



static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
	return global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	struct task_struct *selected_qos = NULL;
        unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0, selected_qos_tasksize;
        short selected_oom_score_adj, selected_qos_oom_score_adj;
        bool selected_mark_as_prefer_kill = 0;
        int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages;
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						total_swapcache_pages();
        int cma_free = global_page_state(NR_FREE_CMA_PAGES);

	if (other_free > cma_free)
		other_free -= cma_free;
        int force_select = 0;
        u8 task_bitmap = 0;          /* bitmap indicates the associated adj category is empty or not */
        u8 task_bitmap_mask = 0;
        u8 candidate_slot = 0;

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
        switch (array_size) {
	case 6:
		task_bitmap_mask = 0x3f;
		break;
	case 4:
		task_bitmap_mask = 0x0f;
		break;
	default:
		pr_info("array size of lowmem_minfree_size should be 4 or 6\n");
		WARN_ON(1);
	}

	force_select = profile_show(&g_lmk_profile, task_bitmap_mask);
	if (force_select > 0) {
		min_score_adj = lowmem_adj[0];
	}
	else {
		candidate_slot = task_bitmap_mask;
		for (i = 0; i < array_size; i++) {
			minfree = lowmem_minfree[i];
			if (other_free < minfree && other_file < minfree) {
				min_score_adj = lowmem_adj[i];
				break;
			}
		candidate_slot >>= 1;
		}

		if (min_score_adj < OOM_SCORE_ADJ_MAX + 1) {
			mutex_lock(&profile_mutex);
			/* want to select something
			* We check whether the empty timestamp is out of date. */
			if ((empty_slot & candidate_slot) == 0 &&
				jiffies_empty_slot != INITIAL_JIFFIES &&
				time_before_eq(jiffies, jiffies_empty_slot)) {
				lowmem_print(3, "skip selection (adj: %d, time remains: %u ms)\n",
					min_score_adj, jiffies_to_msecs(jiffies_empty_slot - jiffies));
				min_score_adj = OOM_SCORE_ADJ_MAX + 1;
			}
			mutex_unlock(&profile_mutex);
		}
	}
	g_lmk_profile.fire++;
	g_lmk_profile.ofree += other_free;
	g_lmk_profile.ofile += other_file;

        if (sc->nr_to_scan > 0) {
		g_lmk_profile.nr_to_scan += sc->nr_to_scan;

 		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %hd\n",
 				sc->nr_to_scan, sc->gfp_mask, other_free,
 				other_file, min_score_adj);
	}
	if (!force_select && min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
                lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);
                g_lmk_profile.reason_adj_max++;
		return 0;
	}

	selected_oom_score_adj = min_score_adj;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;
		short oom_qos;
		unsigned long task_life_time;
		struct timespec delta;
		short oom_qos_endurance;
		bool oom_qos_prefer_kill;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		if (test_tsk_thread_flag(p, TIF_MEMDIE) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
			task_unlock(p);
			rcu_read_unlock();
			/* if pressure is high enough,
			 * select the other one even if someone is teriminting. */
			if (force_select > 0) continue;
			g_lmk_profile.reason_tif_memdie++;
			return 0;
		}
		oom_score_adj = p->signal->oom_score_adj;
		oom_qos = p->signal->oom_qos;
		get_oom_qos_endurance(oom_qos_endurance, oom_qos);
		oom_qos_prefer_kill = is_prefer_to_kill(oom_qos);
		/* mark the associated category is empty or not
		 * The highest value of task_bitmap is 'task_bitmap_mask',
		 * means all slots are not empty. */
		for (i = array_size - 1; i >= 0 && (task_bitmap & task_bitmap_mask) < task_bitmap_mask;i--) {
			if (oom_score_adj >= lowmem_adj[i]) {
				task_bitmap |= 1 << (array_size - 1 - i);
				break;
			}
		}

		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}
		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if(!strcmp(p->comm,"FinalizerDaemon")){
                    lowmem_print(3, "FinalizerDaemon, not killed !!!!\n");
                    continue;
                }

                if(!strcmp(p->comm,"GC")){
                    lowmem_print(3, "GC, not killed !!!!\n");
                    continue;
                }

                /* Exame QOS level of this task */
                if (oom_qos_endurance > 0 && oom_score_adj >= 470) {
                        get_monotonic_boottime(&delta);
                        delta = timespec_sub(delta, p->real_start_time);
                        task_life_time = timespec_to_jiffies(&delta);
                        if (task_life_time < oom_qos_endurance*HZ) {
                                lowmem_print(3, "skip '%s' (%d), adj %hd, for QOS. lifeTime(ms) %u, \n",
                                             p->comm, p->pid, oom_score_adj,
                                             jiffies_to_msecs(task_life_time));
                                /* This is a candicate for killing. */
                                if (selected_qos) {
                                        if (oom_score_adj < selected_qos_oom_score_adj)
                                                continue;
                                        if (oom_score_adj == selected_qos_oom_score_adj &&
                                            tasksize <= selected_qos_tasksize) {
                                                continue;
                                        }
                                }
                                selected_qos = p;
                                selected_qos_tasksize = tasksize;
                                selected_qos_oom_score_adj = oom_score_adj;
                                continue;
                        }
                }
                if (selected) {
                        if (oom_score_adj < selected_oom_score_adj)
                                continue;
                        //For foreground and previous app, choice app allocating lower memory
                        if (oom_score_adj == 0 || oom_score_adj == 411) {
                                if (oom_score_adj == selected_oom_score_adj &&
                                    tasksize > selected_tasksize)
                                        continue;
                        } else {
                                if (oom_score_adj == selected_oom_score_adj) {
                                        /* the same adj level */
                                        if (selected_mark_as_prefer_kill &&
                                            !oom_qos_prefer_kill)
                                                continue;
                                        if (tasksize <= selected_tasksize) {
                                                continue;
                                        }
                                }
                        }
                }
                selected = p;
                selected_tasksize = tasksize;
                selected_oom_score_adj = oom_score_adj;
                selected_mark_as_prefer_kill = oom_qos_prefer_kill;
                lowmem_print(3, "select '%s' (%d), adj %hd, size %d, to kill\n",
                             p->comm, p->pid, oom_score_adj, tasksize);
        }
	/* make a decision between selected and selected_qos */
	if (selected && selected_qos && selected_oom_score_adj < 470) {
		/* Previous app or more critical */
		if (selected_qos_oom_score_adj > selected_oom_score_adj ||
			(selected_qos_oom_score_adj == selected_oom_score_adj &&
			selected_qos_tasksize > selected_tasksize)) {
			selected = selected_qos;
			selected_tasksize = selected_qos_tasksize;
			selected_oom_score_adj = selected_qos_oom_score_adj;
                }
	}
	if (selected) {
		lowmem_print(1, "Killing '%s' (%d), adj %hd,\n" \
				"   to free %ldkB on behalf of '%s' (%d) because\n" \
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
				"   Free memory is %ldkB above reserved, force %d\n",
			     selected->comm, selected->pid,
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     other_file * (long)(PAGE_SIZE / 1024),
			     minfree * (long)(PAGE_SIZE / 1024),
			     min_score_adj,
			     other_free * (long)(PAGE_SIZE / 1024), force_select);
		lowmem_deathpending_timeout = jiffies + HZ;
		send_sig(SIGKILL, selected, 0);
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem += selected_tasksize;
		g_lmk_profile.select++;
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	g_lmk_profile.reason_nop++;
	rcu_read_unlock();

	/* The highest value of task_bitmap is 'task_bitmap_mask',
	 * means all slots are not empty.
	 * So, we don't need to update timestamps. */
	if ((task_bitmap & task_bitmap_mask) >= task_bitmap_mask)
		return rem;

	/* update timestaps(future time) to record the empty slot */
	mutex_lock(&profile_mutex);
	empty_slot = task_bitmap | ~(task_bitmap_mask);
	jiffies_empty_slot = jiffies + msecs_to_jiffies(100);
	mutex_unlock(&profile_mutex);
	lowmem_print(4, "task_bitmap = %x\n", task_bitmap);
	return rem;
}

static int lmk_proc_show(struct seq_file *m, void *v)
{
    unsigned long p;

    mutex_lock(&profile_mutex);
    /* out of date? */
    if (time_after(jiffies, g_lmk_profile.timeout + HZ) ||
        g_lmk_profile.timeout == INITIAL_JIFFIES) {
        p = 0;
    } else {
        p = g_lmk_profile.pressure;
        standardize_lmk_pressure(p);
    }
    mutex_unlock(&profile_mutex);
    seq_printf(m, "pressure:       %8lu\n", p);
    seq_printf(m, "reserve_KB:     %8lu\n", (global_page_state(NR_FREE_CMA_PAGES) + totalreserve_pages)*(PAGE_SIZE / 1024));
    return 0;
}

static int lmk_proc_open(struct inode *inode, struct file *file)
{
       return single_open(file, lmk_proc_show, NULL);
}

static const struct file_operations lmk_proc_fops = {
    .owner = THIS_MODULE,
    .open = lmk_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
 
    proc_create("lmkinfo", 0444, NULL, &lmk_proc_fops);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

