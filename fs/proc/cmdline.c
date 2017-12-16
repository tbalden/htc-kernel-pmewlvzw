#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#if 1
#include <linux/slab.h>
#include <linux/spinlock.h>

static bool done = false;

static DEFINE_SPINLOCK(show_lock);

static const char *replace =      "androidboot.verifiedbootstate=";
static char *replace_with =       "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa=";

static bool magisk = true;

extern bool is_magisk(void);
extern bool is_magisk_sync(void);
extern void init_magisk(void);
#endif

static int cmdline_proc_show(struct seq_file *m, void *v)
{
#if 1
	spin_lock(&show_lock);
	if (done) {
	} else {
		magisk = is_magisk();
		if (!magisk) {
			char *tmp = saved_command_line;
			tmp = strstr(tmp,replace);
			if (tmp) {
				while ((*replace_with)!='\0') {
					*tmp = *replace_with;
					replace_with++;
					tmp++;
				}
			}
		}
		done = true;
	}
	spin_unlock(&show_lock);
#endif
	seq_printf(m, "%s\n", saved_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cmdline_init(void)
{
#if 1
	init_magisk();
#endif
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
