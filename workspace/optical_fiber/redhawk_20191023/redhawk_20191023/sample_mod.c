#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>
#include <linux/pci_ids.h>
#include "sample_mod.h" 

// for put_pid()
MODULE_LICENSE("GPL"); //

#define        RCIM_RTC_1MICRO         (0<<20)    //1 microsec resolution
#define        RCIM_RTC_START          0x10000    //start bit
#define        RCIM_RTC_REPEAT         0x20000    //repeat when count reaches 0
#define        RTC_TESTVAL             0x00002710 //number of  microseconds
#define        ALL_INT_MASK            0xff000f0f //mask of all RCIM interrupts
#define        RCIM_REG_RTC0           0x00000001 //mask of rtc 0 interrupt

typedef struct {
	int             flag;
	int             nopens;
	int             signal_num;  //signal to generate, 0=disabled
	struct pid *    signal_pid;  //pid of process to send signo to
} rtc_info_type;

typedef struct {                //Partial Register Layout of RCIM Memory Space
	volatile u_int32_t      pad0[4];
	volatile u_int32_t      enable;
	volatile u_int32_t      pad1[3];
	volatile u_int32_t      request;
	volatile u_int32_t      pad2[3];
	volatile u_int32_t      clear;
	volatile u_int32_t      pad3[3];
	volatile u_int32_t      arm;
	volatile u_int32_t      pad4[0x7ef];
	volatile u_int32_t      rtc0_control;
	volatile u_int32_t      pad5[3];
	volatile u_int32_t      rtc0_timer;
} rtc_mem_t;        

typedef struct {                //PLX9080 Local Configuration Register Mapping
	volatile u_int32_t      las0rr;
	volatile u_int32_t      las0ba;
	volatile u_int32_t      marbr;
	volatile u_int32_t      bigend;
	volatile u_int32_t      pad0[2];
	volatile u_int32_t      lbrd0;
	volatile u_int32_t      pad1[19];
	volatile u_int32_t      intcsr;
	volatile u_int32_t      cntrl;
} plx_mem_t;
	
static int             major_num = 0;
static int             sample_mod_irq;
static struct pci_dev  *dev = NULL;
static rtc_info_type   rtc_info;
static unsigned long   rcim_mem_base;
static unsigned long   plx_mem_base;
static u_int32_t       rcim_mem_size;
static u_int32_t       plx_mem_size;
static plx_mem_t       *bd_regs;       //mapped address of plx9080 memory
static rtc_mem_t       *bd_rcim_regs;  //mapped address of rcim memory

volatile u_int32_t	arm, enable;   //save off orignal values

#ifdef __LITTLE_ENDIAN
static unsigned int rcim_plx9080_endian  = 4;
#endif
#ifdef __BIG_ENDIAN
static unsigned int rcim_plx9080_endian  = 0;
#endif

int  rcim_rtc_open(struct inode *, struct file *);
int  rcim_rtc_close(struct inode *, struct file *);
long rcim_rtc_ioctl(struct file *, unsigned int, unsigned long);
irqreturn_t rcim_intr(int,void *);

struct file_operations rcim_rtc_fops = {
	open:		rcim_rtc_open,
	release:	rcim_rtc_close,
	unlocked_ioctl:	rcim_rtc_ioctl,
};

/* "init" routine. The module registers itself, prepares the pci device
   (RCIM's real time clock 0) for use, installs its interrupt handler,
   and initializes the internal data structure(rtc_info_type rtc_info).  */

int sample_mod_init_module(void)
{
	int res = 0, ret, rcim_type = 1;

	if ((res = register_chrdev(major_num,"sample_mod",&rcim_rtc_fops)) < 0)
		return res;
	major_num = res; 

	// find rcim board (look for RCIM III, RCIM II, RCIM I, and finally RCIM I old rev)
	dev = pci_get_device(PCI_VENDOR_ID_CONCURRENT, PCI_DEVICE_ID_RCIM_III, dev);
	if (dev)
		rcim_type = 3;
	if (dev == NULL) {                                   //try another id
		dev = pci_get_device(PCI_VENDOR_ID_CONCURRENT, PCI_DEVICE_ID_RCIM_II, dev);
		if (dev)
			rcim_type = 2;
	}
	if (dev == NULL) {                                   //try another id
		 dev = pci_get_device(PCI_VENDOR_ID_CONCURRENT_OLD, PCI_DEVICE_ID_RCIM, dev);
	}
	if (dev == NULL) {                                   //try another id
		 dev = pci_get_device(PCI_VENDOR_ID_CONCURRENT_OLD, PCI_DEVICE_ID_RCIM_OLD, dev);
	}
	if (dev == NULL) {             //no rcim board, just clean up and exit
		ret = -ENODEV;
		goto failed;
	}

	// map in hardware registers

	plx_mem_base = pci_resource_start (dev, 0);
	plx_mem_size = pci_resource_len (dev, 0);
	if ((bd_regs = ioremap_nocache(plx_mem_base, plx_mem_size)) == NULL) {
		ret = -ENOMEM;
		goto failed;
	}
	if (rcim_type == 3)
                writel(0xffff8000, &bd_regs->las0rr);  // 32k of addr space
	else
		writel(0xffffc000, &bd_regs->las0rr); // 16k of addr space
	writel(0x1, &bd_regs->las0ba); // enable local bus, remap to addr 0

	if (!pci_resource_start(dev, 2)) {
		// should have been initialized
		// attempt to set up register area
		dev->resource[2].start = 0;
		dev->resource[2].end = 0x4000-1;
		dev->resource[2].flags = PCI_BASE_ADDRESS_SPACE_MEMORY | IORESOURCE_MEM;
		dev->resource[2].name = "RCIM regs";
		if (pci_assign_resource(dev, 2)) {
			ret = -EBUSY;
			goto failed1;
		}
	}
	rcim_mem_base = pci_resource_start(dev, 2);
	rcim_mem_size = pci_resource_len(dev, 2);
	if ((bd_rcim_regs = ioremap_nocache(rcim_mem_base, rcim_mem_size)) == NULL)
	{
		ret = -ENOMEM;
		goto failed1;
	}
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_2, rcim_mem_base);

	//setup some of the control regs
	writel(0x00200000, &bd_regs->marbr);
	if (rcim_type == 3)
		rcim_plx9080_endian ^= 4;
	writel(rcim_plx9080_endian, &bd_regs->bigend);
	writel(0xf00301c3, &bd_regs->lbrd0);
	writel(0x00010d03, &bd_regs->intcsr);

	if (pci_enable_device(dev)) {
		ret = -ENODEV;
		goto failed2;
	}
	arm = readl(&bd_rcim_regs->arm);
	enable = readl(&bd_rcim_regs->enable);

	writel(0, &bd_rcim_regs->arm);
	writel(0, &bd_rcim_regs->enable);
	writel(0, &bd_rcim_regs->request);
	writel(ALL_INT_MASK, &bd_rcim_regs->clear);

	sample_mod_irq = dev->irq;
	res = request_irq(sample_mod_irq, rcim_intr, IRQF_SHARED, "sample_mod", &rtc_info);
	if (res) {
		ret = -ENODEV;
		goto failed3;
	}

	rtc_info.flag = 1;                   // rtc 0 is now allocated
	rtc_info.nopens = 0;
	rtc_info.signal_num = 0;
	rtc_info.signal_pid = 0;
	return 0;

failed3:
	writel(arm, &bd_rcim_regs->arm);
	writel(enable, &bd_rcim_regs->enable);
	pci_disable_device(dev);
failed2:
	iounmap(bd_rcim_regs);
failed1:
	iounmap(bd_regs);
failed:
	unregister_chrdev(major_num,"sample_mod");
	return ret;
}

/* cleanup function used to undo any registrations and unload the module */

void sample_mod_cleanup_module(void)
{
	unregister_chrdev(major_num,"sample_mod");
	iounmap(bd_rcim_regs);
	iounmap(bd_regs);
	free_irq(sample_mod_irq, &rtc_info);
	writel(arm, &bd_rcim_regs->arm);
	writel(enable, &bd_rcim_regs->enable);
	pci_disable_device(dev);
}

module_init(sample_mod_init_module);
module_exit(sample_mod_cleanup_module);


/* open method. Initializes the device and prepares it for generating interrupts
   at predefined time periods - 10000 microseconds. Interrupts associated with 
   rtc 0 are enabled and then the device is programmed to count from 10000 to 0
   with a resolution of 1 microsecond.Finally, the clock is started.  */
  
int rcim_rtc_open(struct inode *inode, struct file *filep)
{
	u_int32_t val;
	
	if (rtc_info.nopens > 0) {
		printk(KERN_ERR "You can only open the device once.\n");
		return -ENXIO;
	}
	rtc_info.nopens++;
	if (!rtc_info.flag)
		return -ENXIO;
	writel(0, &bd_rcim_regs->request); 
	writel(ALL_INT_MASK, &bd_rcim_regs->clear);
	writel(RCIM_REG_RTC0, &bd_rcim_regs->arm);
	writel(RCIM_REG_RTC0, &bd_rcim_regs->enable);
	writel(RTC_TESTVAL, &bd_rcim_regs->rtc0_timer);//rtc data reg
	val = RCIM_RTC_1MICRO | RCIM_RTC_START|RCIM_RTC_REPEAT;
	writel(val, &bd_rcim_regs->rtc0_control);
	return 0;
}


/*  release method. Real time clock 0 is shut down. The count value is reset
    to 0 and the clock is stopped. Any signals associated with certain processes    that have requested notification from the module are cleared. */
 
int rcim_rtc_close(struct inode *inode,struct file *filep)
{
	if (!rtc_info.flag)
		return (-ENXIO);
	rtc_info.nopens--;
	if(rtc_info.nopens == 0) {
		writel(~RCIM_RTC_START, &bd_rcim_regs->rtc0_control);
		writel(0, &bd_rcim_regs->rtc0_timer);
		rtc_info.signal_num = 0;
		put_pid(rtc_info.signal_pid);
		rtc_info.signal_pid = 0;
	}
	return 0;
}


/* ioctl method. Applications would use this method to request notification from
   the module when rtc 0 generates an interrupt.The method of notifying the user
   process is to send a signal to it. The calling user space program
   specifies the signal number it wishes to receive from the module. The driver
   remembers the process id associated with the requested signal number by
   using the "current" structure. The "signal/process id" pair is stored in the
   module's "rtc_info" structure.  */

long rcim_rtc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	if (!rtc_info.flag)
		return (-ENXIO);

	switch (cmd) 
	{
		// Attach signal to the specified rtc interrupt
		case RCIM_ATTACH_SIGNAL:
			rtc_info.signal_num = (int)arg;
			put_pid(rtc_info.signal_pid);
			rtc_info.signal_pid = get_pid(task_pid(current));
			break;

		default:
			return (-EINVAL);
	}
	return (0);
}


/* interrupt handler. When an interrupt is received from rtc 0, this 
   interrupt service routine determines whether to send a signal to a process
   that has requested it. If there is a registered "process id/signal number"
   pair in the "rtc_info" structure, the specified signal is sent to the
   corresponding process using the function "kill_proc()". */

irqreturn_t rcim_intr(int irq, void *dev_id)
{
	u_int32_t isr;

	// shared interrupt may not be for us
	if (&rtc_info != (rtc_info_type *)dev_id)
		return IRQ_NONE;

	isr = readl(&bd_rcim_regs->request);
	writel(0, &bd_rcim_regs->request);
	writel(ALL_INT_MASK, &bd_rcim_regs->clear);

	/* Use isr to determine whether the interrupt was generated by rtc 0
	 * only if the "rcim" module is not built into the kernel. If "rcim"
	 * is active, its interrupt handler would have cleared "request"
	 * register by the time we get here.
	 */
//	if (isr & RCIM_REG_RTC0) {
		// Send signal to user process if requested 
		if (rtc_info.signal_num && rtc_info.signal_pid &&
			(kill_pid(rtc_info.signal_pid, rtc_info.signal_num, 1) == -ESRCH))
		{
			rtc_info.signal_pid = 0;
		}
//	}
	return IRQ_HANDLED;
}
