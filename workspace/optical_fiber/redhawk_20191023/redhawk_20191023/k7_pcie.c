#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h>

#include "k7_pcie.h"

#define DevName     "k7"
#define ClassName   "class_test"
#define VendorID    0x10ee
#define DeviceID    0x7028

dm_t dm_whole;
#if 0
dm_t dm_whole_bak;  //yangbt add 20190312
static unsigned long rCount = 0; //yangbt add 20190312
#endif
struct class    *mem_class;

struct Pci_Test
{
    struct cdev     _cdev;
    dev_t       devno;
    struct pci_dev  *dev;
    char        msi_enabled;
}*pci_test;

int hexdump(char *name, char *buf, int len)
{
	int i, count;
	unsigned int *p;
	count = len / 32;
	count += 1;
	printk(KERN_ERR "hexdump %s mem:0x%lx len:%d\n", name, (unsigned long)virt_to_phys(buf), len);
	for (i = 0; i < count; i++) {
		p = (unsigned int *)(buf + i * 32);
		printk(KERN_ERR
		       "mem[0x%04x] 0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n",
		       i * 32, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
	}
	return 0;
}

int dmalloc(dm_t *dm, int size)
{
	struct device *dev = &(pci_test->dev->dev);
	
	if (NULL == dev) {
		printk(KERN_ERR "%s(NULL)\n", __func__);
		return -1;
	}
	if (dma_set_mask(dev, DMA_BIT_MASK(64))) {
		printk(KERN_ERR "dma_set_mask err\n");
		if (dma_set_coherent_mask(dev, DMA_BIT_MASK(64))) {
			printk(KERN_ERR "dma_set_coherent_mask err\n");
			return -1;
		}
	}

	dm->vir =
	    (unsigned long)dma_alloc_coherent(dev, size,
					      (dma_addr_t *)(&(dm->phy)), GFP_DMA);
	if (dm->vir == 0) {
		printk(KERN_ERR "dma_alloc_coherent err\n");
		return -1;
	}

	dm->size = size;
	//memset((unsigned char *)(dm->vir), 0x56, size);
	printk("dma_alloc_coherent(%d) 0x%llx 0x%lx\n", size, dm->vir, dm->phy);
	return 0;
}

int dmfree(dm_t *dm)
{
	struct device *dev = &(pci_test->dev->dev);
	if (NULL == dev) {
		printk(KERN_ERR "%s(NULL)\n", __func__);
		return -1;
	}
	printk("dma_free_coherent(%d,0x%llx,0x%lx)\n", dm->size, dm->vir, dm->phy);
	dma_free_coherent(dev, dm->size, (void *)(dm->vir), dm->phy);
	memset(dm, 0x00, sizeof(dm_t));
#if 0
	rCount = 0;
#endif
	return -1;
}

int hwcopy(unsigned char *dest, unsigned char *src, int len)
{
	int i;
	len = ALIGN_N_BYTE(len, 4);
	for (i = 0; i < (len / 4); i++) {
		*((unsigned int *)(dest)) = *((unsigned int *)(src));
		dest += 4;
		src += 4;
	}
	for (i = 0; i < (len % 4); i++) {
		*dest = *src;
		dest++;
		src++;
	}
	return 0;
}

static int Test_open(struct inode *inode,struct file *filp)
{
    return 0;
}

static int Test_release(struct inode *inode,struct file *filp)
{
    dmfree(&dm_whole);
    return 0;
}

ssize_t Test_read(struct file * filp, char __user * buffer, size_t count,
		  loff_t * offset)
{
	int ret = 0;
#if 0
	static unsigned char *buf;
#endif
	if (filp == NULL)
	{
		printk(KERN_ERR "%s(filp == NULL)\n", __func__);
		return 0;
	}

	if (buffer == NULL)
	{
		printk(KERN_ERR "%s(buffer == NULL)\n", __func__);
		return 0;
	}

#if 0
	//yangbt add 20190312
	if (rCount % 2 == 0)
	{
		buf    = (unsigned char *)(dm_whole.vir);
	}
	else
	{
		buf    = (unsigned char *)(dm_whole_bak.vir);  
	}
	rCount++;
	ret = copy_to_user(buffer, (char*)buf, count);
#endif
    //第一个是内核空间中的目的内存地址，第二个是内核空间的源地址，第三个是需要拷贝的数据长度。
	ret = copy_to_user(buffer, (char*)(dm_whole.vir), count); 

	return ret;
}

long Test_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	char *tmp;

	if (filp == NULL)
	{
		printk(KERN_ERR "%s(filp == NULL)\n", __func__);
		return 0;
	}
#if 0
	rCount = 0;
#endif

	printk(KERN_ERR "%s(%d)\n", __func__, cmd);
	switch(cmd)
	{
	case K7_PCIE_GET_DMA:
		dmalloc(&dm_whole, 1*1024*1024);
		tmp = (char*)(&dm_whole);
		ret = copy_to_user((char __user *)arg, tmp, sizeof(dm_whole));
		break;
#if 0
	//yangbt add 20190312
	case K7_PCIE_GET_DMA_BAK:
		dmalloc(&dm_whole_bak, 4*1024*1024);
		tmp = (char*)(&dm_whole_bak);
		ret = copy_to_user((char __user *)arg, tmp, sizeof(dm_whole_bak));
		break;
#endif
	default:
		break;
	}

	printk(KERN_ERR "copy_to_user finished\n");

	return ret;
}

static struct file_operations test_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = Test_ioctl,
	.read = Test_read,
	.open = Test_open,
	.release = Test_release,
};

//字符驱动
static int init_chrdev(struct Pci_Test *test_ptr)
{
    int result = alloc_chrdev_region(&test_ptr->devno, 0, 2, DevName);
    if (result < 0)
    {
        printk("Err:failed in alloc_chrdev_region!\n");
        return result;
    }

    mem_class = class_create(THIS_MODULE,ClassName);// /dev/ create devfile 
        if (IS_ERR(mem_class))
        {
        printk("Err:failed in creating class!\n");
    }
    device_create(mem_class,NULL,test_ptr->devno,NULL,DevName);

    cdev_init(&test_ptr->_cdev,&test_fops);
    test_ptr->_cdev.owner = THIS_MODULE;
    test_ptr->_cdev.ops = &test_fops;//Create Dev and file_operations Connected
    result = cdev_add(&test_ptr->_cdev,test_ptr->devno,1);
    return result;
}

//PCI驱动入口函数
static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int rc = 0;
    
    pci_test->dev = dev;
    pci_set_drvdata(dev, pci_test);
    //在这里创建字符设备驱动
    rc = init_chrdev(pci_test); 
        if (rc) {
            dev_err(&dev->dev, "init_chrdev() failed\n");
            return -1;
        }
	else
	{
		printk(KERN_INFO  "init_chrdev() successful\n");
	}

    rc = pci_enable_device(dev);
        if (rc) {
            dev_err(&dev->dev, "pci_enable_device() failed\n");
            return -1;
        } 
	else
	{
		printk(KERN_INFO  "pci_enable_device() successful\n");
	}

    rc = pci_request_regions(dev, DevName);
        if (rc) {
            dev_err(&dev->dev, "pci_request_regions() failed\n");
            return -1;
        }
	else
	{
		printk(KERN_INFO  "pci_request_regions() successful\n");
	}

        pci_set_master(dev);
        rc = pci_enable_msi(dev);
    if (rc) {
            dev_info(&dev->dev, "pci_enable_msi() failed\n");
            pci_test->msi_enabled = 0;
        } else {
            dev_info(&dev->dev, "pci_enable_msi() successful\n");
            pci_test->msi_enabled = 1;
    }

    return rc;
}

static void __exit pci_remove(struct pci_dev *dev)
{
	if (0 != mem_class)
	{
		device_destroy(mem_class,pci_test->devno);
		class_destroy(mem_class);
		mem_class = 0;
	}

	dmfree(&dm_whole);

    	pci_test = pci_get_drvdata(dev);
	cdev_del(&pci_test->_cdev);
	//unregister_chrdev_region(pci_test->dev, 1);
	unregister_chrdev_region(pci_test->devno, 1);
    	pci_disable_device(dev);

	if(pci_test) {
		if(pci_test->msi_enabled) {
				pci_disable_msi(dev);
				pci_test->msi_enabled = 0;
			}
	}

    	pci_release_regions(dev);
}

static struct pci_device_id pci_ids[] = {
    { PCI_DEVICE( VendorID, DeviceID) },
    { 0 }
};

static struct pci_driver driver_ops = {
    .name = DevName,
    .id_table = pci_ids,
    .probe = pci_probe,
    .remove = pci_remove,
};
//驱动模块入口函数
static int Test_init_module(void)
{
    int rc = 0;
    printk(KERN_INFO  "Enter Test_init_module\n");

    pci_test = kzalloc(sizeof(struct Pci_Test), GFP_KERNEL);
    //配对设备以及注册PCI驱动，如果找到对应设备调用PCI入口函数
    rc = pci_register_driver(&driver_ops);
	if (rc) {
		printk(KERN_ALERT  ": PCI driver registration failed\n");
	}
	else
	{
		printk(KERN_INFO  ": PCI driver registration successful\n");
	}

    return rc;
}

static void Test_exit_module(void)
{
    printk(KERN_INFO  "Enter Test_exit_module\n");
    pci_unregister_driver(&driver_ops);
    kfree(pci_test);
}
module_init(Test_init_module);
module_exit(Test_exit_module);
MODULE_AUTHOR(DevName);
MODULE_LICENSE("GPL");
