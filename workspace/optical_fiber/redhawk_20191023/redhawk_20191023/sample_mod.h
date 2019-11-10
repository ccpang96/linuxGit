
/* attach signal number (in ioctl arg) to device interrupt handler
 */
#define RCIM_ATTACH_SIGNAL       0x10

/* older kernel source does not contain the RCIM III definition
 */
#ifndef PCI_DEVICE_ID_RCIM_III
#define PCI_DEVICE_ID_RCIM_III	0x9271
#endif
