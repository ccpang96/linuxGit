KERNEL_DIR = /lib/modules/`uname -r`/build

obj-m   := k7_pcie.o

default:
	make -C ${KERNEL_DIR} SUBDIRS=`pwd` REDHAWKFLAVOR=`cat /proc/ccur/flavor`  modules

config:
	cd $(KERNEL_DIR); ./ccur-config -n -c

clean:
	rm -f $(obj-m) k7_pcie.mod.*  k7_pcie.ko k7_pcie.o Module.* modules.* .sample*
	rm -fr .tmp_versions

