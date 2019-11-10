#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>

#include "sample_mod.h"

static const char *devname = "/dev/sample_mod";
static int nr_interrupts = 0;
static int quit = 0;

void interrupt_handler (int signum)
{
	nr_interrupts++;
	
	if ((nr_interrupts % 100) == 0) {
		printf (".");
		fflush(stdout);
	}
	if ((nr_interrupts % 1000) == 0)
		printf (" %d interrupts\n", nr_interrupts);
} 

void ctrl_c_handler (int signum)
{
	quit++;
} 

int main()
{
	int fd;
	struct sigaction intr_sig = { .sa_handler = interrupt_handler };
	struct sigaction ctrl_c_sig = { .sa_handler = ctrl_c_handler };

	sigaction (SIGUSR1, &intr_sig, NULL); //改变与信号相关的操作
	sigaction (SIGINT, &ctrl_c_sig, NULL);

	if ((fd = open (devname, O_RDWR)) == -1 ) {
		perror ("open");
		exit(1);
	}

	if (ioctl (fd, RCIM_ATTACH_SIGNAL, SIGUSR1) == -1) {
		perror ("ioctl");
		exit(1);
	}

	printf ("waiting for signals...\n");
	while (! quit)
		pause();

	printf ("\nhandled %d interrupts\n", nr_interrupts);
	close(fd);
	exit(0);
}


