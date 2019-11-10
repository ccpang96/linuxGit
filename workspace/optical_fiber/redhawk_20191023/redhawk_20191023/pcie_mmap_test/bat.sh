#!/bin/bash

for i in {0..200}
do
	echo "The loop index is $i"
	./pcie_test
	echo "The loop $i is over!"
	sleep 2
done

