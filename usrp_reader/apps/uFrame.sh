#!/bin/sh

for i in $(seq 0 5)
do
        #echo $(expr $i \* 500 + 15000)
	if test $i -le 2
	then
        	sudo GR_SCHEDULER=STS nice -n -20 ./single_tag_exp.py $i $(expr $i \* -10000 + 30000)
	else
		sudo GR_SCHEDULER=STS nice -n -20 ./single_tag_exp.py $i $(expr $(expr $i - 3) \* -10000 + 30000)
	fi
done
