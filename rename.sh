#/bin/bash
for x in *.ppm; do export FDATE=`echo $x|cut -f 2 -d -`; mv $x frame_`date -d@${FDATE} -I'ns' -u|sed s/+0000//|sed s/-//g|sed s/://g|sed s/,/./|sed s/000$//`_rgb.ppm; done
for x in *.pgm; do export FDATE=`echo $x|cut -f 2 -d -`; mv $x frame_`date -d@${FDATE} -I'ns' -u|sed s/+0000//|sed s/-//g|sed s/://g|sed s/,/./|sed s/000$//`_depth.pgm; done
for x in *.ppm; do convert $x `echo $x|sed s/ppm/png/`; done
for x in *.pgm; do convert $x `echo $x|sed s/pgm/png/`; done
rm *.ppm *.pgm