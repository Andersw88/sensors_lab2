set xrange [.6:2]
input_file = 'Dokumente/uni/ht15/sas/sensors_lab2/report/measurement/distance_values.csv'
plot input_file using 1:3 with lines title '40x40',\
     input_file using 1:7 with lines title '60x60',\
     input_file using 1:11 with lines title '80x80'

output_file = 'Dokumente/uni/ht15/sas/sensors_lab2/report/measurement/distance_values.eps'
set terminal postscript color
set output output_file
replot

set term wxt