set xrange [.6:1.8]
#input_file = 'Dokumente/uni/ht15/sas/sensors_lab2/report/measurement/distance_values.csv'
input_file = '../../sensors_lab2_node/sensors_lab2_bag_to_csv/data.csv'
#input_file = 'filter_mean.csv'

set style line 101 lc rgb '#808080' lt 1 lw 1
set style line 1 linewidth 5
set border 3 front ls 101
set tics nomirror out scale 0.75
set format '%g'

 bilat_40_variance = 20
 bilat_60_variance = 21
 bilat_80_variance = 22
 gauss_40_variance = 23
 gauss_60_variance = 24
 gauss_80_variance = 25
 mean10_40_variance = 26
 mean10_60_variance = 27
 mean10_80_variance = 28
 median_40_variance = 29
 median_60_variance = 30
 median_80_variance = 31
 median10_40_variance = 32
 median10_60_variance = 33
 median10_80_variance = 34
 org_40_variance = 35
 org_60_variance = 36
 org_80_variance = 37

 bilat_40_error = 56
 bilat_60_error = 57
 bilat_80_error = 58
 gauss_40_error = 59
 gauss_60_error = 60
 gauss_80_error = 61
 mean10_40_error = 62
 mean10_60_error = 63
 mean10_80_error = 64
 median_40_error = 65
 median_60_error = 66
 median_80_error = 67
 median10_40_error = 68
 median10_60_error = 69
 median10_80_error = 70
 org_40_error = 71
 org_60_error = 72
 org_80_error = 73


set terminal postscript color linewidth 3 font 'Helvetica,18'

output_file = '../figures/mean_bilat_error.eps'
set output output_file
plot input_file using 1:bilat_40_error with lines title '40x40',\
     input_file using 1:bilat_60_error with lines title '60x60',\
     input_file using 1:bilat_80_error with lines title '80x80'

output_file = '../figures/mean_bilat_variance.eps'
set output output_file
plot input_file using 1:bilat_40_variance with lines title '40x40',\
     input_file using 1:bilat_60_variance with lines title '60x60',\
     input_file using 1:bilat_80_variance with lines title '80x80'



output_file = '../figures/mean_gauss_error.eps'
set output output_file
plot input_file using 1:gauss_40_error with lines title '40x40',\
     input_file using 1:gauss_60_error with lines title '60x60',\
     input_file using 1:gauss_80_error with lines title '80x80'

output_file = '../figures/mean_gauss_variance.eps'
set output output_file
plot input_file using 1:gauss_40_variance with lines title '40x40',\
     input_file using 1:gauss_60_variance with lines title '60x60',\
     input_file using 1:gauss_80_variance with lines title '80x80'



output_file = '../figures/mean_mean10_error.eps'
set output output_file
plot input_file using 1:mean10_40_error with lines title '40x40',\
     input_file using 1:mean10_60_error with lines title '60x60',\
     input_file using 1:mean10_80_error with lines title '80x80'

output_file = '../figures/mean_mean10_variance.eps'
set output output_file
plot input_file using 1:mean10_40_variance with lines title '40x40',\
     input_file using 1:mean10_60_variance with lines title '60x60',\
     input_file using 1:mean10_80_variance with lines title '80x80'


output_file = '../figures/mean_median_error.eps'
set output output_file
plot input_file using 1:median_40_error with lines title '40x40',\
     input_file using 1:median_60_error with lines title '60x60',\
     input_file using 1:median_80_error with lines title '80x80'

output_file = '../figures/mean_median_variance.eps'
set output output_file
plot input_file using 1:median_40_variance with lines title '40x40',\
     input_file using 1:median_60_variance with lines title '60x60',\
     input_file using 1:median_80_variance with lines title '80x80'



output_file = '../figures/mean_median10_error.eps'
set output output_file
plot input_file using 1:median10_40_error with lines title '40x40',\
     input_file using 1:median10_60_error with lines title '60x60',\
     input_file using 1:median10_80_error with lines title '80x80'

output_file = '../figures/mean_median10_variance.eps'
set output output_file
plot input_file using 1:median10_40_variance with lines title '40x40',\
     input_file using 1:median10_60_variance with lines title '60x60',\
     input_file using 1:median10_80_variance with lines title '80x80'



output_file = '../figures/mean_org_error.eps'
set output output_file
plot input_file using 1:org_40_error with lines title '40x40',\
     input_file using 1:org_60_error with lines title '60x60',\
     input_file using 1:org_80_error with lines title '80x80'

output_file = '../figures/mean_org_variance.eps'
set output output_file
plot input_file using 1:org_40_variance with lines title '40x40',\
     input_file using 1:org_60_variance with lines title '60x60',\
     input_file using 1:org_80_variance with lines title '80x80'



set term wxt