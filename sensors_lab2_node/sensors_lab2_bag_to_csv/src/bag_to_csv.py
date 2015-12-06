#!/usr/bin/env python
import rosbag
import csv
import pprint
import numpy as np
import itertools
from os.path import join
import rospkg
from _csv import Dialect
rospack = rospkg.RosPack()
path = rospack.get_path('sensors_lab2_bag_to_csv')

filters = ["bilat","gauss","mean10","mean","median","org"]
window_sizes = ["40","60","80"]
variables = ["mean","variance","error", "absError"]

# topics = [ filer + "_" + window + "_" variable for filter]
variables_combi = list(itertools.product(*[variables, filters,window_sizes]))
variables_names = ["#Truth"] + [filter + "_" + window + "_" + variable for variable,filter,window in variables_combi]

topics = ["bilat_40/mean", "bilat_40/variance", 
          "bilat_60/mean", "bilat_60/variance",
          "bilat_80/mean", "bilat_80/variance",
          "gauss_40/mean", "gauss_40/variance",
          "gauss_60/mean", "gauss_60/variance", 
          "gauss_80/mean", "gauss_80/variance", 
          "mean10_40/mean", "mean10_40/variance", 
          "mean10_60/mean", "mean10_60/variance", 
          "mean10_80/mean", "mean10_80/variance", 
          "mean_40/mean", "mean_40/variance", 
          "mean_60/mean", "mean_60/variance", 
          "mean_80/mean", "mean_80/variance", 
          "median_40/mean", "median_40/variance", 
          "median_60/mean", "median_60/variance", 
          "median_80/mean", "median_80/variance", 
          "org_40/mean", "org_40/variance", 
          "org_60/mean", "org_60/variance", 
          "org_80/mean", "org_80/variance"]
distances = [0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0]

# pp = pprint.PrettyPrinter(indent=4)

dataList = list()

for dist in distances:
    bag = rosbag.Bag(join(path ,'bagfiles/%sm-f.bag'%dist))
    data = [[x.data for topic2,x,t in bag.read_messages(topics='/depth/%s' % topic)] for topic in topics];
#     pp.pprint(data)
    data2 = [np.mean(datapoint) for datapoint in data]
    means = data2[::2]
    variances = data2[1::2]
    errors = [x - dist for x in means]
    absErrors = errors = [abs(x) for x in errors]

    dataList.append(dict(zip(variables_names,[dist] + means + variances + errors + absErrors)))

    bag.close()

# pp.pprint( dataList)

with open(join(path ,'data.csv',), 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=variables_names)
    writer.writeheader()
    writer.writerows(dataList)

