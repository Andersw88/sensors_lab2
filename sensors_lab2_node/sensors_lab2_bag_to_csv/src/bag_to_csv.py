#!/usr/bin/env python
import rosbag
import csv
import pprint
import numpy as np
from os.path import join

import rospkg
rospack = rospkg.RosPack()
# rospack.list_pkgs() 
path = rospack.get_path('sensors_lab2_bag_to_csv')
# print path


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
# distances = [0.8,1.0,1.2,1.4,1.6,1.8,2.0]

pp = pprint.PrettyPrinter(indent=4)


topicsPlusTruth = ["Truth"] + topics;

dataList = list()

for dist in distances:
    bag = rosbag.Bag(join(path ,'bagfiles/%sm-f.bag'%dist))
    data = [[x.data for topic2,x,t in bag.read_messages(topics='/depth/%s' % topic)] for topic in topics];
#     pp.pprint(data)
    means = [[np.mean(datapoint)] for datapoint in data] 
#     pp.pprint(means)
    
    dataList.append(dict(zip(topicsPlusTruth,[dist] + [item for sublist in means for item in sublist])))
#     dataDict.append([item for sublist in means for item in sublist])
    bag.close()

# pp.pprint( dataList)

with open(join(path ,'data.csv'), 'w') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=topicsPlusTruth)
    writer.writeheader()
    writer.writerows(dataList)

