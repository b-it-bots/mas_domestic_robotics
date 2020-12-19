/*
 * Copyright 2020 Bonn-Rhein-Sieg University
 *
 * Author: Minh Nguyen
 * Author: Sushant Vijay Chavan
 *
 */
#ifndef MDR_CLOUD_OBJECT_DETECTION_ALIASES_H
#define MDR_CLOUD_OBJECT_DETECTION_ALIASES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;

#endif  // MDR_CLOUD_OBJECT_DETECTION_ALIASES_H
