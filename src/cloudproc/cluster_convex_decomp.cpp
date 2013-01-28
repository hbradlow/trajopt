#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>

#include "pcl/io/vtk_io.h" 
#include "pcl/io/vtk_lib_io.h"


#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "cloudproc.hpp"

#include <boost/format.hpp>
#include <boost/foreach.hpp>

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;

pcl::PointIndices::Ptr getPlane(pcl::PointCloud<Point>::Ptr cloud,pcl::ModelCoefficients::Ptr coefficients){
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<Point> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    return inliers;
}


std::vector<pcl::PointIndices> getClusters(pcl::PointCloud<Point>::Ptr cloud){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (0.03); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    return cluster_indices;
}

pcl::PointCloud<ColorPoint>::Ptr convertXYZtoXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<ColorPoint>::Ptr cloud_out (new pcl::PointCloud<ColorPoint>);
    BOOST_FOREACH(pcl::PointXYZ point, cloud->points){
        pcl::PointXYZRGB p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.r = 255;
        p.g = 255;
        p.b = 255;
        cloud_out->push_back(p);
    }
    return cloud_out;
}

void extractPoints(pcl::PointCloud<Point>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative = false){
    // Create the filtering object
    pcl::ExtractIndices<Point> extract;
    pcl::PointCloud<Point> cloud_filtered;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (negative);
    extract.filter (cloud_filtered);
    *cloud = cloud_filtered;
}
pcl::PointIndices::Ptr extractPointsNearModel(pcl::PointCloud<Point>::Ptr cloud, pcl::ModelCoefficients::Ptr coef, float threshold = .1, bool negative=false){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int i = 0; i<cloud->points.size(); i++){
        pcl::PointXYZ point = cloud->points[i];
        float value = coef->values[0]*point.x + coef->values[1]*point.y + coef->values[2]*point.z + coef->values[3];
        if(abs(value)<threshold){
            inliers->indices.push_back(i);
        } 
    }
    return inliers;
}

pcl::PolygonMesh::Ptr getConvexHull(pcl::PointCloud<Point>::Ptr cloud){
    pcl::ConvexHull<Point> ch;
    pcl::PolygonMesh::Ptr pg(new pcl::PolygonMesh);
    ch.setInputCloud(cloud);
    ch.reconstruct(*pg);
    return pg;
}

std::vector<pcl::PolygonMesh::Ptr> convexDecompose(pcl::PointCloud<Point>::Ptr cloud){
    std::vector<pcl::PolygonMesh::Ptr> meshes;

    //remove the NaN's from the input
    pcl::PointCloud<Point> cloud_nonan;
    std::vector<int> filter_indices;
    pcl::removeNaNFromPointCloud(*cloud,cloud_nonan,filter_indices);
    *cloud = cloud_nonan;

    pcl::PointCloud<ColorPoint>::Ptr cloud_out (new pcl::PointCloud<ColorPoint>);
    pcl::PointCloud<Point>::Ptr tmp (new pcl::PointCloud<Point>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);

    viewer->addPolygonMesh(*cloudproc::createMesh_marchingCubes(cloud),"marching_cubes");
    int index = 0;

    for(int i = 0; i<2; i++){
        index++;
        tmp->clear();
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane = getPlane(cloud,coefficients);
        int r = rand()%255;
        int g = rand()%255;
        int b = rand()%255;
        plane = extractPointsNearModel(cloud,coefficients,.3,true);
        BOOST_FOREACH(int i,plane->indices){
            ColorPoint p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = r;
            p.g = g;
            p.b = b;
            cloud_out->push_back(p);
            tmp->push_back(cloud->points[i]);
        }
        extractPoints(cloud,plane,true);
        pcl::PolygonMesh::Ptr master = getConvexHull(tmp);
        viewer->addPolygonMesh(*master,(boost::format("thing_%i")%index).str());
        meshes.push_back(master);
    }


    BOOST_FOREACH(pcl::PointIndices indices, getClusters(cloud)){
        index++;
        tmp->clear();
        pcl::PointIndices::Ptr indicesptr (new pcl::PointIndices);
        int r = rand()%255;
        int g = rand()%255;
        int b = rand()%255;
        BOOST_FOREACH(int i,indices.indices){
            ColorPoint p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = r;
            p.g = g;
            p.b = b;
            tmp->push_back(cloud->points[i]);
            cloud_out->push_back(p);
            indicesptr->indices.push_back(i);
        }
        pcl::PolygonMesh::Ptr master = getConvexHull(tmp);
        viewer->addPolygonMesh(*master,(boost::format("thing_%i")%index).str());
        meshes.push_back(master);
    }

    viewer->spin();

    pcl::io::savePCDFileASCII ("test_cluster.pcd", *cloud_out); 

    return meshes;
}
int main(){
    pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
    if (pcl::io::loadPCDFile<Point> ("../bigdata/down4.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    convexDecompose(cloud);
}

