/**
 *  Plots a grid of rays and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using std::vector;

#define X_amp 0.15
#define Y_amp 0.25
#define Z_amp 0.1


int main(int argc, char **argv){

    ros::init(argc, argv, "ray_trace_grid");

    std::ofstream myfile;

    PlotRayUtils plt;

    tf::Transform t, R;
    t.setOrigin(tf::Vector3(0,0,0));
    t.setRotation(tf::createQuaternionFromRPY(0,0,0));
    double resolution = 0.001;
    int width = (int) (X_amp/resolution*2.0+1.4); // round to nearest integer
    int length = (int) (Y_amp/resolution*2.0+1.4);
    int height = (int) (Z_amp/resolution*2.0+1.4);
    std::cout << "width: " << width << std::endl;
    std::cout << "length: " << length << std::endl;
    std::cout << "height: " << height << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ>);  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("mesh.pcd", *cloud_model) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file mesh.pcd \n");
    return (-1);
  }
    std::cout << "mesh.pcd loaded."<< std::endl;

    vector<vector<vector<int>>> occupancy_grid;
    occupancy_grid.resize(width);

    for (int i = 0; i < width; ++i) {
        occupancy_grid[i].resize(length);
        for (int j = 0; j < length; ++j) {
                occupancy_grid[i][j].resize(height);
            for (int k = 0; k < height; ++k) {
                // 0 unknown; 1 occupied; 2 unoccupied
                if (i*j*k*(i+1-width)*(j+1-length)*(k+1-height) == 0) {
                    occupancy_grid[i][j][k] = 2;
                } else {
                    occupancy_grid[i][j][k] = 0;
                }     
            }
        }
    }

    std::cout << "occupancy grid initialized."<< std::endl;
    for (int i = 0; i < cloud_model->points.size(); i++) {
        int ind0 = floor(cloud_model->points[i].x/resolution + 0.5 + (width-1.0)/2.0);
        int ind1 = floor(cloud_model->points[i].y/resolution + 0.5 + (length-1.0)/2.0);
        int ind2 = floor(cloud_model->points[i].z/resolution + 0.5 + (height-1.0)/2.0);
        occupancy_grid[ind0][ind1][ind2] = 1;
    }



    bool processing = true;
    double dist;
    int n,c0,c1,c2;
    int iter = 0;


    while (ros::ok() && processing) {
        processing = false;
        iter++;
        /*
        std::ostringstream ss;
        ss << "occupancy_" << iter << ".txt"; 
        myfile.open(ss.str());
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < length; ++j) {
                for (int k = 0; k < height; ++k) {
                    myfile << occupancy_grid[i][j][k] << " "; 
                }
                myfile << std::endl;  
            }
        }
        myfile.close();
        */
        c0 = 0; c1 = 0; c2 = 0;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < length; ++j) {
                for (int k = 0; k < height; ++k) {
                    if (occupancy_grid[i][j][k] == 0) {
                        c0++;
                    } 
                    if (occupancy_grid[i][j][k] == 1) {
                        c1++;
                    } 
                    if (occupancy_grid[i][j][k] == 2) {
                        c2++;
                    } 
                }     
            }
        }
        std::cout << "iter:" << iter << " count0:" << c0 << " count1:" << c1 << " count2:" << c2 << std::endl;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < length; ++j) {
                for (int k = 0; k < height; ++k) {
                    if (occupancy_grid[i][j][k] == 2) {
                        if (i > 0 && occupancy_grid[i-1][j][k] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(-X_amp, -Y_amp + j*resolution, -Z_amp + k*resolution);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = i-1; m > 0; --m ) {
                                    if (occupancy_grid[m][j][k] == 0) {
                                        occupancy_grid[m][j][k] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i-n][j][k] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i-m][j][k] = 2;
                                    }
                                }

                            }
                            processing = true;
                        }
                        if (i < (width-1) && occupancy_grid[i+1][j][k] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(X_amp, -Y_amp + j*resolution, -Z_amp + k*resolution);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = i+1; m < width-2; ++m ) {
                                    if (occupancy_grid[m][j][k] == 0) {
                                        occupancy_grid[m][j][k] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i+n][j][k] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i+m][j][k] = 2;
                                    }
                                }
                            } 
                            processing = true;
                        }
                        if (j > 0 && occupancy_grid[i][j-1][k] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(-X_amp + i*resolution, -Y_amp, -Z_amp + k*resolution);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = j-1; m > 0; --m ) {
                                    if (occupancy_grid[i][m][k] == 0) {
                                        occupancy_grid[i][m][k] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i][j-n][k] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i][j-m][k] = 2;
                                    }
                                }
                            } 
                            processing = true;
                        }
                        if (j < (length - 1) && occupancy_grid[i][j+1][k] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(-X_amp + i*resolution, Y_amp, -Z_amp + k*resolution);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = j+1; m < length-2; ++m ) {
                                    if (occupancy_grid[i][m][k] == 0) {
                                        occupancy_grid[i][m][k] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i][j+n][k] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i][j+m][k] = 2;
                                    }
                                }
                            } 
                            processing = true;
                        }
                        if (k > 0 && occupancy_grid[i][j][k-1] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = k-1; m > 0; --m ) {
                                    if (occupancy_grid[i][j][m] == 0) {
                                        occupancy_grid[i][j][m] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i][j][k-n] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i][j][k-m] = 2;
                                    }
                                }
                            } 
                            processing = true;
                        }
                        if (k < (height - 1) && occupancy_grid[i][j][k+1] == 0) {
                            tf::Point start(-X_amp + i*resolution, -Y_amp + j*resolution, -Z_amp + k*resolution);
	                    tf::Point end(-X_amp + i*resolution, -Y_amp + j*resolution, Z_amp);
                            //plt.plotIntersections(start, end, false);
                            dist = plt.getDistToPart(start, end);
                            //ROS_INFO("dist is %f", dist);
                            if (dist > 999) {
                                for (int m = k+1; m < height-2; ++m ) {
                                    if (occupancy_grid[i][j][m] == 0) {
                                        occupancy_grid[i][j][m] = 2;
                                    }
                                }
                            } else {
                                n = int (dist/resolution+0.509);
                                occupancy_grid[i][j][k+n] = 1;
                                if (n > 1) {
                                    for (int m = 1; m < n; ++m) {
                                        occupancy_grid[i][j][k+m] = 2;
                                    }
                                }
                            } 
                            processing = true;
                        }
                            //plt.plotCylinder(start, end, 0.01, 0.002);
	                //ros::Duration(0.01).sleep();
            //std::cout << "x:" << -X_amp + i*resolution << " y:" << -Y_amp + j*resolution << " dist:" << dist[0] << " IG:" << plt.getIG(start, end, 0.01, 0.002) << std::endl;
                    }
                }
            }
        }
    }

    std::ostringstream ss;
    ss << "output_pcd" << iter << ".txt"; 
    myfile.open(ss.str());
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < length; ++j) {
            for (int k = 0; k < height; ++k) {
                if (occupancy_grid[i][j][k] != 2) {
                    myfile << -X_amp + i*resolution << ", " << -Y_amp + j*resolution << ", " << -Z_amp + k*resolution << ";" <<std::endl; 
                }
            }
            //myfile << std::endl;  
        }
    }
    myfile.close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ>);  
    cloud_output->width    = c0+c1;
    cloud_output->height   = 1;
    cloud_output->is_dense = true;
    cloud_output->points.resize (cloud_output->width * cloud_output->height);
   
    int ind = 0;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < length; ++j) {
            for (int k = 0; k < height; ++k) {
                if (occupancy_grid[i][j][k] != 2) {
                    cloud_output->points[ind].x = -X_amp + i*resolution;
                    cloud_output->points[ind].y = -Y_amp + j*resolution;
                    cloud_output->points[ind].z = -Z_amp + k*resolution;
                    ind++;
                }
            }  
        }
    }

    pcl::io::savePCDFileASCII ("output_pcd.pcd", *cloud_output);
    std::cerr << "Saved " << cloud_output->points.size () << " data points to output_pcd.pcd." << std::endl;



  return 0;
}
