//
// Created by taojiang on 2020/1/2.
//

#include <iostream>
//#include <assert.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>



#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


//#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Point.h>
//#include <tf/transform_datatypes.h>


#include <ros/ros.h>

using namespace std;

ros::Publisher pub_octomap;
octomap::OcTree* octree;
octomap::ColorOcTree* read_color_tree;

void cmdCallback(const ros::TimerEvent& e){
    printf("publish octomap!\n");

    //声明message
    octomap_msgs::Octomap map_msg;
    map_msg.header.frame_id = 'world';
    map_msg.header.stamp = ros::Time::now();

//    fullMapToMsg负责转换成message
    if (octomap_msgs::fullMapToMsg(*read_color_tree, map_msg))
        //转换成功，可以发布了
        pub_octomap.publish(map_msg);
    else
        ROS_ERROR("Error serializing OctoMap");
}


void write_ot(void){
    double res = 0.05;  // create empty tree with resolution 0.05 (different from default 0.1 for test)
    octomap::ColorOcTree tree (res);
    // insert some measurements of occupied cells
    for (int x=-20; x<20; x++) {
        for (int y=-20; y<20; y++) {
            for (int z=-20; z<20; z++) {
                octomap::point3d endpoint ((float) x*0.05f+0.01f, (float) y*0.05f+0.01f, (float) z*0.05f+0.01f);
                octomap::ColorOcTreeNode* n = tree.updateNode(endpoint, true);
                n->setColor(z*5+100,x*5+100,y*5+100);
            }
        }
    }

    // insert some measurements of free cells
    for (int x=-30; x<30; x++) {
        for (int y=-30; y<30; y++) {
            for (int z=-30; z<30; z++) {
                octomap::point3d endpoint ((float) x*0.02f+2.0f, (float) y*0.02f+2.0f, (float) z*0.02f+2.0f);
                octomap::ColorOcTreeNode* n = tree.updateNode(endpoint, false);
                n->setColor(255,255,0); // set color to yellow
            }
        }
    }

    // set inner node colors
    tree.updateInnerOccupancy();

    // should already be pruned
//    EXPECT_EQ(tree.size(), tree.calcNumNodes());
    const size_t initialSize = tree.size();
//    EXPECT_EQ(initialSize, 1034);
    tree.prune();
//    EXPECT_EQ(tree.size(), tree.calcNumNodes());
//    EXPECT_EQ(initialSize, tree.size());

    cout << endl;

    std::cout << "\nWriting to / from file\n===============================\n";
    std::string filename ("simple_color_tree.ot");
    std::cout << "Writing color tree to " << filename << std::endl;
    // write color tree
    tree.write(filename);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "pub_octo");
    ros::NodeHandle node;

    write_ot();
//    string input_file = "/home/taojiang/Downloads/new_college.bt";
//    string input_file = "/home/taojiang/Downloads/freiburg1_360.bt";
    string input_file = "/home/taojiang/simple_color_tree.ot";
    cout << "Reading color tree from "<< input_file <<"\n";

    octomap::AbstractOcTree* read_tree = octomap::AbstractOcTree::read(input_file);

    read_color_tree = dynamic_cast<octomap::ColorOcTree*>(read_tree);

    octree = new octomap::OcTree(input_file);

    pub_octomap = node.advertise<octomap_msgs::Octomap>("/map/octomap", 1, true);

    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.2), cmdCallback);

    ros::Duration(0.1).sleep();

    cout << "[octomap publish]: ready." << endl;

    ros::spin();

    return 0;
}

