//
// Created by taojiang on 2020/1/2.
//

#include <iostream>
//#include <assert.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

// #include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>


#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>


#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Point.h>
//#include <tf/transform_datatypes.h>


#include <ros/ros.h>

using namespace std;

#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif

ros::Publisher pub_octomap, pub_inf_octomap, pub_pcd, pub_inf_pcd;
octomap::OcTree* octree;

octomap::OcTree* m_octree;
octomap::ColorOcTree* read_color_tree;

pcl::PointCloud<PCLPoint> pclCloud;
pcl::PointCloud<PCLPoint> inf_pclCloud;
octomap::OcTree* inf_m_octree;


void inflate_octomap(void);
void cmdCallback(const ros::TimerEvent& e){
    printf("publish octomap!\n");

    //声明message
    octomap_msgs::Octomap map_msg, inf_map_msg;

    // map_msg.binary = true;

//    fullMapToMsg负责转换成message
    if (octomap_msgs::fullMapToMsg(*m_octree, map_msg))
    {
        //转换成功，可以发布了
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = ros::Time::now();
        pub_octomap.publish(map_msg);
    } 
    else
        ROS_ERROR("Error serializing OctoMap");

    if (octomap_msgs::fullMapToMsg(*inf_m_octree, inf_map_msg))
    {
        //转换成功，可以发布了
        inf_map_msg.header.frame_id = "map";
        inf_map_msg.header.stamp = ros::Time::now();
        pub_inf_octomap.publish(inf_map_msg);
    }
    else
        ROS_ERROR("Error serializing inflate OctoMap");

    sensor_msgs::PointCloud2 outputPC, output_inf_PC;

    pcl::toROSMsg(pclCloud, outputPC);
    pcl::toROSMsg(inf_pclCloud, output_inf_PC);

    output_inf_PC.header.stamp = outputPC.header.stamp = ros::Time::now();
    output_inf_PC.header.frame_id = outputPC.header.frame_id = "map";

    pub_pcd.publish(outputPC);
    pub_inf_pcd.publish(output_inf_PC);
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
    ros::NodeHandle node("~");

    // write_ot();
//    string input_file = "/home/taojiang/Downloads/new_college.bt";
//    string input_file = "/home/taojiang/Downloads/freiburg1_360.bt";
    string input_file = "/home/taojiang/freiburg1_360.bt";
    cout << "Reading color tree from "<< input_file <<"\n";

    m_octree = new octomap::OcTree(0.2);
    inf_m_octree = new octomap::OcTree(0.2);

    if (input_file.length() <= 3)
        return false;

    std::string suffix = input_file.substr(input_file.length()-3, 3);
    if (suffix== ".bt"){
        if (!m_octree->readBinary(input_file)){
            return false;
        }
    } else if (suffix == ".ot"){
        ROS_INFO("read");
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(input_file);
        if (!tree){
            ROS_ERROR("read failed");
            return false;
        }
        if (m_octree){
            delete m_octree;
            m_octree = NULL;
        }
        m_octree = dynamic_cast<octomap::OcTree*>(tree);

        if (!m_octree){
            ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
            return false;
        }

    } else{
        return false;
    }

    ROS_INFO("Octomap file %s loaded (%zu nodes).", input_file.c_str(),m_octree->size());

    inflate_octomap();
    
    // octomap::AbstractOcTree* read_tree = octomap::AbstractOcTree::read(input_file);
    // read_color_tree = dynamic_cast<octomap::ColorOcTree*>(read_tree);

    // octree = new octomap::OcTree(input_file);

    pub_octomap = node.advertise<octomap_msgs::Octomap>("/map/octomap", 1, true);
    pub_inf_octomap = node.advertise<octomap_msgs::Octomap>("/map/inflate_octomap", 1, true);
    pub_pcd = node.advertise<sensor_msgs::PointCloud2>("/map/pcd", 1, true);
    pub_inf_pcd = node.advertise<sensor_msgs::PointCloud2>("/map/inflate_pcd", 1, true);

    ros::Timer cmd_timer = node.createTimer(ros::Duration(4.0), cmdCallback);

    // ros::Duration(0.5).sleep();

    cout << "[octomap publish]: ready." << endl;

    ros::spin();

    return 0;
}



void inflate_octomap(void){
    int m_maxTreeDepth = m_octree->getTreeDepth()-2;
    float m_res = m_octree->getResolution();
    
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);
    printf("octomap-- depth: %d, res: %f\n", m_maxTreeDepth, m_res);
    
      
      // now, traverse all leafs in the tree:
    int i(0);
    for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),end = m_octree->end(); it != end; ++it)
    {
        // bool inUpdateBBX = isInUpdateBBX(it);

        // // call general hook:
        // handleNode(it);
        // if (inUpdateBBX)
        // handleNodeInBBX(it);
        
        i++;
        if(! (i%100)) {
            ROS_INFO("node: %d, pos: [%f, %f, %f]", i, it.getX(), it.getY(), it.getZ());
        }
        if (m_octree->isNodeOccupied(*it))
        {
            double z = it.getZ();
            double half_size = it.getSize() / 2.0;
            double size = it.getSize();
            double x = it.getX();
            double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
            int r = it->getColor().r;
            int g = it->getColor().g;
            int b = it->getColor().b;
#endif

            // insert into pointcloud:

#ifdef COLOR_OCTOMAP_SERVER
            PCLPoint _point = PCLPoint();
            _point.x = x; _point.y = y; _point.z = z;
            _point.r = r; _point.g = g; _point.b = b;
            pclCloud.push_back(_point);
#else
            pclCloud.push_back(PCLPoint(x, y, z));
            double inf_radis(0.2);
            for(int i(-1); i<=1; i++)
                for(int j(-1); j<=1; j++){
                    inf_pclCloud.push_back(PCLPoint(x+i*inf_radis, y+j*inf_radis, z));

                    inf_m_octree->updateNode(octomap::point3d(x+i*inf_radis, y+j*inf_radis, z), true);
#ifdef COLOR_OCTOMAP_SERVER
                    inf_m_octree->integrateNodeColor(x+i*inf_radis, y+j*inf_radis, z, r, g, b);
#endif
                    inf_m_octree->updateInnerOccupancy();
                }
#endif

        }
    }
}
