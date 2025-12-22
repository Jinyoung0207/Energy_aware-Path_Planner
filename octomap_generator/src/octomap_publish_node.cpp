// octomap_generator_final.cpp
#include <tinyxml2.h>
#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <queue>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/tf.h> 

using namespace tinyxml2;
using std::vector;
using std::tuple;
using std::string;


struct BoxObstacle 
{
  tf::Transform transform;
  tf::Vector3 size;       
};

struct GridParams 
{
  int width = 100;
  int height = 50;
  int depth = 20;
  double resolution = 0.2;
  double z_resolution = 0.2;
  double origin_x = 0.0;
  double origin_y = -5.0;
  double origin_z = 0.0;
};

// 정수 기반 루프를 위해 GridIdx 구조체 정의
struct GridIdx { int x, y, z; }; 

// world 좌표를 정수 GridIdx로 변환하는 헬퍼 함수
GridIdx worldToGrid(double wx, double wy, double wz, const GridParams& g) 
{
    int x = static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
    int y = static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
    int z = static_cast<int>(std::floor((wz - g.origin_z) / g.z_resolution));
    x = std::max(0, std::min(g.width  - 1, x));
    y = std::max(0, std::min(g.height - 1, y));
    z = std::max(0, std::min(g.depth  - 1, z));
    return {x, y, z};
}


vector<BoxObstacle> parseWorldFile(const string& filename) 
{
  vector<BoxObstacle> obstacles;
  XMLDocument doc;
  if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) 
  {
    ROS_ERROR("Failed to load world file: %s", filename.c_str());
    return obstacles;
  }

  auto* sdf = doc.FirstChildElement("sdf");
  if (!sdf) 
  {
      ROS_ERROR("Could not find <sdf> element in file.");
      return obstacles;
  }

  XMLElement* models_container = sdf->FirstChildElement("world");
  if (!models_container) {
    models_container = sdf;
  }

  for (auto* model = models_container->FirstChildElement("model");
       model; model = model->NextSiblingElement("model")) 
  {
    if (strcmp(model->Attribute("name"), "ground_plane") == 0) 
    {
        continue;
    }

    tf::Transform model_transform;
    model_transform.setIdentity();
    if (auto* p = model->FirstChildElement("pose")) 
    {
      double x=0, y=0, z=0, r=0, pi=0, ya=0;
      sscanf(p->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
      model_transform.setOrigin(tf::Vector3(x, y, z));
      tf::Quaternion q;
      q.setRPY(r, pi, ya);
      model_transform.setRotation(q);
    }

    for (auto* link = model->FirstChildElement("link");
         link; link = link->NextSiblingElement("link")) 
    {
      tf::Transform link_transform;
      link_transform.setIdentity();
      if (auto* lp = link->FirstChildElement("pose")) 
      {
        double x=0, y=0, z=0, r=0, pi=0, ya=0;
        sscanf(lp->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
        link_transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(r, pi, ya);
        link_transform.setRotation(q);
      }
      
      auto* col = link->FirstChildElement("collision");
      if (!col) continue;

      tf::Transform collision_transform;
      collision_transform.setIdentity();
      if (auto* cp = col->FirstChildElement("pose")) 
      {
        double x=0, y=0, z=0, r=0, pi=0, ya=0;
        sscanf(cp->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
        collision_transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(r, pi, ya);
        collision_transform.setRotation(q);
      }

      auto* geo = col->FirstChildElement("geometry");
      if (!geo) continue;
      auto* box = geo->FirstChildElement("box");
      if (!box) continue;
      auto* sz_elem  = box->FirstChildElement("size");
      if (!sz_elem) continue;

      double sx, sy, sz;
      sscanf(sz_elem->GetText(), "%lf %lf %lf", &sx, &sy, &sz);

      BoxObstacle ob;
      ob.size = tf::Vector3(sx, sy, sz);
      ob.transform = model_transform * link_transform * collision_transform;
      obstacles.push_back(ob);
    }
  }
  return obstacles;
}

void fillFreeSpace(const GridParams& g, std::shared_ptr<octomap::OcTree>& octree) 
{
  for (int ix = 0; ix < g.width; ++ix) 
  {
    for (int iy = 0; iy < g.height; ++iy) 
    {
      for (int iz = 0; iz < g.depth; ++iz) 
      {
        double x = g.origin_x + (ix + 0.5) * g.resolution;
        double y = g.origin_y + (iy + 0.5) * g.resolution;
        double z = g.origin_z + (iz + 0.5) * g.z_resolution;
        octomap::point3d p(x, y, z);
        octree->updateNode(p, false);
      }
    }
  }
}

void insertKnownBoxesIntoOctoMap
(
  const vector<BoxObstacle>& boxes,
  std::shared_ptr<octomap::OcTree>& octree,
  const GridParams& g)
{
  const double robot_radius = 0.5;

  const double h_res_x = g.resolution / 2.0;
  const double h_res_y = g.resolution / 2.0;
  const double h_res_z = g.z_resolution / 2.0;
  
  const std::vector<tf::Vector3> sample_offsets = 
  {
    tf::Vector3(0.0,     0.0,     0.0),     // Center
    tf::Vector3( h_res_x,  h_res_y,  h_res_z), // Corner 1
    tf::Vector3( h_res_x,  h_res_y, -h_res_z), // Corner 2
    tf::Vector3( h_res_x, -h_res_y,  h_res_z), // Corner 3
    tf::Vector3( h_res_x, -h_res_y, -h_res_z), // Corner 4
    tf::Vector3(-h_res_x,  h_res_y,  h_res_z), // Corner 5
    tf::Vector3(-h_res_x,  h_res_y, -h_res_z), // Corner 6
    tf::Vector3(-h_res_x, -h_res_y,  h_res_z), // Corner 7
    tf::Vector3(-h_res_x, -h_res_y, -h_res_z)  // Corner 8
  };

  for (const auto& box : boxes) 
  {
    double hx = box.size.x() / 2.0 + robot_radius;
    double hy = box.size.y() / 2.0 + robot_radius;
    double hz = box.size.z() / 2.0 + robot_radius + 0.2;

    // double box_bottom_z = box.transform.getOrigin().z() - (box.size.z() / 2.0);
    // if(box_bottom_z < 0.3)
    // {
    //   hz += 0.2;
    // }

    vector<tf::Vector3> local_corners;
    local_corners.push_back(tf::Vector3( hx,  hy,  hz)); local_corners.push_back(tf::Vector3( hx,  hy, -hz));
    local_corners.push_back(tf::Vector3( hx, -hy,  hz)); local_corners.push_back(tf::Vector3( hx, -hy, -hz));
    local_corners.push_back(tf::Vector3(-hx,  hy,  hz)); local_corners.push_back(tf::Vector3(-hx,  hy, -hz));
    local_corners.push_back(tf::Vector3(-hx, -hy,  hz)); local_corners.push_back(tf::Vector3(-hx, -hy, -hz));
    
    tf::Vector3 min_pt( 1e6,  1e6,  1e6);
    tf::Vector3 max_pt(-1e6, -1e6, -1e6);
    for(const auto& lc : local_corners) {
        tf::Vector3 wc = box.transform * lc;
        min_pt.setX(std::min(min_pt.x(), wc.x())); min_pt.setY(std::min(min_pt.y(), wc.y())); min_pt.setZ(std::min(min_pt.z(), wc.z()));
        max_pt.setX(std::max(max_pt.x(), wc.x())); max_pt.setY(std::max(max_pt.y(), wc.y())); max_pt.setZ(std::max(max_pt.z(), wc.z()));
    }

    tf::Transform box_inverse_transform = box.transform.inverse();

    // 월드 좌표 경계를 정수 그리드 인덱스로 변환
    GridIdx min_idx = worldToGrid(min_pt.x(), min_pt.y(), min_pt.z(), g);
    GridIdx max_idx = worldToGrid(max_pt.x(), max_pt.y(), max_pt.z(), g);

    // 정수(int)를 사용해 루프 실행하여 부동 소수점 오류 방지
    for (int ix = min_idx.x; ix <= max_idx.x; ++ix) 
    {
      for (int iy = min_idx.y; iy <= max_idx.y; ++iy) 
      {
        for (int iz = min_idx.z; iz <= max_idx.z; ++iz) 
        {
          
          double x = g.origin_x + (ix + 0.5) * g.resolution;
          double y = g.origin_y + (iy + 0.5) * g.resolution;
          double z = g.origin_z + (iz + 0.5) * g.z_resolution;

          tf::Vector3 voxel_center_world(x, y, z);
          bool is_occupied = false;
          
          for (const auto& offset : sample_offsets) {
            tf::Vector3 sample_point_world = voxel_center_world + offset;
            tf::Vector3 sample_point_local = box_inverse_transform * sample_point_world;
            
            if (std::abs(sample_point_local.x()) <= hx + 1e-3 &&
                std::abs(sample_point_local.y()) <= hy + 1e-3 &&
                std::abs(sample_point_local.z()) <= hz + 1e-3)
            {
              is_occupied = true;
              break;
            }
          }
          
          if (is_occupied) {
            octree->updateNode(octomap::point3d(x, y, z), true);
          }
        }
      }
    }
  }
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "octomap_publish_node");
  ros::NodeHandle nh("~");

  std::string world_file;

  // nh.param<std::string>("world_file", world_file, "/home/jinyoung/building_editor_models/좁은통로.world");
  nh.param<std::string>("world_file", world_file, "/home/jinyoung/building_editor_models/지상장애물_벽.world");


  GridParams grid;
  auto obstacles = parseWorldFile(world_file);
  ROS_INFO("Parsed %zu obstacles from world file.", obstacles.size());

  auto octree = std::make_shared<octomap::OcTree>(grid.resolution);
  

  fillFreeSpace(grid, octree);
  
  insertKnownBoxesIntoOctoMap(obstacles, octree, grid);
  octree->updateInnerOccupancy();

  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_binary", 1, true);
  ros::Publisher obstacle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("known_obstacles", 1, true);


  octomap_msgs::Octomap octo_msg;
  if (octomap_msgs::binaryMapToMsg(*octree, octo_msg)) 
  {
    octo_msg.header.frame_id = "map";
    octo_msg.header.stamp = ros::Time::now();
    octomap_pub.publish(octo_msg);
    ROS_INFO("OctoMap published.");
  } 
  else 
  {
    ROS_ERROR("Failed to convert octree to message.");
  }

  visualization_msgs::MarkerArray boxes_markers;
  int id = 0;
  for (const auto& box : obstacles) 
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "known_box";
    m.id = id++;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    tf::poseTFToMsg(box.transform, m.pose);
    m.scale.x = box.size.x();
    m.scale.y = box.size.y();
    m.scale.z = box.size.z();
    m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.2;
    boxes_markers.markers.push_back(m);
  }
  obstacle_vis_pub.publish(boxes_markers);
  ROS_INFO("Obstacle markers published.");

  ROS_INFO("Static map generated. Node will now spin.");
  ros::spin();

  return 0;
}