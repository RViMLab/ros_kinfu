#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <kinfu_msgs/RequestAction.h>
#include <kinfu_msgs/KinfuCommand.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

std::string file_name = "mesh.ply";
std::string file_dir  = "";

static const std::string OPENCV_WINDOW = "Kinfu Monitor";
static const std::string OCCLUDED_NAME = "occluded.jpg";
static const unsigned    BUTTON_HEIGHT = 50;
static const unsigned    NUM_BUTTONS   = 6;
static const unsigned    BUTTONS_PER_ROW = 3;
static const unsigned    NUM_ROWS      = ceil(NUM_BUTTONS/double(BUTTONS_PER_ROW));

cv::Mat3b canvas;
std::string buttonText("Save Mesh");

struct button{
    cv::Rect location;
    cv::Vec3b colour;
    std::string text;
};

button create_button(std::string text,cv::Vec3b colour)
{
    button b;
    b.text=text;
    b.colour=colour;
    return b;
}

std::vector<button> buttons;
ros::Publisher cmd_pub;


#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/texture_mapping.h>

#include <pcl/io/vtk_lib_io.h>

using namespace pcl;

/** \brief Save a textureMesh object to obj file */
int
saveOBJFile (const std::string &file_name,
             const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
      tex_mesh.cloud.fields[d].name == "normal_x" ||
      tex_mesh.cloud.fields[d].name == "normal_y" ||
      tex_mesh.cloud.fields[d].name == "normal_z"))
      {
        if (!v_written)
        {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
    return (-2);
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  PCL_INFO ("Writing faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0)
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << std::endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
      fs << "# Faces" << std::endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1
        << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //don't do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return (0);

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}

/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  // visualization object
  pcl::visualization::PCLVisualizer visu ("cameras");

  // add a visual for each camera at the correct pose
  for(int i = 0 ; i < cams.size () ; ++i)
  {
    // read current camera
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
    double focal = cam.focal_length;
    double height = cam.height;
    double width = cam.width;

    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, cam.pose);
    p2=pcl::transformPoint (p2, cam.pose);
    p3=pcl::transformPoint (p3, cam.pose);
    p4=pcl::transformPoint (p4, cam.pose);
    p5=pcl::transformPoint (p5, cam.pose);
    std::stringstream ss;
    ss << "Cam #" << i+1;
    visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

    ss.str ("");
    ss << "camera_" << i << "line1";
    visu.addLine (p1, p2,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line2";
    visu.addLine (p1, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line3";
    visu.addLine (p1, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line4";
    visu.addLine (p1, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line5";
    visu.addLine (p2, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line6";
    visu.addLine (p5, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line7";
    visu.addLine (p4, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line8";
    visu.addLine (p3, p2,ss.str ());
  }

  // add a coordinate system
  visu.addCoordinateSystem (1.0, "global");

  // add the mesh's cloud (colored on Z axis)
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
  visu.addPointCloud (cloud, color_handler, "cloud");

  // reset camera
  visu.resetCamera ();

  // wait for user input
  visu.spin ();
}

/** \brief Helper function that jump to a specific line of a text file */
std::ifstream& GotoLine(std::ifstream& file, unsigned int num)
{
  file.seekg (std::ios::beg);
  for(int i=0; i < num - 1; ++i)
  {
    file.ignore (std::numeric_limits<std::streamsize>::max (),'\n');
  }
  return (file);
}

/** \brief Helper function that reads a camera file outputed by Kinfu */
bool readCamPoseFile(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{
  ifstream myReadFile;
  myReadFile.open(filename.c_str (), ios::in);
  if(!myReadFile.is_open ())
  {
    PCL_ERROR ("Error opening file %d\n", filename.c_str ());
    return false;
  }
  myReadFile.seekg(ios::beg);

  char current_line[1024];
  double val;

  // go to line 2 to read translations
  GotoLine(myReadFile, 2);
  myReadFile >> val; cam.pose (0,3)=val; //TX
  myReadFile >> val; cam.pose (1,3)=val; //TY
  myReadFile >> val; cam.pose (2,3)=val; //TZ

  // go to line 7 to read rotations
  GotoLine(myReadFile, 7);

  myReadFile >> val; cam.pose (0,0)=val;
  myReadFile >> val; cam.pose (0,1)=val;
  myReadFile >> val; cam.pose (0,2)=val;

  myReadFile >> val; cam.pose (1,0)=val;
  myReadFile >> val; cam.pose (1,1)=val;
  myReadFile >> val; cam.pose (1,2)=val;

  myReadFile >> val; cam.pose (2,0)=val;
  myReadFile >> val; cam.pose (2,1)=val;
  myReadFile >> val; cam.pose (2,2)=val;

  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale

  // go to line 12 to read camera focal length and size
  GotoLine (myReadFile, 12);
  myReadFile >> val; cam.focal_length=val;
  myReadFile >> val; cam.height=val;
  myReadFile >> val; cam.width=val;

  // close file
  myReadFile.close ();

  return true;

}

int
texture_obj ()
{
  std::string ply_filename = file_dir + file_name;

  // read mesh from plyfile
  PCL_INFO ("\nLoading mesh from file %s...\n", ply_filename.c_str());
  pcl::PolygonMesh triangles;
  pcl::io::loadPolygonFilePLY(ply_filename, triangles);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

  // Create the texturemesh object that will contain our UV-mapped mesh
  TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  std::vector< pcl::Vertices> polygon_1;

  // push faces into the texturemesh object
  polygon_1.resize (triangles.polygons.size ());
  for(size_t i =0; i < triangles.polygons.size (); ++i)
  {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);
  PCL_INFO ("\tInput mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size (), cloud->points.size ());
  PCL_INFO ("...Done.\n");

  // Load textures and cameras poses and intrinsics
  PCL_INFO ("\nLoading textures and camera poses...\n");
  pcl::texture_mapping::CameraVector my_cams;

  const boost::filesystem::path base_dir (file_dir);
  std::string extension (".txt");
  int cpt_cam = 0;
  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if(boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
      readCamPoseFile(it->path ().string (), cam);
      cam.texture_file = boost::filesystem::basename (it->path ()) + ".png";
      my_cams.push_back (cam);
      cpt_cam++ ;
    }
  }
  PCL_INFO ("\tLoaded %d textures.\n", my_cams.size ());
  PCL_INFO ("...Done.\n");

  // Display cameras to user
  PCL_INFO ("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
  showCameras(my_cams, cloud);


  // Create materials for each texture (and one extra for occluded faces)
  mesh.tex_materials.resize (my_cams.size () + 1);
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    pcl::TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    std::stringstream tex_name;
    tex_name << "material_" << i;
    tex_name >> mesh_material.tex_name;

    if(i < my_cams.size ())
      mesh_material.tex_file = my_cams[i].texture_file;
    else
      mesh_material.tex_file = "occluded.jpg";

    mesh.tex_materials[i] = mesh_material;
  }


  // Sort faces
  PCL_INFO ("\nSorting faces by cameras...\n");
  pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
  tm.textureMeshwithMultipleCameras(mesh, my_cams);


  PCL_INFO ("Sorting faces by cameras done.\n");
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
  }


  // compute normals for the mesh
  PCL_INFO ("\nEstimating normals...\n");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  PCL_INFO ("...Done.\n");

  pcl::toPCLPointCloud2 (*cloud_with_normals, mesh.cloud);

  PCL_INFO ("\nSaving mesh to textured_mesh.obj\n");

  saveOBJFile (file_dir+"/textured_mesh.obj", mesh, 5);

  return (0);
}

int save_ply(){

    actionlib::SimpleActionClient<kinfu_msgs::RequestAction> action_client("/kinfu_output/actions/request",true);
    ROS_INFO("Waiting for server...");
    action_client.waitForServer();

    ROS_INFO("Sending goal...");
    kinfu_msgs::RequestGoal req_goal;
    kinfu_msgs::KinfuTsdfRequest & req_publish = req_goal.request;
    req_publish.tsdf_header.request_type = req_publish.tsdf_header.REQUEST_TYPE_GET_MESH;
    req_publish.request_remove_duplicates = true;
    req_publish.request_transformation = true;
    req_publish.transformation.linear.assign(0.f);
    req_publish.transformation.linear.at(0)=1.f;
    req_publish.transformation.linear.at(4)=1.f;
    req_publish.transformation.linear.at(8)=1.f;
    req_publish.transformation.translation.assign(0.f);
    action_client.sendGoal(req_goal);

    ROS_INFO("Waiting for result...");
    action_client.waitForResult(ros::Duration(100.0));
    actionlib::SimpleClientGoalState state = action_client.getState();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Action did not succeed.");
        return 1;
    }
    ROS_INFO("Action succeeded.");

    std::string file_full = file_dir + file_name;
    ROS_INFO("Saving mesh to '%s'...",file_full.c_str());
    kinfu_msgs::RequestResultConstPtr result = action_client.getResult();
    const pcl_msgs::PolygonMesh & mesh_msg = result->mesh;
    pcl::PolygonMesh mesh;
    pcl_conversions::toPCL(mesh_msg,mesh);
    if (pcl::io::savePLYFileBinary(file_full,mesh))
    {
        ROS_ERROR("Mesh could not be saved.");
        return 2;
    }

    // An image for the parts of the model that weren't captured by a texture
    cv::Mat3b occluded_img(30, 30, cv::Vec3b(48, 48, 48));
    std::string occluded_file=file_dir+OCCLUDED_NAME;
    cv::imwrite(occluded_file,occluded_img);

    ROS_INFO("Mesh saved.");
    return 0;
}

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (buttons[0].location.contains(cv::Point(x, y)))
        {
            save_ply();
            //cv::rectangle(canvas(button1), button1, cv::Scalar(0,0,255), 2);
        }
        if (buttons[1].location.contains(cv::Point(x, y)))
        {
            texture_obj();
            //cv::rectangle(canvas(button2), button2, cv::Scalar(0,0,255), 2);
        }
        if (buttons[2].location.contains(cv::Point(x, y)))
        {
            ros::shutdown();
        }
        if (buttons[3].location.contains(cv::Point(x, y)))
        {
            kinfu_msgs::KinfuCommand msg;
            msg.command_type=kinfu_msgs::KinfuCommand::COMMAND_TYPE_RESET;
            cmd_pub.publish(msg);
        }
        if (buttons[4].location.contains(cv::Point(x, y)))
        {
            kinfu_msgs::KinfuCommand msg;
            msg.command_type=kinfu_msgs::KinfuCommand::COMMAND_TYPE_SUSPEND;
            cmd_pub.publish(msg);
        }
        if (buttons[5].location.contains(cv::Point(x, y)))
        {
            kinfu_msgs::KinfuCommand msg;
            msg.command_type=kinfu_msgs::KinfuCommand::COMMAND_TYPE_RESUME;
            cmd_pub.publish(msg);
        }
    }
    if (event == cv::EVENT_LBUTTONUP)
    {
        //cv::rectangle(canvas, button, cv::Scalar(200, 200, 200), 2);
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Timer timer_;
  cv::Mat3b image_;
  bool received_image_;

public:
  ImageConverter()
    : it_(nh_), received_image_(false)
  {
    // Subscribe to input video feed
    image_sub_ = it_.subscribe("image", 1,
      &ImageConverter::imageCb, this);

    timer_ = nh_.createTimer(ros::Duration(0.033), &ImageConverter::timerCallback, this);

    ros::NodeHandle pnh = ros::NodeHandle("~");
    pnh.getParam("file_name",file_name);
    pnh.getParam("file_dir",file_dir);

    cmd_pub = nh_.advertise<kinfu_msgs::KinfuCommand>("command", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::setMouseCallback(OPENCV_WINDOW, callBackFunc);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(image_);



    received_image_=true;

  }

  void timerCallback(const ros::TimerEvent&)
  {
      if(!received_image_){return;}

      // Your buttons

      // The canvas
      canvas = cv::Mat3b(image_.rows + BUTTON_HEIGHT * NUM_ROWS, image_.cols, cv::Vec3b(0,0,0));

      // Draw the buttons
      for(int ii=0;ii<buttons.size();ii++){
          int a=image_.cols*(ii%BUTTONS_PER_ROW)/BUTTONS_PER_ROW;
          int b=BUTTON_HEIGHT*(ii/BUTTONS_PER_ROW);
          buttons[ii].location = cv::Rect(a,b,image_.cols/BUTTONS_PER_ROW, BUTTON_HEIGHT);
          canvas(buttons[ii].location) = buttons[ii].colour;
          putText(canvas(buttons[ii].location), buttons[ii].text, cv::Point(buttons[ii].location.width*0.35, buttons[ii].location.height*0.7), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
      }

      // Draw the image
      image_.copyTo(canvas(cv::Rect(0, BUTTON_HEIGHT * NUM_ROWS, image_.cols, image_.rows)));

      cv::imshow(OPENCV_WINDOW, canvas);
      cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
    buttons.push_back(create_button(buttonText,cv::Vec3b(200,200,200)));
    buttons.push_back(create_button("Make Textured",cv::Vec3b(200,000,200)));
    buttons.push_back(create_button("Exit",cv::Vec3b(200,200,000)));
    buttons.push_back(create_button("Reset",cv::Vec3b(000,200,200)));
    buttons.push_back(create_button("Suspend",cv::Vec3b(000,200,000)));
    buttons.push_back(create_button("Resume",cv::Vec3b(000,000,200)));


    ros::init(argc, argv, "kinfu_monitor");
    ImageConverter ic;
    ros::spin();
    return 0;
}
