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

int texture_obj(){
    ROS_ERROR("NOT YET IMPLEMENTED");
    return 0;
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
