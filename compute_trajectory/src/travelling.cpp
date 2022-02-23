#include <memory>
#include <vector>
#include <algorithm>
#include <random>
//#include <stdio.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "geometry_msgs/msg/Pose.hpp"
//#include "std_msgs/msg/Header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/ball_list.hpp"
#include "interfaces/msg/ball.hpp"
using std::placeholders::_1;


int least(const int n, float **ary, int *completed, float *cost, int c)
{
  int i,nc=999;
  int min=999,kmin;
 
  for(i=0;i < n;i++)
    {
    if((ary[c][i]>=0)&&(completed[i]==0))
      if(ary[c][i]+ary[i][c] < min)
      {
        min=ary[i][0]+ary[c][i];
        kmin=ary[c][i];
        nc=i;
      }
    }
 
  if(min!=999)
    *cost+=kmin;
  return nc;
}

 
void mincost(const int n, float **ary, int *completed, std::vector<int> *ordre, float *cost, int city)
{
  int i,ncity;
 
  completed[city]=1;
  
  //std::cout<<city<<"--->";
  ordre->push_back(city);
  ncity=least(n, ary, completed, cost, city);
 
  if(ncity==999)
  {
    ncity=0;
    //std::cout<<ncity;
    *cost+=ary[city][ncity];
 
    return;
  }
  
  mincost(n, ary, completed, ordre, cost, ncity);
}
 


float score(float x1, float y1, float x2, float y2, float t)
{
  float s = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
  // s += 1/t;
  
  return s;
}

class TravellingTraj : public rclcpp::Node
{
  public:
    TravellingTraj()
    : Node("trajectory_planning")
    {
      subscription_ = this->create_subscription<interfaces::msg::BallList>(
        "ball_list", 10, std::bind(&TravellingTraj::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target", 10);
      robot_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot_pos", 10, std::bind(&TravellingTraj::robot_pos_callback, this, _1));
    }

  private:
    void robot_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      robot_posx = msg->pose.position.x;
      robot_posy = msg->pose.position.y;
      std::cout << "\nROBOT POS UPDATED \nx=" << robot_posx << "\ty=" << robot_posy << "\n";
    }
    
    void topic_callback(const interfaces::msg::BallList::SharedPtr msg) const
    {

      std::vector<float> x{-1, 1};
      std::vector<float> y{1, -1};
      for (auto ball: msg->ball_list)
      {
        x.push_back(ball.x_center);
        y.push_back(ball.y_center);
      }
      x.push_back(robot_posx);
      y.push_back(robot_posy);
      int n_balls = x.size();
      int n = n_balls+4;
      
      float ary[n][n];
      ary[0][0] = -1;
      ary[1][1] = -1;
      ary[0][1] = 0;
      ary[1][0] = 0;
      ary[2][2] = -1;
      ary[3][3] = -1;
      ary[2][3] = 0;
      ary[3][2] = 0;
      ary[0][2] = 10000;
      ary[0][3] = 10000;
      ary[1][2] = 10000;
      ary[1][3] = 10000;
      ary[2][0] = 10000;
      ary[2][1] = 10000;
      ary[3][0] = 10000;
      ary[3][1] = 10000;
      
      for (int i=4; i<n; i++)
      {
        for (int j=0; j<n; j++)
        {
          if (i==j)
            ary[i][i] = -1;
          else if (j<4)
          { 
            if (x[i-4] <= 0)
            {
              if (j==0)
              {
                float bscore = score(x[i-4], 0, y[i-4], 0, -1);
                ary[i][j] = bscore;
                ary[j][i] = bscore;
              }
              else if (j==2)
              {
                float bscore = score(x[i-4], 0, y[i-4], 0, -1);
                ary[i][j] = bscore;
                ary[j][i] = bscore;
              }
              else
              {
                ary[i][j] = 10000;
                ary[j][i] = 10000;
              }
            }
            else
            {
              if (j==1)
              {
                float bscore = score(x[i-4], 0, y[i-4], 0, 1);
                ary[i][j] = bscore;
                ary[j][i] = bscore;
              }
              else if (j==3)
              {
                float bscore = score(x[i-4], 0, y[i-4], 0, -1);
                ary[i][j] = bscore;
                ary[j][i] = bscore;
              }
              else
              {
                ary[i][j] = 10000;
                ary[j][i] = 10000;
              }
            }
          }
          else
          {
            if (x[i-4]*x[j-4] < 0)
            {
              ary[i][j] = 10000;
              ary[j][i] = 10000;
            }
            else
            {
              float bscore = score(x[i-4], x[j-4], y[i-4], y[j-4], 0);
              ary[i][j] = bscore;
              ary[j][i] = bscore;
            }
          }
        }
      }
      
      float *array[n];
      int completed[n];
      std::vector<int> ordre;
      float cost=0;
      
      for (int i = 0; i < n; i++)
        {
        array[i] = ary[i];
        completed[i] = 0;
        }
      
      //std::cout<<"\n\nThe cost list is:";
      //for(int i=0;i < n;i++)
      //{
      //std::cout<<"\n";
      //for(int j=0;j < n;j++)
      //std::cout<<"\t"<<array[i][j];
      //}
      //std::cout<<"\n\nThe Path is:\n";


      mincost(n, array, completed, &ordre, &cost, n-1); //passing 0 because starting vertex
     
      //std::cout<<"\n\nMinimum cost is "<<cost<<"\n";
      
      std::cout << "Zones 0-1: x=" << 0 << "  y=" << 1 << std::endl;
      std::cout << "Zones 2-3: x=" << 0 << "  y=" << -1 << std::endl;
      for (int i = 0; i<n_balls-1; i++)
      {
        std::cout << "Ball " << i+4 << ": x=" << x[i] << "  y=" << y[i] << std::endl;
      }
      std::cout << "Robot: x=" << x[n_balls-1] << "  y=" << y[n_balls-1] << std::endl;
      for (auto i:ordre)
        std::cout << i << "-";
      std::cout << "\n";
      
      auto message = geometry_msgs::msg::PoseStamped();
      auto target = geometry_msgs::msg::Pose();
      
      int ntarget = ordre[1];
      if (ntarget == 0 or ntarget == 1)
      {
        target.position.x = 0;
        target.position.y = 1;
      }
      else if (ntarget == 2 or ntarget == 3)
      {
        target.position.x = 0;
        target.position.y = -1;
      }
      else
      {
        target.position.x = x[ntarget-4];
        target.position.y = y[ntarget-4];
      }
      message.pose = target;
      publisher_->publish(message);
      
    }
    rclcpp::Subscription<interfaces::msg::BallList>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_sub_;
    float robot_posx = 0.5;
    float robot_posy = -0.12;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TravellingTraj>());
  rclcpp::shutdown();
  return 0;
}




