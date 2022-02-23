#include <memory>
#include <vector>
#include <algorithm>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "geometry_msgs/msg/Pose.hpp"
//#include "std_msgs/msg/Header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/msg/ball_list.hpp"
#include "interfaces/msg/ball.hpp"
using std::placeholders::_1;

float longueur(std::vector<float> x, std::vector<float> y, std::vector<int> ordre)
{
  int i = ordre[-1];
  float x0 = x[i];
  float y0 = y[i];
  float d = 0;
  for (auto o:ordre)
  {
    float x1 = x[o];
    float y1 = y[o];
    d += (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1);
    x0 = x1;
    y0 = y1;
  }
  return d;
}

std::vector<int> permutation_rnd(std::vector<float> x, std::vector<float> y, std::vector<int> ordre, int miniter)
{
  float d = longueur(x, y, ordre);
  float d0 = d+1;
  int it = 1;
  while (d < d0 or it < miniter)
  {
    it += 1;
    d0 = d;
    for (int i=1; i < ordre.size()-1; i++)
    {
      for (int j=i+2; j<ordre.size()+1; j++)
      {
        int k = rand() % (ordre.size() - 1) + 1;
        int l = rand() % (ordre.size()) + k+1;
        std::vector<int> r;
        std::reverse_copy(ordre.begin()+k, ordre.begin()+l, r.begin());
        std::vector<int> ordre2;
        for (int abc = 0; abc < k; abc++)
          ordre2.push_back(ordre[abc] + r[abc] + ordre[abc+l]);
        float t = longueur(x, y, ordre2);
        if (t < d)
        {
          d = t;
          ordre = ordre2;
        }
      }
    }
  }
  return ordre;
}


std::vector<int> n_permutation(std::vector<float> x, std::vector<float> y, int miniter)
{
  std::vector<int> ordre;
  for (int i=0; i<x.size(); i++)
    ordre.push_back(i);
  std::vector<int> bordre;
  std::copy(ordre.begin(), ordre.end(), bordre.begin());
  float d0 = longueur(x, y, ordre);
  auto rng = std::default_random_engine {};
  for (int i =0; i<21; i++)
  {
    std::shuffle(std::begin(ordre), std::end(ordre), rng);
    ordre = permutation_rnd(x, y, ordre, 20);
    float d = longueur(x, y, ordre);
    if (d < d0)
    {
      d0 = d;
      std::copy(ordre.begin(), ordre.end(), bordre.begin());
    }
  }
  return bordre;
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
    }

  private:
    void topic_callback(const interfaces::msg::BallList::SharedPtr msg) const
    {
      std::vector<float> x_l{0, 0, -1};
      std::vector<float> y_l{1, -1, 1};
      std::vector<float> x_r{0, 0, 1};
      std::vector<float> y_r{1, -1, -1};
      
      for (auto ball: msg->ball_list)
      {
        std::cout << ball.x_center;
        if (ball.x_center < 0)
        {
          x_l.push_back(ball.x_center);
          y_l.push_back(ball.y_center);
        }
        else
        {
          x_r.push_back(ball.x_center);
          y_r.push_back(ball.y_center);
        }
      }
      
      
      
      auto message = geometry_msgs::msg::PoseStamped();
      auto target = geometry_msgs::msg::Pose();
      target.position.x = 5;
      target.position.y = 5;
      message.pose = target;
      
      
    }
    rclcpp::Subscription<interfaces::msg::BallList>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TravellingTraj>());
  rclcpp::shutdown();
  return 0;
}




