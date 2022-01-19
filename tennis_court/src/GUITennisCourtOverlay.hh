#ifndef _GUI_TENNIS_COURT_OVERLAY_HH__
#define _GUI_TENNIS_COURT_OVERLAY_HH__

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo_ros/node.hpp>
#include <tennis_court/msg/ball_manager_stats.hpp>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
  class GAZEBO_VISIBLE GUITennisCourtOverlay : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: GUITennisCourtOverlay();

    /// A pointer to the GazeboROS node.
    public: gazebo_ros::Node::SharedPtr ros_node_;

    /// Subscriber to command velocities
    public: rclcpp::Subscription<tennis_court::msg::BallManagerStats>::SharedPtr ball_manager_stats_sub_;

    public: void OnBallManagerStats(const tennis_court::msg::BallManagerStats::SharedPtr _msg);

    /// \brief Destructor
    public: virtual ~GUITennisCourtOverlay();

    signals: void SetBallCount(QString _string);

    signals: void SetScore(QString _string);

    signals: void SetCollectedBalls(QString _string);

    protected: void Load(sdf::ElementPtr _sdf) override;

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;
  };
}

#endif