#include <sstream>
#include <string>
#include <gazebo/msgs/msgs.hh>
#include "GUITennisCourtOverlay.hh"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(GUITennisCourtOverlay)

GUITennisCourtOverlay::GUITennisCourtOverlay() : GUIPlugin()
{
  this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QFrame *mainFrame = new QFrame();

  QVBoxLayout *vBoxLayout = new QVBoxLayout();

  QFrame *scoreFrame = new QFrame();
  scoreFrame->setStyleSheet("font-weight: bold; font-size: 18pt");
  QHBoxLayout *scoreLayout = new QHBoxLayout();
  QLabel *scoreTitleLabel = new QLabel(tr("Score:"));
  QLabel *scoreLabel = new QLabel(tr("0"));
  scoreLayout->addWidget(scoreTitleLabel);
  scoreLayout->addWidget(scoreLabel);
  connect(this, SIGNAL(SetScore(QString)), scoreLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  scoreFrame->setLayout(scoreLayout);

  QFrame *ballCountFrame = new QFrame();
  QHBoxLayout *ballCountLayout = new QHBoxLayout();
  QLabel *ballCountTitleLabel = new QLabel(tr("Remaining balls:"));
  QLabel *ballCountLabel = new QLabel(tr("0"));
  ballCountLayout->addWidget(ballCountTitleLabel);
  ballCountLayout->addWidget(ballCountLabel);
  connect(this, SIGNAL(SetBallCount(QString)), ballCountLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  ballCountFrame->setLayout(ballCountLayout);

  QFrame *collectedBallsFrame = new QFrame();
  QHBoxLayout *collectedBallsLayout = new QHBoxLayout();
  QLabel *collectedBallsTitleLabel = new QLabel(tr("Collected balls:"));
  QLabel *collectedBallsLabel = new QLabel(tr("0"));
  collectedBallsLayout->addWidget(collectedBallsTitleLabel);
  collectedBallsLayout->addWidget(collectedBallsLabel);
  connect(this, SIGNAL(SetCollectedBalls(QString)), collectedBallsLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  collectedBallsFrame->setLayout(collectedBallsLayout);

  QFrame *gameStatusFrame = new QFrame();
  QHBoxLayout *gameStatusLayout = new QHBoxLayout();
  QLabel *gameStatusTitleLabel = new QLabel(tr("Game status:"));
  QLabel *gameStatusLabel = new QLabel(tr("---"));
  gameStatusLayout->addWidget(gameStatusTitleLabel);
  gameStatusLayout->addWidget(gameStatusLabel);
  connect(this, SIGNAL(SetGameStatus(QString)), gameStatusLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  gameStatusFrame->setLayout(gameStatusLayout);

  QFrame *timeLeftFrame = new QFrame();
  QHBoxLayout *timeLeftLayout = new QHBoxLayout();
  QLabel *timeLeftLabel = new QLabel(tr("---"));
  timeLeftLayout->addWidget(timeLeftLabel);
  connect(this, SIGNAL(SetTimeLeft(QString)), timeLeftLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  timeLeftFrame->setLayout(timeLeftLayout);

  QFrame *robotSafetyFrame = new QFrame();
  QHBoxLayout *robotSafetyLayout = new QHBoxLayout();
  QLabel *robotSafetyLabel = new QLabel(tr("---"));
  robotSafetyLayout->addWidget(robotSafetyLabel);
  connect(this, SIGNAL(SetRobotSafety(QString)), robotSafetyLabel, SLOT(setText(QString)), Qt::QueuedConnection);
  robotSafetyFrame->setLayout(robotSafetyLayout);

  vBoxLayout->addWidget(scoreFrame);
  vBoxLayout->addWidget(ballCountFrame);
  vBoxLayout->addWidget(collectedBallsFrame);
  vBoxLayout->addWidget(gameStatusFrame);
  vBoxLayout->addWidget(timeLeftFrame);
  vBoxLayout->addWidget(robotSafetyFrame);

  mainFrame->setLayout(vBoxLayout);

  mainLayout->addWidget(mainFrame);

  scoreLayout->setContentsMargins(6, 6, 6, 0);
  ballCountLayout->setContentsMargins(6, 0, 6, 0);
  collectedBallsLayout->setContentsMargins(6, 0, 6, 0);
  gameStatusLayout->setContentsMargins(6, 0, 6, 0);
  timeLeftLayout->setContentsMargins(6, 0, 6, 0);
  robotSafetyLayout->setContentsMargins(6, 0, 6, 6);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  this->move(10, 10);
  this->resize(240, 240);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
}

void GUITennisCourtOverlay::Load(sdf::ElementPtr _sdf)
{
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  this->ball_manager_stats_sub_ = this->ros_node_->create_subscription<tennis_court::msg::BallManagerStats>(
    "ball_manager_stats", rclcpp::QoS(1).transient_local().best_effort(),
    std::bind(&GUITennisCourtOverlay::OnBallManagerStats, this, std::placeholders::_1));

  RCLCPP_INFO(this->ros_node_->get_logger(), "Subscribed to [%s]", this->ball_manager_stats_sub_->get_topic_name());
}

void GUITennisCourtOverlay::OnBallManagerStats(const tennis_court::msg::BallManagerStats::SharedPtr _msg)
{
  std::string game_status;
  std::string robot_safety;
  switch (_msg->game_status)
  {
    case tennis_court::msg::BallManagerStats::GAME_STARTING:
      game_status.assign("Starting");
      robot_safety.append(_msg->robot_is_safe ?
      "<span style=\"color:#2bff00\";>Robot should stay in safe area</span>" : 
      "<span style=\"color:orange\";>Robot should reach safe area</span>");
      break;
    case tennis_court::msg::BallManagerStats::GAME_STARTED:
      game_status.assign("Started");
      robot_safety.append(_msg->robot_is_safe ?
      "<span style=\"color:#2bff00\";>Robot should stay in safe area</span>" : 
      "<span style=\"color:red\";>Robot should be in safe area</span>");
      break;
    case tennis_court::msg::BallManagerStats::GAME_PAUSED:
      game_status.assign("Paused");
      robot_safety.append("Robot is free to move");
      break;
    default:
      game_status.assign("---");
      break;
  }
  
  std::string time_left;
  float time_left_data = _msg->status_time_left;
  std::stringstream stream;
  stream << std::fixed << std::setprecision(2) << time_left_data;
  time_left.append(stream.str());  
  time_left.append("s left");

  this->SetScore(QString::fromStdString(std::to_string(_msg->score)));
  this->SetBallCount(QString::fromStdString(std::to_string(_msg->current_ball_count)));
  this->SetCollectedBalls(QString::fromStdString(std::to_string(_msg->total_ball_count - _msg->current_ball_count)));
  this->SetGameStatus(QString::fromStdString(game_status));
  this->SetTimeLeft(QString::fromStdString(time_left));
  this->SetRobotSafety(QString::fromStdString(robot_safety));
}

GUITennisCourtOverlay::~GUITennisCourtOverlay()
{
}
