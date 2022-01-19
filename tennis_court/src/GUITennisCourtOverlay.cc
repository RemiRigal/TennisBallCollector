#include <sstream>
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

  vBoxLayout->addWidget(scoreFrame);
  vBoxLayout->addWidget(ballCountFrame);
  vBoxLayout->addWidget(collectedBallsFrame);

  mainFrame->setLayout(vBoxLayout);

  mainLayout->addWidget(mainFrame);

  scoreLayout->setContentsMargins(6, 6, 6, 0);
  ballCountLayout->setContentsMargins(6, 0, 6, 0);
  collectedBallsLayout->setContentsMargins(6, 0, 6, 6);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  this->move(10, 10);
  this->resize(180, 120);

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
    this->SetScore(QString::fromStdString(std::to_string(_msg->score)));
    this->SetBallCount(QString::fromStdString(std::to_string(_msg->current_ball_count)));
    this->SetCollectedBalls(QString::fromStdString(std::to_string(_msg->total_ball_count - _msg->current_ball_count)));
}

GUITennisCourtOverlay::~GUITennisCourtOverlay()
{
}
