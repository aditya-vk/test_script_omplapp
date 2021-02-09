#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <Eigen/Dense>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

// \brief Wait for user input.
void waitForUser(std::string message) {
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

// \brief Check if an OMPL state is collision free.
bool isPointValid(SkeletonPtr world, SkeletonPtr robot,
                  std::shared_ptr<CollisionGroup> worldGroup,
                  std::shared_ptr<CollisionGroup> robotGroup,
                  CollisionDetectorPtr collisionDetector,
                  ::dart::collision::CollisionResult collisionResult,
                  ::dart::collision::CollisionOption collisionOptions,
                  const ompl::base::State* state) {
  // Obtain the values from state.
  auto se2State = state->as<ompl::base::SE2StateSpace::StateType>();

  // Convert positions to Eigen.
  // DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd positions(6);
  positions << 0.0, se2State->getYaw(), 0.0, se2State->getX(), 0.0,
      se2State->getY();

  // Set Positions for the robot.
  robot->setPositions(positions);

  // Check collision status at this position.
  bool collisionStatus = collisionDetector->collide(
      worldGroup.get(), robotGroup.get(), collisionOptions, &collisionResult);
  return !collisionStatus;
}

// \brief Check if an Eigen Vector corresponds to a collision free state.
bool isEigenPointValid(SkeletonPtr world, SkeletonPtr robot,
                       std::shared_ptr<CollisionGroup> worldGroup,
                       std::shared_ptr<CollisionGroup> robotGroup,
                       CollisionDetectorPtr collisionDetector,
                       ::dart::collision::CollisionResult collisionResult,
                       ::dart::collision::CollisionOption collisionOptions,
                       Eigen::VectorXd& pos) {
  waitForUser("Press Enter to Check Point");
  // Convert positions to Eigen.
  // DART To OMPL settings: angles, positions: ax, az, ay, x, z, y.
  Eigen::VectorXd positions(6);
  positions << 0.0, pos(2), 0.0, pos(0), 0.0, pos(1);

  // Set Positions for the robot.
  robot->setPositions(positions);

  // Check collisions and return the result.
  bool collisionStatus = collisionDetector->collide(
      worldGroup.get(), robotGroup.get(), collisionOptions, &collisionResult);
  return !collisionStatus;
}

// \brief Constructs a collision object from URDF.
const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri, const Eigen::Isometry3d& transform) {
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton) {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
}

// Example script.
int main(int argc, char* argv[]) {
  std::string worldName = "";
  std::string robotName = "";

  /// Load the environment.
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "ompl_app");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("ompl_env"));

  // Visualization topics
  static const std::string execTopicName = topicName + "/exec";

  // Start the RViz viewer.
  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '"
                  << execTopicName << "' InteractiveMarker topic in RViz.");
  aikido::rviz::InteractiveMarkerViewer viewer(execTopicName, baseFrameName,
                                               env);

  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever =
      std::make_shared<aikido::io::CatkinResourceRetriever>();
  const std::string worldURDFUri =
      "package://test_script_omplapp/ompl_data/" + worldName + ".urdf";
  const std::string robotURDFUri =
      "package://test_script_omplapp/ompl_data/" + robotName + ".urdf";

  // Initial perception
  SkeletonPtr world;
  Eigen::Isometry3d worldPose;

  SkeletonPtr robot;
  Eigen::Isometry3d robotPose;

  // Poses for world
  worldPose = Eigen::Isometry3d::Identity();

  // Poses for robot
  robotPose = Eigen::Isometry3d::Identity();

  // Load objects
  world = makeBodyFromURDF(resourceRetriever, worldURDFUri, worldPose);
  robot = makeBodyFromURDF(resourceRetriever, robotURDFUri, robotPose);

  // Add all objects to World
  env->addSkeleton(world);
  env->addSkeleton(robot);

  // Set the collision model.
  CollisionDetectorPtr collisionDetector =
      dart::collision::FCLCollisionDetector::create();
  ::dart::collision::CollisionOption collisionOptions =
      ::dart::collision::CollisionOption(
          false, 1,
          std::make_shared<::dart::collision::BodyNodeCollisionFilter>());
  ::dart::collision::CollisionResult collisionResult;

  std::shared_ptr<CollisionGroup> worldGroup =
      collisionDetector->createCollisionGroup(world.get());
  std::shared_ptr<CollisionGroup> robotGroup =
      collisionDetector->createCollisionGroup(robot.get());

  // Define the state space: SE(2)
  auto space = std::make_shared<ompl::base::SE2StateSpace>();
  ompl::base::RealVectorBounds bounds(2);

  bounds.setLow(0, -73.0);
  bounds.setLow(1, -179.0);
  bounds.setHigh(0, 300.0);
  bounds.setHigh(1, 168.0);

  space->setBounds(bounds);
  space->setup();

  // Space Information
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
      isPointValid, world, robot, worldGroup, robotGroup, collisionDetector,
      collisionResult, collisionOptions, std::placeholders::_1);
  si->setStateValidityChecker(isStateValid);
  space->setLongestValidSegmentFraction(10.0 / space->getMaximumExtent());
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

  ompl::base::ScopedState<ompl::base::SE2StateSpace> start(si);
  start->setX(-31.19);
  start->setY(-99.85);
  start->setYaw(0.0);

  ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(start);
  goal->setX(250.0);
  goal->setY(-47.85);
  goal->setYaw(0.0);

  pdef->addStartState(start);
  pdef->setGoalState(goal);

  // Setup planner
  ompl::geometric::BITstar planner(si);
  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status =
      planner.solve(ompl::base::plannerNonTerminatingCondition());

  return 0;
}
