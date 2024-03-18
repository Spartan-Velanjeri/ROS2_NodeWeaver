//   Copyright 2024 Robert Bosch GmbH and its subsidiaries

//  All rights reserved, also regarding any disposal, exploitation, reproduction,
//  editing, distribution, as well as in the event of applications for industrial
//  property rights.

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <unistd.h>
#include <memory>

namespace ignition
{
namespace gazebo
{

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class LocalPosePublisher
  : public System,
  public ISystemConfigure,
  public ISystemPostUpdate
{
private:
  Entity linkEntity;

  Model model;

  std::string linkName;

  bool flag_root_link = false;

  bool flag_user_command = false;

  bool get_link = false;

  bool model_initialized = false;

  // Create a transport node and advertise a topic.
  ignition::transport::Node publisher_node;

  // Creating the publisher
  ignition::transport::Node::Publisher pose_publisher;

  // Create a transport node and subscribe to a topic.
  transport::Node subscriber_node;

  std::string subscriber_topic_user_command;

  // Implement Configure callback, provided by ISystemConfigure and called once at startup.
  void Configure(
    const Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    EntityComponentManager & _ecm,
    EventManager & /*_eventMgr*/)
  {
    // Create model object, to access convenient functions
    model = Model(_entity);

    // Read property from SDF
    subscriber_topic_user_command = _sdf->Get<std::string>("set_subscriber_topic_name");

    if (!model.Valid(_ecm)) {
      ignerr << "LocalPosePublisher plugin should be attached to a model entity. " <<
        "Failed to initialize." << std::endl;
      return;
    } else {
      ignmsg << "The LocalPosePublisher plugin is properly attached to the model entity " <<
        "and has been initialized successfully." << std::endl;
    }
    auto subscriber_callback = std::function<void(const msgs::StringMsg &)>(
      [this](const auto & _msg)
      {
        // Get link name as user command
        linkName = _msg.data();
        flag_user_command = true;
        get_link = true;
      });

    // Subscribe to a topic by registering a callback.
    if (!subscriber_node.Subscribe(subscriber_topic_user_command, subscriber_callback)) {
      ignerr << "Error subscribing to topic [" << subscriber_topic_user_command << "]" << std::endl;
      return;
    }
  }
  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  void PostUpdate(
    const UpdateInfo & /*_info*/,
    const EntityComponentManager & _ecm)
  {
    if (!model_initialized) {
      InitializeEntities(_ecm);
      model_initialized = true;
    }

    if (get_link) {
      // Get link entity
      linkEntity = model.LinkByName(_ecm, linkName);
      get_link = false;
    }

    if (flag_root_link || flag_user_command) {
      // Convert from math::Pose3d to msgs::Pose
      msgs::Pose msg = msgs::Convert(worldPose(linkEntity, _ecm));
      auto header = msg.mutable_header();
      auto frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value(linkName);

      // Get link pose and print it
      // ignmsg << worldPose(linkEntity, _ecm) << std::endl;

      pose_publisher.Publish(msg);
      // ignmsg << "Publishing hello on topic [" << publisher_topic << "]" << std::endl;
    }
  }
#include <string>
  void CreatePublisher(std::string publisherTopicName)
  {
    pose_publisher = publisher_node.Advertise<msgs::Pose>(publisherTopicName);
    get_link = true;
  }
#include <stack>
#include <vector>
  void InitializeEntities(const EntityComponentManager & _ecm)
  {
    std::stack<Entity> toCheck;
    toCheck.push(this->model.Entity());
    std::vector<Entity> visited;
    while (!toCheck.empty()) {
      Entity entity = toCheck.top();
      toCheck.pop();
      visited.push_back(entity);

      auto joint = _ecm.Component<components::Joint>(entity);

      // The default link is taken as the parent link
      if (joint) {
        flag_root_link = true;
        // ignerr << "Entering hopefully :(" << std::endl;
        linkName = _ecm.Component<components::ParentLinkName>(entity)->Data();
        // ignerr << linkName << std::endl;
        CreatePublisher("/reference_frame_pose");
        return;
      }

      // Recursively check if child entities need to be published
      auto childEntities =
        _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

      // Use reverse iterators to match the order of entities found so as to match
      // the expected order in the pose_publisher integration test.
      for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend(); ++childIt) {
        auto it = std::find(visited.begin(), visited.end(), *childIt);
        if (it == visited.end()) {
          // Only add to stack if the entity hasn't been already been visited.
          // This also ensures there are no cycles.
          toCheck.push(*childIt);
        }
      }
    }
    CreatePublisher("/marker_pose");
  }
};

}  // namespace gazebo
}  // namespace ignition

// Register plugin
IGNITION_ADD_PLUGIN(
  ignition::gazebo::LocalPosePublisher,
  ignition::gazebo::System,
  ignition::gazebo::LocalPosePublisher::ISystemConfigure,
  ignition::gazebo::LocalPosePublisher::ISystemPostUpdate)
// Add plugin alias so that we can refer to the plugin without the version namespace
IGNITION_ADD_PLUGIN_ALIAS(
  ignition::gazebo::LocalPosePublisher, "ignition::gazebo::systems::LocalPosePublisher")
