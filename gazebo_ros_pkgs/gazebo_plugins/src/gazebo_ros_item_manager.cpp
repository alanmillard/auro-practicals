// Based on https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_template.cpp

#include <gazebo_plugins/gazebo_ros_item_manager.hpp>

#include <gazebo/common/ModelDatabase.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <assessment_interfaces/msg/item_holder.hpp>
#include <assessment_interfaces/msg/item_holders.hpp>
#include <assessment_interfaces/msg/item_log.hpp>

#include <memory>

namespace gazebo_plugins
{
// Class to hold private data members (PIMPL pattern)
class GazeboRosItemManagerPrivate
{
public:
    // Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    // Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;

    gazebo::physics::WorldPtr world_;

    rclcpp::Publisher<assessment_interfaces::msg::ItemLog>::SharedPtr item_log_publisher;
    rclcpp::Publisher<assessment_interfaces::msg::ItemHolders>::SharedPtr item_holders_publisher;

    gazebo::common::Time previous_time;

    std::map<std::string, Cluster> clusters;
    std::map<std::string, Robot> robots;
    std::map<std::string, Item> items;

    int items_returned[3];
    int item_values[3];

    int cluster_counter = 0;
    int item_counter = 0;    
};

GazeboRosItemManager::GazeboRosItemManager()
: impl_(std::make_unique<GazeboRosItemManagerPrivate>())
{
}

GazeboRosItemManager::~GazeboRosItemManager()
{
}

std::pair<int, int> GazeboRosItemManager::generate_cluster_location(Colour colour)
{
    int x, y;

    while(true)
    {
        // Randomly generate a candidate cluster location
        y = ignition::math::Rand::IntUniform(-2, 2);

        switch(colour)
        {
            case RED:
                x = ignition::math::Rand::IntUniform(-2, 1);
                break;
            case GREEN:
                x = ignition::math::Rand::IntUniform(-1, 1);
                break;
            case BLUE:
                x = ignition::math::Rand::IntUniform(1, 2);
                break;
            default:
                break;
        }

        // Invalid if it clashes with an obstacle
        if((x == 1 && y == 1) ||
           (x == 1 && y == -1) ||
           (x == -1 && y == 1) ||
           (x == -1 && y == -1))
            continue;

        // Invalid if it clashes with an existing cluster
        bool invalid = false;

        for(const auto& [cluster_id, cluster]: impl_->clusters)
        {
            ignition::math::Vector2d cluster_xy(cluster.x, cluster.y);

            if((x == cluster.x && y == cluster.y) ||
               (cluster_xy.Distance(ignition::math::Vector2d(x, y)) <= 1.0))
            {
               invalid = true;
               break;
            }
        }

        if(!invalid)
            return {x, y};
    }
}

std::pair<double, double> GazeboRosItemManager::generate_item_position(std::string cluster_id)
{
    double x, y;

    while(true)
    {
        double radius = ignition::math::Rand::DblUniform(0, 0.5);
        
        ignition::math::Angle radians;
        radians.SetDegree(ignition::math::Rand::DblUniform(0, 360));
        double angle = radians.Radian();

        Cluster& cluster = impl_->clusters.find(cluster_id)->second;

        x = cluster.x + radius * cos(angle);
        y = cluster.y + radius * sin(angle);

        bool invalid = false;

        for(const auto& [item_id, item]: impl_->items)
        {
            ignition::math::Vector2d item_xy(item.x, item.y);

            if(item.cluster_id == cluster_id &&
               item_xy.Distance(ignition::math::Vector2d(x, y)) < 0.3)
            {
               invalid = true;
               break;
            }
        }

        if(!invalid)
            return {x, y};
    }
}

void GazeboRosItemManager::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    // Create a GazeboRos node instead of a common ROS node.
    // Pass it SDF parameters so common options like namespace and remapping
    // can be handled.
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Loaded item manager plugin");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Gazebo random seed: %d", ignition::math::Rand::Seed());

    impl_->world_ = _world;
    impl_->previous_time = impl_->world_->SimTime();

    impl_->item_log_publisher = impl_->ros_node_->create_publisher<assessment_interfaces::msg::ItemLog>("/item_log", 10);
    impl_->item_holders_publisher = impl_->ros_node_->create_publisher<assessment_interfaces::msg::ItemHolders>("/item_holders", 10);

    for(int i; i < NUM_COLOURS; i++)
        impl_->items_returned[i] = 0;

    impl_->item_values[RED] = 5;
    impl_->item_values[GREEN] = 10;
    impl_->item_values[BLUE] = 15;

    // Load item model SDF from file based on URI
    sdf::SDFPtr item_sdf;

    item_sdf.reset(new sdf::SDF);
    sdf::initFile("root.sdf", item_sdf);

    std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://item");

    if(!sdf::readFile(filename, item_sdf))
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Unable to read SDF file: %s", filename.c_str());

    // Get pointer to material script element
    sdf::ElementPtr model = item_sdf->Root()->FindElement("model");
    sdf::ElementPtr link = model->FindElement("link");
    sdf::ElementPtr visual = link->FindElement("visual");
    sdf::ElementPtr material = visual->FindElement("material");
    sdf::ElementPtr script = material->FindElement("script");
    sdf::ElementPtr name = script->FindElement("name");

    for(int i = 0; i < 6; i++)
    {
        bool valid = false;
        Colour colour = Colour(ignition::math::Rand::IntUniform(0, NUM_COLOURS - 1));

        while(!valid)
        {
            int count = 0;

            for(const auto& [cluster_id, cluster]: impl_->clusters)
            {
                if(colour == cluster.colour)
                    count++;
            }

            if(count < 2)
                valid = true;
            else
                colour = Colour(ignition::math::Rand::IntUniform(0, NUM_COLOURS - 1));
        }

        std::string cluster_id = "cluster" + std::to_string(impl_->cluster_counter);
        impl_->cluster_counter++;

        auto [x, y] = generate_cluster_location(colour);
        impl_->clusters.insert({cluster_id, Cluster(x, y, colour)});

        for(int j = 0; j < ignition::math::Rand::IntUniform(3, 5); j++)
        {
            std::string item_id = "item" + std::to_string(impl_->item_counter);
            impl_->item_counter++;

            auto [x, y] = generate_item_position(cluster_id);
            impl_->items.insert({item_id, Item(x, y, colour, cluster_id)});

            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Spawning %s %s at (%.2f, %.2f)", to_string(colour).c_str(), item_id.c_str(), x, y);

            std::string material;

            switch(colour)
            {
                case RED:
                    material = "red_outlined";
                    break;
                case GREEN:
                    material = "green_outlined";
                    break;
                case BLUE:
                    material = "blue_outlined";
                    break;
                default:
                    break;
            }

            // Edit SDF material, name, and pose, then insert the model
            name->Set<std::string>(material);
            model->GetAttribute("name")->SetFromString(item_id);
            model->GetElement("pose")->Set(ignition::math::Pose3d(x, y, 0.0, 0.0, 0.0, 0.0));
            _world->InsertModelSDF(*item_sdf);
        }
    }

    name->Set<std::string>("red_outlined");
    model->GetAttribute("name")->SetFromString("ready");
    model->GetElement("pose")->Set(ignition::math::Pose3d(0.0, 0.0, -0.5, 0.0, 0.0, 0.0));
    _world->InsertModelSDF(*item_sdf);

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosItemManager::OnUpdate, this));
}

void GazeboRosItemManager::OnUpdate()
{
    std::vector<gazebo::physics::ModelPtr> models = impl_->world_->Models();

    for(auto& model: models)
    {
        std::string name = model->GetName();

        if(name.find("robot") != std::string::npos)
            impl_->robots.insert({name, Robot()});
    }

    for(auto& [robot_id, robot]: impl_->robots)
    {
        gazebo::physics::EntityPtr robot_entity = impl_->world_->EntityByName(robot_id);
        ignition::math::Pose3d robot_pose = robot_entity->WorldPose();
        ignition::math::Vector2d robot_xy(robot_pose.Pos().X(), robot_pose.Pos().Y());

        for(auto& [item_id, item]: impl_->items)
        {
            if(item_id == robot.item_held)
                continue;

            gazebo::physics::EntityPtr item_entity = impl_->world_->EntityByName(item_id);
            ignition::math::Pose3d item_pose = item_entity->WorldPose();

            double distance = robot_xy.Distance(ignition::math::Vector2d(item_pose.Pos().X(), item_pose.Pos().Y()));
            
            if(distance < 0.15)
            {
                if(!robot.holding_item)
                {
                    RCLCPP_INFO(impl_->ros_node_->get_logger(), "%s collected %s", robot_id.c_str(), item_id.c_str());

                    robot.holding_item = true;
                    robot.item_held = item_id;

                    break;
                }
                else // robot.holding_item
                {
                    Item& robot_item = impl_->items.find(robot.item_held)->second;

                    if(robot_item.colour == item.colour)
                        break;

                    gazebo::common::Time time_difference = impl_->world_->SimTime() - impl_->previous_time;

                    if((robot.previous_item_held != item_id) ||
                       ((robot.previous_item_held == item_id) && (time_difference.sec >= 5)))
                    {
                        RCLCPP_INFO(impl_->ros_node_->get_logger(), "%s swapped %s with %s", robot_id.c_str(), robot.item_held.c_str(), item_id.c_str());

                        gazebo::physics::ModelPtr robot_item_model = impl_->world_->ModelByName(robot.item_held);
                        robot_item_model->SetWorldPose(item_pose);

                        robot_item.x = item_pose.Pos().X();
                        robot_item.y = item_pose.Pos().Y();

                        // Swap cluster membership
                        std::string item_held_cluster_id = robot_item.cluster_id;
                        robot_item.cluster_id = item.cluster_id;
                        item.cluster_id = item_held_cluster_id;

                        impl_->previous_time = impl_->world_->SimTime();
                        robot.previous_item_held = robot.item_held;
                        robot.item_held = item_id;

                        break;
                    }
                }
            }
        }

        if(robot.holding_item)
        {
            if(robot_pose.Pos().X() > HOME_ZONE_MIN_X &&
               robot_pose.Pos().X() < HOME_ZONE_MAX_X &&
               robot_pose.Pos().Y() > HOME_ZONE_MIN_Y &&
               robot_pose.Pos().Y() < HOME_ZONE_MAX_Y)
            {
                RCLCPP_INFO(impl_->ros_node_->get_logger(), "%s returned %s home", robot_id.c_str(), robot.item_held.c_str());

                Item& robot_item = impl_->items.find(robot.item_held)->second;
                impl_->items_returned[robot_item.colour] += 1;

                auto [x, y] = generate_item_position(robot_item.cluster_id);

                gazebo::physics::ModelPtr robot_item_model = impl_->world_->ModelByName(robot.item_held);
                robot_item_model->SetWorldPose(ignition::math::Pose3d(x, y, 0.0, 0.0, 0.0, 0.0));

                robot_item.x = x;
                robot_item.y = y;

                robot.holding_item = false;
                robot.item_held = "";
                robot.previous_item_held = "";
            }
            else
            {                
                gazebo::physics::ModelPtr robot_model = impl_->world_->ModelByName(robot_id);
                gazebo::physics::ModelPtr robot_item_model = impl_->world_->ModelByName(robot.item_held);
                gazebo::physics::LinkPtr robot_link = robot_model->GetLink("base_scan");

                gazebo::physics::EntityPtr item_entity = impl_->world_->EntityByName(robot.item_held);
                ignition::math::Pose3d item_pose = item_entity->WorldPose();

                item_pose.Pos().X() = robot_link->WorldCoGPose().Pos().X();
                item_pose.Pos().Y() = robot_link->WorldCoGPose().Pos().Y();
                item_pose.Pos().Z() = 0.15;

                robot_item_model->SetWorldPose(item_pose);

                Item& robot_item = impl_->items.find(robot.item_held)->second;

                robot_item.x = item_pose.Pos().X();
                robot_item.y = item_pose.Pos().Y();
            }
        }
    }

    // Publish item holders
    assessment_interfaces::msg::ItemHolders item_holders;

    for(auto& [robot_id, robot]: impl_->robots)
    {
        assessment_interfaces::msg::ItemHolder item_holder;
        item_holder.robot_id = robot_id;

        if(!robot.holding_item)
        {
            item_holder.holding_item = false;
            item_holder.item_colour = "";
            item_holder.item_value = 0;
        }
        else // robot.holding_item
        {
            Item& item = impl_->items.find(robot.item_held)->second;

            item_holder.holding_item = true;
            item_holder.item_colour = to_string(item.colour);
            item_holder.item_value = impl_->item_values[item.colour];
        }

        item_holders.data.push_back(item_holder);
    }

    impl_->item_holders_publisher->publish(item_holders);

    // Publish item log
    assessment_interfaces::msg::ItemLog item_log;

    item_log.red_count = impl_->items_returned[RED];
    item_log.green_count = impl_->items_returned[GREEN];
    item_log.blue_count = impl_->items_returned[BLUE];
    item_log.total_count = item_log.red_count + item_log.green_count + item_log.blue_count;

    item_log.red_value = item_log.red_count * impl_->item_values[RED];
    item_log.green_value = item_log.green_count * impl_->item_values[GREEN];
    item_log.blue_value = item_log.blue_count * impl_->item_values[BLUE];
    item_log.total_value = item_log.red_value + item_log.green_value + item_log.blue_value;

    impl_->item_log_publisher->publish(item_log);
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosItemManager)
}  // namespace gazebo_plugins
