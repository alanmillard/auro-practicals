// Based on https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/include/gazebo_plugins/gazebo_ros_template.hpp

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ITEM_MANAGER_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ITEM_MANAGER_HPP_

#include <gazebo/common/Plugin.hh>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_plugins
{

const double HOME_ZONE_MIN_X = -4.0;
const double HOME_ZONE_MAX_X = -3.0;
const double HOME_ZONE_MIN_Y = -3.0;
const double HOME_ZONE_MAX_Y =  3.0;

enum Colour
{
    RED,
    GREEN,
    BLUE,
    NUM_COLOURS
};

std::string to_string(Colour colour)
{
    std::string result;

    switch(colour)
    {
        case Colour::RED:
            result = "RED";
            break;
        case Colour::GREEN:
            result = "GREEN";
            break;
        case Colour::BLUE:
            result = "BLUE";
            break;
        default:
            break;
    }

    return result;
}

class Item
{
public:

    Item(double x, double y, Colour colour, std::string cluster_id):
        x(x),
        y(y),
        colour(colour),
        cluster_id(cluster_id) {}

    friend std::ostream& operator<<(std::ostream& os, const Item& item)
    {
        os << "(";
        os << std::fixed << std::setprecision(2) << item.x;
        os << ", ";
        os << std::fixed << std::setprecision(2) << item.y;
        os << "), ";
        os << to_string(item.colour);
        os << ", ";
        os << item.cluster_id;
        return os;
    }

    double x;
    double y;
    Colour colour;
    std::string cluster_id;
};

class Cluster
{
public:

    Cluster(double x, double y, Colour colour):
        x(x),
        y(y),
        colour(colour) {}

    friend std::ostream& operator<<(std::ostream& os, const Cluster& cluster)
    {
        os << "(" << cluster.x << ", " << cluster.y << "), " << to_string(cluster.colour);
        return os;
    }

    double x;
    double y;
    Colour colour;
};

class Robot
{
public:

    Robot() {}

    friend std::ostream& operator<<(std::ostream& os, const Robot& robot)
    {
        os << robot.holding_item << ", " << robot.item_held << ", " << robot.previous_item_held;
        return os;
    }

    bool holding_item = false;
    std::string item_held;
    std::string previous_item_held;
};

// Forward declaration of private data class.
class GazeboRosItemManagerPrivate;

class GazeboRosItemManager : public gazebo::WorldPlugin
{
public:
    /// Constructor
    GazeboRosItemManager();

    /// Destructor
    virtual ~GazeboRosItemManager();

    std::pair<int, int> generate_cluster_location(Colour colour);
    std::pair<double, double> generate_item_position(std::string cluster_id);

    /// Gazebo calls this when the plugin is loaded.
    /// \param[in] _world Pointer to the World.
    /// \param[in] _sdf Pointer the the SDF element of the plugin containing user-defined parameters.
    void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

protected:
    /// Optional callback to be called at every simulation iteration.
    virtual void OnUpdate();

private:
    /// Recommended PIMPL pattern. This variable should hold all private
    /// data members.
    std::unique_ptr<GazeboRosItemManagerPrivate> impl_;
};
} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS__GAZEBO_ROS_ITEM_MANAGER_HPP_
