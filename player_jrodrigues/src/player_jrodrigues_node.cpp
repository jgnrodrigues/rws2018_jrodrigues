#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "teams.h"
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>
#include <rws2018_msgs/GameQuery.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>

#define DEFAULT_TIME 0.05
#define KP 1.5
#define KD 0.8

using namespace std;
using namespace ros;
using namespace tf;

namespace rws_jrodrigues
{
class Player
{
  public:
    Player(string name)
    {
        this->name = name;
        this->x = 5.7;
        this->y = 4.6;
        this->theta = 0.0;
    }

    //Set team name, if given a correct team name (accessor)

    int setTeamName(int team_index = 0 /*default value*/)
    {
        switch (team_index)
        {
        case RED:
            return setTeamName("red");
            break;
        case GREEN:
            return setTeamName("green");
            break;
        case BLUE:
            return setTeamName("blue");
            break;
        default:
            ROS_ERROR_STREAM("wrong team index given. Cannot set team");
            break;
        }
    }

    int setTeamName(string team)
    {
        if (team == "red" || team == "green" || team == "blue")
        {
            this->team = team;
            return 1;
        }
        else
        {
            ROS_ERROR_STREAM("cannot set team name to " << team);
            return 0;
        }
    }

    //Gets team name (accessor)
    string getTeam(void) { return team; }

    string name;
    boost::shared_ptr<Team> red_team;
    boost::shared_ptr<Team> blue_team;
    boost::shared_ptr<Team> green_team;

    double x;
    double y;
    double theta;

  private:
    string team;
};

class MyPlayer : public Player
{
  public:
    MyPlayer(string name, string team) : Player(name)
    {
        setTeamName(team);

        red_team = boost::shared_ptr<Team>(new Team("red"));
        green_team = boost::shared_ptr<Team>(new Team("green"));
        blue_team = boost::shared_ptr<Team>(new Team("blue"));

        ROS_INFO_STREAM("My name is " << this->name << " and my team is " << this->getTeam());

        if (red_team->playerBelongsToTeam("red"))
        {
            my_team = red_team;
            preys = green_team;
            hunters = blue_team;
        }

        if (red_team->playerBelongsToTeam("green"))
        {
            my_team = green_team;
            preys = blue_team;
            hunters = red_team;
        }

        if (red_team->playerBelongsToTeam("blue"))
        {
            my_team = blue_team;
            preys = red_team;
            hunters = green_team;
        }
    }

    MyPlayer(string name, int team) : Player(name)
    {
        setTeamName(team);

        red_team = boost::shared_ptr<Team>(new Team("red"));
        green_team = boost::shared_ptr<Team>(new Team("green"));
        blue_team = boost::shared_ptr<Team>(new Team("blue"));

        ROS_INFO_STREAM("My name is " << this->name << " and my team is " << this->getTeam());

        if (red_team->playerBelongsToTeam(name))
        {
            my_team = red_team;
            preys = green_team;
            hunters = blue_team;
            setTeamName("red");
        }
        else if (green_team->playerBelongsToTeam(name))
        {
            my_team = green_team;
            preys = blue_team;
            hunters = red_team;
            setTeamName("green");
        }
        else if (blue_team->playerBelongsToTeam(name))
        {
            my_team = blue_team;
            preys = red_team;
            hunters = green_team;
            setTeamName("blue");
        }

        this->refereeSub = n.subscribe("make_a_play", 1000, &MyPlayer::move, this);
        pointCloudSub = n.subscribe("/object_point_cloud", 1000, &MyPlayer::respondPointCloud, this);
        this->bocasPub = n.advertise<visualization_msgs::Marker>("/bocas", 0);
        game_query_srv = boost::shared_ptr<ros::ServiceServer>(new ros::ServiceServer());
        *game_query_srv = n.advertiseService("/" + name + "/game_query", &MyPlayer::respondQuery, this);

        //random position
        struct timeval t1;
        gettimeofday(&t1, NULL);
        srand(t1.tv_usec);
        this->last_time = ros::Time::now();
        this->last_angle = 0.0;

        this->x = ((double)rand() / (double)RAND_MAX) * 10 - 5;
        this->y = ((double)rand() / (double)RAND_MAX) * 10 - 5;

        object = "onion";

        this->warp();
        // ROS_INFO_STREAM(preys->player_names[0]);
    }

    void move(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
    {
        //----------------------------------
        // AI PART
        //----------------------------------
        double min_prey_distance = 99999;
        string player_to_hunt = "no player";
        string player_to_flee = "no player";
        double dist = 1;
        double prey_dist = 99999;
        double hunter_dist = 9999999;
        //catch prey
        for (size_t i = 0; i < msg->red_alive.size(); i++)
        {
            prey_dist = getDistanceToPlayer(msg->red_alive[i]);
            if (isnan(prey_dist))
            {
            }
            else if (prey_dist < min_prey_distance)
            {
                min_prey_distance = prey_dist;
                player_to_hunt = msg->red_alive[i];
            }
        }

        //flee hunter
        double min_hunter_distance = 99999;
        for (size_t i = 0; i < msg->green_alive.size(); i++)
        {
            hunter_dist = getDistanceToPlayer(msg->green_alive[i]);
            if (isnan(hunter_dist))
            {
            }
            else if (hunter_dist < min_hunter_distance)
            {
                min_hunter_distance = hunter_dist;
                player_to_flee = msg->green_alive[i];
            }
        }

        double delta_theta = 0;
        if ((min_prey_distance < min_hunter_distance) || (min_hunter_distance > 2.5))
        {
            min_prey_distance < 1.0 ? dist = min_prey_distance : dist = 999999;
            delta_theta = getAngleToPLayer(player_to_hunt);
        }
        else
        {
            dist = 9999999;
            delta_theta = -getAngleToPLayer(player_to_flee);
            //---------------------------
            //DON'T ESCAPE
            //--------------------------

            if (getDistanceToPlayer("world") > 7)
            {
                // delta_theta = getAngleToPLayer("world") - ((M_PI / 2) * fabs(getAngleToPLayer("world")) / getAngleToPLayer("world"));
                delta_theta = getAngleToPLayer("world") + M_PI / 2;
            }
        }

        if (isnan(delta_theta))
        {
            delta_theta = 0;
        }

        delta_theta = pd(delta_theta);
        this->last_angle = delta_theta;

        //----------------------------------
        // CONSTRAINT PART
        //----------------------------------
        double dist_max = msg->dog;
        double dist_with_constraints;

        double delta_theta_max = M_PI / 30;

        dist > dist_max ? dist = dist_max : dist = dist;
        fabs(delta_theta) > fabs(delta_theta_max) ? delta_theta = delta_theta_max * delta_theta / fabs(delta_theta) : dist = dist;

        // ROS_INFO("Go to x=%f, y=%f, theta=%f", x, y, theta);
        tf::Transform tf_move;
        tf_move.setOrigin(tf::Vector3(dist, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, delta_theta);
        tf_move.setRotation(q);

        transform = transform * tf_move;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", this->name));

        // tf::Vector3 position = transform.getOrigin();
        // this->x = position.getX();
        // this->y = position.getY();
        // this->theta = transform.getRotation().getAngle();

        // ROS_INFO("Go to x=%f, y=%f, theta=%f", x, y, theta);
        publishBoca("I'm catching " + player_to_hunt);
    }

    double getDistanceToPlayer(string other_player, double time_to_wait = DEFAULT_TIME)
    {
        StampedTransform t; //The transform object
        //Time now = Time::now(); //get the time
        Time now = Time(0); //get the latest transform received

        try
        {
            tfListener.waitForTransform(name, other_player, now, Duration(time_to_wait));
            tfListener.lookupTransform(name, other_player, now, t);
        }
        catch (TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return NAN;
        }

        return sqrt(t.getOrigin().y() * t.getOrigin().y() + t.getOrigin().x() * t.getOrigin().x());
    }

    double pd(double angle)
    {
        Time current_time = ros::Time::now();
        double dt = current_time.nsec - last_time.nsec;
        double derivative = (angle - last_angle) / dt;
        double output = KP * angle + KD * derivative;
        last_time = current_time;
        return output;
    }

    double getAngleToPLayer(string other_player, double time_to_wait = DEFAULT_TIME)
    {
        StampedTransform t; //The transform object
        //Time now = Time::now(); //get the time
        Time now = Time(0); //get the latest transform received

        try
        {
            tfListener.waitForTransform(this->name, other_player, now, Duration(time_to_wait));
            tfListener.lookupTransform(this->name, other_player, now, t);
        }
        catch (TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return NAN;
        }

        return atan2(t.getOrigin().y(), t.getOrigin().x());
    }

    void warp(void)
    {
        transform.setOrigin(tf::Vector3(this->x, this->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, this->theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", this->name));

        ROS_INFO("Warp to x=%f, y=%f, theta=%f", x, y, theta);
        publishBoca("YYYYEEEEEYYYY");
    }

    void publishBoca(string boca, int action = visualization_msgs::Marker::ADD)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = this->name;
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = 0;
        marker.ns = this->name;
        marker.action = action;

        marker.pose.position.y = 0.3;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.3;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.lifetime = ros::Duration(1.0);

        marker.text = boca;
        bocasPub.publish(marker);
    }

    void respondPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        ROS_INFO("Recebi uma PointCloud");

        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*msg, pcl_pc2);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        // double x_avg = 0;
        // for (int i = 0; i < temp_cloud->points.size(); i++)
        // {

        // }
        object = "banana";
    }

    bool respondQuery(rws2018_msgs::GameQuery::Request &req, rws2018_msgs::GameQuery::Response &res)
    {
        ROS_WARN_STREAM("I am " << name << "and I am responding to a service request");
        res.resposta = object;
        return true;
    }

    boost::shared_ptr<Team> my_team;
    boost::shared_ptr<Team> preys;
    boost::shared_ptr<Team> hunters;

    ros::NodeHandle n;
    ros::Subscriber refereeSub;
    ros::Subscriber pointCloudSub;
    ros::Publisher bocasPub;

    tf::Transform transform;
    tf::TransformBroadcaster br;
    tf::TransformListener tfListener;
    double last_angle;
    Time last_time;
    string object;
    boost::shared_ptr<ros::ServiceServer> game_query_srv;
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jrodrigues");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    string player_name = "jrodrigues";

    //Creating an instance of class Player
    string team;
    n.getParam("team", team);
    rws_jrodrigues::MyPlayer player(player_name, BLUE);

    ros::spin();

    // while (ros::ok())
    // {
    //     // player.move();

    //     ros::spinOnce();

    //     loop_rate.sleep();
    // }
}