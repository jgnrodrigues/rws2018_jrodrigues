#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "teams.h"
#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>

using namespace std;

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

        this->refereeSub = n.subscribe("make_a_play", 1000, &MyPlayer::move, this);

        //random position
        srand(7492 * time(NULL));

        this->x = ((double)rand() / (double)RAND_MAX) * 10 - 5;
        this->y = ((double)rand() / (double)RAND_MAX) * 10 - 5;

        this->warp();
    }

    void move(const rws2018_msgs::MakeAPlay::ConstPtr &msg)
    {
        this->x += 0.01;
        this->pubPosition();
    }

    void pubPosition(void)
    {
        transform.setOrigin(tf::Vector3(this->x, this->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, this->theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", this->name));

        ROS_INFO("Go to x=%f, y=%f, theta=%f", x, y, theta);
    }

    void warp(void)
    {
        transform.setOrigin(tf::Vector3(this->x, this->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, this->theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", this->name));

        ROS_INFO("Warp to x=%f, y=%f, theta=%f", x, y, theta);
    }

    boost::shared_ptr<Team> my_team;
    boost::shared_ptr<Team> preys;
    boost::shared_ptr<Team> hunters;

    ros::Subscriber refereeSub;
    ros::NodeHandle n;

    tf::Transform transform;
    tf::TransformBroadcaster br;
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