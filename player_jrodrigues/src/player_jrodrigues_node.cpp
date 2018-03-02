#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "teams.h"
#include <rws2018_libs/team.h>
#include <tf/transform_broadcaster.h>

using namespace std;

namespace rws_jrodrigues
{
// class Team
// {
//   public:
//     /**
//        * @brief Constructor
//        * @param team_name the team name
//        */
//     Team(string team_name)
//     {
//         name = team_name;
//     }

//     /**
//        * @brief Prints the name of the team and the names of all its players
//        */
//     void printTeamInfo(void)
//     {
//         //TODO
//     }

//     /**
//        * @brief Checks if a player belongs to the team
//        * @param player_name the name of the player to check
//        * @return true or false, yes or no
//        */
//     bool playerBelongsToTeam(string player_name)
//     {
//         //write code here ...
//     }

//     /**
//        * @brief The team name
//        */
//     string name;

//     /**
//        * @brief A list of the team's player names
//        */
//     vector<string> players;
// };

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
            cout << "wrong team index given. Cannot set team" << endl;
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
            cout << "cannot set team name to " << team << endl;
            return 0;
        }
    }

    void move(void)
    {
        static tf::Transform transform;
        transform.setOrigin(tf::Vector3(this->x, this->y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, this->theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", this->name));
    }

    //Gets team name (accessor)
    string getTeam(void) { return team; }

    string name;
    boost::shared_ptr<Team> red_team;
    boost::shared_ptr<Team> blue_team;
    boost::shared_ptr<Team> green_team;
    tf::TransformBroadcaster br;

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

        cout << "player.name is " << this->name << endl;
        cout << "team is " << this->getTeam() << endl;
    }

    MyPlayer(string name, int team) : Player(name)
    {
        setTeamName(team);

        red_team = boost::shared_ptr<Team>(new Team("red"));
        green_team = boost::shared_ptr<Team>(new Team("green"));
        blue_team = boost::shared_ptr<Team>(new Team("blue"));

        cout << "player.name is " << this->name << endl;
        cout << "team is " << this->getTeam() << endl;

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

    boost::shared_ptr<Team> my_team;
    boost::shared_ptr<Team> preys;
    boost::shared_ptr<Team> hunters;
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

    // cout << "Read team: " << team << endl;
    // ros::spin();

    while (ros::ok())
    {
        player.move();

        ros::spinOnce();

        loop_rate.sleep();
    }
}