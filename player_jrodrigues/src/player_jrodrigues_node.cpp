#include <iostream>

#include "teams.h"

class Player
{
  public:
    Player(std::string name)
    {
        this->name = name;
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
            std::cout << "wrong team index given. Cannot set team" << std::endl;
            break;
        }
    }

    int setTeamName(std::string team)
    {
        if (team == "red" || team == "green" || team == "blue")
        {
            this->team = team;
            return 1;
        }
        else
        {
            std::cout << "cannot set team name to " << team << std::endl;
            return 0;
        }
    }

    //Gets team name (accessor)
    std::string getTeam(void) { return team; }

    std::string name;

  private:
    std::string team;
};

class MyPlayer : public Player
{
  public:
    MyPlayer(std::string name, std::string team) : Player(name)
    {
        setTeamName(team);
    }

    MyPlayer(std::string name, int team) : Player(name)
    {
        setTeamName(team);
    }
};

int main()
{

    std::string player_name = "jrodrigues";
    //Creating an instance of class Player
    Player player(player_name);
    player.setTeamName(RED);

    std::cout << "player.name is " << player.name << std::endl;
    std::cout << "team is " << player.getTeam() << std::endl;
}