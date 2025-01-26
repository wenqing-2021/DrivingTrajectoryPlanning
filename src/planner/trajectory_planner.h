#pragma once
#include <string>

namespace planner {

class TrajPlanner {
  public:
    TrajPlanner(){
        // Constructor
    };
    ~TrajPlanner(){
        // Destructor
    };

    void        process();                                            // main function to generate the trajectory
    std::string getName() const { return name_; };                    // get the name of the planner
    void        setName(const std::string& name) { name_ = name; };   // set the name of the planner
  private:
    std::string name_ = "TrajPlanner";   // name of the planner


};   // class BasePlanner

}   // namespace planner