
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <string>
#include <ompl/config.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <functional>
using namespace std ;
using namespace boost;

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace cv;
Mat img ;
class Plane2DEnvironment
{
public:
    

    Plane2DEnvironment()
    {
           img= imread("../rrt.png");
           bool ok = true;
        
        if (ok)
        {
            auto ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(0.0, img.cols);
            space->addDimension(0.0, img.rows);
            maxWidth_ = img.cols - 1;
            maxHeight_ = img.rows - 1;
            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));
            ss_->setStateValidityChecker(boost::bind(&Plane2DEnvironment::isStateValid, this, _1));
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
             ss_->setPlanner(ob::PlannerPtr(new og::RRTConnect(ss_->getSpaceInformation())));
   
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
           {
            if (ss_->getPlanner())
                ss_->getPlanner()->clear();
            ss_->solve();}
       
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath())
        {
            ss_->simplifySolution();
            og::PathGeometric &p = ss_->getSolutionPath();
            ss_->getPathSimplifier()->simplifyMax(p);
            ss_->getPathSimplifier()->smoothBSpline(p);
            return true;
        }
        
            return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            img.at<Vec3b>(h, w)[2] = 255;
            img.at<Vec3b>(h,w)[1]=0;
            img.at<Vec3b>(h,w)[0]=0;
            
        }
    }


private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);
        return img.at<Vec3b>(h, w)[2] >=200 && img.at<Vec3b>(h, w)[1] >=200  && img.at<Vec3b>(h, w)[0] >=200 ;

    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
   

};

int main(int argc,char **argv)
{
     ros::init(argc,argv,"publisher");
     ros::NodeHandle nh;
    srand ( time(NULL) );          
     ros::Publisher pub=nh.advertise<geometry_msgs::Point>("points",2000);
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    Plane2DEnvironment env ;
    if (env.plan(0, 0, 300,400))
    {
        env.recordSolution();
        imwrite("res2.jpg",img);
    }

    return 0;
}
