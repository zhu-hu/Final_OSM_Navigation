#include "rrt/continue_rrt.h"
#include <math.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace huyao_rrt {

    class ValidityChecker : public ob::StateValidityChecker {
    public:
        ValidityChecker(const ob::SpaceInformationPtr &si, const cv::Mat &grid_map) :
                ob::StateValidityChecker(si), grid_map_(grid_map) {}

        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ob::State *state) const override {
            // We know we're working with a RealVectorStateSpace in this
            // example, so we downcast state into the specific type.
            const auto *state2D =
                    state->as<ob::SE2StateSpace::StateType>();
            double x = state2D->getX();
            double y = state2D->getY();

            return si_->satisfiesBounds(state2D) && isCollisioinFree(x, y);
        }

        // use grid map to choose valid state
        bool isCollisioinFree(double x, double y) const {
            int col = int(x);
            int row = int(map_param::grid_map::kHeight - 1 - y);
            if (grid_map_.at<uchar>(row, col) > map_param::grid_map::kObstacleThreshold) {
//                ROS_ERROR("(%f,%f) is false",x,y);
                return false;
            }
//            ROS_ERROR("(%f,%f) is free",x,y);
            return true;
        }


    private:
        cv::Mat grid_map_;
    };


    // define this class:
    class GradMotionValidator : public ob::MotionValidator
    {
    public:
        GradMotionValidator(const ob::SpaceInformationPtr &si, const cv::Mat &grid_map) :
                ob::MotionValidator(si), grid_map_(grid_map) {
            stateSpace_ = dynamic_cast<ob::DubinsStateSpace *>(si_->getStateSpace().get());
        }

        bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override
        {
            /* assume motion starts in a valid configuration so s1 is valid */

            bool result = true, firstTime = true;
            ob::DubinsStateSpace::DubinsPath path;
            double too_sharp = false;

            const auto *state2D_1 =
                    s1->as<ob::SE2StateSpace::StateType>();

            double x_1 = state2D_1->getX();
            double y_1 = state2D_1->getY();
            int index_col_1 = x_1;
            int index_row_1 = map_param::grid_map::kHeight - 1 - y_1;


            const auto *state2D_2 =
                    s2->as<ob::SE2StateSpace::StateType>();

            double x_2 = state2D_2->getX();
            double y_2 = state2D_2->getY();
            int index_col_2 = x_2;
            int index_row_2 = map_param::grid_map::kHeight - 1 - y_2;

            // 保证坡度不会过大
            double dis = map_param::grid_map::kCellSize * sqrt(pow( x_2 - x_1 , 2)+pow( y_2 - y_1 , 2));
            double delta_height = fabs(grid_map_.at<uchar>(index_row_2, index_col_2)
                                       - grid_map_.at<uchar>(index_row_1, index_col_1)) * map_param::grid_map::kHeightResolution ;

            double ramp = delta_height / dis;

            if(ramp > map_param::grid_map::kRampThreshold)
                too_sharp = true;


            if (too_sharp)
            {
                lastValid.second = 0;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
            }

            if (result)
                valid_++;
            else
                invalid_++;

            return result;
        }

        bool checkTwoMotion(const ob::State *s1, const ob::State *s2) const {

            bool result = true;

            const auto *state2D_1 =
                    s1->as<ob::SE2StateSpace::StateType>();

            double x_1 = state2D_1->getX();
            double y_1 = state2D_1->getY();
            int index_col_1 = x_1;
            int index_row_1 = map_param::grid_map::kHeight - 1 - y_1;


            const auto *state2D_2 =
                    s2->as<ob::SE2StateSpace::StateType>();

            double x_2 = state2D_2->getX();
            double y_2 = state2D_2->getY();
            int index_col_2 = x_2;
            int index_row_2 = map_param::grid_map::kHeight - 1 - y_2;

            // 保证坡度不会过大
            double dis = map_param::grid_map::kCellSize * sqrt(pow( x_2 - x_1 , 2)+pow( y_2 - y_1 , 2));
            int v_1 = grid_map_.at<uchar>(index_row_1, index_col_1);
            int v_2 = grid_map_.at<uchar>(index_row_2, index_col_2);

            double ramp;
            if( v_1==0 || v_2==0){
                ramp = 0.0;
            } else {
                double delta_height = fabs(v_2 - v_1) * map_param::grid_map::kHeightResolution ;
                ramp = delta_height / dis;
            }

            if(ramp > map_param::grid_map::kRampThreshold2)
                result = false;

//            ROS_WARN("(%f,%f) is %d, (%f,%f) is %d",x_1,y_1,v_1,x_2,y_2,v_2);
//            if(result) {
//                ROS_WARN("OK2, %f",ramp);
//            } else {
//                ROS_WARN("NO2, %f ",ramp);
//            }

            return result;
        }


        bool checkMotion(const ob::State *s1, const ob::State *s2) const override
        {
            bool result = true, firstTime = true;
            ob::DubinsStateSpace::DubinsPath path;
            int nd = stateSpace_->validSegmentCount(s1, s2);

            /* initialize the queue of test positions */

            std::queue<std::pair<int, int>> pos;
            if (nd >= 2)
            {
                ob::State *last_check = si_->allocState();
                stateSpace_->interpolate(s1, s2, 0, firstTime, path, last_check);

                ob::State *test = si_->allocState();
                for(int i=1; i<nd; i++)
                {
                    stateSpace_->interpolate(s1, s2, (double)i / (double)nd, firstTime, path, test);
                    if (!checkTwoMotion(last_check, test))
                    {
                        result = false;
                        break;
                    }
                    stateSpace_->interpolate(s1, s2, (double)i / (double)nd, firstTime, path, last_check);

                }

                si_->freeState(test);
                si_->freeState(last_check);
            }

            if (result)
                valid_++;
            else
                invalid_++;

            return result;
        }

        // implement checkMotion()
    private:
        cv::Mat grid_map_;
        ob::DubinsStateSpace *stateSpace_;
    };


    CRRT::CRRT(ros::NodeHandle *nh, tf::TransformListener *listener, double runTime, double range) :
            nh_(nh), listener_(listener), run_time_(runTime), range_(range), bGotGoal_(false),
            bGotStart_(false), bReverse_(false)   {

        it_ = new image_transport::ImageTransport(*nh_);
        subGridMap = it_->subscribe("/dilated_grid_map", 1, &CRRT::GridMapCallback, this);
        trajectory_pub = nh_->advertise<nav_msgs::Path>("/path_continue_rrt", 1);
        goalRRTChooseSub_ = nh_->subscribe("/rrt_choose_local_target", 1, &CRRT::GoalCallback, this);
        startLocalTrajSub_ = nh_->subscribe("/rrt_plan_start", 1, &CRRT::StartCallback, this);
//        directionSub_ = nh_->subscribe("/vehicle_direction", 1, &CRRT::DirectionCallback, this);
    }

    void CRRT::InitRRT() {

//        bFirst_ = false;

    }

    void CRRT::DirectionCallback(const dstar_map::StampedFloat64Ptr &goal){

        if(goal->data < 0)
            bReverse_ = true;
        else
            bReverse_ = false;
    }

    void CRRT::StartCallback(const geometry_msgs::PoseStampedPtr &start) {

        start_world_ = *start;
        //ROS_ERROR("%f, %f", start->pose.position.x, start->pose.position.y);
        bGotStart_ = true ;

        if(!bGotGoal_){
            return;
        }

        // get goal in base_link
        std::vector<double> local_goal;
        local_goal.emplace_back(goal_world_.pose.position.x);
        local_goal.emplace_back(goal_world_.pose.position.y);
        local_goal.emplace_back(tf::getYaw(goal_world_.pose.orientation));

        // get start in base_link
        geometry_msgs::PoseStamped start_local_pose;
        if (!TransformPose(listener_, "world", "base_link", start_world_, start_local_pose))
            return;

        std::vector<double> local_start;
        local_start.emplace_back(start_local_pose.pose.position.x);
        local_start.emplace_back(start_local_pose.pose.position.y);
        local_start.emplace_back(tf::getYaw(start_local_pose.pose.orientation));

        Plan(local_start,local_goal, false);

        trajectory_pub.publish(temp_path_);

    }


    void CRRT::GridMapCallback(const sensor_msgs::ImageConstPtr &map_in) {

        cv_bridge::toCvShare(map_in, "mono8")->image.copyTo(localMap_);
//        cv::Mat kernelDilateL = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(map_param::grid_map::kDDilateSize, map_param::grid_map::kDDilateSize));
//        dilate(localMap_, localMap_, kernelDilateL);

    }


    void CRRT::GoalCallback(const geometry_msgs::PoseStampedPtr &goal){
        goal_world_ = *goal;   // is use goal given by rrt choose, it is not world;
        bGotGoal_ = true;

    }


    void perpBisector(double x1, double y1, double x2, double y2, double &AM, double &BM, double &CM) {
        double A = y2 - y1;
        double B = x2 - x1;

        double xm1 = 0.5 * (x1 + x2);
        double ym1 = 0.5 * (y1 + y2);

        double D = B * xm1 + A * ym1;

        AM = B;
        BM = A;
        CM = D;
    }

    double estimateKappa(cyber_msgs::LocalTrajPoint &p1, cyber_msgs::LocalTrajPoint &p2,
                         cyber_msgs::LocalTrajPoint &p3) {

        // Ax+By=C
        double A1, B1, C1, A2, B2, C2;
        perpBisector(p1.position.x, p1.position.y, p2.position.x, p2.position.y, A1, B1, C1);
        perpBisector(p2.position.x, p2.position.y, p3.position.x, p3.position.y, A2, B2, C2);

        double det = A1 * B2 - A2 * B1;
        if (det == 0) {
            return 0.0;
        }

        double cx = (B2 * C1 - B1 * C2) / det;
        double cy = (A1 * C2 - A2 * C1) / det;

        double kappa = 1.0 / hypot(p1.position.x - cx, p1.position.y - cy);
        return kappa;
    }

    bool CRRT::Plan(std::vector<double> start_in, std::vector<double> goal_in, bool reverse) {
        if (start_in.size() != 3)
            return false;

        if (goal_in.size() != 3)
            return false;

        int start_pix_x, start_pix_y;
        TFXY2PixInCar(start_in[0], start_in[1], start_pix_x, start_pix_y);
        std::vector<double> start;
        start.emplace_back(start_pix_x);
        start.emplace_back(map_param::grid_map::kHeight - 1 - start_pix_y);
        start.emplace_back(start_in[2] + M_PI / 2);

        double test1,test2;
        TFPix2XYInCar(start_pix_x, start_pix_y, test1, test2);
        geometry_msgs::PoseStamped test_local, test_world;
        test_local.pose.position.x = test1;
        test_local.pose.position.y = test2;
        tf::Quaternion temp_q = tf::createQuaternionFromYaw(0);
        test_local.pose.orientation.x = temp_q.x();
        test_local.pose.orientation.y = temp_q.y();
        test_local.pose.orientation.z = temp_q.z();
        test_local.pose.orientation.w = temp_q.w();
        std::cout << "here 1#" << std::endl;
        if (!TransformPose(listener_, "base_link", "world", test_local, test_world))
            return false;///TF转换失败
        std::cout << "here 2#" << std::endl;


//        ROS_WARN("start world: %f,%f",start_world_.pose.position.x,start_world_.pose.position.y);
//        ROS_WARN("test world: %f,%f",test_world.pose.position.x,test_world.pose.position.y);


        int goal_pix_x, goal_pix_y;
        TFXY2PixInCar(goal_in[0], goal_in[1], goal_pix_x, goal_pix_y);
        std::vector<double> goal;
        goal.emplace_back(goal_pix_x);
        goal.emplace_back(map_param::grid_map::kHeight - 1 - goal_pix_y);
        goal.emplace_back(goal_in[2] + M_PI / 2);

//        if (reverse) {
//            start.emplace_back(start_in[2]);
//            goal.emplace_back(goal_in[2]);
//        } else {
//            start.emplace_back(start_in[2]);
//            goal.emplace_back(goal_in[2]);
//        }

        return Plancore(start, goal, reverse);
    }


    bool CRRT::Plan(std::vector<double> goal_in, bool reverse) {
        if (goal_in.size() != 3)
            return false;

        std::vector<double> start;
        start.emplace_back(map_param::grid_map::kCarCenterX);
        start.emplace_back(map_param::grid_map::kCarCenterY);


        int goal_pix_x, goal_pix_y;
        TFXY2PixInCar(goal_in[0], goal_in[1], goal_pix_x, goal_pix_y);
        std::vector<double> goal;
        goal.emplace_back(goal_pix_x);
        goal.emplace_back(map_param::grid_map::kHeight - 1 - goal_pix_y);
        if (reverse) {
            goal.emplace_back(goal_in[2] + M_PI / 2);
//            goal.emplace_back(goal_in[2] + M_PI / 2);
            start.emplace_back(-M_PI / 2);
        } else {
            goal.emplace_back(goal_in[2] + M_PI / 2);
//            goal.emplace_back(goal_in[2] + M_PI / 2);
            start.emplace_back(M_PI / 2);
        }

        return Plancore(start, goal, reverse);
    }


    bool CRRT::Plancore(std::vector<double> start_in, std::vector<double> goal_in,
                       bool reverse)  // default: runTime = 0.05, reverse = false
    {
//        struct timeval begin;
//        struct timeval end;
//
//        gettimeofday(&begin, NULL);

        space_ = std::make_shared<ob::DubinsStateSpace>(5.5 * map_param::grid_map::kCellFactor);
//    space_ = std::make_shared<ob::ReedsSheppStateSpace>(5.5 * map_param::grid_map::kCellFactor);

        // 2dimensions ,x -> 1, y -> 2
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, 0);
        bounds.setLow(1, 0);
        bounds.setHigh(0, map_param::grid_map::kWidth - 1);
        bounds.setHigh(1, map_param::grid_map::kHeight - 1);

        space_->as<ob::SE2StateSpace>()->setBounds(bounds);

        // define a simple setup class
        ss_ = std::make_shared<og::SimpleSetup>(space_);

        // set state validity checking for this space_
        const ob::SpaceInformationPtr &si = ss_->getSpaceInformation();
        ss_->setStateValidityChecker(std::make_shared<ValidityChecker>(si, localMap_));
        si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
//        si->setMotionValidator(std::make_shared<GradMotionValidator>(si, localMap_));

        const ob::StateSpacePtr &SE2ss = si->getStateSpace();
        SE2ss->setLongestValidSegmentFraction(0.05);

        // this call is optional, but we put it in to get more output information
        si->setStateValidityCheckingResolution(0.05);

        // Construct the optimal planner specified by our command line argument.
        // This helper function is simply a switch statement.
        ss_->setPlanner(std::make_shared<og::RRT>(si));
        auto planner = ss_->getPlanner();
        auto *rrtplanner = planner->as<og::RRT>();
        rrtplanner->setRange(range_);

        ob::ScopedState<> start(space_), goal(space_);
        // set the start and goal states
        start[0] = start_in[0];
        start[1] = start_in[1];
        start[2] = start_in[2];

//        ROS_WARN("start point (%f,%f,%f)",start[0],start[1],start[2]);

        goal[0] = goal_in[0];
        goal[1] = goal_in[1];
        goal[2] = goal_in[2];

        ss_->setStartAndGoalStates(start, goal);


        ss_->setup();
//        ss_->print();


//    gettimeofday(&end,NULL);
//    std::cout << "here is" << 1000000 * (end.tv_sec - begin.tv_sec) + end.tv_usec - begin.tv_usec << std::endl;


        // attempt to solve the problem within 30 seconds of planning time
        ob::PlannerStatus solved = ss_->solve(run_time_);

//    gettimeofday(&end,NULL);
//    std::cout << "here is" << 1000000 * (end.tv_sec - begin.tv_sec) + end.tv_usec - begin.tv_usec << std::endl;

        if (solved) {
//        // Output the length of the path found
//        std::cout
//                << ss_->getPlanner()->getName()
//                << " found a solution of length "
//                << ss_->getSolutionPath().length()
//                << std::endl;

            ss_->simplifySolution();
            og::PathGeometric path = ss_->getSolutionPath();
            path.interpolate(100);

            auto states = path.getStates();
            temp_path_.poses.clear();

            int sum_avg = 0, sum_var = 0;
            int index_col = 0, index_row = 0;
            int compare_index_col = 0;
            int compare_index_row = 0;
            int compare_grey, current_grey;
            // tranform from pix to world
            geometry_msgs::PoseStamped local_map_pose;
//            , world_pose;

            temp_path_.header.stamp = ros::Time::now();
            temp_path_.header.frame_id = "base_link";

             for (auto iter = states.begin(); iter < states.end(); iter++) {
                const auto *state2D =
                        (*iter)->as<ob::SE2StateSpace::StateType>();

                index_col = state2D->getX();
                index_row = map_param::grid_map::kHeight - 1 - state2D->getY();
//                sum_avg += localMap_.at<uchar>(index_row, index_col);
//                sum_var += localMap_.at<uchar>(index_row, index_col) * localMap_.at<uchar>(index_row, index_col);

                // transform XY to world
                TFPix2XYInCar(state2D->getX(), map_param::grid_map::kHeight - 1 - state2D->getY(),
                              local_map_pose.pose.position.x, local_map_pose.pose.position.y);
                local_map_pose.pose.position.z = 0.0;

//                current_grey = localMap_.at<uchar>(index_row, index_col);
//                auto compare_iter = iter+5;
//                if(compare_iter < states.end()){
//                    const auto *comparestate2D =
//                            (*iter)->as<ob::SE2StateSpace::StateType>();
//
//                    compare_index_col = comparestate2D->getX();
//                    compare_index_row = map_param::grid_map::kHeight - 1 - comparestate2D->getY();
//                    compare_grey = localMap_.at<uchar>(compare_index_row, compare_index_col);
//                    cost = std::max(cost, fabs(compare_grey-current_grey));
//
//                }
//
//                if (reverse)
//                local_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-state2D->getYaw()-M_PI/2);
//                else
                    local_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state2D->getYaw()-M_PI/2);

                temp_path_.poses.emplace_back(local_map_pose);
            }

            double cost = pow(temp_path_.poses.back().pose.position.y-temp_path_.poses.front().pose.position.y,2)
                          + pow(temp_path_.poses.back().pose.position.x-temp_path_.poses.front().pose.position.x,2);


            if(states.size() < 8) {
                cost = std::numeric_limits<double>::infinity();
            }

//            double mean = sum_avg / states.size();
//            double var = sum_var / states.size() - mean * mean;
//
//            double disToGoal = pow(local_map_pose.pose.position.x - goal_world_.pose.position.x , 2) + pow(local_map_pose.pose.position.y - goal_world_.pose.position.y , 2);  // TODO: change mrthod

            geometry_msgs::PoseStamped temp_pose;
            temp_pose.pose.position.x = temp_path_.poses.back().pose.position.x;
            temp_pose.pose.position.y = temp_path_.poses.back().pose.position.y;;
            temp_pose.pose.position.z = cost;
            temp_path_.poses.emplace_back(temp_pose);

            return true;
        } else {
            std::cout << "No solution found." << std::endl;
            return false;
        }
    }

}// namespace huyao_rrt

