#include "rrt/rrt.h"
#include <math.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Float32.h>


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
//            ROS_WARN("(%f,%f) is %d",x,y, grid_map_.at<uchar>(map_param::grid_map::kHeight - 1 - y, x));

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


    void DubinsMotionTestValidator::defaultSettings()
    {
        stateSpace_ = dynamic_cast<ob::DubinsStateSpace *>(si_->getStateSpace().get());
        if (stateSpace_ == nullptr)
            throw ompl::Exception("No state space for motion validator");
    }

    bool DubinsMotionTestValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                                        std::pair<ob::State *, double> &lastValid) const
    {
        /* assume motion starts in a valid configuration so s1 is valid */

        bool result = true, firstTime = true;
        ob::DubinsStateSpace::DubinsPath path;
        int nd = stateSpace_->validSegmentCount(s1, s2);

        ROS_ERROR("2# is ");

        if (nd > 1)
        {
            /* temporary storage for the checked state */
            ob::State *test = si_->allocState();

            for (int j = 1; j < nd; ++j)
            {
                stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
                if (!si_->isValid(test))
                {
                    lastValid.second = (double)(j - 1) / (double)nd;
                    if (lastValid.first != nullptr)
                        stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                    result = false;
                    break;
                }
            }
            si_->freeState(test);
        }

        if (result)
            if (!si_->isValid(s2))
            {
                lastValid.second = (double)(nd - 1) / (double)nd;
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

    bool DubinsMotionTestValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
    {
        /* assume motion starts in a valid configuration so s1 is valid */
        if (!si_->isValid(s2))
            return false;

        bool result = true, firstTime = true;
        ob::DubinsStateSpace::DubinsPath path;
        int nd = stateSpace_->validSegmentCount(s1, s2);
        ROS_ERROR("Dis: %f",ceil(stateSpace_->distance(s1, s2)));
        ROS_ERROR("Factor: %d",stateSpace_->getValidSegmentCountFactor());
        ROS_ERROR("Length %f",stateSpace_->getLongestValidSegmentLength());


        const auto *state2D_2 =
                s1->as<ob::SE2StateSpace::StateType>();
        double x_2 = state2D_2->getX();
        double y_2 = state2D_2->getY();
        ROS_ERROR("(%f,%f) is ",x_2,y_2);
        const auto *state2D_3 =
                s2->as<ob::SE2StateSpace::StateType>();
        double x_3 = state2D_3->getX();
        double y_3 = state2D_3->getY();
        ROS_ERROR("(%f,%f) is ",x_3,y_3);

        ROS_ERROR("seg: %d", nd);

        /* initialize the queue of test positions */
        std::queue<std::pair<int, int>> pos;
        if (nd >= 2)
        {
            pos.push(std::make_pair(1, nd - 1));

            /* temporary storage for the checked state */
            ob::State *test = si_->allocState();

            /* repeatedly subdivide the path segment in the middle (and check the middle) */
            while (!pos.empty())
            {
                std::pair<int, int> x = pos.front();

                int mid = (x.first + x.second) / 2;
                stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

                const auto *state2D_1 =
                        test->as<ob::SE2StateSpace::StateType>();

                double x_1 = state2D_1->getX();
                double y_1 = state2D_1->getY();
                ROS_ERROR("(%f,%f) is ",x_1,y_1);

                if (!si_->isValid(test))
                {
                    result = false;
                    ROS_WARN("NULL ");
                    break;
                }
                ROS_WARN("OK ");

                pos.pop();

                if (x.first < mid)
                    pos.push(std::make_pair(x.first, mid - 1));
                if (x.second > mid)
                    pos.push(std::make_pair(mid + 1, x.second));
            }

            si_->freeState(test);
        }

        if (result)
            valid_++;
        else
            invalid_++;

        return result;
    }

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

//            if(result) {
//                ROS_WARN("OK, %f",ramp);
//            } else {
//                ROS_WARN("NO, %f",ramp);
//            }
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
//
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

//            ROS_ERROR("seg: %d", nd);

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

//            bool result = checkMotion()

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
//    ob::SpaceInformationPtr si(space);
//    si->setMotionValidator(std::make_shared<myMotionValidator>(si));
//    si->setup();


    RRT::RRT(ros::NodeHandle *nh, tf::TransformListener *listener, double runTime, double range) :
            nh_(nh), listener_(listener), run_time_(runTime), range_(range), bGotGoal_(false),
            bGotStart_(false), bReverse_(false), bStartfromCar_(true), timer_count_(0)  {

        it_ = new image_transport::ImageTransport(*nh_);
        timer_ = nh->createTimer(ros::Duration(1.0), &RRT::TimerCallback, this);
        subGridMap = it_->subscribe("/dilated_grid_map", 1, &RRT::GridMapCallback, this);
        trajectory_pub = nh_->advertise<nav_msgs::Path>("/path_rrt", 1);
        goalRRTChooseSub_ = nh_->subscribe("/rrt_choose_local_target", 1, &RRT::GoalCallback, this);
        startLocalTrajSub_ = nh_->subscribe("/rrt_plan_start", 1, &RRT::StartCallback, this);
//        directionSub_ = nh_->subscribe("/vehicle_direction", 1, &RRT::DirectionCallback, this);
    }

    void RRT::InitRRT() {

//        bFirst_ = false;

    }


    void RRT::TimerCallback(const ros::TimerEvent& event){
        timer_count_ = std::min(20, timer_count_+1);
        bGotGoal_ = timer_count_ > 10 ? false : bGotGoal_;
    }


    void RRT::DirectionCallback(const std_msgs::Float32 &direction){

        if(direction.data < 0)
            bReverse_ = true;
        else
            bReverse_ = false;
    }

    void RRT::StartCallback(const geometry_msgs::PoseStampedPtr &start) {

        if( (start->pose.position.x == std::numeric_limits<double>::infinity()) && (start->pose.position.y == std::numeric_limits<double>::infinity()) ){
            bStartfromCar_ = true;
        } else {
            start_world_ = *start;
            bStartfromCar_ = false;
        }
        //ROS_ERROR("%f, %f", start->pose.position.x, start->pose.position.y);

        bGotStart_ = true ;

    }


    void RRT::GridMapCallback(const sensor_msgs::ImageConstPtr &map_in) {

        cv_bridge::toCvShare(map_in, "mono8")->image.copyTo(localMap_);
        cv::Mat kernelDilateL = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                cv::Size(map_param::grid_map::kDDilateDeltaSize, map_param::grid_map::kDDilateDeltaSize));
        dilate(localMap_, localMapWider_, kernelDilateL);

        if(!bGotGoal_){
            return;
        }

        std::vector<double> local_goal;
        local_goal.emplace_back(goal_world_.pose.position.x);
        local_goal.emplace_back(goal_world_.pose.position.y);
        local_goal.emplace_back(tf::getYaw(goal_world_.pose.orientation));

        Plan(local_goal, false);

        trajectory_pub.publish(temp_path_);

        bGotStart_ = false;
    }


    void RRT::GoalCallback(const geometry_msgs::PoseStampedPtr &goal){
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

    bool RRT::Plan(std::vector<double> start_in, std::vector<double> goal_in, bool reverse) {
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


    bool RRT::Plan(std::vector<double> goal_in, bool reverse) {
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


    bool RRT::Plancore(std::vector<double> start_in, std::vector<double> goal_in,
                       bool reverse)  // default: runTime = 0.05, reverse = false
    {
//        struct timeval begin;
//        struct timeval end;
//
//        gettimeofday(&begin, NULL);

        space_ = std::make_shared<ob::DubinsStateSpace>(4.8 * map_param::grid_map::kCellFactor);
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
        si->setMotionValidator(std::make_shared<ob::DubinsMotionValidator>(si));
//        si->setMotionValidator(std::make_shared<DubinsMotionTestValidator>(si));

        const ob::StateSpacePtr &SE2ss = si->getStateSpace();
        SE2ss->setLongestValidSegmentFraction(0.02);
        SE2ss->setValidSegmentCountFactor(5);

        // this call is optional, but we put it in to get more output information
        si->setStateValidityCheckingResolution(0.02);

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
            auto state_test = path.getStates();
            path.interpolate(100);

            auto states = path.getStates();
            temp_path_.poses.clear();

            int sum_avg = 0, sum_var = 0;
            int index_col = 0, index_row = 0;
            int compare_index_col = 0;
            int compare_index_row = 0;
            int compare_grey, current_grey;
            // tranform from pix to world
            geometry_msgs::PoseStamped local_map_pose;//, world_pose;

            temp_path_.header.stamp = ros::Time::now();
            temp_path_.header.frame_id = "base_link";
//            ROS_ERROR("======================================================");
//            ROS_ERROR("states.size(): %d", state_test.size());

            bool bInValid = false;
            bool bStrictInValid = false;

            for (auto iter = states.begin(); iter < states.end(); iter++) {
                const auto *state2D =
                        (*iter)->as<ob::SE2StateSpace::StateType>();

                index_col = state2D->getX();
                index_row = map_param::grid_map::kHeight - 1 - state2D->getY();
                if(localMap_.at<uchar>(index_row, index_col) > map_param::grid_map::kObstacleThreshold){
                    bInValid = true;
                    break;
                }

                if(localMapWider_.at<uchar>(index_row, index_col) > map_param::grid_map::kObstacleThreshold){
                    bStrictInValid = true;
                }

                // transform XY to world
                TFPix2XYInCar(state2D->getX(), map_param::grid_map::kHeight - 1 - state2D->getY(),
                              local_map_pose.pose.position.x, local_map_pose.pose.position.y);
                local_map_pose.pose.position.z = 0.0;

                local_map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state2D->getYaw()-M_PI/2);

                temp_path_.poses.emplace_back(local_map_pose);
            }

            double disToGoal = pow(local_map_pose.pose.position.x - goal_world_.pose.position.x , 2)
                               + pow(local_map_pose.pose.position.y - goal_world_.pose.position.y , 2);
            double cost = disToGoal;
//            double cost = -path_length;



//            if(states.size() < 8 || bInValid) {
            if(bInValid) {
                cost = std::numeric_limits<double>::infinity();
//                ROS_ERROR("collision");
            } else if(bStrictInValid) {
//                ROS_ERROR("saft collision");
                cost += 9;
            }

//            double mean = sum_avg / states.size();
//            double var = sum_var / states.size() - mean * mean;

//            double disToGoal = pow(local_map_pose.pose.position.x - goal_world_.pose.position.x , 2)
//                               + pow(local_map_pose.pose.position.y - goal_world_.pose.position.y , 2); // TODO: change mrthod


            geometry_msgs::PoseStamped temp_pose;
            if(temp_path_.poses.empty()){
                temp_pose.pose.position.x = 0.0;
                temp_pose.pose.position.y = 0.0;
            } else {
                temp_pose.pose.position.x = temp_path_.poses.back().pose.position.x;
                temp_pose.pose.position.y = temp_path_.poses.back().pose.position.y;
            }
            temp_pose.pose.position.z = cost;
            temp_path_.poses.emplace_back(temp_pose);
//        gettimeofday(&end,NULL);
//        std::cout << "here is" << 1000000 * (end.tv_sec - begin.tv_sec) + end.tv_usec - begin.tv_usec << std::endl;

            return true;
        } else {
            std::cout << "No solution found." << std::endl;
            return false;
        }
    }

}// namespace huyao_rrt

