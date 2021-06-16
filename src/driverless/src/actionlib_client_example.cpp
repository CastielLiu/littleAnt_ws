#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <driverless_actions/DoDriverlessTaskAction.h>

class ActionlibClientTest
{
public:
    typedef actionlib::SimpleActionClient<driverless_actions::DoDriverlessTaskAction> DoDriverlessTaskClient;
    ActionlibClientTest() {}
    bool init()
    {
        ac_ = new DoDriverlessTaskClient("/driverless/do_driverless_task", true); // true -> don't need ros::spin()
        std::cout << "waitForServer\r\n ";
        ac_->waitForServer();
        std::cout << "waitForServer ok \r\n ";
        return true;
    }



    void taskFeedbackCallback(const driverless_actions::DoDriverlessTaskFeedbackConstPtr& fd)
    {
        std::cout << "taskFeedbackCallback \r\n";
    }

    void taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const driverless_actions::DoDriverlessTaskResultConstPtr& res)
    {
        std::cout <<   "taskDoneCallback \r\n";
         
    }

    void taskActivedCallback()
    {
        std::cout <<  "taskActivedCallback \r\n";
  
    }

    void run()
    {
        while(ros::ok())
        {
            std::cout << " isServerConnected: " << ac_->isServerConnected() << std::endl;

            driverless_actions::DoDriverlessTaskGoal goal;
            goal.roadnet_file = "11111";
            goal.expect_speed = 2.0;
            goal.type = goal.FILE_TYPE;
            goal.task = goal.DRIVE_TASK;

            ac_->sendGoal(goal,boost::bind(&ActionlibClientTest::taskDoneCallback,this,_1,_2),
                                boost::bind(&ActionlibClientTest::taskActivedCallback,this),
                                boost::bind(&ActionlibClientTest::taskFeedbackCallback,this,_1));
            
            ac_->waitForResult();
            std::cout << ac_->getState().toString() << std::endl;
            ros::Duration(1.5).sleep();
        }
    }



private:
    DoDriverlessTaskClient* ac_;


};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "actionlib_client_node");
	ros::AsyncSpinner spinner(2);
	spinner.start(); //非阻塞

	ros::NodeHandle nh, nh_private("~");
    ActionlibClientTest app;
    if(app.init())
        app.run();
    ros::waitForShutdown();
    return 0;
}  

