#include "planner.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

Planner::Planner()
{
    subsciber_ = nh_.subscribe<multi_agent_planner::Agent>("agent_feedback",
                                                           10,
                                                           &Planner::msgCallBack,
                                                           this);
    service_ = nh_.advertiseService("get_plan", &Planner::getPlan, this);
    //初始化预约表
    occupation_.resize(SQURE);
    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            occupation_[i].emplace_back(false);
        }
    }
}

bool Planner::getPlan(multi_agent_planner::AgentSrvRequest &goal,
                      multi_agent_planner::AgentSrvResponse &res)
{
    ROS_INFO("planning ...");
    //集合S，u分别记录已经找到最短路径的点和为找到最短路径的点
    vector<TrajType> s, u;
    //mask 用于标记各点是否检查过
    vector<vector<bool> > mask;
    mask.resize(SQURE);
    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            mask[i].emplace_back(false);
        }
    }
    //初始化起始点
    Point start;
    start.x = current_pose_[goal.sreial_id].x;
    start.y = current_pose_[goal.sreial_id].y;
    mask[start.x][start.y] = true;//标记起始点
    
    //遍历起始点之外的所有点，添加到集合u中
    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            if (start.x == i && start.y == j)
                continue;

            TrajType tt;
            tt.current = road_map_.scatter_map_[i][j];
            tt.last = start;//父节点设置成起始点
            if (road_map_.scatter_map_[start.x][start.y].isAdjoin(i, j)) { //如果当前点（i，j）与起始点相邻花费为10，否则设置为INF
                tt.cost = 10;
            } else {
                tt.cost = INF;
            }

            if (occupation_[i][j]) {//检查当前点是否被其他agent占用，被占用设置花费为INF
                tt.cost = INF;
            }
            u.emplace_back(tt);
        }
    }
    //根据花费降序排列，sort默认升序排列，所以用rbegin(),rend()
    sort(u.rbegin(), u.rend());

    while (!u.empty()) {
        TrajType best = u.back();//最优点是u集合最后的点
        u.pop_back();//抛弃使检查过的点
        if (mask[best.current.x][best.current.y]) {//如果此节点检查过进入下次循环
            continue;
        }

        mask[best.current.x][best.current.y] = true;//标记此节点
        s.emplace_back(best);//将最优节点放入集合s
        //更新u集合的花费
        for (int i = 0; i < u.size(); i++) {
            int cost = 0;
            if (road_map_.scatter_map_[best.current.x][best.current.y].isAdjoin(u[i].current.x,
                                                                                u[i].current.y)) {
                cost = best.cost + 10;//最优点与u[i]相邻花费为10
            } else {
                cost = best.cost + INF;//不相邻花费为INF
            }

            if (occupation_[best.current.x][best.current.y]) {//若u[i]被其他agent占用，花费为INF
                cost = INF;
            }
            if (cost < u[i].cost) {
                u[i].last = best.current;//父节点设置为最优点
                u[i].cost = cost;//更新当前花费
            }
        }

        sort(u.rbegin(), u.rend());//重新降序排列
    }

    //起始点到所有节点的花费计算完毕
    //从集合s中取出最短路径经过的点
    TrajType tts;
    tts.current.x = goal.x;
    tts.current.y = goal.y;
    vector<Point> path;
    path.emplace_back(tts.current);
    //从goal点开始，查找父节点
    while (getLast(s, tts)) {
        tts.current = tts.last;
        path.emplace_back(tts.last);
        //如果tts的父节点是起始点，结束
        if ((tts.last.x == start.x) && (tts.last.y == start.y)) {
            break;
        }
    }
    //path中的点是从goal到start,倒序
    reverse(path.begin(), path.end());
    //填充response
    res.path.header.stamp = ros::Time::now();
    res.path.header.frame_id = "map";

    for (int i = 0; i < path.size(); i++) {
        geometry_msgs::PoseStamped tps;
        tps.header.frame_id = "map";

        tps.pose.position.x = path[i].x;
        tps.pose.position.y = path[i].y;
        occupation_[path[i].x][path[i].y] = true;
        tps.pose.position.z = 0;
        tps.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        res.path.poses.emplace_back(tps);
    }

    return (true);
}

void Planner::msgCallBack(const multi_agent_planner::AgentConstPtr &msg)
{
    map<string, multi_agent_planner::Agent>::iterator it;
    it = current_pose_.find(msg->sreial_id);//检查serial_id是否注册
    if (it == current_pose_.end()) {
        current_pose_.insert(make_pair(msg->sreial_id, *msg));//注册当前serial_id
    } else {
        current_pose_[msg->sreial_id] = *msg;//更新位姿
    }
}

void Planner::run()
{
    ros::spin();
}

bool Planner::getLast(vector<TrajType> &s, TrajType &goal)
{
    vector<TrajType>::iterator it = find(s.begin(), s.end(), goal);//查找当前节点goal是否在集合s中
    if (it != s.end()) {
        goal.last = it->last;//返回父节点
        return (true);
    }
    return (false);
}
