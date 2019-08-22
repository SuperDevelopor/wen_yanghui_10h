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
    vector<TrajType> s, u;
    vector<vector<bool> > mask;
    mask.resize(SQURE);
    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            mask[i].emplace_back(false);
        }
    }
    Point start;
    start.x = current_pose_[goal.sreial_id].x;
    start.y = current_pose_[goal.sreial_id].y;
    mask[start.x][start.y] = true;

    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            if (start.x == i && start.y == j)
                continue;

            TrajType tt;
            tt.current = road_map_.scatter_map_[i][j];
            tt.last = start;
            if (road_map_.scatter_map_[start.x][start.y].isAdjoin(i, j)) {
                tt.cost = 10;
            } else {
                tt.cost = INF;
            }

            if (occupation_[i][j]) {
                tt.cost = INF;
            }
            u.emplace_back(tt);
        }
    }

    sort(u.rbegin(), u.rend());

    while (!u.empty()) {
        TrajType best = u.back();
        u.pop_back();
        if (mask[best.current.x][best.current.y]) {
            continue;
        }

        mask[best.current.x][best.current.y] = true;
        s.emplace_back(best);
        //update cost in set u
        for (int i = 0; i < u.size(); i++) {
            int cost = 0;
            if (road_map_.scatter_map_[best.current.x][best.current.y].isAdjoin(u[i].current.x,
                                                                                u[i].current.y)) {
                cost = best.cost + 10;
            } else {
                cost = best.cost + INF;
            }

            if (occupation_[best.current.x][best.current.y]) {
                cost = INF;
            }
            if (cost < u[i].cost) {
                u[i].last = best.current;
                u[i].cost = cost;
            }
        }

        sort(u.rbegin(), u.rend());
    }

    TrajType tts;
    tts.current.x = goal.x;
    tts.current.y = goal.y;
    vector<Point> path;
    path.emplace_back(tts.current);

    while (getLast(s, tts)) {
        tts.current = tts.last;
        path.emplace_back(tts.last);

        if ((tts.last.x == start.x) && (tts.last.y == start.y)) {
            break;
        }
    }

    reverse(path.begin(), path.end());

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
    it = current_pose_.find(msg->sreial_id);
    if (it == current_pose_.end()) {
        current_pose_.insert(make_pair(msg->sreial_id, *msg));
    } else {
        current_pose_[msg->sreial_id] = *msg;
    }
}

void Planner::run()
{
    ros::spin();
}

bool Planner::getLast(vector<TrajType> &s, TrajType &goal)
{
    vector<TrajType>::iterator it = find(s.begin(), s.end(), goal);
    if (it != s.end()) {
        goal.last = it->last;
        return (true);
    }
    return (false);
}
