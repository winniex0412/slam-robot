#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <common/point.hpp>
#include <random>

std::random_device rd;
std::mt19937 generator(rd());

// helper function declare wrap angle to [-M_PI, M_PI]
float wrapAngleToPI(float angle){
    while(fabs(angle) > M_PI){
        if(angle > 0){
            angle = angle - 2*M_PI;
        }
        else{
            angle = angle + 2*M_PI;
        }
    }
    return angle; 
}

ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    odomInitFlag = 1;
    alfa1 = 1;
    alfa2 = 0.4;
    alfa3 = 0.3;
    alfa4 = 0.02;
    odomPrev = {0};
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(odomInitFlag){
        odomInitFlag = 0;
        odomPrev = odometry;
        return false;
    }
    if(odomPrev.x == odometry.x && 
        odomPrev.y == odometry.y &&
        odomPrev.theta == odometry.theta ){
            odomPrev = odometry;
            return false;
        }

    // probabilistic robotics page 136
    Point<float> startOdom = {odomPrev.x, odomPrev.y};
    Point<float> endOdom = {odometry.x, odometry.y};
    float angleTemp = angle_to_point(startOdom, endOdom);
    if(distance_between_points(startOdom, endOdom)<0.01){
        rot1 = wrap_to_pi((odometry.theta - odomPrev.theta));
        trans = distance_between_points(startOdom, endOdom);
        rot2 = 0;
    }
    else{
        rot1 = wrap_to_pi(angleTemp - odomPrev.theta);
        trans = distance_between_points(startOdom, endOdom);
        rot2 = wrap_to_pi(odometry.theta - angleTemp);
    }
    
    //printf("action model: %f, %f, %f \n", rot1, trans, rot2);
    odomPrev = odometry;
    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    //std::default_random_engine generator;
    /*   
    float covRot1 = alfa1*rot1*rot1 + alfa2*trans*trans;
    float covTrans = alfa3*trans*trans + alfa4*rot1*rot1 + alfa4*rot2*rot2;
    float covRot2 = alfa1*rot2*rot2 + alfa2*trans*trans;
    */
    float covRot1 = alfa1*rot1;
    float covTrans = alfa2*trans;
    float covRot2 = alfa1*rot2;
    std::normal_distribution<> distributionRot1(rot1, fabs(covRot1));
    std::normal_distribution<> distributionTrans(trans, fabs(covTrans));
    std::normal_distribution<> distributionRot2(rot2, fabs(covRot2));

    float rot1_hat = distributionRot1(generator);
    float trans_hat = distributionTrans(generator);
    float rot2_hat = distributionRot2(generator);

    particle_t sampleAction;
    /*
    sampleAction.pose.x = sample.pose.x + trans_hat*cos(sample.pose.theta + rot1_hat);
    sampleAction.pose.y = sample.pose.y + trans_hat*sin(sample.pose.theta + rot1_hat);
    sampleAction.pose.theta = sample.pose.theta + rot1_hat + rot2_hat;
    sampleAction.pose.utime = odomPrev.utime;
    sampleAction.parent_pose = sample.pose;
    sampleAction.weight = sample.weight;*/
    sampleAction.pose = sample.pose;
    sampleAction.pose.theta += rot1_hat;
    sampleAction.pose.x += cos(sampleAction.pose.theta)*trans_hat;
    sampleAction.pose.y += sin(sampleAction.pose.theta)*trans_hat;
    sampleAction.pose.theta += rot2_hat;
    sampleAction.pose.utime = odomPrev.utime;
    sampleAction.parent_pose = sample.pose;
    sampleAction.weight = sample.weight;
    return sampleAction;
}
