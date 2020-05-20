#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <algorithm>
std::random_device rd2;
std::mt19937 generator2(rd2());

bool myComp(particle_t first, particle_t second){
    return first.weight < second.weight;
}

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    std::normal_distribution<> distributionX(pose.x, fabs(0.015));
    std::normal_distribution<> distributionY(pose.y, fabs(0.015));
    std::normal_distribution<> distributionTheta(pose.theta, fabs(0.0001));
    //std::default_random_engine generator;// need to makesure !!!!
    // generate the particles accoding to the normal distribution initialized
    pose_xyt_t tempPose;
    for(int i = 0; i < kNumParticles_; i++){
        tempPose.x = distributionX(generator2);
        tempPose.y = distributionY(generator2);
        tempPose.theta = distributionTheta(generator2);
        posterior_[i] = {tempPose, tempPose, 1.0f/kNumParticles_}; 
    }
    posteriorPose_ = pose;
    
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //printf("resamplePosteriorDistribution\n");
        auto prior = resamplePosteriorDistribution();
        //printf("computeProposalDistribution\n");
        auto proposal = computeProposalDistribution(prior);
        //printf("computeNormalizedPosterior\n");
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        //printf("estimatePosteriorPose\n");
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior;
    //initialize random seed http://www.cplusplus.com/reference/cstdlib/rand/
    srand(time(NULL));
    for(int i = 0; i < kNumParticles_; i++){
        double randNum = (double)rand()/(double)RAND_MAX;
        int count = -1;
        double weightSum = 0;
        while(weightSum <= randNum){    
            count ++;
            weightSum = weightSum + posterior_[count].weight;
            
        }
        prior.push_back(posterior_[count]);
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(int i = 0; i < kNumParticles_; i++){
        proposal.push_back(actionModel_.applyAction(prior[i]));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior = proposal;
    //posterior = proposal;
    double weightSum = 0.0;
    printf("new set of weight\n");
    //printf("proposal size: %d\n", proposal.size());
    for(int i = 0; i < kNumParticles_; i++){
        double w = sensorModel_.likelihood(proposal[i], laser, map);
        //double newWeight = exp(sensorModel_.likelihood(proposal[i], laser, map));
        //printf("calculate newWeight %d: %f\n", i, newWeight);
        //printf("proposal size %d: %d\n", i, proposal.size());
        posterior[i].weight = exp(w);
        weightSum += posterior[i].weight;

    }
    //printf("new set of weight assigned\n");
    for(int i = 0; i < kNumParticles_; i++){
        posterior[i].weight = (double)posterior[i].weight/(double)weightSum;
        //printf("newWeight %f\n", posterior[i].weight);
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    double weightSum = 0.0;
    //std::vector<particle_t> tempPosterior = posterior;
    sort(posterior_.begin(), posterior_.end(), myComp);
    /*
    for(auto ele = posterior.begin(); ele != posterior.end(); ele++){
        pose.x += ele->pose.x*ele->weight;
        pose.y += ele->pose.y*ele->weight;
        pose.theta += ele->pose.theta*ele->weight;
        weightSum += ele->weight;
    }*/
    for(int i =0; i < 50; i++){
        pose.x += posterior_[i].pose.x*posterior_[i].weight;
        pose.y += posterior_[i].pose.y*posterior_[i].weight;
        pose.theta += posterior_[i].pose.theta*posterior_[i].weight;
        weightSum += posterior_[i].weight;
    }
    pose.x = pose.x/weightSum;//actually weightSum should be 1
    pose.y = pose.y/weightSum;
    pose.theta = pose.theta/weightSum;
    return pose;
}


