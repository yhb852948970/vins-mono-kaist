#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

const int LAST_TRACK_NUMBER_KEYFRAME_THRESHOLD = 15;
const int LONG_TRACK_NUMBER_KEYFRAME_THRESHOLD = 30;

// "long tracked feature" should be tracked for at least these frames
const int LONG_TRACK_THRESHOLD = 4;

const double MIN_FEATURE_DEPTH = 1.0;
const double MAX_FEATURE_DEPTH = 200.0;

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
    bool isFeatureValid();
};

class FeatureManager
{
  public:
    FeatureManager();

    void clearState();

    int getFeatureCount();

    // check if the input image is a KeyFrame or not;
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);

    // get matching 2d feature points in the two input frames, used by visual only orientation calculation for initialization
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    // update the feature depth from optimization
    // the input is inverse depth from optimization, while inside this function, it is true depth
    void setDepthVector(const VectorXd &x, bool set_solver_flag=true);

    // remove failed features based on solver flag
    void removeFailures();

    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    list<FeaturePerId> feature;

  private:
    // calculate the parallax of a feature point in the last two KeyFrames
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
};

#endif