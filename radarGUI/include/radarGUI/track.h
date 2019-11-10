#ifndef TRACK_START_H
#define TRACK_START_H
#include <vector>
#include "include/radarGUI/track_package.h"
#include "Dense"
#include "include/radarGUI/kalman_filter.h"
#include "include/radarGUI/FusionEKF.h"
#include "include/radarGUI/measurement_package.h"
using namespace std;

static Tools tools2;
const double PI = M_PI;

//航迹管理函数：调用下面四个函数
void  track(vector<MeasurementPackage> &measurement_pack_list,
            vector<TrackDataOutputPackage> &track_data_output_pack_list,
            vector<TempPointPackage>&temp_point_pack_list,
            vector<TrustTrackPackage>&trust_track_pack_list,
            int number_frame,int number_of_track,double dt,double time_accumulate);

//航迹起始函数
void track_start(vector<TempPointPackage>&temp_point_pack_list,
                 vector<NowFramePoint>&now_frame_point_pack_list,
                 vector<TrustTrackPackage>&trust_track_pack_list,
                 int K_start,double dt,
                 vector<TrackDataOutputPackage> &track_data_output_pack_list,
                 int number_of_track );

//航迹关联函数
void point_track_association(vector<NowFramePoint>&now_frame_point_pack_list,
                             vector<TrustTrackPackage>&trust_track_pack_list,
                             vector<TrackDataOutputPackage> &track_data_output_pack_list,
                             double dt,int K_association, double time_accumulate);

//航迹补点函数
void point_supplement(vector<TrustTrackPackage>&trust_track_pack_list,
                      vector<TrackDataOutputPackage> &track_data_output_pack_list,
                      double dt, double time_accumulate
                      );
//航迹消亡函数
void track_die_out(vector<TrustTrackPackage>&trust_track_pack_list,
                   vector<TrackDataOutputPackage> &track_data_output_pack_list,
                   int number_of_track
                   );

#endif // TRACK_START_H
