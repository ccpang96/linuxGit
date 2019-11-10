#include "include/radarGUI/track.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


/**
  TODO:定义一个函数对vector进行升序排列
 */
bool less_sort(TrackDataOutputPackage a,TrackDataOutputPackage b) {return (a.raw_track_data_output_(0) < b.raw_track_data_output_(0)); }
bool less_sort_d(TrustTrackPackage a,TrustTrackPackage b) {return (a.raw_track_data_output_(0) < b.raw_track_data_output_(0)); }


/**
 * @brief track  :航迹管理函数
 * @param measurement_pack_list :经过CFAR后的目标数据，内有多个目标
 * @param track_data_output_pack_list: 数据处理后，最终的输出
 * @param temp_point_pack_list :临时点迹数据
 * @param trust_track_pack_list :可靠航迹数据
 * @param number_frame :第几帧数据(批)
 * @param number_of_track : 第几条航迹
 * @param dt :两帧(批)数据之间的间隔
 * @param time_accumulate : 从第一批数据到now积累的时间
 */
void  track(vector<MeasurementPackage> &measurement_pack_list,
            vector<TrackDataOutputPackage> &track_data_output_pack_list,
            vector<TempPointPackage>&temp_point_pack_list,
            vector<TrustTrackPackage>&trust_track_pack_list,
            int number_frame,int number_of_track,double dt,double time_accumulate) {

    //float r_min = 0;   //真实目标的距离门限值
    //float r_max = 10000;
    double phi_min = static_cast<double>(-PI); //真实目标的方位角门限值
    double phi_max = static_cast<double>(PI);
   // float r_dot_min = 0; //真实目标的速度门限值
    //float r_dot_max = 10000;
    int DBF_inform = 1; //本项目中未用到通道号，即将所有通道号都设置为1

    int K_start = 100; //控制起始波门大小的系数，本程序中起始波门取了常数
    int K_association = 2; //控制相关波门大小的系数
    int gate_of_point_delete = 5; //点迹消除的门限次数 5

    /**先要判断新来的点是否为空*/
    size_t K = measurement_pack_list.size(); //经过CFAR以后的目标数据
    size_t N = temp_point_pack_list.size(); //临时点迹数据大小
    size_t H = trust_track_pack_list.size(); //可靠航迹数据包中的大小
    if (K > 0) {  //经过CFAR以后的目标数据不为空
        /**
         * TODO:数据预处理，后续这个判断还需要优化
         */
        auto itm = measurement_pack_list.begin();
        while(itm < measurement_pack_list.end()) {
            if((*itm).raw_measurements_(1) < phi_min || (*itm).raw_measurements_(1) > phi_max) { //如果方位角不满足条件，删除
                itm = measurement_pack_list.erase(itm);
            } else {
                ++itm;
            }
        }

        size_t M = measurement_pack_list.size(); //数据预处理后的目标个数

        TempPointPackage temp_point; //临时点迹数据 中间变量
        NowFramePoint  now_point;    //当前点迹数据 中间变量
        TempPointPackage  temp_point1; //临时点迹数据  中间变量
        vector<NowFramePoint> now_frame_point_pack_list; //不是第一批，且预处理后存在目标，这些目标就放进这个包里

        /**
          TODO:如果是第一批数据，就将它放入临时点迹中
        */
        if (number_frame == 1) {  //处理的是第一帧(批)数据
            time_accumulate = dt; //时间积累
            if(M < 0.01) { //经过数据预处理后的目标个数为0 说明第一批数据为空，接着处理下一批
                /**
                  TODO: 此处是否需要做些什么
                */
                //return;
            } else { //滤波后的输入数据不为空
                int number_of_unuse_point = 1; //点迹未用次数
                int flag_of_loop = 0; //航迹起始中，已用/未用标志，0表示未用
                for(size_t i = 0; i < M; ++i) { //数据预处理后的目标个数
                    temp_point.temp_point_ = measurement_pack_list[i].raw_measurements_;
                    temp_point.time_accumulate_ = time_accumulate;
                    temp_point.raw_temp_point_ << DBF_inform,number_of_unuse_point,gate_of_point_delete,flag_of_loop;  //通道号、点迹未使用次数、点迹消亡门限值、已用/未用
                    temp_point_pack_list.push_back(temp_point);
                } //第一帧(批)数据存放进临时点迹数据中
            }
        } else {  //如果输入的不是第一批数据
            time_accumulate += dt; //时间积累

            /**
              TODO:输入数据为空，让临时点迹数据未使用次数+1；且让可靠航迹消亡和补点
            */

            if (M < 0.01) {  //不是第一批，且经过数据预处理后的目标个数为0
                if (N > 0) { //临时点迹数据不为空,因为此帧数据中没有目标，所以无法关联，只能让临时点迹数据未使用次数+1
                    auto itc = temp_point_pack_list.begin();
                    while(itc < temp_point_pack_list.end()) {
                        (*itc).raw_temp_point_(1) += 1;   //未使用次数+1
                        if((*itc).raw_temp_point_(1) == (*itc).raw_temp_point_(2)) { //若点迹未使用次数等于删除门限
                            itc = temp_point_pack_list.erase(itc); //删除它
                        } else {
                            ++itc;
                        }
                    }
                 }

                if( H > 0) { //如果可靠航迹非空,就调用航迹消亡函数和补点函数

                    /**
                     * TODO: 后续考虑交换这两个函数的执行顺序，是先补点还是先消亡，私以为是先补点，不过师兄的仿真程序是先消亡
                     */
                    point_supplement(trust_track_pack_list,track_data_output_pack_list,dt,time_accumulate); //调用航迹补点函数
                    track_die_out(trust_track_pack_list,track_data_output_pack_list,number_of_track); //调用航迹消亡函数
                }

            } else { //不是第一批，且经过数据预处理后存在目标
                int number_of_unuse_point = 1; //点迹未用次数
                int flag_of_loop = 0; //航迹起始中，已用/未用标志，0表示未用
                for(size_t i = 0; i < M; ++i) {
                    now_point.now_point_ = measurement_pack_list[i].raw_measurements_;
                    now_point.time_accumulate_ = time_accumulate;  //积累时间
                    now_point.raw_now_frame_point_<< DBF_inform,number_of_unuse_point,gate_of_point_delete,flag_of_loop; //通道号、点迹未用次数,
                    now_frame_point_pack_list.push_back(now_point); //保存在待处理数据now_frame_point_pack_list中
                }

                if (H > 0) { //如果可靠航迹非空
                    //存在待处理数据，先进行点迹航迹关联过程，然后判断哪条航迹消亡，然后看哪些航迹未更新，进行补点。然后看哪些航迹未更新次数等于门限值，消亡。
                    point_track_association(now_frame_point_pack_list,trust_track_pack_list,track_data_output_pack_list,dt,K_association,time_accumulate);
                    point_supplement(trust_track_pack_list,track_data_output_pack_list,dt,time_accumulate); //调用航迹补点函数
                    track_die_out(trust_track_pack_list,track_data_output_pack_list,number_of_track); //调用航迹消亡
                }

                if(N < 0.01) { //如果临时点迹为空，将点迹航迹关联后剩余的点迹now_frame_point_pack_list放入临时点迹temp_point_pack_list
                    size_t J = now_frame_point_pack_list.size();
                    for (size_t j = 0 ;j < J; ++j ) {
                        temp_point1.temp_point_ = now_frame_point_pack_list[j].now_point_;
                        temp_point1.raw_temp_point_ = now_frame_point_pack_list[j].raw_now_frame_point_;
                        temp_point1.time_accumulate_ = now_frame_point_pack_list[j].time_accumulate_;
                        temp_point_pack_list.push_back(temp_point1); //将now点迹放入temp点迹
                    }
                } else { //否则就执行航迹起始，看看能否构成新的航迹
                    track_start(temp_point_pack_list,now_frame_point_pack_list,trust_track_pack_list,K_start,dt,track_data_output_pack_list,number_of_track);
                } //如果临时点迹非空
            }
        }
    } else {  //如果经过CFAR后的目标为空
        if(number_frame == 1) { //如果是第一批数据
            time_accumulate = 0;  //时间积累为0
        }
        else { //不是第一批数据
            time_accumulate += dt;
            if (N > 0) { //临时点迹数据不为空
                auto itc = temp_point_pack_list.begin();
                while(itc < temp_point_pack_list.end()) {
                    (*itc).raw_temp_point_(1) += 1;   //未使用次数+1
                    if((*itc).raw_temp_point_(1) == (*itc).raw_temp_point_(2)) { //若点迹未使用次数等于删除门限
                        itc = temp_point_pack_list.erase(itc); //删除它
                    } else {
                        ++itc;
                    }
                }
             }//临时点迹数据非空

            if( H > 0) { //如果可靠航迹非空
                point_supplement(trust_track_pack_list,track_data_output_pack_list,dt,time_accumulate); //调用航迹补点函数
                track_die_out(trust_track_pack_list,track_data_output_pack_list,number_of_track); //调用航迹消亡函数
            }
        } //如果不是第一批数据
    } //恒虚警处理后的数据为空


    /**
      TODO:一批数据处理结束后，所有航迹更新标志置0
    */
    size_t S = trust_track_pack_list.size();
    if (S > 0) {
        for (size_t s = 0; s < S; ++s) {
            trust_track_pack_list[s].raw_trust_track_(1) = 0; //所有航迹更新标志置0
        }

        sort(track_data_output_pack_list.begin(), track_data_output_pack_list.end(),  less_sort); //按航迹标号升序排列
        sort(trust_track_pack_list.begin(), trust_track_pack_list.end(),  less_sort_d); //按航迹标号升序排列
    }


}

/**
 * @brief track_start
 * 说明：航迹起始函数
 * 功能：采用滑窗法，将满足一定要求的两个点迹关联成一条航迹，如果某一个点迹和temp_point_pack_list中的点迹
 * 关联上了，那么它不能再与其他点迹进行关联，以保证每个点迹只使用一次。
 * 说明：track_start在临时点迹数据与当前帧经过点迹航迹关联后的剩余点迹数据中进行，依次查询两个数据package中的
 * 点迹，当两个点迹各方面之差都满足波门大小时，认为航迹起始成功，并初始化卡尔曼滤波器，将起始成功的航迹进行存储。
 * @param temp_point_pack_list 临时点迹数据
 * @param trust_track_pack_list 可靠航迹数据
 * @param track_data_output_pack_list 本帧数据处理完成后，输出的航迹数据
 * @param K_start  调整航迹起始波门的大小
 * @param time_accumulate 从第一批数据到此批处理数据之间所经过的时间
 * @param dt 每两批数据之间的间隔时间：
 * @param sigma_r 距离向测量噪声的标准差   显然应该写一个矩阵的 就是测量噪声协方差矩阵R_radar_
 * @param sigma_a 方位角测量噪声标准差
 * @param sigma_e 俯仰角测量噪声标准差
 * @param number_of_track 已形成的航迹数量
 */
void track_start(vector<TempPointPackage>&temp_point_pack_list,
                 vector<NowFramePoint>&now_frame_point_pack_list,
                 vector<TrustTrackPackage>&trust_track_pack_list,
                 int K_start,double dt,
                 vector<TrackDataOutputPackage> &track_data_output_pack_list,
                 int number_of_track ) {

    int number_of_point = 1; //是该航迹中的第一个点
    int flag_of_come_go = 0; //0来/1去标志 //本项目中未用到来去标志
    int flag_of_true_void = 0; //0实点/1虚点
    int flag_of_renew = 0; // 航迹更新标志 0未更新1更新
    int number_of_unrenew = 0; //航迹未被更新次数

    /**
     * TODO:这些波门的大小必然是要修改的
     */
    double r_gate_of_start = K_start * 10; //航迹起始中的距离波门大小
    double phi_gate_of_start = K_start * 2; //航迹起始中的角度波门大小
    double r_dot_gate_of_start = K_start * 10; //航迹起始中的速度波门大小

    FusionEKF tsEKF;
    Eigen::Vector4d first_point_x_; //新航迹笛卡尔坐标系下的第一点
    /**
     * TODO：这个P要修改，因为是初始化卡尔曼滤波器
     */
    Eigen::Matrix4d first_point_P_ = MatrixXd(4, 4);
    first_point_P_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;

    TrackDataOutputPackage temp_track_dataout;

    TrustTrackPackage temp_trust;
    size_t M = now_frame_point_pack_list.size();  //当前帧中待处理的点迹
    size_t N = temp_point_pack_list.size();  //临时点迹大小
    for (size_t k = 0; k < M ; ++k) {
        for (size_t l = 0; l < N; ++l) {
            if((now_frame_point_pack_list[k].raw_now_frame_point_(3) == 0) && (temp_point_pack_list[l].raw_temp_point_(3) == 0)) {  //如果该点没有被查询

                if(abs(now_frame_point_pack_list[k].now_point_(0) - temp_point_pack_list[l].temp_point_(0)) <= r_gate_of_start
                        && abs(now_frame_point_pack_list[k].now_point_(1) - temp_point_pack_list[l].temp_point_(1)) <= phi_gate_of_start
                               && abs(now_frame_point_pack_list[k].now_point_(2) - temp_point_pack_list[l].temp_point_(2)) <= r_dot_gate_of_start) {
                    //点迹与点迹关联成功，航迹起始
                    temp_point_pack_list[l].raw_temp_point_(3) = 1; //临时点迹中该点已经被使用
                    now_frame_point_pack_list[k].raw_now_frame_point_(3) = 1; //当前帧点迹中该点已被使用

                    number_of_track += 1; //形成的可靠航迹数+1 也就是航迹标号

                    //temp_point_pack_list[l].temp_point_ 新航迹中的第一个点
                    //now_frame_point_pack_list[k].now_point_ 新航迹中的第二个点

                    //此处可以加入判断目标运动方向的代码

                    /**
                      @brief:第一步：利用前两点初始化扩展卡尔曼滤波器
                    */
                    first_point_x_ = tools2.RadarPolarToCartesian(temp_point_pack_list[l].temp_point_);  //将航迹中的第一个点转到笛卡尔坐标系
                    tsEKF.onlyPredict(first_point_x_,first_point_P_,dt); //先利用第一点进行一次初始化

                    //将第一个点迹信息push进track_data_output_pack_list
                    temp_track_dataout.track_dataout_point_ = tools2.RadarCartesianToPolar(tsEKF.ekf_.x_);  //新航迹第一个的点迹信息
                    //分别是 航迹标识、来去、是第几个点、实点/虚点，通道号
                    temp_track_dataout.raw_track_data_output_ << number_of_track,flag_of_come_go,number_of_point,flag_of_true_void,temp_point_pack_list[l].raw_temp_point_(0);
                    temp_track_dataout.time_accumulate_ = temp_point_pack_list[l].time_accumulate_;
                    track_data_output_pack_list.push_back(temp_track_dataout); //将新航迹中的第一个点加入航迹信息


                    tsEKF.ProcessMeasurement(now_frame_point_pack_list[k].now_point_); //再利用第2点进行一次更新


                    //然后将第二个点迹信息push进track_data_output_pack_list
                    temp_track_dataout.track_dataout_point_ = tools2.RadarCartesianToPolar(tsEKF.ekf_.x_);; //新航迹第2个点的点迹信息
                    //分别是 航迹标识、来去、是第几个点、实点/虚点，通道号
                    temp_track_dataout.raw_track_data_output_ << number_of_track, flag_of_come_go,number_of_point + 1,flag_of_true_void,now_frame_point_pack_list[k].raw_now_frame_point_(0);
                    temp_track_dataout.time_accumulate_  = now_frame_point_pack_list[k].time_accumulate_;
                    track_data_output_pack_list.push_back(temp_track_dataout);


                    //更新新航迹在可靠航迹中的最后一个点的信息
                    temp_trust.last_point_x_ = tsEKF.ekf_.x_; //状态变量信息
                    temp_trust.last_point_P_ = tsEKF.ekf_.P_; //最后一个点的协方差矩阵信息
                    temp_trust.raw_track_data_output_ << number_of_track,flag_of_come_go,number_of_point + 1,flag_of_true_void,now_frame_point_pack_list[k].raw_now_frame_point_(0);
                    //分别是航迹未被实测数据更新次数，航迹更新标志，航迹消亡门限值
                    temp_trust.raw_trust_track_ << number_of_unrenew,flag_of_renew,now_frame_point_pack_list[k].raw_now_frame_point_(2);

                    trust_track_pack_list.push_back(temp_trust);  //更新可靠信息

                }//如果距离、角度、速度都位于起始门限内，说明航迹起始成功
            }
        }//遍历临时点迹数据中的所有点迹
    }//遍历当前帧中待处理的所有点迹

    /**
      @brief:删除当前帧点迹数据和临时点迹数据中已经使用过的数据，并将临时点迹数据中剩余数据的未被使用次数+1，然后将输入点迹信息放入临时点迹信息中。
    */

    //删除当前帧待处理点迹中的已关联的点迹
    auto itn = now_frame_point_pack_list.begin();
    while(itn < now_frame_point_pack_list.end() ) {
        if ((*itn).raw_now_frame_point_(3) == 1) { //如果点迹已用,删除它
            itn = now_frame_point_pack_list.erase(itn);
        } else {
            ++itn;
        }
    }

    //删除临时点迹中已经关联的点迹
    auto itt = temp_point_pack_list.begin();
    while(itt < temp_point_pack_list.end() ) {
        if ((*itt).raw_temp_point_(3) == 1) { //如果点迹已用,删除它
            itt = temp_point_pack_list.erase(itt);
        } else {
            ++itt;
        }
    }


    //处理临时点迹数据中剩余的点迹信息
    auto itc = temp_point_pack_list.begin();
    while(itc < temp_point_pack_list.end()) {
        (*itc).raw_temp_point_(1) += 1;   //未使用次数+1
        if((*itc).raw_temp_point_(1) == (*itc).raw_temp_point_(2)) { //若点迹未使用次数等于删除门限
            itc = temp_point_pack_list.erase(itc); //删除它
        } else {
            ++itc;
        }
    }

    TempPointPackage temp_point;
    //将当前输入点迹中未关联上的点迹数据送入临时点迹数据中
    size_t H = now_frame_point_pack_list.size();
    if (H > 0) {
        for (size_t i = 0 ; i < H; ++i) {


            temp_point.temp_point_ = now_frame_point_pack_list[i].now_point_;
            temp_point.raw_temp_point_ = now_frame_point_pack_list[i].raw_now_frame_point_;
            temp_point.time_accumulate_ = now_frame_point_pack_list[i].time_accumulate_;

            temp_point_pack_list.push_back(temp_point); //能不能用类强制转换
        }
    }

}


/**
 * @brief point_track_association
 * name:点迹航迹关联函数
 * 功能：经过CFAR和数据预处理的数据与可靠航迹数据(trust_track_pack_list)关联，能够关联上的，更新可靠航迹数据(trust_track_pack_list),
 * 否则存入临时点迹数据(temp_point_pack_list),用于航迹起始，或者最终成为噪声。
 * 一个点迹只能关联一条航迹，即最先与该点关联上的航迹，用该点来更新该航迹（细读一下，第一次读未知具体意思）
 *
 * 说明：点迹航迹关联在当前帧待处理数据(now_frame_point_pack_list)与已经形成的可靠航迹（trust_track_pack_list）之间进行，
 * 依次对每条航迹进行循环，查找关联上的点迹，并将点迹进行扩展卡尔曼滤波后更新航迹。
 * @param now_frame_point_pack_list  经过CFAR和数据预处理后的当前帧待处理数据
 * @param trust_track_pack_list      可靠航迹数据
 * @param track_data_output_pack_list  本帧(批)数据处理完毕后，输出的航迹数据
 * @param dt   两帧数据之间的时间差
 * @param K_association  可选参数，用于调节相关波门大小
 * @param time_accumulate  积累时间，即从第一批数据到此批数据之间所经过的时间
 */

void point_track_association(vector<NowFramePoint>&now_frame_point_pack_list,
                             vector<TrustTrackPackage>&trust_track_pack_list,
                             vector<TrackDataOutputPackage> &track_data_output_pack_list,
                             double dt,int K_association, double time_accumulate) {
    FusionEKF ptaEKF;
    double sigma_x_predict;
    double sigma_y_predict;
    double x_gate_of_association;  //x方向的相关波门
    double y_gate_of_association;   //y方向的相关波门
    TrackDataOutputPackage temp_track_dataout;

    VectorXd temp_now_point_Cartesian = VectorXd(4); //用于存放now_point从极坐标系变换到笛卡尔坐标下的临时变量

    size_t N = trust_track_pack_list.size(); //可靠航迹个数
    size_t M = now_frame_point_pack_list.size(); // 当前帧中待处理的点迹的数目，按照对方那边的说法，1s一次性给我70多个包，每个包中包含1~n个point(n是目标个数)
    for (size_t k = 0; k < N; ++k) {   //遍历可靠航迹
        for (size_t l = 0; l < M; ++l) { //遍历当前帧中待处理的点迹
            if (now_frame_point_pack_list[l].raw_now_frame_point_(3) == 0 &&  //点迹未使用
                    trust_track_pack_list[k].raw_trust_track_(1) == 0)  {// 航迹未更新

                /**
                  @brief: 第一步：先计算出可靠航迹中最后一个点的扩展卡尔曼预测值
                */
                ptaEKF.onlyPredict(trust_track_pack_list[k].last_point_x_, trust_track_pack_list[k].last_point_P_,dt); //航迹上最后一个点的状态变量和协方差矩阵


                /**
                  @brief: 第二步：计算相关波门
                */
                sigma_x_predict = ptaEKF.ekf_.P_(0,0); //x方向的协方差
                sigma_y_predict = ptaEKF.ekf_.P_(1,1); //y方向的协方差


                /**
                  TODO:注意此处这个波门与仿真值相比，没有加入测量值的噪声R  如果后续需要调节应该修改此处
                */
                x_gate_of_association = K_association*sqrt(pow(sigma_x_predict,2) + (pow(ptaEKF.noise_ax,2) * pow(dt,4) / 4));
                y_gate_of_association = K_association*sqrt(pow(sigma_y_predict,2) + (pow(ptaEKF.noise_ay,2) * pow(dt,4) / 4));


                /**
                  TODO: 第三步：找到在波门内的当前帧中的目标点，对其进行扩展卡尔曼滤波
                */
                temp_now_point_Cartesian = tools2.RadarPolarToCartesian(now_frame_point_pack_list[l].now_point_);
                if (abs(ptaEKF.ekf_.x_(0) - temp_now_point_Cartesian(0)) <= x_gate_of_association &&
                        abs(ptaEKF.ekf_.x_(1) - temp_now_point_Cartesian(1)) <= y_gate_of_association) {  //点迹航迹关联成功

                        now_frame_point_pack_list[l].raw_now_frame_point_(3) = 1; // 该点被关联，已用

                        /**
                          TODO:对其做扩展卡尔曼估计
                        */
                        ptaEKF.ProcessMeasurement(now_frame_point_pack_list[l].now_point_);
                        trust_track_pack_list[k].last_point_x_ = ptaEKF.ekf_.x_; //更新航迹中最后一个点状态
                        trust_track_pack_list[k].last_point_P_ = ptaEKF.ekf_.P_; //更新航集中最后一个点的协方差矩阵
                        trust_track_pack_list[k].raw_track_data_output_(2) += 1;  // 这个点是这条航迹的第几个点  +1
                        trust_track_pack_list[k].raw_track_data_output_(3) = 0;  //实点/补点标志，实点为0，补点为1
                        trust_track_pack_list[k].raw_trust_track_(0) = 0;  //航迹未被实点更新次数置0
                        trust_track_pack_list[k].raw_trust_track_(1) = 1;  //更新标志为1

                        /**
                            TODO:记得弄清楚这句话的意思：将当前帧点的删除门限赋予给可靠航迹中的航迹消亡的门限值
                        */
                        trust_track_pack_list[k].raw_trust_track_(2) = now_frame_point_pack_list[l].raw_now_frame_point_(2); //

                        /**
                            TODO:如果不是刚起始的航迹，则输出将点迹加入到track_data_output_pack_list
                        */
                        if (trust_track_pack_list[k].raw_trust_track_(2) > 2) {//判断该点是该航迹的第几个点，如果大于2说明不是刚起始的航迹

                            //此处应该将扩展卡尔曼的估计值放入航迹信息Track_data_output_list，而且应该是将4维的估计值从笛卡尔坐标系变换到极坐标系
                            temp_track_dataout. track_dataout_point_ = tools2.RadarCartesianToPolar(trust_track_pack_list[k].last_point_x_);
                            temp_track_dataout.time_accumulate_ = time_accumulate;  //时间积累
                            temp_track_dataout.raw_track_data_output_ =  trust_track_pack_list[k].raw_track_data_output_; //航迹属性
                            track_data_output_pack_list.push_back(temp_track_dataout); //输出航迹
                        }
                } //点迹与航迹相关成功
            } //如果当前帧的目标点未用，且可靠航迹也未更新
        } //遍历当前帧数据包中的目标点
    }  //遍历可靠航迹数据包内的航迹


    /**
      TODO:删除与可靠航迹关联上的点迹信息
    */
    auto itt = now_frame_point_pack_list.begin();
    while(itt < now_frame_point_pack_list.end() ) {
        if ((*itt).raw_now_frame_point_(3) == 1) { //如果点迹已用,删除它
            itt = now_frame_point_pack_list.erase(itt);
        } else {
            ++itt;
        }
    }

    /**
      TODO:track_data_output_pack_list按照航迹标志号排序
    */
    sort(track_data_output_pack_list.begin(), track_data_output_pack_list.end(),less_sort); //升序排列
}



/**
 * @brief point_supplement
 * name:航迹补点函数
 * 功能：当航迹没有得到实测点迹进行更新，则采取航迹补点方法，即利用航迹的最后一个点迹的扩展卡尔曼预测值作为更新点迹。
 * @param trust_track_pack_list  可靠航迹数据
 * @param track_data_output_pack_list 当前帧数据处理完成后，输出的航迹数据
 * @param dt  两帧（批）数据之间的间隔时间
 * @param time_accumulate 积累时间，即从第一帧（批）数据到当前帧数据之间所经过的时间
 */
void point_supplement(vector<TrustTrackPackage>&trust_track_pack_list,
                      vector<TrackDataOutputPackage> &track_data_output_pack_list,
                      double dt,double time_accumulate
                      ) {

    MatrixXd F;  // 状态转移矩阵F 相比于实例化一个类，宁愿在栈上开辟一个4*4数组
    F = MatrixXd(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    vector<int>no_update_track_index_pack_list; //未被更新的可靠航迹的index值pack
    TrackDataOutputPackage no_updata_temp_track_dataout;  //航迹信息 中间变量
    VectorXd track_dataout_point = VectorXd(3); //放入航迹信息中的点迹
    /**
     * @brief 第一步：找出都是哪些航迹未被更新
     */
    size_t N = trust_track_pack_list.size(); //航迹个数
    for (size_t k = 0; k < N; ++k) {

         if (trust_track_pack_list[k].raw_trust_track_(1) == 0 ) { //航迹更新标志 = 0，即航迹未更新

             //那就将这个第k条航迹加入未被更新列表
             no_update_track_index_pack_list.push_back(static_cast<int>(k));  //这个未被更新的的航迹委屈的说："别人都更新了，为啥不更新我"
         }
     }

    /**
     * @brief 第二步：对未被更新(委屈)的航迹进行补点，即采用扩展卡尔曼的进行预测值，用预测值进行更新
     */
    size_t M = no_update_track_index_pack_list.size(); //未被更新的航迹的数目
    for (size_t m = 0 ; m < M; ++m) {

        //采用卡尔曼预测，得到补点,并用补点更新航迹信息
        trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].last_point_x_ =
                F*trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].last_point_x_;

        trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].raw_track_data_output_(2) += 1; //航迹点数加一
        trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].raw_track_data_output_(3) = 1; //补点标志位置1
        trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].raw_trust_track_(0) += 1; //航迹未用实测点更新次数加1


        //将补点得到的状态变量先变换到极坐标中去
        track_dataout_point = tools2.RadarCartesianToPolar(trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].last_point_x_);

        no_updata_temp_track_dataout.track_dataout_point_  =  track_dataout_point; //航迹信息的更新点迹
        no_updata_temp_track_dataout.time_accumulate_ = time_accumulate; //积累时间
        no_updata_temp_track_dataout.raw_track_data_output_ =  trust_track_pack_list[static_cast<size_t>(no_update_track_index_pack_list[m])].raw_track_data_output_;

        //更新航迹信息
        track_data_output_pack_list.push_back(no_updata_temp_track_dataout); //将补点加入航迹信息
    }
    //对航迹信息和可靠航迹数据排序
    /**
     * TODO:这两个函数记得检查下
     */
    sort(track_data_output_pack_list.begin(), track_data_output_pack_list.end(),less_sort); //升序排列
    sort(trust_track_pack_list.begin(),trust_track_pack_list.end(),less_sort_d);

}


/**
 * @brief track_die_out
 * name: 航迹消亡函数
 * 功能：当未用实测点更新次数达到该航迹消亡的门限值时，该条航迹消亡（为啥不叫死亡呢，小声哔哔）,即删除该条航迹的所有信息
 * @param trust_track_pack_list   可靠航迹数据
 * @param track_data_output_pack_list  当前帧（批）数据处理完毕后，输出的航迹数据
 * @param member_of_track  已经形成的航迹个数
 */
void track_die_out(vector<TrustTrackPackage>&trust_track_pack_list,
                   vector<TrackDataOutputPackage> &track_data_output_pack_list,
                   int number_of_track
                   ) {

    int die_track_index;
    vector<int>die_track_index_pack_list;  //已经消亡的的航迹识别标志
    /**
     * @brief 第一步：找出都是哪些航迹消亡。
     */
    size_t N = trust_track_pack_list.size(); //可靠航迹个数
     for (size_t k = 0; k < N; ++k) {  //遍历可靠航迹数据中的每条航迹
         if (trust_track_pack_list[k].raw_trust_track_(0) == trust_track_pack_list[k].raw_trust_track_(2)) { //未被实测数据更新次数 = 航迹消亡门限值
             die_track_index = trust_track_pack_list[k].raw_track_data_output_(0); //即将消亡的航迹的识别标志
             die_track_index_pack_list.push_back(die_track_index);
         }  //如果第k条航迹的未被更新次数值达到航迹消亡的未被更新次数的门限值
     }

     /**
    * @brief M 消亡的航迹的数目
    */
   size_t M = die_track_index_pack_list.size(); //为什么不改成迭代器，because lazy
   for (size_t k = 0 ; k < M; ++k) {

       /**
        * @brief 在track_data_output_pack_list中找到标识符为die_track_index_pack_list[k]的这条消亡航迹，然后删除它
        */
       auto ito = track_data_output_pack_list.begin(); //it类型是vector<TrackDataOutputPackage>::iterator
       while(ito < track_data_output_pack_list.end() ) { //注意使用while的技巧，因为一直在erase，所以vector的大小在一直变化
           if((*ito).raw_track_data_output_(0) == die_track_index_pack_list[k]) { //在track_data_output_pack_list中找到标识符为死了的航迹
               ito = track_data_output_pack_list.erase(ito); //erase返回的是当前删除元素的下一个元素的迭代器
           } else {
               ++ito;
           }
       }

       /**
        * @brief 在trust_track_pack_list中找到标识符为die_index_pack_list[k]的这条消亡航迹，然后删除它
        */
       auto itt = trust_track_pack_list.begin();
       while(itt < trust_track_pack_list.end() ) {
           if ((*itt).raw_track_data_output_(0) == die_track_index_pack_list[k]) {
               itt = trust_track_pack_list.erase(itt);
           } else {
               ++itt;
           }
       }

       /**
        * @brief 如果消亡的不是最后一条航迹，需要将航迹号重新赋值
        */
       if (die_track_index_pack_list[k] < number_of_track) {

           for (int cnt_index = die_track_index_pack_list[k] + 1; cnt_index < number_of_track; ++cnt_index) {

               /**
                * @brief 修改可靠航迹数据trust_track_pack_list中的航迹标识符
                */
               auto itt = trust_track_pack_list.begin();
               while (itt < trust_track_pack_list.end() ) {
                   if ((*itt).raw_track_data_output_(0) == cnt_index) //
                       (*itt).raw_trust_track_(7) = cnt_index -1;
                    ++itt;
               }

               /**
                * @brief 修改航迹信息数据track_data_output_pack_list中的航迹标识符
                */

               auto ito = track_data_output_pack_list.begin();
               while (ito < track_data_output_pack_list.end() ) {
                   if((*ito).raw_track_data_output_(0) == cnt_index)
                       (*ito).raw_track_data_output_(3) = cnt_index -1;
                   ++ito;
               }
           } //遍历死了的第k条航迹后面的航迹，将它们的航迹标识符都减小1
       }

       /**
      * @brief 因为找到部分点迹后，更新die_track_index_pack_list中的值
      */

       /**
         TODO:以后将j改为k+1试一下
       */
       for (size_t  j = 0; j < M;  ++j ) { //注意此处j = 0;或者j = k +1;都可以
            die_track_index_pack_list[j] = die_track_index_pack_list[j] - 1;
       }
   }

   number_of_track = number_of_track - static_cast<int>(M); //更新一下已经形成的航迹的数目
}


































