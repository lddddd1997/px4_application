#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint
{
private:
    // double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _Ct;
    
    Eigen::VectorXd _Px, _Py, _Pz;

    Eigen::MatrixXd getQ(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getM(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num, const int d_order);

    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                         const Eigen::MatrixXd &M,
                                         const Eigen::MatrixXd &Ct,
                                         const Eigen::VectorXd &WayPoints1D,
                                         const Eigen::VectorXd &StartState1D,
                                         const Eigen::VectorXd &EndState1D,
                                         const int seg_num, 
                                         const int d_order);

public:
    TrajectoryGeneratorWaypoint();

    ~TrajectoryGeneratorWaypoint();

    Eigen::MatrixXd PolyQPGeneration(
        const int d_order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time);

    Eigen::VectorXd timeAllocation(Eigen::MatrixXd &Path, double vel, double acc);
    Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int order, int k, double t);
    int Factorial(int x);
};

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(const int d_order,           // the order of derivative，3 for min jerk，4 for min snap
                                                              const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
                                                              const Eigen::MatrixXd &Vel,  // boundary velocity
                                                              const Eigen::MatrixXd &Acc,  // boundary acceleration
                                                              const Eigen::VectorXd &Time) // time allocation in each segment
{
    int p_order = 2 * d_order - 1;      // the order of polynomial
    int p_num1d = p_order + 1;          // the number of Coefficients in each segment
    int seg_num = Time.size();          // the number of segments

    Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(seg_num, 3 * p_num1d);     // position(x,y,z), so we need (3 * p_num1d) coefficients

    Eigen::VectorXd Px(p_num1d * seg_num);     // coefficients in each axis
    Eigen::VectorXd Py(p_num1d * seg_num);
    Eigen::VectorXd Pz(p_num1d * seg_num);

    // enforce initial and final position,velocity and accleration, for higher order derivatives, just assume them be 0
    Eigen::MatrixXd StartState(d_order, 3);
    Eigen::MatrixXd EndState(d_order, 3);
    StartState.row(0) = Path.row(0);
    StartState.row(1) = Vel.row(0);
    StartState.row(2) = Acc.row(0);
    EndState.row(0) = Path.row((Path.rows()-1));
    EndState.row(1) = Vel.row(1);
    EndState.row(2) = Acc.row(1);
    if(d_order == 4)
    {
        StartState.row(3) = Eigen::VectorXd::Zero(3);  // jerk
        EndState.row(3) = Eigen::VectorXd::Zero(3); 
    }
    // cout << " StartState = " << endl;
    // cout << StartState << endl;
    // cout << " EndState = " << endl;
    // cout << EndState << endl;


    // Eigen::MatrixXd DiffMat = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    // for(int i = 0; i < DiffMat.rows()-1; i++)
    //     DiffMat(i,i+1) = i+1;
    // cout << " DiffMat = " << endl;
    // cout << DiffMat << endl;

    // Eigen::MatrixXd A = Eigen::MatrixXd::Identity(p_num1d, p_num1d);
    // for(int i = 0; i < d_order; i++)   
    // {
    //     A *= DiffMat;                // coefficients of n-order derivative
    //     cout << " A = " << endl;
    //     cout << A << endl;
    // } 
 

    _Q = Eigen::MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
    _M = Eigen::MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
    _Ct = Eigen::MatrixXd::Zero(2 * d_order * seg_num, d_order * (seg_num + 1));

    for(int seg_index = 0; seg_index < seg_num; seg_index++)
    {
        // calculate Matrix Q
        _Q.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time, seg_index);
        // calculate Matrix M
        _M.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, seg_index);
    }
    // calculate Matrix Ct
    _Ct = getCt(seg_num, d_order);

    // cout << " Q = " << endl;
    // cout << _Q << endl;
    // cout << " M = " << endl;
    // cout << _M << endl;
    // cout << " Ct = " << endl;
    // cout << _Ct << endl;

    Px = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(0), StartState.col(0), EndState.col(0), seg_num, d_order);
    Py = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(1), StartState.col(1), EndState.col(1), seg_num, d_order);
    Pz = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(2), StartState.col(2), EndState.col(2), seg_num, d_order);
    // cout << " Px = " << endl;
    // cout << Px << endl;
    // cout << " Py = " << endl;
    // cout << Py << endl;
    // cout << " Pz = " << endl;
    // cout << Pz << endl;

    for(int i = 0; i < seg_num; i++)
    {
        PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d);
    }
    // cout << " PolyCoeff = " << endl;
    // cout << PolyCoeff << endl;

    return PolyCoeff;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    // calculate Matrix Q_k of the seg_index-th segment
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < p_num1d; i++)
    {
        for (int j = 0; j < p_num1d; j++)
        {
            if (i >= p_num1d - d_order && j >= p_num1d - d_order)
            {
                Q_k(i, j) = (Factorial(i) / Factorial(i - d_order)) * ((Factorial(j) / Factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time(seg_index), (i + j - 2 * d_order + 1)); // Q of one segment
            }
        }
    }
    // cout << " Q_k = " << endl;
    // cout << Q_k << endl;

    return Q_k;
}


Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    Eigen::MatrixXd M_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
    Eigen::VectorXd t_pow = Eigen::VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        t_pow(i) = pow(Time(seg_index),i);
    }
    // cout << "t_pow = " << endl;
    // cout << t_pow << endl;

    if(p_num1d == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
               1, t_pow(1),   t_pow(2),   t_pow(3),    t_pow(4),    t_pow(5),    t_pow(6),    t_pow(7),
               0,     1   , 2*t_pow(1), 3*t_pow(2),  4*t_pow(3),  5*t_pow(4),  6*t_pow(5),  7*t_pow(6),
               0,     0   ,     2     , 6*t_pow(1), 12*t_pow(2), 20*t_pow(3), 30*t_pow(4), 42*t_pow(5),
               0,     0   ,     0     ,     6     , 24*t_pow(1), 60*t_pow(2),120*t_pow(3),210*t_pow(4);
    }
    // cout << "M_k = " << endl;
    // cout << M_k << endl;

    return M_k;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(const int seg_num, const int d_order)
{
    int d_num = 2 * d_order * seg_num;
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;
    // int dp_num = (d_order - 1) * mid_waypts_num;

    Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(d_num, df_and_dp_num);
    
    // Ct for the first segment: pos,vel,acc,(jerk)
    Ct.block(0, 0, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);
    // Ct for the last segment: pos,vel,acc,(jerk)
    Ct.block(d_num - d_order, df_num - d_order, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

    for(int mid_waypts_index = 0; mid_waypts_index < mid_waypts_num; mid_waypts_index++)
    {
        // Ct for middle waypoints: pos
        Ct(d_order+2*d_order*mid_waypts_index, d_order+mid_waypts_index) = 1;
        Ct(d_order+(d_order+2*d_order*mid_waypts_index), d_order+mid_waypts_index) = 1;

        // Ct for middle waypoints: vel
        Ct(d_order+1+2*d_order*mid_waypts_index, df_num+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+1+2*d_order*mid_waypts_index), df_num+(d_order-1)*mid_waypts_index) = 1;

        // Ct for middle waypoints: acc
        Ct(d_order+2+2*d_order*mid_waypts_index, (df_num+1)+(d_order-1)*mid_waypts_index) = 1;
        Ct(d_order+(d_order+2+2*d_order*mid_waypts_index), (df_num+1)+(d_order-1)*mid_waypts_index) = 1;

        if(d_order == 4)  // minimum snap
        {
            // Ct for middle waypoints: jerk
            Ct(d_order+3+2*d_order*mid_waypts_index, (df_num+2)+(d_order-1)*mid_waypts_index) = 1;
            Ct(d_order+(d_order+3+2*d_order*mid_waypts_index), (df_num+2)+(d_order-1)*mid_waypts_index) = 1;   
        }
    }
    // cout << "Ct = " << endl;
    // cout << Ct << endl;

    return Ct;
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                                                  const Eigen::MatrixXd &M,
                                                                  const Eigen::MatrixXd &Ct,
                                                                  const Eigen::VectorXd &WayPoints1D,
                                                                  const Eigen::VectorXd &StartState1D,
                                                                  const Eigen::VectorXd &EndState1D,
                                                                  const int seg_num,
                                                                  const int d_order)
{
    int df_and_dp_num = d_order * (seg_num + 1);
    int mid_waypts_num = seg_num - 1;
    int df_num = 2 * d_order + mid_waypts_num;
    int dp_num = (d_order - 1) * mid_waypts_num;

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

    // compute dF
    Eigen::VectorXd dF(df_num);
    dF.head(d_order) = StartState1D;    // start state: pos,vel,acc,(jerk)
    dF.segment(d_order, mid_waypts_num) = WayPoints1D.segment(1,WayPoints1D.rows()-2);  // middle waypoints: pos
    dF.tail(d_order) = EndState1D;      // end state: pos,vel,acc,(jerk)
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;   // closed-form solution of Unconstrained quadratic programming

    Eigen::VectorXd dF_and_dP(df_and_dp_num);
    dF_and_dP << dF, dP;
    Eigen::VectorXd PolyCoeff1D = M_inv * Ct * dF_and_dP;   // all coefficients of one segment

    return PolyCoeff1D;
}


Eigen::VectorXd TrajectoryGeneratorWaypoint::timeAllocation(Eigen::MatrixXd &Path, double vel, double acc) // 梯形速度曲线
{ 
    Eigen::VectorXd time(Path.rows() - 1);

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = vel * vel / (2 * acc);  // v1^2 - v0^2 / 2 * a
        double x2 = distance - 2 * x1; // 匀速阶段位移dist - x加速 - x减速
        double t1 = vel / acc; // 加速阶段时间
        double t2 = x2 / vel; // 匀速阶段
        time(i) = 2 * t1 + t2; // 总时间=加速 + 匀速 + 减速
    }
    // cout << time << endl;

    return time;
}

Eigen::Vector3d TrajectoryGeneratorWaypoint::getPosPoly(Eigen::MatrixXd polyCoeff, int minimum_type, int k, double t)
{
    Eigen::Vector3d pos;
    int poly_num = minimum_type * 2;
    Eigen::VectorXd vec_time  = Eigen::VectorXd::Zero( poly_num );
    vec_time(0) = 1.0;
    for(int i = 1; i < poly_num; i ++)
    {
        vec_time(i) = t * vec_time(i - 1);
    }

    for ( int dim = 0; dim < 3; dim++ )
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * poly_num, poly_num ); // 取出每一段的多项式系数
        pos(dim) = coeff.dot(vec_time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return pos;
}

#endif
