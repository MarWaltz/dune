/**
 *    \file   rl.cpp
 *    \brief  Defines the Reinforcement Learning collision avoidance algorithm class.
 *    \author Martin Waltz
 */

// Local headers.
#include <DUNE/Navigation/rl.hpp>
#include <DUNE/DUNE.hpp>
//#include <DUNE/example-app/example-app.h>
//#include <eFLL/Fuzzy.h>

static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;

namespace DUNE
{
  namespace Navigation
  {
    //! Constructor
    rlAgent::rlAgent():
    DHEAD_MAX_(0.0)
    {}

    //! Destructor
    rlAgent::~rlAgent()
    {}
	
	void
	rlAgent::create(double DHEAD_MAX)
	{
		DHEAD_MAX_ = DHEAD_MAX;
	}

	std::tuple<double, double>  
	rlAgent::getAction(double psi_des, double U_des, const std::vector<double>& asv_state, const Math::Matrix& obst_states)
	{
		return std::make_tuple(0.5 * M_PI , U_des);
		//return std::make_tuple(psi_des + 0.05 * M_PI , U_des);
		/*
		Eigen::Vector2d Vs, Ps, Vo, Po, trans_Vo_Vs;
		double dist, theta_o_s, theta_obst, theta_obst_left, theta_obst_right, psi_desired, U_desired, cost;
		
		// A matrix with a row of [trans_Vo_Vs(0), trans_Vo_Vs(1), bound_left(0), bound_left(1), bound_right(0), bound_right(1), dist, 2*obs_radius] for each obstacle
		Math::Matrix VO_all;
		VO_all.resize(obst_states.rows(), 12); 

		Ps(0) = asv_state[0];
		Ps(1) = asv_state[1];
		Vs(0) = asv_state[3] * std::cos(asv_state[2]);
		Vs(1) = asv_state[3] * std::sin(asv_state[2]);
		
		double obs_radius = D_SAFE_;

		// Create velocity obstacles for each obstacle
		for (int i=0; i<obst_states.rows(); i++)
		{
			Po(0) = obst_states(i,0);
			Po(1) = obst_states(i,1);
			Vo(0) = obst_states(i,11)*std::cos(Angles::radians(obst_states(i,10)));
			Vo(1) = obst_states(i,11)*std::sin(Angles::radians(obst_states(i,10)));

			// RVO
			trans_Vo_Vs(0) = Ps(0)+0.5*(Vo(0)+Vs(0));
			trans_Vo_Vs(1) = Ps(1)+0.5*(Vo(1)+Vs(1));

			// VO
			//trans_Vo_Vs(0) = Ps(0)+Vo(0);
			//trans_Vo_Vs(1) = Ps(1)+Vo(1);

			dist = distance(Ps, Po);
			theta_o_s = atan2(Po(1)-Ps(1), Po(0)-Ps(0));
			if (2*obs_radius > dist)
			{
				dist = 2*obs_radius;
			}
			theta_obst = asin(2*obs_radius/dist);
			theta_obst_left = theta_o_s + theta_obst;
			theta_obst_right = theta_o_s - theta_obst;
			
			VO_all(i, 0) = trans_Vo_Vs(0);
			VO_all(i, 1) = trans_Vo_Vs(1);
			VO_all(i, 2) = std::cos(theta_obst_left);
			VO_all(i, 3) = std::sin(theta_obst_left);
			VO_all(i, 4) = std::cos(theta_obst_right);
			VO_all(i, 5) = std::sin(theta_obst_right);
			VO_all(i, 6) = dist;
			VO_all(i, 7) = 2*obs_radius;
			VO_all(i, 8) = Vo(0);
			VO_all(i, 9) = Vo(1);
			VO_all(i, 10) = Po(0);
			VO_all(i, 11) = Po(1);
		}
		std::tie(psi_desired, U_desired, cost) = intersect(Ps, psi_des, U_des, VO_all);
		
		double psi_os_temp = normalize_angle(psi_desired) - asv_state[2];
		double u_os_temp = U_desired / asv_state[3];
		//std::cout << "Psi:" << asv_state[2] << " + Psi_off:" << psi_os_temp << " = " << Angles::normalizeRadian(asv_state[2]+psi_os_temp) << " Psi_des:" << psi_desired << std::endl;
		//std::cout << "U:" << asv_state[3] << " + U_off:" << u_os_temp << " = "  << asv_state[3]*u_os_temp << " U_des:" << U_desired << std::endl; 
		return std::make_tuple(psi_os_temp, u_os_temp, cost);
		*/
	}

	/*
	double 
	velocityObstacle::distance(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Po)
	{
		Eigen::Vector2d d;
		double dist;
		d(0) = Po(0)-Ps(0);
		d(1) = Po(1)-Ps(1);
		dist = d.norm();
		return dist;
	}

	std::tuple<double, double> 
	rlAgent::calculateCPA(const Eigen::Vector2d& Ps, const Eigen::Vector2d& Vs, const Eigen::Vector2d& Po, const Eigen::Vector2d& Vo)
	{
		double delta_alpha = 0.0;
		double delta_chi = 0.0;
		double D_r, U_r, alpha_r, chi_r, beta, dcpa, tcpa;

		// Distance between self and targetship
        D_r = distance(Ps, Po);

        // Relative speed between self and targetship
        U_r = distance(Vs, Vo);

        // Alpha_r (True bearing of the targetship)
		if ((Po(1) - Ps(1) >= 0.0) && (Po(0) - Ps(0) >= 0.0))
		{
		    delta_alpha = 0.0;
		}
		else if ((Po(1) - Ps(1) >= 0.0) && (Po(0) - Ps(0)) < 0)
		{
		    delta_alpha = 0.0;
		}
		else if ((Po(1) - Ps(1) < 0.0) && (Po(0) - Ps(0)) < 0)
		{
		    delta_alpha = 2*M_PI;
		}
		else if ((Po(1) - Ps(1) < 0.0) && (Po(0) - Ps(0)) >= 0)
		{
		    delta_alpha = 2*M_PI;
		}
		alpha_r = atan2((Po(1)-Ps(1)), (Po(0)-Ps(0))) + delta_alpha; 

		// Chi_r (Relative course of targetship - from 0 to U_r)
		if ((Vo(1) - Vs(1) >= 0) && (Vs(0) - Vs(0) >= 0))
		{
		    delta_chi = 0;
		}
		else if ((Vs(1) - Vs(1) >= 0) && (Vs(0) - Vs(0) < 0))
		{
		    delta_chi = 0;
		}
		else if ((Vs(1) - Vs(1) < 0) && (Vs(0) - Vs(0) < 0))
		{
		    delta_chi = 2*M_PI;
		}
		else if ((Vs(1) - Vs(1) < 0) && (Vs(0) - Vs(0) >= 0))
		{
		    delta_chi = 2*M_PI;
		}
		chi_r = atan2((Vs(1)-Vs(1)), (Vs(0)-Vs(0))) + delta_chi;

		// beta
        beta = chi_r - alpha_r - M_PI;

        // DCPA and TCPA
        dcpa = std::abs(D_r * sin(beta));
        tcpa = (D_r * cos(beta)) / (std::abs(U_r)+1);

        return std::make_tuple(dcpa, tcpa);
	}


	// Normalize angle
	inline double 
	velocityObstacle::normalize_angle(double angle)
	{
		while(angle <= -M_PI) angle += 2*M_PI;
		while (angle > M_PI) angle -= 2*M_PI;
		return angle;
	}

	inline double 
	velocityObstacle::normalize_angle_360(double angle)
	{
		angle = fmod(angle,2*M_PI);
		if(angle < 0)
		angle += 2*M_PI;
		return angle;
	}
	*/
  }
}