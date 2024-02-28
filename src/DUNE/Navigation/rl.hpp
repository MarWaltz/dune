/**
 *    \file   rl.hpp
 *    \brief  Declares the Reinforcement Learning collision avoidance algorithm class.
 *    \author Martin Waltz.
 */

#ifndef DUNE_NAVIGATION_RL_HPP_INCLUDED_
#define DUNE_NAVIGATION_RL_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <Eigen/Dense>
//#include <torch/script.h> 
//#include <iostream>
//#include <memory>
//#include <eFLL/Fuzzy.h>

namespace DUNE
{
  namespace Navigation
  {
	// Export DLL Symbol.
    class DUNE_DLL_SYM rlAgent;
    
		class rlAgent
		{
		public:
			// Constructor
			rlAgent();
		
			// Destructor
			~rlAgent();
		
			// Ressource initialization
			void create(double DHEAD_MAX);

			// Control action (heading + speed)	
			std::tuple<double, double> getAction(double psi_des, double U_des, const std::vector<double>& asv_state, const Math::Matrix& obst_states);

			// Setters
			void setDHeadMax(double dHeadMax);

			// Getters
			double getDHeadMax();

		private:
			double DHEAD_MAX_;
		};
  }
}

#endif
