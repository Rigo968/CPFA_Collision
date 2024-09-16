#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm> 

#pragma push_macro("slots")
#undef slots
#include "Python.h"
#pragma pop_macro("slots")

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve

class CPFA_loop_functions : public argos::CLoopFunctions
{

	friend class CPFA_controller;
	friend class CPFA_qt_user_functions;

	public:

		CPFA_loop_functions();
	   
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
		/* public helper functions */
		void UpdatePheromoneList();
		void SetFoodDistribution();

		argos::Real getSimTimeInSeconds();

		std::vector<argos::CColor>   TargetRayColorList;

		struct RobotState {
			CVector2 position; // Current position (x, y)
			//make velocity  8 cm/s
			//CVector2 velocity; // Current velocity (vx, vy)

			argos::CRadians heading;      // Current heading (theta)
		};
		struct CollisionInfo {
			size_t robot1;    // Index of the first robot
			size_t robot2;    // Index of the second robot
			size_t time_step; // Time step at which the collision is predicted

			CollisionInfo(size_t r1, size_t r2, size_t ts)
				: robot1(r1), robot2(r2), time_step(ts) {}
		};
		unsigned int getNumberOfRobots();
        void increaseNumDistributedFoodByOne();
		double getProbabilityOfSwitchingToSearching();
		double getProbabilityOfReturningToNest();
		double getUninformedSearchVariation();
		double getRateOfInformedSearchDecay();
		double getRateOfSiteFidelity();
		double getRateOfLayingPheromone();
		double getRateOfPheromoneDecay();
		
	protected:

		void setScore(double s);

		argos::CRandom::CRNG* RNG;
                size_t NumDistributedFood;
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t FoodDistribution;
		size_t FoodItemCount;
		size_t PowerlawFoodUnitCount;
		size_t NumberOfClusters;
		size_t ClusterWidthX;
		size_t ClusterWidthY;
		size_t PowerRank;
                size_t ArenaWidth;
                size_t SimTime; 
                Real curr_time_in_minutes; 
                Real last_time_in_minutes; 
  
		/* CPFA variables */
		argos::Real ProbabilityOfSwitchingToSearching;
		argos::Real ProbabilityOfReturningToNest;
		argos::CRadians UninformedSearchVariation;
		argos::Real RateOfInformedSearchDecay;
		argos::Real RateOfSiteFidelity;
		argos::Real RateOfLayingPheromone;
		argos::Real RateOfPheromoneDecay;
		
		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;
		argos::Real CameraRadiusSquared;

		/* list variables for food & pheromones */
		std::vector<argos::CVector2> FoodList;
		std::vector<argos::CColor>   FoodColoringList;
		vector<argos::CVector2> CollectedFoodList;
                map<string, argos::CVector2> FidelityList; 
		std::vector<Pheromone>   PheromoneList; 
		std::vector<argos::CRay3>    TargetRayList;
		map<string, std::vector<CVector2> >  Trajectory;
		
		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;
		map<string, argos::CVector2> robotPosList; //qilu 06/2023
		//vector<argos::CVector2> robotPosList; //qilu 06/2023
		map<string, vector<argos::CVector2>> robotPosList3;

		
                Real   CollisionTime;
                size_t currCollisionTime; 
                size_t lastCollisionTime; 
                size_t lastNumCollectedFood;
                size_t currNumCollectedFood;
                size_t Num_robots;
      
                vector<size_t>		ForageList;
		argos::CVector2 NestPosition;
	private:
		bool SetupPythonEnvironment();
		
		size_t counter_nest; // Current count of robots near the nest
		std::vector<size_t> counter_nest_history; 
		std::vector<size_t> collision_history;
		// vector<int> RunCongestion(std::vector<std::pair<double, double>> dataset);
		vector<int> RunCongestion(const std::vector<argos::CVector2>& robotPosList2);
		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();

		// Add member variables for state information, communication, etc.
		// Function to predict trajectory
		//void PredictTrajectory();
		void PredictTrajectory(std::vector<CVector2>& predicted_positions, const RobotState& state, Real time_horizon, Real time_step);
		// Function to detect potential collisions
		//void DetectCollisions();
    	//void DetectCollisions(const std::vector<std::vector<CVector2>>& predicted_trajectories, std::vector<CollisionInfo>& collisions);
		void DetectCollisions(const std::unordered_map<size_t, std::vector<CVector2>>& predicted_trajectories, std::vector<size_t>& collisions, std::vector<size_t>& intersections);
		//make predicted_trajectories a dictionary 
		//std::unordered_map<size_t, std::vector<CVector2>> predicted_trajectories;

		bool DetectIntersection(const std::vector<CVector2>& trajectory1, const std::vector<CVector2>& trajectory2);

		// Function to adjust path to avoid collisions
		//void AdjustPath();
    	void AdjustPath(std::vector<std::vector<CVector2>>& predicted_trajectories, const std::vector<CollisionInfo>& collisions);

        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		double score;
		int PrintFinalScore;
		PyObject *pyFileName, *pyModule;
		PyObject *pyCongestion,	*pyCallCongestion,	*pyCongestionArgs;

		//std::vector<std::vector<CVector2>> m_predicted_trajectories; // Vector to store predicted trajectories for all robots
		std::unordered_map<size_t, std::vector<CVector2>> m_predicted_trajectories;
    	Real m_collision_threshold; // Distance threshold for detecting collisions
		//map<string, vector<argos::CVector2>> dropped_trajectories;
		std::map<std::string, std::vector<std::vector<argos::CVector2>>> dropped_trajectories;

		std::vector<argos::CVector2> dropped_trajectories2;
};

#endif /* CPFA_LOOP_FUNCTIONS_H */
