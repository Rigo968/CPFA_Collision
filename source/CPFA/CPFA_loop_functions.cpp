#include "CPFA_loop_functions.h"

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
        SimTime(0),
	//MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
    MaxSimTime(0),//qilu 02/05/2021
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FoodItemCount(256),
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	CameraRadiusSquared(2.25),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0)
{}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error
	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
    argos::GetNodeAttribute(settings_node, "NestPosition", NestPosition);
    FoodRadiusSquared = FoodRadius*FoodRadius;
    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }
    
	m_collision_threshold = 0.1f;
	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

        ArenaWidth = ArenaSize[0];
        
       /* if(abs(NestPosition.GetX()) < -1) //quad arena
        {
            NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
        }
        else
        {
            NestRadius *= sqrt(log(ArenaWidth)/log(2));
        } */
        
        //argos::LOG<<"NestRadius="<<NestRadius<<endl;
	   // Send a pointer to this loop functions object to each controller.
	   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        c2.SetLoopFunctions(this);
	    }
     
     
   NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
  
	ForageList.clear(); 
	last_time_in_minutes=0;
	SetupPythonEnvironment();
 
}


void CPFA_loop_functions::Reset() {
	   if(VariableFoodPlacement == 0) {
		      RNG->Reset();
	   }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();
    FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();
    Trajectory.clear();
    
    SetFoodDistribution();
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){
		      
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }
    UpdatePheromoneList();
	//print timestep
	//argos::LOG << "timestep: " << GetSpace().GetSimulationClock() << std::endl;
	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
      for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = argos::CColor::BLACK;
      }
	}
	argos::CVector2 position;
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    
    robotPosList.clear();
    for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
      argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
      position = c2.GetPosition();
      robotPosList[c2.GetId()] = position;
      //robotPosList.push_back(position);
    }
    
    //for(map<string, CVector2>::iterator it= robotPosList.begin(); it!=robotPosList.end(); ++it) {
	//	argos::LOG << "pos["<< it->first <<"]="<< it->second << endl;
	//}
         
    if(FoodList.size() == 0) {
	FidelityList.clear();
	PheromoneList.clear();
        TargetRayList.clear();
        Trajectory.clear();
    }
}

void CPFA_loop_functions::PostStep() {
	// get x,y coordinate of each robot
	argos::CVector2 position;
	vector<argos::CVector2> robotPosList2;
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	//dictionary that holds the robot id as a key and the trajectory of the robot that has dropped a resource up to the last 50 values.
	std::map<std::string, std::vector<argos::CVector2>> dropped_trajectories;

	robotPosList.clear();
	robotPosList2.clear();

	size_t index = 0;
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++, index++) {
	  argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	  BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	  CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
	  position = c2.GetPosition();
	  //robotPosList[c2.GetId()] = position;
	  robotPosList2.push_back(position);

		if(c2.GetStatus() == "DROPPED"){
			dropped_trajectories[c2.GetId()] = robotPosList2[index-50:];
		}
	}
		







	// for(map<string, CVector2>::iterator it= robotPosList.begin(); it!=robotPosList.end(); ++it) {
	// 	argos::LOG << "pos["<< it->first <<"]="<< it->second << endl;
	// }

	// for(size_t i = 0; i < robotPosList2.size(); i++) {
	// 	argos::LOG << "pos["<< i <<"]="<< robotPosList2[i] << endl;
	// }

	//argos::LOG << GetSpace().GetSimulationClock() << std::endl; 	
	// run congestion algorithm
    vector<int> congestionResults = RunCongestion(robotPosList2);
	// argos::LOG << congestionResults.size() << " robots in the arena" << std::endl;
    // // Print the results
    // for (size_t i = 0; i < congestionResults.size(); ++i) {
    //     argos::LOG << "Robot " << congestionResults[i] << " is congested" << std::endl;
    // }
	//argos::LOG << std::endl;
	// vector<argos::CVector2> robotPosList;
	// argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	// for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
	// 	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	// 	BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	// 	CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
	// 	robotPosList.push_back(c2.GetPosition());

	//if a robot is congested, set the robot's status to "CONGESTED"
	// for(size_t i = 0; i < congestionResults.size(); i++) {
	// 	argos::CSpace::TMapPerType::iterator it = footbots.begin();
	// 	if (i == 0) {
	// 		std::advance(it, congestionResults[i]);
	// 	}
	// 	else {
	// 		std::advance(it, congestionResults[i] - congestionResults[i-1]);
	// 	}
	// 	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	// 	BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	// 	CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
	// 	// if (c2.GetStatus() == "SEARCHING") {
	// 	// 	c2.setStatus("CONGESTED");
	// 	// }
	// 	//argos::LOG << i << std::endl;
	// 	string status = c2.GetStatus();
	// 	c2.setStatus("CONGESTED");
	// 	c2.setStatus(status);
	// }	
	// get status the status of each robot and store it in a list

	// size_t index = 0;
	// for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++, index++) {
	// 	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	// 	BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	// 	CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		
	// 	// if the index is in congestionResults, switch status
	// 	if (c2.GetStatus() == "SEARCHING" && std::count(congestionResults.begin(), congestionResults.end(), index) > 0) {
	// 		c2.setStatus("CONGESTED");
	// 	}

	// 	//argos::LOG << "Robot: " << c2.GetId() << ", Status: " << c2.GetStatus() << std::endl;
	// 	// if(c2.GetStatus() == "CONGESTED" && std::find(congestionResults.begin(), congestionResults.end(), index) == congestionResults.end()) {
	// 	// 	c2.setStatus("SEARCHING");
	// 	// }

	// 	if(c2.GetStatus() == "CONGESTED" && std::count(congestionResults.begin(), congestionResults.end(), index) == 0) {
	// 		c2.setStatus("SEARCHING");
	// 	}
	// }


	//get velocity of each robot
	vector<CVector2> robotVelocities;

	size_t index = 0;
	size_t index1 = 0;


    // Add member variables for state information, communication, etc.

	vector<CVector2> headings;
	
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++, index++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		if(c2.GetStatus() == "RETURNING"){
			//headings.push_back(c2.GetHeading());
			RobotState current_state;
			current_state.position = c2.GetPosition(); // Function to get current position
			//get velocity using distance between two positions over delta time
			//current_state.velocity = (current_state.position - c2.GetPosition()) / 0.1; // Function to get current velocity

			current_state.heading = c2.GetHeading();   // Function to get current heading

			// Time parameters
			Real time_horizon = 0.625f;
			Real time_step = 0.03125f;

			// Vector to store predicted positions
			std::vector<CVector2> predicted_positions;
			PredictTrajectory(predicted_positions, current_state, time_horizon, time_step);
			//std::unordered_map<size_t, CVector2> predicted_positions;
			// Predict trajectory
			//PredictTrajectory(c2.GetId(), predicted_positions, current_state, time_horizon, time_step);
			// LOG << "Current position: " << current_state.position << " For Robot: " << c2.GetId() << std::endl;

			std::vector<CVector2> predicted_positions_odd;
			for (size_t i = 0; i < predicted_positions.size(); i++) {
				if (i % 2 == 1) {
					predicted_positions_odd.push_back(predicted_positions[i]);
				}
			}

			// for (const CVector2& pos : predicted_positions_odd) {				
			// 	LOG << "Predicted position: " << pos << " For Robot: " << c2.GetId() << std::endl;
			// }
			
			//m_predicted_trajectories.push_back(predicted_positions_odd);
			// change predicted positions to a dictionary with robot id as key
			m_predicted_trajectories[index] = predicted_positions_odd;
		}
	}

	std::vector<size_t> collisions;
	std::vector<size_t> intersections;
	DetectCollisions(m_predicted_trajectories, collisions, intersections);
	//print contents of collisions vector and intersections vector

	// for (size_t i = 0; i < collisions.size(); i++) {
	// 	LOG << "Collision detected for Robot: " << collisions[i] << std::endl;
	// }
	// for (size_t i = 0; i < intersections.size(); i++) {
	// 	LOG << "Intersection detected for Robot: " << intersections[i] << std::endl;
	// }


	
	//log detected collisions
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++, index1++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		// if (std::find(collisions.begin(), collisions.end(), index1) != collisions.end()) {
		// 	c2.setStatus("CONGESTED");
		// }
		// if(c2.GetStatus() == "CONGESTED" && std::count(collisions.begin(), collisions.end(), index1) == 0) {
		// 	c2.setStatus("RETURNING");
		// }

		// this logic is only if a robot is in the returning, but the algorithm detects it is in one of the two congested states.
		if (c2.GetStatus() == "RETURNING" && std::count(collisions.begin(), collisions.end(), index1) > 0) {
			c2.setStatus("CONGESTED");
			argos::LOG << "Robot " << c2.GetId() << " is at " << c2.GetStatus() << " state." << std::endl;
		}

		// if (c2.GetStatus() == "RETURNING" && std::count(intersections.begin(), intersections.end(), index1) > 0) {
		// 	c2.setStatus("INTERSECTION");
		// 	argos::LOG << "Robot " << c2.GetId() << " is at " << c2.GetStatus() << " state." << std::endl;
		// }

		//print status of each robot
		//print contents of collisions vector

		// for (size_t i = 0; i < collisions.size(); i++) {
		// 	LOG << "Collision detected for Robot: " << collisions[i] << std::endl;
		// }


		//no need for this logic anymore since robots will exit the congested states when they are no longer in the collision or intersection states
		// if(c2.GetStatus() == "CONGESTED" && std::count(collisions.begin(), collisions.end(), index1) == 0) {
		// 	c2.setStatus("RETURNING");
		// 	argos::LOG << "Robot " << c2.GetId() << " is now back to " << c2.GetStatus() << std::endl;
		// }

		// if(c2.GetStatus() == "CONGESTED" && std::count(intersections.begin(), intersections.end(), index1) == 0) {
		// 	c2.setStatus("RETURNING");
		// 	argos::LOG << "Robot " << c2.GetId() << " is now back to " << c2.GetStatus() << std::endl;
		// }		
	}


	// for (size_t i = 0; i < collisions.size(); i++) {
	// 	LOG << "Collision detected between Robot: " << collisions[i].robot1 << " and Robot: " << collisions[i].robot2 << std::endl;
	// }
	// // Adjust paths based on detected collisions
	// AdjustPath(m_predicted_trajectories, collisions);
	size_t index = 0;
	size_t index1 = 0;
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++, index++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		if(c2.GetStatus() == "RETURNING"){
			argos::CVector2 position = c2.GetPosition();

			//append the position to a dictionary with robot id as key and the position as value
			robotPosList[c2.GetId()] = position;
		}

	}
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
		isFinished = true;
	}
    //set to collected 88% food and then stop
    if(score >= NumDistributedFood){
		isFinished = true;
		}
         
         
    
	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;
        argos::LOG<< "time out..."<<endl; 
		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
    //  printf("%f, %f, %lu\n", score, getSimTimeInSeconds(), RandomSeed);
    //  printf("%f\n", score);  
                  
    if (PrintFinalScore == 1) {
        string type="";
        if (FoodDistribution == 0) type = "random";
        else if (FoodDistribution == 1) type = "cluster";
        else type = "powerlaw";
            
        ostringstream num_tag;
        num_tag << FoodItemCount; 
              
        ostringstream num_robots;
        num_robots <<  Num_robots;
   
        ostringstream arena_width;
        arena_width << ArenaWidth;
        
        ostringstream quardArena;
        if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
             quardArena << 1;
         }
         else{
             quardArena << 0;
        }
        
        string header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";
       
        unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
        /* Real total_travel_time=0;
        Real total_search_time=0;
        ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
        */
        
        
        argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
        for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
            argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
            BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
            CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
            CollisionTime += c2.GetCollisionTime();
            
            /*if(c2.GetStatus() == "SEARCHING"){
                total_search_time += SimTime-c2.GetTravelingTime();
                total_travel_time += c2.GetTravelingTime();
	    }
            else {
		total_search_time += c2.GetSearchingTime();
		total_travel_time += SimTime-c2.GetSearchingTime();
            } */        
        }
        //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
        //travelSearchTimeDataOutput.close();   
             
        ofstream dataOutput( (header+ "iAntTagDa.txt").c_str(), ios::app);
        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
        }
    
        //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
        //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput.close();

		/*
        ofstream densityOutput( ("./results/densities.txt"), ios::app);
        densityOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        densityOutput.close();
		*/
        ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
        if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
        for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
        forageDataOutput<<"\n";
        forageDataOutput.close();
        
        ofstream trajOutput( (header+ "iAntTrajData.txt").c_str(), ios::app);
        // output to file
        //if(trajOutput.tellp() == 0) {
            trajOutput << "trajs\n";//qilu 11/2023
        //}
        
        for(map<string, std::vector<CVector2>>::iterator it= Trajectory.begin(); it!= Trajectory.end(); ++it) {
			
			for(size_t j = 0; j < it->second.size(); j++) {
				trajOutput << it->second[j]<<"; ";
			}
			trajOutput << "\n";
		
		}
        
		trajOutput.close();
        
      }  

	// get food collected for each robot at each timestep
	ofstream foodOutput( "./results/foodData.txt", ios::app);
	if(foodOutput.tellp() == 0) {
	   foodOutput << "food_collected\n";
	   //foodOutput << CollectedFoodList.size() << endl;
	}
	for(size_t i = 0; i < CollectedFoodList.size(); i++) {
	   foodOutput << CollectedFoodList[i] << ", ";
	}
	foodOutput << endl;
	foodOutput.close();


}


argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();
	    for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
      }
     	PheromoneList = new_p_list;
	new_p_list.clear();
}
void CPFA_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void CPFA_loop_functions::RandomFoodDistribution() {
	FoodList.clear();
        FoodColoringList.clear();
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}


void CPFA_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
    FoodColoringList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	FoodItemCount = foodPlaced;
}
 
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

      return ( (p - NestPosition).SquareLength() < NRPB_squared) ;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
	if (score >= NumDistributedFood) {
		PostExperiment();
	}
}

double CPFA_loop_functions::Score() {	
	return score;
}

void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

bool CPFA_loop_functions::SetupPythonEnvironment(){

	Py_Initialize();
	if(Py_IsInitialized()){
		LOG << "Python version: " << Py_GetVersion() << endl;
	} else {
		LOGERR << "ERROR: Python failed to initialize." << endl;
		return 0;	
	}

	
	PyObject *sys = PyImport_ImportModule("sys");
	PyObject *path = PyObject_GetAttrString(sys, "path");
	PyList_Append(path, PyUnicode_FromString("/home/arturo/src/argos3/build_simulator/Collision_Free_CPFA/source/CPFA"));
	PyObject *repr = PyObject_Repr(path);
	const char* s = PyUnicode_AsUTF8(repr);
	printf("Python path: ");
	Py_DECREF(repr);
	Py_DECREF(path);
	Py_DECREF(sys);

	// Load the module
	pyFileName = PyUnicode_FromString("congestion");
	if (pyFileName == NULL) {
		LOG << "Error converting module name to PyUnicode" << std::endl;
		Py_Finalize();
		return 0;
	}

	pyModule = PyImport_Import(pyFileName);
	Py_DECREF(pyFileName);

	if (pyModule == NULL) {
		LOG << "Failed to load Python module" << std::endl;
		Py_Finalize();
		return 0;
	}

	// Load the function from the module
	pyCongestion = PyObject_GetAttrString(pyModule, "run_congestion_logic");
	Py_DECREF(pyModule);

	if (pyCongestion == NULL || !PyCallable_Check(pyCongestion)) {
		if (PyErr_Occurred()) {
			PyErr_Print();
		}
		LOG << "Failed to load Python function" << std::endl;
		Py_XDECREF(pyCongestion);
		Py_Finalize();
		return 0;
	}
	// PyObject *result = PyObject_CallObject(pyCongestion, NULL);
    // PyObject* str = PyObject_Repr(result);
    // const char* c_str = PyUnicode_AsUTF8(str);
    // printf("Python function returned: %s\n", c_str);
    // Py_XDECREF(str);
	return 1;

}


vector<int> CPFA_loop_functions::RunCongestion(const std::vector<argos::CVector2>& robotPosList2) {
    // if (pyCongestion == NULL) {
    //     LOGERR << "Python function not loaded" << std::endl;
    //     Py_XDECREF(pyCongestion);
    //     Py_Finalize();
    //     return {};
    // }

    // Create a Python list from robotPosList2
    PyObject *pyData = PyList_New(robotPosList2.size());
    for (size_t i = 0; i < robotPosList2.size(); i++) {
        PyObject *pyPair = PyTuple_New(2);
        PyTuple_SetItem(pyPair, 0, PyFloat_FromDouble(robotPosList2[i].GetX()));
        PyTuple_SetItem(pyPair, 1, PyFloat_FromDouble(robotPosList2[i].GetY()));
        PyList_SetItem(pyData, i, pyPair);
    }

    PyObject *args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyData);
    PyObject *pyResult = PyObject_CallObject(pyCongestion, args);
    Py_DECREF(args);

    // if (pyResult == NULL) {
    //     LOGERR << "Failed to call Python function" << std::endl;
    //     PyErr_Print();
    //     Py_XDECREF(pyResult);
    //     Py_XDECREF(pyCongestion);
    //     Py_Finalize();
    //     return {};
    // }

    // if (!PyList_Check(pyResult)) {
    //     LOGERR << "Python function did not return a list" << std::endl;
    //     Py_XDECREF(pyResult);
    //     Py_XDECREF(pyCongestion);
    //     Py_Finalize();
    //     return {};
    // }

    vector<int> result;
    // for (size_t i = 0; i < PyList_Size(pyResult); i++) {
    //     PyObject *item = PyList_GetItem(pyResult, i);
    //     if (!PyLong_Check(item)) {
    //         LOGERR << "Python list item is not an integer" << std::endl;
    //         Py_XDECREF(pyResult);
    //         Py_XDECREF(pyCongestion);
    //         Py_Finalize();
    //         return {};
    //     }
    //     result.push_back(PyLong_AsLong(item));
    // }

    if (PyList_Check(pyResult)) {
        Py_ssize_t listSize = PyList_Size(pyResult);
		//LOGERR << "Result is valid" << std::endl;
        for (Py_ssize_t i = 0; i < listSize; ++i) {
            PyObject* pLabel = PyList_GetItem(pyResult, i);
            int label = PyLong_AsLong(pLabel);
            result.push_back(label);
        }
    }

    Py_DECREF(pyResult);
    //Py_XDECREF(pyResult);
    return result;
}

void CPFA_loop_functions::PredictTrajectory(std::vector<CVector2>& predicted_positions, const RobotState& state, Real time_horizon, Real time_step) {
    predicted_positions.clear();

    CVector2 current_position = state.position;
    //CVector2 current_velocity = state.velocity;
    argos::CRadians current_heading = state.heading;
	Real velocity = 0.08f;
	Real heading_angle = current_heading.UnsignedNormalize().GetValue();
    for (Real t = 0; t <= time_horizon; t += time_step) {
            CVector2 displacement = CVector2(velocity * std::cos(heading_angle) * time_step,
                                             velocity * std::sin(heading_angle) * time_step);

            CVector2 new_position = current_position + displacement;
            // Update current state for the next iteration
			predicted_positions.push_back(new_position);
            current_position = new_position;
    }
}

// void CPFA_loop_functions::PredictTrajectory(std::unordered_map<size_t, CVector2>& predicted_positions, const RobotState& state, Real time_horizon, Real time_step, size_t robot_id) {
//     predicted_positions.clear();

//     CVector2 current_position = state.position;
//     argos::CRadians current_heading = state.heading;
//     Real velocity = 0.08f;
//     Real heading_angle = current_heading.UnsignedNormalize().GetValue();
//     for (Real t = 0; t <= time_horizon; t += time_step) {
//         CVector2 displacement = CVector2(velocity * std::cos(heading_angle) * time_step,
//                                          velocity * std::sin(heading_angle) * time_step);

//         CVector2 new_position = current_position + displacement;
//         // Update current state for the next iteration
//         predicted_positions[robot_id] = new_position;
//         current_position = new_position;
//     }
// }

// void CPFA_loop_functions::DetectCollisions(const std::vector<std::vector<CVector2>>& predicted_trajectories, std::vector<size_t>& collisions) {
//     // predicted_trajectories contains the predicted positions of all robots(10 for each)
// 	for (size_t i = 0; i < predicted_trajectories.size(); ++i) {
// 		for (size_t j = i + 1; j < predicted_trajectories.size(); ++j) {
// 			const std::vector<CVector2>& trajectory1 = predicted_trajectories[i];
// 			const std::vector<CVector2>& trajectory2 = predicted_trajectories[j];

// 			// Check if the two trajectories intersect
// 			for (size_t t = 0; t < trajectory1.size(); ++t) {
// 				if (t < trajectory2.size()) {
// 					if ((trajectory1[t] - trajectory2[t]).SquareLength() < 0.1) {
// 						//collisions.push_back(CollisionInfo(i, j, t));
// 						collisions.push_back(i);
// 						break;
// 					}
// 				}
// 			}
// 		}
// 	}
// }

// void CPFA_loop_functions::DetectCollisions(const std::unordered_map<size_t, std::vector<CVector2>>& predicted_trajectories, std::vector<size_t>& collisions, std::vector<size_t>& intersections) {
//     std::vector<size_t> collisionstemp;
//     std::unordered_map<size_t, size_t> collision_count;

//     // Get keys from predicted trajectories
//     std::vector<size_t> keys;
//     for (auto const& element : predicted_trajectories) {
//         keys.push_back(element.first);
//     }

//     // Loop through predicted trajectories and compare each robot with all other robots
//     for (size_t i = 0; i < keys.size(); ++i) {
//         bool collision_found = false;

//         if(std::find(collisionstemp.begin(), collisionstemp.end(), keys[i]) != collisionstemp.end()){
//             continue;
//         }

//         for (size_t j = i + 1; j < keys.size() && !collision_found; ++j) {
//             const std::vector<CVector2>& trajectory1 = predicted_trajectories.at(keys[i]);
//             const std::vector<CVector2>& trajectory2 = predicted_trajectories.at(keys[j]);

//             // Check if the two trajectories coincide
//             for (size_t t = 0; t < trajectory1.size(); ++t) {
//                 if (t < trajectory2.size()) {
//                     if ((trajectory1[t] - trajectory2[t]).SquareLength() < 0.05) {
// 						// this is an error because I am adding the collision_count wrong. for example, if i have 3 robot and compare robot 1 with robot 2 and 3, 
// 						//this will add both collision_count of robot 2 and 3 to robot 1 where as I want to keep seperate records for each pair.
//                         collision_count[keys[i]]++;                        
// 						// if enters this loop then it is a conincident path
//                         if(collision_count[keys[i]] > 1){
//                             collisions.push_back(keys[i]);
// 							collisionstemp.push_back(keys[j]);
//                             collision_found = true;
//                             break;
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     // Append keys with only one collision to the intersections vector
//     for (auto const& element : collision_count) {
//         if (element.second == 1) {
//             intersections.push_back(element.first);
//         }
//     }
// }

void CPFA_loop_functions::DetectCollisions(const std::unordered_map<size_t, std::vector<CVector2>>& predicted_trajectories, std::vector<size_t>& collisions, std::vector<size_t>& intersections) {
    std::unordered_map<size_t, std::unordered_map<size_t, size_t>> collision_count;
    std::vector<size_t> collisionstemp;

    // Get keys from predicted trajectories
    std::vector<size_t> keys;
    for (auto const& element : predicted_trajectories) {
        keys.push_back(element.first);
    }

    // Loop through predicted trajectories and compare each robot with all other robots
    for (size_t i = 0; i < keys.size(); ++i) {
        bool collision_found = false;

        if (std::find(collisionstemp.begin(), collisionstemp.end(), keys[i]) != collisionstemp.end()) {
            continue;
        }

        for (size_t j = i + 1; j < keys.size() && !collision_found; ++j) {
            const std::vector<CVector2>& trajectory1 = predicted_trajectories.at(keys[i]);
            const std::vector<CVector2>& trajectory2 = predicted_trajectories.at(keys[j]);

            // Iterate through all points in the trajectories
            for (size_t t = 0; t < trajectory1.size(); ++t) {
                if (t < trajectory2.size()) {
                    if ((trajectory1[t] - trajectory2[t]).SquareLength() < 0.05) {
                        // Increase the collision count for this specific pair of robots
                        collision_count[keys[i]][keys[j]]++;
                        collision_count[keys[j]][keys[i]]++; // Symmetric increment
                    }
                }
            }

            // After checking all points, determine if there is a coincident path
            if (collision_count[keys[i]][keys[j]] > 3) {
                collisions.push_back(keys[i]);
                collisionstemp.push_back(keys[j]);
                collision_found = true;
            }

            if (collision_count[keys[i]][keys[j]] == 1) {
                intersections.push_back(keys[i]);
                collisionstemp.push_back(keys[j]);
                collision_found = true;
            }
        }
    }
}


REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
