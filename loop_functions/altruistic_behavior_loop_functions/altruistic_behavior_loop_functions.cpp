#include "altruistic_behavior_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>
#include <sstream>
#include <stack>
#include <string>     // std::string, std::to_string
#include <numeric>
#include <controllers/altruistic_behavior/altruistic_behavior.h>
/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>
#define GetCurrentDir getcwd

/****************************************/
/****************************************/
CAltBehLoopFunctions::CAltBehLoopFunctions() :
	m_pcFloor(NULL),
	m_pcRNG(NULL),
	G(10,std::vector<int>(10)),
	AG(10,std::vector<int>(10)),
	nodes(10),
	degrees(10),
	m_unDegree(5),
	m_bShowLinks(false),
	m_fProbability(0.5),
	m_strNetwork("local"),
	m_seed(1),
	b_dynamic_env(false),
	item_discovery_probability(0.05920716)
{
}

/****************************************/
/****************************************/

void CAltBehLoopFunctions::Init(TConfigurationNode& t_node) {
	try {
		m_pcRNG = CRandom::CreateRNG("argos");
		/* Read in different parameters from the XML and initialize them */
		m_pcFloor = &GetSpace().GetFloorEntity();
		TConfigurationNode& tEnergy = GetNode(t_node, "energy");

		GetNodeAttribute(tEnergy, "items", unEnergyItems);
		
		// ----- DYNAMIC? -----
		// ---------------------------------------
		GetNodeAttribute(tEnergy, "dynamic_env", b_dynamic_env);
		GetNodeAttribute(tEnergy, "dynamic_env_type", s_dynamic_env_type);
		// ---------------------------------------
		
		// ----- NETWORK-SPECIFIC PROPERTIES -----
		// ---------------------------------------
		GetNodeAttribute(tEnergy, "network_type", m_strNetwork);
		GetNodeAttribute(tEnergy, "degree", m_unDegree);
		GetNodeAttribute(tEnergy, "probability", m_fProbability);
		GetNodeAttribute(tEnergy, "seed", m_seed);
		/* Show network links? */
		GetNodeAttribute(tEnergy, "show_links", m_bShowLinks);
		
		GenerateGraph(m_strNetwork);
		// ---------------------------------------
		
		numberOfFootbots = 10;
		/* Get the number of energy items we want to be scattered from XML */
		GetNodeAttribute(tEnergy, "radius", m_fEnergySquareRadius);
		/* Distribute uniformly the items in the environment */
		//GREEN SPOTS
		for (UInt32 i = 0; i < unEnergyItems; ++i) {
			m_cEnergyPos.push_back(
			    CVector2(m_pcRNG->Uniform(CRange<Real>(-arenaX + 0.1, arenaX - 0.1)),
			             m_pcRNG->Uniform(CRange<Real>(-arenaY + 0.1, arenaY - 0.1))));
		}

		GetNodeAttribute(tEnergy, "static_to_dynamic", staticToDynamic);
		
		// ----- OUTPUT-SPECIFIC PROPERTIES -----------------------------------------
		// --------------------------------------------------------------------------
		std::string dirID = "/media/sf_Shared_VM_Folder/";
		
		GetNodeAttribute(tEnergy, "directory", dirID);
		GetNodeAttribute(tEnergy, "output1", m_strOutput1);
		GetNodeAttribute(tEnergy, "output2", m_strOutput2);
		GetNodeAttribute(tEnergy, "output3", m_strOutput3);
		
		std::string filename = dirID + m_strOutput1.c_str();
		//~ m_cOutput1.open(filename, std::ios_base::trunc | std::ios_base::out);
		
		filename = dirID + m_strOutput2;
		//~ m_cOutput2.open(filename, std::ios_base::trunc | std::ios_base::out);
		
		filename = dirID + "initial_distribution.txt";
		
		m_strOutputTEMP.open(filename, std::ios_base::trunc | std::ios_base::out);
		m_strOutputTEMP<<"ID\tENERGY"<<std::endl;
		
		filename = dirID + m_strNetwork + "_" + std::to_string(m_unDegree) + "_" + std::to_string(m_seed) + ".csv";
		//~ filename = dirID + "degree_distribution.csv";
		
		m_cOutput3.open(filename, std::ios_base::trunc | std::ios_base::out);
		
		filename = dirID + m_strNetwork + "_" + std::to_string(m_unDegree) + "_" + std::to_string(m_seed) + "_pipuck.csv";
		m_cOutput4.open(filename, std::ios_base::trunc | std::ios_base::out);
		// --------------------------------------------------------------------------
	}
	catch (CARGoSException& ex) {
		THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
	}
	
	CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
	//Go through them	
	int prefcounter = 0;
	for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		/* Create a pointer to the current foot-bot */
		CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(pcFB->GetControllableEntity().GetController());
		int startEnergy = (int)m_pcRNG->Gaussian(1, 100);
		
		cController.SetEnergy(startEnergy);
		m_strOutputTEMP << cController.IntegerId() << "\t" << cController.GetEnergy() << std::endl;
		
		int id = cController.IntegerId();
		
		// Create a pointer to the current rab-equipped entity and set the communication range
		CRABEquippedEntity&  cRAB = pcFB->GetRABEquippedEntity();
		cRAB.SetRange(1.5);
	}

	m_strOutputTEMP.close();
}


/****************************************/
/****************************************/

void CAltBehLoopFunctions::GenerateGraph(std::string m_strNetworkInput) {
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	int V = m_cFootbots.size();
	G.resize(V);
	AG.resize(V);
	for(int i = 0; i < V; i++) { G[i].resize(0); AG[i].resize(0); }
		
	if(m_strNetworkInput=="scale-free"){
		nodes.resize(0);
		degrees.resize(V);
		for(int i = 0; i < V; i++) { degrees[i] = 0; }
		CRange<int> ProbRangeInt(0, V);
		CRange<Real> ProbRangeReal(0.0, 1.0);
		int node = m_pcRNG->Uniform(ProbRangeInt);
		
		int initNumberOfEdges = 10;
		int edgesToAdd = m_unDegree;
		int totNumEdges = 0;
		
		/**
		 *  Generate a network according to the BarabasiAlbertModel.
		 *
		 *  An intial complete network is composed of (init_num_nodes) many nodes.  At each time step afterwards
		 *  a single node is added to the network, until (num)nodes) many nodes have been added.  When a node is added,
		 *  it is given (edgeToAdd)-many initial edges, the endpoint of each edge is chosen using preferential attachment.
		 *  The probability of an existing node <i> u </i> being selected is;  degree(<i>u</i>)/ 2*|E|, where |E| is the 
		 *  current number of edges in the network, not what it will be when all edges have been added.  In this way 
		 *  nodes with more edges get more edges and nodes with few edges, remain with low degree.
		 *
		 */
		 
		 /*======== First, construct a complete network with initNumberOfEdges ========*/
		 /*================================================================================*/
		 int x = m_pcRNG->Uniform(ProbRangeInt);	// RANDOM PICK
		 //int x = 0;									// ITERATIVE PICK
		 nodes.push_back(x);
		 int vertices = 1;
		 
		 // Pick 10 nodes randomly from a uniform distribution 
		 while(vertices < initNumberOfEdges){
			x = m_pcRNG->Uniform(ProbRangeInt);		// RANDOM PICK
			//x++;									// ITERATIVE PICK
			if(std::find(nodes.begin(), nodes.end(), x) != nodes.end()) {
				// nodes contains x, keep searching 
			} else {
				// add x to G[node] 
				nodes.push_back(x);
				vertices++;
			}
		 }
		 
		 // Build the complete network 
		 for(int i = 0; i < initNumberOfEdges; i++){
			 for(int j = 0; j < initNumberOfEdges; j++){
				if(i!=j){
					AG[nodes[i]].push_back(nodes[j]);AG[nodes[j]].push_back(nodes[i]);
				}
			 }
			 degrees[nodes[i]] += initNumberOfEdges;
		 }
	     totNumEdges += initNumberOfEdges*(initNumberOfEdges-1.0)*1.0/2.0;
		 /*================================================================================*/
		 /*================================================================================*/
		 
		 /*=================== Second, at each time step add a new node with edgesToAdd new edges ===================*/
		 /*======== The connections to the existing nodes are established according to preferential attachment ========*/
		 /*============================================================================================================*/
		 int added = 0;
		 double prob = 0.0, randNum = 0.0;
			
		 
		 for(int i = vertices; i < V; i++){
			 
			added = 0;
			//Add the appropriate number of edges
			while(std::find(nodes.begin(), nodes.end(), x) != nodes.end()) {
				x = m_pcRNG->Uniform(ProbRangeInt);			// RANDOM PICK
			}
			//x = i;											// ITERATIVE PICK
			nodes.push_back(x);
			
			for (int m = 0; m < edgesToAdd; m++) 
			{
				//keep a running talley of the probability
				prob = 0.0;
				//Choose a random number
				randNum = m_pcRNG->Uniform(ProbRangeReal);
				
				//Try to add this node to every existing node
				for (int j = 0; j < i; j++) 
				{
					//Check for an existing connection between these two nodes
					if(std::find(AG[nodes[j]].begin(), AG[nodes[j]].end(), x) != AG[nodes[j]].end()) {
						// nodes j and x already have a connection 
					} else {
						//Increment the talley by the jth node's probability
						prob += (double) ((double) degrees[nodes[j]]) / ((double) (2.0 * totNumEdges) );
					}

					//If this pushes us past the the probability
					if (randNum <= prob) 
					{
						// Create and edge between node x and node j
						AG[nodes[j]].push_back(x);
						AG[x].push_back(nodes[j]);

						//increment the number of edges
						added++;
						//increment the degrees of each node
						degrees[x]++;
						degrees[nodes[j]]++;
						
						//Stop iterating for this probability, once we have found a single edge
						break;
					}
				}
			}
			totNumEdges += added;
		 }
		 /*============================================================================================================*/
		 /*============================================================================================================*/
		 //~ for(int i = 0; i < degrees.size(); i++){
			//~ m_cOutput << degrees[i] << "\n";
		 //~ }
	}
	
	else if(m_strNetworkInput=="random"){
		CRange<int> ProbRangeInt(0, V);
		int j = 0;
		 
		 for(int i = 0; i < V; i++){
			 
			for (int m = 0; m < m_unDegree; m++) 
			{
				
				j = m_pcRNG->Uniform(ProbRangeInt); 
				
				while(j==i){
					j = m_pcRNG->Uniform(ProbRangeInt); 
					if(std::find(AG[i].begin(), AG[i].end(), j) == AG[i].end()){
						//LOG << AG[i].size() << std::endl;
					}
					else{
						j=i;
					}
				}
				
				AG[j].push_back(i);
				AG[i].push_back(j);
				
			}
			
		 }
		 
	}
    
    else if(m_strNetworkInput=="complete"){
	/**
     * Returns the complete graph on {@code V} vertices.
     * @param V the number of vertices
     * @return the complete graph on {@code V} vertices
   
	*/
		for (int v = 0; v < V; v++){
            for (int w = v+1; w < V; w++){
                AG[v].push_back(w); 
                AG[w].push_back(v); 
            }
         }   
	}
	else if(m_strNetworkInput=="small-world"){
		/**
	 *  Generates random networks according to the Watts-Strogatz Model. 
	 * <br>
	 *  The algorithm works in two phases:<br>
	 *  (1) Create a regular lattice, where each node is connected to its (degree)-many nearest neighbors.
	 *  (2) Perturb each edge created in step (1) with probability Beta.  Notices that we do not add any edges in this step.
	 * 
	 * @return The generated random network
	 */
		Real p = m_fProbability;
        if (p < 0.0 || p > 1.0){ THROW_ARGOSEXCEPTION("Probability for node connection must be between 0 and 1 to generate a proper interaction network."); }
        
        for (int v = 0; v < V; v++){
            for (int w = v+1; w < V; w++){
                if (m_pcRNG->Bernoulli(p)){ AG[v].push_back(w); AG[w].push_back(v); }
            }
         } 
	}
	else if(m_strNetworkInput=="regular"){
		/**
     * Returns a uniformly random {@code k}-regular graph on {@code V} vertices
     * (not necessarily simple). The graph is simple with probability only about e^(-k^2/4),
     * which is tiny when k = 14.
     *
     * @param V the number of vertices in the graph
     * @param k degree of each vertex
     * @return a uniformly random {@code k}-regular graph on {@code V} vertices.
     */
		int k = m_unDegree;
        if (V*k % 2 != 0) {THROW_ARGOSEXCEPTION("Number of nodes * degree must be even!");}
        
        // create k copies of each vertex
        std::vector<int> vertices;
        vertices.resize(V*k);
        
        for (int v = 0; v < V; v++) {
            for (int j = 0; j < k; j++) {
                vertices[v + V*j] = v;
            }
        }
		
        // pick a random perfect matching
        /*
        std::random_shuffle(vertices.begin(), vertices.end());
        for (int i = 0; i < V*k/2; i++) {
            AG[vertices[2*i]].push_back(vertices[2*i + 1]);
            AG[vertices[2*i + 1]].push_back(vertices[2*i]);
        }
        */
        int newNode;
        
        for (int i = 0; i < V; i++) {
            for (int j = i+1; j < i+k+1; j++) {
				newNode = j%V;
				AG[i].push_back(newNode);
			}
        }
        
	}
	else if(m_strNetworkInput=="watts-strogatz"){
		/**
     * Returns a uniformly random {@code k}-regular graph on {@code V} vertices
     * (not necessarily simple). The graph is simple with probability only about e^(-k^2/4),
     * which is tiny when k = 14.
     *
     * @param V the number of vertices in the graph
     * @param k degree of each vertex
     * @return a uniformly random {@code k}-regular graph on {@code V} vertices.
     */
		int k = m_unDegree;
        if (V*k % 2 != 0) {THROW_ARGOSEXCEPTION("Number of nodes * degree must be even!");}
		CRange<Real> ProbRangeReal(0.0, 1.0);
		CRange<int> ProbRangeInt(0, V);
		Real p = m_fProbability;
        
        // create k copies of each vertex
        std::vector<int> vertices;
        vertices.resize(V*k);
        
        for (int v = 0; v < V; v++) {
            for (int j = 0; j < k; j++) {
                vertices[v + V*j] = v;
            }
        }
        
        int newNode;
        
        for (int i = 0; i < V; i++) {
            for (int j = i+1; j < i+k+1; j++) {
				newNode = j%V;
				AG[i].push_back(newNode);
				AG[newNode].push_back(i);
			}
        }
		
		//For each edge
		for(int v = 0; v < V; v++){
			for(int w = 0; w < AG[v].size(); w++){
				int source = v;
				int target = AG[v][w];
				//Throw a random dart
				Real percent = m_pcRNG->Uniform(ProbRangeReal);
			
				//If this should be shuffled
				if(percent <= p)
				{
					//Choose a new node 
					k = 0;
					bool candidate = false;
			 
					//Keep looping until we have a suitable candidate
					while(!candidate){
						//If source is actually connected to all other nodes than we should stop trying
						//there is no way to switch it (no node that we are not connected to).
						if(AG[source].size() == V-1){
							candidate = false;
							break;
						}
						
						//choose a new edge
						k = m_pcRNG->Uniform(ProbRangeInt);
						//reset variable for this pass
						candidate = true;
						
						//Make sure to avoid a self loop
						if(k == source || k == target){ 
							candidate = false; 
						}
						//Check to see if this edge already exists
						else if(std::find(AG[source].begin(), AG[source].end(), k) != AG[source].end()) {
							// nodes source and k already have a connection 
							candidate = false;
						} 
						else { 
							AG[source].erase(std::remove(AG[source].begin(), AG[source].end(), target), AG[source].end());
							AG[target].erase(std::remove(AG[target].begin(), AG[target].end(), source), AG[target].end());
							AG[source].push_back(k);
							AG[k].push_back(source);
						}
					}
				}
			}
		}
	}
	else if(m_strNetworkInput=="triadic_238"){
		try{
			std::string fileName = "/groups/wall2-ilabt-iminds-be/pl-compas/exp/code/collective_learning/TRGs/238/Id238mFind_T343_" + std::to_string(m_seed) + ".txt";
			std::ifstream infile(fileName);
		   if(!infile.good()){
			THROW_ARGOSEXCEPTION("No file " << fileName << " found!");
		   }
				
			std::string line;
			while (std::getline(infile, line))
			{			
				std::string delimiter = "\t";

				size_t pos = 0;
				std::vector<std::string> str_segs;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					
					str_segs.push_back( line.substr(0, pos) );
					line.erase(0, pos + delimiter.length());
					
				}
				
				int segs0, segs1;
				segs0 = std::stoi (str_segs[0]) - 1;
				segs1 = std::stoi (str_segs[1]) - 1;
				
				AG[segs0].push_back(segs1);
				
			}
			
			infile.close();
		}
		catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error: File not found!", ex);
		}
	
	}
	else if(m_strNetworkInput=="triadic_98"){
		try{
			std::string fileName = "/groups/wall2-ilabt-iminds-be/pl-compas/exp/code/collective_learning/TRGs/98/Id98mFind_T686_" + std::to_string(m_seed) + ".txt";
			std::ifstream infile(fileName);
		   if(!infile.good()){
			THROW_ARGOSEXCEPTION("No file " << fileName << " found!");
		   }
				
			std::string line;
			while (std::getline(infile, line))
			{
				
				std::string delimiter = "\t";

				size_t pos = 0;
				std::vector<std::string> str_segs;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					
					str_segs.push_back( line.substr(0, pos) );
					line.erase(0, pos + delimiter.length());
					
				}
				
				int segs0, segs1;
				segs0 = std::stoi (str_segs[0]) - 1;
				segs1 = std::stoi (str_segs[1]) - 1;
				
				AG[segs0].push_back(segs1);
				
			}
			
			infile.close();
		}
		catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error: File not found!", ex);
		}
	}
	else if(m_strNetworkInput=="triadic_38"){
		try{
			std::string fileName = "/groups/wall2-ilabt-iminds-be/pl-compas/exp/code/collective_learning/TRGs/38/Id38mFind_T686_" + std::to_string(m_seed) + ".txt";
			std::ifstream infile(fileName);
		   if(!infile.good()){
			THROW_ARGOSEXCEPTION("No file " << fileName << " found!");
		   }
				
			std::string line;
			while (std::getline(infile, line))
			{			
				std::string delimiter = "\t";

				size_t pos = 0;
				std::vector<std::string> str_segs;
				while ((pos = line.find(delimiter)) != std::string::npos) {
					
					str_segs.push_back( line.substr(0, pos) );
					line.erase(0, pos + delimiter.length());
					
				}
				
				int segs0, segs1;
				segs0 = std::stoi (str_segs[0]) - 1;
				segs1 = std::stoi (str_segs[1]) - 1;
				
				AG[segs0].push_back(segs1);
				
			}
			
			infile.close();
		}
		catch (CARGoSException& ex) {
			THROW_ARGOSEXCEPTION_NESTED("Error: File not found!", ex);
		}
	}
	else if(m_strNetworkInput=="none" || m_strNetworkInput=="local" || m_strNetworkInput=="regularFromLocal"){
	}
    else{
	 THROW_ARGOSEXCEPTION("Unknown network topology \"" << m_strNetworkInput << "\".");
    }
}

/****************************************/
/****************************************/

void CAltBehLoopFunctions::Reset() {
	//~ m_cOutput1.close();
	//~ m_cOutput2.close();
	m_cOutput3.close();
	m_cOutput4.close();
}

/****************************************/
/****************************************/

void CAltBehLoopFunctions::Destroy() {
	Reset();
}

/****************************************/
/****************************************/

CColor CAltBehLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
	for (UInt32 i = 0; i < m_cEnergyPos.size() ; ++i) {
		if ((c_position_on_plane - m_cEnergyPos[i]).SquareLength() < m_fEnergySquareRadius) {
			return CColor::GRAY50;
		}
	}

	return CColor::WHITE;
}

void CAltBehLoopFunctions::PreStep() {
	/****************************************/
	/************* Graph Generation Code *************/
	/****************************************/
	
	int currTime = GetSpace().GetSimulationClock();
	CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	
	int newNode;
	std::vector<int> energies;
	std::vector<Real> attitudes;
	std::vector<int> newAgentsToShareWith;
	energies.resize(m_cFootbots.size());
	attitudes.resize(m_cFootbots.size());
	newAgentsToShareWith.resize(m_cFootbots.size());
	
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		
		// First clear the old list of network connections
		G[unID].clear();
		
		// Create a pointer to the current foot-bot
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		
		// Clear the list of positions
		m_tNeighbors[pcFB].clear();
		
		energies[unID] = cController.GetEnergy();
		attitudes[unID] = cController.GetAttitude();
		newAgentsToShareWith[unID] = cController.GetAgentToShareWith();
	}
			
	
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		
		// Copy the list of network connections from the data of the range-and-bearing sensor into G
		// Add local links from range-and-bearing sensors		
		if(m_strNetwork=="local"){	
			CCI_RangeAndBearingSensor::TReadings tPackets = cController.GetRealTPackets();
			for(size_t i = 0; i < tPackets.size(); ++i) {
					newNode = tPackets[i].Data[0]*255 + tPackets[i].Data[1];
					if(std::find(G[unID].begin(), G[unID].end(), newNode) == G[unID].end() 
					&& std::find(G[newNode].begin(), G[newNode].end(), unID) == G[newNode].end()
					){
						G[unID].push_back(newNode);
						G[newNode].push_back(unID);
					}
				}
		}
		else if(m_strNetwork=="triadic_38" || m_strNetwork=="triadic_98" || m_strNetwork=="triadic_238"){
		// Add links from artificially generated communication graph
			for(int AGElem = 0; AGElem < AG[unID].size(); AGElem++){ 
				newNode = AG[unID][AGElem];
				if(std::find(G[unID].begin(), G[unID].end(), newNode) == G[unID].end()){
					G[unID].push_back(newNode);
				}
			}
		}
		else{
		// Add links from artificially generated communication graph
			for(int AGElem = 0; AGElem < AG[unID].size(); AGElem++){ 
				newNode = AG[unID][AGElem];
				if(std::find(G[unID].begin(), G[unID].end(), newNode) == G[unID].end() 
				&& std::find(G[newNode].begin(), G[newNode].end(), unID) == G[newNode].end()
				){
					G[unID].push_back(newNode);
					G[newNode].push_back(unID);
				}
			}
		}
	}
		
	for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
		   it != m_cFootbots.end();
		   ++it) {
		  // Create a pointer to the current foot-bot 
		  CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
		  CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(cFootBot.GetControllableEntity().GetController());
        CFootBotDiffusion::SharingData& sSharingData = cController.GetSharingData();
		  std:: string strID = cController.GetId().substr (2,5);
		  int unID = std::stoi (strID,nullptr,10);
		  
		// --- UPDATE INFORMATION RECEIVED FROM NEIGHBORS ---
		CCI_RangeAndBearingSensor::TReadings newTPackets;
		CCI_RangeAndBearingSensor::SPacket newSPacket;
	    m_tNeighbors[pcFB].push_back(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position);
	    
	    //~ <pipuck id="pipuck66" controller="pipuck" can_send_to="pipuck329,pipuck293,pipuck336,pipuck272" />
	    
	    if(currTime==10){
			m_cOutput4 << "\t<pipuck id=\"pipuck" << unID << "\" controller=\"pipuck\" can_send_to=\"pipuck";
		}
		
		for(int j = 0; j < G[unID].size(); j++){
			newSPacket.Data.Resize(8);
			// --- Get the ID of the node --- 
			newNode = G[unID][j];
			
			if(m_bShowLinks) {
				std::string fbID = "fb" + std::to_string(newNode); 
				CFootBotEntity& cFootBot2 = *any_cast<CFootBotEntity*>(m_cFootbots[fbID]);
				
				m_tNeighbors[pcFB].push_back(cFootBot2.GetEmbodiedEntity().GetOriginAnchor().Position);
			}
			
			if(currTime == 10){ 
				m_cOutput3 << unID << "\t" << newNode << std::endl; 
				m_cOutput4 << newNode;
				if(j<G[unID].size()-1){
					m_cOutput4 << ",pipuck";
				}
				else{
					m_cOutput4 << "\" />\n";
				}
			}
			
			// --- Save the ID of the node in the fakeSensor --- 
			UInt8 rest = newNode % 255;
			UInt8 multiplier = (newNode - rest) / 255;
			newSPacket.Data[0] = multiplier;
			newSPacket.Data[1] = rest;
			
			// --- Save the attitude of the node in the fakeSensor --- 
			int newAttitude = (int) round (attitudes[newNode] * 100);
			rest = newAttitude % 255;
			multiplier = (newAttitude - rest) / 255;
			newSPacket.Data[2] = multiplier;
			newSPacket.Data[3] = rest;
			
			// --- Save the energy of the node in the fakeSensor --- 
			int newEnergy = energies[newNode];
			rest = newEnergy % 255;
			multiplier = (newEnergy - rest) / 255;
			newSPacket.Data[4] = multiplier;
			newSPacket.Data[5] = rest;
			
			// --- Save the energy of the node in the fakeSensor --- 
			int newAgentToShareWith = newAgentsToShareWith[newNode];
			rest = newAgentToShareWith % 255;
			multiplier = (newAgentToShareWith - rest) / 255;
			newSPacket.Data[6] = multiplier;
			newSPacket.Data[7] = rest;
			
			// --- Add the fake newS to the vector of all readings --- 
			newTPackets.push_back(newSPacket);
			newSPacket.Data.Clear();
			
			// --- Output the local degree distribution ---
			if(currTime % 500 == 0 || currTime == 2500){
				//~ m_cOutput4 << currTime << "\t" << cController.IntegerId() << "\t" << newNode << "\t" << newAttitude << std::endl;
			}
		}
		cController.SetFakeTPackets(newTPackets);
		
		for(int newNode=0; newNode < newAgentsToShareWith.size(); newNode++){
			if(unID == newAgentsToShareWith[newNode]) { cController.ReceiveEnergyInstantly(); } 
		}
		cController.ResetAgentsToShareWith();
	}
	
	/****************************************/
	/****************************************/
	
	std::vector<double> sumOfProbabilities(10, 0);
	std::vector<int> energyLevels;
	Real avgAttitude = 0.0, avgDegree = 0.0, energySum = 0.0;
	numberOfFootbots = m_cFootbots.size();
	
	Real discovery_events_count = 0.0;
	
	CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
	//Go through them
	for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
		CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
		/* Create a pointer to the current foot-bot */
		CFootBotDiffusion& cController = dynamic_cast<CFootBotDiffusion&>(pcFB->GetControllableEntity().GetController());
        CFootBotDiffusion::SharingData& sSharingData = cController.GetSharingData();

		if(sSharingData.isSharing){
			if(cController.isDebug()){LOG << sSharingData.ownID << " -> " << sSharingData.sharingWith << std::endl;}
			//~ m_cOutput3 << sSharingData.ownID << " -> " << sSharingData.sharingWith << "\t" << cController.GetEnergy() << std::endl;
		}
		
		if(staticToDynamic){
			// ----------------------- ITEM DISCOVERY SIMULATION (PROBABILISTIC) ----------------------- 
			if(currTime>=2500){
				if(item_discovery_probability*0.5 > m_pcRNG->Uniform(CRange<Real>(0.0, 1.0))){
					cController.IsDetectedEnergy(true);
					discovery_events_count += 1.0;
				}
			}
			else{
				if(currTime > 5 && item_discovery_probability > m_pcRNG->Uniform(CRange<Real>(0.0, 1.0))){
					cController.IsDetectedEnergy(true);
					discovery_events_count += 1.0;
				}
			}
			// ----------------------------------------------------------------------------------------- 
		}
		else{ 
			CVector2 cPos;
			cPos.Set(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
					 pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
			bool done = false;

			if(currTime>=2500){
				for (size_t i = 0; i < (int) round (m_cEnergyPos.size()/2) && !done; i++) {
					if ((cPos - m_cEnergyPos[i]).SquareLength() < m_fEnergySquareRadius) {
						m_cEnergyPos[i].Set(m_pcRNG->Uniform(CRange<Real>(-arenaX + 0.1, arenaX - 0.1)),
											m_pcRNG->Uniform(CRange<Real>(-arenaY + 0.1, arenaY - 0.1))
											);
						/* The floor texture must be updated */
						m_pcFloor->SetChanged();
						cController.IsDetectedEnergy(true);
						done = true;
						discovery_events_count += 1.0;
					}
				}
			}
			else{
				for (size_t i = 0; i < m_cEnergyPos.size() && !done; i++) {
					if (currTime > 5 && (cPos - m_cEnergyPos[i]).SquareLength() < m_fEnergySquareRadius) {
						m_cEnergyPos[i].Set(m_pcRNG->Uniform(CRange<Real>(-arenaX + 0.1, arenaX - 0.1)),
											m_pcRNG->Uniform(CRange<Real>(-arenaY + 0.1, arenaY - 0.1))
											);
						/* The floor texture must be updated */
						m_pcFloor->SetChanged();
						cController.IsDetectedEnergy(true);
						done = true;
						discovery_events_count += 1.0;
					}
				}
			}
			
			if(done){
				if(cController.isDebug()){LOG << cController.GetId() << "detected" << std::endl;}
			}
		}
		if(cController.isDebug()){LOG << cController.GetDegree() << std::endl;}
		
		energyLevels.push_back(cController.GetEnergy());
		avgAttitude += cController.GetAttitude();
		avgDegree += cController.GetDegree();
		energySum += cController.GetEnergy();

		if(currTime % 500 == 0){
			//~ m_cOutput2 << currTime << "\t" << cController.IntegerId() << "\t" << cController.GetAttitude() << std::endl;
			m_cOutput3 << currTime << "\t" << cController.IntegerId() << "\t" << cController.GetEnergy() << std::endl;
		}
	}
	
	avgAttitude /= (1.0*numberOfFootbots);
	avgDegree /= (1.0*numberOfFootbots);
	discovery_events_count /= (1.0*numberOfFootbots);
	
	if(currTime % 100 == 0){
		LOG << energySum << std::endl;
	}
	
	//~ m_cOutput3 << currTime << "\t" << discovery_events_count << std::endl;
	//~ m_cOutput3 << currTime << "\t" << avgDegree << std::endl;
	
	if(currTime % 10 == 0){
		//~ m_cOutput1 << currTime << "\t" << avgAttitude << "\t" << discovery_events_count << std::endl;
	}
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAltBehLoopFunctions, "altruistic_behavior_loop_functions")
