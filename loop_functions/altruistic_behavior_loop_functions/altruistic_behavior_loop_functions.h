#ifndef ALTRUISTIC_BEHAVIOR_LOOP_FUNCTIONS_H
#define ALTRUISTIC_BEHAVIOR_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CAltBehLoopFunctions : public CLoopFunctions {

public:

  CAltBehLoopFunctions();
  virtual ~CAltBehLoopFunctions() {}

  //Initialization of logfiles and other parameters of the experiment
  virtual void Init(TConfigurationNode& t_tree);

  /*
   * Reset experiment to starting values
   */
  virtual void Reset();
  /*
   * Method called upon completion of experiment used to log distribution of energy levels across the swarm
   */
  virtual void Destroy();

  /*
   * Method used to set floor color of the arena
   */
  virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

  /*
   * Method called before every tick of the experiment:
   * - Used to extract averaged data from the footbots and
   * - Used in order to control the number of available energy spots in the environment if configured in this way
   */
  virtual void PreStep();
  
  // Connectivity Graph (G) and "artificial graph" (AG) used for graph construction
   std::vector<std::vector<int>> G;
   std::vector<std::vector<int>> AG;
   
   // Function for constructing the connectivity graph
   void GenerateGraph(std::string m_strNetwork);
   std::vector<int> nodes;
   std::vector<int> degrees;
   
   // ----------------------------------------
   // This is used to draw the links.
   // It is used by the qt_functions in the background, so it should not be removed.
   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TNeighborsMap;
   TNeighborsMap m_tNeighbors;
   
   inline const TNeighborsMap& GetNeighborPositions() const {
      return m_tNeighbors;
   }
   // ----------------------------------------

private:
  /* Random number generator */
  CRandom::CRNG* m_pcRNG;

  /* Vector containing the different positions (x,y) of the consumable energy spots in the arena */
  std::vector<CVector2> m_cEnergyPos;

  /* Pointer to the arena floor entity */
  CFloorEntity* m_pcFloor;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the section
   * of the XML configuration file, under the
   * <loop_functions> <energy> section.
   */

  /* Arena dimensions */
  double arenaX = 10.0;
  double arenaY = 10.0;


  int numberOfFootbots;

  // True if the swarm is static, i.e. the robots are not moving. 
  // The purpose is to simulate probabilistic item discovery.
  bool staticToDynamic;

  UInt32 unEnergyItems;
  
  /* Square radius of an energy spot */
  Real m_fEnergySquareRadius;
  /* File names for logging data */
  std::string m_strOutput1;
  std::string m_strOutput2;
  std::string m_strOutput3;

  /* Ofstreams for logging data */
  std::ofstream m_expInfo;
  std::ofstream m_cOutput1;
  std::ofstream m_cOutput2;
  std::ofstream m_strOutputTEMP;
  std::ofstream m_cOutput3;
  std::ofstream m_cOutput4;
  
  std::string m_strNetwork;
   
  int m_unDegree;
  int m_seed;
  bool m_bShowLinks;
  Real m_fProbability;
  
  bool b_dynamic_env;
  std::string s_dynamic_env_type;
  
  Real item_discovery_probability;

};

#endif
