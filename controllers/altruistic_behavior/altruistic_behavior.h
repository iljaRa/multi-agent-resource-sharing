/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef ALTRUISTIC_BEHAVIOR_H
#define ALTRUISTIC_BEHAVIOR_2_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>


#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {

public:

	/* Class constructor. */
	CFootBotDiffusion();

	/* Class destructor. */
	virtual ~CFootBotDiffusion() {}

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	virtual void Init(TConfigurationNode& t_node);

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	virtual void ControlStep();

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Reset() {}

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Destroy() {}



	virtual void ObstacleAvoidance();

	/*
	 * Method used to detect energy spots on the floor using the Footbot Motor Ground Sensor
	 */
	void DetectEnergy();

	/*
	 * Called to listen for messages broadcasted by other footbots
	 * Could be a message of these types:
	 *
	 * 1. A Request to share your energy level
	 * 2. A Response with an energy level inside
	 * 4. A Selection message: the footbot is selected for consumption and can choose wether or not it obeys the consumption command
	 * 5. An Experience sharing message: the footbot is receiving a message that contains a reward/penalty experience of another footbot
	 *    and will use that experience to shape its own probabilities associated with the action further in a positive or negative manner
	 * 6. A request for approval message: a footbot is in need for approval / dissaproval for an action it has performed
	 * 7. An approval/dissproval message as a response to a certain action performed
	 */
	void MessageListener();

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	 
	 
	 struct SharingData{
		bool isSharing;
		int ownID;
		int sharingWith; 

		SharingData();
	 };
	 
	 
	SharingData sSharingData;

	/*
    * Returns the sharing data
    */
	inline SharingData& GetSharingData() {
	  return sSharingData;
	}

	/*
	 * Make a decision about wheter or not to consume an energy spot after asking surrounding footbots for their energy level, this choice is based on probabilities stored inside the footbot
	 */
	void DecisionAboutConsumptionAfterAsking();

	/*
   	* Method called to make a prediction about the future energy value of a footbot.
   	* Can result in the footbot turning to selfish mode if the net input of energy is lower than the output (decay) and the current energy level is low enough
   	*/
  	void MakePrediction(int);
	/*
	 * Called when a footbot must perform an action after encountering an energy spot
	 */
	void PerformEncounterAction();


	/*
	 * Called when a footbot must perform an action after receiving a request to share its energy level
	 */
	void PerformListenerAction();


	/*
	 * Method used to broadcast a request to neighboring footbots indicating that the footbot wants to know their energy levels
	 */
	void RequestEnergyLevels();

	/*
	 * Method used to broadcast the footbot its energy level
	 */
	void ShareEnergyLevel();
	
	/*
	 * Method used to decide upon encountering an energy item whther to consime or share it
	 */
	void Decide();
	
	/*
	 * Method used to share energy with another robot
	 */
	void Share();
	
	/*
	 * Called to consume an actual energy spot, obtaining energy
	 */
	void ConsumeEnergy();
	
	/*
	 * Method used to receive energy from another robot
	 */
	void ReceiveEnergy();
	
	/*
	 * Sets whether the robot has detected an energy item
	 */
	void IsDetectedEnergy(bool );
	
	/*
	 * Returns the ID of the neighbor to share the energy with
	 */
	int GetAgentToShareWith();
	
	/*
	 * Returns the degree of the robot
	 */
	int GetDegree();

	/*
	 * Sets the agent attitude towards altruistic/selfish behavior
	 */
	void SetAttitude(Real newAttitude);


	/*
	 * Returns an int indicating whether or not the footbot has the lowest energy of its neighbors who responded to the request to share their energy level
	 */
	int CheckNeighbourEnergy();

	/*
	 * Called to reward a footbot for a certain action
	 */
	void Reward(int a, int );


	void UpdateState();

	/*
	 * Called to penalise a footbot for a certain action
	 */
	void Penalise(int amount, int);


	/*
	 * Called when the footbot needs to recalculate the probabilities associated with a certain actions following an obtained reward/penalty through an action or sharing of experience
	 */
	void RecomputeProbabilities(int state,int);

	/*
	 * Set velocity of both footbot wheels from loop functions
	 */
	void SetSpeed(float);


	/*
	 * Method used in order to make the probabilities readable for use by other programs e.g. R
	 */
	void computeLoggableProbabilities();

	/*
	 * Gets cost of communication value
	 */
	double GetCostOfCommunication();

	/*
	 * Returns probabilities stored inside footbot
	 */
	std::map<int, double>&  GetProbabilities();

	/*
	 * Return current footbot state
	 */
	int GetState();
	/*
	 * Reset footbot state to ROAMING
	 */
	void ResetState();
	
	/*
	 * Returns current footbot attitude
	 */
	Real GetAttitude();

	/*
	 * Return int value of footbot id
	 */
	int IntegerId();
	
	void ReceiveEnergyInstantly();
	void ResetAgentsToShareWith();


	/*
	 * Get energy value of footbot
	 */
	Real GetEnergy();

	/*
	 * Set energy value of footbot
	 */
	void SetEnergy(Real);


	/*
	 * Called to linearly decay the energy value inside the footbot
	 */
	void EnergyDecay();


	/*
	 * Method used for footbots in order to avoid colliding in to each other and to other object in the environments such as walls
	 */
	int GetScore();


	/*
	 * Method used to get current speed of a footbot
	 */	
	float GetSpeed();


	/*
	 * Getter for footbot preference
	 */
	int GetPreference();
	/*
	 * Setter for footbot preference
	 */
	void SetPreference(int );
	


	/*
	 * Called to initialize starting probabilities of footbot according to preference
	 */

	void InitializeProbabilities(int);

	/*
	 * Method executed to finalize an approval sequence; e.g. a footbot has asked for approval of a certain action and the time window for responses has passed:
	 * the votes will be tallied up and a reward or punishment will be assigned according to the number of approving vs dissapproving votes
	 */
	void ApprovalSequence();
	enum States {

		CONSUME_WITHOUT_ASKING = 0 ,
		INTERMEDIATE_REQUESTING_ENERGY_LEVELS,
		CONSUME_AFTER_ASKING_LOWEST,
		NOT_CONSUME_AFTER_ASKING_LOWEST,
		CONSUME_AFTER_ASKING_NOT_LOWEST,
		NOT_CONSUME_AFTER_ASKING_NOT_LOWEST,
		NOT_SHARE,
		INTERMEDIATE_SHARING,
		SHARE_AND_NOT_OBEY,
		SHARE_AND_OBEY,
		ROAMING

	};
	
	CCI_RangeAndBearingSensor::TReadings GetRealTPackets();
	void SetFakeTPackets(CCI_RangeAndBearingSensor::TReadings& newTPackets);
	
	CCI_RangeAndBearingSensor::TReadings tPackets;
	
	int degree;


private:
	/* Map of actions to probabilities; with corresponding copy used when a footbot returns from selfish mode to normal mode */
	std::map<int, double> encounterProbabilities;
	std::map<int, double>encounterProbabilitiesCopy;
	std::map<int, double> loggableProbabilities;
	std::map<int, int> neighbourEnergy;
	std::map<int, double> listenerProbabilities;


	/* Pointer to the differential steering actuator */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	/* Pointer to the foot-bot proximity sensor */
	CCI_FootBotProximitySensor* m_pcProximity;

	CCI_FootBotMotorGroundSensor* m_pcGround;


	/* Pointer to the range-and-bearing actuator */
	CCI_RangeAndBearingActuator* m_pcRABA;

	/* Pointer to the range-and-bearing sensor */
	CCI_RangeAndBearingSensor* m_pcRABSens;


	/* Footbot State */
	States state = ROAMING;

	/* Energy level of footbot, has no impact on functionality of footbot and can't go below zero, set by loop functions */
	Real energy;

	/* Number of approved and disapproved actions */
	double approvedChoices = 0;
	double disapprovedChoices = 0;

	/* Internal clock of footbot */
	int tick = 0;



	/* Number of energy spots consumed */
	int consumptions = 0;

	/* Rate of consumption */
	double roc;

	/* Rewards or penalties obtained during the experiment */
	int score = 0;

	/* Double value associating a cost of communication, gets incremented each time a message gets broadcasted with the range and bearing module */
	double costOfCommunication = 0;

	/* This boolean is used in order to delay the clearing of the range and bearing message broadcasting in order to make sure that our experience is broadcasted to neighbouring footbots */
	bool sharingExperience = false;

	/* Footbot will enter selfish mode if their predicted (a certain amount of ticks ahead) energy level goes below a certain value
    * This causes the footbot to take selfish actions, regardless of learned probabilities, e.g. immediate consumption is performed when encountering energy spots when in selfish mode
    */
    bool selfishMode = false;

    /* Threshold at which footbots will enter selfish mode if their predicted energy level (a configurable amount of ticks ahead) goes below this value */
    int energyLowerBound = 30;



	/* Boolean whether or not a footbot is requesting neighbouring footbots their energy levels in order to not get disturbed by requests for energy level by other encountering footbots */
	bool requesting = false;

	/* Tick at which a footbot has sent a request for neighbouring footbots their energy levels, is also used to limit listening time window for gathering responses to an energy level request when there are no other footbots around*/
	int requestTick = -1;
	
	/* Count for received energy level share messages */
	int receiveCount = 0;

	/* Map that is used for remembering the rewards / penalties that are obtained through performing a certain actions
	 * This gives the footbot a good view of the average reward it obtains by performing a certain actions
	 * which is then stored in the avgReward map
	 */
	std::map<int, std::pair<int, int>> rewardExperience;
	std::map<int, double> avgReward;

	/* Id of the footbot sending the first message in a communication sequence */
	int source;


	CRandom::CRNG* m_pcRNG;

	Real attitude;
	
	bool isDetectedEnergy;

	bool b_DEBUG;
	
	int agentToShareWith;


	/* Action that has to undergo social approval, default value -1 */
	int actionForApproval = -1;

	/* Preference for certain models of actions, footbots will use this preference for their approval or dissaproval of actions */
	//0 -> SELFISH MODEL meaning a 0.7 or lesser chance of picking the action
	//1 -> COOPERATIVE MODEL meaning a 0.7 or greater chance of picking the acton
	//-1 -> undecided
	int preference;


	// Increment at which probabilities are updated
	int probabilityIncrement = 5;

	/* Tick at which a request for approval of an action is sent, used for creating a time window where responses can be heard */
	int approvalRequestTick = -1;

	/* Boolean indicating whether or not a footbot has received any approvals/dissaprovals yet */
	bool receivedVotes = false;

	/* Number of approvals / dissaprovals for a vote */
	int approvals = 0;
	int disapprovals = 0;

	/* Boolean indicating that a footbot needs a social response on an action */
	bool socialResponseNeeded = false;


	Real attitudeIncrement;
	Real attitudeIncrement_social;


	/*
	 * The following variables are used as parameters for the
	 * algorithm. You can set their value in the <parameters> section
	 * of the XML configuration file, under the
	 * <controllers><footbot_diffusion_controller> section.
	 */

	/* Maximum tolerance for the angle between
	 * the robot heading direction and
	 * the closest obstacle detected. */
	CDegrees m_cAlpha;
	/* Maximum tolerance for the proximity reading between
	 * the robot and the closest obstacle.
	 * The proximity reading is 0 when nothing is detected
	 * and grows exponentially to 1 when the obstacle is
	 * touching the robot.
	 */
	Real m_fDelta;
	/* Wheel speed. */
	Real m_fWheelVelocity;
	/* Angle tolerance range to go straight.
	 * It is set to [-alpha,alpha]. */
	CRange<CRadians> m_cGoStraightAngleRange;
	/* Rate at which energy level decays over time */
	double decayrate;
	Real energyGain;
	
public:
	
	inline bool isDebug(){
		return b_DEBUG;
	}

};

#endif
