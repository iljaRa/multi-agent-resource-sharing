/* Include the controller definition */
#include "altruistic_behavior.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Function definitions for logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <cmath>

#include <algorithm>

#include <limits>

/****************************************/
/****************************************/

CFootBotDiffusion::SharingData::SharingData() :
   isSharing(false),
   ownID(0),
   sharingWith(0) {}

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
	m_pcWheels(NULL),
	m_pcProximity(NULL),
	m_cAlpha(10.0f),
	m_fDelta(0.5f),
	m_pcRNG(NULL),
	m_fWheelVelocity(2.5f),
	energyGain(1),
	m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
	                        ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

/****************************************/
/************* INIT *************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {

	m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcRNG = CRandom::CreateRNG("argos");
	m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
	m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
	m_pcRABA  = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing");
	m_pcRABSens = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing" );
	
	GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
	GetNodeAttribute(t_node, "decayrate", decayrate);
	GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
	GetNodeAttribute(t_node, "debug", b_DEBUG);
	GetNodeAttribute(t_node, "initial_attitude", attitude);
	GetNodeAttribute(t_node, "energy_gain", energyGain);	
	
	isDetectedEnergy = false;
	energy = 100;
	degree = 10;
	
	agentToShareWith = 9999;
	attitudeIncrement = 0.006;
	
	sSharingData.isSharing = false;
	sSharingData.ownID = IntegerId();
	sSharingData.sharingWith = IntegerId();
	
	//~ Broadcast own ID
	int ownID = IntegerId();
	UInt8 rest = ownID % 255;
	UInt8 multiplier = (ownID - rest) / 255;
	m_pcRABA->SetData(0,multiplier);
	m_pcRABA->SetData(1,rest);
}

/****************************************/
/************* CONTROL STEP *************/
/****************************************/
void CFootBotDiffusion::ControlStep() {
	
	ObstacleAvoidance();
	
	if(tick > 5){
		if(isDetectedEnergy){
			Decide();
			isDetectedEnergy = false;
		}
		else if(sSharingData.isSharing){
			state = ROAMING;
			sSharingData.isSharing = false;
			attitude -= attitudeIncrement;
		}
		else{
			//~ Reset the sharing ID
			agentToShareWith = 9999;
			attitude -= attitudeIncrement;
		}
		//~ EnergyDecay();
		UpdateState();
	}
	tick++;
	
	//~ Broadcast the agent's state / attitude
	int dummy = (int) (attitude * 100);
	UInt8 rest = dummy % 255;
	UInt8 multiplier = (dummy - rest) / 255;
	m_pcRABA->SetData(2,multiplier);
	m_pcRABA->SetData(3,rest);
	
	//~ Broadcast the energy level
	dummy = (int) energy;
	rest = dummy % 255;
	multiplier = (dummy - rest) / 255;
	m_pcRABA->SetData(4,multiplier);
	m_pcRABA->SetData(5,rest);
	
	rest = agentToShareWith % 255;
	multiplier = (agentToShareWith - rest) / 255;
	m_pcRABA->SetData(6,multiplier);
	m_pcRABA->SetData(7,rest);
}

/****************************************/
/****************************************/

/****************************************/
/************* DETECT ENERGY *************/
/****************************************/

void CFootBotDiffusion::IsDetectedEnergy(bool detectedEnergy) {
	isDetectedEnergy = detectedEnergy;

}

/****************************************/
/****************************************/

/****************************************/
/************* Decide *************/
/****************************************/

void CFootBotDiffusion::Decide() {
	Real r = m_pcRNG->Uniform(CRange<Real>(0, 1.0));
		
	if (r < attitude) {
		Share();
	}
	else{
		if(b_DEBUG){LOG << GetId() << "is consuming" << std::endl;}
		ConsumeEnergy();
	}

}

/****************************************/
/****************************************/

/****************************************/
/************* SHARE ENERGY *************/
/****************************************/

void CFootBotDiffusion::Share() {
	//~ const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
	degree = tPackets.size();
	if(degree > 0){
		int neighborEnergy = 0;
		int lowestEnergy = std::numeric_limits<int>::max();
		Real neighborAttitude = 0.0;
		Real neighborAttitude_Altruistic = 0.0, neighborAttitude_Selfish = 0.0;

		if(b_DEBUG){LOG << "neighbors' energies: ";}
		for(size_t i = 0; i < degree; i++) {
			//~ 1. Compute the average altruism/selfishness attitude
			neighborAttitude = 0.01 * (tPackets[i].Data[2]*255 + tPackets[i].Data[3]);
			if(neighborAttitude > 0.5){
				neighborAttitude_Altruistic += 1.0;
			}
			else if(neighborAttitude < 0.5){
				neighborAttitude_Selfish += 1.0;
			}
			
			//~ 2. Get the neighbor with the lowest energy
			neighborEnergy = tPackets[i].Data[4]*255 + tPackets[i].Data[5];
			if(neighborEnergy < lowestEnergy){
				lowestEnergy = neighborEnergy;
				agentToShareWith = tPackets[i].Data[0]*255 + tPackets[i].Data[1];
			}
			
			if(b_DEBUG){LOG << agentToShareWith << "<>" << neighborEnergy << " :: ";}
		}
		if(b_DEBUG){LOG << std::endl;}
		neighborAttitude_Altruistic = neighborAttitude_Altruistic/(1.0*degree);
		neighborAttitude_Selfish = neighborAttitude_Selfish/(1.0*degree);
		
		attitude += (neighborAttitude_Altruistic - neighborAttitude_Selfish);
		
		//~ Broadcast who I am sharing with
		UInt8 rest = agentToShareWith % 255;
		UInt8 multiplier = (agentToShareWith - rest) / 255;
		m_pcRABA->SetData(6,multiplier);
		m_pcRABA->SetData(7,rest);
		
		sSharingData.isSharing = true;
		sSharingData.sharingWith = agentToShareWith;
		if(b_DEBUG){LOG << GetId() << " -> " << agentToShareWith << std::endl;}		
	}
	else{
		ConsumeEnergy();
	}
}

/****************************************/
/****************************************/

/****************************************/
/************* CONSUME ENERGY *************/
/****************************************/

void CFootBotDiffusion::ConsumeEnergy() {
	energy += 1;
	//~ energy += energyGain;
	
	degree = tPackets.size();
	if(degree > 0){
		Real neighborAttitude = 0.0;
		Real neighborAttitude_Altruistic = 0.0, neighborAttitude_Selfish = 0.0;

		for(size_t i = 0; i < degree; i++) {
			neighborAttitude = 0.01 * (tPackets[i].Data[2]*255 + tPackets[i].Data[3]);
			if(neighborAttitude > 0.5){
				neighborAttitude_Altruistic += 1.0;
			}
			else if(neighborAttitude < 0.5){
				neighborAttitude_Selfish += 1.0;
			}
		}	
		neighborAttitude_Altruistic = neighborAttitude_Altruistic/(1.0*degree);
		neighborAttitude_Selfish = neighborAttitude_Selfish/(1.0*degree);
		
		attitude += (neighborAttitude_Altruistic - neighborAttitude_Selfish);
	}
}

/****************************************/
/****************************************/

/****************************************/
/************* RECEIVE ENERGY *************/
/****************************************/

/****************************************/
/****************************************/

void CFootBotDiffusion::ReceiveEnergyInstantly(){
	//~ energy += energyGain;
	energy += 1;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ResetAgentsToShareWith(){
	agentToShareWith = -1;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::UpdateState(){
	if(attitude < 0.0){
		attitude = 0.0;
	}
	else if(attitude > 1.0){
		attitude = 1.0;
	}
}

/****************************************/
/****************************************/

/****************************************/
/************* MISC HELPER FUNCTIONS *************/
/****************************************/

int CFootBotDiffusion::IntegerId() {
	std::string id = GetId();
	id = id.substr(2, id.size() - 1);
	return stoi(id);
}


void CFootBotDiffusion::EnergyDecay() {
	decayrate = 1;
	if (energy > decayrate) {
		energy -= decayrate*0;
	}
}

/****************************************/
/****************************************/

/****************************************/
/************* GETTERS *************/
/****************************************/

CCI_RangeAndBearingSensor::TReadings CFootBotDiffusion::GetRealTPackets(){
   return m_pcRABSens->GetReadings();
}

Real CFootBotDiffusion::GetEnergy() {
	return energy;
}

int CFootBotDiffusion::GetDegree() {
	return degree;
}

float CFootBotDiffusion::GetSpeed(){
	return m_fWheelVelocity;
}

Real CFootBotDiffusion::GetAttitude() {
	return attitude;
}

int CFootBotDiffusion::GetAgentToShareWith() {
	return agentToShareWith;
}

/****************************************/
/****************************************/

/****************************************/
/************* SETTERS *************/
/****************************************/

void CFootBotDiffusion::SetFakeTPackets(CCI_RangeAndBearingSensor::TReadings& newTPackets){
	degree = newTPackets.size();
	tPackets.clear();
	
	for(size_t i = 0; i < degree; i++) {
		tPackets.push_back(newTPackets[i]);
	}	
}

void CFootBotDiffusion::SetSpeed(float speed){
	m_pcWheels->SetLinearVelocity(speed, speed);
	m_fWheelVelocity =speed;
}

void CFootBotDiffusion::SetAttitude(Real newAttitude) {
	attitude = newAttitude;
}

void CFootBotDiffusion::SetEnergy(Real newVal) {
	energy = newVal;
}

/****************************************/
/****************************************/

/****************************************/
/************* OBSTACLE AVOIDANCE *************/
/****************************************/

void CFootBotDiffusion::ObstacleAvoidance() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for (size_t i = 0; i < tProxReads.size(); ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();
	/* If the angle of the vector is small enough and the closest obstacle
	 * is far enough, continue going straight, otherwise curve a little
	 */
	CRadians cAngle = cAccumulator.Angle();
	if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
	        cAccumulator.Length() < m_fDelta ) {
		/* Go straight */
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	}
	else {
		/* Turn, depending on the sign of the angle */
		if (cAngle.GetValue() > 0.0f) {
			m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
		}
		else {
			m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
		}
	}

}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "altruistic_behavior_controller")
