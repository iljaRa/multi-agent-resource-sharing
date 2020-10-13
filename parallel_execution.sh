#!/bin/bash

NODE=1;
TOPOLOGY=scale-free
TYPE=sudden
GLOB_DEGREE=8;
MIN_SEED=1;
MAX_SEED=30;
PARENTDIR="$TOPOLOGY"_testTest
CODENAME="$PARENTDIR"_"$TYPE"_"$NODE";   # <-------------------------------------- CODENAME
argExp=~/argos_altruistic_behavior;
argosFile=$argExp/results/$CODENAME/altruistic_behavior.argos;

dir=$CODENAME;

sudo localedef -i en_US -f UTF-8 en_US.UTF-8;
export LANGUAGE=en_US.UTF-8;
export LANG=en_US.UTF-8;
export LC_ALL=en_US.UTF-8;
sudo locale-gen en_US.UTF-8;

# --- Edit the configuration file ---------------------------------------

xmlstarlet ed --inplace -u 'argos-configuration//framework/experiment/@length' -v 2005 $argosFile;

xmlstarlet ed --inplace -u 'argos-configuration//controllers/altruistic_behavior_controller/params/@initial_attitude' -v 0.8 $argosFile;
xmlstarlet ed --inplace -u 'argos-configuration//controllers/altruistic_behavior_controller/params/@attitude_increment' -v 0.1 $argosFile;
xmlstarlet ed --inplace -u 'argos-configuration//controllers/altruistic_behavior_controller/params/@energy_gain' -v 10 $argosFile;

xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@network_type' -v "$TOPOLOGY" $argosFile;
xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@degree' -v 4 $argosFile;
xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@probability' -v 0.1 $argosFile;

xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@dynamic_env' -v true $argosFile;
xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@dynamic_env_type' -v "$TYPE" $argosFile;

fullDir=/users/irausch/argos_altruistic_behavior/results/$CODENAME/;
xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/enery/@directory' -v "$fullDir/" $argosFile;

xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/enery/@percentageCooperative' -v 0.8 $argosFile; 

xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@range' -v 1.485 $argosFile;
#~ ---------------------------------------------------------------------


# --- Set the parameters of collective behaviour ---------------------------------------
#~ ATTRIBUTE_INCREMENT=3;
INITIAL_ATTRIBUTE=3;
MIN_ATTINCRNT=3;
MAX_ATTINCRNT=3;
#~ MIN_INITATT=0;
#~ MAX_INITATT=2;
MIN_INITATT=$INITIAL_ATTRIBUTE;
MAX_INITATT=$INITIAL_ATTRIBUTE;

#~ MIN_DEGREE=2;
#~ MAX_DEGREE=5;
MIN_DEGREE=$GLOB_DEGREE;
MAX_DEGREE=$GLOB_DEGREE;
#~ -------------------------------------------------------------------------------------


# --- Create sub-paths ------------------------------------------------------------------
cd $fullDir;

for degreeElem in `seq $MIN_DEGREE $MAX_DEGREE`;   
	do mkdir "$degreeElem"; 
	for attincrElem in `seq $MIN_ATTINCRNT $MAX_ATTINCRNT`;  
		do mkdir "$degreeElem"/"$attincrElem"; 
	done;
done;

cd $argExp; # <---------------------------------- This one is important!
# ---------------------------------------------------------------------

for DEGREE in `seq $MIN_DEGREE $MAX_DEGREE`;  
	do echo "RUNNING $TOPOLOGY with attincrnt values $MIN_ATTINCRNT to $MAX_ATTINCRNT"

	parallel --delay 1.0 'PATH_ARG={1}; SEED={2}; ATTINCRNT={3}; INITATT={4}; DEGREE={5};
		fullPath=$PATH_ARG/$DEGREE/$ATTINCRNT/$SEED; 
		mkdir $fullPath;
		cp $PATH_ARG/altruistic_behavior.argos $fullPath;
		argFile=$fullPath/altruistic_behavior.argos;
		
		arrayATTINCRNT=(0.01 0.1 0.5 1.0);
		arrayINITATT=(0.0 0.25 0.5 0.75 1.0);	
		arrayDEGREE=(0 2 4 6 8 10);		
		
		xmlstarlet ed --inplace -u 'argos-configuration//framework/experiment/@random_seed' -v $SEED $argFile; 
		xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@directory' -v "$fullPath/" $argFile;
		xmlstarlet ed --inplace -u 'argos-configuration//controllers/altruistic_behavior_controller/params/@attitude_increment' -v ${arrayATTINCRNT["$ATTINCRNT"]} $argFile;
		xmlstarlet ed --inplace -u 'argos-configuration//controllers/altruistic_behavior_controller/params/@initial_attitude' -v ${arrayINITATT["$INITATT"]} $argFile;
		xmlstarlet ed --inplace -u 'argos-configuration//loop_functions/energy/@degree' -v ${arrayDEGREE["$DEGREE"]} $argFile;
		
	argos3 -l $fullPath/log -e $fullPath/logerr -c $argFile' ::: "$fullDir" ::: `seq $MIN_SEED $MAX_SEED` ::: `seq $MIN_ATTINCRNT $MAX_ATTINCRNT` ::: "$INITIAL_ATTRIBUTE" ::: "$DEGREE"

	cd $fullDir;
	tar czf "$CODENAME"_DEGREE_"$DEGREE"_NODE_"$NODE".tar.gz "$DEGREE"/;

	cd $argExp; # <---------------------------------- This one is important!
	
done;
	
echo "DONE."
