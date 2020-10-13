# A C++ based simulation of a collective resource sharing behaviour of an altruistic multi-agent system, using ARGoS. 
Repository for the code corresponding to our research "Modeling the Influence of Social Feedback on Altruism using Multi-Agent Systems" published by the MIT Press https://doi.org/10.1162/isal_a_00256 .

## Installation

- Install ARGoS following: https://www.argos-sim.info/ or https://github.com/ilpincy/argos3
- Compile code inside the cloned `multi-agent-resource-sharing/` directory like this:
```
mkdir build
cd build
cmake ..
make
cd .. 
```
- Run your first experiment
```
argos3 -c experiments/altruistic_behavior.argos
```

For brief tutorials on how to use ARGoS, please see http://argos-sim.info/examples.php and https://osf.io/48b9h/wiki/Controlling%20the%20experiment/ . 

The experiments can take a long time to run if the experient duration is long or if the swarm is large. 
To easr parallel execution of several experiments with different seeds of the random number generator, we added a convenience script `parallel_execution.sh`.
You might need to install the GNU parallel package:
```
sudo apt-get install parallel
```

## Citing our work
If these simulations helped you in your scientific or commercial project, please consider supporting us by citing our corresponding work:
```
@inproceedings{Rausch2020modeling,
  title={Modeling the Influence of Social Feedback on Altruism using Multi-Agent Systems},
  author={Rausch, Ilja and Nauta, Johannes and Simoens, Pieter and Khaluf, Yara},
  booktitle={Artificial Life Conference Proceedings},
  pages={727--735},
  year={2020},
  organization={MIT Press}
}

```
