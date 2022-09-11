# Flow-Constraints
## Authors and Contributors
Apurva Badithela, Josefine B. Graebener,  Wyatt Ubellacker <br />


## Description
A module to generate constrained test environments that leverage flow algorithms.

This branch contains code for reproducing the results in "Multi-Commodity Flow Framework for Synthesis of Reactive Test Environments for LTL Specifications" submitted to ICRA 2023. (to be updated)

## Requirements
Python 3.x<br />
Packages: see **requirements.txt** and TuLiP (to do) <br />

## Instructions
1. Install the required packages by running 'pip install -r requirements.txt' <br />
2. Install TuLiP from https://github.com/tulip-control/tulip-control
3. Run corridor_example/simulate.py for a demonstration of the corridor example <br />
4. Run search_and_rescue_simulation/simulate.py for a demonstration of the robot navigation example <br />
5. To produce an animation of the result run animate.py after step 3. or 4. in the project directory

## Examples
### Corridor Example
![](corridor_example/animations/test_strategy.gif)
### Robot Navigation Example
![](search_and_rescue_simulation/animations/test_strategy.gif)
