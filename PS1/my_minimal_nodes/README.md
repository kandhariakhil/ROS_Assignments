# my_minimal_nodes
# Akhil Kandhari Assignment 1

To run please use "roslaunch my_minimal_nodes my_minimal_nodes.launch"

One can also use rosrun to run all three files seperately.

It will prompt user to input an amplitude for the wave and a frequency
After inputs are eneterd, the velocity command values will be printed on the screen at a sampling rate of 10 Hz. 
Please use rqt_plot to see the sin waves and rqt_graph to see the link between the commander, simulataor and controller.

The Commander has the capability to be subscribed to a node that gives a frequency and amplitude as well, however the code by itself uses std::cin for an input by the user.

    
