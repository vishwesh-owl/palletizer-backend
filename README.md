This repository contains python programs to run the OWL CS66 robotic arm for palletizing applications.

The codes can be divided into three categories: Constant Time Based Movements, TCP Feedback Based Movements, Layer Package Movements

Constant Time Based Movements: This uses the time.sleep function to execute waypoints one by one after a set time that can be adjusted according to requirements.

1. mark1_const_time.py - First iteration of the algorithm for palletization. Movements are executed one by one after constant time delays.
2. oddeven.py - Deals with alternation of layer configurations in a pallet.
3. twopallet.py - Deals with placement of boxes from  single line in an alternate fashion between two pallets. Required: Teach 2 master points instead of 1

TCP Feedback Movements: This utilises the constant feedback of TCP position and matches it within a threshold of target positions to execute next movement.
                        Fixes unnecessary time delays.

1. feedbackwithpreoffset.py - Uses TCP Feedback to execute target points.
                              Also has offsets in x ad y axes for the pre-placing points along with z-axis to avoid potential 'z-axis-only' collisions.

Layer Package Movements: Here, points are sent to the robot layer wise. Meaning, all points of layer 1 are sent at one. Once the robot completes layer 1, it sends points of layer 2.

1. continuous_motion.py - Uses package functionality to send all points at once to the robot. Once user inputs are entered, and all points are sent, the code will end execution.
2. layerpackage_plus_liftkit.py - Integrates Ewellix liftkit (7th axis) with the robot for better reachability. NOTE: Ethernet switch is necessary for this.

P.S: To test the 7th axis individually, the code named 'ewellix_liftkit.py' can be used.
