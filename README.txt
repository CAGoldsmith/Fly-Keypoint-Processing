----- angleAnalysis.m -----
Plots the angles for each mobile DoF in a kinematic leg chain as it attempts to best match the fly keypoint data. Keypoint analysis itself is done by runLegErrorTrials.m, this just pulls the angle data from the corresponding "Robot Angles New" file and plots it. DoF angles from all keypoint datasets are plotted together to aid in comparison.

----- flyLimbProportions.m -----
Calculates the ratios of the leg segments/the distances between them on the body for the fly keypoint data, then averages them to create the proportions of a stereotypical fly. Then applies these proportions to a robot model with a given desired leg segment length (currently file has lines for a 10cm middle leg femur or a 28mm middle leg trochanter). These values are given for an idealized robot, as well as for one with actuator size limitations considered.

----- ThCExamples.m -----
Some old code testing how to implement the forward kinematics for the ThC joint. Unused for anything currently.

----- ThCTrFAnalysis.m -----
Old code plotting how the femur-tibia leg plane rotates with respect to the trochanter for the fly keypoint data.

----- runLegErrorTrials.m -----
Automates analyzing the fly keypoint data by calling the findMinLegError function for a variety of keypoint files, legs, and DoF fixture configurations specified at the top of the file. For each combination of keypoint file and leg ID, the file processes the keypoint data to determine the fly's joint positions and leg segment magnitudes at each frame, then passes that data to findMinLegError to do the analysis.

---- findMinLegError.m -----
Function that tries to map a kinematic leg chain with the provided DoF fixturing configuration to the provided fly keypoint data. For each frame of keypoint data, uses the InvKinError function to minimize the error of each of the leg chain's joint's spatial position to the fly data, then records the new leg chain configuration and plots it alongside the fly data. If told to plot by runLegErrorTrials, also plots how each joint's spatial error and the leg plane of the leg chain and fly leg change over the trial, as well as the TiTar position as an analog to the tarsus. Exports a GIF of the leg chain over the trial and plots of the joint point error, the leg plane comparison, and the TiTar position.

---- InvKinError.m -----
Function that finds the error in the spatial position of each joint in the leg chain between the robot and the fly. Error for each joint is normalized by the magnitude of the proximal segment and multiplied by a weight set to nudge findMindLegError.m into prioritizing more distal joints in its error minimization. 

-----  oneLegForwKin.m -----
Function that computes the forward kinematics for the fly-inspired leg chain given joint angles for each DoF and leg segment magnitudes. Used by InvKinError.m and findMinLegError.m to perform inverse kinematics for the robot leg chain.
 