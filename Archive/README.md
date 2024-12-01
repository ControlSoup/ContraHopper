# Quick Overview [No Longer Maintained as of 07-12-23]
The ContraHopper is a small rotor craft using two counter rotating propellers and jet vanes to maintain attitude and altitude. This project is a personal project with unnecessary constraints to mimic the control of VTVL rockets. 

This repository is not intended for collaborative use at this time.


## Formal Introduction
ContraHopper is a project that stems from my personal interest in the design, control, and operation of rocket-powered landers. Landers operate under the final stage of a planetary mission and serve the purpose of transferring the payload to the surface of your target planetary body. Over the years there has been a considerable number of terrestrial landers that attempt to help test technology associated with landing on other planets and in my opinion, these vehicles are the most interesting flying things to leave the surface of the earth. 

Ultimately I have had to scrap this project due to professional goals, I am keeping this up for reference on my future topics.

![](https://github.com/ControlSoup/ContraHopper/blob/main/Documentation/Figures/Terrestial%20Landers.jpg)
Figure 1: Left to Right: Pixel (Armadillo), Xodiac (Masten), Delta Clipper (NASA), Mighty Eagle NASA)

Vehicles like this give me incredible joy. My goal with the project is create a low budget version that is
capable of many of the same things that terrestrial landers do, all from scratch as much as I can help to do.

## Constraints and Milestones

### The constraints are as follows:
  - Minimum 1.5 minutes of flight time
  - Affordable at the Hobby scale
  - Dynamics of the vehicle a close to a rocket as possible at this scale

### The milestones are as follows:
  - Document design decisions and rational
  - Document initial expected vehicle capabilities based on design decisions
  - Full characterization of vehicle using simulation
  - Completion of a 10s hover and landing
  - Completion of a 30s hover and landing with complete position control within a reasonable    tolerance
  - Completion of a 1min flight with a translation in both height and distance, demonstrating position,
    velocity, attitude and trajectory control within a reasonable tolerance


## Folder Structure
Each folder acts as a container for a part of the project, these will eventually hold all files that I used to design an operate ContraHopper for better or for worse. Currently every folder is a work in progress, many contain their own README.md with an overview and and a list of TO-DOs.  
