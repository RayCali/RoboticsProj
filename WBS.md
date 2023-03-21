# Work Breakdown Structure
Here we write down the larger tasks. For each task we recursively break it down into smaller tasks until we have tasks small enough that each one of us understands what needs to be done in order to complete the task. 
There are two parts to the WBS: 
* **The core process** This is MS1-3 criteria
* **The support process** This is everything that improves our productivity

# The core process
* ESTIMATE THE POSITION OF BOXES & DISPLAY RVIZ
  * SLAM
    * ODOMETRY
      * FILTERING
        * IMU & DEAD RECKONING (12)
    * PERCEPTION
      * ARUCO MARKER
        * MAP & MEMORY
          * ESTIMATE TRANSFORM FROM MAP TO OBJECT (4)
          * PUBLISH TRANSFORM TO TF2 (2)
      * OBJECT DETECTION
        * CREATE DATASET (4*)
        * LEARN PYTORCH (8)
  * DISPLAY
    * DISPLAY NEW OBJECT (1)
    * REMOTE RVIZ (8*)

  * MAPPING
    * GRID MAPPING (UNK/OBJ/OBST) (200)

  * LOCALIZATION (200)
  * INTEGRATION (240-T)

* PATH PLANNING
  * OBSTACLE AVOIDANCE (200)
  * ARM KINEMATICS 
    * REVERSE KINEMATICS (16)
    * PLANNER (8)

* INTEGRATION (100)

# The support process

* REMOTE
  * GIT
    * CREATE BRANCHES
    * CLONE ON PERSONAL SSDs (8)
      * LAUNCH WITH BAG (16)
    * ROSBAGS (4)

  * SSH
    * TUNNEL FROM SSD (1)
    * LAUNCH TMUX
      * DOWNLOAD CHEAT SHEET (1)

  * REMOTE RVIZ
    * LAUNCH FROM SSD (8)

  * GOOGLE COLAB (4)