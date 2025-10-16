## Robot Control Dashboard - Local Web UI for PyBullet Simulation
TEAM 4 : 
Delphine Oben, 
Rod Raemon, 
Ruozi Jiao, 
Manuel Gonzalez, 
Bolun Du, 
Yuerong Wu _Cathy

# problem
Coding Dependency: Many robotics simulations rely on command line interfaces or require to write code. This approach is powerful but can be time consuming.

Exclusion of non programmers: Dependance on coding means that individuals like mechanical engineers, designers and managers who test a robot's design or operations cannot do so without a programmer's assistance. This approach slows down the design cycle.

This highlights the need of accessible and intuitive tools. A solution would offer a graphical user interface for ease of control and operation. It allows for a wider range of users to participate in design and testing and accelerate innovation and advancement.

# How to run
1. Download the Docker image of this ROS2 built:  intel4coro/jupyter-ros2:jazzy-py3.12
2. Using this command to create a container:
  docker run -d -p 8888:8888 -p 8501:8501 -v <path_to_folder_in_your_own_computer>:/home/jovyan/work --name my_jupyter_ros2 intel4coro/jupyter-ros2:jazzy-py3.12 jupyter lab --NotebookApp.token=''
3. Go to http://localhost:8888/ to start JupyterLab
4. The first time this container needs to install some packages:
   pip install streamlit pybullet 
5. download all files in this repo to the <path_to_folder_in_your_own_computer>
6. In the Jupyter Desktop, open a terminal and go to /home/jovyan/work
7. Run this to start the UI, it will be accessed at http://localhost:8501/ :
  streamlit run src/app.py --server.headless true --server.port 8501 

# Success Metrics:

Start/Stop: Start and stop the PyBullet simulation
Go-to-pose: Upon user input the target pose and hit move button, the robot in simulation should perform the moving action towards the target. 
Speed slider: When a user changes the robot speed using a slider, the robot in the simulation should change its moving speed in real time. The speed slider will have a minimum and maximum speed preconfigured to reasonable numbers
Smooth UI with minimal latency and should be intuitive to user.

# demo
Demo videos are in data folder
