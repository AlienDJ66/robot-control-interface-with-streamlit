# problem
//todo find the slides and fill out the problem section

How to run
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

metric
//todo find the slides to fill
