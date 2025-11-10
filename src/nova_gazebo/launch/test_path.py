import os

gazebo_models_path = "src/nova_gazebo/custom_models"

print("Adding gazebo models path in test_path.py: ", os.getcwd() + "/" + gazebo_models_path)
os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

print("Gazebo models path in test_path.py: ", os.environ["GZ_SIM_RESOURCE_PATH"])