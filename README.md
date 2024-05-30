# Tooling for Summer/Winter Schools in Software Engineering for Robotics
---
## Instructors and Students
---
## Prerequisites:

### 1. Install Docker Engine on Ubuntu

First, install Docker Enigne on your Ubuntu system. Follow the official Docker documentation for instructions: [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

## Docker infrastructure
### 2. Clone the Repository
Create a directory for the school environment and clone the repository:
```sh
mkdir ~/school && cd ~/school
git clone https://github.com/IntelligentRoboticsLabs/docker_infrastructure.git
```

### 3. Install the NVIDIA Container Toolkit

To use GPU resources within your Docker containers, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Execute the script:

```sh
cd ~/school/docker_infrastructure/docker/
./nvidia_install.sh
```

- If you have an NVIDIA card in your system and it is configured with the proper drivers, you can execute the following command to switch between the integrated graphics card and the NVIDIA GPU:
```sh 
sudo prime-select nvidia
``` 

- After running prime-select, you will need to restart your system for the changes to take effect:
```sh 
sudo reboot
``` 

### 4. Docker image
There are two ways to get the Docker image:
- 4.1. Downloading it from Docker Hub (**RECOMMENDED**):
```sh
docker pull jmguerreroh/school:ubuntu22
```

- 4.2. Building the image
Navigate to the Docker directory and build the Docker image:
```sh
cd ~/school/docker_infrastructure/docker/
docker buildx build -t jmguerreroh/school:ubuntu22 -f Dockerfile .
```

### 5. Run Docker image:
Run the Docker image using the provided script:
```
./run_docker.sh
```
**Note: If you run this command without having the image, it will attempt to download it from Docker Hub as in step 4.1.

### 6. Access the Environment
Open your browser and go to: http://localhost:6080/

You should see the environment running:

![Environment](images/environment.png)

### 7. Stop and run a container
- 7.1. If the container is currently running and you need to stop it, follow these steps:

    - First, log out of the Docker container environment in your browser:
        ![logout](images/logout.png)

    - Then, stop the Docker container using the following command
```sh
docker stop school
```

This won't delete anything done inside the container unless explicitly removed.

- 7.2. If it's stopped and you want to start it again:
```sh
docker start school
```
---
## Instructors
---
### Before building the Docker image, you can:
Create an installation script with the necessary setup for your courses and place it inside the `installation_scripts` folder.
- Follow steps 4.2 and 5 again to build the Docker image.
- Execute your script inside the Docker container. Once the Docker container is running, open a terminal inside the container and execute your script:

```sh
source /installation_scripts/your_script.sh
```

### With a Docker image built and running
If the students have a Docker image running, consider providing the script via a URL. The students can then download it using the Firefox browser within the Docker environment.