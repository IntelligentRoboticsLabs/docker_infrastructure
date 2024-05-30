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
There are two ways to get the Docker image (change XX to select the Ubuntu version 20 or 22):
- 4.1. Downloading it from DockerHub:
```sh
docker pull jmguerreroh/school:ubuntuXX
```

- 4.2. Building the image:
Navigate to the Docker directory and build the Docker image:
```sh
cd ~/school/docker_infrastructure/docker/
docker buildx build -t jmguerreroh/school:ubuntuXX -f Dockerfile_ubuntuXX .
```

### 5. Run Docker image:
Run the Docker image using the provided script (change XX to select the Ubuntu version 20 or 22):
```
./run_docker.sh ubuntuXX
```
**Note: If you run this command without having the image, it will attempt to download it from Docker Hub as in step 4.1.

### 6. Access the Environment
Open your browser and go to: http://localhost:6080/

You should see the environment running:

![Environment](images/environment.png)

### 7. Stop and run a container
Before to stop the docker container. If the container is currently running, to stop it you need to execute (change XX to select the Ubuntu version 20 or 22):
```sh
docker stop school_ubuntuXX
```

This won't delete anything done inside the container unless explicitly removed.

If it's stopped and you want to start it again:
```sh
docker start school_ubuntuXX
```
---
## Instructors
---

### 8. Create an installation script
Create a custom installation script with the necessary setup for your courses and place it inside the `installation_scripts` folder.

### 9. Follow steps 4 and 5 again
Build the Docker image and run it as described in steps 4.2 and 5.

### 10. Execute Your Script Inside the Docker
Once the Docker container is running, open a terminal inside the container and execute your script:
```sh
source /installation_scripts/your_script.sh
```

### Install inside a running Docker
You can provide the script on an URL and download it using the Firefox Browser inside the Docker.