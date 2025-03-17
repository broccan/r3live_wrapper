# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Copy the input files into the container
COPY python_packages.txt /tmp/python_packages.txt
COPY requirements.txt /tmp/installed_packages.txt
COPY repo_urls.txt /tmp/repo_urls.txt

# Install system packages from installed_packages.txt
RUN apt-get update && \
    xargs -a /tmp/installed_packages.txt apt-get install -y && \
    rm -rf /var/lib/apt/lists/*


# Install Python packages from requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Clone GitHub repositories from repo_urls.txt
RUN while read -r repo_url; do \
      repo_name=$(basename "$repo_url" .git); \
      git clone "$repo_url" /catkin_ws/src/"$repo_name"; \
    done < /tmp/repo_urls.txt

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set up the entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
