# Copyright (c) 2019 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

FROM nvidia/cudagl:10.2-devel-ubuntu18.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility,compute

RUN apt-get update && \
    apt-get install --no-install-recommends -y curl apt-transport-https python-pip python3-pip redis && \
    apt-get clean

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -L https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# install depending packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    ros-melodic-desktop-full \
    ros-melodic-gazebo-ros ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control libgazebo9-dev libignition-transport4-dev libpoco-dev python-scipy libgsl-dev \
    ros-melodic-dwa-local-planner \
    ros-melodic-rviz \
    ros-melodic-smach-ros \
    ros-melodic-moveit-commander \
    ros-melodic-eigen-conversions \
    ros-melodic-robot-state-publisher \
    ros-melodic-moveit-core \
    ros-melodic-moveit-plugins \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-ros-planning \
    ros-melodic-moveit-ros-planning-interface \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-ros-manipulation \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-urdfdom-py \
    ros-melodic-roslint \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-move-base \
    ros-melodic-map-server \
    ros-melodic-xacro \
    ros-melodic-joint-state-publisher \
    liburdfdom-tools \
    ros-melodic-image-proc \
    ros-melodic-depth-image-proc \
    ros-melodic-effort-controllers \
    ros-melodic-ros-controllers \
    ros-melodic-navigation \
    ros-melodic-slam-gmapping \
    ros-melodic-pcl-ros \
    ros-melodic-tf-conversions \
    ros-melodic-vision-msgs \
    ros-melodic-moveit-ros-perception && \
    pip install -U --ignore-installed wheel pyassimp supervisor supervisor_twiddler redis && \
    apt-get autoremove -y && \
    apt-get clean

RUN mkdir /wrs_ws
ADD src /wrs_ws/src
#RUN cd /wrs_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace || true
RUN cd /wrs_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace
#RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y
#RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 -DCATKIN_WHITELIST_PACKAGES="darknet_ros_msgs;butia_vision_msgs" -DCATKIN_BLACKLIST_PACKAGES="hector_gazebo_plugins"
#RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="hector_gazebo_plugins"
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=0 -DCATKIN_WHITELIST_PACKAGES="darknet_ros_msgs;butia_vision_msgs" -DCATKIN_BLACKLIST_PACKAGES="hector_gazebo_plugins"
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=0 -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="hector_gazebo_plugins"

ADD entrypoint-wrs.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#ADD filterable-rosmaster.py /opt/ros/melodic/bin/
#RUN rm /opt/ros/$ROS_DISTRO/bin/rosmaster && ln -s /opt/ros/$ROS_DISTRO/bin/filterable-rosmaster.py /opt/ros/$ROS_DISTRO/bin/rosmaster

RUN source /wrs_ws/devel/setup.bash && rosrun tmc_gazebo_task_evaluators setup_score_widget

ADD supervisord.conf /etc/supervisor/supervisord.conf

VOLUME [ "/opt/ros/melodic", "/wrs_ws/" ]

CMD ["/usr/local/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]

