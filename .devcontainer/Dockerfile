FROM ros:humble

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
        ros-humble-image-pipeline ros-humble-tracetools-* \
        ros-humble-image-common lttng-tools liblttng-ust-dev \
        babeltrace python3-bt2 ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cyclonedds* net-tools less python3 python3-pip \
        python3-tk software-properties-common vim babeltrace2 \
        python3-babeltrace ros-humble-stereo-image-proc \
        ros-humble-theora-image-transport ros-humble-gazebo-msgs \
        ros-humble-rqt* ros-humble-depthimage-to-laserscan \
        ros-humble-rviz2 libogre-1.12-dev

RUN pip3 install wasabi bokeh==2.4.3 selenium plotly kaleido arrow
RUN pip3 install xacro

RUN add-apt-repository ppa:mozillateam/ppa -y
RUN echo "Package: *\nPin: release o=LP-PPA-mozillateam\nPin-Priority: 1001\n\nPackage: firefox\nPin: version 1:1snap1-0ubuntu2\nPin-Priority: -1" > /etc/apt/preferences.d/mozilla-firefox
RUN apt-get update && apt-get install -y firefox firefox-geckodriver

# tracing
# NOTE: lttng-modules-dkms should go in the host machine and NOT
      # in the container. Should be mounted as a volume in the container
      # at runtime.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
        ros-humble-image-transport ros-humble-tracetools-* \
        ros-humble-image-common lttng-tools liblttng-ust-dev \
        liblttng-ust-common1 babeltrace2 \
        python3-lttng python3-lttnganalyses python3-lttngust \
        python3-bt2 babeltrace2 ros-humble-rqt-graph && \
        rm -rf /var/lib/apt/lists/*      

# setup entrypoint
COPY .devcontainer/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
