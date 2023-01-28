FROM ros:humble

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
        ros-humble-image-pipeline ros-humble-tracetools-* \
        ros-humble-image-common lttng-tools liblttng-ust-dev \
        babeltrace python3-bt2 ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cyclonedds* net-tools
