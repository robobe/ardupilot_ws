FROM jazzy:allision

# TODO fix it build base docker again
RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-robot-localization \
    ros-jazzy-rqt-robot-steering \
    ros-jazzy-plotjuggler \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-tf-transformations \
  && rm -rf /var/lib/apt/lists/*

# region install mavros
RUN apt-get update \
&& apt-get -y install --no-install-recommends \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*

RUN ros2 run mavros install_geographiclib_datasets.sh

# endregion install mavros

# region dds ros packages
RUN apt-get update \
&& apt-get -y install --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*
# end region

RUN apt-get update \
    && apt-get -y install --no-install-recommends \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-tools \
        python3-gi \
        gir1.2-gstreamer-1.0 \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-libav \
        gstreamer-1.0 \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*