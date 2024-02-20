FROM python:3.11

ENV DEBIAN_FRONTEND noninteractive

# create a non-root user
ARG USER_ID=1000
ARG DEVELOP=true
RUN apt update && apt install -y --no-install-recommends \
                              # x11vnc \
							  # xvfb \
	                          ca-certificates \
							  git \
							  wget \
							  sudo \
							  python3-pip \
							  python3-distutils \
							  python3-dev \
							  python3-numpy \
                              cmake \
                              python3-dev \ 
                              python3-numpy \
							  libeigen3-dev \
							  libboost-all-dev \
							  libopencv-dev \
							  clang-tidy \
							  lcov \
							  libgl1-mesa-dev \
							  libglu1-mesa-dev \
							  libglfw3-dev  \
							  libsuitesparse-dev
							  # software-properties-common

RUN python3.11 -m pip install --upgrade pip
RUN ln -sv /usr/bin/python3.11 /usr/bin/python

# RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN useradd -m --no-log-init --system  --uid ${USER_ID} st_handeye_graph -g sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER st_handeye_graph
WORKDIR /home/st_handeye_graph
ENV PATH="/home/st_handeye_graph/.local/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib/:${LD_LIBRARY_PATH}"
COPY ./setup/install_g2o.sh /home/st_handeye_graph/
COPY ./setup/install_visp.sh /home/st_handeye_graph/
RUN ./install_g2o.sh 
RUN ./install_visp.sh
