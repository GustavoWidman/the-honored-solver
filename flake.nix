{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/25.05";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs =
    {
      flake-utils,
      nixpkgs,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };

        colconDefaults = pkgs.writeText "defaults.yaml" ''
          build:
            cmake-args:
              - -DPython_EXECUTABLE=/opt/micromamba/envs/ros_env/bin/python
              - -DPython3_EXECUTABLE=/opt/micromamba/envs/ros_env/bin/python
              - -DPYTHON_EXECUTABLE=/opt/micromamba/envs/ros_env/bin/python
              - -DPython3_FIND_STRATEGY=LOCATION
              - -DPython_FIND_STRATEGY=LOCATION
        '';
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            micromamba
            cmake
            pkg-config
            ninja
            gnumake

            (pkgs.writeShellScriptBin "ros-install" ''
              #!/usr/bin/env bash
              micromamba install -y -r "/opt/micromamba" -n ros_env -c conda-forge -c \
                robostack-humble \
                ros-humble-desktop \
                colcon-common-extensions \
                catkin_tools \
                rosdep \
                ros-humble-turtlebot3 \
                ros-humble-turtlebot3-gazebo \
                ros-humble-turtlebot3-teleop \
                ros-humble-example-interfaces \
                ros-humble-turtlesim \
                ros-humble-rosidl-default-generators \
                ros-humble-rosidl-adapter \
                ros-humble-rosidl-typesupport-c \
                ros-humble-rosidl-typesupport-cpp \
                ros-humble-rosidl-typesupport-interface \
                ros-humble-rosidl-typesupport-introspection-c \
                ros-humble-rosidl-typesupport-introspection-cpp \
                pygame \
                $@
            '')

            (pkgs.writeShellScriptBin "ros-update" ''
              #!/usr/bin/env bash
              micromamba update -y --all -r "/opt/micromamba" -n ros_env
            '')
          ];

          shellHook = ''
            #!/usr/bin/env bash
            if [ ! -d "/opt/micromamba" ]; then
                sudo mkdir -p /opt/micromamba
                sudo chown $USER /opt/micromamba
            fi

            if micromamba env list | grep -q "ros_env"; then
              ros-install -q

              source "/opt/micromamba/envs/ros_env/setup.bash"
            else
              micromamba create -y -r "/opt/micromamba" -n ros_env -q
              ros-install

              source "/opt/micromamba/envs/ros_env/setup.bash"
            fi

            export DYLD_FALLBACK_LIBRARY_PATH="/opt/micromamba/envs/ros_env/lib:$DYLD_FALLBACK_LIBRARY_PATH"
            export CMAKE_PREFIX_PATH=/opt/micromamba/envs/ros_env:$CMAKE_PREFIX_PATH
            export COLCON_DEFAULTS_FILE=${colconDefaults}

            if [ ! -d "culling_games" ]; then
             git clone https://github.com/rmnicola/culling_games.git
            fi

            cd culling_games
            colcon build &> /dev/null
            source install/setup.bash
            cd ..
          '';
        };
      }
    );
}
