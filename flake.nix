{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };

  outputs =
    inputs@{
      flake-parts,
      systems,
      devshell,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;
      imports = [
        devshell.flakeModule
      ];
      perSystem =
        { pkgs, ... }:
        let
          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                - -Wno-dev
                ${pkgs.lib.optionalString pkgs.stdenv.isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';
        in
        {
          devshells.default = {
            env = [
              {
                name = "COLCON_DEFAULTS_FILE";
                value = builtins.toString colconDefaults;
              }
            ];
            devshell = {
              packages = with pkgs; [
                pixi
              ];
              startup = {
                activate-ros.text = ''
                  if [ -f pixi.toml ]; then
                    ${pkgs.lib.optionalString pkgs.stdenv.isDarwin ''
                      export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                    ''}
                    eval "$(pixi shell-hook)";
                  fi
                '';
                clone-culling-games = {
                  text = ''
                    if [ ! -d "culling_games" ]; then
                      echo "cloning cg repo..."
                      git clone https://github.com/rmnicola/culling_games.git
                    fi
                  '';
                };
                build-and-source-culling-games = {
                  text = ''
                    cd culling_games

                    if [ ! -d "build" ]; then
                      echo "running colcon build..."
                      colcon build
                    fi

                    source install/setup.bash

                    cd ..
                  '';
                  deps = [ "clone-culling-games" ];
                };
              };
              motd = "";
            };
          };
        };
    };
}
