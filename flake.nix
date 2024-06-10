#{
#  description = "My Nix shell for Python project with Texlive";
#  inputs = {
#    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
#  };
#
#  outputs = { self, nixpkgs, }: {
#    devShell = nixpkgs.mkShell {
#      nativeBuildInputs = with nixpkgs.pkgs; [
#        (python311.withPackages (pp: with pp; [
#          pip
#        ]))
#        texliveBasic
#        texlivePackages.type1cm
#        texlivePackages.dvipng
#      ];
#      NIX_LD = nixpkgs.lib.fileContents "${nixpkgs.stdenv.cc}/nix-support/dynamic-linker";
#      NIX_LD_LIBRARY_PATH = with nixpkgs.pkgs; lib.makeLibraryPath [
#        stdenv.cc.cc
#      ];
#      shellHook = ''
#        export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH
#        if [ ! -d "./venv" ]; then
#          python -m venv ./venv
#        fi
#        source ./venv/bin/activate
#        pip install -r ./requirements.txt
#      '';
#    };
#  };
#}

#{
#  description = "My Nix shell for Python project with Texlive";
#  inputs = {
#    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
#  };
#
#  outputs = { self, nixpkgs }: {
#    devShell = nixpkgs.mkDerivation {
#      name = "my-shell";
#      buildInputs = with nixpkgs.pkgs; [
#        (python311.withPackages (pp: with pp; [
#          pip
#        ]))
#        texliveBasic
#        texlivePackages.type1cm
#        texlivePackages.dvipng
#      ];
#      NIX_LD = nixpkgs.lib.fileContents "${nixpkgs.stdenv.cc}/nix-support/dynamic-linker";
#      NIX_LD_LIBRARY_PATH = with nixpkgs.pkgs; lib.makeLibraryPath [
#        stdenv.cc.cc
#      ];
#      shellHook = ''
#        export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH
#        if [ ! -d "./venv" ]; then
#          python -m venv ./venv
#        fi
#        source ./venv/bin/activate
#        pip install -r ./requirements.txt
#      '';
#    };
#  };
#}
#
#
#{
#  inputs = { 
#    # (cut)
#  };
#  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
#    flake-utils.lib.eachDefaultSystem
#      (system:
#        let
#          overlays = [ (import rust-overlay) ];
#          pkgs = import nixpkgs {
#            inherit system overlays;
#          };
#          rustToolchain = pkgs.pkgsBuildHost.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
#          # new! 👇
#          nativeBuildInputs = with pkgs; [ rustToolchain pkg-config ];
#          # also new! 👇
#          buildInputs = with pkgs; [ openssl ];
#        in
#        with pkgs;
#        {
#          devShells.default = mkShell {
#            # 👇 and now we can just inherit them
#            inherit buildInputs nativeBuildInputs;
#          };
#        }
#      );
#}
