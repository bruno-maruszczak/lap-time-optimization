#================IMPORTANT==================
#  Don't use the shell on non NixOS systems
#===========================================

with import <nixpkgs> {};

mkShell {

  nativeBuildInputs = with pkgs; [
    (python311.withPackages (pp: with pp; [
            pip
          ]))
    texliveBasic
    texlivePackages.type1cm
    texlivePackages.dvipng
  ];
  NIX_LD = lib.fileContents "${stdenv.cc}/nix-support/dynamic-linker";

  NIX_LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath [
    stdenv.cc.cc
    zlib
  ];

  # If you want to use the shell on distributions other than NixOS
  # remove the line `export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH`
  shellHook = ''
    export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH
    if [ ! -d "./venv" ]; then
      python -m venv ./venv
    fi
    source ./venv/bin/activate
    pip install -r ./requirements.txt
  '';



}


