with import <nixpkgs> {};

mkShell {

  nativeBuildInputs = with pkgs; [
    (python311.withPackages (pp: with pp; [
            pip
          ]))

  ];
  NIX_LD = lib.fileContents "${stdenv.cc}/nix-support/dynamic-linker";

  NIX_LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath [
    stdenv.cc.cc
    zlib
  ];


  shellHook = ''
    if [ ! -d "./venv" ]; then
      python -m venv ./venv
    fi
    source ./venv/bin/activate
    pip install -r ./requirements.txt
  '';



}


