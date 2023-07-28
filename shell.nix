let 
pkgs = import <nixpkgs> { overlays = [ (import /home/ashwin/source/env-setup/nix/ashwin-nixpkgs) ] ; }; 

in
  pkgs.mkShell {
    # nativeBuildInputs is usually what you want -- tools you need to run
    nativeBuildInputs = with pkgs.buildPackages; [ 
        cmake 
        eigen
        manif
    ];
}
