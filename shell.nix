let 
  pkgs = import <nixpkgs> { }; 
  #eigen-patch = pkgs.eigen.overrideAttrs (final : prev :  { patches = []; } );
in
  pkgs.mkShell {
    # nativeBuildInputs is usually what you want -- tools you need to run
    nativeBuildInputs = with pkgs.buildPackages; [ 
        cmake 
	eigen
        manif
    ];
}
