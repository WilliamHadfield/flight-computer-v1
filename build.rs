fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic"); // disables page allignment of sections.
    println!("cargo:rustc-link-arg-bins=-Tlink.x"); // tells the linker to use cortex-m-rts linker script this is the script that makes this whole project work lol
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x"); // adds defmt linker section so i can use defmt logging eg embedded println statements or stuff to the console
    
}
