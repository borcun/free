// If you did intend mutation, then the solution is quite easy: add mut.

fn main() {
    // the variables( pattern ) are immutable which is defined with let.
    let x = 5;
    let (a, b) = (3, 8);
    let y: i32 = 10;

    // to define mutable variables, let mux keywords are used.
    let mux z = 3;

    z = 12;
}
