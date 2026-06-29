use core::marker::PhantomData;
use embassy_time::Timer;


struct current_phase<State> {
state : PhantomData<State>,
}



struct idle;


struct boost;

struct coasting;

struct descent;

struct landed;
// this method of encoding flight states can ensure that you have a singleton like defintion of your flight phase.
// you will at all times only have ONE instance of the current_phase struct and therefore acts like a dyanmic tracker 
// of where roughly your rocket is at in its flight trajectory.




impl current_phase<idle> {
pub fn new() -> Self {
current_phase {
    state : PhantomData,
}

}
async fn boost(self) -> current_phase<boost> {

// boost logic goes here













    
return current_phase {
    state : PhantomData,
}
}
}

impl current_phase<boost> {

}

impl current_phase<coasting> {

}

impl current_phase<descent> {

}

impl current_phase<landed> {

}




#[embassy_executor::task]
async fn flight_statemachine() {
let start_phase = current_phase::<idle>::new();
defmt::info!("ROCKET IDLE-awaiting go,");
let boost_phase = start_phase.boost().await;
}