$(document).ready(function() {
    "use strict";

    const TIMER_STATE = { NONE: 0, START: 1, STOP: 2, RESET: 3 };
    const INTERVAL = 1000;
    
    let start_button = document.querySelector("#start");
    let stop_button = document.querySelector("#stop");
    let reset_button = document.querySelector("#reset");
    let time_panel = document.getElementById("time_panel");
    let sample = document.getElementById("sample");
    let timerStr = "";
    let timer = { sec: 0, min: 0, hour: 0 };
    let state = TIMER_STATE.NONE;
    
    time_panel.innerHTML = "00:00:00";
    
    start_button.onclick = function() {
	if (TIMER_STATE.NONE == state || TIMER_STATE.STOP == state) {	    
	    timerStr = setInterval(update, INTERVAL);
	    start_button.innerHTML = "Resume";

	    state = TIMER_STATE.START;
	}
    }

    stop_button.onclick = function() {
	if (TIMER_STATE.START == state || TIMER_STATE.RESET == state) {
	    sample.innerHTML = " - " + time_panel.innerHTML + " - "; 
	    clearInterval(timerStr);
	    
	    state = TIMER_STATE.STOP;
	}
    }

    reset_button.onclick = function() {
	if (TIMER_STATE.NONE != state) {
	    timer.sec = 0;
	    timer.min = 0;
	    timer.hour = 0;

	    sample.innerHTML = "";
	    time_panel.innerHTML = "00:00:00";

	    clearInterval(timerStr);
	    timerStr = setInterval(update, INTERVAL);
	    state = TIMER_STATE.RESET;
	}
    }

    function update() {	
	timer.sec += 1;
	
	if (timer.sec == 60) {
	    timer.min += 1;
	    timer.sec = 0;
	}

	if (timer.min == 60) {
	    timer.hour += 1;
	    timer.min = 0;
	}
	
	time_panel.innerHTML = (timer.hour < 10 ? "0" + timer.hour : "" + timer.hour) + ":" +
	    (timer.min < 10 ? "0" + timer.min : "" + timer.min) + ":" +
	    (timer.sec < 10 ? "0" + timer.sec : "" + timer.sec);
    }
});
