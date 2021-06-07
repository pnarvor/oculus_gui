
// Ticks utilities taken from 

class Tick
{
    static nice_number(value, round = false) {

    const exponent = Math.floor(Math.log10(value));
    const fraction = value / Math.pow(10, exponent);
    
    let niceFraction;
    
    if (round) {
        if (fraction < 1.5) {
          niceFraction = 1.0;
        } else if (fraction < 3.0) {
          niceFraction = 2.0;
        } else if (fraction < 7.0) {
          niceFraction = 5.0;
        } else {
          niceFraction = 10.0;
        }
    } else {
        if (fraction <= 1.0) {
          niceFraction = 1.0;
        } else if (fraction <= 2.0) {
          niceFraction = 2.0;
        } else if (fraction <= 5.0) {
          niceFraction = 5.0;
        } else {
          niceFraction = 10.0;
        }
    }
    
    return niceFraction * Math.pow(10, exponent);
}

    static nice_tick_values(minValue, maxValue, ticksCount) {

        const range = Tick.nice_number(maxValue - minValue);
        
        const tickValue = Tick.nice_number(range / (ticksCount - 1), true);
        
        let ticks = [];
        for (let i = 0; i < ticksCount; i++) {
             ticks.push(minValue + tickValue * i);
        }
        
        return ticks;
    }

    constructor(value) {
        this.value = value;
        this.label = document.createElement("div");
        this.label.classList.add("tick-label");
        this.label.innerHTML = this.value;
        console.log(this.label);
    }
};
