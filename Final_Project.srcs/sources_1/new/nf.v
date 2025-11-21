`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////////
//// Company: UTSA
//// Engineer: Ekong
////////////////////////////////////////////////////////////////////////////////////


module BallBeamTop(clk100, seg, DP, disp, Trig, Echo);
input wire clk100;
input wire [15:0] sw;
output wire [6:0] seg;
output wire DP;
output wire [7:0] disp;
input wire Echo;
output wire Trig;
//output PWM;

wire [31:0] echo_period;
wire [15:0] ball_distance;

//PID_Controller controller()
//PWM_Gen IN105 (clk100,{sw,4'b0000}, PWM); //Replace the concatination with the PID output
ping_sensor reader(clk100, Trig, Echo, echo_period);
distanceCalc ruler(echo_period, ball_distance);
SSEG display (clk100, seg, disp, DP, ball_distance);

endmodule

module ping_sensor(clk, trig_out, echo_in, echo_period);
input wire clk;
input wire echo_in;
output reg trig_out;
output reg [31:0] echo_period; 

reg [31:0] counter;
reg [31:0] echo_pulse_time;
reg [1:0] State;

initial begin
    State           = 2'd0;
    counter         = 32'd0;
    trig_out        = 1'b0;
    echo_pulse_time = 32'd0;
    echo_period     = 32'd0;
end

always @(posedge clk) begin //This holds the state machine for the project tick "posedge
    counter <= counter + 32'd1;
    case (State) // 3 states
         2'd0: begin //Initilize Trig
            trig_out        <= 1'b0; //Reset the trigger signal
            echo_pulse_time <= 32'd0; //Reset the measured echo time
            counter         <= 32'd0; // restart timing
            State           <= 2'd1; //Move to measurement state
        end 
        2'd1: begin
            if (counter < 32'd1_000) begin //for 10us, send out trig pulse
                trig_out <= 1'b1;
                echo_pulse_time <= 32'd0;
            end
            else if ((counter >= 32'd1_000) && (counter < 32'd1_001_000)) begin //after 10us read amount of time echo is high
                trig_out <= 1'b0;
                if(echo_in == 1) 
                    echo_pulse_time <= echo_pulse_time +32'd1;
            end
            else begin
                echo_period <= echo_pulse_time;
                State <= 2'd2;//go to final state
            end 
        end
        2'd2: begin
            State <= 2'd0; //go to reset state
        end
        default: State <= 2'd0;
    endcase 
end

endmodule

module distanceCalc(pulse_time, distance_in_cm);
input wire [31:0] pulse_time;
output wire [15:0] distance_in_cm;

assign distance_in_cm = pulse_time / 32'd580;

endmodule

module SSEG (clk, seg, disp, DP, distance);
input wire clk;
output wire [6:0] seg;
output reg [7:0] disp;
output reg DP;
input wire [15:0] distance; //distance is in 0.1 cm units example 634 will be 63.4

wire [15:0] whole_cm = distance / 10;  // 63 
wire [3:0] tenths = distance % 10;  // .4
wire [3:0] ones_cm = whole_cm % 10;  // 3
wire [3:0] tens_cm = whole_cm / 10;  // 6

reg [1:0] Location = 0;
reg [3:0] Digit;

initial begin
    Location = 0;
    Digit = 0;
end

wire Clk_Multi;
CLK100MHZ_divider divider(clk, Clk_Multi);

always@(posedge Clk_Multi) begin
    Location <= Location + 1;
    case(Location)
        0: begin
            disp <= 8'b11111110;
            Digit <= tenths;
            DP <= 1'b1;
        end
        1: begin
            disp <= 8'b11111101;
            Digit <= ones_cm; 
            DP <= 1'b0;
        end
        2: begin
            disp <= 8'b11111011;
            Digit <= tens_cm;
            DP <= 1'b1;
        end
        3: begin
            disp <= 8'b11110111;
            Digit <= 0;
            DP <= 1'b1;
        end
    endcase
end

seg_decoder decoder(Digit, seg);
endmodule

module seg_decoder(val, seg);
input wire [3:0] val;
output reg [6:0] seg;

always @* begin
    case (val)
        4'b0000: seg[6:0] = 7'b1000000; // Make 0 with LEDs
        4'b0001: seg[6:0] = 7'b1001111; // Make 1 with LEDs
        4'b0010: seg[6:0] = 7'b0100100; // Make 2 with LEDs
        4'b0011: seg[6:0] = 7'b0110000; // Make 3 with LEDs
        4'b0100: seg[6:0] = 7'b0011001; // Make 4 with LEDs
        4'b0101: seg[6:0] = 7'b0010010; // Make 5 with LEDs
        4'b0110: seg[6:0] = 7'b0000010; // Make 6 with LEDs
        4'b0111: seg[6:0] = 7'b1111000; // Make 7 with LEDs
        4'b1000: seg[6:0] = 7'b0000000; // Make 8 with LEDs
        4'b1001: seg[6:0] = 7'b0011000; // Make 9 with LEDs
        4'b1010: seg[6:0] = 7'b0001000; // Make A with LEDs
        4'b1011: seg[6:0] = 7'b0000011; // Make b with LEDs
        4'b1100: seg[6:0] = 7'b1000110; // Make C with LEDs
        4'b1101: seg[6:0] = 7'b0100001; // Make d with LEDs
        4'b1110: seg[6:0] = 7'b0000110; // Make E with LEDs
        4'b1111: seg[6:0] = 7'b0001110; // Make F with LEDs
    endcase
end
endmodule

module CLK100MHZ_divider(clk100, New_Clock);//New_Clock = 100M/10,000 = 10KHz
input wire clk100; // Input clock signal
output reg New_Clock; // Divided clock output

reg [31:0] count = 32'b00000000; // [31:0] is large enough to hold any value

initial begin
    New_Clock = 0;
    count = 0;
end

always @(posedge clk100) begin
    count <= count + 1; // Increment count
    if (count == 31'd10000) begin 
        New_Clock <= ~New_Clock; // Toggle New Clock
        count <= 31'b0; // Reset count
    end
end
endmodule


//// ***********************************************************
//module PWM_Gen(clk100, Input, PWM_Pulse);
//input clk100;
//input [63:0] Input;
//output reg PWM_Pulse;

//reg [63:0] Count;
//// reg [63:0] RInput;

//wire [63:0] RInput;
//initial begin
//    Count = 0;
//    PWM_Pulse = 0;
//end

//always@(posedge clk100) begin
//    Count <= Count + 64'd1;
//    if(Count == 64'd600_000) Count <= 64'd0000000; // ***Changed to 6ms period
//end

////Limits PWM output to range of 1ms to 2ms = useful range
//assign RInput = (Input < 64'd100_000)? 64'd100_000:((Input > 64'd200_000)? 64'd200_000:Input);

//always@(*) begin
//    if(RInput > Count) PWM_Pulse = 1;
//    else PWM_Pulse = 0;
//end

//endmodule
 

//// ************************************************************************

// ***********************************************************

