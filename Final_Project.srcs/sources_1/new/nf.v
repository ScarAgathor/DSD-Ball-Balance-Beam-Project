`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////////
//// Company: UTSA
//// Engineer: Ekong
////////////////////////////////////////////////////////////////////////////////////

module BallBeamTop(clk100, seg, DP, disp, Trig, Echo, PWM, btnc, sw);
input wire clk100;
output wire [6:0] seg;
output wire DP;
output wire [7:0] disp;
output wire Trig;
input wire Echo;
output wire PWM;
input wire btnc;
input wire [7:0] sw;

wire [31:0] echo_period;
wire [15:0] ball_distance;
wire [19:0] servo_input_raw;
wire [15:0] target_distance;
wire set_mode;

ping_sensor reader(clk100, Trig, Echo, echo_period);
distanceCalc ruler(echo_period, ball_distance);
Controller control(clk100, ball_distance, servo_input_raw, sw, btnc, target_distance, set_mode);
PWM_Gen servo (clk100, servo_input_raw, PWM); 
SSEG display (clk100, seg, disp, DP, ball_distance, target_distance, set_mode);

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

assign distance_in_cm = pulse_time / 32'd360;

endmodule

module Controller (clk, ball_dist, servo_input, sw, btn, target_dist, mode); 
    input wire clk;
    input wire [15:0] ball_dist;
    input wire [7:0] sw;
    input wire btn;
    output reg  [19:0] servo_input;
    output wire [15:0] target_dist;
    output wire mode;
    
    reg [15:0] target_reg = 16'd380;  // default target distance of 38.0 cm
    reg mode_reg = 1'b0; // 0 is normal, 1 is set target mode
    reg btn_prev  = 1'b0; //previous btnc state
    
    wire [15:0] target_for_control = target_reg + 16'd5;
    
    wire btn_pulse = btn & ~btn_prev;
    
    wire [3:0] sw_tens = sw[7:4]; // tens
    wire [3:0] sw_ones = sw[3:0]; // ones

    wire [15:0] sw_distance_in_tenths_raw = (sw_tens * 16'd100) + (sw_ones * 16'd10); // e.g., if sw = 0x25, then distance
                                                                                  // 25.0 cm => 250   
    localparam [15:0] MIN_TARGET = 16'd23;   // 2.3 cm
    localparam [15:0] MAX_TARGET = 16'd550;  // 55.0 cm
    
    wire [15:0] sw_dist_tenths =
        (sw_distance_in_tenths_raw < MIN_TARGET) ? MIN_TARGET :
        (sw_distance_in_tenths_raw > MAX_TARGET) ? MAX_TARGET :
        sw_distance_in_tenths_raw;
    
    always @(posedge clk) begin
        btn_prev <= btn;

        if (btn_pulse) begin
            if (!mode) begin
                mode_reg <= 1'b1; // target can be set
            end else begin
                mode_reg <= 1'b0; // target cannot be set
                target_reg <= sw_dist_tenths;
            end
        end
    end
    
    assign target_dist = target_reg;
    assign mode = mode_reg;
    
    localparam [31:0] SERVO_MIN = 20'd100000; //1ms
    localparam [31:0] SERVO_MAX = 20'd200000; //2ms
    localparam [31:0] SERVO_MID = 20'd144000;
    
    ////////////For the smoothing problem////////////////
    
    // Gains: coarse (far) vs fine (near)
    localparam integer KP_COARSE = 100;   // ticks per 0.1 cm when far
    localparam integer KP_FINE = 200;  // ticks per 0.1 cm when near
    localparam [15:0] ERROR_THRESHOLD = 16'd10; // 1.0 cm threshold
    localparam [15:0] DEADBAND = 16'd2;  // ±0.2 cm deadband

    // Smoothing factor: 1/(2^SMOOTH_SHIFT)
    localparam integer SMOOTH_SHIFT = 2;  // 1/4 smoothing
    
    /////////////For the smooting problem/////////////////
    
    wire signed [15:0] error_raw = $signed(target_for_control) - $signed(ball_dist);
    wire [15:0] abs_error = error_raw[15] ? -error_raw : error_raw;
    
    // Deadband around target
    wire signed [15:0] error_deadband = (abs_error < DEADBAND) ? 16'sd0 : error_raw; //16'sd0 is how signed values are written in this format in verilog
        
    wire signed [31:0] servo_offset = (abs_error > ERROR_THRESHOLD) ? (error_deadband * KP_COARSE) : (error_deadband * KP_FINE);
    
    wire signed [31:0] servo_raw = SERVO_MID + servo_offset;
    
    //clamp target to max and min of servo
    wire signed [31:0] servo_target_clamped = (servo_raw < SERVO_MIN) ? SERVO_MIN : (servo_raw > SERVO_MAX) ? SERVO_MAX : servo_raw;

    // Smoothed servo command
    reg signed [31:0] servo_smooth = SERVO_MID;
    
     // Low-pass filter: new = old + (target - old)/2^N
    always @(posedge clk) begin
        servo_smooth <= servo_smooth + ((servo_target_clamped - servo_smooth) >>> SMOOTH_SHIFT);
    end
    
    // Final clamp + take 20 bits for PWM
    always @(posedge clk) begin
        if (servo_smooth < SERVO_MIN)
            servo_input <= SERVO_MIN[19:0];
        else if (servo_smooth > SERVO_MAX)
            servo_input <= SERVO_MAX[19:0];
        else
            servo_input <= servo_smooth[19:0];
    end
  
endmodule

module PWM_Gen(clk, Input, PWM_Pulse);
input wire clk;
input wire [19:0] Input;
output reg PWM_Pulse;

reg [19:0] Count;
wire [19:0] RInput;

initial begin
    Count = 0;
    PWM_Pulse = 0;
end

always@(posedge clk) begin
    Count <= Count + 20'd1;
    if(Count == 20'd600_000) Count <= 20'd0; // clamp counter to a 6ms period
end

//Limits PWM output to range of 1ms to 2ms = useful range
assign RInput = (Input < 20'd100_000)? 20'd100_000:((Input > 20'd200_000)? 20'd200_000:Input);

always@(*) begin //send out a pwm pulse as long as Rinput is greater than the current count, but stop once count passes 6ms
    if(RInput > Count) PWM_Pulse = 1;
    else PWM_Pulse = 0;
end

endmodule
 
module SSEG (clk, seg, disp, DP, distance, target, mode);
input wire clk;
output wire [6:0] seg;
output reg [7:0] disp;
output reg DP;
input wire [15:0] distance; //distance is in 0.1 cm units example 634 will be 63.4
input wire [15:0] target;
input wire mode;

//For ball distance
wire [15:0] whole_cm = distance / 10;  // 63 
wire [3:0] tenths = distance % 10;  // .4
wire [3:0] ones_cm = whole_cm % 10;  // 3
wire [3:0] tens_cm = whole_cm / 10;  // 6

//For target distance
wire [15:0] whole_cm_target = target / 10;
wire [3:0]  tenths_target = target % 10;
wire [3:0]  ones_target = whole_cm_target % 10;
wire [3:0]  tens_target = whole_cm_target / 10;

reg [2:0] Location = 3'd0;
reg [3:0] Digit;

initial begin
    Location = 3'd0;
    Digit = 4'd0;
end

wire Clk_Multi;
CLK100MHZ_divider divider(clk, Clk_Multi);

always@(posedge Clk_Multi) begin
    Location <= Location + 3'd1;
    case(Location)
    //Lower digits are for the measured distance
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
        // Upper digits are fir target distance
        3'd4: begin
            disp <= 8'b11101111;
            Digit <= tenths_target;
            // Light DP on target side when in set_mode to give a visual cue
            DP <= mode ? 1'b0 : 1'b1;
        end
        3'd5: begin
            disp <= 8'b11011111;
            Digit <= ones_target;
            DP <= 1'b0;
        end
        3'd6: begin
            disp <= 8'b10111111;
            Digit <= tens_target;
            DP <= 1'b1;
        end
        3'd7: begin
            disp <= 8'b01111111;
            Digit <= 4'd0;
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



