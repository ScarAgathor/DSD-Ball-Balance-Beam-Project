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

assign distance_in_cm = pulse_time / 32'd580;

endmodule

module Controller (clk, ball_dist, servo_input, sw, btn, target_dist, mode); 
    input wire clk;
    input wire [15:0] ball_dist;
    input wire [7:0] sw;
    input wire btn;
    output reg  [19:0] servo_input;
    output wire [15:0] target_dist;
    output wire mode;
    
    reg [15:0] target_reg = 16'd320;  // default target distance of 32.0 cm
    reg mode_reg = 1'b0; // 0 is normal, 1 is set target mode
    reg btn_prev = 1'b0; //previous btnc state
    
    wire btn_pulse = btn & ~btn_prev;
   
    wire [3:0] sw_tens = sw[7:4]; // tens
    wire [3:0] sw_ones = sw[3:0]; // ones 
    
    
    //represents switches as distance XX.0 cm in tenths, 32.0 is 320
    wire [15:0] sw_distance_in_tenths_raw = (sw_tens * 16'd100) + (sw_ones * 16'd10);
    
    localparam [15:0] MIN_TARGET = 16'd30;   // 3.0 cm
    localparam [15:0] MAX_TARGET = 16'd550;  // 55.0 cm
    
    //clamps user target distance to the physical limits of the beam and ping sensor
    wire [15:0] sw_dist_tenths =
        (sw_distance_in_tenths_raw < MIN_TARGET) ? MIN_TARGET :
        (sw_distance_in_tenths_raw > MAX_TARGET) ? MAX_TARGET :
        sw_distance_in_tenths_raw;
    
    //This is how we toggle set mode
    always @(posedge clk) begin
        btn_prev <= btn;
        
        if (btn_pulse) begin
            if (!mode_reg) begin
                // Enter set-target mode
                mode_reg <= 1'b1;
            end else begin
                // Exit set-target mode and set new target
                mode_reg   <= 1'b0;
                target_reg <= sw_dist_tenths;
            end
        end
    end
    
    assign mode = mode_reg; //send this sends this state to the sseg display
    assign target_dist = target_reg; //sets the target
    
    //servo timing constants
    localparam [31:0] SERVO_MIN = 20'd100_000; //1ms
    localparam [31:0] SERVO_MAX = 20'd200_000; //2ms
    localparam [31:0] SERVO_MID = 20'd144_000; //got this through trial and error
    
    //For the PI part of the controller
    localparam integer Kp = 70; // ticks per 0.1 cm (for proportional gain with error)
    localparam integer Ki = 3; // integral gain (for every control update)
    localparam integer Kd = 10; // small derivative gain
    
    localparam [15:0] DEADBAND = 16'd5; // ±0.5 cm deadband
    localparam [15:0] ERROR_CLAMP = 16'd100; // limit error used for integral to about 10 cm
    localparam integer SMOOTH_SHIFT = 5; // 1/32 smoothing on servo command
    
    //control clock to update integral value
    reg [19:0] ctrl_div = 20'd0;
    reg ctrl_tick = 1'b0;

    always @(posedge clk) begin
        // 100 MHz / 600000 ≈ 166.7 Hz (~6 ms)
        if (ctrl_div == 20'd599_999) begin
            ctrl_div  <= 20'd0;
            ctrl_tick <= 1'b1;
        end else begin
            ctrl_div  <= ctrl_div + 20'd1;
            ctrl_tick <= 1'b0;
        end
    end
    
    localparam [15:0] P_ERROR_CLAMP = 16'd50; //trying to slow down the ball by stopping tilting

        //to ignore super tiny errors for I
    localparam [15:0] I_DEADBAND   = 16'd3;   // ±0.4 cm: ignore super tiny errors for I
    localparam [15:0] I_ACTIVE_MAX = 16'd80;  // 3.0 cm: only integrate when close
    
    //For calculating the error
    wire signed [15:0] error_raw = $signed(target_reg) - $signed(ball_dist); //get raw error value
    wire [15:0] abs_error = error_raw[15] ? (~error_raw + 16'd1) : error_raw; //get absolute value of error for checks
    
    wire signed [15:0] error_deadband = (abs_error < DEADBAND) ? 16'sd0 : error_raw; //apply deadband so beam doesn't twitch when the ball is really close to the target
    
    wire signed [15:0] error_I_window = (abs_error < I_DEADBAND) ? 16'sd0 : error_raw;
//    wire signed [15:0] error_for_PD = (abs_error > P_ERROR_CLAMP) ? (error_deadband[15] ? -P_ERROR_CLAMP : P_ERROR_CLAMP) : error_deadband;
    
    wire signed [15:0] error_for_integral = (abs_error > ERROR_CLAMP) ? (error_raw[15] ? -ERROR_CLAMP : ERROR_CLAMP) : error_I_window; // clamp the error used for the integral to avoid massive jumps
    
   // Integral accumulator
    reg signed [31:0] I_accum = 32'sd0;
    
    // Anti-windup limits for integral accumulator
    localparam signed [31:0] I_MAX = 32'sd28000;
    localparam signed [31:0] I_MIN = -32'sd28000;
    
    // Hold the current error used for P at the control rate
    reg signed [15:0] error_z = 16'sd0;
    reg signed [15:0] error_prev = 16'sd0; // previous error value for derivative
    

    always @(posedge clk) begin
        if (ctrl_tick) begin
            // get error at control rate
            //store error in error previous as well
            error_prev <= error_z;
            error_z <= error_deadband;

            // Integral only active near target
            if (abs_error < I_ACTIVE_MAX) begin
                I_accum <= I_accum + error_for_integral;
                
                // anti-windup clamp i_accum to prevent wind up
                if (I_accum > I_MAX)
                    I_accum <= I_MAX;
                else if (I_accum < I_MIN)
                    I_accum <= I_MIN;
            end
        end
    end
    
    wire signed [15:0] error_diff = error_z - error_prev;

    
    // Proportional term
    wire signed [31:0] P_term = error_z * Kp;

    // Integral term
    wire signed [31:0] I_term = I_accum * Ki;
    
    // Derivative Term
    wire signed [31:0] D_term = error_diff * Kd;
    
    // Raw PI output around mid pulse
    wire signed [31:0] servo_pi_raw = $signed(SERVO_MID) + P_term + I_term + D_term;
    
    // Clamp to valid servo pulse range
    wire signed [31:0] servo_target_clamped = (servo_pi_raw < $signed(SERVO_MIN)) ? $signed(SERVO_MIN) : (servo_pi_raw > $signed(SERVO_MAX)) ? $signed(SERVO_MAX) : servo_pi_raw;
    
    reg signed [31:0] smoothed_servo = SERVO_MID;
    //smooth servo using a low pass filter: new = old + (target - old)/2^N, where N is smoothing constant
    always @(posedge clk) begin
        smoothed_servo <= smoothed_servo + ((servo_target_clamped - smoothed_servo) >>> SMOOTH_SHIFT);
    end
    
   // clamp and reduce to 20 bits for PWM Gen module
    always @(posedge clk) begin
        if (smoothed_servo < $signed(SERVO_MIN))
            servo_input <= SERVO_MIN[19:0];
        else if (smoothed_servo > $signed(SERVO_MAX))
            servo_input <= SERVO_MAX[19:0];
        else
            servo_input <= smoothed_servo[19:0];
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

reg [2:0] Location;
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
        // Upper digits are for target distance
        3'd4: begin
            disp <= 8'b11101111;
            Digit <= tenths_target;
            // Lights DP on target side when in set_mode
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