//------------------------------------------------------------------------------
//
// ecdsa256_base_point_multiplier.v
// -----------------------------------------------------------------------------
// ECDSA base point scalar multiplier.
//
// Authors: Pavel Shatov
//
// Copyright (c) 2016, 2018 NORDUnet A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// - Neither the name of the NORDUnet nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//------------------------------------------------------------------------------

module ecdsa256_base_point_multiplier
(
    clk, rst_n,
    ena, rdy,
    k_addr, rxy_addr,
    rx_wren, ry_wren,
    k_din,
    rxy_dout
);


    //
    // Microcode Header
    //
`include "ecdsa_uop.vh"
    

    //
    // Ports
    //
    input           clk;        // system clock
    input           rst_n;      // active-low async reset

    input           ena;        // enable input
    output          rdy;        // ready output

    output  [ 2:0]  k_addr;     //
    output  [ 2:0]  rxy_addr;   //
    output          rx_wren;    //
    output          ry_wren;    //
    input   [31:0]  k_din;      //
    output  [31:0]  rxy_dout;   //


    //
    // FSM
    //
    localparam [4:0] FSM_STATE_IDLE                 = 5'd00;
    localparam [4:0] FSM_STATE_PREPARE_TRIG         = 5'd01;
    localparam [4:0] FSM_STATE_PREPARE_WAIT         = 5'd02;
    localparam [4:0] FSM_STATE_CYCLE_DBL_TRIG       = 5'd03;
    localparam [4:0] FSM_STATE_CYCLE_DBL_WAIT       = 5'd04;
    localparam [4:0] FSM_STATE_CYCLE_ADD_TRIG       = 5'd05;
    localparam [4:0] FSM_STATE_CYCLE_ADD_WAIT       = 5'd06;
    localparam [4:0] FSM_STATE_CYCLE_ADD_EXTRA_TRIG = 5'd07;
    localparam [4:0] FSM_STATE_CYCLE_ADD_EXTRA_WAIT = 5'd08;
    localparam [4:0] FSM_STATE_AFTER_CYCLE_TRIG     = 5'd09;
    localparam [4:0] FSM_STATE_AFTER_CYCLE_WAIT     = 5'd10;
    localparam [4:0] FSM_STATE_INVERT_TRIG          = 5'd11;
    localparam [4:0] FSM_STATE_INVERT_WAIT          = 5'd12;
    localparam [4:0] FSM_STATE_CONVERT_TRIG         = 5'd13;
    localparam [4:0] FSM_STATE_CONVERT_WAIT         = 5'd14;
    localparam [4:0] FSM_STATE_CONVERT_EXTRA_TRIG   = 5'd15;
    localparam [4:0] FSM_STATE_CONVERT_EXTRA_WAIT   = 5'd16;
    localparam [4:0] FSM_STATE_DONE                 = 5'd17;

    reg [4:0] fsm_state = FSM_STATE_IDLE;
    reg [4:0] fsm_state_next;


    //
    // Round Counter
    //
    reg  [7:0] bit_counter;
    wire [7:0] bit_counter_last = 8'hFF;    // 255
    wire [7:0] bit_counter_zero = 8'h00;    // 0
    wire [7:0] bit_counter_prev =
        (bit_counter > bit_counter_zero) ? bit_counter - 1'b1 : bit_counter_last;

    assign k_addr = bit_counter[7:5];


    //
    // Worker Trigger Logic
    //
    reg  worker_trig = 1'b0;
    wire worker_done;

    wire fsm_wait_done = !worker_trig && worker_done;

    always @(posedge clk or negedge rst_n)
        //
        if (rst_n == 1'b0)                      worker_trig <= 1'b0;
        else case (fsm_state)
            FSM_STATE_PREPARE_TRIG,
            FSM_STATE_CYCLE_DBL_TRIG,
            FSM_STATE_CYCLE_ADD_TRIG,
            FSM_STATE_CYCLE_ADD_EXTRA_TRIG,
            FSM_STATE_AFTER_CYCLE_TRIG,
            FSM_STATE_INVERT_TRIG,
            FSM_STATE_CONVERT_TRIG,
            FSM_STATE_CONVERT_EXTRA_TRIG:       worker_trig <= 1'b1;
            default:                            worker_trig <= 1'b0;
        endcase
        
        
    //
    // Round Counter Increment Logic
    //
    always @(posedge clk)
        //
        case (fsm_state_next)
            FSM_STATE_PREPARE_TRIG:         bit_counter <= bit_counter_last;
            FSM_STATE_AFTER_CYCLE_TRIG:     bit_counter <= bit_counter_prev;
        endcase


    //
    // Final Cycle Detection Logic
    //
    wire [ 3: 0] fsm_state_after_cycle = (bit_counter == bit_counter_last) ?
        FSM_STATE_INVERT_TRIG : FSM_STATE_CYCLE_DBL_TRIG;
        

    //
    // K Latch
    //
    reg [31:0] k_din_shreg;
    
    wire [4:0] k_bit_index = bit_counter[4:0];
    
    always @(posedge clk)
        //
        if (fsm_state_next == FSM_STATE_CYCLE_DBL_TRIG)
            //
            if (k_bit_index == 5'd31) k_din_shreg <= k_din;
            else                      k_din_shreg <= {k_din_shreg[30:0], ~k_din_shreg[31]};
    

    //
    // Worker Flags
    //
    wire worker_flagz_sz;
    wire worker_flagz_rz;
    wire worker_flagz_e;
    wire worker_flagz_f;
    
    wire [2:0] worker_flagz_cycle_add =
        {worker_flagz_sz, worker_flagz_e,  worker_flagz_f};

    wire worker_flagz_convert_extra =
        worker_flagz_rz;

    
    //
    // Worker Offset Logic
    //
    reg [UOP_ADDR_WIDTH-1:0] worker_offset;
    
    always @(posedge clk)
        //
        case (fsm_state)
        
            FSM_STATE_PREPARE_TRIG:         worker_offset <= UOP_OFFSET_PREPARE;
            
            FSM_STATE_CYCLE_DBL_TRIG:       worker_offset <= UOP_OFFSET_CYCLE_DOUBLE;
            FSM_STATE_CYCLE_ADD_TRIG:       worker_offset <= UOP_OFFSET_CYCLE_ADD;
            
            FSM_STATE_CYCLE_ADD_EXTRA_TRIG:
                // {sz, e, f}
                casez(worker_flagz_cycle_add)
                    3'b1??:                 worker_offset <= UOP_OFFSET_CYCLE_ADD_AT_INFINITY;
                    3'b011:                 worker_offset <= UOP_OFFSET_CYCLE_ADD_SAME_X_SAME_Y;
                    3'b010:                 worker_offset <= UOP_OFFSET_CYCLE_ADD_SAME_X;
                    3'b00?:                 worker_offset <= UOP_OFFSET_CYCLE_ADD_REGULAR;
                endcase
                
            FSM_STATE_AFTER_CYCLE_TRIG:     worker_offset <= k_din_shreg[31] ?
                                            UOP_OFFSET_CYCLE_K1 : UOP_OFFSET_CYCLE_K0;
                                            
            FSM_STATE_INVERT_TRIG:          worker_offset <= UOP_OFFSET_INVERT;
            FSM_STATE_CONVERT_TRIG:         worker_offset <= UOP_OFFSET_CONVERT;
            
            FSM_STATE_CONVERT_EXTRA_TRIG:   worker_offset <= worker_flagz_convert_extra ?
                                            UOP_OFFSET_CONVERT_AT_INFINITY : UOP_OFFSET_CONVERT_REGULAR;
            
            default:                        worker_offset <= {UOP_ADDR_WIDTH{1'bX}};
            
        endcase
            

    //
    // FSM Process
    //
    always @(posedge clk or negedge rst_n)
        //
        if (rst_n == 1'b0)  fsm_state <= FSM_STATE_IDLE;
        else                fsm_state <= fsm_state_next;


    //
    // FSM Transition Logic
    //
    always @* begin
        //
        fsm_state_next = FSM_STATE_IDLE;
        //
        case (fsm_state)

            FSM_STATE_IDLE:                 fsm_state_next = ena           ? FSM_STATE_PREPARE_TRIG         : FSM_STATE_IDLE;
            
            FSM_STATE_PREPARE_TRIG:         fsm_state_next =                 FSM_STATE_PREPARE_WAIT         ;
            FSM_STATE_PREPARE_WAIT:         fsm_state_next = fsm_wait_done ? FSM_STATE_CYCLE_DBL_TRIG       : FSM_STATE_PREPARE_WAIT;
            FSM_STATE_CYCLE_DBL_TRIG:       fsm_state_next =                 FSM_STATE_CYCLE_DBL_WAIT       ;
            FSM_STATE_CYCLE_DBL_WAIT:       fsm_state_next = fsm_wait_done ? FSM_STATE_CYCLE_ADD_TRIG       : FSM_STATE_CYCLE_DBL_WAIT;
            FSM_STATE_CYCLE_ADD_TRIG:       fsm_state_next =                 FSM_STATE_CYCLE_ADD_WAIT       ;
            FSM_STATE_CYCLE_ADD_WAIT:       fsm_state_next = fsm_wait_done ? FSM_STATE_CYCLE_ADD_EXTRA_TRIG : FSM_STATE_CYCLE_ADD_WAIT;
            FSM_STATE_CYCLE_ADD_EXTRA_TRIG: fsm_state_next =                 FSM_STATE_CYCLE_ADD_EXTRA_WAIT ;
            FSM_STATE_CYCLE_ADD_EXTRA_WAIT: fsm_state_next = fsm_wait_done ? FSM_STATE_AFTER_CYCLE_TRIG     : FSM_STATE_CYCLE_ADD_EXTRA_WAIT;
            FSM_STATE_AFTER_CYCLE_TRIG:     fsm_state_next =                 FSM_STATE_AFTER_CYCLE_WAIT     ;
            FSM_STATE_AFTER_CYCLE_WAIT:     fsm_state_next = fsm_wait_done ? fsm_state_after_cycle          : FSM_STATE_AFTER_CYCLE_WAIT;
            FSM_STATE_INVERT_TRIG:          fsm_state_next =                 FSM_STATE_INVERT_WAIT          ;
            FSM_STATE_INVERT_WAIT:          fsm_state_next = fsm_wait_done ? FSM_STATE_CONVERT_TRIG         : FSM_STATE_INVERT_WAIT;
            FSM_STATE_CONVERT_TRIG:         fsm_state_next =                 FSM_STATE_CONVERT_WAIT         ;
            FSM_STATE_CONVERT_WAIT:         fsm_state_next = fsm_wait_done ? FSM_STATE_CONVERT_EXTRA_TRIG   : FSM_STATE_CONVERT_WAIT;
            FSM_STATE_CONVERT_EXTRA_TRIG:   fsm_state_next =                 FSM_STATE_CONVERT_EXTRA_WAIT   ;
            FSM_STATE_CONVERT_EXTRA_WAIT:   fsm_state_next = fsm_wait_done ? FSM_STATE_DONE                 : FSM_STATE_CONVERT_EXTRA_WAIT;
            FSM_STATE_DONE:                 fsm_state_next =                 FSM_STATE_IDLE                 ;

        endcase
        //
    end


    //
    // Worker
    //
    wire worker_output_now = (fsm_state == FSM_STATE_CONVERT_EXTRA_WAIT);
    
    ecdsa256_uop_worker uop_worker
    (
        .clk            (clk),
        .rst_n          (rst_n),
          
        .ena            (worker_trig),
        .rdy            (worker_done),
        .uop_offset     (worker_offset),
        .output_now     (worker_output_now),
          
        .flagz_sz       (worker_flagz_sz),
        .flagz_rz       (worker_flagz_rz),
        .flagz_e        (worker_flagz_e),
        .flagz_f        (worker_flagz_f),
        
        .xy_addr        (rxy_addr),
        .xy_dout        (rxy_dout),
        .x_wren         (rx_wren),
        .y_wren         (ry_wren)
    );


    //
    // Ready Flag Logic
    //
    reg rdy_reg = 1'b1;
    assign rdy = rdy_reg;

    always @(posedge clk or negedge rst_n)
        //
        if (rst_n == 1'b0)              rdy_reg <= 1'b1;
        else case (fsm_state)
            FSM_STATE_IDLE: if (ena)    rdy_reg <= 1'b0;
            FSM_STATE_DONE:             rdy_reg <= 1'b1;
        endcase
        

endmodule


//------------------------------------------------------------------------------
// End-of-File
//------------------------------------------------------------------------------
