//------------------------------------------------------------------------------
//
// ecdsa256_uop_worker.v
// -----------------------------------------------------------------------------
// ECDSA uOP Worker for P-256
//
// Authors: Pavel Shatov
//
// Copyright (c) 2018, NORDUnet A/S
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

module ecdsa256_uop_worker
(
    clk, rst_n,
    ena, rdy,
    uop_offset,
    output_now,
    flagz_sz, flagz_rz,
    flagz_e,  flagz_f,
    xy_addr, xy_dout, x_wren, y_wren
);


    //
    // Microcode Header
    //
`include "ecdsa_uop.vh"


    //
    // Ports
    //
    input   clk;    // system clock
    input   rst_n;  // active-low async reset

    input   ena;    // enable input
    output  rdy;    // ready output
    
    input [UOP_ADDR_WIDTH-1:0] uop_offset;  // starting offset
    
    input   output_now;     // produce output    

    output  flagz_sz;   // SZ is zero
    output  flagz_rz;   // RZ is zero
    output  flagz_e;    // E is zero
    output  flagz_f;    // F is zero

    output  [ 2: 0] xy_addr;
    output  [31: 0] xy_dout;
    output          x_wren;
    output          y_wren;
    
    
    //
    // Constants
    //
    localparam integer OPERAND_NUM_WORDS = 8;   // 256 bits -> 8 x 32-bit words
    localparam integer WORD_COUNTER_WIDTH = 3;  // 0..7 -> 3 bits
    

    //
    // FSM
    //
    localparam [1:0] FSM_STATE_IDLE     = 2'b00;
    localparam [1:0] FSM_STATE_FETCH    = 2'b01;
    localparam [1:0] FSM_STATE_DECODE   = 2'b10;
    localparam [1:0] FSM_STATE_BUSY     = 2'b11;

    reg  [1:0] fsm_state = FSM_STATE_IDLE;
    reg  [1:0] fsm_state_next;

  
    //
    // Microcode
    //
    reg  [UOP_ADDR_WIDTH-1:0] uop_addr;
    wire [UOP_DATA_WIDTH-1:0] uop_data;
    
    wire [5:0] uop_data_opcode        = uop_data[1 + 3*6 +: 6];
    wire       uop_data_banks         = uop_data[0 + 3*6 +: 1];
    wire [5:0] uop_data_operand_src1  = uop_data[0 + 2*6 +: 6];
    wire [5:0] uop_data_operand_src2  = uop_data[0 + 1*6 +: 6];
    wire [5:0] uop_data_operand_dst   = uop_data[0 + 0*6 +: 6];
    
    wire uop_data_opcode_is_stop = uop_data_opcode[5];
    wire uop_data_opcode_is_mul  = uop_data_opcode[4];
    wire uop_data_opcode_is_sub  = uop_data_opcode[3];
    wire uop_data_opcode_is_add  = uop_data_opcode[2];
    wire uop_data_opcode_is_copy = uop_data_opcode[1];
    wire uop_data_opcode_is_cmpz = uop_data_opcode[0];
    
    ecdsa256_microcode_rom microcode_rom
    (
        .clk    (clk),
        .addr   (uop_addr),
        .data   (uop_data)
    );

    
    //
    // Microcode Address Increment Logic
    //
    always @(posedge clk)
        //
        if (fsm_state_next == FSM_STATE_FETCH)
            uop_addr <= (fsm_state == FSM_STATE_IDLE) ? uop_offset : uop_addr + 1'b1;


    //
    // Multi-Word Mover
    //
    reg  mw_mover_ena = 1'b0;
    wire mw_mover_rdy;
    
    wire [WORD_COUNTER_WIDTH-1:0]   mw_mover_x_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mw_mover_y_addr;
    wire [                32-1:0]   mw_mover_x_din;
    wire [                32-1:0]   mw_mover_y_dout;
    wire                            mw_mover_y_wren;
    
    multiword_mover #
    (
        .WORD_COUNTER_WIDTH     (WORD_COUNTER_WIDTH),
        .OPERAND_NUM_WORDS      (OPERAND_NUM_WORDS)
    )
    mw_mover_inst
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (mw_mover_ena),
        .rdy        (mw_mover_rdy),
        .x_addr     (mw_mover_x_addr),
        .y_addr     (mw_mover_y_addr),
        .y_wren     (mw_mover_y_wren),
        .x_din      (mw_mover_x_din),
        .y_dout     (mw_mover_y_dout)
    );


    //
    // Modular Multiplier
    //
    reg  mod_mul_ena = 1'b0;
    wire mod_mul_rdy;
    
    wire [WORD_COUNTER_WIDTH-1:0]   mod_mul_a_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_mul_b_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_mul_p_addr;
    wire [                32-1:0]   mod_mul_a_din;
    wire [                32-1:0]   mod_mul_b_din;
    wire [                32-1:0]   mod_mul_p_dout;
    wire                            mod_mul_p_wren;
    
    ecdsa256_modular_multiplier mod_mul_inst
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (mod_mul_ena),
        .rdy        (mod_mul_rdy),
        .a_addr     (mod_mul_a_addr),
        .b_addr     (mod_mul_b_addr),
        .p_addr     (mod_mul_p_addr),
        .p_wren     (mod_mul_p_wren),
        .a_din      (mod_mul_a_din),
        .b_din      (mod_mul_b_din),
        .p_dout     (mod_mul_p_dout)
    );    
    
    
    //
    // Modular Adder
    //
    reg  mod_add_ena = 1'b0;
    wire mod_add_rdy;
    
    wire [WORD_COUNTER_WIDTH-1:0]   mod_add_ab_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_add_n_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_add_s_addr;
    wire [                32-1:0]   mod_add_a_din;
    wire [                32-1:0]   mod_add_b_din;
    wire [                32-1:0]   mod_add_n_din;
    wire [                32-1:0]   mod_add_s_dout;
    wire                            mod_add_s_wren;
        
    modular_adder #
    (
        .OPERAND_NUM_WORDS(OPERAND_NUM_WORDS),
        .WORD_COUNTER_WIDTH(WORD_COUNTER_WIDTH)
    )
    mod_add_inst
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (mod_add_ena),
        .rdy        (mod_add_rdy),
        .ab_addr    (mod_add_ab_addr),
        .n_addr     (mod_add_n_addr),
        .s_addr     (mod_add_s_addr),
        .s_wren     (mod_add_s_wren),
        .a_din      (mod_add_a_din),
        .b_din      (mod_add_b_din),
        .n_din      (mod_add_n_din),
        .s_dout     (mod_add_s_dout)
    );
    
    
    //
    // Modular Subtractor
    //
    reg  mod_sub_ena = 1'b0;
    wire mod_sub_rdy;
    
    wire [WORD_COUNTER_WIDTH-1:0]   mod_sub_ab_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_sub_n_addr;
    wire [WORD_COUNTER_WIDTH-1:0]   mod_sub_d_addr;
    wire [                32-1:0]   mod_sub_a_din;
    wire [                32-1:0]   mod_sub_b_din;
    wire [                32-1:0]   mod_sub_n_din;
    wire [                32-1:0]   mod_sub_d_dout;
    wire                            mod_sub_d_wren;
        
    modular_subtractor #
    (
        .OPERAND_NUM_WORDS(OPERAND_NUM_WORDS),
        .WORD_COUNTER_WIDTH(WORD_COUNTER_WIDTH)
    )
    mod_sub_inst
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (mod_sub_ena),
        .rdy        (mod_sub_rdy),
        .ab_addr    (mod_sub_ab_addr),
        .n_addr     (mod_sub_n_addr),
        .d_addr     (mod_sub_d_addr),
        .d_wren     (mod_sub_d_wren),
        .a_din      (mod_sub_a_din),
        .b_din      (mod_sub_b_din),
        .n_din      (mod_sub_n_din),
        .d_dout     (mod_sub_d_dout)
    );
    
    
    //
    // Modulus (two instances for better placement and routing)
    //
    ecdsa256_modulus_distmem modulus_add
    (
        .clk    (clk),
        .b_addr (mod_add_n_addr),
        .b_out  (mod_add_n_din)
    );    

    ecdsa256_modulus_distmem modulus_sub
    (
        .clk    (clk),
        .b_addr (mod_sub_n_addr),
        .b_out  (mod_sub_n_din)
    );        

    
    //
    // Multi-Word Comparator
    //
    reg  mw_comp_ena = 1'b0;
    wire mw_comp_rdy;
    
    wire mw_comp_cmp_e;
    
    wire [WORD_COUNTER_WIDTH-1:0]   mw_comp_x_addr;
    wire [                32-1:0]   mw_comp_x_din;
    
    multiword_comparator #
    (
        .WORD_COUNTER_WIDTH     (WORD_COUNTER_WIDTH),
        .OPERAND_NUM_WORDS      (OPERAND_NUM_WORDS)    
    )
    mw_comp_inst
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (mw_comp_ena),
        .rdy        (mw_comp_rdy),
        .xy_addr    (mw_comp_x_addr),
        .x_din      (mw_comp_x_din),
        .y_din      ({32{1'b0}}),
        .cmp_l      (),
        .cmp_e      (mw_comp_cmp_e),
        .cmp_g      ()
    );
    
    
    //
    // Comparison Flags
    //
    reg flagz_sz_reg;
    reg flagz_rz_reg;
    reg flagz_e_reg;
    reg flagz_f_reg;
    
    assign flagz_sz = flagz_sz_reg; 
    assign flagz_rz = flagz_rz_reg;
    assign flagz_e  = flagz_e_reg;
    assign flagz_f  = flagz_f_reg;
    
    reg mw_comp_rdy_dly = 1'b1;
    
    always @(posedge clk) mw_comp_rdy_dly <= mw_comp_rdy;

    always @(posedge clk)
        //
        if (mw_comp_rdy && !mw_comp_rdy_dly)
            //
            case (uop_data_operand_src1)
                UOP_OPERAND_CYCLE_SZ: flagz_sz_reg <= mw_comp_cmp_e;
                UOP_OPERAND_CYCLE_RZ: flagz_rz_reg <= mw_comp_cmp_e;
                UOP_OPERAND_CYCLE_E:  flagz_e_reg  <= mw_comp_cmp_e;
                UOP_OPERAND_CYCLE_F:  flagz_f_reg  <= mw_comp_cmp_e;
            endcase
            
    
    
    //
    // uOP Trigger Logic
    //
    always @(posedge clk)
        //
        if (fsm_state == FSM_STATE_DECODE) begin
            mw_comp_ena     <= uop_data_opcode_is_cmpz;
            mw_mover_ena    <= uop_data_opcode_is_copy;
            mod_mul_ena     <= uop_data_opcode_is_mul;
            mod_add_ena     <= uop_data_opcode_is_add;
            mod_sub_ena     <= uop_data_opcode_is_sub;
        end else begin
            mw_comp_ena     <= 1'b0;
            mw_mover_ena    <= 1'b0;
            mod_mul_ena     <= 1'b0;
            mod_add_ena     <= 1'b0;
            mod_sub_ena     <= 1'b0;
        end

    
    //
    // uOP Completion Detector
    //
    reg fsm_exit_from_busy;
    
    always @* begin
        //
        fsm_exit_from_busy = 0;
        //
        if (uop_data_opcode_is_cmpz)    fsm_exit_from_busy = ~mw_comp_ena  & mw_comp_rdy;
        if (uop_data_opcode_is_copy)    fsm_exit_from_busy = ~mw_mover_ena & mw_mover_rdy;
        if (uop_data_opcode_is_mul)     fsm_exit_from_busy = ~mod_mul_ena  & mod_mul_rdy;
        if (uop_data_opcode_is_add)     fsm_exit_from_busy = ~mod_add_ena  & mod_add_rdy;
        if (uop_data_opcode_is_sub)     fsm_exit_from_busy = ~mod_sub_ena  & mod_sub_rdy;
        //
    end

        
        //
        // Banks
        //
    reg     [ 2:0]  banks_src1_addr;
    reg     [ 2:0]  banks_src2_addr;
    reg     [ 2:0]  banks_dst_addr;
    
    reg             banks_dst_wren;
    
    reg     [31:0]  banks_dst_din;

    wire    [31:0]  banks_src1_dout;
    wire    [31:0]  banks_src2_dout;    
    
    ecdsa256_banks_array banks_array
    (
        .clk            (clk),
    
        .banks          (uop_data_banks),
    
        .src1_operand   (uop_data_operand_src1),
        .src2_operand   (uop_data_operand_src2),
        .dst_operand    (uop_data_operand_dst),
    
        .src1_addr      (banks_src1_addr),
        .src2_addr      (banks_src2_addr),
        .dst_addr       (banks_dst_addr),
    
        .dst_wren       (banks_dst_wren),
    
        .src1_dout      (banks_src1_dout),
        .src2_dout      (banks_src2_dout),
    
        .dst_din        (banks_dst_din)
    );
    
    assign mw_comp_x_din    = banks_src1_dout;
    assign mw_mover_x_din   = banks_src1_dout;
    assign mod_mul_a_din    = banks_src1_dout;
    assign mod_mul_b_din    = banks_src2_dout;
    assign mod_add_a_din    = banks_src1_dout;
    assign mod_add_b_din    = banks_src2_dout;
    assign mod_sub_a_din    = banks_src1_dout;
    assign mod_sub_b_din    = banks_src2_dout;
    
    always @*
        //
        case (uop_data_opcode)
            //
            UOP_OPCODE_CMPZ: begin
                banks_src1_addr = mw_comp_x_addr;
                banks_src2_addr = {3{1'bX}};
                //
                banks_dst_addr  = {3{1'bX}};
                //
                banks_dst_wren  = 1'b0;
                //
                banks_dst_din   = {32{1'bX}};
            end
            //
            UOP_OPCODE_COPY: begin
                //
                banks_src1_addr = mw_mover_x_addr;
                banks_src2_addr = {3{1'bX}};
                //
                banks_dst_addr  = mw_mover_y_addr;
                //
                banks_dst_wren  = mw_mover_y_wren;
                //
                banks_dst_din   = mw_mover_y_dout;
                //
            end
            //
            UOP_OPCODE_ADD: begin
                //
                banks_src1_addr = mod_add_ab_addr;
                banks_src2_addr = mod_add_ab_addr;
                //
                banks_dst_addr  = mod_add_s_addr;
                //
                banks_dst_wren  = mod_add_s_wren;
                //
                banks_dst_din   = mod_add_s_dout;
                //                
            end
            //
            UOP_OPCODE_SUB: begin
                //
                banks_src1_addr = mod_sub_ab_addr;
                banks_src2_addr = mod_sub_ab_addr;
                //
                banks_dst_addr  = mod_sub_d_addr;
                //
                banks_dst_wren  = mod_sub_d_wren;
                //
                banks_dst_din   = mod_sub_d_dout;
                //                
            end
            //
            UOP_OPCODE_MUL: begin
                //
                banks_src1_addr = mod_mul_a_addr;
                banks_src2_addr = mod_mul_b_addr;
                //
                banks_dst_addr  = mod_mul_p_addr;
                //
                banks_dst_wren  = mod_mul_p_wren;
                //
                banks_dst_din   = mod_mul_p_dout;
                //                
            end
            //
            default: begin
                //
                banks_src1_addr = {3{1'bX}};
                banks_src2_addr = {3{1'bX}};
                //
                banks_dst_addr  = {3{1'bX}};
                //
                banks_dst_wren  = 1'b0;
                //
                banks_dst_din   = {32{1'bX}};
                //
            end
            //
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
            FSM_STATE_IDLE:     fsm_state_next = ena                     ? FSM_STATE_FETCH : FSM_STATE_IDLE;
            FSM_STATE_FETCH:    fsm_state_next = FSM_STATE_DECODE;
            FSM_STATE_DECODE:   fsm_state_next = uop_data_opcode_is_stop ? FSM_STATE_IDLE  : FSM_STATE_BUSY;
            FSM_STATE_BUSY:     fsm_state_next = fsm_exit_from_busy      ? FSM_STATE_FETCH : FSM_STATE_BUSY;
        endcase
        //
    end


    //
    // Ready Flag Logic
    //
    reg rdy_reg = 1'b1;
    assign rdy = rdy_reg;

    always @(posedge clk or negedge rst_n)
        //
        if (rst_n == 1'b0)      rdy_reg <= 1'b1;
        else case (fsm_state)
            FSM_STATE_IDLE:     rdy_reg <= ~ena;
            FSM_STATE_DECODE:   rdy_reg <= uop_data_opcode_is_stop;
        endcase


    //
    // Output Logic
    //
    reg [ 2: 0] xy_addr_reg = 3'b000;
    reg [31: 0] xy_dout_reg = 32'h00000000;
    reg         x_wren_reg  = 1'b0;
    reg         y_wren_reg  = 1'b0;

    assign xy_addr = xy_addr_reg;
    assign xy_dout = xy_dout_reg;
    assign x_wren  = x_wren_reg;
    assign y_wren  = y_wren_reg;

    reg xy_phase;   // 0 - x, 1 - y

    always @(posedge clk)
        //
        if (output_now) begin
        
            if (ena)
                xy_phase <= 1'b0;
            else if (!mw_mover_ena && mw_mover_rdy && (fsm_state == FSM_STATE_BUSY))
                xy_phase <= 1'b1;
        
        end 
           

    always @(posedge clk)
        //
        if (output_now && mw_mover_y_wren) xy_addr_reg <= mw_mover_y_addr;
        else                               xy_addr_reg <= 3'b000;
        
    always @(posedge clk)
        //
        if (output_now && mw_mover_y_wren) xy_dout_reg <= mw_mover_y_dout;
        else                               xy_dout_reg <= 32'h00000000;
        
    always @(posedge clk)
            //
            if (output_now && mw_mover_y_wren) {y_wren_reg, x_wren_reg} <= {xy_phase, ~xy_phase};
            else                               {y_wren_reg, x_wren_reg} <= 2'b00;        

endmodule


//------------------------------------------------------------------------------
// End-of-File
//------------------------------------------------------------------------------
