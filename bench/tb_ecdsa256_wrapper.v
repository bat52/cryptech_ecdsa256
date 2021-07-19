//------------------------------------------------------------------------------
//
// tb_ecdsa256_wrapper.v
// -----------------------------------------------------------------------------
// Testbench for 256-bit curve base point scalar multiplier.
//
// Authors: Pavel Shatov
//
// Copyright 2016, 2018-2019 NORDUnet A/S
// Copyright 2021 The Commons Conservancy Cryptech Project
// SPDX-License-Identifier: BSD-3-Clause
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
// - Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
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

module tb_ecdsa256_wrapper;


        //
        // Test Vectors
        //
    `include "ecdsa256_test_vector_nsa.vh"
    `include "ecdsa_test_vector_randomized.vh"


        //
        // Core Parameters
        //
    localparam WORD_COUNTER_WIDTH = 3;
    localparam OPERAND_NUM_WORDS  = 8;


        //
        // Register Offsets
        //
    localparam CORE_ADDR_NAME0		= 24'h000000;
    localparam CORE_ADDR_NAME1		= 24'h000001;
    localparam CORE_ADDR_VERSION	= 24'h000002;
    localparam CORE_ADDR_CONTROL	= 24'h000008;
    localparam CORE_ADDR_STATUS		= 24'h000009;

    localparam CORE_ADDR_BUF_K		= 24'h000020;
    localparam CORE_ADDR_BUF_X		= 24'h000028;
    localparam CORE_ADDR_BUF_Y		= 24'h000030;

    localparam CORE_CONTROL_BIT_NEXT    = 32'h00000002;
    localparam CORE_STATUS_BIT_VALID    = 32'h00000002;


        //
        // STM32 Settings
        //
    localparam STM32_FMC_LATENCY = 6;


       //
       // P-256 Domain Parameters
       //
    localparam ECDSA_P256_N =
        {32'hffffffff, 32'h00000000, 32'hffffffff, 32'hffffffff,
         32'hbce6faad, 32'ha7179e84, 32'hf3b9cac2, 32'hfc632551};

    localparam ECDSA_P256_GX =
        {32'h6b17d1f2, 32'he12c4247, 32'hf8bce6e5, 32'h63a440f2,
         32'h77037d81, 32'h2deb33a0, 32'hf4a13945, 32'hd898c296};

    localparam ECDSA_P256_GY =
        {32'h4fe342e2, 32'hfe1a7f9b, 32'h8ee7eb4a, 32'h7c0f9e16,
         32'h2bce3357, 32'h6b315ece, 32'hcbb64068, 32'h37bf51f5};

    localparam ECDSA_P256_HX =
        {32'h7cf27b18, 32'h8d034f7e, 32'h8a523803, 32'h04b51ac3,
         32'hc08969e2, 32'h77f21b35, 32'ha60b48fc, 32'h47669978};
    
    localparam ECDSA_P256_HY =
        {32'h07775510, 32'hdb8ed040, 32'h293d9ac6, 32'h9f7430db,
         32'hba7dade6, 32'h3ce98229, 32'h9e04b79d, 32'h227873d1};


        //
        // FMC Clock (50 MHz)
        //
    localparam FMC_CLOCK_PERIOD         = 20.0;
    localparam FMC_CLOCK_PERIOD_HALF    = 0.5 * FMC_CLOCK_PERIOD;
    localparam FMC_CLOCK_PERIOD_QUARTER = 0.5 * FMC_CLOCK_PERIOD_HALF;

    reg fmc_clk = 1'b0;

    initial forever #FMC_CLOCK_PERIOD_HALF fmc_clk = ~fmc_clk;


        //
        // Clock Manager
        //
    wire io_clk;     // i/o clock
    wire sys_clk;    // system clock
    wire sys_rst_n;  // active-low reset

	alpha_clkmgr clkmgr_inst
	(        
		.fmc_clk		(fmc_clk),
        
        .io_clk         (io_clk),
		.sys_clk		(sys_clk),
		.sys_rst_n		(sys_rst_n),
        .core_clk       ()
	);
    
    
    //
    // FMC Arbiter - FPGA Side
    //
    wire [23: 0] sys_fmc_addr;
    wire         sys_fmc_wren;
    wire         sys_fmc_rden;
    wire [31: 0] sys_fmc_dout;
    wire [31: 0] sys_fmc_din;


    //
    // FMC Arbiter - STM32 Side
    //
    reg  [23: 0] fmc_a = {24{1'bX}};
    reg  [31: 0] fmc_d_drive;
    wire [31: 0] fmc_d_bidir;
    reg          fmc_ne1 = 1'b1;
    reg          fmc_noe = 1'b1;
    reg          fmc_nwe = 1'b1;
    reg          fmc_nl = 1'b1;
    wire         fmc_nwait_dummy;

    assign fmc_d_bidir = fmc_noe ? fmc_d_drive : 32'hZZZZZZZZ;

    fmc_arbiter #(.NUM_ADDR_BITS(24))
    fmc_arbiter_inst
    (
        // fmc bus
        .fmc_a     (fmc_a),
        .fmc_d     (fmc_d_bidir),
        .fmc_ne1   (fmc_ne1),
        .fmc_nl    (fmc_nl),
        .fmc_nwe   (fmc_nwe),
        .fmc_noe   (fmc_noe),
        .fmc_nwait (fmc_nwait_dummy),

        // system clock, i/o clock
        .io_clk  (io_clk),
        .sys_clk (sys_clk),

        // user bus
        .sys_addr     (sys_fmc_addr),
        .sys_wr_en    (sys_fmc_wren),
        .sys_data_out (sys_fmc_dout),
        .sys_rd_en    (sys_fmc_rden),
        .sys_data_in  (sys_fmc_din)
    );
    
    ecdsa256_wrapper uut
    (
        .clk            (sys_clk),
        .reset_n        (sys_rst_n),

        .cs             (sys_fmc_wren | sys_fmc_rden),
        .we             (sys_fmc_wren),

        .address        (sys_fmc_addr[5:0]),
        .write_data     (sys_fmc_dout),
        .read_data      (sys_fmc_din)
    );


        //
        // Testbench Routine
        //
    reg ok = 1;
    
    reg [31:0] core_name0;
    reg [31:0] core_name1;
    reg [31:0] core_version;
    
    initial begin

            /* wait for some time */
        #4000;

            /* sanity checks */
        fmc_read(CORE_ADDR_NAME0,   core_name0);
        fmc_read(CORE_ADDR_NAME1,   core_name1);
        fmc_read(CORE_ADDR_VERSION, core_version);
             
        $display("CORE_NAME0:   %x", core_name0);             
        $display("CORE_NAME1:   %x", core_name1);
        $display("CORE_VERSION: %x", core_version);
             
            /* run tests */
        $display("1. Q1 = d1 * G...");
        test_curve_multiplier(ECDSA_P256_D_NSA, ECDSA_P256_QX_NSA, ECDSA_P256_QY_NSA);
        
        $display("2. R = k * G...");
        test_curve_multiplier(ECDSA_P256_K_NSA, ECDSA_P256_RX_NSA, ECDSA_P256_RY_NSA);
        
        $display("3. Q2 = d2 * G...");
        test_curve_multiplier(ECDSA_P256_D_RANDOM, ECDSA_P256_QX_RANDOM, ECDSA_P256_QY_RANDOM);

        $display("4. O = n * G...");
        test_curve_multiplier(ECDSA_P256_N, 256'd0, 256'd0);

        $display("5. G = (n + 1) * G...");
        test_curve_multiplier(ECDSA_P256_N + 256'd1, ECDSA_P256_GX, ECDSA_P256_GY);

        $display("6. H = 2 * G...");
        test_curve_multiplier(256'd2, ECDSA_P256_HX, ECDSA_P256_HY);

        $display("7. H = (n + 2) * G...");
        test_curve_multiplier(ECDSA_P256_N + 256'd2, ECDSA_P256_HX, ECDSA_P256_HY);

            /* print result */
        if (ok) $display("tb_curve_multiplier_256: SUCCESS");
        else    $display("tb_curve_multiplier_256: FAILURE");

        $finish;

    end


        //
        // Test Task
        //
    reg p_ok;
    integer w;
    reg busy;
    reg [31:0] reg_control;
    reg [31:0] reg_status;
    task test_curve_multiplier;
    
        input [255:0] k;
        input [255:0] px;
        input [255:0] py;

        reg [255:0] k_shreg;
        reg [255:0] px_shreg;
        reg [255:0] py_shreg;

        begin
        
                // initialize shift registers
            k_shreg = k;

                // write all the words
            for (w=0; w<OPERAND_NUM_WORDS; w=w+1) begin
                fmc_write(CORE_ADDR_BUF_K + w, k_shreg[31:0]);
                k_shreg = {{32{1'bX}}, k_shreg[255:32]};
            end

                // start operation
            reg_control = {32{1'b0}};
            fmc_write(CORE_ADDR_CONTROL, reg_control);
            reg_control = CORE_CONTROL_BIT_NEXT;
            fmc_write(CORE_ADDR_CONTROL, reg_control);
            
                // wait for operation to complete
            busy = 1;
            while (busy) begin
                fmc_read(CORE_ADDR_STATUS, reg_status);
                if ((reg_status & CORE_STATUS_BIT_VALID) == CORE_STATUS_BIT_VALID) busy = 0;
            end
            
                // read result
            for (w=0; w<OPERAND_NUM_WORDS; w=w+1) begin
                px_shreg = {{32{1'bX}}, px_shreg[255:32]};
                py_shreg = {{32{1'bX}}, py_shreg[255:32]};
                fmc_read(CORE_ADDR_BUF_X + w, px_shreg[255:224]);
                fmc_read(CORE_ADDR_BUF_Y + w, py_shreg[255:224]);
            end

                // compare
            p_ok = (px_shreg === px) &&
                   (py_shreg === py);

                // display results
            if (p_ok) $display("test_curve_multiplier(): OK");
            else begin
                $display("test_curve_multiplier(): ERROR");
                $display("ref_px  == %x", px);
                $display("calc_px == %x", px_shreg);
                $display("ref_py  == %x", py);
                $display("calc_py == %x", py_shreg);
            end

                // update global flag
            ok = ok && p_ok;
        
        end

    endtask

    
    //
    // Helper Function
    //
    function  [31:0] _fmc_swap;
        input [31:0]  data;
        _fmc_swap = {data[7:0], data[15:8], data[23:16], data[31:24]};
    endfunction
    
    
    //-------------
    task fmc_write;
    //-------------
        input [23: 0] addr;
        input [31: 0] data;
        begin
            fmc_ne1 = 1'b0;                     // select
            fmc_nl = 1'b0;                      // set latch flag
            fmc_a = addr;                       // set address
            fmc_nwe = 1'b0;                     // set write-enable
            wait_full_tick();                   // mimic latency

            fmc_nl = 1'b1;                      // clear latch flag
            fmc_a = {24{1'bX}};                 // clear address
            wait_n_ticks(STM32_FMC_LATENCY);    // mimic latency

            fmc_d_drive = _fmc_swap(data);      // set data            

            wait_half_tick();                   // mimic latency
            wait_quarter_tick();

            fmc_ne1 = 1'b1;                     // deselect
            fmc_nwe = 1'b1;                     // clear write-enable
            fmc_d_drive = 32'hXXXXXXXX;         // clear data

            wait_quarter_tick();                // finish clock period
            wait_full_tick();                   // pause
        end
    endtask

    //------------
    task fmc_read;
    //------------
        input  [23: 0] addr;
        output [31: 0] data;
        begin
            fmc_ne1 = 1'b0;                     // select
            fmc_nl = 1'b0;                      // set latch flag
            fmc_a = addr;                       // set address

            wait_full_tick();                   // mimic latency

            fmc_nl = 1'b1;                      // clear latch flag
            fmc_a = {24{1'bX}};                 // clear address
            wait_full_tick();                   // mimic latency
            fmc_noe = 1'b0;                     // tri-state bus
            wait_n_ticks(STM32_FMC_LATENCY-1);  // mimic latency

            wait_half_tick();                   // mimic latency
            data = _fmc_swap(fmc_d_bidir);      // sample data
            wait_half_tick();                   // mimic latency
            
            wait_full_tick();                   // mimic latency
            
            wait_half_tick();                   // mimic latency
            wait_quarter_tick();

            fmc_ne1 = 1'b1;                     // deselect
            fmc_noe = 1'b1;                     // drive bus

            wait_quarter_tick();                // finish clock period
            wait_full_tick();                   // pause
        end
    endtask
    
    
    //
    // Helper Tasks
    //
    
    //----------------------
    task  wait_quarter_tick;
    //----------------------
        begin
            #FMC_CLOCK_PERIOD_QUARTER;
        end
    endtask

    //------------------
    task wait_half_tick;
    //------------------
        begin
            wait_quarter_tick();
            wait_quarter_tick();
        end
    endtask

    //------------------
    task wait_full_tick;
    //------------------
        begin
            wait_half_tick();
            wait_half_tick();
        end
    endtask

    //----------------
    task wait_n_ticks;
    //----------------
        input integer n;
        integer i;
        begin
            for (i=0; i<n; i=i+1)
                wait_full_tick();
        end
    endtask
    
    
endmodule


//------------------------------------------------------------------------------
// End-of-File
//------------------------------------------------------------------------------
