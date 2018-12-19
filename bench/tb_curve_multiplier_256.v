//------------------------------------------------------------------------------
//
// tb_curve_multiplier_256.v
// -----------------------------------------------------------------------------
// Testbench for 256-bit curve base point scalar multiplier.
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

module tb_curve_multiplier_256;


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
        // Clock (100 MHz)
        //
    reg clk = 1'b0;
    always #5 clk = ~clk;


        //
        // Inputs, Outputs
        //
    reg  rst_n;
    reg  ena;
    wire rdy;


        //
        // Buffers (K, PX, PY)
        //
    wire [WORD_COUNTER_WIDTH-1:0] core_k_addr;
    wire [WORD_COUNTER_WIDTH-1:0] core_pxy_addr;

    wire                          core_px_wren;
    wire                          core_py_wren;

    wire [                32-1:0] core_k_data;
    wire [                32-1:0] core_pxy_data;

    reg  [WORD_COUNTER_WIDTH-1:0] tb_k_addr;
    reg  [WORD_COUNTER_WIDTH-1:0] tb_pxy_addr;

    reg                           tb_k_wren;

    reg  [                  31:0] tb_k_data;
    wire [                  31:0] tb_px_data;
    wire [                  31:0] tb_py_data;

    bram_1rw_1ro_readfirst # (.MEM_WIDTH(32), .MEM_ADDR_BITS(WORD_COUNTER_WIDTH))
    bram_k
    (   .clk(clk),
        .a_addr(tb_k_addr), .a_wr(tb_k_wren), .a_in(tb_k_data), .a_out(),
        .b_addr(core_k_addr), .b_out(core_k_data)
    );

    bram_1rw_1ro_readfirst # (.MEM_WIDTH(32), .MEM_ADDR_BITS(WORD_COUNTER_WIDTH))
    bram_px
    (   .clk(clk),
        .a_addr(core_pxy_addr), .a_wr(core_px_wren), .a_in(core_pxy_data), .a_out(),
        .b_addr(tb_pxy_addr), .b_out(tb_px_data)
    );

    bram_1rw_1ro_readfirst # (.MEM_WIDTH(32), .MEM_ADDR_BITS(WORD_COUNTER_WIDTH))
    bram_py
    (   .clk(clk),
        .a_addr(core_pxy_addr), .a_wr(core_py_wren), .a_in(core_pxy_data), .a_out(),
        .b_addr(tb_pxy_addr), .b_out(tb_py_data)
    );


        //
        // UUT
        //
    ecdsa256_base_point_multiplier uut
    (
        .clk        (clk),
        .rst_n      (rst_n),

        .ena        (ena),
        .rdy        (rdy),

        .k_addr     (core_k_addr),
        .rxy_addr   (core_pxy_addr),

        .rx_wren    (core_px_wren),
        .ry_wren    (core_py_wren),

        .k_din      (core_k_data),

        .rxy_dout   (core_pxy_data)
    );


        //
        // Testbench Routine
        //
    reg ok = 1;
    initial begin

            /* initialize control inputs */
        rst_n = 0;
        ena   = 0;
        
            /* wait for some time */
        #200;
        
            /* de-assert reset */
        rst_n = 1;
        
            /* wait for some time */
        #100;
        
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

        //$finish;

    end


        //
        // Test Task
        //
    reg p_ok;

    integer w;

    task test_curve_multiplier;
    
        input [255:0] k;
        input [255:0] px;
        input [255:0] py;

        reg [255:0] k_shreg;
        reg [255:0] px_shreg;
        reg [255:0] py_shreg;

        begin
        
                /* start filling memories */
            tb_k_wren = 1;

                /* initialize shift registers */
            k_shreg = k;

                /* write all the words */
            for (w=0; w<OPERAND_NUM_WORDS; w=w+1) begin

                    /* set addresses */
                tb_k_addr = w[WORD_COUNTER_WIDTH-1:0];

                    /* set data words */
                tb_k_data   = k_shreg[31:0];

                    /* shift inputs */
                k_shreg = {{32{1'bX}}, k_shreg[255:32]};

                    /* wait for 1 clock tick */
                #10;

            end

                /* wipe addresses */
            tb_k_addr = {WORD_COUNTER_WIDTH{1'bX}};

                /* wipe data words */
            tb_k_data = {32{1'bX}};

                /* stop filling memories */
            tb_k_wren = 0;

                /* start operation */
            ena = 1;

                /* clear flag */
            #10 ena = 0;

                /* wait for operation to complete */
            while (!rdy) #10;

                /* read result */
            for (w=0; w<OPERAND_NUM_WORDS; w=w+1) begin

                    /* set address */
                tb_pxy_addr = w[WORD_COUNTER_WIDTH-1:0];

                    /* wait for 1 clock tick */
                #10;

                    /* store data word */
                px_shreg = {tb_px_data, px_shreg[255:32]};
                py_shreg = {tb_py_data, py_shreg[255:32]};

            end

                /* compare */
            p_ok = (px_shreg === px) &&
                   (py_shreg === py);

                /* display results */
            if (p_ok) $display("test_curve_multiplier(): OK");
            else begin
                $display("test_curve_multiplier(): ERROR");
                $display("ref_px  == %x", px);
                $display("calc_px == %x", px_shreg);
                $display("ref_py  == %x", py);
                $display("calc_py == %x", py_shreg);
            end

                /* update global flag */
            ok = ok && p_ok;
        
        end

    endtask

endmodule


//------------------------------------------------------------------------------
// End-of-File
//------------------------------------------------------------------------------
