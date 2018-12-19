//======================================================================
//
// Copyright (c) 2018, NORDUnet A/S All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// - Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// - Neither the name of the NORDUnet nor the names of its contributors may
//   be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module ecdsa256_operand_bank
(
    input               clk,

    input   [ 9-1:0]    a_addr,
    input               a_wr,
    input   [32-1:0]    a_in,

    input   [ 9-1:0]    b_addr,
    output  [32-1:0]    b_out
);


    //
    // BRAM
    //
    reg [31:0] bram[0:64*8-1];


    //
    // Initialization
    //
    initial begin
        //
        // CONST_ZERO 
        //
        bram[ 0*8 + 7] = 32'h00000000;
        bram[ 0*8 + 6] = 32'h00000000;
        bram[ 0*8 + 5] = 32'h00000000;
        bram[ 0*8 + 4] = 32'h00000000;
        bram[ 0*8 + 3] = 32'h00000000;
        bram[ 0*8 + 2] = 32'h00000000;
        bram[ 0*8 + 1] = 32'h00000000;
        bram[ 0*8 + 0] = 32'h00000000;
        //
        // CONST_ONE
        //
        bram[ 1*8 + 7] = 32'h00000000;
        bram[ 1*8 + 6] = 32'h00000000;
        bram[ 1*8 + 5] = 32'h00000000;
        bram[ 1*8 + 4] = 32'h00000000;
        bram[ 1*8 + 3] = 32'h00000000;
        bram[ 1*8 + 2] = 32'h00000000;
        bram[ 1*8 + 1] = 32'h00000000;
        bram[ 1*8 + 0] = 32'h00000001;
        //
        // CONST_DELTA
        //
        bram[ 2*8 + 7] = 32'h7fffffff;
        bram[ 2*8 + 6] = 32'h80000000;
        bram[ 2*8 + 5] = 32'h80000000;
        bram[ 2*8 + 4] = 32'h00000000;
        bram[ 2*8 + 3] = 32'h00000000;
        bram[ 2*8 + 2] = 32'h80000000;
        bram[ 2*8 + 1] = 32'h00000000;
        bram[ 2*8 + 0] = 32'h00000000;
        //
        // G_X
        //
        bram[ 3*8 + 7] = 32'h6b17d1f2;
        bram[ 3*8 + 6] = 32'he12c4247;
        bram[ 3*8 + 5] = 32'hf8bce6e5;
        bram[ 3*8 + 4] = 32'h63a440f2;
        bram[ 3*8 + 3] = 32'h77037d81;
        bram[ 3*8 + 2] = 32'h2deb33a0;
        bram[ 3*8 + 1] = 32'hf4a13945;
        bram[ 3*8 + 0] = 32'hd898c296;
        //
        // G_Y
        //
        bram[ 4*8 + 7] = 32'h4fe342e2;
        bram[ 4*8 + 6] = 32'hfe1a7f9b;
        bram[ 4*8 + 5] = 32'h8ee7eb4a;
        bram[ 4*8 + 4] = 32'h7c0f9e16;
        bram[ 4*8 + 3] = 32'h2bce3357;
        bram[ 4*8 + 2] = 32'h6b315ece;
        bram[ 4*8 + 1] = 32'hcbb64068;
        bram[ 4*8 + 0] = 32'h37bf51f5;
        //
        // H_X
        //
        bram[ 5*8 + 7] = 32'h7cf27b18;
        bram[ 5*8 + 6] = 32'h8d034f7e;
        bram[ 5*8 + 5] = 32'h8a523803;
        bram[ 5*8 + 4] = 32'h04b51ac3;
        bram[ 5*8 + 3] = 32'hc08969e2;
        bram[ 5*8 + 2] = 32'h77f21b35;
        bram[ 5*8 + 1] = 32'ha60b48fc;
        bram[ 5*8 + 0] = 32'h47669978;
        //
        // H_Y
        //
        bram[ 6*8 + 7] = 32'h07775510;
        bram[ 6*8 + 6] = 32'hdb8ed040;
        bram[ 6*8 + 5] = 32'h293d9ac6;
        bram[ 6*8 + 4] = 32'h9f7430db;
        bram[ 6*8 + 3] = 32'hba7dade6;
        bram[ 6*8 + 2] = 32'h3ce98229;
        bram[ 6*8 + 1] = 32'h9e04b79d;
        bram[ 6*8 + 0] = 32'h227873d1;
	end


    //
    // Output Register
    //
    reg [32-1:0] bram_reg_b;

    assign b_out = bram_reg_b;


    //
    // Write Port A
    //
    always @(posedge clk)
        //
        if (a_wr) bram[a_addr] <= a_in;


    //
    // Read Port B
    //
    always @(posedge clk)
        //
        bram_reg_b <= bram[b_addr];


endmodule
