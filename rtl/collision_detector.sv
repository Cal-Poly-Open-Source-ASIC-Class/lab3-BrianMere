`timescale 1ns/1ps

`define VALID_TRANSACTION_A (pA_wb_cyc_i)
`define VALID_TRANSACTION_B (pB_wb_cyc_i)

`define COLLISION (`VALID_TRANSACTION_A && `VALID_TRANSACTION_B && a_ramsel & b_ramsel)

/**
    Contains the output register to buffer to 
*/
module collision_detector(
    // Both Ports
    input logic clk,         // Sync. Clock
    input logic rst,         // Sync. Reset

    // Port A (wishbone)
    output logic pA_wb_stall_o, // hold high if this port needs to stall on a mem read
    input logic pA_wb_stb_i, // valid transfer signal, starts transaction
    output logic pA_wb_ack_o,   // terminates normal bus cycle, indicating end of transaction
    input logic pA_wb_cyc_i, // held high during the whole transaction

    // Port B (wishbone)
    output logic pB_wb_stall_o,// hold high if this port needs to stall on a mem read
    input logic pB_wb_stb_i, // valid transfer signal, qualifies addr, write enable, ...
    output logic pB_wb_ack_o,   // terminates normal bus cycle, indicating end of transaction
    input logic pB_wb_cyc_i, // held high during the whole transaction

    // To/From top module 
    input logic a_ramsel, input logic b_ramsel, // what each port wants to use
    output logic sel0_useA // indicates if RAM0 should use port A. Else uses port B.
);

    // We want to register the choice of sel0_useA and sel1_useA so that 
    // We can reference these values after a collision

    logic tiebreaker; // on low, use A as the tiebreaker, else B. 
    logic successful_A, successful_B; // flag to indicate success of the transaction

    // The calculation for sel0_useA and sel1_useB have to be in here.
    // This is because this has to be calculated before the clock goes high for a transaction
    always_comb begin : selCalculations
        if(`COLLISION) begin
            case (tiebreaker)
                0: begin // prioritize A
                    sel0_useA = !a_ramsel;
                end
                1: begin // prioritize B
                    sel0_useA = b_ramsel; // flipped since it's NOT B.
                end
            endcase
            successful_A = !tiebreaker;
            successful_B = tiebreaker;
        end
        else begin  // on no collision it's easy, just use the selections
            sel0_useA = !a_ramsel;
            successful_A = 1;
            successful_B = 1;
        end
    end
    
    always_ff @( posedge clk ) begin : CollisionReg
        if(rst) begin
            tiebreaker <= 0;
        end
        else if (`COLLISION) begin 
            tiebreaker <= !tiebreaker; // flip between ports on collisions for ties
        end
    end

    // On a collision make sure to set the right signals. Must be registered!
    logic valid_o; initial begin valid_o = 0; end
    always_ff @( posedge clk ) begin : CollisionResponse 
        if(rst) begin 
            valid_o <= 0;

            pA_wb_stall_o <= 0;
            pB_wb_stall_o <= 0;

            // Set the ACKs
            pA_wb_ack_o <= 0;
            pB_wb_ack_o <= 0;
        
        end else begin 
            if(valid_o == 0)
                valid_o <= valid_o + 1;
            if(valid_o) begin
                pA_wb_stall_o <= (sel0_useA == a_ramsel);
                pB_wb_stall_o <= (!sel0_useA == b_ramsel);

                // Set the ACKs
                pA_wb_ack_o <= successful_A;
                pB_wb_ack_o <= successful_B;
            end
        end
    end

endmodule