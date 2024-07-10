/*
 * Copyright (C) 2024-2024 ETH Zurich and University of Bologna
 *
 * Licensed under the Solderpad Hardware License, Version 0.51 
 * (the "License"); you may not use this file except in compliance 
 * with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: SHL-0.51
 *
 * Authors:  Maurus Item     <itemm@student.ethz.ch>
 * 
 * RedMulE TCDM Deduplication Cache
 */

`include "hci_helpers.svh"
`include "common_cells/registers.svh"

module redmule_deduplicator 
  import redmule_pkg::*;
  import hci_package::*;
  import hwpe_stream_package::*;
#(
  parameter hci_size_parameter_t `HCI_SIZE_PARAM(tcdm) = '0
) (
  input logic                       clk_i,
  input logic                       rst_ni,
  hci_core_intf.target    tcdm_target,
  hci_core_intf.initiator tcdm_initiator
);

  localparam int unsigned AW  = `HCI_SIZE_GET_AW(tcdm);
  localparam int unsigned DW = `HCI_SIZE_GET_DW(tcdm);

  //////////////////////////////////////////////////////////////////////////////////
  // Forward Deduplication
  // TLDR If to elements back-to-back have the same address then drop the second one
  logic [AW-1:0] add_q;
  logic wen_q;
  logic request_in;
  assign request_in = tcdm_target.req && tcdm_target.gnt;

  `FFL(add_q, tcdm_target.add, request_in, '0);
  `FFL(wen_q, tcdm_target.wen, request_in, '0);

  logic drop_request;
  assign drop_request = 1'b0; // tcdm_target.req && (tcdm_target.add == add_q && tcdm_target.wen == wen_q);

  // Handshake Injection
  always_comb begin
    if (drop_request) begin
      tcdm_initiator.req  = 1'b0;
      tcdm_target.gnt     = 1'b1;   
    end else begin
      tcdm_initiator.req  = tcdm_target.req;
      tcdm_target.gnt     = tcdm_initiator.gnt;
    end
  end

  ////////////////////////////////////////////////////////////////////////////////
  // Forward path reduplication

  // Find all requests in forwards path
  logic read_request_in, drop_read;
  assign read_request_in  = request_in && tcdm_target.wen;
  assign drop_read = read_request_in & drop_request;

  logic pickup_read, response_out, empty;

  // For each request store if it was dropped by the forwards path
  fifo_v3 #(
    .FALL_THROUGH ( 0  ), // Every response can only be sent in the next cycle at the earliest
    .DATA_WIDTH   ( 1  ),
    .DEPTH        ( 32 )
  ) i_fifo (
    .clk_i,
    .rst_ni,
    .flush_i     (                1'b0 ),
    .testmode_i  (                1'b0 ),
    .data_i      (           drop_read ),
    .push_i      (     read_request_in ),
    .data_o      (         pickup_read ),
    .pop_i       (        response_out ),
    .usage_o     (        /* Unused */ ),
    .full_o      (        /* Unused */ ),
    .empty_o     (               empty )
  );

  logic replicate_read;
  assign replicate_read = pickup_read && !empty;

  assign response_in = tcdm_initiator.r_valid && !empty && tcdm_initiator.r_ready;
  assign response_out = response_in || replicate_read;

  // Store data whenever there is a request recieved
  logic [DW-1:0] r_data_q;
  `FFL(r_data_q, tcdm_initiator.r_data, response_in, '0);

  // Data / Handshake Injection
  always_comb begin
    if (replicate_read) begin
      tcdm_initiator.r_ready = 1'b0;
      tcdm_target.r_valid = 1'b1;
      tcdm_target.r_data = r_data_q;
    end else begin
      tcdm_initiator.r_ready = tcdm_target.r_ready;
      tcdm_target.r_valid = tcdm_initiator.r_valid;
      tcdm_target.r_data = tcdm_initiator.r_data;
    end
  end

  // All other signals assigned without difference
  assign tcdm_initiator.add     = tcdm_target.add;
  assign tcdm_initiator.wen     = tcdm_target.wen;
  assign tcdm_initiator.data    = tcdm_target.data;
  assign tcdm_initiator.be      = tcdm_target.be;
  assign tcdm_initiator.user    = tcdm_target.user;
  assign tcdm_initiator.id      = tcdm_target.id;
  assign tcdm_target.r_user     = tcdm_initiator.r_user;
  assign tcdm_target.r_id       = tcdm_initiator.r_id;
  assign tcdm_target.r_opc      = tcdm_initiator.r_opc;

  // ECC signals
  assign tcdm_initiator.ereq     = tcdm_target.ereq;
  assign tcdm_target.egnt        = tcdm_initiator.egnt;
  assign tcdm_target.r_evalid    = tcdm_initiator.r_evalid;
  assign tcdm_initiator.r_eready = tcdm_target.r_eready;
  assign tcdm_initiator.ecc      = tcdm_target.ecc;
  assign tcdm_target.r_ecc       = tcdm_initiator.r_ecc;

endmodule // redmule_deduplicator
