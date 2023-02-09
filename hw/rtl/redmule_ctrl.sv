/*
 * Copyright (C) 2022-2023 ETH Zurich and University of Bologna
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
 * Authors: Yvan Tortorella <yvan.tortorella@unibo.it>
 * 
 * RedMulE Control Unit
 */

import fpnew_pkg::*;
import hci_package::*;
import redmule_pkg::*;
import hwpe_ctrl_package::*;

module redmule_ctrl #(
parameter  int unsigned N_CORES       = 8                      ,
parameter  int unsigned IO_REGS       = REDMULE_REGS           ,
parameter  int unsigned ID_WIDTH      = 8                      ,
parameter  int unsigned N_CONTEXT     = 2                      ,
parameter  int unsigned Height        = 4                      ,
parameter  int unsigned Width         = 8                      ,
parameter  int unsigned NumPipeRegs   = 3                      ,
localparam int unsigned TILE          = (NumPipeRegs +1)*Height,
localparam int unsigned W_ITERS       = W_ITERS                ,
localparam int unsigned LEFT_PARAMS   = LEFT_PARAMS            
)(
  input  logic                    clk_i             ,
  input  logic                    rst_ni            ,
  input  logic                    test_mode_i       ,
  output logic                    busy_o            ,
  output logic                    clear_o           ,
  output logic [N_CORES-1:0][1:0] evt_o             ,
  output logic                    output_fill_o     ,
  output logic                    w_shift_o         ,
  output logic                    out_wrap_clk_en_o ,
  output ctrl_regfile_t           reg_file_o        ,
  input  logic                    reg_enable_i      ,
  // Flags coming from the wrap registers
  input  z_buffer_flgs_t          flgs_output_wrap_i,
  // Flags coming from the engine
  input  flgs_engine_t            flgs_engine_i     ,
  // Flags coming from the state machine
  input  logic                    w_loaded_i        ,
  // Control signals for the engine
  output logic                    flush_o           ,
  output logic                    accumulate_o      ,
  // Control signals for the state machine
  output cntrl_scheduler_t        cntrl_scheduler_o ,
  // Peripheral slave port
  hwpe_ctrl_intf_reqrsp           cfg
);

  logic        clear;
  logic        accumulate_q;
  logic        w_computed_en, w_computed_rst, count_weight_q, accumulate_en, accumulate_rst, storing_rst;
  logic        last_w_row, last_w_row_en, last_w_row_rst;
  logic        in_feat_sub, weight_sub;
  logic        out_wrap_clk_en;
  logic        enable_depth_count, reset_depth_count;
  logic [4:0]  w_computed;
  logic [15:0] in_rows, in_columns, w_rows, w_columns, out_rows, out_columns;
  logic [31:0] /*in_base_add,*/ w_base_add, out_base_add;
  logic [15:0] in_rows_iter, in_cols_iter, in_rows_lftovr, in_cols_lftovr;
  logic [15:0] w_rows_iter, w_cols_iter, w_rows_lftovr, w_cols_lftovr, w_row_count_d, w_row_count_q;
  logic [15:0] out_rows_iter, out_cols_iter, out_rows_lftovr, out_cols_lftovr, out_storings_d, out_storings_q, tot_stores;

  typedef enum logic [2:0] {REDMULE_IDLE, REDMULE_CFGMUL, REDMULE_STARTING, REDMULE_COMPUTING, REDMULE_BUFFERING, REDMULE_STORING, REDMULE_FINISHED} redmule_ctrl_state;
  redmule_ctrl_state current, next;

  hwpe_ctrl_package::ctrl_regfile_t reg_file;
  hwpe_ctrl_package::ctrl_slave_t   cntrl_slave;
  hwpe_ctrl_package::flags_slave_t  flgs_slave;

  // Control slave interface
  hwpe_ctrl_reqrsp_target #(
    .N_IO_REGS      ( REDMULE_REGS_EVEN ),
    .N_GENERIC_REGS ( 0                 )
  ) i_slave         (
    .clk_i          ( clk_i             ),
    .rst_ni         ( rst_ni            ),
    .clear_o        ( clear             ),
    .cfg            ( cfg               ),
    .ctrl_i         ( cntrl_slave       ),
    .flags_o        ( flgs_slave        ),
    .reg_file       ( reg_file          )
  );
  /*---------------------------------------------------------------------------------------------*/
  /*                                       Register island                                       */
  /*---------------------------------------------------------------------------------------------*/

  // State register
  always_ff @(posedge clk_i or negedge rst_ni) begin : state_register
    if(~rst_ni) begin
       current <= REDMULE_IDLE;
     end else begin
       if (clear) 
         current <= REDMULE_IDLE;
       else
         current <= next;
     end
  end

  // This register counts the number of weight rows loaded
  always_ff @(posedge clk_i or negedge rst_ni) begin : weight_rows_counter
    if(~rst_ni) begin
      w_row_count_q <= '0;
    end else begin
      if (clear) 
        w_row_count_q <= '0;
      else
        w_row_count_q <= w_row_count_d;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin 
    if(~rst_ni) begin
      count_weight_q <= 1'b0;
    end else begin
      if (clear || w_computed_rst)
          count_weight_q <= 1'b0;
      else if (w_computed_en)
          count_weight_q <= 1'b1;
    end
  end

  // This one counts how many weights have been computed by the last
  // PE in each row of the array
  always_ff @(posedge clk_i or negedge rst_ni) begin : w_computed_counter
    if(~rst_ni) begin
      w_computed <= '0;
    end else begin
      if (w_computed_rst || clear)
        w_computed <= '0;
      else if (count_weight_q && reg_enable_i)
        w_computed <= w_computed + 1;
    end
  end

  logic accumulate_ctrl_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      accumulate_ctrl_q <= 1'b0;
    end else begin
      if (clear || reg_enable_i)
        accumulate_ctrl_q <= 1'b0;
      else if (!reg_enable_i && !accumulate_q)
        accumulate_ctrl_q <= 1'b1;
    end
  end

  // This register generates the accumulation signal for the engine
  always_ff @(posedge clk_i or negedge rst_ni) begin : accumale_sampler
    if(~rst_ni) begin
      accumulate_q <= 1'b0;
    end else begin
      if (accumulate_rst || clear)
        accumulate_q <= 1'b0;
      else if (accumulate_en)
        accumulate_q <= 1'b1;
    end
  end
  assign accumulate_o =  accumulate_q & !accumulate_ctrl_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin : last_w_row_reg
    if(~rst_ni) begin
      last_w_row <= 1'b0;
    end else begin
      if (last_w_row_rst || clear)
        last_w_row <= 1'b0;
      else if (last_w_row_en) 
        last_w_row <= 1'b1;
    end
  end

  // This register counts the number of times we exit from the REDMULE_STORING
  // state and go to the REDMULE_COMPUTING one. Every time this happens, it
  // means that a piece of computation fas done, and we can track the number
  // the number of storage operations to see when the last one occurs
  always_ff @(posedge clk_i or negedge rst_ni) begin : out_storings_counter
    if(~rst_ni) begin
      out_storings_q <= '0;
    end else begin
      if (clear || storing_rst) 
        out_storings_q <= '0;
      else
        out_storings_q <= out_storings_d;
    end
  end

  /*---------------------------------------------------------------------------------------------*/
  /*                                   Register file assignment                                  */
  /*---------------------------------------------------------------------------------------------*/
  logic cfg_valid;
  logic cfg_start;
  logic cfg_first_push_d, cfg_first_push_q;
  assign cfg_start = cfg.q_write & cfg.q_valid & (cfg.q_addr[7:0] == 8'h20) & cfg_first_push_q;

  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni)
      cfg_first_push_q <= '0;
    else if(clear | current==REDMULE_STARTING)
      cfg_first_push_q <= 1'b1;
    else if(cfg.q_write & cfg.q_valid & (cfg.q_addr[7:0] == 8'h20))
      cfg_first_push_q <= 1'b0;
  end 

  redmule_config_decoder #(
    .ADDR_WIDTH   ( ADDR_W       ),
    .ARRAY_WIDTH  ( ARRAY_WIDTH  ),
    .ARRAY_HEIGHT ( ARRAY_HEIGHT ),
    .PIPE_REGS    ( PIPE_REGS    ),
    .FPFORMAT     ( 16           ),
    .DATA_WIDTH   ( DATAW        )
  ) i_config_decoder (
    .clk_i      ( clk_i      ),
    .rst_ni     ( rst_ni     ),
    .clear_i    ( clear      ),
    .start_i    ( cfg_start  ),
    .valid_o    ( cfg_valid  ),
    .reg_file_i ( reg_file   ),
    .reg_file_o ( reg_file_o )
  );
  assign w_rows_iter = reg_file_o.hwpe_params [W_ITERS    ][31:16];
  assign tot_stores  = reg_file_o.hwpe_params [LEFT_PARAMS][31:16];

  assign out_wrap_clk_en_o = out_wrap_clk_en;

  /*---------------------------------------------------------------------------------------------*/
  /*                                        Controller FSM                                       */
  /*---------------------------------------------------------------------------------------------*/
  // This is a local FSM who's only work is to make the first 
  // input load operation and to start the tensorcore_fsm
  always_comb begin : controller_fsm
    cntrl_scheduler_o  = '0;
    cntrl_slave        = '0;
    // Engine default control signals assignment
    flush_o            = 1'b0;
    // Other local default signals
    w_shift_o          = 1'b1;
    output_fill_o      = 1'b0;
    busy_o             = 1'b1;
    w_computed_en      = 1'b0;
    w_computed_rst     = 1'b0;
    last_w_row_en      = 1'b0;
    last_w_row_rst     = 1'b0;
    w_row_count_d      = w_row_count_q;
    out_storings_d     = out_storings_q;
    accumulate_en      = 1'b0;
    accumulate_rst     = 1'b0;
    storing_rst        = 1'b0;
    out_wrap_clk_en    = '0;
    enable_depth_count = '0;
    reset_depth_count  = '0;
    next               = current;

    case (current)
      REDMULE_IDLE: begin
        w_shift_o = 1'b0;
        busy_o    = 1'b0;
        out_storings_d = '0;
        w_row_count_d  = '0;
        if (clear)
          out_wrap_clk_en = 1'b1;
        if (cfg_valid & (flgs_slave.start || test_mode_i))
          next = REDMULE_STARTING;
        else if (flgs_slave.start || test_mode_i)
          next = REDMULE_CFGMUL;
        else 
          next = REDMULE_IDLE;
      end

      REDMULE_CFGMUL: begin
        w_shift_o = 1'b0;
        busy_o    = 1'b0;
        out_storings_d = '0;
        w_row_count_d  = '0;
        if (clear)
          out_wrap_clk_en = 1'b1;
        if (cfg_valid)
          next = REDMULE_STARTING;
      end
  
      REDMULE_STARTING: begin
        w_shift_o              = 1'b0;
        cntrl_scheduler_o.first_load = 1'b1;
        if (w_loaded_i) begin
          next = REDMULE_COMPUTING;
          w_row_count_d = w_row_count_q + 1;
        end else
          next = REDMULE_STARTING;
      end

      REDMULE_COMPUTING: begin
        if (w_loaded_i)
          w_row_count_d = w_row_count_q + 1;
        
        if (w_row_count_d == Height && !count_weight_q)
          w_computed_en = 1'b1;
        else if (w_row_count_q == w_rows_iter) begin
          if (!count_weight_q)
              w_computed_en = 1'b1;
          if (!last_w_row)
              last_w_row_en = 1'b1;
        end
        
        case (last_w_row)
          1'b0: begin
            if (w_computed == Height - 1) begin
              if (!accumulate_q)
                  accumulate_en = 1'b1;
              if (count_weight_q)
                  w_computed_rst = 1'b1;
            end
          end

          1'b1: begin
            if (w_computed == Height - 2 && reg_enable_i) begin
              w_row_count_d = 16'd1;
              next = REDMULE_BUFFERING;
              if (accumulate_q)
                  accumulate_rst = 1'b1;
              if (count_weight_q)
                  w_computed_rst = 1'b1;
            end else
              next = REDMULE_COMPUTING;
          end
        endcase
      end
  
      REDMULE_BUFFERING: begin
        out_wrap_clk_en = 1'b1;
        if (last_w_row)
            last_w_row_rst = 1'b1;
        if (w_loaded_i)
          w_row_count_d = w_row_count_q + 1;
        output_fill_o = reg_enable_i;
        if (flgs_output_wrap_i.full) begin
          accumulate_en = 1'b1;
          next = REDMULE_STORING;
        end
        else
          next = REDMULE_BUFFERING;
      end
  
      REDMULE_STORING: begin
        cntrl_scheduler_o.storing = 1'b1;
      
        if (w_loaded_i)
          w_row_count_d = w_row_count_q + 1;

        if (flgs_output_wrap_i.empty) begin
          out_storings_d = out_storings_q + 1;
          if (out_storings_q == tot_stores - 1) begin
            next = REDMULE_FINISHED;
            storing_rst = 1'b1;
            cntrl_scheduler_o.finished = 1'b1;
          end else
          if (out_storings_q < tot_stores) begin
            next = REDMULE_COMPUTING;
          end
        end
      end
  
      REDMULE_FINISHED: begin
        cntrl_slave.done = 1'b1;
        busy_o           = 1'b0;
        flush_o          = 1'b1;
        cntrl_scheduler_o.rst  = 1'b1;
        cntrl_scheduler_o.finished = 1'b1;
        next = REDMULE_IDLE;
        // Reset for all the registers
        w_row_count_d  = '0;
        w_computed_rst = 1'b1;
        accumulate_rst = 1'b1;
        last_w_row_rst = 1'b1;
        storing_rst    = 1'b1;
      end
    endcase
  end

  /*---------------------------------------------------------------------------------------------*/
  /*                            Other combinational assigmnets                                   */
  /*---------------------------------------------------------------------------------------------*/
  assign evt_o   = flgs_slave.evt[7:0];
  assign clear_o = clear; 

endmodule : redmule_ctrl
