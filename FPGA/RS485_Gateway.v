module rs485_Gateway (
    input  wire clk_12m,
    input  wire rs485_rx_slave,
    output reg  rs485_tx_master,
    output wire rs485_en_master,
    output wire rs485_en_slave,
    input  wire spi_sck,
    input  wire spi_mosi,
    input  wire spi_cs_n,
    output wire spi_miso,
    output reg  frame_block_done
);

    (* keep = "true" *) wire clk;
    SB_PLL40_CORE #(
        .FEEDBACK_PATH("SIMPLE"),
        .DIVR(4'd0),
        .DIVF(7'd39),
        .DIVQ(3'd3),
        .FILTER_RANGE(3'b001)
    ) pll_inst (
        .REFERENCECLK(clk_12m),
        .PLLOUTGLOBAL(clk),
        .RESETB(1'b1),
        .BYPASS(1'b0)
    );

    assign rs485_en_master = 1'b1;
    assign rs485_en_slave = 1'b0;

    localparam integer RX_BIT_TICKS     = 48;
    localparam integer RX_HALF_BIT      = 24;
    localparam integer FRAME_GAP_TICKS  = 9600;
    localparam integer NODE_ROUTE_GAP   = 3000;
    localparam integer INTER_BYTE_GAP   = 144;
    localparam integer RESPONSE_TIMEOUT = 24000;
    localparam integer RETRY_GAP_TICKS  = 24000;
    localparam integer GAP_SHORT_TICKS  = 30000;
    localparam integer GAP_MID_TICKS    = 90000;
    localparam integer GAP_LONG_TICKS   = 330000;
    localparam integer GAP_INIT_SHORT   = 150000;
    localparam integer GAP_INIT_MID     = 450000;

    localparam [2:0] RX_IDLE  = 3'd0,
                     RX_START = 3'd1,
                     RX_DATA  = 3'd2,
                     RX_MDB   = 3'd3,
                     RX_STOP  = 3'd4;

    localparam [2:0] PHASE_IDLE   = 3'd0,
                     PHASE_INIT_A = 3'd1,
                     PHASE_INIT_B = 3'd2,
                     PHASE_INIT_C = 3'd3,
                     PHASE_INIT_D = 3'd4,
                     PHASE_LOOP_E = 3'd5;

    localparam [1:0] CTRL_IDLE      = 2'd0,
                     CTRL_WAIT_TX   = 2'd1,
                     CTRL_WAIT_RESP = 2'd2,
                     CTRL_WAIT_GAP  = 2'd3;

    localparam [7:0] SPI_CMD_READ_STATUS = 8'hB0;
    localparam [7:0] SPI_CMD_READ_FRAMES = 8'hB1;
    localparam [7:0] SPI_CMD_START_INIT  = 8'hB2;
    localparam [7:0] SPI_CMD_RESET_CTRL  = 8'hB3;
    localparam [7:0] SPI_CMD_GET_AUTH_STATE = 8'hB4;
    localparam [7:0] SPI_CMD_TAKE_AUTHORITY = 8'hB5;
    localparam [7:0] SPI_CMD_RETURN_AUTHORITY = 8'hB6;
    localparam [7:0] SPI_CMD_WRITE_POSITION = 8'hB7;
    localparam [7:0] SPI_CMD_WRITE_EXTRA = 8'hB8;
    localparam [7:0] SPI_CMD_READ_EXTRA = 8'hB9;

    function [7:0] node_id;
        input [2:0] index;
        begin
            case (index)
                3'd0: node_id = 8'h10;
                3'd1: node_id = 8'h08;
                3'd2: node_id = 8'h11;
                3'd3: node_id = 8'h09;
                3'd4: node_id = 8'h12;
                3'd5: node_id = 8'h0A;
                3'd6: node_id = 8'h20;  // Extra frame (Gripper/Reset)
                default: node_id = 8'h20;
            endcase
        end
    endfunction

    function [5:0] resp_base;
        input [2:0] index;
        begin
            case (index)
                3'd0: resp_base = 6'd0;
                3'd1: resp_base = 6'd7;
                3'd2: resp_base = 6'd14;
                3'd3: resp_base = 6'd21;
                3'd4: resp_base = 6'd28;
                default: resp_base = 6'd35;
            endcase
        end
    endfunction

    function [4:0] pos_base;
        input [2:0] index;
        begin
            case (index)
                3'd0: pos_base = 5'd0;
                3'd1: pos_base = 5'd3;
                3'd2: pos_base = 5'd6;
                3'd3: pos_base = 5'd9;
                3'd4: pos_base = 5'd12;
                default: pos_base = 5'd15;
            endcase
        end
    endfunction

    function [7:0] crc6;
        input [7:0] b0;
        input [7:0] b1;
        input [7:0] b2;
        input [7:0] b3;
        input [7:0] b4;
        input [7:0] b5;
        begin
            crc6 = b0 ^ b1 ^ b2 ^ b3 ^ b4 ^ b5;
        end
    endfunction

    reg [7:0] response_slots [0:41];
    reg [7:0] shadow_positions [0:17];

    // ESP2FPGA Input Buffers for position writes (7 nodes: 0x10..0x0A + 0x20)
    reg [7:0] esp2fpga_buffer [0:48];  // 7 nodes * 7 bytes = 49 bytes
    reg [6:0] buffer_writeable = 7'b1111111;  // Bit per node: 1=writeable, 0=blocked
    reg esp_has_authority = 1'b0;  // 0=Robot has authority, 1=ESP has authority
    reg extra_frame_pending = 1'b0;  // Extra frame (0x20) ready to send
    reg [7:0] extra_response [0:6];  // Extra frame response (7 bytes)
    reg extra_response_valid = 1'b0;  // Extra response has been received

    reg [2:0] controller_phase = PHASE_IDLE;
    reg [1:0] controller_state = CTRL_IDLE;
    reg [2:0] current_node = 3'd0;
    reg [19:0] cycle_counter = 20'd0;
    reg [19:0] gap_target = 20'd0;
    reg [5:0] slot_valid_mask = 6'd0;
    reg [15:0] timeout_count = 16'd0;
    reg [15:0] crc_error_count = 16'd0;
    reg [1:0] init_a_valid_mask = 2'b00;
    reg run_active = 1'b0;
    reg init_done = 1'b0;

    // SPI-domain registers (clocked on spi_sck)
    reg [7:0] spi_shift_in = 8'd0;
    reg [3:0] spi_bit_phase = 4'd0;
    reg [5:0] spi_byte_idx = 6'd0;
    reg [7:0] spi_cmd = 8'd0;
    reg [7:0] spi_tx_curr = 8'd0;
    reg [7:0] spi_tx_next = 8'd0;
    reg spi_start_init_flag = 1'b0;
    reg spi_reset_flag = 1'b0;
    reg spi_take_authority_flag = 1'b0;
    reg spi_return_authority_flag = 1'b0;
    reg [2:0] spi_write_node_idx = 3'd0;  // Which node to write (0-6)
    reg spi_write_trigger = 1'b0;

    // Cross-domain synchronizers (60 MHz domain)
    reg [1:0] sync_start_init = 2'b00;
    reg        sync_start_prev = 1'b0;
    reg [1:0] sync_reset_ctrl = 2'b00;
    reg        sync_reset_prev = 1'b0;
    reg [1:0] sync_cs_n = 2'b11;
    reg [1:0] sync_take_authority = 2'b00;
    reg        sync_take_auth_prev = 1'b0;
    reg [1:0] sync_return_authority = 2'b00;
    reg        sync_return_auth_prev = 1'b0;

    wire start_init_pulse = sync_start_init[1] && !sync_start_prev;
    wire controller_reset_pulse = sync_reset_ctrl[1] && !sync_reset_prev;
    wire cs_fall_60m = (sync_cs_n == 2'b10);
    wire take_authority_pulse = sync_take_authority[1] && !sync_take_auth_prev;
    wire return_authority_pulse = sync_return_authority[1] && !sync_return_auth_prev;

    // Snapshot registers (latched in 60 MHz domain on CS falling edge)
    reg [7:0] snap_status [0:7];
    reg [7:0] snap_frames [0:41];
    reg [7:0] snap_extra [0:6];
    reg snap_init_ack = 1'b0;

    assign spi_miso = (!spi_cs_n && spi_bit_phase >= 4'd1 && spi_bit_phase <= 4'd8) ?
                      spi_tx_curr[4'd8 - spi_bit_phase] : 1'b0;

    reg [7:0] request_b0 = 8'd0;
    reg [7:0] request_b1 = 8'd0;
    reg [7:0] request_b2 = 8'd0;
    reg [7:0] request_b3 = 8'd0;
    reg [7:0] request_b4 = 8'd0;
    reg [7:0] request_b5 = 8'd0;
    reg [7:0] request_b6 = 8'd0;
    reg       tx_frame_pending = 1'b0;

    reg [2:0] tx_frame_byte_idx = 3'd0;
    reg [12:0] tx_gap_counter = 13'd0;
    reg [10:0] tx_shift = 11'h7FF;
    reg [5:0] tx_bit_timer = 6'd0;
    reg [3:0] tx_bits_left = 4'd0;
    reg tx_frame_busy = 1'b0;
    reg tx_byte_busy = 1'b0;
    reg tx_frame_done = 1'b0;

    reg [7:0] req_template_b0;
    reg [7:0] req_template_b1;
    reg [7:0] req_template_b2;
    reg [7:0] req_template_b3;
    reg [7:0] req_template_b4;
    reg [7:0] req_template_b5;
    reg [7:0] req_template_b6;
    reg [19:0] normal_gap_ticks;

    wire rx_raw = rs485_rx_slave;
    reg [1:0] rx_sync = 2'b11;
    reg [2:0] rx_filt = 3'b111;
    wire rx_in = (rx_filt[2] & rx_filt[1]) | (rx_filt[2] & rx_filt[0]) | (rx_filt[1] & rx_filt[0]);

    reg [2:0] rx_state = RX_IDLE;
    reg [5:0] rx_timer = 6'd0;
    reg [2:0] rx_bit = 3'd0;
    reg [7:0] rx_shift = 8'd0;
    reg rx_prev = 1'b1;
    reg [13:0] rx_idle_ticks = 14'd0;
    reg rx_start_after_gap = 1'b0;
    reg [7:0] rx_byte = 8'd0;
    reg rx_byte_valid = 1'b0;
    reg rx_byte_after_gap = 1'b0;
    reg rx_mdb = 1'b0;

    reg response_active = 1'b0;
    reg [2:0] response_count = 3'd0;
    reg response_start_mdb = 1'b0;
    reg [7:0] response_build_b0 = 8'd0;
    reg [7:0] response_build_b1 = 8'd0;
    reg [7:0] response_build_b2 = 8'd0;
    reg [7:0] response_build_b3 = 8'd0;
    reg [7:0] response_build_b4 = 8'd0;
    reg [7:0] response_build_b5 = 8'd0;
    reg [7:0] response_frame_b0 = 8'd0;
    reg [7:0] response_frame_b1 = 8'd0;
    reg [7:0] response_frame_b2 = 8'd0;
    reg [7:0] response_frame_b3 = 8'd0;
    reg [7:0] response_frame_b4 = 8'd0;
    reg [7:0] response_frame_b5 = 8'd0;
    reg [7:0] response_frame_b6 = 8'd0;
    reg response_frame_valid = 1'b0;
    reg response_frame_crc_error = 1'b0;
    wire [7:0] response_crc_calc = response_build_b0 ^ response_build_b1 ^ response_build_b2 ^ response_build_b3 ^ response_build_b4 ^ response_build_b5;



    function [7:0] status_byte;
        input [3:0] index;
        reg [7:0] flags;
        begin
            flags = 8'h00;
            flags[0] = run_active;
            flags[1] = init_done;
            flags[2] = (controller_state == CTRL_WAIT_RESP);
            flags[3] = tx_frame_busy | tx_byte_busy | tx_frame_pending;
            case (index)
                4'd0: status_byte = flags;
                4'd1: status_byte = {5'b0, controller_phase};
                4'd2: status_byte = {2'b0, slot_valid_mask};
                4'd3: status_byte = node_id(current_node);
                4'd4: status_byte = timeout_count[7:0];
                4'd5: status_byte = timeout_count[15:8];
                4'd6: status_byte = crc_error_count[7:0];
                default: status_byte = crc_error_count[15:8];
            endcase
        end
    endfunction

    wire [7:0] tx_data_byte = (tx_frame_byte_idx == 3'd0) ? request_b0 :
                              (tx_frame_byte_idx == 3'd1) ? request_b1 :
                              (tx_frame_byte_idx == 3'd2) ? request_b2 :
                              (tx_frame_byte_idx == 3'd3) ? request_b3 :
                              (tx_frame_byte_idx == 3'd4) ? request_b4 :
                              (tx_frame_byte_idx == 3'd5) ? request_b5 : request_b6;

    integer init_index;

    initial begin
        rs485_tx_master = 1'b1;
        frame_block_done = 1'b0;
        for (init_index = 0; init_index < 42; init_index = init_index + 1) begin
            response_slots[init_index] = 8'h00;
        end
        for (init_index = 0; init_index < 18; init_index = init_index + 1) begin
            shadow_positions[init_index] = 8'h00;
        end
        for (init_index = 0; init_index < 49; init_index = init_index + 1) begin
            esp2fpga_buffer[init_index] = 8'h00;
        end
        for (init_index = 0; init_index < 7; init_index = init_index + 1) begin
            extra_response[init_index] = 8'h00;
        end
    end

    always @(*) begin
        req_template_b0 = node_id(current_node);
        req_template_b1 = 8'h00;
        req_template_b2 = 8'h00;
        req_template_b3 = 8'h00;
        req_template_b4 = 8'h00;
        req_template_b5 = 8'h00;
        normal_gap_ticks = GAP_SHORT_TICKS;

        case (controller_phase)
            PHASE_INIT_A: begin
                req_template_b1 = 8'h8F;
                req_template_b2 = 8'h09;
                req_template_b5 = 8'h97;
                normal_gap_ticks = (current_node == 3'd0) ? GAP_INIT_SHORT : GAP_INIT_MID;
            end

            PHASE_INIT_B: begin
                req_template_b1 = 8'h7A;
                req_template_b2 = 8'h01;
                req_template_b3 = 8'hE6;
                req_template_b4 = 8'h06;
                req_template_b5 = 8'h01;
                case (current_node)
                    3'd0, 3'd2, 3'd4: normal_gap_ticks = GAP_INIT_SHORT;
                    3'd1, 3'd3: normal_gap_ticks = GAP_INIT_MID;
                    default: normal_gap_ticks = GAP_MID_TICKS;
                endcase
            end

            PHASE_INIT_C,
            PHASE_INIT_D: begin
                req_template_b1 = 8'h31;
                req_template_b5 = 8'h26;
                case (current_node)
                    3'd0, 3'd2, 3'd4: normal_gap_ticks = GAP_SHORT_TICKS;
                    3'd1, 3'd3: normal_gap_ticks = GAP_MID_TICKS;
                    default: normal_gap_ticks = GAP_LONG_TICKS;
                endcase
            end

            PHASE_LOOP_E: begin
                req_template_b1 = 8'h31;
                req_template_b2 = shadow_positions[pos_base(current_node) + 0];
                req_template_b3 = shadow_positions[pos_base(current_node) + 1];
                req_template_b4 = shadow_positions[pos_base(current_node) + 2];
                req_template_b5 = 8'h26;
                case (current_node)
                    3'd0, 3'd2, 3'd4: normal_gap_ticks = GAP_SHORT_TICKS;
                    3'd1, 3'd3: normal_gap_ticks = GAP_MID_TICKS;
                    default: normal_gap_ticks = GAP_LONG_TICKS;
                endcase
            end

            default: begin
                normal_gap_ticks = GAP_SHORT_TICKS;
            end
        endcase

        req_template_b6 = crc6(req_template_b0, req_template_b1, req_template_b2, req_template_b3, req_template_b4, req_template_b5);
    end

    always @(posedge clk) begin
        rx_sync <= {rx_sync[0], rx_raw};
        rx_filt <= {rx_filt[1:0], rx_sync[1]};
    end

    always @(posedge clk) begin
        if (controller_reset_pulse) begin
            rx_state <= RX_IDLE;
            rx_timer <= 6'd0;
            rx_bit <= 3'd0;
            rx_shift <= 8'd0;
            rx_prev <= 1'b1;
            rx_idle_ticks <= 14'd0;
            rx_start_after_gap <= 1'b0;
            rx_byte <= 8'd0;
            rx_byte_valid <= 1'b0;
            rx_byte_after_gap <= 1'b0;
            rx_mdb <= 1'b0;
        end else begin
            rx_byte_valid <= 1'b0;
            rx_byte_after_gap <= 1'b0;
            rx_prev <= rx_in;

            case (rx_state)
                RX_IDLE: begin
                    if (rx_in) begin
                        if (rx_idle_ticks != 14'h3FFF)
                            rx_idle_ticks <= rx_idle_ticks + 1'b1;
                    end else if (rx_prev) begin
                        rx_state <= RX_START;
                        rx_timer <= RX_HALF_BIT - 1;
                        rx_start_after_gap <= (rx_idle_ticks >= FRAME_GAP_TICKS);
                        rx_idle_ticks <= 14'd0;
                    end
                end

                RX_START: begin
                    if (rx_timer == 0) begin
                        if (!rx_in) begin
                            rx_state <= RX_DATA;
                            rx_timer <= RX_BIT_TICKS - 1;
                            rx_bit <= 3'd0;
                            rx_shift <= 8'd0;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                        rx_idle_ticks <= 14'd0;
                    end else begin
                        rx_timer <= rx_timer - 1'b1;
                        rx_idle_ticks <= 14'd0;
                    end
                end

                RX_DATA: begin
                    if (rx_timer == 0) begin
                        rx_shift <= {rx_in, rx_shift[7:1]};
                        rx_timer <= RX_BIT_TICKS - 1;
                        if (rx_bit == 3'd7)
                            rx_state <= RX_MDB;
                        else
                            rx_bit <= rx_bit + 1'b1;
                    end else begin
                        rx_timer <= rx_timer - 1'b1;
                        rx_idle_ticks <= 14'd0;
                    end
                end

                RX_MDB: begin
                    if (rx_timer == 0) begin
                        rx_mdb <= rx_in;
                        rx_state <= RX_STOP;
                        rx_timer <= RX_BIT_TICKS - 1;
                    end else begin
                        rx_timer <= rx_timer - 1'b1;
                        rx_idle_ticks <= 14'd0;
                    end
                end

                RX_STOP: begin
                    if (rx_timer == 0) begin
                        if (rx_in) begin
                            rx_byte <= rx_shift;
                            rx_byte_valid <= 1'b1;
                            rx_byte_after_gap <= rx_start_after_gap;
                        end
                        rx_start_after_gap <= 1'b0;
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_timer <= rx_timer - 1'b1;
                        rx_idle_ticks <= 14'd0;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (controller_reset_pulse) begin
            response_active <= 1'b0;
            response_count <= 3'd0;
            response_start_mdb <= 1'b0;
            response_build_b0 <= 8'd0;
            response_build_b1 <= 8'd0;
            response_build_b2 <= 8'd0;
            response_build_b3 <= 8'd0;
            response_build_b4 <= 8'd0;
            response_build_b5 <= 8'd0;
            response_frame_b0 <= 8'd0;
            response_frame_b1 <= 8'd0;
            response_frame_b2 <= 8'd0;
            response_frame_b3 <= 8'd0;
            response_frame_b4 <= 8'd0;
            response_frame_b5 <= 8'd0;
            response_frame_b6 <= 8'd0;
            response_frame_valid <= 1'b0;
            response_frame_crc_error <= 1'b0;
        end else begin
            response_frame_valid <= 1'b0;
            response_frame_crc_error <= 1'b0;

            if (rx_byte_valid) begin
                if (rx_byte_after_gap) begin
                    response_active <= 1'b1;
                    response_count <= 3'd1;
                    response_start_mdb <= rx_mdb;
                    response_build_b0 <= rx_byte;
                    response_build_b1 <= 8'd0;
                    response_build_b2 <= 8'd0;
                    response_build_b3 <= 8'd0;
                    response_build_b4 <= 8'd0;
                    response_build_b5 <= 8'd0;
                end else if (response_active) begin
                    case (response_count)
                        3'd1: response_build_b1 <= rx_byte;
                        3'd2: response_build_b2 <= rx_byte;
                        3'd3: response_build_b3 <= rx_byte;
                        3'd4: response_build_b4 <= rx_byte;
                        3'd5: response_build_b5 <= rx_byte;
                        default: ;
                    endcase

                    if (response_count == 3'd6) begin
                        response_active <= 1'b0;
                        response_count <= 3'd0;
                        if ((rx_byte == response_crc_calc) && !response_start_mdb) begin
                            response_frame_b0 <= response_build_b0;
                            response_frame_b1 <= response_build_b1;
                            response_frame_b2 <= response_build_b2;
                            response_frame_b3 <= response_build_b3;
                            response_frame_b4 <= response_build_b4;
                            response_frame_b5 <= response_build_b5;
                            response_frame_b6 <= rx_byte;
                            response_frame_valid <= 1'b1;
                            
                            // If this is an extra frame response (NodeID 0x20), store it
                            if (response_build_b0 == 8'h20) begin
                                extra_response[0] <= response_build_b0;
                                extra_response[1] <= response_build_b1;
                                extra_response[2] <= response_build_b2;
                                extra_response[3] <= response_build_b3;
                                extra_response[4] <= response_build_b4;
                                extra_response[5] <= response_build_b5;
                                extra_response[6] <= rx_byte;
                                extra_response_valid <= 1'b1;
                            end
                        end else begin
                            response_frame_crc_error <= 1'b1;
                        end
                    end else begin
                        response_count <= response_count + 1'b1;
                    end
                end
            end
        end
    end

    always @(posedge clk) begin
        tx_frame_done <= 1'b0;

        if (controller_reset_pulse) begin
            rs485_tx_master <= 1'b1;
            tx_frame_byte_idx <= 3'd0;
            tx_gap_counter <= 13'd0;
            tx_shift <= 11'h7FF;
            tx_bit_timer <= 6'd0;
            tx_bits_left <= 4'd0;
            tx_frame_busy <= 1'b0;
            tx_byte_busy <= 1'b0;
        end else if (tx_byte_busy) begin
            if (tx_bit_timer == 0) begin
                rs485_tx_master <= tx_shift[0];
                tx_shift <= {1'b1, tx_shift[10:1]};
                tx_bit_timer <= RX_BIT_TICKS - 1;
                tx_bits_left <= tx_bits_left - 1'b1;

                if (tx_bits_left == 4'd1) begin
                    tx_byte_busy <= 1'b0;
                    rs485_tx_master <= 1'b1;

                    if (tx_frame_byte_idx == 3'd6) begin
                        tx_frame_busy <= 1'b0;
                        tx_frame_done <= 1'b1;
                    end else begin
                        tx_frame_byte_idx <= tx_frame_byte_idx + 1'b1;
                        if (tx_frame_byte_idx == 3'd0)
                            tx_gap_counter <= NODE_ROUTE_GAP + INTER_BYTE_GAP - 1;
                        else
                            tx_gap_counter <= INTER_BYTE_GAP;
                    end
                end
            end else begin
                tx_bit_timer <= tx_bit_timer - 1'b1;
            end
        end else if (tx_frame_busy) begin
            rs485_tx_master <= 1'b1;
            if (tx_gap_counter != 0) begin
                tx_gap_counter <= tx_gap_counter - 1'b1;
            end else begin
                tx_byte_busy <= 1'b1;
                tx_bit_timer <= RX_BIT_TICKS - 1;
                tx_bits_left <= 4'd11;
                tx_shift <= {1'b1, (tx_frame_byte_idx == 3'd0), tx_data_byte, 1'b0};
            end
        end else if (tx_frame_pending) begin
            rs485_tx_master <= 1'b1;
            tx_frame_busy <= 1'b1;
            tx_frame_byte_idx <= 3'd0;
            tx_gap_counter <= 13'd0;
        end else begin
            rs485_tx_master <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (controller_reset_pulse) begin
            controller_phase <= PHASE_IDLE;
            controller_state <= CTRL_IDLE;
            current_node <= 3'd0;
            cycle_counter <= 20'd0;
            gap_target <= 20'd0;
            slot_valid_mask <= 6'd0;
            timeout_count <= 16'd0;
            crc_error_count <= 16'd0;
            init_a_valid_mask <= 2'b00;
            run_active <= 1'b0;
            init_done <= 1'b0;
            tx_frame_pending <= 1'b0;
            for (init_index = 0; init_index < 42; init_index = init_index + 1) begin
                response_slots[init_index] <= 8'h00;
            end
            for (init_index = 0; init_index < 18; init_index = init_index + 1) begin
                shadow_positions[init_index] <= 8'h00;
            end
        end else begin
            if (controller_state != CTRL_IDLE && cycle_counter != 20'hFFFFF)
                cycle_counter <= cycle_counter + 1'b1;

            if (start_init_pulse && controller_state == CTRL_IDLE) begin
                controller_phase <= PHASE_INIT_A;
                controller_state <= CTRL_WAIT_TX;
                current_node <= 3'd0;
                cycle_counter <= 20'd0;
                gap_target <= 20'd0;
                slot_valid_mask <= 6'd0;
                timeout_count <= 16'd0;
                crc_error_count <= 16'd0;
                init_a_valid_mask <= 2'b00;
                run_active <= 1'b1;
                init_done <= 1'b0;
                request_b0 <= 8'h10;
                request_b1 <= 8'h8F;
                request_b2 <= 8'h09;
                request_b3 <= 8'h00;
                request_b4 <= 8'h00;
                request_b5 <= 8'h97;
                request_b6 <= 8'h01;
                tx_frame_pending <= 1'b1;
                for (init_index = 0; init_index < 42; init_index = init_index + 1) begin
                    response_slots[init_index] <= 8'h00;
                end
                for (init_index = 0; init_index < 18; init_index = init_index + 1) begin
                    shadow_positions[init_index] <= 8'h00;
                end
            end else begin
                case (controller_state)
                    CTRL_IDLE: begin
                        cycle_counter <= 20'd0;
                    end

                    CTRL_WAIT_TX: begin
                        if (tx_frame_busy)
                            tx_frame_pending <= 1'b0;
                        if (tx_frame_done) begin
                            controller_state <= CTRL_WAIT_RESP;
                        end
                    end

                    CTRL_WAIT_RESP: begin
                        if (response_frame_valid && (response_frame_b0 == node_id(current_node))) begin
                            controller_state <= CTRL_WAIT_GAP;
                            if (controller_phase == PHASE_INIT_A) begin
                                init_a_valid_mask <= init_a_valid_mask | (2'b01 << current_node[0]);
                                gap_target <= GAP_INIT_SHORT;

                                if ((init_a_valid_mask | (2'b01 << current_node[0])) == 2'b11) begin
                                    controller_phase <= PHASE_INIT_B;
                                    current_node <= 3'd0;
                                end else if ((init_a_valid_mask | (2'b01 << current_node[0])) == 2'b01) begin
                                    current_node <= 3'd1;
                                end else begin
                                    current_node <= 3'd0;
                                end
                            end else begin
                                gap_target <= normal_gap_ticks;

                                if ((controller_phase == PHASE_INIT_C) || (controller_phase == PHASE_INIT_D) || (controller_phase == PHASE_LOOP_E)) begin
                                    response_slots[resp_base(current_node) + 0] <= response_frame_b0;
                                    response_slots[resp_base(current_node) + 1] <= response_frame_b1;
                                    response_slots[resp_base(current_node) + 2] <= response_frame_b2;
                                    response_slots[resp_base(current_node) + 3] <= response_frame_b3;
                                    response_slots[resp_base(current_node) + 4] <= response_frame_b4;
                                    response_slots[resp_base(current_node) + 5] <= response_frame_b5;
                                    response_slots[resp_base(current_node) + 6] <= response_frame_b6;
                                    slot_valid_mask <= slot_valid_mask | (6'b000001 << current_node);
                                end

                                if ((controller_phase == PHASE_INIT_D) || (controller_phase == PHASE_LOOP_E)) begin
                                    shadow_positions[pos_base(current_node) + 0] <= response_frame_b1;
                                    shadow_positions[pos_base(current_node) + 1] <= response_frame_b2;
                                    shadow_positions[pos_base(current_node) + 2] <= response_frame_b3;
                                end

                                case (controller_phase)
                                    PHASE_INIT_B: begin
                                        if (current_node == 3'd5) begin
                                            controller_phase <= PHASE_INIT_C;
                                            current_node <= 3'd0;
                                        end else begin
                                            current_node <= current_node + 1'b1;
                                        end
                                    end

                                    PHASE_INIT_C: begin
                                        if (current_node == 3'd5) begin
                                            controller_phase <= PHASE_INIT_D;
                                            current_node <= 3'd0;
                                        end else begin
                                            current_node <= current_node + 1'b1;
                                        end
                                    end

                                    PHASE_INIT_D: begin
                                        if (current_node == 3'd5) begin
                                            controller_phase <= PHASE_LOOP_E;
                                            current_node <= 3'd0;
                                            init_done <= 1'b1;
                                        end else begin
                                            current_node <= current_node + 1'b1;
                                        end
                                    end

                                    default: begin
                                        if (current_node == 3'd5)
                                            current_node <= 3'd0;
                                        else
                                            current_node <= current_node + 1'b1;
                                    end
                                endcase
                            end
                        end else if (response_frame_crc_error) begin
                            crc_error_count <= crc_error_count + 1'b1;
                            controller_state <= CTRL_WAIT_GAP;

                            if (controller_phase == PHASE_INIT_A) begin
                                if (current_node == 3'd0) begin
                                    current_node <= 3'd1;
                                    gap_target <= GAP_INIT_SHORT;
                                end else begin
                                    if (init_a_valid_mask == 2'b00) begin
                                        current_node <= 3'd0;
                                        gap_target <= GAP_INIT_MID;
                                    end else if (init_a_valid_mask == 2'b01) begin
                                        current_node <= 3'd1;
                                        gap_target <= GAP_INIT_SHORT;
                                    end else begin
                                        current_node <= 3'd0;
                                        gap_target <= GAP_INIT_SHORT;
                                    end
                                end
                            end else begin
                                gap_target <= RETRY_GAP_TICKS;
                            end
                        end else if (cycle_counter >= RESPONSE_TIMEOUT) begin
                            timeout_count <= timeout_count + 1'b1;
                            controller_state <= CTRL_WAIT_GAP;

                            if (controller_phase == PHASE_INIT_A) begin
                                if (current_node == 3'd0) begin
                                    current_node <= 3'd1;
                                    gap_target <= GAP_INIT_SHORT;
                                end else begin
                                    if (init_a_valid_mask == 2'b00) begin
                                        current_node <= 3'd0;
                                        gap_target <= GAP_INIT_MID;
                                    end else if (init_a_valid_mask == 2'b01) begin
                                        current_node <= 3'd1;
                                        gap_target <= GAP_INIT_SHORT;
                                    end else begin
                                        current_node <= 3'd0;
                                        gap_target <= GAP_INIT_SHORT;
                                    end
                                end
                            end else begin
                                gap_target <= RETRY_GAP_TICKS;
                            end
                        end
                    end

                    CTRL_WAIT_GAP: begin
                        if (cycle_counter >= gap_target) begin
                            // Copy from ESP2FPGA buffer if ESP has authority, otherwise use template
                            if (esp_has_authority && (controller_phase == PHASE_LOOP_E) && (current_node < 3'd6)) begin
                                // ESP has authority: copy from ESP2FPGA buffer for this node
                                request_b0 <= esp2fpga_buffer[current_node * 7 + 0];
                                request_b1 <= esp2fpga_buffer[current_node * 7 + 1];
                                request_b2 <= esp2fpga_buffer[current_node * 7 + 2];
                                request_b3 <= esp2fpga_buffer[current_node * 7 + 3];
                                request_b4 <= esp2fpga_buffer[current_node * 7 + 4];
                                request_b5 <= esp2fpga_buffer[current_node * 7 + 5];
                                request_b6 <= esp2fpga_buffer[current_node * 7 + 6];
                                // Mark buffer as writeable immediately after copy
                                buffer_writeable[current_node] <= 1'b1;
                            end else begin
                                // Robot has authority OR init phase: use template
                                request_b0 <= req_template_b0;
                                request_b1 <= req_template_b1;
                                request_b2 <= req_template_b2;
                                request_b3 <= req_template_b3;
                                request_b4 <= req_template_b4;
                                request_b5 <= req_template_b5;
                                request_b6 <= req_template_b6;
                            end
                            tx_frame_pending <= 1'b1;
                            cycle_counter <= 20'd0;
                            controller_state <= CTRL_WAIT_TX;
                            
                            // Pulse frame_block_done when returning to node 0
                            if (current_node == 3'd5) begin
                                frame_block_done <= 1'b1;
                            end
                        end else begin
                            frame_block_done <= 1'b0;
                        end
                    end

                    default: controller_state <= CTRL_IDLE;
                endcase
            end
        end
    end

    // ---- Cross-domain synchronizers for SPI request flags ----
    always @(posedge clk) begin
        sync_start_init <= {sync_start_init[0], spi_start_init_flag};
        sync_start_prev <= sync_start_init[1];
        sync_reset_ctrl <= {sync_reset_ctrl[0], spi_reset_flag};
        sync_reset_prev <= sync_reset_ctrl[1];
        sync_cs_n <= {sync_cs_n[0], spi_cs_n};
        sync_take_authority <= {sync_take_authority[0], spi_take_authority_flag};
        sync_take_auth_prev <= sync_take_authority[1];
        sync_return_authority <= {sync_return_authority[0], spi_return_authority_flag};
        sync_return_auth_prev <= sync_return_authority[1];
    end

    // ---- Authority control ----
    always @(posedge clk) begin
        if (controller_reset_pulse) begin
            esp_has_authority <= 1'b0;
            buffer_writeable <= 7'b1111111;
        end else begin
            if (take_authority_pulse) begin
                esp_has_authority <= 1'b1;
                buffer_writeable <= 7'b1111111;  // All buffers writeable when taking authority
            end
            if (return_authority_pulse) begin
                esp_has_authority <= 1'b0;
            end
        end
    end

    // ---- Snapshot latch: capture status + frames on CS falling edge (60 MHz) ----
    always @(posedge clk) begin
        if (cs_fall_60m) begin
            snap_status[0] <= status_byte(4'd0);
            snap_status[1] <= status_byte(4'd1);
            snap_status[2] <= status_byte(4'd2);
            snap_status[3] <= status_byte(4'd3);
            snap_status[4] <= status_byte(4'd4);
            snap_status[5] <= status_byte(4'd5);
            snap_status[6] <= status_byte(4'd6);
            snap_status[7] <= status_byte(4'd7);
            for (init_index = 0; init_index < 42; init_index = init_index + 1) begin
                snap_frames[init_index] <= response_slots[init_index];
            end
            for (init_index = 0; init_index < 7; init_index = init_index + 1) begin
                snap_extra[init_index] <= extra_response[init_index];
            end
            snap_init_ack <= (controller_state == CTRL_IDLE && !run_active) ? 1'b1 : 1'b0;
        end
    end

    // ---- SPI Slave (clocked on SPI SCK, combinatorial MISO) ----
    always @(posedge spi_sck or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            spi_shift_in <= 8'd0;
            spi_bit_phase <= 4'd0;
            spi_byte_idx <= 6'd0;
            spi_cmd <= 8'd0;
            spi_tx_curr <= 8'd0;
            spi_tx_next <= 8'd0;
            // Flags are NOT cleared here - they latch until controller domain acknowledges them
            // spi_start_init_flag <= 1'b0;
            // spi_reset_flag <= 1'b0;
            // spi_take_authority_flag <= 1'b0;
            // spi_return_authority_flag <= 1'b0;
            spi_write_node_idx <= 3'd0;
            spi_write_trigger <= 1'b0;
        end else begin
            spi_shift_in <= {spi_shift_in[6:0], spi_mosi};

            // Clear all command flags at very start of transaction (before first bit)
            // This prevents race condition where old flags trigger actions before new command is received
            if (spi_bit_phase == 4'd0) begin
                spi_start_init_flag <= 1'b0;
                spi_reset_flag <= 1'b0;
                spi_take_authority_flag <= 1'b0;
                spi_return_authority_flag <= 1'b0;
            end

            if (spi_bit_phase == 4'd8) begin
                spi_bit_phase <= 4'd1;
                spi_tx_curr <= spi_tx_next;
            end else begin
                spi_bit_phase <= spi_bit_phase + 1'b1;
            end

            if (spi_bit_phase == 4'd7) begin
                spi_byte_idx <= spi_byte_idx + 1'b1;

                if (spi_byte_idx == 6'd0) begin
                    spi_cmd <= {spi_shift_in[6:0], spi_mosi};

                    case ({spi_shift_in[6:0], spi_mosi})
                        SPI_CMD_READ_STATUS: begin
                            spi_tx_next <= snap_status[0];
                        end

                        SPI_CMD_READ_FRAMES: begin
                            spi_tx_next <= snap_frames[0];
                        end

                        SPI_CMD_START_INIT: begin
                            spi_start_init_flag <= 1'b1;
                            spi_tx_next <= snap_init_ack ? 8'hA5 : 8'hEE;
                        end

                        SPI_CMD_RESET_CTRL: begin
                            spi_reset_flag <= 1'b1;
                            spi_tx_next <= 8'hA5;
                        end

                        SPI_CMD_GET_AUTH_STATE: begin
                            // Return: bit7=authority, bit6-0=writeable flags
                            spi_tx_next <= {esp_has_authority, buffer_writeable};
                        end

                        SPI_CMD_TAKE_AUTHORITY: begin
                            spi_take_authority_flag <= 1'b1;
                            spi_tx_next <= 8'hA5;
                        end

                        SPI_CMD_RETURN_AUTHORITY: begin
                            spi_return_authority_flag <= 1'b1;
                            spi_tx_next <= 8'hA5;
                        end

                        SPI_CMD_WRITE_POSITION: begin
                            // Next byte will be node index (0-5)
                            spi_tx_next <= 8'hA5;
                        end

                        SPI_CMD_WRITE_EXTRA: begin
                            // Write extra frame (node 6 = 0x20)
                            spi_write_node_idx <= 3'd6;
                            spi_tx_next <= 8'hA5;
                        end

                        SPI_CMD_READ_EXTRA: begin
                            spi_tx_next <= snap_extra[0];
                        end

                        default: begin
                            spi_tx_next <= 8'hEE;
                        end
                    endcase
                end else begin
                    case (spi_cmd)
                        SPI_CMD_READ_STATUS: begin
                            if (spi_byte_idx < 6'd8)
                                spi_tx_next <= snap_status[spi_byte_idx[2:0]];
                            else
                                spi_tx_next <= 8'h00;
                        end

                        SPI_CMD_READ_FRAMES: begin
                            if (spi_byte_idx < 6'd42)
                                spi_tx_next <= snap_frames[spi_byte_idx];
                            else
                                spi_tx_next <= 8'h00;
                        end

                        SPI_CMD_READ_EXTRA: begin
                            if (spi_byte_idx < 6'd7)
                                spi_tx_next <= snap_extra[spi_byte_idx];
                            else
                                spi_tx_next <= 8'h00;
                        end

                        SPI_CMD_WRITE_POSITION: begin
                            if (spi_byte_idx == 6'd1) begin
                                // Byte 1: node index (0-5) - take lower 3 bits
                                spi_write_node_idx <= {spi_shift_in[1:0], spi_mosi};
                                spi_tx_next <= 8'h00;
                            end else if (spi_byte_idx >= 6'd2 && spi_byte_idx <= 6'd8) begin
                                // Bytes 2-8: frame data (7 bytes)
                                if (buffer_writeable[spi_write_node_idx]) begin
                                    esp2fpga_buffer[spi_write_node_idx * 7 + (spi_byte_idx - 6'd2)] <= {spi_shift_in[6:0], spi_mosi};
                                end
                                spi_tx_next <= 8'h00;
                                // Mark buffer as not writeable after last byte
                                if (spi_byte_idx == 6'd8) begin
                                    buffer_writeable[spi_write_node_idx] <= 1'b0;
                                end
                            end else begin
                                spi_tx_next <= 8'h00;
                            end
                        end

                        SPI_CMD_WRITE_EXTRA: begin
                            if (spi_byte_idx >= 6'd1 && spi_byte_idx <= 6'd7) begin
                                // Bytes 1-7: frame data (7 bytes)
                                if (buffer_writeable[6]) begin
                                    esp2fpga_buffer[6 * 7 + (spi_byte_idx - 6'd1)] <= {spi_shift_in[6:0], spi_mosi};
                                end
                                spi_tx_next <= 8'h00;
                                // Mark buffer as not writeable after last byte
                                if (spi_byte_idx == 6'd7) begin
                                    buffer_writeable[6] <= 1'b0;
                                end
                            end else begin
                                spi_tx_next <= 8'h00;
                            end
                        end

                        default: begin
                            spi_tx_next <= 8'h00;
                        end
                    endcase
                end
            end
        end
    end

    wire _unused = &{1'b0,
                     response_frame_b0[0], response_frame_b1[0], response_frame_b2[0],
                     response_frame_b3[0], response_frame_b4[0], response_frame_b5[0],
                     response_frame_b6[0]};

endmodule
