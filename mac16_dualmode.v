module mac16_dualmode (
    input  wire         clk,        // 系统时钟
    input  wire         rst_n,      // 异步复位（低有效）
    input  wire         inA,        // 串行输入A（MSB先行）
    input  wire         inB,        // 串行输入B（MSB先行）
    input  wire         in_ready,   // 输入使能（高表示当前时钟有有效输入）
    input  wire         mode,       // 累加模式选择：0-乘后两两累加；1-累计累加
    output wire         sum_out,    // 串行输出结果（MSB先行）
    output reg          carry,      // 累加进位输出
    output wire         out_ready   // 输出使能（高表示当前时钟有有效输出）
);

// 局部参数定义：提高代码可维护性
localparam INPUT_W  = 16;          // 输入数据位宽
localparam OUTPUT_W = 24;          // 输出数据位宽
localparam RX_CNT   = 4'd15;       // 输入移位计数最大值（16位输入）
localparam TX_CNT   = 5'd23;       // 输出移位计数最大值（24位输出）

// 输入阶段寄存器
reg [3:0]              rx_cnt;         // 输入移位计数器（0~15）
reg                    pingpong_sel;   // 输入乒乓寄存器选择（0-ping；1-pong）
reg [INPUT_W-1:0]      ping_A, ping_B; // ping通道：存储16位并行A/B
reg [INPUT_W-1:0]      pong_A, pong_B; // pong通道：存储16位并行A/B
reg [INPUT_W-1:0]      mult_inA, mult_inB; // 乘法器输入（并行A/B）
reg                    mult_en;        // 乘法器使能（高表示输入完成，触发乘法）
wire [OUTPUT_W-1:0]    mult_result;    // 乘法器24位输出结果
// 模式控制寄存器
reg [1:0]              mode_sync;      // 模式信号同步寄存器（防亚稳态）
wire                   mode_toggle;    // 模式切换标志（mode信号边沿检测）
// 累加器寄存器
reg [OUTPUT_W-1:0]     acc;            // 累加器（存储累加结果）
reg [OUTPUT_W-1:0]     last_mult;      // 上一次乘法结果（用于两两累加模式）
// 输出阶段寄存器
reg                    out_pingpong_sel; // 输出乒乓缓冲选择（0-buf0；1-buf1）
reg [OUTPUT_W-1:0]     out_buf0;       // 输出缓冲0（存储24位累加结果）
reg [OUTPUT_W-1:0]     out_buf1;       // 输出缓冲1（存储24位累加结果）
reg                    buf0_valid;     // 输出缓冲0有效标志
reg                    buf1_valid;     // 输出缓冲1有效标志
reg [4:0]              tx_cnt;         // 输出移位计数器（0~23）
reg [OUTPUT_W-1:0]     tx_shift_reg;   // 输出移位寄存器（串行输出数据）

// ---------------------- 输入阶段：串行转并行 + 乒乓缓冲 ----------------------
// 功能：将串行inA/inB移位存入乒乓寄存器，满16位后触发乘法器
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin // 复位初始化
        rx_cnt        <= 4'd0;
        pingpong_sel  <= 1'b0;
        mult_en       <= 1'b0;
        ping_A        <= 16'd0;
        ping_B        <= 16'd0;
        pong_A        <= 16'd0;
        pong_B        <= 16'd0;
    end else begin
        mult_en <= 1'b0; // 默认乘法器不使能
        if(in_ready) begin // 输入使能有效
            rx_cnt <= rx_cnt + 1'b1; // 输入计数器+1
            // 根据乒乓选择，将串行数据移入对应寄存器（MSB先行）
            if(!pingpong_sel) begin
                ping_A <= {ping_A[14:0], inA};
                ping_B <= {ping_B[14:0], inB};
            end else begin
                pong_A <= {pong_A[14:0], inA};
                pong_B <= {pong_B[14:0], inB};
            end
            // 输入满16位（计数器到15）
            if(rx_cnt == RX_CNT) begin
                rx_cnt       <= 4'd0;       // 计数器清零
                mult_en      <= 1'b1;       // 触发乘法器使能
                pingpong_sel <= ~pingpong_sel; // 切换乒乓寄存器
            end
        end
    end
end

// 乘法器输入选择：根据乒乓选择，输出当前满数据的通道
always @(*) begin
    mult_inA = pingpong_sel ? ping_A : pong_A;
    mult_inB = pingpong_sel ? ping_B : pong_B;
end

// ---------------------- 乘法器实例化：4阶Booth+华莱士树+CLA ----------------------
booth4_wallace_cla_24bit u_mult (
    .A      (mult_inA),    // 乘法器输入A（16位并行）
    .B      (mult_inB),    // 乘法器输入B（16位并行）
    .Result (mult_result)  // 乘法器输出（24位并行）
);

reg [23:0] mult_result_reg;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) mult_result_reg <= 24'd0;
    else mult_result_reg <= mult_result;
end

// ---------------------- 模式控制：同步 + 边沿检测 ----------------------
// mode信号同步（两级寄存器防亚稳态）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) mode_sync <= 2'b00;
    else mode_sync <= {mode_sync[0], mode};
end
// 检测mode信号的跳变（上升/下降沿），用于复位累加器
assign mode_toggle = (mode_sync[1] != mode_sync[0]);

reg mult_en_reg;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) mult_en_reg <= 1'b0;
    else mult_en_reg <= mult_en;                    //mult_en延迟一拍
end

// ---------------------- 累加器：双模累加逻辑 ----------------------
// 功能：根据mode选择累加模式，同时生成进位carry
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin // 复位初始化
        acc       <= 24'd0;
        last_mult <= 24'd0;
        carry     <= 1'b0;
    end else if(mode_toggle) begin // 模式切换时，复位累加器
        acc       <= 24'd0;
        last_mult <= 24'd0;
        carry     <= 1'b0;
    end else if(mult_en_reg) begin // 乘法器使能有效（新乘法结果生成）
        last_mult <= mult_result_reg; // 保存本次乘法结果
        // 模式0：两两累加（本次乘法结果 + 上一次乘法结果）
        if(mode_sync[1] == 1'b0)
            {carry, acc} <= mult_result_reg + last_mult;
        // 模式1：累计累加（累加器当前值 + 本次乘法结果）
        else
            {carry, acc} <= acc + mult_result_reg;
    end
end

// ---------------------- 输出阶段：乒乓缓冲 + 并行转串行 ----------------------
// 功能：将累加结果存入输出乒乓缓冲，供串行输出
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin // 复位初始化
        out_pingpong_sel <= 1'b0;
        out_buf0 <= 24'd0;
        out_buf1 <= 24'd0;
        buf0_valid <= 1'b0;
        buf1_valid <= 1'b0;
    end else if(mult_en_reg) begin // 乘法器使能有效（新累加结果生成）
        // 根据输出乒乓选择，存入对应缓冲并置有效标志
        if(!out_pingpong_sel) begin
            out_buf0 <= acc;
            buf0_valid <= 1'b1;
        end else begin
            out_buf1 <= acc;
            buf1_valid <= 1'b1;
        end
        out_pingpong_sel <= ~out_pingpong_sel; // 切换输出乒乓缓冲
    end else if(tx_cnt == 5'd1) begin // 输出即将完成（最后1位），清空有效标志
        if(!out_pingpong_sel) buf0_valid <= 1'b0;
        else buf1_valid <= 1'b0;
    end
end

// 输出使能：tx_cnt非0表示正在输出有效数据
assign out_ready = (tx_cnt != 5'd0);
// 串行输出：取移位寄存器最高位（MSB先行）
assign sum_out   = tx_shift_reg[23];

// 输出移位逻辑：将并行缓冲数据串行输出（MSB先行）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin // 复位初始化
        tx_cnt <= 5'd0;
        tx_shift_reg <= 24'd0;
    end else if(tx_cnt == 5'd0) begin // 输出计数器为0（空闲状态）
        // 缓冲0有效：加载数据到移位寄存器，初始化输出计数器
        if(buf0_valid) begin
            tx_shift_reg <= out_buf0;
            tx_cnt <= TX_CNT + 1'b1;
        end 
        // 缓冲1有效：加载数据到移位寄存器，初始化输出计数器
        else if(buf1_valid) begin
            tx_shift_reg <= out_buf1;
            tx_cnt <= TX_CNT + 1'b1;
        end
    end else if(out_ready) begin // 输出使能有效（正在输出）
        tx_shift_reg <= {tx_shift_reg[22:0], 1'b0}; // 左移1位（MSB先行）
        tx_cnt <= tx_cnt - 1'b1; // 输出计数器-1
    end
end

endmodule

// ---------------------- 乘法器子模块：4阶Booth+华莱士树+24位CLA ----------------------
// 功能：16位×16位乘法，输出24位结果，采用4阶Booth编码+华莱士树压缩+CLA加法
module booth4_wallace_cla_24bit (
    input  wire [15:0]          A, B,    // 乘法输入A/B（16位）
    output wire [23:0]          Result   // 乘法结果（24位）
);
    wire [23:0] pp[0:7];                 // 8个部分积（24位）
    wire [23:0] sum_w, cout_w;           // 华莱士树输出：和/进位

    // 4阶Booth编码器：生成8个24位部分积
    booth4_encoder_24bit u_enc(
        .A(A),.B(B),
        .pp0(pp[0]),.pp1(pp[1]),.pp2(pp[2]),.pp3(pp[3]),
        .pp4(pp[4]),.pp5(pp[5]),.pp6(pp[6]),.pp7(pp[7])
    );
    // 华莱士树压缩：8个部分积压缩为2个（和+进位）
    wallace_8to2_24bit    u_wal(
        .pp0(pp[0]),.pp1(pp[1]),.pp2(pp[2]),.pp3(pp[3]),
        .pp4(pp[4]),.pp5(pp[5]),.pp6(pp[6]),.pp7(pp[7]),
        .S(sum_w),.Co(cout_w)
    );
    // 24位超前进位加法器（CLA）：求和+进位，得到最终乘法结果
    cla_24bit             u_cla(
        .A(sum_w),.B(cout_w),.Cin(1'b0),.S(Result)
    );
endmodule

// ---------------------- 4阶Booth编码器：生成8个24位部分积 ----------------------
// 功能：对16位B进行4阶Booth编码，生成8个24位部分积（基于16位A）
module booth4_encoder_24bit (
    input  wire [15:0]      A, B,    // 输入A（被乘数）、B（乘数）
    output wire [23:0]      pp0,pp1,pp2,pp3,pp4,pp5,pp6,pp7 // 8个部分积
);
    wire [17:0] B_ext = {2'b00, B}; // B扩展为18位（补2位0，适配4阶编码）
    
    // 生成第0个部分积（B[1:0]编码）
    assign pp0 = B_ext[1:0] == 2'b00 ? 24'd0 : B_ext[1] ? {8'd0,A} : ~{8'd0,A} + 1'b1;
    // 生成第1个部分积（B[3:2]编码，左移2位）
    assign pp1 = B_ext[3:2] == 2'b00 ? 24'd0 : B_ext[3] ? {6'd0,A,2'b00} : ~{6'd0,A,2'b00} + 1'b1;
    // 生成第2个部分积（B[5:4]编码，左移4位）
    assign pp2 = B_ext[5:4] == 2'b00 ? 24'd0 : B_ext[5] ? {4'd0,A,4'b0000} : ~{4'd0,A,4'b0000} + 1'b1;
    // 生成第3个部分积（B[7:6]编码，左移6位）
    assign pp3 = B_ext[7:6] == 2'b00 ? 24'd0 : B_ext[7] ? {2'd0,A,6'b000000} : ~{2'd0,A,6'b000000} + 1'b1;
    // 生成第4个部分积（B[9:8]编码，左移8位）
    assign pp4 = B_ext[9:8] == 2'b00 ? 24'd0 : B_ext[9] ? {A,8'b00000000} : ~{A,8'b00000000} + 1'b1;
    // 生成第5个部分积（B[11:10]编码，左移10位）
    assign pp5 = B_ext[11:10]==2'b00 ? 24'd0 : B_ext[11]? {A[13:0],10'b0}:~{A[13:0],10'b0}+1'b1;
    // 生成第6个部分积（B[13:12]编码，左移12位）
    assign pp6 = B_ext[13:12]==2'b00 ? 24'd0 : B_ext[13]? {A[11:0],12'b0}:~{A[11:0],12'b0}+1'b1;
    // 生成第7个部分积（B[15:14]编码，左移14位）
    assign pp7 = B_ext[15:14]==2'b00 ? 24'd0 : B_ext[15]? {A[9:0],14'b0} :~{A[9:0],14'b0} +1'b1;
endmodule

// ---------------------- 华莱士树压缩器：8→2（24位） ----------------------
// 功能：将8个24位部分积压缩为2个（和+进位），采用3-2压缩器（全加器）级联
module wallace_8to2_24bit (
    input wire [23:0] pp0,pp1,pp2,pp3,pp4,pp5,pp6,pp7, // 8个输入部分积
    output wire [23:0] S, Co                           // 输出：和/进位
);
    wire [23:0] w1[0:5]; // 第一级压缩结果（6个24位）
    wire [23:0] w2[0:3]; // 第二级压缩结果（4个24位）
    wire [23:0] w3[0:1]; // 第三级压缩结果（2个24位）

    // 第一级压缩：3个部分积→2个（和+进位）
    full_adder_3to2_24bit u1(.A(pp0),.B(pp1),.C(pp2),.S(w1[0]),.Co(w1[1]));
    full_adder_3to2_24bit u2(.A(pp3),.B(pp4),.C(pp5),.S(w1[2]),.Co(w1[3]));
    full_adder_3to2_24bit u3(.A(pp6),.B(pp7),.C(24'd0),.S(w1[4]),.Co(w1[5]));
    // 第二级压缩：3个中间结果→2个
    full_adder_3to2_24bit u4(.A(w1[0]),.B(w1[1]),.C(w1[2]),.S(w2[0]),.Co(w2[1]));
    full_adder_3to2_24bit u5(.A(w1[3]),.B(w1[4]),.C(w1[5]),.S(w2[2]),.Co(w2[3]));
    // 第三级压缩：3个中间结果→2个
    full_adder_3to2_24bit u6(.A(w2[0]),.B(w2[1]),.C(w2[2]),.S(w3[0]),.Co(w3[1]));

    assign S <= w3[0];
    assign Co <= w3[1] << 1;
endmodule

// ---------------------- 3-2压缩器（24位全加器） ----------------------
// 功能：24位宽的3输入→2输出压缩器（和+进位），构成华莱士树的基本单元
module full_adder_3to2_24bit (
    input wire [23:0] A,B,C,  // 3个24位输入
    output wire [23:0] S, Co  // 输出：和（24位）/进位（24位）
);
    assign S  = A ^ B ^ C;          // 按位异或：生成和
    assign Co = (A&B) | (B&C) | (A&C); // 按位与或：生成进位
endmodule

// ---------------------- 24位超前进位加法器（CLA） ----------------------
// 功能：24位加法，超前进位结构，降低加法延迟
module cla_24bit (
    input wire [23:0]  A, B,   // 24位加数/被加数
    input wire         Cin,    // 进位输入
    output wire [23:0] S       // 加法结果
);
    wire [23:0] G = A & B;     // 生成项（Generate）
    wire [23:0] P = A ^ B;     // 传播项（Propagate）
    wire [24:0] C;             // 进位链（25位，C[0]为Cin）
    assign C[0] = Cin;         // 最低位进位输入
    
    // 生成24位超前进位逻辑
    genvar i;
    generate for(i=0; i<24; i=i+1) begin
        assign C[i+1] = G[i] | (P[i] & C[i]); // 超前进位公式
        assign S[i]   = P[i] ^ C[i];          // 每一位和
    end endgenerate
endmodule
